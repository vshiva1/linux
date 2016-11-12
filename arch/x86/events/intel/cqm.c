/*
 * Intel Cache Quality-of-Service Monitoring (CQM) support.
 *
 * Based very, very heavily on work by Peter Zijlstra.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/perf_event.h>
#include <linux/slab.h>
#include <asm/cpu_device_id.h>
#include <asm/intel_rdt_common.h>
#include "../perf_event.h"

#define MSR_IA32_QM_CTR		0x0c8e
#define MSR_IA32_QM_EVTSEL	0x0c8d

#define MBM_CNTR_WIDTH		24
/*
 * Guaranteed time in ms as per SDM where MBM counters will not overflow.
 */
#define MBM_CTR_OVERFLOW_TIME	1000
#define RMID_DEFAULT_QUEUE_TIME	250

static u32 cqm_max_rmid = -1;
static unsigned int cqm_l3_scale; /* supposedly cacheline size */
static bool cqm_enabled, mbm_enabled;
unsigned int mbm_socket_max;

/*
 * The cached intel_pqr_state is strictly per CPU and can never be
 * updated from a remote CPU. Both functions which modify the state
 * (intel_cqm_event_start and intel_cqm_event_stop) are called with
 * interrupts disabled, which is sufficient for the protection.
 */
DEFINE_PER_CPU(struct intel_pqr_state, pqr_state);
static struct hrtimer *mbm_timers;
/**
 * struct sample - mbm event's (local or total) data
 * @total_bytes    #bytes since we began monitoring
 * @prev_msr       previous value of MSR
 */
struct sample {
	u64	total_bytes;
	u64	prev_msr;
};

/*
 * samples profiled for total memory bandwidth type events
 */
static struct sample *mbm_total;
/*
 * samples profiled for local memory bandwidth type events
 */
static struct sample *mbm_local;

#define pkg_id	topology_physical_package_id(smp_processor_id())
/*
 * rmid_2_index returns the index for the rmid in mbm_local/mbm_total array.
 * mbm_total[] and mbm_local[] are linearly indexed by socket# * max number of
 * rmids per socket, an example is given below
 * RMID1 of Socket0:  vrmid =  1
 * RMID1 of Socket1:  vrmid =  1 * (cqm_max_rmid + 1) + 1
 * RMID1 of Socket2:  vrmid =  2 * (cqm_max_rmid + 1) + 1
 */
#define rmid_2_index(rmid)  ((pkg_id * (cqm_max_rmid + 1)) + rmid)
/*
 * Protects cache_cgroups and cqm_rmid_free_lru and cqm_rmid_limbo_lru.
 * Also protects event->hw.cqm_rmid
 *
 * Hold either for stability, both for modification of ->hw.cqm_rmid.
 */
static DEFINE_MUTEX(cache_mutex);
static DEFINE_RAW_SPINLOCK(cache_lock);

/*
 * Groups of events that have the same target(s), one RMID per group.
 */
static LIST_HEAD(cache_groups);

/*
 * Mask of CPUs for reading CQM values. We only need one per-socket.
 */
static cpumask_t cqm_cpumask;

#define RMID_VAL_ERROR		(1ULL << 63)
#define RMID_VAL_UNAVAIL	(1ULL << 62)

/*
 * Event IDs are used to program IA32_QM_EVTSEL before reading event
 * counter from IA32_QM_CTR
 */
#define QOS_L3_OCCUP_EVENT_ID	0x01
#define QOS_MBM_TOTAL_EVENT_ID	0x02
#define QOS_MBM_LOCAL_EVENT_ID	0x03

#define INVALID_RMID		(-1)

/*
 * Is @rmid valid for programming the hardware?
 *
 * rmid 0 is reserved by the hardware for all non-monitored tasks, which
 * means that we should never come across an rmid with that value.
 * Likewise, an rmid value of -1 is used to indicate "no rmid currently
 * assigned" and is used as part of the rotation code.
 */
static inline bool __rmid_valid(u32 rmid)
{
	if (!rmid || rmid > cqm_max_rmid)
		return false;

	return true;
}

static u64 __rmid_read(u32 rmid)
{
	u64 val;

	/*
	 * Ignore the SDM, this thing is _NOTHING_ like a regular perfcnt,
	 * it just says that to increase confusion.
	 */
	wrmsr(MSR_IA32_QM_EVTSEL, QOS_L3_OCCUP_EVENT_ID, rmid);
	rdmsrl(MSR_IA32_QM_CTR, val);

	/*
	 * Aside from the ERROR and UNAVAIL bits, assume this thing returns
	 * the number of cachelines tagged with @rmid.
	 */
	return val;
}

enum rmid_recycle_state {
	RMID_AVAILABLE = 0,
	RMID_DIRTY,
};

struct cqm_rmid_entry {
	u32 rmid;
	enum rmid_recycle_state state;
	struct list_head list;
	unsigned long queue_time;
};

/*
 * cqm_rmid_free_lru - A least recently used list of RMIDs.
 *
 * Oldest entry at the head, newest (most recently used) entry at the
 * tail. This list is never traversed, it's only used to keep track of
 * the lru order. That is, we only pick entries of the head or insert
 * them on the tail.
 *
 * All entries on the list are 'free', and their RMIDs are not currently
 * in use. To mark an RMID as in use, remove its entry from the lru
 * list.
 *
 *
 * cqm_rmid_limbo_lru - list of currently unused but (potentially) dirty RMIDs.
 *
 * This list is contains RMIDs that no one is currently using but that
 * may have a non-zero occupancy value associated with them. The
 * rotation worker moves RMIDs from the limbo list to the free list once
 * the occupancy value drops below __intel_cqm_threshold.
 *
 * Both lists are protected by cache_mutex.
 */
static LIST_HEAD(cqm_rmid_free_lru);
static LIST_HEAD(cqm_rmid_limbo_lru);

/*
 * We use a simple array of pointers so that we can lookup a struct
 * cqm_rmid_entry in O(1). This alleviates the callers of __get_rmid()
 * and __put_rmid() from having to worry about dealing with struct
 * cqm_rmid_entry - they just deal with rmids, i.e. integers.
 *
 * Once this array is initialized it is read-only. No locks are required
 * to access it.
 *
 * All entries for all RMIDs can be looked up in the this array at all
 * times.
 */
static struct cqm_rmid_entry **cqm_rmid_ptrs;

static inline struct cqm_rmid_entry *__rmid_entry(u32 rmid)
{
	struct cqm_rmid_entry *entry;

	entry = cqm_rmid_ptrs[rmid];
	WARN_ON(entry->rmid != rmid);

	return entry;
}

/*
 * Returns < 0 on fail.
 *
 * We expect to be called with cache_mutex held.
 */
static u32 __get_rmid(void)
{
	struct cqm_rmid_entry *entry;

	lockdep_assert_held(&cache_mutex);

	if (list_empty(&cqm_rmid_free_lru))
		return INVALID_RMID;

	entry = list_first_entry(&cqm_rmid_free_lru, struct cqm_rmid_entry, list);
	list_del(&entry->list);

	return entry->rmid;
}

static void __put_rmid(u32 rmid)
{
	struct cqm_rmid_entry *entry;

	lockdep_assert_held(&cache_mutex);

	WARN_ON(!__rmid_valid(rmid));
	entry = __rmid_entry(rmid);

	entry->queue_time = jiffies;
	entry->state = RMID_DIRTY;

	list_add_tail(&entry->list, &cqm_rmid_limbo_lru);
}

static void cqm_cleanup(void)
{
	int i;

	if (!cqm_rmid_ptrs)
		return;

	for (i = 0; i < cqm_max_rmid; i++)
		kfree(cqm_rmid_ptrs[i]);

	kfree(cqm_rmid_ptrs);
	cqm_rmid_ptrs = NULL;
	cqm_enabled = false;
}

static int intel_cqm_setup_rmid_cache(void)
{
	struct cqm_rmid_entry *entry;
	unsigned int nr_rmids;
	int r = 0;

	nr_rmids = cqm_max_rmid + 1;
	cqm_rmid_ptrs = kzalloc(sizeof(struct cqm_rmid_entry *) *
				nr_rmids, GFP_KERNEL);
	if (!cqm_rmid_ptrs)
		return -ENOMEM;

	for (; r <= cqm_max_rmid; r++) {
		struct cqm_rmid_entry *entry;

		entry = kmalloc(sizeof(*entry), GFP_KERNEL);
		if (!entry)
			goto fail;

		INIT_LIST_HEAD(&entry->list);
		entry->rmid = r;
		cqm_rmid_ptrs[r] = entry;

		list_add_tail(&entry->list, &cqm_rmid_free_lru);
	}

	/*
	 * RMID 0 is special and is always allocated. It's used for all
	 * tasks that are not monitored.
	 */
	entry = __rmid_entry(0);
	list_del(&entry->list);

	return 0;

fail:
	cqm_cleanup();
	return -ENOMEM;
}

/*
 * Determine if @a and @b measure the same set of tasks.
 *
 * If @a and @b measure the same set of tasks then we want to share a
 * single RMID.
 */
static bool __match_event(struct perf_event *a, struct perf_event *b)
{
	/* Per-cpu and task events don't mix */
	if ((a->attach_state & PERF_ATTACH_TASK) !=
	    (b->attach_state & PERF_ATTACH_TASK))
		return false;

#ifdef CONFIG_CGROUP_PERF
	if (a->cgrp != b->cgrp)
		return false;
#endif

	/* If not task event, we're machine wide */
	if (!(b->attach_state & PERF_ATTACH_TASK))
		return true;

	/*
	 * Events that target same task are placed into the same cache group.
	 * Mark it as a multi event group, so that we update ->count
	 * for every event rather than just the group leader later.
	 */
	if (a->hw.target == b->hw.target) {
		b->hw.is_group_event = true;
		return true;
	}

	/*
	 * Are we an inherited event?
	 */
	if (b->parent == a)
		return true;

	return false;
}

#ifdef CONFIG_CGROUP_PERF
static inline struct perf_cgroup *event_to_cgroup(struct perf_event *event)
{
	if (event->attach_state & PERF_ATTACH_TASK)
		return perf_cgroup_from_task(event->hw.target, event->ctx);

	return event->cgrp;
}
#endif

struct rmid_read {
	u32 rmid;
	u32 evt_type;
	atomic64_t value;
};

static void __intel_cqm_event_count(void *info);
static void init_mbm_sample(u32 rmid, u32 evt_type);
static void __intel_mbm_event_count(void *info);

static bool is_cqm_event(int e)
{
	return (e == QOS_L3_OCCUP_EVENT_ID);
}

static bool is_mbm_event(int e)
{
	return (e >= QOS_MBM_TOTAL_EVENT_ID && e <= QOS_MBM_LOCAL_EVENT_ID);
}

static void cqm_mask_call(struct rmid_read *rr)
{
	if (is_mbm_event(rr->evt_type))
		on_each_cpu_mask(&cqm_cpumask, __intel_mbm_event_count, rr, 1);
	else
		on_each_cpu_mask(&cqm_cpumask, __intel_cqm_event_count, rr, 1);
}

/*
 * __intel_cqm_max_threshold provides an upper bound on the threshold,
 * and is measured in bytes because it's exposed to userland.
 */
static unsigned int __intel_cqm_threshold;
static unsigned int __intel_cqm_max_threshold;

static struct pmu intel_cqm_pmu;

static u64 update_sample(unsigned int rmid, u32 evt_type, int first)
{
	struct sample *mbm_current;
	u32 vrmid = rmid_2_index(rmid);
	u64 val, bytes, shift;
	u32 eventid;

	if (evt_type == QOS_MBM_LOCAL_EVENT_ID) {
		mbm_current = &mbm_local[vrmid];
		eventid     = QOS_MBM_LOCAL_EVENT_ID;
	} else {
		mbm_current = &mbm_total[vrmid];
		eventid     = QOS_MBM_TOTAL_EVENT_ID;
	}

	wrmsr(MSR_IA32_QM_EVTSEL, eventid, rmid);
	rdmsrl(MSR_IA32_QM_CTR, val);
	if (val & (RMID_VAL_ERROR | RMID_VAL_UNAVAIL))
		return mbm_current->total_bytes;

	if (first) {
		mbm_current->prev_msr = val;
		mbm_current->total_bytes = 0;
		return mbm_current->total_bytes;
	}

	/*
	 * The h/w guarantees that counters will not overflow
	 * so long as we poll them at least once per second.
	 */
	shift = 64 - MBM_CNTR_WIDTH;
	bytes = (val << shift) - (mbm_current->prev_msr << shift);
	bytes >>= shift;

	bytes *= cqm_l3_scale;

	mbm_current->total_bytes += bytes;
	mbm_current->prev_msr = val;

	return mbm_current->total_bytes;
}

static u64 rmid_read_mbm(unsigned int rmid, u32 evt_type)
{
	return update_sample(rmid, evt_type, 0);
}

static void __intel_mbm_event_init(void *info)
{
	struct rmid_read *rr = info;

	update_sample(rr->rmid, rr->evt_type, 1);
}

static void init_mbm_sample(u32 rmid, u32 evt_type)
{
	struct rmid_read rr = {
		.rmid = rmid,
		.evt_type = evt_type,
		.value = ATOMIC64_INIT(0),
	};

	/* on each socket, init sample */
	on_each_cpu_mask(&cqm_cpumask, __intel_mbm_event_init, &rr, 1);
}

/*
 * Find a group and setup RMID.
 *
 * If we're part of a group, we use the group's RMID.
 */
static int intel_cqm_setup_event(struct perf_event *event,
				  struct perf_event **group)
{
	struct perf_event *iter;
	u32 rmid;

	event->hw.is_group_event = false;
	list_for_each_entry(iter, &cache_groups, hw.cqm_groups_entry) {
		rmid = iter->hw.cqm_rmid;

		if (__match_event(iter, event)) {
			/* All tasks in a group share an RMID */
			event->hw.cqm_rmid = rmid;
			*group = iter;
			if (is_mbm_event(event->attr.config) && __rmid_valid(rmid))
				init_mbm_sample(rmid, event->attr.config);
			return 0;
		}

	}

	rmid = __get_rmid();

	if (!__rmid_valid(rmid)) {
		pr_info("out of RMIDs\n");
		return -EINVAL;
	}

	if (is_mbm_event(event->attr.config) && __rmid_valid(rmid))
		init_mbm_sample(rmid, event->attr.config);

	event->hw.cqm_rmid = rmid;

	return 0;
}

static void intel_cqm_event_read(struct perf_event *event)
{
	unsigned long flags;
	u32 rmid;
	u64 val;

	/*
	 * Task events are handled by intel_cqm_event_count().
	 */
	if (event->cpu == -1)
		return;

	raw_spin_lock_irqsave(&cache_lock, flags);
	rmid = event->hw.cqm_rmid;

	if (!__rmid_valid(rmid))
		goto out;

	if (is_mbm_event(event->attr.config))
		val = rmid_read_mbm(rmid, event->attr.config);
	else
		val = __rmid_read(rmid);

	/*
	 * Ignore this reading on error states and do not update the value.
	 */
	if (val & (RMID_VAL_ERROR | RMID_VAL_UNAVAIL))
		goto out;

	local64_set(&event->count, val);
out:
	raw_spin_unlock_irqrestore(&cache_lock, flags);
}

static void __intel_cqm_event_count(void *info)
{
	struct rmid_read *rr = info;
	u64 val;

	val = __rmid_read(rr->rmid);

	if (val & (RMID_VAL_ERROR | RMID_VAL_UNAVAIL))
		return;

	atomic64_add(val, &rr->value);
}

static inline bool cqm_group_leader(struct perf_event *event)
{
	return !list_empty(&event->hw.cqm_groups_entry);
}

static void __intel_mbm_event_count(void *info)
{
	struct rmid_read *rr = info;
	u64 val;

	val = rmid_read_mbm(rr->rmid, rr->evt_type);
	if (val & (RMID_VAL_ERROR | RMID_VAL_UNAVAIL))
		return;
	atomic64_add(val, &rr->value);
}

static enum hrtimer_restart mbm_hrtimer_handle(struct hrtimer *hrtimer)
{
	struct perf_event *iter, *iter1;
	int ret = HRTIMER_RESTART;
	struct list_head *head;
	unsigned long flags;
	u32 grp_rmid;

	/*
	 * Need to cache_lock as the timer Event Select MSR reads
	 * can race with the mbm/cqm count() and mbm_init() reads.
	 */
	raw_spin_lock_irqsave(&cache_lock, flags);

	if (list_empty(&cache_groups)) {
		ret = HRTIMER_NORESTART;
		goto out;
	}

	list_for_each_entry(iter, &cache_groups, hw.cqm_groups_entry) {
		grp_rmid = iter->hw.cqm_rmid;
		if (!__rmid_valid(grp_rmid))
			continue;
		if (is_mbm_event(iter->attr.config))
			update_sample(grp_rmid, iter->attr.config, 0);

		head = &iter->hw.cqm_group_entry;
		if (list_empty(head))
			continue;
		list_for_each_entry(iter1, head, hw.cqm_group_entry) {
			if (!iter1->hw.is_group_event)
				break;
			if (is_mbm_event(iter1->attr.config))
				update_sample(iter1->hw.cqm_rmid,
					      iter1->attr.config, 0);
		}
	}

	hrtimer_forward_now(hrtimer, ms_to_ktime(MBM_CTR_OVERFLOW_TIME));
out:
	raw_spin_unlock_irqrestore(&cache_lock, flags);

	return ret;
}

static void __mbm_start_timer(void *info)
{
	hrtimer_start(&mbm_timers[pkg_id], ms_to_ktime(MBM_CTR_OVERFLOW_TIME),
			     HRTIMER_MODE_REL_PINNED);
}

static void __mbm_stop_timer(void *info)
{
	hrtimer_cancel(&mbm_timers[pkg_id]);
}

static void mbm_start_timers(void)
{
	on_each_cpu_mask(&cqm_cpumask, __mbm_start_timer, NULL, 1);
}

static void mbm_stop_timers(void)
{
	on_each_cpu_mask(&cqm_cpumask, __mbm_stop_timer, NULL, 1);
}

static void mbm_hrtimer_init(void)
{
	struct hrtimer *hr;
	int i;

	for (i = 0; i < mbm_socket_max; i++) {
		hr = &mbm_timers[i];
		hrtimer_init(hr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		hr->function = mbm_hrtimer_handle;
	}
}

static u64 intel_cqm_event_count(struct perf_event *event)
{
	struct rmid_read rr = {
		.evt_type = event->attr.config,
		.value = ATOMIC64_INIT(0),
	};

	/*
	 * We only need to worry about task events. System-wide events
	 * are handled like usual, i.e. entirely with
	 * intel_cqm_event_read().
	 */
	if (event->cpu != -1)
		return __perf_event_count(event);

	/*
	 * Only the group leader gets to report values except in case of
	 * multiple events in the same group, we still need to read the
	 * other events.This stops us
	 * reporting duplicate values to userspace, and gives us a clear
	 * rule for which task gets to report the values.
	 *
	 * Note that it is impossible to attribute these values to
	 * specific packages - we forfeit that ability when we create
	 * task events.
	 */
	if (!cqm_group_leader(event) && !event->hw.is_group_event)
		return 0;

	/*
	 * Getting up-to-date values requires an SMP IPI which is not
	 * possible if we're being called in interrupt context. Return
	 * the cached values instead.
	 */
	if (unlikely(in_interrupt()))
		goto out;

	/*
	 * Notice that we don't perform the reading of an RMID
	 * atomically, because we can't hold a spin lock across the
	 * IPIs.
	 */
	rr.rmid = ACCESS_ONCE(event->hw.cqm_rmid);
	cqm_mask_call(&rr);
	local64_set(&event->count, atomic64_read(&rr.value));

out:
	return __perf_event_count(event);
}

static void intel_cqm_event_start(struct perf_event *event, int mode)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);
	u32 rmid = event->hw.cqm_rmid;

	if (!(event->hw.cqm_state & PERF_HES_STOPPED))
		return;

	event->hw.cqm_state &= ~PERF_HES_STOPPED;

	state->rmid = rmid;
	wrmsr(MSR_IA32_PQR_ASSOC, rmid, state->closid);
}

static void intel_cqm_event_stop(struct perf_event *event, int mode)
{
	if (event->hw.cqm_state & PERF_HES_STOPPED)
		return;

	event->hw.cqm_state |= PERF_HES_STOPPED;
}

static int intel_cqm_event_add(struct perf_event *event, int mode)
{
	unsigned long flags;
	u32 rmid;

	raw_spin_lock_irqsave(&cache_lock, flags);

	event->hw.cqm_state = PERF_HES_STOPPED;
	rmid = event->hw.cqm_rmid;

	if (__rmid_valid(rmid) && (mode & PERF_EF_START))
		intel_cqm_event_start(event, mode);

	raw_spin_unlock_irqrestore(&cache_lock, flags);

	return 0;
}

static void intel_cqm_event_destroy(struct perf_event *event)
{
	struct perf_event *group_other = NULL;
	unsigned long flags;

	mutex_lock(&cache_mutex);
	/*
	* Hold the cache_lock as mbm timer handlers could be
	* scanning the list of events.
	*/
	raw_spin_lock_irqsave(&cache_lock, flags);

	/*
	 * If there's another event in this group...
	 */
	if (!list_empty(&event->hw.cqm_group_entry)) {
		group_other = list_first_entry(&event->hw.cqm_group_entry,
					       struct perf_event,
					       hw.cqm_group_entry);
		list_del(&event->hw.cqm_group_entry);
	}

	/*
	 * And we're the group leader..
	 */
	if (cqm_group_leader(event)) {
		/*
		 * If there was a group_other, make that leader, otherwise
		 * destroy the group and return the RMID.
		 */
		if (group_other) {
			list_replace(&event->hw.cqm_groups_entry,
				     &group_other->hw.cqm_groups_entry);
		} else {
			u32 rmid = event->hw.cqm_rmid;

			if (__rmid_valid(rmid))
				__put_rmid(rmid);
			list_del(&event->hw.cqm_groups_entry);
		}
	}

	raw_spin_unlock_irqrestore(&cache_lock, flags);

	/*
	 * Stop the mbm overflow timers when the last event is destroyed.
	*/
	if (mbm_enabled && list_empty(&cache_groups))
		mbm_stop_timers();

	mutex_unlock(&cache_mutex);
}

static int intel_cqm_event_init(struct perf_event *event)
{
	struct perf_event *group = NULL;
	unsigned long flags;
	int ret = 0;

	if (event->attr.type != intel_cqm_pmu.type)
		return -ENOENT;

	if ((event->attr.config < QOS_L3_OCCUP_EVENT_ID) ||
	     (event->attr.config > QOS_MBM_LOCAL_EVENT_ID))
		return -EINVAL;

	if ((is_cqm_event(event->attr.config) && !cqm_enabled) ||
	    (is_mbm_event(event->attr.config) && !mbm_enabled))
		return -EINVAL;

	/* unsupported modes and filters */
	if (event->attr.exclude_user   ||
	    event->attr.exclude_kernel ||
	    event->attr.exclude_hv     ||
	    event->attr.exclude_idle   ||
	    event->attr.exclude_host   ||
	    event->attr.exclude_guest  ||
	    event->attr.sample_period) /* no sampling */
		return -EINVAL;

	INIT_LIST_HEAD(&event->hw.cqm_group_entry);
	INIT_LIST_HEAD(&event->hw.cqm_groups_entry);

	event->destroy = intel_cqm_event_destroy;

	mutex_lock(&cache_mutex);

	/* Will also set rmid, return error on RMID not being available*/
	if (intel_cqm_setup_event(event, &group)) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * Start the mbm overflow timers when the first event is created.
	*/
	if (mbm_enabled && list_empty(&cache_groups))
		mbm_start_timers();

	/*
	* Hold the cache_lock as mbm timer handlers be
	* scanning the list of events.
	*/
	raw_spin_lock_irqsave(&cache_lock, flags);

	if (group)
		list_add_tail(&event->hw.cqm_group_entry,
			      &group->hw.cqm_group_entry);
	else
		list_add_tail(&event->hw.cqm_groups_entry,
			      &cache_groups);

	raw_spin_unlock_irqrestore(&cache_lock, flags);
out:
	mutex_unlock(&cache_mutex);

	return ret;
}

EVENT_ATTR_STR(llc_occupancy, intel_cqm_llc, "event=0x01");
EVENT_ATTR_STR(llc_occupancy.per-pkg, intel_cqm_llc_pkg, "1");
EVENT_ATTR_STR(llc_occupancy.unit, intel_cqm_llc_unit, "Bytes");
EVENT_ATTR_STR(llc_occupancy.scale, intel_cqm_llc_scale, NULL);
EVENT_ATTR_STR(llc_occupancy.snapshot, intel_cqm_llc_snapshot, "1");

EVENT_ATTR_STR(total_bytes, intel_cqm_total_bytes, "event=0x02");
EVENT_ATTR_STR(total_bytes.per-pkg, intel_cqm_total_bytes_pkg, "1");
EVENT_ATTR_STR(total_bytes.unit, intel_cqm_total_bytes_unit, "MB");
EVENT_ATTR_STR(total_bytes.scale, intel_cqm_total_bytes_scale, "1e-6");

EVENT_ATTR_STR(local_bytes, intel_cqm_local_bytes, "event=0x03");
EVENT_ATTR_STR(local_bytes.per-pkg, intel_cqm_local_bytes_pkg, "1");
EVENT_ATTR_STR(local_bytes.unit, intel_cqm_local_bytes_unit, "MB");
EVENT_ATTR_STR(local_bytes.scale, intel_cqm_local_bytes_scale, "1e-6");

static struct attribute *intel_cqm_events_attr[] = {
	EVENT_PTR(intel_cqm_llc),
	EVENT_PTR(intel_cqm_llc_pkg),
	EVENT_PTR(intel_cqm_llc_unit),
	EVENT_PTR(intel_cqm_llc_scale),
	EVENT_PTR(intel_cqm_llc_snapshot),
	NULL,
};

static struct attribute *intel_mbm_events_attr[] = {
	EVENT_PTR(intel_cqm_total_bytes),
	EVENT_PTR(intel_cqm_local_bytes),
	EVENT_PTR(intel_cqm_total_bytes_pkg),
	EVENT_PTR(intel_cqm_local_bytes_pkg),
	EVENT_PTR(intel_cqm_total_bytes_unit),
	EVENT_PTR(intel_cqm_local_bytes_unit),
	EVENT_PTR(intel_cqm_total_bytes_scale),
	EVENT_PTR(intel_cqm_local_bytes_scale),
	NULL,
};

static struct attribute *intel_cmt_mbm_events_attr[] = {
	EVENT_PTR(intel_cqm_llc),
	EVENT_PTR(intel_cqm_total_bytes),
	EVENT_PTR(intel_cqm_local_bytes),
	EVENT_PTR(intel_cqm_llc_pkg),
	EVENT_PTR(intel_cqm_total_bytes_pkg),
	EVENT_PTR(intel_cqm_local_bytes_pkg),
	EVENT_PTR(intel_cqm_llc_unit),
	EVENT_PTR(intel_cqm_total_bytes_unit),
	EVENT_PTR(intel_cqm_local_bytes_unit),
	EVENT_PTR(intel_cqm_llc_scale),
	EVENT_PTR(intel_cqm_total_bytes_scale),
	EVENT_PTR(intel_cqm_local_bytes_scale),
	EVENT_PTR(intel_cqm_llc_snapshot),
	NULL,
};

static struct attribute_group intel_cqm_events_group = {
	.name = "events",
	.attrs = NULL,
};

PMU_FORMAT_ATTR(event, "config:0-7");
static struct attribute *intel_cqm_formats_attr[] = {
	&format_attr_event.attr,
	NULL,
};

static struct attribute_group intel_cqm_format_group = {
	.name = "format",
	.attrs = intel_cqm_formats_attr,
};

static ssize_t
max_recycle_threshold_show(struct device *dev, struct device_attribute *attr,
			   char *page)
{
	ssize_t rv;

	mutex_lock(&cache_mutex);
	rv = snprintf(page, PAGE_SIZE-1, "%u\n", __intel_cqm_max_threshold);
	mutex_unlock(&cache_mutex);

	return rv;
}

static ssize_t
max_recycle_threshold_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int bytes, cachelines;
	int ret;

	ret = kstrtouint(buf, 0, &bytes);
	if (ret)
		return ret;

	mutex_lock(&cache_mutex);

	__intel_cqm_max_threshold = bytes;
	cachelines = bytes / cqm_l3_scale;

	/*
	 * The new maximum takes effect immediately.
	 */
	if (__intel_cqm_threshold > cachelines)
		__intel_cqm_threshold = cachelines;

	mutex_unlock(&cache_mutex);

	return count;
}

static DEVICE_ATTR_RW(max_recycle_threshold);

static struct attribute *intel_cqm_attrs[] = {
	&dev_attr_max_recycle_threshold.attr,
	NULL,
};

static const struct attribute_group intel_cqm_group = {
	.attrs = intel_cqm_attrs,
};

static const struct attribute_group *intel_cqm_attr_groups[] = {
	&intel_cqm_events_group,
	&intel_cqm_format_group,
	&intel_cqm_group,
	NULL,
};

static struct pmu intel_cqm_pmu = {
	.hrtimer_interval_ms = RMID_DEFAULT_QUEUE_TIME,
	.attr_groups	     = intel_cqm_attr_groups,
	.task_ctx_nr	     = perf_sw_context,
	.event_init	     = intel_cqm_event_init,
	.add		     = intel_cqm_event_add,
	.del		     = intel_cqm_event_stop,
	.start		     = intel_cqm_event_start,
	.stop		     = intel_cqm_event_stop,
	.read		     = intel_cqm_event_read,
	.count		     = intel_cqm_event_count,
};

static inline void cqm_pick_event_reader(int cpu)
{
	int reader;

	/* First online cpu in package becomes the reader */
	reader = cpumask_any_and(&cqm_cpumask, topology_core_cpumask(cpu));
	if (reader >= nr_cpu_ids)
		cpumask_set_cpu(cpu, &cqm_cpumask);
}

static int intel_cqm_cpu_starting(unsigned int cpu)
{
	struct intel_pqr_state *state = &per_cpu(pqr_state, cpu);
	struct cpuinfo_x86 *c = &cpu_data(cpu);

	state->rmid = 0;
	state->closid = 0;
	state->rmid_usecnt = 0;

	WARN_ON(c->x86_cache_max_rmid != cqm_max_rmid);
	WARN_ON(c->x86_cache_occ_scale != cqm_l3_scale);

	cqm_pick_event_reader(cpu);
	return 0;
}

static int intel_cqm_cpu_exit(unsigned int cpu)
{
	int target;

	/* Is @cpu the current cqm reader for this package ? */
	if (!cpumask_test_and_clear_cpu(cpu, &cqm_cpumask))
		return 0;

	/* Find another online reader in this package */
	target = cpumask_any_but(topology_core_cpumask(cpu), cpu);

	if (target < nr_cpu_ids)
		cpumask_set_cpu(target, &cqm_cpumask);

	return 0;
}

static const struct x86_cpu_id intel_cqm_match[] = {
	{ .vendor = X86_VENDOR_INTEL, .feature = X86_FEATURE_CQM_OCCUP_LLC },
	{}
};

static void mbm_cleanup(void)
{
	if (!mbm_enabled)
		return;

	kfree(mbm_local);
	kfree(mbm_total);
	mbm_enabled = false;
}

static const struct x86_cpu_id intel_mbm_local_match[] = {
	{ .vendor = X86_VENDOR_INTEL, .feature = X86_FEATURE_CQM_MBM_LOCAL },
	{}
};

static const struct x86_cpu_id intel_mbm_total_match[] = {
	{ .vendor = X86_VENDOR_INTEL, .feature = X86_FEATURE_CQM_MBM_TOTAL },
	{}
};

static int intel_mbm_init(void)
{
	int ret = 0, array_size, maxid = cqm_max_rmid + 1;

	mbm_socket_max = topology_max_packages();
	array_size = sizeof(struct sample) * maxid * mbm_socket_max;
	mbm_local = kmalloc(array_size, GFP_KERNEL);
	if (!mbm_local)
		return -ENOMEM;

	mbm_total = kmalloc(array_size, GFP_KERNEL);
	if (!mbm_total) {
		ret = -ENOMEM;
		goto out;
	}

	array_size = sizeof(struct hrtimer) * mbm_socket_max;
	mbm_timers = kmalloc(array_size, GFP_KERNEL);
	if (!mbm_timers) {
		ret = -ENOMEM;
		goto out;
	}
	mbm_hrtimer_init();

out:
	if (ret)
		mbm_cleanup();

	return ret;
}

static int __init intel_cqm_init(void)
{
	char *str = NULL, scale[20];
	int cpu, ret;

	if (x86_match_cpu(intel_cqm_match))
		cqm_enabled = true;

	if (x86_match_cpu(intel_mbm_local_match) &&
	     x86_match_cpu(intel_mbm_total_match))
		mbm_enabled = true;

	if (!cqm_enabled && !mbm_enabled)
		return -ENODEV;

	cqm_l3_scale = boot_cpu_data.x86_cache_occ_scale;

	/*
	 * It's possible that not all resources support the same number
	 * of RMIDs. Instead of making scheduling much more complicated
	 * (where we have to match a task's RMID to a cpu that supports
	 * that many RMIDs) just find the minimum RMIDs supported across
	 * all cpus.
	 *
	 * Also, check that the scales match on all cpus.
	 */
	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct cpuinfo_x86 *c = &cpu_data(cpu);

		if (c->x86_cache_max_rmid < cqm_max_rmid)
			cqm_max_rmid = c->x86_cache_max_rmid;

		if (c->x86_cache_occ_scale != cqm_l3_scale) {
			pr_err("Multiple LLC scale values, disabling\n");
			ret = -EINVAL;
			goto out;
		}
	}

	/*
	 * A reasonable upper limit on the max threshold is the number
	 * of lines tagged per RMID if all RMIDs have the same number of
	 * lines tagged in the LLC.
	 *
	 * For a 35MB LLC and 56 RMIDs, this is ~1.8% of the LLC.
	 */
	__intel_cqm_max_threshold =
		boot_cpu_data.x86_cache_size * 1024 / (cqm_max_rmid + 1);

	__intel_cqm_threshold = __intel_cqm_max_threshold / cqm_l3_scale;

	snprintf(scale, sizeof(scale), "%u", cqm_l3_scale);
	str = kstrdup(scale, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto out;
	}

	event_attr_intel_cqm_llc_scale.event_str = str;

	ret = intel_cqm_setup_rmid_cache();
	if (ret)
		goto out;

	if (mbm_enabled)
		ret = intel_mbm_init();
	if (ret && !cqm_enabled)
		goto out;

	if (cqm_enabled && mbm_enabled)
		intel_cqm_events_group.attrs = intel_cmt_mbm_events_attr;
	else if (!cqm_enabled && mbm_enabled)
		intel_cqm_events_group.attrs = intel_mbm_events_attr;
	else if (cqm_enabled && !mbm_enabled)
		intel_cqm_events_group.attrs = intel_cqm_events_attr;

	ret = perf_pmu_register(&intel_cqm_pmu, "intel_cqm", -1);
	if (ret) {
		pr_err("Intel CQM perf registration failed: %d\n", ret);
		goto out;
	}

	if (cqm_enabled)
		pr_info("Intel CQM monitoring enabled\n");
	if (mbm_enabled)
		pr_info("Intel MBM enabled\n");

	/*
	 * Setup the hot cpu notifier once we are sure cqm
	 * is enabled to avoid notifier leak.
	 */
	cpuhp_setup_state(CPUHP_AP_PERF_X86_CQM_STARTING,
			  "perf/x86/cqm:starting",
			  intel_cqm_cpu_starting, NULL);
	cpuhp_setup_state(CPUHP_AP_PERF_X86_CQM_ONLINE, "perf/x86/cqm:online",
			  NULL, intel_cqm_cpu_exit);

out:
	put_online_cpus();

	if (ret) {
		kfree(str);
		cqm_cleanup();
		mbm_cleanup();
	}

	return ret;
}
device_initcall(intel_cqm_init);
