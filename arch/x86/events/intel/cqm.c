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
#include "cqm.h"

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
unsigned int cqm_socket_max;

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

DEFINE_STATIC_KEY_FALSE(cqm_enable_key);

/*
 * Groups of events that have the same target(s), one RMID per group.
 */
static LIST_HEAD(cache_groups);

/*
 * Mask of CPUs for reading CQM values. We only need one per-socket.
 */
static cpumask_t cqm_cpumask;

struct pkg_data **cqm_pkgs_data;
struct cgrp_cqm_info cqm_rootcginfo;

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
bool __rmid_valid(u32 rmid)
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

static inline struct cqm_rmid_entry *__rmid_entry(u32 rmid, int domain)
{
	struct cqm_rmid_entry *entry;

	entry = &cqm_pkgs_data[domain]->cqm_rmid_ptrs[rmid];
	WARN_ON(entry->rmid != rmid);

	return entry;
}

/*
 * Returns < 0 on fail.
 *
 * We expect to be called with cache_mutex held.
 */
u32 __get_rmid(int domain)
{
	struct list_head *cqm_flist;
	struct cqm_rmid_entry *entry;

	lockdep_assert_held(&cache_lock);

	cqm_flist = &cqm_pkgs_data[domain]->cqm_rmid_free_lru;

	if (list_empty(cqm_flist))
		return INVALID_RMID;

	entry = list_first_entry(cqm_flist, struct cqm_rmid_entry, list);
	list_del(&entry->list);

	return entry->rmid;
}

static void __put_rmid(u32 rmid, int domain)
{
	struct cqm_rmid_entry *entry;

	lockdep_assert_held(&cache_lock);

	WARN_ON(!rmid);
	entry = __rmid_entry(rmid, domain);

	entry->queue_time = jiffies;
	entry->state = RMID_DIRTY;

	list_add_tail(&entry->list, &cqm_pkgs_data[domain]->cqm_rmid_limbo_lru);
}

static bool is_task_event(struct perf_event *e)
{
	return (e->attach_state & PERF_ATTACH_TASK);
}

static void cqm_cleanup(void)
{
	int i;

	if (!cqm_pkgs_data)
		return;

	for (i = 0; i < cqm_socket_max; i++) {
		if (cqm_pkgs_data[i]) {
			kfree(cqm_pkgs_data[i]->cqm_rmid_ptrs);
			kfree(cqm_pkgs_data[i]);
		}
	}
	kfree(cqm_pkgs_data);
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
	if ((is_cgroup_event(a) && is_cgroup_event(b)) &&
		(a->cgrp == b->cgrp))
		return true;
#endif

	/*
	 * Events that target same task are placed into the same cache group.
	 * Mark it as a multi event group, so that we update ->count
	 * for every event rather than just the group leader later.
	 */
	if ((is_task_event(a) && is_task_event(b)) &&
		(a->hw.target == b->hw.target)) {
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
	u32 *rmid;
	u32 evt_type;
	atomic64_t value;
};

static void __intel_cqm_event_count(void *info);
static void init_mbm_sample(u32 *rmid, u32 evt_type);
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

	if (__rmid_valid(rr->rmid[pkg_id]))
		update_sample(rr->rmid[pkg_id], rr->evt_type, 1);
}

static void init_mbm_sample(u32 *rmid, u32 evt_type)
{
	struct rmid_read rr = {
		.rmid = rmid,
		.evt_type = evt_type,
		.value = ATOMIC64_INIT(0),
	};

	/* on each socket, init sample */
	on_each_cpu_mask(&cqm_cpumask, __intel_mbm_event_init, &rr, 1);
}

#ifdef CONFIG_CGROUP_PERF
struct cgrp_cqm_info *cqminfo_from_tsk(struct task_struct *tsk)
{
	struct cgrp_cqm_info *ccinfo = NULL;
	struct perf_cgroup *pcgrp;

	pcgrp = perf_cgroup_from_task(tsk, NULL);

	if (!pcgrp)
		return NULL;
	else
		ccinfo = cgrp_to_cqm_info(pcgrp);

	return ccinfo;
}
#endif

static inline void cqm_enable_mon(struct cgrp_cqm_info *cqm_info, u32 *rmid)
{
	if (rmid != NULL) {
		cqm_info->mon_enabled = true;
		cqm_info->rmid = rmid;
	} else {
		cqm_info->mon_enabled = false;
		cqm_info->rmid = NULL;
	}
}

static void cqm_assign_hier_rmid(struct cgroup_subsys_state *rcss, u32 *rmid)
{
	struct cgrp_cqm_info *ccqm_info, *rcqm_info;
	struct cgroup_subsys_state *pos_css;

	rcu_read_lock();

	rcqm_info = css_to_cqm_info(rcss);

	/* Enable or disable monitoring based on rmid.*/
	cqm_enable_mon(rcqm_info, rmid);

	pos_css = css_next_descendant_pre(rcss, rcss);
	while (pos_css) {
		ccqm_info = css_to_cqm_info(pos_css);

		/*
		 * Monitoring is being enabled.
		 * Update the descendents to monitor for you, unless
		 * they were already monitoring for a descendent of yours.
		 */
		if (rmid && (rcqm_info->level > ccqm_info->mfa->level))
			ccqm_info->mfa = rcqm_info;

		/*
		 * Monitoring is being disabled.
		 * Update the descendents who were monitoring for you
		 * to monitor for the ancestor you were monitoring.
		 */
		if (!rmid && (ccqm_info->mfa == rcqm_info))
			ccqm_info->mfa = rcqm_info->mfa;
		pos_css = css_next_descendant_pre(pos_css, rcss);
	}
	rcu_read_unlock();
}

static int cqm_assign_rmid(struct perf_event *event, u32 *rmid)
{
#ifdef CONFIG_CGROUP_PERF
	if (is_cgroup_event(event)) {
		cqm_assign_hier_rmid(&event->cgrp->css, rmid);
	}
#endif
	return 0;
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
	u32 *rmid, sizet;

	event->hw.is_group_event = false;
	list_for_each_entry(iter, &cache_groups, hw.cqm_groups_entry) {
		rmid = iter->hw.cqm_rmid;

		if (__match_event(iter, event)) {
			/* All tasks in a group share an RMID */
			event->hw.cqm_rmid = rmid;
			*group = iter;
			if (is_mbm_event(event->attr.config))
				init_mbm_sample(rmid, event->attr.config);
			return 0;
		}
	}

	/*
	 * RMIDs are allocated in LAZY mode by default only when
	 * tasks monitored are scheduled in.
	 */
	sizet = sizeof(u32) * cqm_socket_max;
	event->hw.cqm_rmid = kzalloc(sizet, GFP_KERNEL);
	if (!event->hw.cqm_rmid)
		return -ENOMEM;

	return 0;
}

static u64 cqm_read_subtree(struct perf_event *event, struct rmid_read *rr);

static void intel_cqm_event_read(struct perf_event *event)
{
	struct rmid_read rr = {
		.evt_type = event->attr.config,
		.value = ATOMIC64_INIT(0),
	};

	/*
	 * Task events are handled by intel_cqm_event_count().
	 */
	if (event->cpu == -1)
		return;

	rr.rmid = ACCESS_ONCE(event->hw.cqm_rmid);

	cqm_read_subtree(event, &rr);
}

static void __intel_cqm_event_count(void *info)
{
	struct rmid_read *rr = info;
	u64 val;

	if (__rmid_valid(rr->rmid[pkg_id])) {
		val = __rmid_read(rr->rmid[pkg_id]);
		if (val & (RMID_VAL_ERROR | RMID_VAL_UNAVAIL))
			return;
		atomic64_add(val, &rr->value);
	}
}

static inline bool cqm_group_leader(struct perf_event *event)
{
	return !list_empty(&event->hw.cqm_groups_entry);
}

static void __intel_mbm_event_count(void *info)
{
	struct rmid_read *rr = info;
	u64 val;

	if (__rmid_valid(rr->rmid[pkg_id])) {
		val = rmid_read_mbm(rr->rmid[pkg_id], rr->evt_type);
		if (val & (RMID_VAL_ERROR | RMID_VAL_UNAVAIL))
			return;
		atomic64_add(val, &rr->value);
	}
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
		grp_rmid = iter->hw.cqm_rmid[pkg_id];
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
				update_sample(iter1->hw.cqm_rmid[pkg_id],
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

	for (i = 0; i < cqm_socket_max; i++) {
		hr = &mbm_timers[i];
		hrtimer_init(hr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		hr->function = mbm_hrtimer_handle;
	}
}

static void cqm_mask_call_local(struct rmid_read *rr)
{
	if (is_mbm_event(rr->evt_type))
		__intel_mbm_event_count(rr);
	else
		__intel_cqm_event_count(rr);
}

static inline void
	delta_local(struct perf_event *event, struct rmid_read *rr, u32 *rmid)
{
	atomic64_set(&rr->value, 0);
	rr->rmid = ACCESS_ONCE(rmid);

	cqm_mask_call_local(rr);
	local64_add(atomic64_read(&rr->value), &event->count);
}

/*
 * Since cgroup follows hierarchy, add the count of
 *  the descendents who were being monitored as well.
 */
static u64 cqm_read_subtree(struct perf_event *event, struct rmid_read *rr)
{
#ifdef CONFIG_CGROUP_PERF

	struct cgroup_subsys_state *rcss, *pos_css;
	struct cgrp_cqm_info *ccqm_info;

	cqm_mask_call_local(rr);
	local64_set(&event->count, atomic64_read(&(rr->value)));

	if (is_task_event(event))
		return __perf_event_count(event);

	rcu_read_lock();
	rcss = &event->cgrp->css;
	css_for_each_descendant_pre(pos_css, rcss) {
		ccqm_info = (css_to_cqm_info(pos_css));

		/* Add the descendent 'monitored cgroup' counts */
		if (pos_css != rcss && ccqm_info->mon_enabled)
			delta_local(event, rr, ccqm_info->rmid);
	}
	rcu_read_unlock();
#endif
	return __perf_event_count(event);
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

void alloc_needed_pkg_rmid(u32 *cqm_rmid)
{
	unsigned long flags;
	u32 rmid;

	if (WARN_ON(!cqm_rmid))
		return;

	if (cqm_rmid == cqm_rootcginfo.rmid || cqm_rmid[pkg_id])
		return;

	raw_spin_lock_irqsave(&cache_lock, flags);

	rmid = __get_rmid(pkg_id);
	if (__rmid_valid(rmid))
		cqm_rmid[pkg_id] = rmid;

	raw_spin_unlock_irqrestore(&cache_lock, flags);
}

static void intel_cqm_event_start(struct perf_event *event, int mode)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);

	if (!(event->hw.cqm_state & PERF_HES_STOPPED))
		return;

	event->hw.cqm_state &= ~PERF_HES_STOPPED;

	if (is_task_event(event)) {
		alloc_needed_pkg_rmid(event->hw.cqm_rmid);
		state->next_task_rmid = event->hw.cqm_rmid[pkg_id];
	}
}

static void intel_cqm_event_stop(struct perf_event *event, int mode)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);

	if (event->hw.cqm_state & PERF_HES_STOPPED)
		return;

	event->hw.cqm_state |= PERF_HES_STOPPED;
	state->next_task_rmid = 0;
}

static int intel_cqm_event_add(struct perf_event *event, int mode)
{
	event->hw.cqm_state = PERF_HES_STOPPED;

	if ((mode & PERF_EF_START))
		intel_cqm_event_start(event, mode);

	return 0;
}

static inline void
	cqm_event_free_rmid(struct perf_event *event)
{
	u32 *rmid = event->hw.cqm_rmid;
	int d;

	for (d = 0; d < cqm_socket_max; d++) {
		if (__rmid_valid(rmid[d]))
			__put_rmid(rmid[d], d);
	}
	kfree(event->hw.cqm_rmid);
	cqm_assign_rmid(event, NULL);
	list_del(&event->hw.cqm_groups_entry);
}

static void intel_cqm_event_terminate(struct perf_event *event)
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
		if (group_other)
			list_replace(&event->hw.cqm_groups_entry,
				     &group_other->hw.cqm_groups_entry);
		else
			cqm_event_free_rmid(event);
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

	/*
	 * CQM driver handles cgroup recursion and since only noe
	 * RMID can be programmed at the time in each core, then
	 * it is incompatible with the way generic code handles
	 * cgroup hierarchies.
	 */
	event->event_caps |= PERF_EV_CAP_CGROUP_NO_RECURSION;

	mutex_lock(&cache_mutex);

	/* Delay allocating RMIDs */
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
	.event_terminate     = intel_cqm_event_terminate,
	.add		     = intel_cqm_event_add,
	.del		     = intel_cqm_event_stop,
	.start		     = intel_cqm_event_start,
	.stop		     = intel_cqm_event_stop,
	.read		     = intel_cqm_event_read,
	.count		     = intel_cqm_event_count,
};

#ifdef CONFIG_CGROUP_PERF
int perf_cgroup_arch_css_alloc(struct cgroup_subsys_state *parent_css,
				      struct cgroup_subsys_state *new_css)
{
	struct cgrp_cqm_info *cqm_info, *pcqm_info;
	struct perf_cgroup *new_cgrp;

	if (!parent_css) {
		cqm_rootcginfo.level = 0;

		cqm_rootcginfo.mon_enabled = true;
		cqm_rootcginfo.cont_mon = true;
		cqm_rootcginfo.mfa = NULL;
		INIT_LIST_HEAD(&cqm_rootcginfo.tskmon_rlist);

		if (new_css) {
			new_cgrp = css_to_perf_cgroup(new_css);
			new_cgrp->arch_info = &cqm_rootcginfo;
		}
		return 0;
	}

	mutex_lock(&cache_mutex);

	new_cgrp = css_to_perf_cgroup(new_css);

	cqm_info = kzalloc(sizeof(struct cgrp_cqm_info), GFP_KERNEL);
	if (!cqm_info) {
		mutex_unlock(&cache_mutex);
		return -ENOMEM;
	}

	pcqm_info = (css_to_cqm_info(parent_css));
	cqm_info->level = pcqm_info->level + 1;
	cqm_info->rmid = pcqm_info->rmid;

	cqm_info->cont_mon = false;
	cqm_info->mon_enabled = false;
	INIT_LIST_HEAD(&cqm_info->tskmon_rlist);
	if (!pcqm_info->mfa)
		cqm_info->mfa = pcqm_info;
	else
		cqm_info->mfa = pcqm_info->mfa;

	new_cgrp->arch_info = cqm_info;
	mutex_unlock(&cache_mutex);

	return 0;
}

void perf_cgroup_arch_css_free(struct cgroup_subsys_state *css)
{
	struct perf_cgroup *cgrp = css_to_perf_cgroup(css);

	mutex_lock(&cache_mutex);
	kfree(cgrp_to_cqm_info(cgrp));
	cgrp->arch_info = NULL;
	mutex_unlock(&cache_mutex);
}

void perf_cgroup_arch_attach(struct cgroup_taskset *tset)
{}
int perf_cgroup_arch_can_attach(struct cgroup_taskset *tset)
{}
#endif

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

static int pkg_data_init_cpu(int cpu)
{
	struct cqm_rmid_entry *ccqm_rmid_ptrs = NULL, *entry = NULL;
	int curr_pkgid = topology_physical_package_id(cpu);
	struct pkg_data *pkg_data = NULL;
	int i = 0, nr_rmids, ret = 0;

	if (cqm_pkgs_data[curr_pkgid])
		return 0;

	pkg_data = kzalloc_node(sizeof(struct pkg_data),
				GFP_KERNEL, cpu_to_node(cpu));
	if (!pkg_data)
		return -ENOMEM;

	INIT_LIST_HEAD(&pkg_data->cqm_rmid_free_lru);
	INIT_LIST_HEAD(&pkg_data->cqm_rmid_limbo_lru);

	mutex_init(&pkg_data->pkg_data_mutex);
	raw_spin_lock_init(&pkg_data->pkg_data_lock);

	pkg_data->rmid_work_cpu = cpu;

	nr_rmids = cqm_max_rmid + 1;
	ccqm_rmid_ptrs = kzalloc(sizeof(struct cqm_rmid_entry) *
			   nr_rmids, GFP_KERNEL);
	if (!ccqm_rmid_ptrs) {
		ret = -ENOMEM;
		goto fail;
	}

	for (; i <= cqm_max_rmid; i++) {
		entry = &ccqm_rmid_ptrs[i];
		INIT_LIST_HEAD(&entry->list);
		entry->rmid = i;

		list_add_tail(&entry->list, &pkg_data->cqm_rmid_free_lru);
	}

	pkg_data->cqm_rmid_ptrs = ccqm_rmid_ptrs;
	cqm_pkgs_data[curr_pkgid] = pkg_data;

	/*
	 * RMID 0 is special and is always allocated. It's used for all
	 * tasks that are not monitored.
	 */
	entry = __rmid_entry(0, curr_pkgid);
	list_del(&entry->list);

	cqm_rootcginfo.rmid = kzalloc(sizeof(u32) * cqm_socket_max, GFP_KERNEL);
	if (!cqm_rootcginfo.rmid) {
		ret = -ENOMEM;
		goto fail;
	}

	return 0;
fail:
	kfree(ccqm_rmid_ptrs);
	ccqm_rmid_ptrs = NULL;
	kfree(pkg_data);
	pkg_data = NULL;
	cqm_pkgs_data[curr_pkgid] = NULL;
	return ret;
}

static int cqm_init_pkgs_data(void)
{
	int i, cpu, ret = 0;

	cqm_pkgs_data = kzalloc(
		sizeof(struct pkg_data *) * cqm_socket_max,
		GFP_KERNEL);
	if (!cqm_pkgs_data)
		return -ENOMEM;

	for (i = 0; i < cqm_socket_max; i++)
		cqm_pkgs_data[i] = NULL;

	for_each_online_cpu(cpu) {
		ret = pkg_data_init_cpu(cpu);
		if (ret)
			goto fail;
	}

	return 0;
fail:
	cqm_cleanup();
	return ret;
}

static int intel_mbm_init(void)
{
	int ret = 0, array_size, maxid = cqm_max_rmid + 1;

	array_size = sizeof(struct sample) * maxid * cqm_socket_max;
	mbm_local = kmalloc(array_size, GFP_KERNEL);
	if (!mbm_local)
		return -ENOMEM;

	mbm_total = kmalloc(array_size, GFP_KERNEL);
	if (!mbm_total) {
		ret = -ENOMEM;
		goto out;
	}

	array_size = sizeof(struct hrtimer) * cqm_socket_max;
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

	cqm_socket_max = topology_max_packages();
	ret = cqm_init_pkgs_data();
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

	static_branch_enable(&cqm_enable_key);

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
		cqm_enabled = false;
		mbm_cleanup();
	}

	return ret;
}
device_initcall(intel_cqm_init);
