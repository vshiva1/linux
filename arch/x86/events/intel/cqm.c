/*
 * Intel Cache Quality-of-Service Monitoring (CQM) support.
 *
 * Based very, very heavily on work by Peter Zijlstra.
 */

#include <linux/slab.h>
#include <asm/cpu_device_id.h>
#include "cqm.h"
#include "../perf_event.h"

#define MSR_IA32_QM_CTR		0x0c8e
#define MSR_IA32_QM_EVTSEL	0x0c8d

static unsigned int cqm_l3_scale; /* supposedly cacheline size */

#define RMID_VAL_ERROR		(1ULL << 63)
#define RMID_VAL_UNAVAIL	(1ULL << 62)

#define QOS_L3_OCCUP_EVENT_ID	(1 << 0)

#define QOS_EVENT_MASK		QOS_L3_OCCUP_EVENT_ID

/*
 * Update if enough time has passed since last read.
 *
 * Must be called in a cpu in the package where prmid belongs.
 * This function is safe to be called concurrently since it is guaranteed
 * that entry->last_read_value is updated to a occupancy value obtained
 * after the time set in entry->last_read_time .
 * Return 1 if value was updated, 0 if not, negative number if error.
 */
static inline int __cqm_prmid_update(struct prmid *prmid,
				     unsigned long jiffies_min_delta)
{
	unsigned long now = jiffies;
	unsigned long last_read_time;
	u64 val;

	/*
	 * Shortcut the calculation of elapsed time for the
	 * case jiffies_min_delta == 0
	 */
	if (jiffies_min_delta > 0) {
		last_read_time = atomic64_read(&prmid->last_read_time);
		if (time_after(last_read_time + jiffies_min_delta, now))
			return 0;
	}

	wrmsr(MSR_IA32_QM_EVTSEL, QOS_L3_OCCUP_EVENT_ID, prmid->rmid);
	rdmsrl(MSR_IA32_QM_CTR, val);

	/*
	 * Ignore this reading on error states and do not update the value.
	 */
	WARN_ON_ONCE(val & (RMID_VAL_ERROR | RMID_VAL_UNAVAIL));
	if (val & RMID_VAL_ERROR)
		return -EINVAL;
	if (val & RMID_VAL_UNAVAIL)
		return -ENODATA;

	atomic64_set(&prmid->last_read_value, val);
	/*
	 * Protect last_read_time from being updated before last_read_value is.
	 * So reader always receive an updated value even if sometimes values
	 * are updated twice.
	 */
	smp_wmb();

	atomic64_set(&prmid->last_read_time, now);

	return 1;
}

static inline int cqm_prmid_update(struct prmid *prmid)
{
	return __cqm_prmid_update(prmid, __rmid_min_update_time);
}

/*
 * A cache groups is a group of perf_events with the same target (thread,
 * cgroup, CPU or system-wide). Each cache group receives has one RMID.
 * Cache groups are protected by cqm_mutex.
 */
static LIST_HEAD(cache_groups);
static DEFINE_MUTEX(cqm_mutex);

struct monr *monr_hrchy_root;

struct pkg_data **cqm_pkgs_data;

static inline bool __pmonr__in_astate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	return pmonr->prmid;
}

static inline bool __pmonr__in_ustate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	return !pmonr->prmid;
}

static inline bool monr__is_root(struct monr *monr)
{
	return monr_hrchy_root == monr;
}

static inline bool monr__is_mon_active(struct monr *monr)
{
	return monr->flags & MONR_MON_ACTIVE;
}

static inline void __monr__set_summary_read_rmid(struct monr *monr, u32 rmid)
{
	int i;
	struct pmonr *pmonr;
	union prmid_summary summary;

	monr_hrchy_assert_held_raw_spin_locks();

	cqm_pkg_id_for_each_online(i) {
		pmonr = monr->pmonrs[i];
		WARN_ON_ONCE(!__pmonr__in_ustate(pmonr));
		summary.value = atomic64_read(&pmonr->prmid_summary_atomic);
		summary.read_rmid = rmid;
		atomic64_set(&pmonr->prmid_summary_atomic, summary.value);
	}
}

static inline void __monr__set_mon_active(struct monr *monr)
{
	monr_hrchy_assert_held_raw_spin_locks();
	__monr__set_summary_read_rmid(monr, 0);
	monr->flags |= MONR_MON_ACTIVE;
}

/*
 * All pmonrs must be in (U)state.
 * clearing MONR_MON_ACTIVE prevents (U)state prmids from transitioning
 * to another state.
 */
static inline void __monr__clear_mon_active(struct monr *monr)
{
	monr_hrchy_assert_held_raw_spin_locks();
	__monr__set_summary_read_rmid(monr, INVALID_RMID);
	monr->flags &= ~MONR_MON_ACTIVE;
}

static inline bool __valid_pkg_id(u16 pkg_id)
{
	return pkg_id < topology_max_packages();
}

/* Init cqm pkg_data for @cpu 's package. */
static int pkg_data_init_cpu(int cpu)
{
	struct pkg_data *pkg_data;
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	u16 pkg_id = topology_physical_package_id(cpu);

	if (cqm_pkgs_data[pkg_id])
		return 0;


	pkg_data = kmalloc_node(sizeof(struct pkg_data),
				GFP_KERNEL, cpu_to_node(cpu));
	if (!pkg_data)
		return -ENOMEM;

	pkg_data->max_rmid = c->x86_cache_max_rmid;

	/* Does hardware has more rmids than this driver can handle? */
	if (WARN_ON(pkg_data->max_rmid >= INVALID_RMID))
		pkg_data->max_rmid = INVALID_RMID - 1;

	if (c->x86_cache_occ_scale != cqm_l3_scale) {
		pr_err("Multiple LLC scale values, disabling\n");
		kfree(pkg_data);
		return -EINVAL;
	}

	pkg_data->prmids_by_rmid = kmalloc_node(
		sizeof(struct prmid *) * (1 + pkg_data->max_rmid),
		GFP_KERNEL, cpu_to_node(cpu));

	if (!pkg_data) {
		kfree(pkg_data);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&pkg_data->free_prmids_pool);
	INIT_LIST_HEAD(&pkg_data->active_prmids_pool);
	INIT_LIST_HEAD(&pkg_data->nopmonr_limbo_prmids_pool);

	INIT_LIST_HEAD(&pkg_data->astate_pmonrs_lru);

	mutex_init(&pkg_data->pkg_data_mutex);
	raw_spin_lock_init(&pkg_data->pkg_data_lock);

	/* XXX: Chose randomly*/
	pkg_data->rotation_cpu = cpu;

	cqm_pkgs_data[pkg_id] = pkg_data;
	return 0;
}

static inline bool __valid_rmid(u16 pkg_id, u32 rmid)
{
	return rmid <= cqm_pkgs_data[pkg_id]->max_rmid;
}

static inline bool __valid_prmid(u16 pkg_id, struct prmid *prmid)
{
	struct pkg_data *pkg_data = cqm_pkgs_data[pkg_id];
	bool valid = __valid_rmid(pkg_id, prmid->rmid);

	WARN_ON_ONCE(valid && pkg_data->prmids_by_rmid[
			prmid->rmid]->rmid != prmid->rmid);
	return valid;
}

static inline struct prmid *
__prmid_from_rmid(u16 pkg_id, u32 rmid)
{
	struct prmid *prmid;

	if (!__valid_rmid(pkg_id, rmid))
		return NULL;
	prmid = cqm_pkgs_data[pkg_id]->prmids_by_rmid[rmid];
	WARN_ON_ONCE(!__valid_prmid(pkg_id, prmid));
	return prmid;
}

static struct pmonr *pmonr_alloc(int cpu)
{
	struct pmonr *pmonr;
	union prmid_summary summary;

	pmonr = kmalloc_node(sizeof(struct pmonr),
			     GFP_KERNEL, cpu_to_node(cpu));
	if (!pmonr)
		return ERR_PTR(-ENOMEM);

	pmonr->prmid = NULL;

	pmonr->monr = NULL;
	INIT_LIST_HEAD(&pmonr->rotation_entry);

	pmonr->pkg_id = topology_physical_package_id(cpu);
	summary.sched_rmid = INVALID_RMID;
	summary.read_rmid = INVALID_RMID;
	atomic64_set(&pmonr->prmid_summary_atomic, summary.value);

	return pmonr;
}

static void pmonr_dealloc(struct pmonr *pmonr)
{
	kfree(pmonr);
}

/*
 * @root: Common ancestor.
 * a bust be distinct to b.
 * @true if a is ancestor of b.
 */
static inline bool
__monr_hrchy_is_ancestor(struct monr *root,
			 struct monr *a, struct monr *b)
{
	WARN_ON_ONCE(!root || !a || !b);
	WARN_ON_ONCE(a == b);

	if (root == a)
		return true;
	if (root == b)
		return false;

	b = b->parent;
	/* Break at the root */
	while (b != root) {
		WARN_ON_ONCE(!b);
		if (a == b)
			return true;
		b = b->parent;
	}
	return false;
}

/* helper function to finish transition to astate. */
static inline void
__pmonr__finish_to_astate(struct pmonr *pmonr, struct prmid *prmid)
{
	union prmid_summary summary;

	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));

	pmonr->prmid = prmid;

	list_move_tail(
		&prmid->pool_entry, &__pkg_data(pmonr, active_prmids_pool));
	list_move_tail(
		&pmonr->rotation_entry, &__pkg_data(pmonr, astate_pmonrs_lru));

	summary.sched_rmid = pmonr->prmid->rmid;
	summary.read_rmid = pmonr->prmid->rmid;
	atomic64_set(&pmonr->prmid_summary_atomic, summary.value);
}

static inline void
__pmonr__ustate_to_astate(struct pmonr *pmonr, struct prmid *prmid)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	__pmonr__finish_to_astate(pmonr, prmid);
}

static inline void
__pmonr__to_ustate(struct pmonr *pmonr)
{
	union prmid_summary summary;

	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));

	/* Do not warn on re-enter state for (U)state, to simplify cleanup
	 * of initialized states that were not scheduled.
	 */
	if (__pmonr__in_ustate(pmonr))
		return;

	if (__pmonr__in_astate(pmonr)) {
		WARN_ON_ONCE(!pmonr->prmid);

		list_move_tail(&pmonr->prmid->pool_entry,
			       &__pkg_data(pmonr, nopmonr_limbo_prmids_pool));
		pmonr->prmid =  NULL;
	} else {
		WARN_ON_ONCE(true);
		return;
	}
	list_del_init(&pmonr->rotation_entry);

	summary.sched_rmid = INVALID_RMID;
	summary.read_rmid  =
		monr__is_mon_active(pmonr->monr) ? 0 : INVALID_RMID;

	atomic64_set(&pmonr->prmid_summary_atomic, summary.value);
	WARN_ON_ONCE(!__pmonr__in_ustate(pmonr));
}

static int intel_cqm_setup_pkg_prmid_pools(u16 pkg_id)
{
	int r;
	unsigned long flags;
	struct prmid *prmid;
	struct pkg_data *pkg_data = cqm_pkgs_data[pkg_id];
	struct pmonr *root_pmonr;

	if (!__valid_pkg_id(pkg_id))
		return -EINVAL;

	for (r = 0; r <= pkg_data->max_rmid; r++) {

		prmid = kmalloc_node(sizeof(struct prmid), GFP_KERNEL,
				     cpu_to_node(pkg_data->rotation_cpu));
		if (!prmid)
			goto fail;

		atomic64_set(&prmid->last_read_value, 0L);
		atomic64_set(&prmid->last_read_time, 0L);
		INIT_LIST_HEAD(&prmid->pool_entry);
		prmid->rmid = r;

		/* Lock needed if called during CPU hotplug. */
		raw_spin_lock_irqsave_nested(
			&pkg_data->pkg_data_lock, flags, pkg_id);
		pkg_data->prmids_by_rmid[r] = prmid;

		list_add_tail(&prmid->pool_entry, &pkg_data->free_prmids_pool);

		/* RMID 0 is special and makes the root of rmid hierarchy. */
		if (r == 0) {
			root_pmonr = monr_hrchy_root->pmonrs[pkg_id];
			__pmonr__ustate_to_astate(root_pmonr, prmid);
		}
		raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	}
	return 0;
fail:
	while (!list_empty(&pkg_data->free_prmids_pool)) {
		prmid = list_first_entry(&pkg_data->free_prmids_pool,
					 struct prmid, pool_entry);
		list_del(&prmid->pool_entry);
		kfree(pkg_data->prmids_by_rmid[prmid->rmid]);
		kfree(prmid);
	}
	return -ENOMEM;
}


/* Alloc monr with all pmonrs in (U)state. */
static struct monr *monr_alloc(void)
{
	int i;
	struct pmonr *pmonr;
	struct monr *monr;

	monr = kmalloc(sizeof(struct monr), GFP_KERNEL);

	if (!monr)
		return ERR_PTR(-ENOMEM);

	monr->flags = 0;
	monr->parent = NULL;
	INIT_LIST_HEAD(&monr->children);
	INIT_LIST_HEAD(&monr->parent_entry);
	monr->mon_event_group = NULL;

	monr->pmonrs = kmalloc(
		sizeof(struct pmonr *) * topology_max_packages(), GFP_KERNEL);

	if (!monr->pmonrs)
		return ERR_PTR(-ENOMEM);

	/* Iterate over all pkgs, even unitialized ones. */
	for (i = 0; i < topology_max_packages(); i++) {
		/* Do not create pmonrs for unitialized packages. */
		if (!cqm_pkgs_data[i]) {
			monr->pmonrs[i] = NULL;
			continue;
		}
		/* Rotation cpu is on pmonr's package. */
		pmonr = pmonr_alloc(cqm_pkgs_data[i]->rotation_cpu);
		if (IS_ERR(pmonr))
			goto clean_pmonrs;
		pmonr->monr = monr;
		monr->pmonrs[i] = pmonr;
	}
	return monr;

clean_pmonrs:
	while (i--) {
		if (cqm_pkgs_data[i])
			kfree(monr->pmonrs[i]);
	}
	kfree(monr);
	return ERR_CAST(pmonr);
}

/* Only can dealloc monrs with all pmonrs in (U)state. */
static void monr_dealloc(struct monr *monr)
{
	int i;

	cqm_pkg_id_for_each_online(i)
		pmonr_dealloc(monr->pmonrs[i]);

	kfree(monr);
}

/*
 * Wrappers for monr manipulation in events.
 *
 */
static inline struct monr *monr_from_event(struct perf_event *event)
{
	return (struct monr *) READ_ONCE(event->hw.cqm_monr);
}

static inline void event_set_monr(struct perf_event *event, struct monr *monr)
{
	WRITE_ONCE(event->hw.cqm_monr, monr);
}

/*
 * Always finds a rmid_entry to schedule. To be called during scheduler.
 * A fast path that only uses read_lock for common case when rmid for current
 * package has been used before.
 * On failure, verify that monr is active, if it is, try to obtain a free rmid
 * and set pmonr to (A)state.
 * On failure, transverse up monr_hrchy until finding one prmid for this
 * pkg_id and set pmonr to (I)state.
 * Called during task switch, it will set pmonr's prmid_summary to reflect the
 * sched and read rmids that reflect pmonr's state.
 */
static inline void
monr_hrchy_get_next_prmid_summary(struct pmonr *pmonr)
{
	union prmid_summary summary;

	/*
	 * First, do lock-free fastpath.
	 */
	summary.value = atomic64_read(&pmonr->prmid_summary_atomic);
	if (summary.sched_rmid != INVALID_RMID)
		return;

	if (!prmid_summary__is_mon_active(summary))
		return;

	/*
	 * Lock-free path failed at first attempt. Now acquire lock and repeat
	 * in case the monr was modified in the mean time.
	 * This time try to obtain free rmid and update pmonr accordingly,
	 * instead of failing fast.
	 */
	raw_spin_lock_nested(&__pkg_data(pmonr, pkg_data_lock), pmonr->pkg_id);

	summary.value = atomic64_read(&pmonr->prmid_summary_atomic);
	if (summary.sched_rmid != INVALID_RMID) {
		raw_spin_unlock(&__pkg_data(pmonr, pkg_data_lock));
		return;
	}

	/* Do not try to obtain RMID if monr is not active. */
	if (!prmid_summary__is_mon_active(summary)) {
		raw_spin_unlock(&__pkg_data(pmonr, pkg_data_lock));
		return;
	}

	/*
	 * Can only fail if it was in (U)state.
	 * Try to obtain a free prmid and go to (A)state, if not possible,
	 * it should go to (I)state.
	 */
	WARN_ON_ONCE(!__pmonr__in_ustate(pmonr));

	if (list_empty(&__pkg_data(pmonr, free_prmids_pool))) {
		/* Failed to obtain an valid rmid in this package for this
		 * monr. In next patches it will transition to (I)state.
		 * For now, stay in (U)state (do nothing).
		 */
	} else {
		/* Transition to (A)state using free prmid. */
		__pmonr__ustate_to_astate(
			pmonr,
			list_first_entry(&__pkg_data(pmonr, free_prmids_pool),
				struct prmid, pool_entry));
	}
	raw_spin_unlock(&__pkg_data(pmonr, pkg_data_lock));
}

static inline void __assert_monr_is_leaf(struct monr *monr)
{
	int i;

	monr_hrchy_assert_held_mutexes();
	monr_hrchy_assert_held_raw_spin_locks();

	cqm_pkg_id_for_each_online(i)
		WARN_ON_ONCE(!__pmonr__in_ustate(monr->pmonrs[i]));

	WARN_ON_ONCE(!list_empty(&monr->children));
}

static inline void
__monr_hrchy_insert_leaf(struct monr *monr, struct monr *parent)
{
	monr_hrchy_assert_held_mutexes();
	monr_hrchy_assert_held_raw_spin_locks();

	__assert_monr_is_leaf(monr);

	list_add_tail(&monr->parent_entry, &parent->children);
	monr->parent = parent;
}

static inline void
__monr_hrchy_remove_leaf(struct monr *monr)
{
	/* Since root cannot be removed, monr must have a parent */
	WARN_ON_ONCE(!monr->parent);

	monr_hrchy_assert_held_mutexes();
	monr_hrchy_assert_held_raw_spin_locks();

	__assert_monr_is_leaf(monr);

	list_del_init(&monr->parent_entry);
	monr->parent = NULL;
}

static int __monr_hrchy_attach_cpu_event(struct perf_event *event)
{
	lockdep_assert_held(&cqm_mutex);
	WARN_ON_ONCE(monr_from_event(event));

	event_set_monr(event, monr_hrchy_root);
	return 0;
}

/* task events are always leaves in the monr_hierarchy */
static int __monr_hrchy_attach_task_event(struct perf_event *event,
					  struct monr *parent_monr)
{
	struct monr *monr;
	unsigned long flags;
	int i;

	lockdep_assert_held(&cqm_mutex);

	monr = monr_alloc();
	if (IS_ERR(monr))
		return PTR_ERR(monr);
	event_set_monr(event, monr);
	monr->mon_event_group = event;

	monr_hrchy_acquire_locks(flags, i);
	__monr_hrchy_insert_leaf(monr, parent_monr);
	__monr__set_mon_active(monr);
	monr_hrchy_release_locks(flags, i);

	return 0;
}

/*
 * Find appropriate position in hierarchy and set monr. Create new
 * monr if necessary.
 * Locks rmid hrchy.
 */
static int monr_hrchy_attach_event(struct perf_event *event)
{
	struct monr *monr_parent;

	if (!event->cgrp && !(event->attach_state & PERF_ATTACH_TASK))
		return __monr_hrchy_attach_cpu_event(event);

	/* Two-levels hierarchy: Root and all event monr underneath it. */
	monr_parent = monr_hrchy_root;
	return __monr_hrchy_attach_task_event(event, monr_parent);
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

	/* If not task event, it's a a cgroup or a non-task cpu event. */
	if (!(b->attach_state & PERF_ATTACH_TASK))
		return true;

	/*
	 * Events that target same task are placed into the same cache group.
	 */
	if (a->hw.target == b->hw.target)
		return true;

	/*
	 * Are we an inherited event?
	 */
	if (b->parent == a)
		return true;

	return false;
}

static struct pmu intel_cqm_pmu;

/*
 * Find a group and setup RMID.
 *
 * If we're part of a group, we use the group's monr.
 */
static int
intel_cqm_setup_event(struct perf_event *event, struct perf_event **group)
{
	struct perf_event *iter;
	struct monr *monr;
	*group = NULL;

	lockdep_assert_held(&cqm_mutex);

	list_for_each_entry(iter, &cache_groups, hw.cqm_event_groups_entry) {
		monr = monr_from_event(iter);
		if (__match_event(iter, event)) {
			/* All tasks in a group share an monr. */
			event_set_monr(event, monr);
			*group = iter;
			return 0;
		}
	}
	/*
	 * Since no match was found, create a new monr and set this
	 * event as head of a new cache group. All events in this cache group
	 * will share the monr.
	 */
	return monr_hrchy_attach_event(event);
}

/* Read current package immediately and remote pkg (if any) from cache. */
static void intel_cqm_event_read(struct perf_event *event)
{
	union prmid_summary summary;
	struct prmid *prmid;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
	struct pmonr *pmonr = monr_from_event(event)->pmonrs[pkg_id];

	summary.value = atomic64_read(&pmonr->prmid_summary_atomic);
	prmid = __prmid_from_rmid(pkg_id, summary.read_rmid);
	cqm_prmid_update(prmid);
	local64_set(&event->count, atomic64_read(&prmid->last_read_value));
}

static inline void __intel_cqm_event_start(
	struct perf_event *event, union prmid_summary summary)
{
	if (!(event->hw.state & PERF_HES_STOPPED))
		return;

	event->hw.state &= ~PERF_HES_STOPPED;
	pqr_update_rmid(summary.sched_rmid);
}

static void intel_cqm_event_start(struct perf_event *event, int mode)
{
	union prmid_summary summary;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
	struct pmonr *pmonr = monr_from_event(event)->pmonrs[pkg_id];

	/* Utilize most up to date pmonr summary. */
	monr_hrchy_get_next_prmid_summary(pmonr);
	summary.value = atomic64_read(&pmonr->prmid_summary_atomic);
	__intel_cqm_event_start(event, summary);
}

static void intel_cqm_event_stop(struct perf_event *event, int mode)
{
	union prmid_summary summary;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
	struct pmonr *root_pmonr = monr_hrchy_root->pmonrs[pkg_id];

	if (event->hw.state & PERF_HES_STOPPED)
		return;

	event->hw.state |= PERF_HES_STOPPED;

	summary.value = atomic64_read(&root_pmonr->prmid_summary_atomic);
	/* Occupancy of CQM events is obtained at read. No need to read
	 * when event is stopped since read on inactive cpus succeed.
	 */
	pqr_update_rmid(summary.sched_rmid);
}

static int intel_cqm_event_add(struct perf_event *event, int mode)
{
	struct monr *monr;
	struct pmonr *pmonr;
	union prmid_summary summary;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());

	monr = monr_from_event(event);
	pmonr = monr->pmonrs[pkg_id];

	event->hw.state = PERF_HES_STOPPED;

	/* Utilize most up to date pmonr summary. */
	monr_hrchy_get_next_prmid_summary(pmonr);
	summary.value = atomic64_read(&pmonr->prmid_summary_atomic);

	if (!prmid_summary__is_mon_active(summary))
		return -1;

	if (mode & PERF_EF_START)
		__intel_cqm_event_start(event, summary);

	/* (I)state pmonrs cannot report occupancy for themselves. */
	return 0;
}

static inline bool cqm_group_leader(struct perf_event *event)
{
	return !list_empty(&event->hw.cqm_event_groups_entry);
}

static void intel_cqm_event_destroy(struct perf_event *event)
{
	struct perf_event *group_other = NULL;
	struct monr *monr;
	int i;
	unsigned long flags;

	mutex_lock(&cqm_mutex);
	/*
	 * If there's another event in this group...
	 */
	if (!list_empty(&event->hw.cqm_event_group_entry)) {
		group_other = list_first_entry(&event->hw.cqm_event_group_entry,
					       struct perf_event,
					       hw.cqm_event_group_entry);
		list_del(&event->hw.cqm_event_group_entry);
	}
	/*
	 * And we're the group leader..
	 */
	if (!cqm_group_leader(event))
		goto exit;

	monr = monr_from_event(event);

	/*
	 * If there was a group_other, make that leader, otherwise
	 * destroy the group and return the RMID.
	 */
	if (group_other) {
		/* Update monr reference to group head. */
		monr->mon_event_group = group_other;
		list_replace(&event->hw.cqm_event_groups_entry,
			     &group_other->hw.cqm_event_groups_entry);
		goto exit;
	}

	/*
	 * Event is the only event in cache group.
	 */

	event_set_monr(event, NULL);
	list_del(&event->hw.cqm_event_groups_entry);

	if (monr__is_root(monr))
		goto exit;

	/* Transition all pmonrs to (U)state. */
	monr_hrchy_acquire_locks(flags, i);

	cqm_pkg_id_for_each_online(i)
		__pmonr__to_ustate(monr->pmonrs[i]);

	__monr__clear_mon_active(monr);
	monr->mon_event_group = NULL;
	__monr_hrchy_remove_leaf(monr);
	monr_hrchy_release_locks(flags, i);

	monr_dealloc(monr);
exit:
	mutex_unlock(&cqm_mutex);
}

static int intel_cqm_event_init(struct perf_event *event)
{
	struct perf_event *group = NULL;
	int ret;

	if (event->attr.type != intel_cqm_pmu.type)
		return -ENOENT;

	if (event->attr.config & ~QOS_EVENT_MASK)
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

	INIT_LIST_HEAD(&event->hw.cqm_event_groups_entry);
	INIT_LIST_HEAD(&event->hw.cqm_event_group_entry);

	event->destroy = intel_cqm_event_destroy;

	mutex_lock(&cqm_mutex);


	/* Will also set rmid */
	ret = intel_cqm_setup_event(event, &group);
	if (ret) {
		mutex_unlock(&cqm_mutex);
		return ret;
	}

	if (group) {
		list_add_tail(&event->hw.cqm_event_group_entry,
				&group->hw.cqm_event_group_entry);
	} else {
		list_add_tail(&event->hw.cqm_event_groups_entry,
				&cache_groups);
	}

	mutex_unlock(&cqm_mutex);

	return 0;
}

EVENT_ATTR_STR(llc_occupancy, intel_cqm_llc, "event=0x01");
EVENT_ATTR_STR(llc_occupancy.per-pkg, intel_cqm_llc_pkg, "1");
EVENT_ATTR_STR(llc_occupancy.unit, intel_cqm_llc_unit, "Bytes");
EVENT_ATTR_STR(llc_occupancy.scale, intel_cqm_llc_scale, NULL);
EVENT_ATTR_STR(llc_occupancy.snapshot, intel_cqm_llc_snapshot, "1");

static struct attribute *intel_cqm_events_attr[] = {
	EVENT_PTR(intel_cqm_llc),
	EVENT_PTR(intel_cqm_llc_pkg),
	EVENT_PTR(intel_cqm_llc_unit),
	EVENT_PTR(intel_cqm_llc_scale),
	EVENT_PTR(intel_cqm_llc_snapshot),
	NULL,
};

static struct attribute_group intel_cqm_events_group = {
	.name = "events",
	.attrs = intel_cqm_events_attr,
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
max_recycle_threshold_show(
	struct device *dev, struct device_attribute *attr, char *page)
{
	ssize_t rv;

	monr_hrchy_acquire_mutexes();
	rv = snprintf(page, PAGE_SIZE - 1, "%u\n",
		      __intel_cqm_max_threshold);
	monr_hrchy_release_mutexes();

	return rv;
}

static ssize_t
max_recycle_threshold_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int bytes;
	int ret;

	ret = kstrtouint(buf, 0, &bytes);
	if (ret)
		return ret;

	/* Mutex waits for rotation logic in all packages to complete. */
	monr_hrchy_acquire_mutexes();

	__intel_cqm_max_threshold = bytes;

	monr_hrchy_release_mutexes();

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
	.hrtimer_interval_ms = CQM_DEFAULT_ROTATION_PERIOD,
	.attr_groups	     = intel_cqm_attr_groups,
	.task_ctx_nr	     = perf_sw_context,
	.event_init	     = intel_cqm_event_init,
	.add		     = intel_cqm_event_add,
	.del		     = intel_cqm_event_stop,
	.start		     = intel_cqm_event_start,
	.stop		     = intel_cqm_event_stop,
	.read		     = intel_cqm_event_read,
};

static inline void cqm_pick_event_reader(int cpu)
{
	u16 pkg_id = topology_physical_package_id(cpu);
	/* XXX: lock, check if rotation cpu is online, maybe */
	/*
	 * Pick a reader if there isn't one already.
	 */
	if (cqm_pkgs_data[pkg_id]->rotation_cpu != -1)
		cqm_pkgs_data[pkg_id]->rotation_cpu = cpu;
}

static void intel_cqm_cpu_starting(unsigned int cpu)
{
	struct intel_pqr_state *state = &per_cpu(pqr_state, cpu);
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	u16 pkg_id = topology_physical_package_id(cpu);

	state->rmid = 0;
	state->closid = 0;

	/* XXX: lock */
	/* XXX: Make sure this case is handled when hotplug happens. */
	WARN_ON(c->x86_cache_max_rmid != cqm_pkgs_data[pkg_id]->max_rmid);
	WARN_ON(c->x86_cache_occ_scale != cqm_l3_scale);
}

static void intel_cqm_cpu_exit(unsigned int cpu)
{
	/*
	 * Is @cpu a designated cqm reader?
	 */
	u16 pkg_id = topology_physical_package_id(cpu);

	if (cqm_pkgs_data[pkg_id]->rotation_cpu != cpu)
		return;
	/* XXX: do remove unused packages */
	cqm_pkgs_data[pkg_id]->rotation_cpu = cpumask_any_but(
		topology_core_cpumask(cpu), cpu);
}

static int intel_cqm_cpu_notifier(struct notifier_block *nb,
				  unsigned long action, void *hcpu)
{
	unsigned int cpu  = (unsigned long)hcpu;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DOWN_PREPARE:
		intel_cqm_cpu_exit(cpu);
		break;
	case CPU_STARTING:
		pkg_data_init_cpu(cpu);
		intel_cqm_cpu_starting(cpu);
		cqm_pick_event_reader(cpu);
		break;
	}

	return NOTIFY_OK;
}

static const struct x86_cpu_id intel_cqm_match[] = {
	{ .vendor = X86_VENDOR_INTEL, .feature = X86_FEATURE_CQM_OCCUP_LLC },
	{}
};

static int __init intel_cqm_init(void)
{
	char *str, scale[20];
	int i, cpu, ret = 0, min_max_rmid = 0;

	if (!x86_match_cpu(intel_cqm_match))
		return -ENODEV;

	cqm_l3_scale = boot_cpu_data.x86_cache_occ_scale;
	if (WARN_ON(cqm_l3_scale == 0))
		cqm_l3_scale = 1;

	cqm_pkgs_data = kmalloc(
		sizeof(struct pkg_data *) * topology_max_packages(),
		GFP_KERNEL);
	if (!cqm_pkgs_data)
		return -ENOMEM;

	for (i = 0; i < topology_max_packages(); i++)
		cqm_pkgs_data[i] = NULL;

	/*
	 * It's possible that not all resources support the same number
	 * of RMIDs. Instead of making scheduling much more complicated
	 * (where we have to match a task's RMID to a cpu that supports
	 * that many RMIDs) just find the minimum RMIDs supported across
	 * all cpus.
	 *
	 * Also, check that the scales match on all cpus.
	 */
	cpu_notifier_register_begin();

	/* XXX: assert all cpus in pkg have same nr rmids (they should). */
	for_each_online_cpu(cpu) {
		ret = pkg_data_init_cpu(cpu);
		if  (ret)
			goto error;
	}

	monr_hrchy_root = monr_alloc();
	if (IS_ERR(monr_hrchy_root)) {
		ret = PTR_ERR(monr_hrchy_root);
		goto error;
	}

	/* Select the minimum of the maximum rmids to use as limit for
	 * threshold. XXX: per-package threshold.
	 */
	cqm_pkg_id_for_each_online(i) {
		if (min_max_rmid < cqm_pkgs_data[i]->max_rmid)
			min_max_rmid = cqm_pkgs_data[i]->max_rmid;
		intel_cqm_setup_pkg_prmid_pools(i);
	}
	monr_hrchy_root->flags |= MONR_MON_ACTIVE;

	/*
	 * A reasonable upper limit on the max threshold is the number
	 * of lines tagged per RMID if all RMIDs have the same number of
	 * lines tagged in the LLC.
	 *
	 * For a 35MB LLC and 56 RMIDs, this is ~1.8% of the LLC.
	 */
	__intel_cqm_max_threshold =
		boot_cpu_data.x86_cache_size * 1024 / (min_max_rmid + 1);

	snprintf(scale, sizeof(scale), "%u", cqm_l3_scale);
	str = kstrdup(scale, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto error;
	}

	event_attr_intel_cqm_llc_scale.event_str = str;

	for_each_online_cpu(i) {
		intel_cqm_cpu_starting(i);
		cqm_pick_event_reader(i);
	}

	__perf_cpu_notifier(intel_cqm_cpu_notifier);

	ret = perf_pmu_register(&intel_cqm_pmu, "intel_cqm", -1);
	if (ret)
		goto error;

	cpu_notifier_register_done();

	pr_info("Intel CQM monitoring enabled with at least %u rmids per package.\n",
		min_max_rmid + 1);

	return ret;

error:
	pr_err("Intel CQM perf registration failed: %d\n", ret);
	cpu_notifier_register_done();

	return ret;
}

device_initcall(intel_cqm_init);
