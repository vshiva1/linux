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

int total_events;



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

/*
 * Synchronizes initialization of cqm with cgroups.
 */
static DEFINE_MUTEX(cqm_init_mutex);

DEFINE_STATIC_KEY_FALSE(cqm_initialized_key);

static inline bool __pmonr__in_astate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	return pmonr->prmid && !pmonr->ancestor_pmonr;
}

static inline bool __pmonr__in_ustate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	return !pmonr->prmid && !pmonr->ancestor_pmonr;
}

static inline bool __pmonr__in_istate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	return pmonr->ancestor_pmonr;
}

static inline bool __pmonr__in_ilstate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	return __pmonr__in_istate(pmonr) && pmonr->limbo_prmid;
}

static inline bool __pmonr__in_instate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	return __pmonr__in_istate(pmonr) && !__pmonr__in_ilstate(pmonr);
}

/* Whether the monr is root. Recall that the cgroups can not be root and yet
 * point to a root monr.
 */
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

static inline bool monr_is_event_type(struct monr *monr)
{
	return !monr->mon_cgrp && monr->mon_event_group;
}

#ifdef CONFIG_CGROUP_PERF
static inline struct cgroup_subsys_state *get_root_perf_css(void)
{
	/* Get css for root cgroup */
	return  init_css_set.subsys[perf_event_cgrp_id];
}
#endif

static inline bool __valid_pkg_id(u16 pkg_id)
{
	return pkg_id < topology_max_packages();
}

static int anode_pool__alloc_one(u16 pkg_id);

/* Init cqm pkg_data for @cpu 's package. */
static int pkg_data_init_cpu(int cpu)
{
	int i, nr_anodes;
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
	INIT_LIST_HEAD(&pkg_data->pmonr_limbo_prmids_pool);
	INIT_LIST_HEAD(&pkg_data->nopmonr_limbo_prmids_pool);

	INIT_LIST_HEAD(&pkg_data->astate_pmonrs_lru);
	INIT_LIST_HEAD(&pkg_data->istate_pmonrs_lru);
	INIT_LIST_HEAD(&pkg_data->ilstate_pmonrs_lru);

	pkg_data->nr_instate_pmonrs = 0;
	pkg_data->nr_ilstate_pmonrs = 0;

	mutex_init(&pkg_data->pkg_data_mutex);
	raw_spin_lock_init(&pkg_data->pkg_data_lock);

	INIT_LIST_HEAD(&pkg_data->anode_pool_head);
	raw_spin_lock_init(&pkg_data->anode_pool_lock);

	INIT_DELAYED_WORK(
		&pkg_data->rotation_work, intel_cqm_rmid_rotation_work);
	/* XXX: Chose randomly*/
	pkg_data->rotation_cpu = cpu;

	INIT_DELAYED_WORK(
		&pkg_data->timed_update_work, intel_cqm_timed_update_work);
	/* XXX: Chose randomly*/
	pkg_data->timed_update_cpu = cpu;

	cqm_pkgs_data[pkg_id] = pkg_data;

	/* Pre-allocate pool with one anode more than minimum needed to contain
	 * all the RMIDs in the package.
	 */
	nr_anodes = (pkg_data->max_rmid + NR_RMIDS_PER_NODE) /
		NR_RMIDS_PER_NODE + 1;

	for (i = 0; i < nr_anodes; i++)
		anode_pool__alloc_one(pkg_id);
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

	pmonr->ancestor_pmonr = NULL;

	/*
	 * Since (A)state and (I)state have union in members,
	 * initialize one of them only.
	 */
	INIT_LIST_HEAD(&pmonr->pmonr_deps_head);
	pmonr->prmid = NULL;
	INIT_LIST_HEAD(&pmonr->limbo_rotation_entry);

	pmonr->monr = NULL;
	INIT_LIST_HEAD(&pmonr->rotation_entry);

	pmonr->last_enter_istate = 0;
	pmonr->last_enter_astate = 0;
	pmonr->nr_enter_istate = 0;

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

	pmonr->last_enter_astate = jiffies;

	list_move_tail(
		&prmid->pool_entry, &__pkg_data(pmonr, active_prmids_pool));
	list_move_tail(
		&pmonr->rotation_entry, &__pkg_data(pmonr, astate_pmonrs_lru));

	summary.sched_rmid = pmonr->prmid->rmid;
	summary.read_rmid = pmonr->prmid->rmid;
	atomic64_set(&pmonr->prmid_summary_atomic, summary.value);
}

/*
 * Transition to (A)state from (IN)state, given a valid prmid.
 * Cannot fail. Updates ancestor dependants to use this pmonr as new ancestor.
 */
static inline void
__pmonr__instate_to_astate(struct pmonr *pmonr, struct prmid *prmid)
{
	struct pmonr *pos, *tmp, *ancestor;
	union prmid_summary old_summary, summary;

	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));

	/* If in (I) state, cannot have limbo_prmid, otherwise prmid
	 * in function's argument is superfluous.
	 */
	WARN_ON_ONCE(pmonr->limbo_prmid);

	__pkg_data(pmonr, nr_instate_pmonrs)--;

	/* Do not depend on ancestor_pmonr anymore. Make it (A)state. */
	ancestor = pmonr->ancestor_pmonr;
	list_del_init(&pmonr->pmonr_deps_entry);
	pmonr->ancestor_pmonr = NULL;
	__pmonr__finish_to_astate(pmonr, prmid);

	/* Update ex ancestor's dependants that are pmonr descendants. */
	list_for_each_entry_safe(pos, tmp, &ancestor->pmonr_deps_head,
				 pmonr_deps_entry) {
		if (!__monr_hrchy_is_ancestor(monr_hrchy_root,
					      pmonr->monr, pos->monr))
			continue;
		list_move_tail(&pos->pmonr_deps_entry, &pmonr->pmonr_deps_head);
		pos->ancestor_pmonr = pmonr;
		old_summary.value = atomic64_read(&pos->prmid_summary_atomic);
		summary.sched_rmid = prmid->rmid;
		summary.read_rmid = old_summary.read_rmid;
		atomic64_set(&pos->prmid_summary_atomic, summary.value);
	}
}

/*
 * Transition from (IL)state  to (A)state.
 */
static inline void
__pmonr__ilstate_to_astate(struct pmonr *pmonr)
{
	struct prmid *prmid;

	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	WARN_ON_ONCE(!pmonr->limbo_prmid);

	prmid = pmonr->limbo_prmid;
	pmonr->limbo_prmid = NULL;
	list_del_init(&pmonr->limbo_rotation_entry);

	__pkg_data(pmonr, nr_ilstate_pmonrs)--;
	__pkg_data(pmonr, nr_instate_pmonrs)++;
	list_del_init(&prmid->pool_entry);

	__pmonr__instate_to_astate(pmonr, prmid);
}

static inline void
__pmonr__ustate_to_astate(struct pmonr *pmonr, struct prmid *prmid)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));
	__pmonr__finish_to_astate(pmonr, prmid);
}

/*
 * Find lowest active ancestor.
 * Always successful since monr_hrchy_root is always in (A)state.
 */
static struct monr *
__monr_hrchy__find_laa(struct monr *monr, u16 pkg_id)
{
	lockdep_assert_held(&cqm_pkgs_data[pkg_id]->pkg_data_lock);

	while ((monr = monr->parent)) {
		if (__pmonr__in_astate(monr->pmonrs[pkg_id]))
			return monr;
	}
	/* Should have hitted monr_hrchy_root */
	WARN_ON_ONCE(true);
	return NULL;
}

/*
 * __pmnor__move_dependants: Move dependants from one ancestor to another.
 * @old: Old ancestor.
 * @new: New ancestor.
 *
 * To be called on valid pmonrs. @new must be ancestor of @old.
 */
static inline void
__pmonr__move_dependants(struct pmonr *old, struct pmonr *new)
{
	struct pmonr *dep;
	union prmid_summary old_summary, summary;

	WARN_ON_ONCE(old->pkg_id != new->pkg_id);
	lockdep_assert_held(&__pkg_data(old, pkg_data_lock));

	/* Update this pmonr dependencies to use new ancestor. */
	list_for_each_entry(dep, &old->pmonr_deps_head, pmonr_deps_entry) {
		/* Set next summary for dependent pmonrs. */
		dep->ancestor_pmonr = new;

		old_summary.value = atomic64_read(&dep->prmid_summary_atomic);
		summary.sched_rmid = new->prmid->rmid;
		summary.read_rmid = old_summary.read_rmid;
		atomic64_set(&dep->prmid_summary_atomic, summary.value);
	}
	list_splice_tail_init(&old->pmonr_deps_head,
			      &new->pmonr_deps_head);
}

static inline void
__pmonr__to_ustate(struct pmonr *pmonr)
{
	struct pmonr *ancestor;
	u16 pkg_id = pmonr->pkg_id;
	union prmid_summary summary;

	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));

	/* Do not warn on re-enter state for (U)state, to simplify cleanup
	 * of initialized states that were not scheduled.
	 */
	if (__pmonr__in_ustate(pmonr))
		return;

	if (__pmonr__in_astate(pmonr)) {
		WARN_ON_ONCE(!pmonr->prmid);

		ancestor = __monr_hrchy__find_laa(
			pmonr->monr, pkg_id)->pmonrs[pkg_id];
		WARN_ON_ONCE(!ancestor);
		__pmonr__move_dependants(pmonr, ancestor);
		list_move_tail(&pmonr->prmid->pool_entry,
			       &__pkg_data(pmonr, nopmonr_limbo_prmids_pool));
		pmonr->prmid =  NULL;
	} else if (__pmonr__in_istate(pmonr)) {
		list_del_init(&pmonr->pmonr_deps_entry);
		/* limbo_prmid is already in limbo pool */
		if (__pmonr__in_ilstate(pmonr)) {
			WARN_ON(!pmonr->limbo_prmid);
			list_move_tail(
				&pmonr->limbo_prmid->pool_entry,
				&__pkg_data(pmonr, nopmonr_limbo_prmids_pool));

			pmonr->limbo_prmid = NULL;
			list_del_init(&pmonr->limbo_rotation_entry);
			__pkg_data(pmonr, nr_ilstate_pmonrs)--;
		} else {
			__pkg_data(pmonr, nr_instate_pmonrs)--;
		}
		pmonr->ancestor_pmonr = NULL;
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

static inline void __pmonr__set_istate_summary(struct pmonr *pmonr)
{
	union prmid_summary summary;

	summary.sched_rmid = pmonr->ancestor_pmonr->prmid->rmid;
	summary.read_rmid =
		pmonr->limbo_prmid ? pmonr->limbo_prmid->rmid : INVALID_RMID;
	atomic64_set(
		&pmonr->prmid_summary_atomic, summary.value);
}

/*
 * Transition to (I)state from no (I)state..
 * Finds a valid ancestor transversing monr_hrchy. Cannot fail.
 */
static inline void
__pmonr__to_istate(struct pmonr *pmonr)
{
	struct pmonr *ancestor;
	u16 pkg_id = pmonr->pkg_id;

	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));

	if (!(__pmonr__in_ustate(pmonr) || __pmonr__in_astate(pmonr))) {
		/* Invalid initial state. */
		WARN_ON_ONCE(true);
		return;
	}

	ancestor = __monr_hrchy__find_laa(pmonr->monr, pkg_id)->pmonrs[pkg_id];
	WARN_ON_ONCE(!ancestor);

	if (__pmonr__in_astate(pmonr)) {
		/* Active pmonr->prmid becomes limbo in transition to (I)state.
		 * Note that pmonr->prmid and pmonr->limbo_prmid are in an
		 * union, so no need to copy.
		 */
		__pmonr__move_dependants(pmonr, ancestor);
		list_move_tail(&pmonr->limbo_prmid->pool_entry,
			       &__pkg_data(pmonr, pmonr_limbo_prmids_pool));
		__pkg_data(pmonr, nr_ilstate_pmonrs)++;
	} else {
		__pkg_data(pmonr, nr_instate_pmonrs)++;
	}

	pmonr->ancestor_pmonr = ancestor;
	list_add_tail(&pmonr->pmonr_deps_entry, &ancestor->pmonr_deps_head);

	list_move_tail(
		&pmonr->rotation_entry, &__pkg_data(pmonr, istate_pmonrs_lru));

	if (pmonr->limbo_prmid)
		list_move_tail(&pmonr->limbo_rotation_entry,
			       &__pkg_data(pmonr, ilstate_pmonrs_lru));

	pmonr->last_enter_istate = jiffies;
	pmonr->nr_enter_istate++;

	__pmonr__set_istate_summary(pmonr);

}

static inline void
__pmonr__ilstate_to_instate(struct pmonr *pmonr)
{
	lockdep_assert_held(&__pkg_data(pmonr, pkg_data_lock));

	list_move_tail(&pmonr->limbo_prmid->pool_entry,
		       &__pkg_data(pmonr, free_prmids_pool));
	pmonr->limbo_prmid = NULL;

	__pkg_data(pmonr, nr_ilstate_pmonrs)--;
	__pkg_data(pmonr, nr_instate_pmonrs)++;

	list_del_init(&pmonr->limbo_rotation_entry);
	__pmonr__set_istate_summary(pmonr);
}

/* Count all limbo prmids, including the ones still attached to pmonrs.
 * Maximum number of prmids is fixed by hw and generally small.
 */
static int count_limbo_prmids(struct pkg_data *pkg_data)
{
	unsigned int c = 0;
	struct prmid *prmid;

	lockdep_assert_held(&pkg_data->pkg_data_mutex);

	list_for_each_entry(
		prmid, &pkg_data->pmonr_limbo_prmids_pool, pool_entry) {
		c++;
	}
	list_for_each_entry(
		prmid, &pkg_data->nopmonr_limbo_prmids_pool, pool_entry) {
		c++;
	}

	return c;
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
	monr->mon_cgrp = NULL;
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
 * Logic for reading sets of rmids into per-package lists.
 * This package lists can be used to update occupancies without
 * holding locks in the hierarchies of pmonrs.
 * @pool: free pool.
 */
struct astack {
	struct list_head	pool;
	struct list_head	items;
	int			top_idx;
	int			max_idx;
	u16			pkg_id;
};

static void astack__init(struct astack *astack, int max_idx, u16 pkg_id)
{
	INIT_LIST_HEAD(&astack->items);
	INIT_LIST_HEAD(&astack->pool);
	astack->top_idx = -1;
	astack->max_idx = max_idx;
	astack->pkg_id = pkg_id;
}

/* Try to enlarge astack->pool with a anode from this pkgs pool. */
static int astack__try_add_pool(struct astack *astack)
{
	unsigned long flags;
	int ret = -1;
	struct pkg_data *pkg_data = cqm_pkgs_data[astack->pkg_id];

	raw_spin_lock_irqsave(&pkg_data->anode_pool_lock, flags);

	if (!list_empty(&pkg_data->anode_pool_head)) {
		list_move_tail(pkg_data->anode_pool_head.prev, &astack->pool);
		ret = 0;
	}

	raw_spin_unlock_irqrestore(&pkg_data->anode_pool_lock, flags);
	return ret;
}

static int astack__push(struct astack *astack)
{
	if (!list_empty(&astack->items) && astack->top_idx < astack->max_idx) {
		astack->top_idx++;
		return 0;
	}

	if (list_empty(&astack->pool) && astack__try_add_pool(astack))
		return -1;
	list_move_tail(astack->pool.prev, &astack->items);
	astack->top_idx = 0;
	return 0;
}

/* Must be non-empty */
# define __astack__top(astack_, member_) \
	list_last_entry(&(astack_)->items, \
	struct anode, entry)->member_[(astack_)->top_idx]

static void astack__clear(struct astack *astack)
{
	list_splice_tail_init(&astack->items, &astack->pool);
	astack->top_idx = -1;
}

/* Put back into pkg_data's pool. */
static void astack__release(struct astack *astack)
{
	unsigned long flags;
	struct pkg_data *pkg_data = cqm_pkgs_data[astack->pkg_id];

	astack__clear(astack);
	raw_spin_lock_irqsave(&pkg_data->anode_pool_lock, flags);
	list_splice_tail_init(&astack->pool, &pkg_data->anode_pool_head);
	raw_spin_unlock_irqrestore(&pkg_data->anode_pool_lock, flags);
}

static int anode_pool__alloc_one(u16 pkg_id)
{
	unsigned long flags;
	struct anode *anode;
	struct pkg_data *pkg_data = cqm_pkgs_data[pkg_id];

	anode = kmalloc_node(sizeof(struct anode), GFP_KERNEL,
			     cpu_to_node(pkg_data->rotation_cpu));
	if (!anode)
		return -ENOMEM;
	raw_spin_lock_irqsave(&pkg_data->anode_pool_lock, flags);
	list_add_tail(&anode->entry, &pkg_data->anode_pool_head);
	raw_spin_unlock_irqrestore(&pkg_data->anode_pool_lock, flags);
	return 0;
}

static int astack__end(struct astack *astack, struct anode *anode, int idx)
{
	return list_is_last(&anode->entry, &astack->items) &&
	       idx > astack->top_idx;
}

static int __rmid_fn__cqm_prmid_update(struct prmid *prmid, u64 *val)
{
	int ret = cqm_prmid_update(prmid);

	if (ret >= 0)
		*val = atomic64_read(&prmid->last_read_value);
	return ret;
}

/* Apply function to all elements in all nodes.
 * On error returns first error in read, zero otherwise.
 */
static int astack__rmids_sum_apply(
	struct astack *astack,
	u16 pkg_id, int (*fn)(struct prmid *, u64 *), u64 *total)
{
	struct prmid *prmid;
	struct anode *anode;
	u32 rmid;
	int i, ret, first_error = 0;
	u64 count;
	*total = 0;

	list_for_each_entry(anode, &astack->items, entry) {
		for (i = 0; i <= astack->max_idx; i++) {
			/* node in tail only has astack->top_idx elements. */
			if (astack__end(astack, anode, i))
				break;
			rmid = anode->rmids[i];
			prmid = cqm_pkgs_data[pkg_id]->prmids_by_rmid[rmid];
			WARN_ON_ONCE(!prmid);
			ret = fn(prmid, &count);
			if (ret < 0) {
				if (!first_error)
					first_error = ret;
				continue;
			}
			*total += count;
		}
	}
	return first_error;
}

/* Does not need mutex since protected by locks when transversing
 * astate_pmonrs_lru and updating atomic prmids.
 */
static int update_rmids_in_astate_pmonrs_lru(u16 pkg_id)
{
	struct astack astack;
	struct pkg_data *pkg_data;
	struct pmonr *pmonr;
	int ret = 0;
	unsigned long flags;
	u64 count;

	astack__init(&astack, NR_RMIDS_PER_NODE - 1, pkg_id);
	pkg_data = cqm_pkgs_data[pkg_id];

retry:
	if (ret) {
		anode_pool__alloc_one(pkg_id);
		ret = 0;
	}
	raw_spin_lock_irqsave_nested(&pkg_data->pkg_data_lock, flags, pkg_id);
	list_for_each_entry(pmonr,
			    &pkg_data->astate_pmonrs_lru, rotation_entry) {
		ret = astack__push(&astack);
		if (ret)
			break;
		__astack__top(&astack, rmids) = pmonr->prmid->rmid;
	}
	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	if (ret) {
		astack__clear(&astack);
		goto retry;
	}
	/* count is not used. */
	ret = astack__rmids_sum_apply(&astack, pkg_id,
				      &__rmid_fn__cqm_prmid_update, &count);
	astack__release(&astack);
	return ret;
}

/*
 * Wrappers for monr manipulation in events and cgroups.
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

#ifdef CONFIG_CGROUP_PERF
static inline struct monr *monr_from_perf_cgroup(struct perf_cgroup *cgrp)
{
	struct monr *monr;
	struct cgrp_cqm_info *cqm_info;

	cqm_info = (struct cgrp_cqm_info *)READ_ONCE(cgrp->arch_info);
	WARN_ON_ONCE(!cqm_info);
	monr = READ_ONCE(cqm_info->monr);
	return monr;
}

static inline struct perf_cgroup *monr__get_mon_cgrp(struct monr *monr)
{
	WARN_ON_ONCE(!monr);
	return READ_ONCE(monr->mon_cgrp);
}

static inline void
monr__set_mon_cgrp(struct monr *monr, struct perf_cgroup *cgrp)
{
	WRITE_ONCE(monr->mon_cgrp, cgrp);
}

static inline void
perf_cgroup_set_monr(struct perf_cgroup *cgrp, struct monr *monr)
{
	WRITE_ONCE(cgrp_to_cqm_info(cgrp)->monr, monr);
}

/*
 * A perf_cgroup is monitored when it's set in a monr->mon_cgrp.
 * There is a many-to-one relationship between perf_cgroup's monrs
 * and monrs' mon_cgrp. A monitored cgroup is necesarily referenced
 * back by its monr's mon_cgrp.
 */
static inline bool perf_cgroup_is_monitored(struct perf_cgroup *cgrp)
{
	struct monr *monr;
	struct perf_cgroup *monr_cgrp;

	/* monr can be referenced by a cgroup other than the one in its
	 * mon_cgrp, be careful.
	 */
	monr = monr_from_perf_cgroup(cgrp);

	monr_cgrp = monr__get_mon_cgrp(monr);
	/* Root monr do not have a cgroup associated before initialization.
	 * mon_cgrp and mon_event_group are union, so the pointer must be set
	 * for all non-root monrs.
	 */
	return  monr_cgrp && monr__get_mon_cgrp(monr) == cgrp;
}

/* Set css's monr to the monr of its lowest monitored ancestor. */
static inline void __css_set_monr_to_lma(struct cgroup_subsys_state *css)
{
	lockdep_assert_held(&cqm_mutex);
	if (!css->parent) {
		perf_cgroup_set_monr(css_to_perf_cgroup(css), monr_hrchy_root);
		return;
	}
	perf_cgroup_set_monr(
		css_to_perf_cgroup(css),
		monr_from_perf_cgroup(css_to_perf_cgroup(css->parent)));
}

static inline void
perf_cgroup_make_monitored(struct perf_cgroup *cgrp, struct monr *monr)
{
	monr_hrchy_assert_held_mutexes();
	perf_cgroup_set_monr(cgrp, monr);
	/* Make sure that monr is a valid monr for css before it's visible
	 * to any reader of css.
	 */
	smp_wmb();
	monr__set_mon_cgrp(monr, cgrp);
}

static inline void
perf_cgroup_make_unmonitored(struct perf_cgroup *cgrp)
{
	struct monr *monr = monr_from_perf_cgroup(cgrp);

	monr_hrchy_assert_held_mutexes();
	__css_set_monr_to_lma(&cgrp->css);
	/* Make sure that all readers of css'monr see lma css before
	 * monr stops being a valid monr for css.
	 */
	smp_wmb();
	monr__set_mon_cgrp(monr, NULL);
}
#endif

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
		 * monr. Use an inherited one.
		 */
		__pmonr__to_istate(pmonr);
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

#ifdef CONFIG_CGROUP_PERF
static struct perf_cgroup *__perf_cgroup_parent(struct perf_cgroup *cgrp)
{
	struct cgroup_subsys_state *parent_css = cgrp->css.parent;

	if (parent_css)
		return css_to_perf_cgroup(parent_css);
	return NULL;
}

/* Get cgroup for both task and cgroup event. */
static inline struct perf_cgroup *
perf_cgroup_from_event(struct perf_event *event)
{
#ifdef CONFIG_LOCKDEP
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
	bool rcu_safe = lockdep_is_held(
		&cqm_pkgs_data[pkg_id]->pkg_data_lock);
#endif

	if (!(event->attach_state & PERF_ATTACH_TASK))
		return event->cgrp;

	return container_of(
		task_css_check(event->hw.target, perf_event_cgrp_id, rcu_safe),
		struct perf_cgroup, css);
}

/* Find lowest ancestor that is monitored, not including this cgrp.
 * Return NULL if no ancestor is monitored.
 */
struct perf_cgroup *__cgroup_find_lma(struct perf_cgroup *cgrp)
{
	do {
		cgrp = __perf_cgroup_parent(cgrp);
	} while (cgrp && !perf_cgroup_is_monitored(cgrp));
	return cgrp;
}

/* Similar to css_next_descendant_pre but skips the subtree rooted by pos. */
struct cgroup_subsys_state *
css_skip_subtree_pre(struct cgroup_subsys_state *pos,
		     struct cgroup_subsys_state *root)
{
	struct cgroup_subsys_state *next;

	WARN_ON_ONCE(!pos);
	while (pos != root) {
		next = css_next_child(pos, pos->parent);
		if (next)
			return next;
		pos = pos->parent;
	}
	return NULL;
}

/* Make all monrs of css descendants of css to depend on new_monr. */
inline void __css_subtree_update_monrs(struct cgroup_subsys_state *css,
				       struct monr *new_monr)
{
	struct cgroup_subsys_state *pos_css;
	int i;
	unsigned long flags;

	lockdep_assert_held(&cqm_mutex);
	monr_hrchy_assert_held_mutexes();

	rcu_read_lock();

	/* Iterate over descendants of css in pre-order, in a way
	 * similar to css_for_each_descendant_pre, but skipping the subtrees
	 * rooted by css's with a monitored cgroup, since the elements
	 * in those subtrees do not need to be updated.
	 */
	pos_css = css_next_descendant_pre(css, css);
	while (pos_css) {
		struct perf_cgroup *pos_cgrp = css_to_perf_cgroup(pos_css);
		struct monr *pos_monr = monr_from_perf_cgroup(pos_cgrp);

		/* Skip css that are not online, sync'ed with cqm_mutex. */
		if (!(pos_css->flags & CSS_ONLINE)) {
			pos_css = css_next_descendant_pre(pos_css, css);
			continue;
		}
		/* Update descendant pos's mnor pointers to monr_parent. */
		if (!perf_cgroup_is_monitored(pos_cgrp)) {
			perf_cgroup_set_monr(pos_cgrp, new_monr);
			pos_css = css_next_descendant_pre(pos_css, css);
			continue;
		}
		monr_hrchy_acquire_raw_spin_locks_irq_save(flags, i);
		pos_monr->parent = new_monr;
		list_move_tail(&pos_monr->parent_entry, &new_monr->children);
		monr_hrchy_release_raw_spin_locks_irq_restore(flags, i);
		/* Dont go down the subtree in pos_css since pos_monr is the
		 * lma for all its descendants.
		 */
		pos_css = css_skip_subtree_pre(pos_css, css);
	}
	rcu_read_unlock();
}

static inline int __css_start_monitoring(struct cgroup_subsys_state *css)
{
	struct perf_cgroup *cgrp, *cgrp_lma, *pos_cgrp;
	struct monr *monr, *monr_parent, *pos_monr, *tmp_monr;
	unsigned long flags;
	int i;

	lockdep_assert_held(&cqm_mutex);

	/* Hold mutexes to prevent all rotation threads in all packages from
	 * messing with this.
	 */
	monr_hrchy_acquire_mutexes();
	cgrp = css_to_perf_cgroup(css);
	if (WARN_ON_ONCE(perf_cgroup_is_monitored(cgrp)))
		return -1;

	/* When css is root cgroup's css, attach to the pre-existing
	 * and active root monr.
	 */
	cgrp_lma = __cgroup_find_lma(cgrp);
	if (!cgrp_lma) {
		/* monr of root cgrp must be monr_hrchy_root. */
		WARN_ON_ONCE(!monr__is_root(monr_from_perf_cgroup(cgrp)));
		perf_cgroup_make_monitored(cgrp, monr_hrchy_root);
		monr_hrchy_release_mutexes();
		return 0;
	}
	/* The monr for the lowest monitored ancestor is direct ancestor
	 * of monr in the monr hierarchy.
	 */
	monr_parent = monr_from_perf_cgroup(cgrp_lma);

	/* Create new monr. */
	monr = monr_alloc();
	if (IS_ERR(monr)) {
		monr_hrchy_release_mutexes();
		return PTR_ERR(monr);
	}

	/* monr has no children yet so it is to be inserted in hierarchy with
	 * all its pmors in (U)state.
	 * We hold locks until monr_hrchy changes are complete, to prevent
	 * possible state transition for the pmonrs in monr while still
	 * allowing to read the prmid_summary in the scheduler path.
	 */
	monr_hrchy_acquire_raw_spin_locks_irq_save(flags, i);
	__monr_hrchy_insert_leaf(monr, monr_parent);
	monr_hrchy_release_raw_spin_locks_irq_restore(flags, i);

	/* Make sure monr is in hierarchy before attaching monr to cgroup. */
	barrier();

	perf_cgroup_make_monitored(cgrp, monr);
	__css_subtree_update_monrs(css, monr);

	monr_hrchy_acquire_raw_spin_locks_irq_save(flags, i);
	/* Move task-event monrs that are descendant from css's cgroup. */
	list_for_each_entry_safe(pos_monr, tmp_monr,
				 &monr_parent->children, parent_entry) {
		if (!monr_is_event_type(pos_monr))
			continue;
		/* all events in event group must have the same cgroup.
		 * No RCU read lock necessary for task_css_check since calling
		 * inside critical section.
		 */
		pos_cgrp = perf_cgroup_from_event(pos_monr->mon_event_group);
		if (!cgroup_is_descendant(pos_cgrp->css.cgroup,
					  cgrp->css.cgroup))
			continue;
		pos_monr->parent = monr;
		list_move_tail(&pos_monr->parent_entry, &monr->children);
	}
	/* Make sure monitoring starts after all monrs have moved. */
	barrier();

	__monr__set_mon_active(monr);
	monr_hrchy_release_raw_spin_locks_irq_restore(flags, i);

	monr_hrchy_release_mutexes();
	return 0;
}

static inline int __css_stop_monitoring(struct cgroup_subsys_state *css)
{
	struct perf_cgroup *cgrp, *cgrp_lma;
	struct monr *monr, *monr_parent, *pos_monr;
	unsigned long flags;
	int i;

	lockdep_assert_held(&cqm_mutex);

	monr_hrchy_acquire_mutexes();
	cgrp = css_to_perf_cgroup(css);
	if (WARN_ON_ONCE(!perf_cgroup_is_monitored(cgrp)))
		return -1;

	monr = monr_from_perf_cgroup(cgrp);

	/* When css is root cgroup's css, detach cgroup but do not
	 * destroy monr.
	 */
	cgrp_lma = __cgroup_find_lma(cgrp);
	if (!cgrp_lma) {
		/* monr of root cgrp must be monr_hrchy_root. */
		WARN_ON_ONCE(!monr__is_root(monr_from_perf_cgroup(cgrp)));
		perf_cgroup_make_unmonitored(cgrp);
		monr_hrchy_release_mutexes();
		return 0;
	}
	/* The monr for the lowest monitored ancestor is direct ancestor
	 * of monr in the monr hierarchy.
	 */
	monr_parent = monr_from_perf_cgroup(cgrp_lma);

	/* Lock together the transition to (U)state and clearing
	 * MONR_MON_ACTIVE to prevent prmids to return to (A)state
	 * or (I)state in between.
	 */
	monr_hrchy_acquire_raw_spin_locks_irq_save(flags, i);
	cqm_pkg_id_for_each_online(i)
		__pmonr__to_ustate(monr->pmonrs[i]);
	barrier();
	__monr__clear_mon_active(monr);
	monr_hrchy_release_raw_spin_locks_irq_restore(flags, i);

	__css_subtree_update_monrs(css, monr_parent);


	/*
	 * Move the children monrs that are no cgroups.
	 */
	monr_hrchy_acquire_raw_spin_locks_irq_save(flags, i);

	list_for_each_entry(pos_monr, &monr->children, parent_entry)
		pos_monr->parent = monr_parent;
	list_splice_tail_init(&monr->children, &monr_parent->children);
	perf_cgroup_make_unmonitored(cgrp);
	__monr_hrchy_remove_leaf(monr);

	monr_hrchy_release_raw_spin_locks_irq_restore(flags, i);

	monr_hrchy_release_mutexes();
	monr_dealloc(monr);
	return 0;
}

/* Attaching an event to a cgroup starts monitoring in the cgroup.
 * If the cgroup is already monitoring, just use its pre-existing mnor.
 */
static int __monr_hrchy_attach_cgroup_event(struct perf_event *event,
					    struct perf_cgroup *perf_cgrp)
{
	struct monr *monr;
	int ret;

	lockdep_assert_held(&cqm_mutex);
	WARN_ON_ONCE(event->attach_state & PERF_ATTACH_TASK);
	WARN_ON_ONCE(monr_from_event(event));
	WARN_ON_ONCE(!perf_cgrp);

	if (!perf_cgroup_is_monitored(perf_cgrp)) {
		css_get(&perf_cgrp->css);
		ret = __css_start_monitoring(&perf_cgrp->css);
		css_put(&perf_cgrp->css);
		if (ret)
			return ret;
	}

	/* At this point, cgrp is always monitored, use its monr. */
	monr = monr_from_perf_cgroup(perf_cgrp);

	event_set_monr(event, monr);
	monr->mon_event_group = event;
	return 0;
}
#endif

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
	bool has_cgrp = false;
#ifdef CONFIG_CGROUP_PERF
	struct perf_cgroup *perf_cgrp;

	has_cgrp = event->cgrp;
#endif

	if (!has_cgrp && !(event->attach_state & PERF_ATTACH_TASK))
		return __monr_hrchy_attach_cpu_event(event);

#ifdef CONFIG_CGROUP_PERF
	/* Task events become leaves, cgroup events reuse the cgroup's monr */
	if (event->cgrp)
		return __monr_hrchy_attach_cgroup_event(event, event->cgrp);

	rcu_read_lock();
	perf_cgrp = perf_cgroup_from_event(event);
	rcu_read_unlock();

	monr_parent = monr_from_perf_cgroup(perf_cgrp);
#else
	/* Two-levels hierarchy: Root and all event monr underneath it. */
	monr_parent = monr_hrchy_root;
#endif
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
	/* Cgroup/non-task per-cpu and task events don't mix */
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

/*
 * Try to reuse limbo prmid's for pmonrs at the front of  ilstate_pmonrs_lru.
 */
static int __try_reuse_ilstate_pmonrs(struct pkg_data *pkg_data)
{
	int reused = 0;
	struct pmonr *pmonr;

	lockdep_assert_held(&pkg_data->pkg_data_mutex);
	lockdep_assert_held(&pkg_data->pkg_data_lock);

	while ((pmonr = list_first_entry_or_null(
		&pkg_data->istate_pmonrs_lru, struct pmonr, rotation_entry))) {

		if (__pmonr__in_instate(pmonr))
			break;
		__pmonr__ilstate_to_astate(pmonr);
		reused++;
	}
	return reused;
}

static int try_reuse_ilstate_pmonrs(struct pkg_data *pkg_data)
{
	int reused;
	unsigned long flags;
#ifdef CONFIG_LOCKDEP
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
#endif

	lockdep_assert_held(&pkg_data->pkg_data_mutex);

	raw_spin_lock_irqsave_nested(&pkg_data->pkg_data_lock, flags, pkg_id);
	reused = __try_reuse_ilstate_pmonrs(pkg_data);
	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	return reused;
}


/*
 * A monr is only readable when all it's used pmonrs have a RMID.
 * Therefore, the time a monr entered (A)state is the maximum of the
 * last_enter_astate times for all (A)state pmonrs if no pmonr is in (I)state.
 * A monr with any pmonr in (I)state has no entered (A)state.
 * Returns monr_enter_astate time if available, otherwise min_inh_pkg is
 * set to the smallest pkg_id where the monr's pmnor is in (I)state and
 * the return value is undefined.
 */
static unsigned long
__monr__last_enter_astate(struct monr *monr, int *min_inh_pkg)
{
	struct pkg_data *pkg_data;
	u16 pkg_id;
	unsigned long flags, astate_time = 0;

	*min_inh_pkg = -1;
	cqm_pkg_id_for_each_online(pkg_id) {
		struct pmonr *pmonr;

		if (min_inh_pkg >= 0)
			break;

		raw_spin_lock_irqsave_nested(
				&pkg_data->pkg_data_lock, flags, pkg_id);

		pmonr = monr->pmonrs[pkg_id];
		if (__pmonr__in_istate(pmonr) && min_inh_pkg < 0)
			*min_inh_pkg = pkg_id;
		else if (__pmonr__in_astate(pmonr) &&
				astate_time < pmonr->last_enter_astate)
			astate_time = pmonr->last_enter_astate;

		raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	}
	return astate_time;
}

/*
 * Steal as many rmids as possible.
 * Transition pmonrs that have stayed at least __cqm_min_mon_slice in
 * (A)state to (I)state.
 */
static inline int
__try_steal_active_pmonrs(
	struct pkg_data *pkg_data, unsigned int max_to_steal)
{
	struct pmonr *pmonr, *tmp;
	int nr_stolen = 0, min_inh_pkg;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
	unsigned long flags, monr_astate_end_time, now = jiffies;
	struct list_head *alist = &pkg_data->astate_pmonrs_lru;

	lockdep_assert_held(&pkg_data->pkg_data_mutex);

	/* pmonrs don't leave astate outside of rotation logic.
	 * The pkg mutex protects against the pmonr leaving
	 * astate_pmonrs_lru. The raw_spin_lock protects these list
	 * operations from list insertions at tail coming from the
	 * sched logic ( (U)state -> (A)state )
	 */
	raw_spin_lock_irqsave_nested(&pkg_data->pkg_data_lock, flags, pkg_id);

	pmonr = list_first_entry(alist, struct pmonr, rotation_entry);
	WARN_ON_ONCE(pmonr != monr_hrchy_root->pmonrs[pkg_id]);
	WARN_ON_ONCE(pmonr->pkg_id != pkg_id);

	list_for_each_entry_safe_continue(pmonr, tmp, alist, rotation_entry) {
		bool steal_rmid = false;

		WARN_ON_ONCE(!__pmonr__in_astate(pmonr));
		WARN_ON_ONCE(pmonr->pkg_id != pkg_id);

		raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);

		monr_astate_end_time =
			__monr__last_enter_astate(pmonr->monr, &min_inh_pkg) +
			__cqm_min_mon_slice;

		/* pmonr in this pkg is supposed to be in (A)state. */
		WARN_ON_ONCE(min_inh_pkg == pkg_id);

		/* Steal a pmonr if:
		 *   1) Any pmonr in a pkg with pkg_id < local pkg_id is
		 *	in (I)state.
		 *   2) It's monr has been active for enough time.
		 * Note that since the min_inh_pkg for a monr cannot decrease
		 * while the monr is not active, then the monr eventually will
		 * become active again despite the stealing of pmonrs in pkgs
		 * with id larger than min_inh_pkg.
		 */
		if (min_inh_pkg >= 0 && min_inh_pkg < pkg_id)
			steal_rmid = true;
		if (min_inh_pkg < 0 && monr_astate_end_time <= now)
			steal_rmid = true;

		raw_spin_lock_irqsave_nested(
			&pkg_data->pkg_data_lock, flags, pkg_id);
		if (!steal_rmid)
			continue;

		__pmonr__to_istate(pmonr);
		nr_stolen++;
		if (nr_stolen == max_to_steal)
			break;
	}

	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);

	return nr_stolen;
}

/* It will remove the prmid from the list its attached, if used. */
static inline int __try_use_free_prmid(struct pkg_data *pkg_data,
				       struct prmid *prmid, bool *succeed)
{
	struct pmonr *pmonr;
	int nr_activated = 0;

	lockdep_assert_held(&pkg_data->pkg_data_mutex);
	lockdep_assert_held(&pkg_data->pkg_data_lock);

	*succeed = false;
	nr_activated += __try_reuse_ilstate_pmonrs(pkg_data);
	pmonr = list_first_entry_or_null(&pkg_data->istate_pmonrs_lru,
					 struct pmonr, rotation_entry);
	if (!pmonr)
		return nr_activated;
	WARN_ON_ONCE(__pmonr__in_ilstate(pmonr));
	WARN_ON_ONCE(!__pmonr__in_instate(pmonr));

	/* the state transition function will move the prmid to
	 * the active lru list.
	 */
	__pmonr__instate_to_astate(pmonr, prmid);
	nr_activated++;
	*succeed = true;
	return nr_activated;
}

static inline int __try_use_free_prmids(struct pkg_data *pkg_data)
{
	struct prmid *prmid, *tmp_prmid;
	unsigned long flags;
	int nr_activated = 0;
	bool succeed;
#ifdef CONFIG_DEBUG_SPINLOCK
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
#endif

	lockdep_assert_held(&pkg_data->pkg_data_mutex);
	/* Lock protects free_prmids_pool, istate_pmonrs_lru and
	 * the monr hrchy.
	 */
	raw_spin_lock_irqsave_nested(&pkg_data->pkg_data_lock, flags, pkg_id);

	list_for_each_entry_safe(prmid, tmp_prmid,
				 &pkg_data->free_prmids_pool, pool_entry) {

		/* Removes the free prmid if used. */
		nr_activated += __try_use_free_prmid(pkg_data,
						     prmid, &succeed);
	}

	nr_activated += __try_reuse_ilstate_pmonrs(pkg_data);
	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);

	return nr_activated;
}

/* Update prmid's of pmonrs in ilstate. To mantain fairness of rotation
 * logic, Try to activate (IN)state pmonrs with recovered prmids when
 * possible rather than simply adding them to free rmids list. This prevents,
 * ustate pmonrs (pmonrs that haven't wait in istate_pmonrs_lru) to obtain
 * the newly available RMIDs before those waiting in queue.
 */
static inline int
__try_free_ilstate_prmids(struct pkg_data *pkg_data,
			  unsigned int cqm_threshold,
			  unsigned int *min_occupancy_dirty)
{
	struct pmonr *pmonr, *tmp_pmonr, *istate_pmonr;
	struct prmid *prmid;
	unsigned long flags;
	u64 val;
	bool succeed;
	int ret, nr_activated = 0;
#ifdef CONFIG_LOCKDEP
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
#endif

	lockdep_assert_held(&pkg_data->pkg_data_mutex);

	WARN_ON_ONCE(try_reuse_ilstate_pmonrs(pkg_data));

	/* No need to acquire pkg lock to iterate over ilstate_pmonrs_lru
	 * since only rotation logic modifies it.
	 */
	list_for_each_entry_safe(
		pmonr, tmp_pmonr,
		&pkg_data->ilstate_pmonrs_lru, limbo_rotation_entry) {

		if (WARN_ON_ONCE(list_empty(&pkg_data->istate_pmonrs_lru)))
			return nr_activated;

		istate_pmonr = list_first_entry(&pkg_data->istate_pmonrs_lru,
						struct pmonr, rotation_entry);

		if (pmonr == istate_pmonr) {
			raw_spin_lock_irqsave_nested(
				&pkg_data->pkg_data_lock, flags, pkg_id);

			nr_activated++;
			__pmonr__ilstate_to_astate(pmonr);

			raw_spin_unlock_irqrestore(
				&pkg_data->pkg_data_lock, flags);
			continue;
		}

		ret = __cqm_prmid_update(pmonr->limbo_prmid,
					  __rmid_min_update_time);
		if (WARN_ON_ONCE(ret < 0))
			continue;

		val = atomic64_read(&pmonr->limbo_prmid->last_read_value);
		if (val > cqm_threshold) {
			if (val < *min_occupancy_dirty)
				*min_occupancy_dirty = val;
			continue;
		}

		raw_spin_lock_irqsave_nested(
			&pkg_data->pkg_data_lock, flags, pkg_id);

		prmid = pmonr->limbo_prmid;

		/* moves the prmid to free_prmids_pool. */
		__pmonr__ilstate_to_instate(pmonr);

		/* Do not affect ilstate_pmonrs_lru.
		 * If succeeds, prmid will end in active_prmids_pool,
		 * otherwise, stays in free_prmids_pool where the
		 * ilstate_to_instate transition left it.
		 */
		nr_activated += __try_use_free_prmid(pkg_data,
						     prmid, &succeed);

		raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	}
	return nr_activated;
}

/* Update limbo prmid's no associated to a pmonr. To mantain fairness of
 * rotation logic, Try to activate (IN)state pmonrs with recovered prmids when
 * possible rather than simply adding them to free rmids list. This prevents,
 * ustate pmonrs (pmonrs that haven't wait in istate_pmonrs_lru) to obtain
 * the newly available RMIDs before those waiting in queue.
 */
static inline int
__try_free_limbo_prmids(struct pkg_data *pkg_data,
			unsigned int cqm_threshold,
			unsigned int *min_occupancy_dirty)
{
	struct prmid *prmid, *tmp_prmid;
	unsigned long flags;
	bool succeed;
	int ret, nr_activated = 0;

#ifdef CONFIG_LOCKDEP
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
#endif
	u64 val;

	lockdep_assert_held(&pkg_data->pkg_data_mutex);

	list_for_each_entry_safe(
		prmid, tmp_prmid,
		&pkg_data->nopmonr_limbo_prmids_pool, pool_entry) {

		/* If min update time is good enough for user, it is good
		 * enough for rotation.
		 */
		ret = __cqm_prmid_update(prmid, __rmid_min_update_time);
		if (WARN_ON_ONCE(ret < 0))
			continue;

		val = atomic64_read(&prmid->last_read_value);
		if (val > cqm_threshold) {
			if (val < *min_occupancy_dirty)
				*min_occupancy_dirty = val;
			continue;
		}
		raw_spin_lock_irqsave_nested(
			&pkg_data->pkg_data_lock, flags, pkg_id);

		nr_activated = __try_use_free_prmid(pkg_data, prmid, &succeed);
		if (!succeed)
			list_move_tail(&prmid->pool_entry,
				       &pkg_data->free_prmids_pool);

		raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	}
	return nr_activated;
}

/*
 * Activate (I)state pmonrs.
 *
 * @min_occupancy_dirty: pointer to store the minimum occupancy of any
 *   dirty prmid.
 *
 * Try to activate as many pmonrs as possible before utilizing limbo prmids
 * pointed by ilstate pmonrs in order to minimize the number of dirty rmids
 * that move to other pmonr when cqm_threshold > 0.
 */
static int __try_activate_istate_pmonrs(
	struct pkg_data *pkg_data, unsigned int cqm_threshold,
	unsigned int *min_occupancy_dirty)
{
	int nr_activated = 0;

	lockdep_assert_held(&pkg_data->pkg_data_mutex);

	/* Start reusing limbo prmids no pointed by any ilstate pmonr. */
	nr_activated += __try_free_limbo_prmids(pkg_data, cqm_threshold,
						min_occupancy_dirty);

	/* Try to use newly available free prmids */
	nr_activated += __try_use_free_prmids(pkg_data);

	/* Continue reusing limbo prmids pointed by a ilstate pmonr. */
	nr_activated += __try_free_ilstate_prmids(pkg_data, cqm_threshold,
						  min_occupancy_dirty);
	/* Try to use newly available free prmids */
	nr_activated += __try_use_free_prmids(pkg_data);

	WARN_ON_ONCE(try_reuse_ilstate_pmonrs(pkg_data));
	return nr_activated;
}

/* Number of pmonrs that have been in (I)state for at least min_wait_jiffies.
 * XXX: Use rcu to access to istate_pmonrs_lru.
 */
static int
count_istate_pmonrs(struct pkg_data *pkg_data,
		    unsigned int min_wait_jiffies, bool exclude_limbo)
{
	unsigned long flags;
	unsigned int c = 0;
	struct pmonr *pmonr;
#ifdef CONFIG_DEBUG_SPINLOCK
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
#endif

	lockdep_assert_held(&pkg_data->pkg_data_mutex);

	raw_spin_lock_irqsave_nested(&pkg_data->pkg_data_lock, flags, pkg_id);
	list_for_each_entry(
		pmonr, &pkg_data->istate_pmonrs_lru, rotation_entry) {

		if (jiffies - pmonr->last_enter_istate < min_wait_jiffies)
			break;

		WARN_ON_ONCE(!__pmonr__in_istate(pmonr));
		if (exclude_limbo && __pmonr__in_ilstate(pmonr))
			continue;
		c++;
	}
	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);

	return c;
}

static inline int
read_nr_instate_pmonrs(struct pkg_data *pkg_data, u16 pkg_id) {
	unsigned long flags;
	int n;

	raw_spin_lock_irqsave_nested(&pkg_data->pkg_data_lock, flags, pkg_id);
	n = READ_ONCE(cqm_pkgs_data[pkg_id]->nr_instate_pmonrs);
	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	WARN_ON_ONCE(n < 0);
	return n;
}

/*
 * Rotate RMIDs among rpgks.
 *
 * For reads to be meaningful valid rmids had to be programmed for
 * enough time to capture enough instances of cache allocation/retirement
 * to yield useful occupancy values. The approach to handle that problem
 * is to guarantee that every pmonr will spend at least T time in (A)state
 * when such transition has occurred and hope that T is long enough.
 *
 * The hardware retains occupancy for 'old' tags, even after changing rmid
 * for a task/cgroup. To workaround this problem, we keep retired rmids
 * as limbo in each pmonr and use their occupancy. Also we prefer reusing
 * such limbo rmids rather than free ones since their residual occupancy
 * is valid occupancy for the task/cgroup.
 *
 * Rotation works by taking away an RMID from a group (the old RMID),
 * and assigning the free RMID to another group (the new RMID). We must
 * then wait for the old RMID to not be used (no cachelines tagged).
 * This ensure that all cachelines are tagged with 'active' RMIDs. At
 * this point we can start reading values for the new RMID and treat the
 * old RMID as the free RMID for the next rotation.
 */
static void
__intel_cqm_rmid_rotate(struct pkg_data *pkg_data,
			unsigned int nr_max_limbo,
			unsigned int nr_min_activated)
{
	int nr_instate, nr_to_steal, nr_stolen, nr_slo_violated;
	int limbo_cushion = 0;
	unsigned int cqm_threshold = 0, min_occupancy_dirty;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());

	/*
	 * To avoid locking the process, keep track of pmonrs that
	 * are activated during this execution of rotaton logic, so
	 * we don't have to rely on the state of the pmonrs lists
	 * to estimate progress, that can be modified during
	 * creation and destruction of events and cgroups.
	 */
	int nr_activated = 0;

	mutex_lock_nested(&pkg_data->pkg_data_mutex, pkg_id);

	/*
	 * Since ilstates are created only during stealing or destroying pmonrs,
	 * but destroy requires pkg_data_mutex, then it is only necessary to
	 * try to reuse ilstate once per call. Furthermore, new ilstates during
	 * iteration in rotation logic is an error.
	*/
	nr_activated += try_reuse_ilstate_pmonrs(pkg_data);

again:
	nr_stolen = 0;
	min_occupancy_dirty = UINT_MAX;
	/*
	 * Three types of actions are taken in rotation logic:
	 *   1) Try to activate pmonrs using limbo RMIDs.
	 *   2) Steal more RMIDs. Ideally the number of RMIDs in limbo equals
	 *   the number of pmonrs in (I)state plus the limbo_cushion aimed to
	 *   compensate for limbo RMIDs that do no drop occupancy fast enough.
	 *   The actual number stolen is constrained
	 *   prevent having more than nr_max_limbo RMIDs in limbo.
	 *   3) Increase cqm_threshold so even RMIDs with residual occupancy
	 *   are utilized to activate (I)state primds. Doing so increases the
	 *   error in the reported value in a way undetectable to the user, so
	 *   it is left as a last resource.
	 */

	/* Verify all available ilimbo where activated where they
	 * were supposed to.
	 */
	WARN_ON_ONCE(try_reuse_ilstate_pmonrs(pkg_data) > 0);

	/* Activate all pmonrs that we can by recycling rmids in limbo */
	nr_activated += __try_activate_istate_pmonrs(
		pkg_data, cqm_threshold, &min_occupancy_dirty);

	/* Count nr of pmonrs that are inherited and do not have limbo_prmid */
	nr_instate = read_nr_instate_pmonrs(pkg_data, pkg_id);
	WARN_ON_ONCE(nr_instate < 0);
	/*
	 * If no pmonr needs rmid, then it's time to let go. pmonrs in ilimbo
	 * are not counted since the limbo_prmid can be reused, once its time
	 * to activate them.
	 */
	if (nr_instate == 0)
		goto exit;

	WARN_ON_ONCE(!list_empty(&pkg_data->free_prmids_pool));
	WARN_ON_ONCE(try_reuse_ilstate_pmonrs(pkg_data) > 0);

	/* There are still pmonrs waiting for RMID, check if the SLO about
	 * _cqm_max_wait_mon has been violated. If so, use a more
	 * aggresive version of RMID stealing and reutilization.
	 */
	nr_slo_violated = count_istate_pmonrs(
		pkg_data, msecs_to_jiffies(__cqm_max_wait_mon), false);

	/* First measure against SLO violation is to increase number of stolen
	 * RMIDs beyond the number of pmonrs waiting for RMID. The magnitud of
	 * the limbo_cushion is proportional to nr_slo_violated (but
	 * arbitarily weighthed).
	 */
	if (nr_slo_violated)
		limbo_cushion = (nr_slo_violated + 1) / 2;

	/*
	 * Need more free rmids. Steal RMIDs from active pmonrs and place them
	 * into limbo lru. Steal enough to have high chances that eventually
	 * occupancy of enough RMIDs in limbo will drop enough to be reused
	 * (the limbo_cushion).
	 */
	nr_to_steal = min(nr_instate + limbo_cushion,
			  max(0, (int)nr_max_limbo -
				 count_limbo_prmids(pkg_data)));

	if (nr_to_steal)
		nr_stolen = __try_steal_active_pmonrs(pkg_data, nr_to_steal);

	/* Already stole as many as possible, finish if no SLO violations. */
	if (!nr_slo_violated)
		goto exit;

	/*
	 * There are SLO violations due to recycling RMIDs not progressing
	 * fast enough. Possible (non-exclusive) causal factors are:
	 *   1) Too many RMIDs in limbo do not drop occupancy despite having
	 *      spent a "reasonable" time in limbo lru.
	 *   2) RMIDs in limbo have not been for long enough to have drop
	 *	occupancy, but they will within "reasonable" time.
	 *
	 * If (2) only, it is ok to wait, since eventually the rmids
	 * will rotate. If (1), there is a danger of being stuck, in that case
	 * the dirty threshold, cqm_threshold, must be increased.
	 * The notion of "reasonable" time is ambiguous since the more SLOs
	 * violations, the more urgent it is to rotate. For now just try
	 * to guarantee any progress is made (activate at least one prmid
	 * with SLO violated).
	 */

	/* Using the minimum observed occupancy in dirty rmids guarantees to
	 * to recover at least one rmid per iteration. Check if constrainst
	 * would allow to use such threshold, otherwise makes no sense to
	 * retry.
	 */
	if (nr_activated < nr_min_activated && min_occupancy_dirty <=
		READ_ONCE(__intel_cqm_max_threshold) / cqm_l3_scale) {

		cqm_threshold = min_occupancy_dirty;
		goto again;
	}
exit:
	mutex_unlock(&pkg_data->pkg_data_mutex);
}

static void
__intel_cqm_timed_update(u16 pkg_id)
{
	int ret;

	mutex_lock_nested(&cqm_pkgs_data[pkg_id]->pkg_data_mutex, pkg_id);
	ret = update_rmids_in_astate_pmonrs_lru(pkg_id);
	mutex_unlock(&cqm_pkgs_data[pkg_id]->pkg_data_mutex);
	WARN_ON_ONCE(ret);
}

static struct pmu intel_cqm_pmu;

/* Rotation only needs to be run when there is any pmonr in (I)state. */
static bool intel_cqm_need_rotation(u16 pkg_id)
{

	struct pkg_data *pkg_data;
	bool need_rot;

	pkg_data = cqm_pkgs_data[pkg_id];

	mutex_lock_nested(&pkg_data->pkg_data_mutex, pkg_id);
	/* Rotation is needed if prmids in limbo need to be recycled or if
	 * there are pmonrs in (I)state.
	 */
	need_rot = !list_empty(&pkg_data->nopmonr_limbo_prmids_pool) ||
		   !list_empty(&pkg_data->istate_pmonrs_lru);

	mutex_unlock(&pkg_data->pkg_data_mutex);
	return need_rot;
}

static bool intel_cqm_need_timed_update(u16 pkg_id)
{

	struct pkg_data *pkg_data;
	bool need_update;

	pkg_data = cqm_pkgs_data[pkg_id];

	mutex_lock_nested(&pkg_data->pkg_data_mutex, pkg_id);
	/* Update is needed if prmids if there is any active prmid. */
	need_update = !list_empty(&pkg_data->active_prmids_pool);
	mutex_unlock(&pkg_data->pkg_data_mutex);

	return need_update;
}

/*
 * Schedule rotation in one package.
 */
static void __intel_cqm_schedule_rotation_for_pkg(u16 pkg_id)
{
	struct pkg_data *pkg_data;
	unsigned long delay;

	delay = msecs_to_jiffies(intel_cqm_pmu.hrtimer_interval_ms);
	pkg_data = cqm_pkgs_data[pkg_id];
	schedule_delayed_work_on(
		pkg_data->rotation_cpu, &pkg_data->rotation_work, delay);
}

static void __intel_cqm_schedule_timed_update_for_pkg(u16 pkg_id)
{
	struct pkg_data *pkg_data;
	unsigned long delay;

	delay = msecs_to_jiffies(__rmid_timed_update_period);
	pkg_data = cqm_pkgs_data[pkg_id];
	schedule_delayed_work_on(
		pkg_data->timed_update_cpu,
		&pkg_data->timed_update_work, delay);
}


/*
 * Schedule rotation and rmid's timed update in all packages.
 * Reescheduling will stop when no longer needed.
 */
static void intel_cqm_schedule_work_all_pkgs(void)
{
	int pkg_id;

	cqm_pkg_id_for_each_online(pkg_id) {
		__intel_cqm_schedule_rotation_for_pkg(pkg_id);
		__intel_cqm_schedule_timed_update_for_pkg(pkg_id);
	}
}

static void intel_cqm_rmid_rotation_work(struct work_struct *work)
{
	struct pkg_data *pkg_data = container_of(
		to_delayed_work(work), struct pkg_data, rotation_work);
	/* Allow max 25% of RMIDs to be in limbo. */
	unsigned int max_limbo_rmids = max(1u, (pkg_data->max_rmid + 1) / 4);
	unsigned int min_activated = max(1u, (intel_cqm_pmu.hrtimer_interval_ms
		* __cqm_min_progress_rate) / 1000);
	u16 pkg_id = topology_physical_package_id(pkg_data->rotation_cpu);

	WARN_ON_ONCE(pkg_data != cqm_pkgs_data[pkg_id]);

	__intel_cqm_rmid_rotate(pkg_data, max_limbo_rmids, min_activated);

	if (intel_cqm_need_rotation(pkg_id))
		__intel_cqm_schedule_rotation_for_pkg(pkg_id);
}

static void intel_cqm_timed_update_work(struct work_struct *work)
{
	struct pkg_data *pkg_data = container_of(
		to_delayed_work(work), struct pkg_data, timed_update_work);
	u16 pkg_id = topology_physical_package_id(pkg_data->timed_update_cpu);

	WARN_ON_ONCE(pkg_data != cqm_pkgs_data[pkg_id]);

	__intel_cqm_timed_update(pkg_id);

	if (intel_cqm_need_timed_update(pkg_id))
		__intel_cqm_schedule_timed_update_for_pkg(pkg_id);
}

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

static struct monr *
monr_next_child(struct monr *pos, struct monr *parent)
{
#ifdef CONFIG_LOCKDEP
	WARN_ON(!monr_hrchy_count_held_raw_spin_locks());
#endif
	if (!pos)
		return list_first_entry_or_null(
			&parent->children, struct monr, parent_entry);
	if (list_is_last(&pos->parent_entry, &parent->children))
		return NULL;
	return list_next_entry(pos, parent_entry);
}

static struct monr *
monr_next_descendant_pre(struct monr *pos, struct monr *root)
{
	struct monr *next;

#ifdef CONFIG_LOCKDEP
	WARN_ON(!monr_hrchy_count_held_raw_spin_locks());
#endif
	if (!pos)
		return root;
	next = monr_next_child(NULL, pos);
	if (next)
		return next;
	while (pos != root) {
		next = monr_next_child(pos, pos->parent);
		if (next)
			return next;
		pos = pos->parent;
	}
	return NULL;
}

/* Read pmonr's summary, safe to call without pkg's prmids lock.
 * The possible scenarios are:
 *  - summary's occupancy cannot be read, return -1.
 *  - summary has no RMID but could be read as zero occupancy, return 0 and set
 *    rmid = INVALID_RMID.
 *  - summary has valid read RMID, set rmid to it.
 */
static inline int
pmonr__get_read_rmid(struct pmonr *pmonr, u32 *rmid, bool fail_on_inherited)
{
	union prmid_summary summary;

	*rmid = INVALID_RMID;

	summary.value = atomic64_read(&pmonr->prmid_summary_atomic);
	/* A pmonr in (I)state that doesn't fail can report it's limbo_prmid
	 * or NULL.
	 */
	if (prmid_summary__is_istate(summary) && fail_on_inherited)
		return -1;
	/* A pmonr with inactive monitoring can be safely ignored. */
	if (!prmid_summary__is_mon_active(summary))
		return 0;

	/* A pmonr that hasnt run in a pkg is safe to ignore since it
	 * cannot have occupancy there.
	 */
	if (prmid_summary__is_ustate(summary))
		return 0;
	/* At this point the pmonr is either in (A)state or (I)state
	 * with fail_on_inherited=false . In the latter case,
	 * read_rmid is INVALID_RMID and is a successful read_rmid.
	 */
	*rmid = summary.read_rmid;
	return 0;
}

/* Read occupancy for all pmonrs in the subtree rooted at monr
 * for the current package.
 * Best effort two-stages read. First, obtain all RMIDs in subtree
 * with locks held. The rmids are added to stack. If stack is full
 * proceed to update and read in place. After finish storing the RMIDs,
 * update and read occupancy for rmids in stack.
 */
static int pmonr__read_subtree(struct monr *monr, u16 pkg_id,
			       u64 *total, bool fail_on_inh_descendant)
{
	struct monr *pos = NULL;
	struct astack astack;
	int ret;
	unsigned long flags;
	u64 count;
	struct pkg_data *pkg_data = cqm_pkgs_data[pkg_id];

	*total = 0;
	/* Must run in a CPU in the package to read. */
	if (WARN_ON_ONCE(pkg_id !=
			 topology_physical_package_id(smp_processor_id())))
		return -1;

	astack__init(&astack, NR_RMIDS_PER_NODE - 1, pkg_id);

	/* Lock to protect againsts changes in pmonr hierarchy. */
	raw_spin_lock_irqsave_nested(&pkg_data->pkg_data_lock, flags, pkg_id);

	while ((pos = monr_next_descendant_pre(pos, monr))) {
		struct prmid *prmid;
		u32 rmid;
		/* the pmonr of the monr to read cannot be inherited,
		 * descendants may, depending on flag.
		 */
		bool fail_on_inh = pos == monr || fail_on_inh_descendant;

		ret = pmonr__get_read_rmid(pos->pmonrs[pkg_id],
					   &rmid, fail_on_inh);
		if (ret)
			goto exit_error;

		if (rmid == INVALID_RMID)
			continue;

		ret = astack__push(&astack);
		if (!ret) {
			__astack__top(&astack, rmids) = rmid;
			continue;
		}
		/* If no space in stack, update and read here (slower). */
		prmid = __prmid_from_rmid(pkg_id, rmid);
		if (WARN_ON_ONCE(!prmid))
			goto exit_error;

		ret = cqm_prmid_update(prmid);
		if (ret < 0)
			goto exit_error;

		*total += atomic64_read(&prmid->last_read_value);
	}
	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);

	ret = astack__rmids_sum_apply(&astack, pkg_id,
				      &__rmid_fn__cqm_prmid_update, &count);
	if (ret < 0)
		return ret;

	*total += count;
	astack__release(&astack);

	return 0;

exit_error:
	raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	astack__release(&astack);
	return ret;
}

/* Read current package immediately and remote pkg (if any) from cache. */
static int __read_task_event(struct perf_event *event)
{
	int i, ret;
	u64 count = 0;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
	struct monr *monr = monr_from_event(event);

	/* Read either local or polled occupancy from all packages. */
	cqm_pkg_id_for_each_online(i) {
		struct prmid *prmid;
		u32 rmid;
		struct pmonr *pmonr = monr->pmonrs[i];

		ret = pmonr__get_read_rmid(pmonr, &rmid, true);
		if (ret)
			return ret;
		if (rmid == INVALID_RMID)
			continue;
		prmid = __prmid_from_rmid(i, rmid);
		if (WARN_ON_ONCE(!prmid))
			return -1;

		/* update and read local for this cpu's package. */
		if (i == pkg_id) {
			ret = cqm_prmid_update(prmid);
			if (ret < 0)
				return ret;
		}
		count += atomic64_read(&prmid->last_read_value);
	}
	local64_set(&event->count, count);
	return 0;
}

/* Read current package immediately and remote pkg (if any) from cache. */
static int intel_cqm_event_read(struct perf_event *event)
{
	struct monr *monr;
	u64 count;
	int ret;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());

	monr = monr_from_event(event);

	WARN_ON_ONCE(event->cpu != -1 &&
		     topology_physical_package_id(event->cpu) != pkg_id);

	/* Only perf_event leader can return a value, everybody else share
	 * the same RMID.
	 */
	if (event->parent) {
		local64_set(&event->count, 0);
		return 0;
	}

	if (event->attach_state & PERF_ATTACH_TASK) {
		return __read_task_event(event);
	}

	/* It's either a cgroup or a cpu event. */
	if (WARN_ON_ONCE(event->cpu < 0))
		return -1;

	/* XXX: expose fail_on_inh_descendant as a configuration parameter? */
	ret =  pmonr__read_subtree(monr, pkg_id, &count, false);
	if (ret < 0)
		return ret;

	local64_set(&event->count, count);

	pr_info("reading cgruop event\n");

	return 0;
}

static inline void __intel_cqm_event_start(
	struct perf_event *event, union prmid_summary summary)
{
	if (!(event->hw.state & PERF_HES_STOPPED))
		return;
	event->hw.state &= ~PERF_HES_STOPPED;

/*
pr_info("in start cpu:%d, rmid:%d, pid:%d\n",
		smp_processor_id(), summary.sched_rmid, event->hw.target->pid);

	dump_stack();
*/

	pqr_cache_update_rmid(summary.sched_rmid, PQR_RMID_MODE_EVENT);
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
	pqr_cache_update_rmid(summary.sched_rmid, PQR_RMID_MODE_NOEVENT);
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
	return prmid_summary__is_istate(summary) ? -1 : 0;
}

static inline bool cqm_group_leader(struct perf_event *event)
{
	return !list_empty(&event->hw.cqm_event_groups_entry);
}

static void intel_cqm_event_terminate(struct perf_event *event)
{
	struct perf_event *group_other = NULL;
	struct monr *monr;
	int i;
	unsigned long flags;

	mutex_lock(&cqm_mutex);

	total_events--;
	pr_info("destroyed event #%d\n",total_events);

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

#ifdef CONFIG_CGROUP_PERF
	/* Handle cgroup event. */
	if (event->cgrp) {
		monr->mon_event_group = NULL;
		if ((event->cgrp->css.flags & CSS_ONLINE) &&
		    !cgrp_to_cqm_info(event->cgrp)->cont_monitoring)
			__css_stop_monitoring(&monr__get_mon_cgrp(monr)->css);

		goto exit;
	}
	WARN_ON_ONCE(!monr_is_event_type(monr));
#endif

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
	    event->attr.inherit_stat   || /* cqm groups share rmid */
	    event->attr.sample_period) /* no sampling */
		return -EINVAL;

	INIT_LIST_HEAD(&event->hw.cqm_event_groups_entry);
	INIT_LIST_HEAD(&event->hw.cqm_event_group_entry);

	/*
	 * CQM driver handles cgroup recursion and since only noe
	 * RMID can be programmed at the time in each core, then
	 * it is incompatible with the way generic code handles
	 * cgroup hierarchies.
	 */
	event->pmu_event_flags |= PERF_CGROUP_NO_RECURSION;

	/* Events in CQM PMU are per-package and can be read even when
	 * the cpu is not running the event.
	 */
	if (event->cpu < 0) {
		WARN_ON_ONCE(!(event->attach_state & PERF_ATTACH_TASK));
		event->pmu_event_flags |= PERF_INACTIVE_EV_READ_ANY_CPU;
	} else	{
		event->pmu_event_flags |= PERF_INACTIVE_CPU_READ_PKG;
	}

	mutex_lock(&cqm_mutex);

	total_events++;
	pr_info("created event #%d\n",total_events);

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

	intel_cqm_schedule_work_all_pkgs();

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
	.event_terminate     = intel_cqm_event_terminate,
	.add		     = intel_cqm_event_add,
	.del		     = intel_cqm_event_stop,
	.start		     = intel_cqm_event_start,
	.stop		     = intel_cqm_event_stop,
	.read		     = intel_cqm_event_read,
};

#ifdef CONFIG_CGROUP_PERF
/* XXX: Add hooks for attach dettach task with monr to a cgroup. */
int perf_cgroup_arch_css_alloc(struct cgroup_subsys_state *parent_css,
				      struct cgroup_subsys_state *new_css)
{
	struct perf_cgroup *new_cgrp;
	struct cgrp_cqm_info *cqm_info;

	new_cgrp = css_to_perf_cgroup(new_css);
	cqm_info = kmalloc(sizeof(struct cgrp_cqm_info), GFP_KERNEL);
	if (!cqm_info)
		return -ENOMEM;
	cqm_info->cont_monitoring = false;
	cqm_info->monr = NULL;
	new_cgrp->arch_info = cqm_info;

	return 0;
}

void perf_cgroup_arch_css_free(struct cgroup_subsys_state *css)
{
	struct perf_cgroup *cgrp = css_to_perf_cgroup(css);

	kfree(cgrp_to_cqm_info(cgrp));
	cgrp->arch_info = NULL;
}

/* Do the bulk of arch_css_online. To be called when CQM starts after
 * css has gone online.
 */
static inline int __css_go_online(struct cgroup_subsys_state *css)
{
	lockdep_assert_held(&cqm_mutex);

	/* css must not be used in monr hierarchy before having
	 * set its monr in this step.
	 */
	__css_set_monr_to_lma(css);
	/* Root monr is always monitoring. */
	if (!css->parent)
		css_to_cqm_info(css)->cont_monitoring = true;

	if (css_to_cqm_info(css)->cont_monitoring)
		return __css_start_monitoring(css);
	return 0;
}

int perf_cgroup_arch_css_online(struct cgroup_subsys_state *css)
{
	int ret = 0;

	/* use cqm_init_mutex to synchronize with
	 * __start_monitoring_all_cgroups.
	 */
	mutex_lock(&cqm_init_mutex);

	if (static_branch_unlikely(&cqm_initialized_key)) {
		mutex_lock(&cqm_mutex);
		ret = __css_go_online(css);
		mutex_unlock(&cqm_mutex);
		WARN_ON_ONCE(ret);
	}

	mutex_unlock(&cqm_init_mutex);
	return ret;
}

void perf_cgroup_arch_css_offline(struct cgroup_subsys_state *css)
{
	int ret = 0;
	struct monr *monr;
	struct perf_cgroup *cgrp = css_to_perf_cgroup(css);

	mutex_lock(&cqm_init_mutex);

	if (!static_branch_unlikely(&cqm_initialized_key))
		goto out;

	mutex_lock(&cqm_mutex);

	monr = monr_from_perf_cgroup(cgrp);
	if (!perf_cgroup_is_monitored(cgrp))
		goto out_cqm;

	/* Stop monitoring for the css's monr only if no more events need it.
	 * If events need the monr, it will be destroyed when the events that
	 * use it are destroyed.
	 */
	if (monr->mon_event_group) {
		monr_hrchy_acquire_mutexes();
		perf_cgroup_make_unmonitored(cgrp);
		monr_hrchy_release_mutexes();
	} else {
		ret = __css_stop_monitoring(css);
		WARN_ON_ONCE(ret);
	}

out_cqm:
	mutex_unlock(&cqm_mutex);
out:
	mutex_unlock(&cqm_init_mutex);
	WARN_ON_ONCE(ret);
}

void perf_cgroup_arch_css_released(struct cgroup_subsys_state *css)
{
	mutex_lock(&cqm_init_mutex);

	if (static_branch_unlikely(&cqm_initialized_key)) {
		mutex_lock(&cqm_mutex);
		/*
		 * Remove css from monr hierarchy now that css is about to
		 * leave the cgroup hierarchy.
		 */
		perf_cgroup_set_monr(css_to_perf_cgroup(css), NULL);
		mutex_unlock(&cqm_mutex);
	}

	mutex_unlock(&cqm_init_mutex);
}

#endif

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
	state->next_rmid = 0;
	state->next_rmid_mode = PQR_RMID_MODE_NOEVENT;
	state->closid = 0;
	state->next_closid = 0;

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

#ifdef CONFIG_CGROUP_PERF
/* Start monitoring for all cgroups in cgroup hierarchy. */
static int __start_monitoring_all_cgroups(void)
{
	int ret;
	struct cgroup_subsys_state *css, *css_root;

	lockdep_assert_held(&cqm_init_mutex);

	rcu_read_lock();
	/* Get css for root cgroup */
	css_root =  get_root_perf_css();

	css_for_each_descendant_pre(css, css_root) {
		if (!css_tryget_online(css))
			continue;

		rcu_read_unlock();
		mutex_lock(&cqm_mutex);
		ret = __css_go_online(css);
		mutex_unlock(&cqm_mutex);

		css_put(css);
		if (ret)
			return ret;

		rcu_read_lock();
	}
	rcu_read_unlock();
	return 0;
}
#endif

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

	/* Use cqm_init_mutex to synchronize with css's online/offline. */
	mutex_lock(&cqm_init_mutex);

#ifdef CONFIG_CGROUP_PERF
	ret = __start_monitoring_all_cgroups();
	if (ret)
		goto error_init_mutex;
#endif

	ret = perf_pmu_register(&intel_cqm_pmu, "intel_cqm", -1);
	if (ret)
		goto error_init_mutex;

	cpu_notifier_register_done();

	static_branch_enable(&cqm_initialized_key);

	mutex_unlock(&cqm_init_mutex);

	pr_info("Intel CQM monitoring enabled with at least %u rmids per package.\n",
		min_max_rmid + 1);

	/* Make sure pqr_common_enable_key is enabled after
	 * cqm_initialized_key.
	 */
	barrier();

	static_branch_enable(&pqr_common_enable_key);
	return ret;

error_init_mutex:
	mutex_unlock(&cqm_init_mutex);
error:
	pr_err("Intel CQM perf registration failed: %d\n", ret);
	cpu_notifier_register_done();

	return ret;
}

/* Schedule task without a CQM perf_event. */
inline void __intel_cqm_no_event_sched_in(void)
{
#ifdef CONFIG_CGROUP_PERF
	struct monr *monr;
	struct pmonr *pmonr;
	union prmid_summary summary;
	u16 pkg_id = topology_physical_package_id(smp_processor_id());
	struct pmonr *root_pmonr = monr_hrchy_root->pmonrs[pkg_id];

	/* Assume CQM enabled is likely given that PQR is enabled. */
	if (!static_branch_likely(&cqm_initialized_key))
		return;

	/* Safe to call from_task since we are in scheduler lock. */
	monr = monr_from_perf_cgroup(perf_cgroup_from_task(current, NULL));
	pmonr = monr->pmonrs[pkg_id];

	/* Utilize most up to date pmonr summary. */
	monr_hrchy_get_next_prmid_summary(pmonr);
	summary.value = atomic64_read(&pmonr->prmid_summary_atomic);

	if (!prmid_summary__is_mon_active(summary))
		goto no_rmid;

	if (WARN_ON_ONCE(!__valid_rmid(pkg_id, summary.sched_rmid)))
		goto no_rmid;

	pqr_cache_update_rmid(summary.sched_rmid, PQR_RMID_MODE_NOEVENT);
	return;

no_rmid:
	summary.value = atomic64_read(&root_pmonr->prmid_summary_atomic);
	pqr_cache_update_rmid(summary.sched_rmid, PQR_RMID_MODE_NOEVENT);
#endif
}

#ifdef CONFIG_CGROUP_PERF

/* kernfs guarantees that css doesn't need to be pinned. */
static u64 cqm_cont_monitoring_read_u64(struct cgroup_subsys_state *css,
					struct cftype *cft)
{
	int ret = -1;
	struct perf_cgroup *perf_cgrp = css_to_perf_cgroup(css);
	struct monr *monr;

	mutex_lock(&cqm_init_mutex);
	if (!static_branch_likely(&cqm_initialized_key))
		goto out;

	mutex_lock(&cqm_mutex);

	ret = css_to_cqm_info(css)->cont_monitoring;
	monr = monr_from_perf_cgroup(perf_cgrp);
	WARN_ON(!monr->mon_event_group &&
		(ret != perf_cgroup_is_monitored(perf_cgrp)));

	mutex_unlock(&cqm_mutex);
out:
	mutex_unlock(&cqm_init_mutex);
	return ret;
}

/* kernfs guarantees that css doesn't need to be pinned. */
static int cqm_cont_monitoring_write_u64(struct cgroup_subsys_state *css,
					 struct cftype *cft, u64 value)
{
	int ret = 0;
	struct perf_cgroup *perf_cgrp = css_to_perf_cgroup(css);
	struct monr *monr;

	if (value > 1)
		return -1;

	mutex_lock(&cqm_init_mutex);
	if (!static_branch_likely(&cqm_initialized_key)) {
		ret = -1;
		goto out;
	}

	/* Root cgroup cannot stop being monitored. */
	if (css == get_root_perf_css())
		goto out;

	mutex_lock(&cqm_mutex);

	monr = monr_from_perf_cgroup(perf_cgrp);

	if (value && !perf_cgroup_is_monitored(perf_cgrp))
		ret = __css_start_monitoring(css);
	else if (!value &&
		 !monr->mon_event_group && perf_cgroup_is_monitored(perf_cgrp))
		ret = __css_stop_monitoring(css);

	WARN_ON(!monr->mon_event_group &&
		(value != perf_cgroup_is_monitored(perf_cgrp)));

	css_to_cqm_info(css)->cont_monitoring = value;

	mutex_unlock(&cqm_mutex);
out:
	mutex_unlock(&cqm_init_mutex);
	return ret;
}

struct cftype perf_event_cgrp_arch_subsys_cftypes[] = {
	{
		.name = "cqm_cont_monitoring",
		.read_u64 = cqm_cont_monitoring_read_u64,
		.write_u64 = cqm_cont_monitoring_write_u64,
	},

	{}	/* terminate */
};

#endif

device_initcall(intel_cqm_init);
