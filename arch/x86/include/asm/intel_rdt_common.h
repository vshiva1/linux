#ifndef _ASM_X86_INTEL_RDT_COMMON_H
#define _ASM_X86_INTEL_RDT_COMMON_H

#define MSR_IA32_PQR_ASSOC	0x0c8f

/**
 * struct intel_pqr_state - State cache for the PQR MSR
 * @rmid:		The cached Resource Monitoring ID
 * @closid:		The cached Class Of Service ID
 * @rmid_usecnt:	The usage counter for rmid
 *
 * The upper 32 bits of MSR_IA32_PQR_ASSOC contain closid and the
 * lower 10 bits rmid. The update to MSR_IA32_PQR_ASSOC always
 * contains both parts, so we need to cache them.
 *
 * The cache also helps to avoid pointless updates if the value does
 * not change.
 */
struct intel_pqr_state {
	u32			rmid;
	u32			next_task_rmid;
	u32			closid;
	int			rmid_usecnt;
};

DECLARE_PER_CPU(struct intel_pqr_state, pqr_state);

u32 __get_rmid(int domain);
bool __rmid_valid(u32 rmid);
void alloc_needed_pkg_rmid(u32 *cqm_rmid);
struct cgrp_cqm_info *cqminfo_from_tsk(struct task_struct *tsk);

extern struct cgrp_cqm_info cqm_rootcginfo;

DECLARE_STATIC_KEY_FALSE(cqm_enable_key);
DECLARE_STATIC_KEY_FALSE(rdt_enable_key);

/**
 * struct cgrp_cqm_info - perf_event cgroup metadata for cqm
 * @cont_mon     Continuous monitoring flag
 * @mon_enabled  Whether monitoring is enabled
 * @level        Level in the cgroup tree. Root is level 0.
 * @rmid        The rmids of the cgroup.
 * @mfa          'Monitoring for ancestor' points to the cqm_info
 *  of the ancestor the cgroup is monitoring for. 'Monitoring for ancestor'
 *  means you will use an ancestors RMID at sched_in if you are
 *  not monitoring yourself.
 *
 *  Due to the hierarchical nature of cgroups, every cgroup just
 *  monitors for the 'nearest monitored ancestor' at all times.
 *  Since root cgroup is always monitored, all descendents
 *  at boot time monitor for root and hence all mfa points to root except
 *  for root->mfa which is NULL.
 *  1. RMID setup: When cgroup x start monitoring:
 *    for each descendent y, if y's mfa->level < x->level, then
 *    y->mfa = x. (Where level of root node = 0...)
 *  2. sched_in: During sched_in for x
 *    if (x->mon_enabled) choose x->rmid
 *    else choose x->mfa->rmid.
 *  3. read: for each descendent of cgroup x
 *     if (x->monitored) count += rmid_read(x->rmid).
 *  4. evt_destroy: for each descendent y of x, if (y->mfa == x) then
 *     y->mfa = x->mfa. Meaning if any descendent was monitoring for x,
 *     set that descendent to monitor for the cgroup which x was monitoring for.
 *
 * @tskmon_rlist List of tasks being monitored in the cgroup
 *  When a task which belongs to a cgroup x is being monitored, it always uses
 *  its own task->rmid even if cgroup x is monitored during sched_in.
 *  To account for the counts of such tasks, cgroup keeps this list
 *  and parses it during read.
 *
 *  Perf handles hierarchy for other events, but because RMIDs are per pkg
 *  this is handled here.
*/
struct cgrp_cqm_info {
	bool cont_mon;
	bool mon_enabled;
	int level;
	u32 *rmid;
	struct cgrp_cqm_info *mfa;
	struct list_head tskmon_rlist;
};

struct tsk_rmid_entry {
	u32 *rmid;
	struct list_head list;
};

#ifdef CONFIG_CGROUP_PERF

# define css_to_perf_cgroup(css_) container_of(css_, struct perf_cgroup, css)
# define cgrp_to_cqm_info(cgrp_) ((struct cgrp_cqm_info *)cgrp_->arch_info)
# define css_to_cqm_info(css_) cgrp_to_cqm_info(css_to_perf_cgroup(css_))

#else

# define css_to_perf_cgroup(css_) NULL
# define cgrp_to_cqm_info(cgrp_) NULL
# define css_to_cqm_info(css_) NULL

#endif
#endif /* _ASM_X86_INTEL_RDT_COMMON_H */
