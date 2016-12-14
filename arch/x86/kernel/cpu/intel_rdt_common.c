#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/cacheinfo.h>
#include <linux/cpuhotplug.h>
#include <asm/intel-family.h>
#include <asm/intel_rdt.h>

/*
 * The cached intel_pqr_state is strictly per CPU and can never be
 * updated from a remote CPU. Both functions which modify the state
 * (intel_cqm_event_start and intel_cqm_event_stop) are called with
 * interrupts disabled, which is sufficient for the protection.
 */
DEFINE_PER_CPU(struct intel_pqr_state, pqr_state);

#define pkg_id	topology_physical_package_id(smp_processor_id())

#ifdef CONFIG_INTEL_RDT_M
static inline int get_cgroup_sched_rmid(void)
{
#ifdef CONFIG_CGROUP_PERF
	struct cgrp_cqm_info *ccinfo = NULL;

	ccinfo = cqminfo_from_tsk(current);

	if (!ccinfo)
		return 0;

	/*
	 * A cgroup is always monitoring for itself or
	 * for an ancestor(default is root).
	 */
	if (ccinfo->mon_enabled) {
		alloc_needed_pkg_rmid(ccinfo->rmid);
		return ccinfo->rmid[pkg_id];
	} else {
		alloc_needed_pkg_rmid(ccinfo->mfa->rmid);
		return ccinfo->mfa->rmid[pkg_id];
	}
#endif

	return 0;
}

static inline int get_sched_in_rmid(void)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);
	u32 rmid = 0;

	rmid = state->next_task_rmid;

	return rmid ? rmid : get_cgroup_sched_rmid();
}
#endif

/*
 * intel_rdt_sched_in() - Writes the task's CLOSid to IA32_PQR_MSR
 *
 * Following considerations are made so that this has minimal impact
 * on scheduler hot path:
 * - This will stay as no-op unless we are running on an Intel SKU
 *   which supports resource control and we enable by mounting the
 *   resctrl file system or it supports resource monitoring.
 * - Caches the per cpu CLOSid/RMID values and does the MSR write only
 *   when a task with a different CLOSid/RMID is scheduled in.
 */
void __intel_rdt_sched_in(void)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);
	int closid = 0;
	u32 rmid = 0;

#ifdef CONFIG_INTEL_RDT_A
	if (static_branch_likely(&rdt_enable_key)) {
		/*
		 * If this task has a closid assigned, use it.
		 * Else use the closid assigned to this cpu.
		 */
		closid = current->closid;
		if (closid == 0)
			closid = this_cpu_read(cpu_closid);
	}
#endif

#ifdef CONFIG_INTEL_RDT_M
	if (static_branch_unlikely(&cqm_enable_key))
		rmid = get_sched_in_rmid();
#endif

	if (closid != state->closid || rmid != state->rmid) {

		pr_info("sched in cpu:%d,rmidnew:%d,old:%d \n",
				smp_processor_id(),rmid, state->rmid);

		state->closid = closid;
		state->rmid = rmid;
		wrmsr(MSR_IA32_PQR_ASSOC, rmid, closid);
	}
}
