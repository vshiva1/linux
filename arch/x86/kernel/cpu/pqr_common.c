#include <asm/pqr_common.h>

/*
 * The cached intel_pqr_state is strictly per CPU and can never be
 * updated from a remote CPU. Functions that modify pqr_state
 * must ensure interruptions are properly handled.
 */
DEFINE_PER_CPU(struct intel_pqr_state, pqr_state);

DEFINE_STATIC_KEY_FALSE(pqr_common_enable_key);

/* Update hw's RMID using cgroup's if perf_event did not.
 * Sync pqr cache with MSR.
 */
inline void __pqr_ctx_switch(void)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);

	/* If perf_event did set rmid that is used, do not try
	 * to obtain another one from current task.
	 */
	if (state->next_rmid_mode == PQR_RMID_MODE_NOEVENT)
		__intel_cqm_no_event_sched_in();

	/* __intel_cqm_no_event_sched_in might have changed next_rmid. */
	if (state->rmid == state->next_rmid &&
	    state->closid == state->next_closid)
		return;

	state->rmid = state->next_rmid;
	state->closid = state->next_closid;
	wrmsr(MSR_IA32_PQR_ASSOC, state->rmid, state->closid);
}
