#ifndef _X86_PQR_COMMON_H_
#define _X86_PQR_COMMON_H_

#if defined(CONFIG_INTEL_RDT)

#include <linux/jump_label.h>
#include <linux/types.h>
#include <asm/percpu.h>
#include <asm/msr.h>

#define MSR_IA32_PQR_ASSOC	0x0c8f

#define INVALID_RMID		(-1)
#define INVALID_CLOSID		(-1)


extern struct static_key_false pqr_common_enable_key;

enum intel_pqr_rmid_mode {
	/* RMID has no perf_event associated. */
	PQR_RMID_MODE_NOEVENT = 0,
	/* RMID has a perf_event associated. */
	PQR_RMID_MODE_EVENT
};

/**
 * struct intel_pqr_state - State cache for the PQR MSR
 * @rmid:		Last RMID written to hw.
 * @rmid_mode:		Next RMID's mode.
 * @closid:		The current Class Of Service ID
 *
 * The upper 32 bits of MSR_IA32_PQR_ASSOC contain closid and the
 * lower 10 bits rmid. The update to MSR_IA32_PQR_ASSOC always
 * contains both parts, so we need to cache them.
 *
 * The cache also helps to avoid pointless updates if the value does not
 * change. It also keeps track of the type of RMID set (event vs no event)
 * used to determine when a cgroup RMID is required.
 */
struct intel_pqr_state {
	u32				rmid;
	enum intel_pqr_rmid_mode	rmid_mode;
	u32				closid;
};

DECLARE_PER_CPU(struct intel_pqr_state, pqr_state);

static inline void pqr_update_rmid(u32 rmid, enum intel_pqr_rmid_mode mode)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);

	state->rmid_mode = mode;

	if (state->rmid == rmid)
		return;
	state->rmid = rmid;
	wrmsr(MSR_IA32_PQR_ASSOC, rmid, state->closid);
}

void __pqr_ctx_switch(void);

inline void __intel_cqm_no_event_sched_in(void);

static inline void pqr_ctx_switch(void)
{
	if (static_branch_unlikely(&pqr_common_enable_key))
		__pqr_ctx_switch();
}

#else

static inline void pqr_ctx_switch(void)
{
}

#endif
#endif
