#ifndef _X86_PQR_COMMON_H_
#define _X86_PQR_COMMON_H_

#if defined(CONFIG_INTEL_RDT)

#include <linux/types.h>
#include <asm/percpu.h>
#include <asm/msr.h>

#define MSR_IA32_PQR_ASSOC	0x0c8f

#define INVALID_RMID		(-1)

/**
 * struct intel_pqr_state - State cache for the PQR MSR
 * @rmid:		The cached Resource Monitoring ID
 * @closid:		The cached Class Of Service ID
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
	u32			closid;
};

DECLARE_PER_CPU(struct intel_pqr_state, pqr_state);

static inline void pqr_update_rmid(u32 rmid)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);

	if (state->rmid == rmid)
		return;
	state->rmid = rmid;
	wrmsr(MSR_IA32_PQR_ASSOC, rmid, state->closid);
}

#endif
#endif
