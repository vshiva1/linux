#ifndef _ASM_X86_INTEL_PQR_COMMON_H
#define _ASM_X86_INTEL_PQR_COMMON_H

#ifdef CONFIG_INTEL_RDT

#include <linux/jump_label.h>
#include <linux/types.h>
#include <asm/percpu.h>
#include <asm/msr.h>
#include <asm/intel_rdt_common.h>

void __intel_rdt_sched_in(void);

/*
 * intel_rdt_sched_in() - Writes the task's CLOSid to IA32_PQR_MSR
 *
 * Following considerations are made so that this has minimal impact
 * on scheduler hot path:
 * - This will stay as no-op unless we are running on an Intel SKU
 *   which supports resource control and we enable by mounting the
 *   resctrl file system.
 * - Caches the per cpu CLOSid values and does the MSR write only
 *   when a task with a different CLOSid is scheduled in.
 */
static inline void intel_rdt_sched_in(void)
{
	if (static_branch_likely(&rdt_enable_key) ||
		static_branch_unlikely(&cqm_enable_key)) {
		__intel_rdt_sched_in();
	}
}

#else

static inline void intel_rdt_sched_in(void) {}

#endif
#endif
