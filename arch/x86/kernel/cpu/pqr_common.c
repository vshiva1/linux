#include <asm/pqr_common.h>

/*
 * The cached intel_pqr_state is strictly per CPU and can never be
 * updated from a remote CPU. Functions that modify pqr_state
 * must ensure interruptions are properly handled.
 */
DEFINE_PER_CPU(struct intel_pqr_state, pqr_state);
