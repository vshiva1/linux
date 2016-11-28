#ifndef _ASM_X86_CQM_H
#define _ASM_X86_CQM_H

#ifdef CONFIG_INTEL_RDT_M

#include <linux/perf_event.h>

/**
 * struct pkg_data - cqm per package(socket) meta data
 * @cqm_rmid_free_lru    A least recently used list of free RMIDs
 *     These RMIDs are guaranteed to have an occupancy less than the
 * threshold occupancy
 * @cqm_rmid_limbo_lru       list of currently unused but (potentially)
 *     dirty RMIDs.
 *     This list contains RMIDs that no one is currently using but that
 *     may have a occupancy value > __intel_cqm_threshold. User can change
 *     the threshold occupancy value.
 * @cqm_rmid_entry - The entry in the limbo and free lists.
 * @delayed_work - Work to reuse the RMIDs that have been freed.
 * @rmid_work_cpu - The cpu on the package on which work is scheduled.
 */
struct pkg_data {
	struct list_head	cqm_rmid_free_lru;
	struct list_head	cqm_rmid_limbo_lru;

	struct cqm_rmid_entry	*cqm_rmid_ptrs;

	struct mutex		pkg_data_mutex;
	raw_spinlock_t		pkg_data_lock;

	struct delayed_work	intel_cqm_rmid_work;
	atomic_t		reuse_scheduled;

	int			rmid_work_cpu;
};
#endif
#endif
