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
