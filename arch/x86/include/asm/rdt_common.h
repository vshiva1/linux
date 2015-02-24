#ifndef _X86_RDT_H_
#define _X86_RDT_H_

#define MSR_IA32_PQR_ASSOC	0x0c8f

struct intel_pqr_state {
	raw_spinlock_t	lock;
	int		rmid;
	int		clos;
	int		cnt;
};

#endif
