#ifndef _RDT_H_
#define _RDT_H_

#ifdef CONFIG_INTEL_RDT

#include <linux/seq_file.h>
#include <linux/jump_label.h>

#define IA32_L3_CBM_BASE		0xc90
#define L3_CBM_FROM_INDEX(x)		(IA32_L3_CBM_BASE + x)

#define MSR_IA32_L3_QOS_CFG		0xc81

enum resource_type {
	RESOURCE_L3  = 0,
	RESOURCE_NUM = 1,
};

#define MAX_CACHE_LEAVES        4
#define MAX_CACHE_DOMAINS       64

DECLARE_PER_CPU_READ_MOSTLY(int, cpu_l3_domain);
DECLARE_PER_CPU_READ_MOSTLY(struct rdtgroup *, cpu_rdtgroup);

extern spinlock_t rdtgroup_task_lock;
extern struct static_key rdt_enable_key;
void __intel_rdt_sched_in(void *dummy);
extern bool use_rdtgroup_tasks;

extern bool cdp_enabled;

struct rdt_opts {
	bool cdp_enabled;
	bool verbose;
	bool simulate_cat_l3;
};

struct cache_domain {
	cpumask_t shared_cpu_map[MAX_CACHE_DOMAINS];
	unsigned int max_cache_domains_num;
	unsigned int level;
	unsigned int shared_cache_id[MAX_CACHE_DOMAINS];
};

extern struct cache_domain cache_domains[MAX_CACHE_LEAVES];


extern struct rdt_opts rdt_opts;

struct clos_cbm_table {
	unsigned long cbm;
	unsigned long mthrtl;
	unsigned int clos_refcnt;
};

struct clos_config {
	unsigned long **closmap;
	u32 catl3_max_closid;
	u32 mbe_max_closid;
};

struct shared_domain {
	struct cpumask cpumask;
	int l3_domain;
};

#define for_each_cache_domain(domain, start_domain, max_domain)	\
	for (domain = start_domain; domain < max_domain; domain++)

extern struct clos_config cconfig;
extern struct shared_domain *shared_domain;
extern int shared_domain_num;

extern struct rdtgroup *root_rdtgrp;
extern void rdtgroup_fork(struct task_struct *child);
extern void rdtgroup_post_fork(struct task_struct *child);

extern struct clos_cbm_table **l3_cctable;

extern unsigned int min_bitmask_len;
extern void msr_cpu_update(void *arg);
extern inline void closid_get(u32 closid, int domain);
extern void closid_put(u32 closid, int domain);
extern void closid_free(u32 closid, int domain, int level);
extern int closid_alloc(u32 *closid, int domain, bool mbe_dontcare);
extern struct mutex rdtgroup_mutex;
extern bool cat_l3_enabled;
extern unsigned int get_domain_num(int level);
extern struct shared_domain *shared_domain;
extern int shared_domain_num;
extern inline int get_dcbm_table_index(int x);
extern inline int get_icbm_table_index(int x);

extern int get_cache_leaf(int level, int cpu);

extern void cbm_update_l3_msr(void *pindex);
extern int level_to_leaf(int level);

extern void init_msrs(bool cdpenabled);
extern bool cat_enabled(int level);
extern u64 max_cbm(int level);
extern u32 max_cbm_len(int level);

extern void rdtgroup_exit(struct task_struct *tsk);

/*
 * intel_rdt_sched_in() - Writes the task's CLOSid to IA32_PQR_MSR
 *
 * Following considerations are made so that this has minimal impact
 * on scheduler hot path:
 * - This will stay as no-op unless we are running on an Intel SKU
 * which supports L3 cache allocation.
 * - When support is present and enabled, does not do any
 * IA32_PQR_MSR writes until the user starts really using the feature
 * ie creates a rdtgroup directory and assigns a cache_mask thats
 * different from the root rdtgroup's cache_mask.
 * - Caches the per cpu CLOSid values and does the MSR write only
 * when a task with a different CLOSid is scheduled in. That
 * means the task belongs to a different rdtgroup.
 * - Closids are allocated so that different rdtgroup directories
 * with same cache_mask gets the same CLOSid. This minimizes CLOSids
 * used and reduces MSR write frequency.
 */
static inline void intel_rdt_sched_in(void)
{
	/*
	 * Call the schedule in code only when RDT is enabled.
	 */
	if (static_key_false(&rdt_enable_key))
		__intel_rdt_sched_in(NULL);
}

#else

static inline void intel_rdt_sched_in(void) {}
static inline void rdtgroup_fork(struct task_struct *child) {}
static inline void rdtgroup_post_fork(struct task_struct *child) {}
static inline void rdtgroup_exit(struct task_struct *tsk) {}

#endif
#endif
