/*
 * Resource Director Technology(RDT)
 * - Cache Allocation code.
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * 2015-05-25 Written by
 *    Vikas Shivappa <vikas.shivappa@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * More information about RDT be found in the Intel (R) x86 Architecture
 * Software Developer Manual.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/cacheinfo.h>
#include <asm/pqr_common.h>
#include <asm/intel_rdt.h>
#include <asm/intel_rdt_rdtgroup.h>

/*
 * cctable maintains 1:1 mapping between CLOSid and cache bitmask.
 */
struct clos_cbm_table **l3_cctable;

/*
 * Minimum bits required in Cache bitmask.
 */
unsigned int min_bitmask_len = 1;

/*
 * Mask of CPUs for writing CBM values. We only need one CPU per-socket.
 */
cpumask_t rdt_l3_cpumask;

bool cat_l3_enabled;
bool mbe_enabled;

struct static_key __read_mostly rdt_enable_key = STATIC_KEY_INIT_FALSE;
struct clos_config cconfig;
bool cdp_enabled;

#define __DCBM_TABLE_INDEX(x) (x << 1)
#define __ICBM_TABLE_INDEX(x) ((x << 1) + 1)
#define __ICBM_MSR_INDEX(x)                    \
	L3_CBM_FROM_INDEX(__ICBM_TABLE_INDEX(x))

#define DCBM_TABLE_INDEX(x)    (x << cdp_enabled)
#define ICBM_TABLE_INDEX(x)    ((x << cdp_enabled) + cdp_enabled)

inline int get_dcbm_table_index(int x)
{
	return DCBM_TABLE_INDEX(x);
}
inline int get_icbm_table_index(int x)
{
	return ICBM_TABLE_INDEX(x);
}

struct rdt_remote_data {
	int msr;
	u64 val;
};

/*
 * cache_alloc_hsw_probe() - Have to probe for Intel haswell server CPUs
 * as it does not have CPUID enumeration support for Cache allocation.
 *
 * Probes by writing to the high 32 bits(CLOSid) of the IA32_PQR_MSR and
 * testing if the bits stick. Max CLOSids is always 4 and max cbm length
 * is always 20 on hsw server parts. The minimum cache bitmask length
 * allowed for HSW server is always 2 bits. Hardcode all of them.
 */
static inline bool cache_alloc_hsw_probe(void)
{
	u32 l, h_old, h_new, h_tmp;

	if (rdmsr_safe(MSR_IA32_PQR_ASSOC, &l, &h_old))
		return false;

	/*
	 * Default value is always 0 if feature is present.
	 */
	h_tmp = h_old ^ 0x1U;
	if (wrmsr_safe(MSR_IA32_PQR_ASSOC, l, h_tmp) ||
	    rdmsr_safe(MSR_IA32_PQR_ASSOC, &l, &h_new))
		return false;

	if (h_tmp != h_new)
		return false;

	wrmsr_safe(MSR_IA32_PQR_ASSOC, l, h_old);

	boot_cpu_data.x86_l3_max_closid = 4;
	boot_cpu_data.x86_l3_max_cbm_len = 20;
	min_bitmask_len = 2;

	return true;
}

u32 max_cbm_len(int level)
{
	switch (level) {
	case CACHE_LEVEL3:
		return boot_cpu_data.x86_l3_max_cbm_len;
	default:
		break;
	}

	return (u32)~0;
}

u64 max_cbm(int level)
{
	switch (level) {
	case CACHE_LEVEL3:
		return (1ULL << boot_cpu_data.x86_l3_max_cbm_len) - 1;
	default:
		break;
	}

	return (u64)~0;
}

static u32 hw_max_closid(int level)
{
	u32 maxid = 0;

	switch (level) {
	case CACHE_LEVEL3:
		/*
		 * MBE and L3 cat both have L3 as their domain.
		 * If CAT is present its known to have more CLOSids than MBE.
		 */
		if (cat_l3_enabled)
			maxid = boot_cpu_data.x86_l3_max_closid;
		else if (mbe_enabled)
			maxid = boot_cpu_data.x86_mbe_max_closid;

		return maxid;
	default:
		break;
	}

	WARN(1, "invalid level\n");
	return 0;
}

bool resource_allocl3_enabled(void)
{
	return cat_l3_enabled || mbe_enabled;
}

static int cbm_from_index(u32 i, int level)
{
	switch (level) {
	case CACHE_LEVEL3:
		return L3_CBM_FROM_INDEX(i);
	default:
		break;
	}

	WARN(1, "invalid level\n");
	return 0;
}

bool cat_enabled(int level)
{
	switch (level) {
	case CACHE_LEVEL3:
		return cat_l3_enabled;
	default:
		break;
	}

	return false;
}

static inline bool mbe_supported(struct cpuinfo_x86 *c)
{
	/*
	 * Only support MBE with linear throttling support.
	 */
	if (cpu_has(c, X86_FEATURE_MBE) && cpu_has(c, X86_FEATURE_MBE_LIN))
		return true;

	return false;
}

static inline bool cat_l3_supported(struct cpuinfo_x86 *c)
{
	if (cpu_has(c, X86_FEATURE_CAT_L3))
		return true;

	/*
	 * Probe for Haswell server CPUs.
	 */
	if (c->x86 == 0x6 && c->x86_model == 0x3f)
		return cache_alloc_hsw_probe();

	return false;
}

DEFINE_MUTEX(rdtgroup_mutex);

DEFINE_PER_CPU_READ_MOSTLY(int, cpu_l3_domain) = -1;
DEFINE_PER_CPU_READ_MOSTLY(int, cpu_shared_domain) = -1;
DEFINE_PER_CPU_READ_MOSTLY(struct rdtgroup *, cpu_rdtgroup) = 0;

void __intel_rdt_sched_in(void *dummy)
{
	struct intel_pqr_state *state = this_cpu_ptr(&pqr_state);
	struct rdtgroup *rdtgrp;
	int closid;
	int cpu = smp_processor_id();
	int domain;

	/* Don't write PQR register if rscctrl is not mounted. */
	if (!rdtgroup_mounted)
		return;

	/*
	 * First find rdtgroup for this cpu.
	 * If no rdtgroup is found for this cpu, find the task's rdtgroup.
	 */
	rdtgrp = per_cpu(cpu_rdtgroup, cpu);
	if (!rdtgrp) {
		rdtgrp = current->rdtgroup;

		if (!rdtgrp)
			return;
	}

	domain = per_cpu(cpu_shared_domain, cpu);
	closid = rdtgrp->resource.closid[domain];

	if (closid == state->closid)
		return;

	state->closid = closid;
	/* Don't really write PQR register in simulation mode. */
	if (unlikely(rdt_opts.simulate_cat_l3))
		return;

	wrmsr(MSR_IA32_PQR_ASSOC, state->rmid, closid);
}

/*
 * When cdp mode is enabled, refcnt is maintained in the dcache_cbm entry.
 */
inline void closid_get(u32 closid, int domain)
{
	lockdep_assert_held(&rdtgroup_mutex);

	if (resource_allocl3_enabled()) {
		int l3_domain;
		int dindex;

		l3_domain = shared_domain[domain].l3_domain;
		dindex = DCBM_TABLE_INDEX(closid);
		l3_cctable[l3_domain][dindex].clos_refcnt++;
	}
}

int closid_alloc(u32 *closid, int domain, bool mbe_dontcare)
{
	u32 maxid;
	u32 id;

	lockdep_assert_held(&rdtgroup_mutex);

	if (mbe_dontcare)
		maxid = cconfig.catl3_max_closid;
	else
		maxid = cconfig.mbe_max_closid;
	
	id = find_first_zero_bit((unsigned long *)cconfig.closmap[domain],
				 maxid);

	if (id == maxid)
		return -ENOSPC;

	set_bit(id, (unsigned long *)cconfig.closmap[domain]);
	closid_get(id, domain);
	*closid = id;

	return 0;
}

unsigned int get_domain_num(int level)
{
	if (level == CACHE_LEVEL3)
		return cpumask_weight(&rdt_l3_cpumask);
	else
		return -EINVAL;
}

int level_to_leaf(int level)
{
	switch (level) {
	case CACHE_LEVEL3:
		return 3;
	default:
		return -EINVAL;
	}
}

void closid_free(u32 closid, int domain, int level)
{
	struct clos_cbm_table **cctable;

	if (level == CACHE_LEVEL3) {
		cctable = l3_cctable;
		cctable[domain][closid].cbm = max_cbm(level);
	}

	clear_bit(closid, (unsigned long *)cconfig.closmap[domain]);
}

static void _closid_put(u32 closid, struct clos_cbm_table *cct,
			int domain, int level)
{
	lockdep_assert_held(&rdtgroup_mutex);
	if (WARN_ON(!cct->clos_refcnt))
		return;

	if (!--cct->clos_refcnt)
		closid_free(closid, domain, level);
}

void closid_put(u32 closid, int domain)
{
	struct clos_cbm_table *cct;

	if (resource_allocl3_enabled()) {
		int l3_domain = shared_domain[domain].l3_domain;

		cct = &l3_cctable[l3_domain][DCBM_TABLE_INDEX(closid)];
		_closid_put(closid, cct, l3_domain, CACHE_LEVEL3);
	}
}

void msr_cpu_update(void *arg)
{
	struct rdt_remote_data *info = arg;

	if (unlikely(rdt_opts.verbose))
		pr_info("Write %lx to msr %x on cpu%d\n",
			(unsigned long)info->val, info->msr,
			smp_processor_id());

	if (unlikely(rdt_opts.simulate_cat_l3))
		return;

	wrmsrl(info->msr, info->val);
}

static struct cpumask *rdt_cache_cpumask(int level)
{
	return &rdt_l3_cpumask;
}

/*
 * msr_update_all() - Update the msr for all packages.
 */
static inline void msr_update_all(int msr, u64 val, int level)
{
	struct rdt_remote_data info;

	info.msr = msr;
	info.val = val;
	on_each_cpu_mask(rdt_cache_cpumask(level), msr_cpu_update, &info, 1);
}

static void init_qos_msrs(int level)
{
	if (cat_enabled(level)) {
		u32 maxcbm;
		u32 i;

		maxcbm = max_cbm(level);
		for (i = 0; i < hw_max_closid(level); i++)
			msr_update_all(cbm_from_index(i, level), maxcbm, level);
	}
}

/*
 * Initialize QOS_MASK_n registers to all 1's.
 *
 * Initialize L3_QOS_CFG register to enable or disable CDP.
 */
void init_msrs(bool cdpenabled)
{
	if (cat_enabled(CACHE_LEVEL3)) {
		init_qos_msrs(CACHE_LEVEL3);
		msr_update_all(MSR_IA32_L3_QOS_CFG, cdpenabled, CACHE_LEVEL3);
	}

}

int get_cache_leaf(int level, int cpu)
{
	int index;
	struct cpu_cacheinfo *this_cpu_ci = get_cpu_cacheinfo(cpu);
	struct cacheinfo *this_leaf;
	int num_leaves = this_cpu_ci->num_leaves;

	for (index = 0; index < num_leaves; index++) {
		this_leaf = this_cpu_ci->info_list + index;
		if (this_leaf->level == level)
			return index;
	}

	return -EINVAL;
}

static struct cpumask *get_shared_cpu_map(int cpu, int level)
{
	int index;
	struct cacheinfo *leaf;
	struct cpu_cacheinfo *cpu_ci = get_cpu_cacheinfo(cpu);

	index = get_cache_leaf(level, cpu);
	if (index < 0)
		return 0;

	leaf = cpu_ci->info_list + index;

	return &leaf->shared_cpu_map;
}

inline bool rdt_cpumask_update(struct cpumask *cpumask, int cpu, int level)
{
	struct cpumask *shared_cpu_map;
	cpumask_t tmp_cpumask;

	shared_cpu_map = get_shared_cpu_map(cpu, level);
	if (!shared_cpu_map)
		return false;

	cpumask_and(&tmp_cpumask, cpumask, shared_cpu_map);
	if (cpumask_empty(&tmp_cpumask)) {
		cpumask_set_cpu(cpu, cpumask);
		return true;
	}

	return false;
}

void cbm_update_l3_msr(void *pindex)
{
	int index;
	int dindex;
	int l3_domain;
	struct clos_cbm_table *pl3_cctable;
	struct rdt_remote_data info;

	index = *(int *)pindex;
	dindex = DCBM_TABLE_INDEX(index);
	l3_domain =  per_cpu(cpu_l3_domain, smp_processor_id());
	pl3_cctable = &l3_cctable[l3_domain][dindex];
	if (pl3_cctable->clos_refcnt) {
		info.msr = L3_CBM_FROM_INDEX(dindex);
		info.val = pl3_cctable->cbm;
		msr_cpu_update(&info);
		if (cdp_enabled) {
			info.msr = __ICBM_MSR_INDEX(index);
			info.val = l3_cctable[l3_domain][dindex+1].cbm;
			msr_cpu_update(&info);
		}
	}
}

/*
 * cbm_update_msrs() - Updates all the existing IA32_L3_MASK_n MSRs
 * which are one per CLOSid on the current package.
 */
static void cbm_update_msrs(void *dummy)
{
	int maxid;
	int index;

	maxid = cconfig.catl3_max_closid;
	if (cat_l3_enabled) {
		for (index = 0; index < maxid; index++)
			cbm_update_l3_msr(&index);
	}
}

static inline void intel_rdt_cpu_start(int cpu)
{
	struct intel_pqr_state *state = &per_cpu(pqr_state, cpu);

	state->closid = 0;
	mutex_lock(&rdtgroup_mutex);
	if (rdt_cpumask_update(&rdt_l3_cpumask, cpu, CACHE_LEVEL3))
		smp_call_function_single(cpu, cbm_update_msrs, NULL, 1);
	mutex_unlock(&rdtgroup_mutex);
}

static void intel_rdt_cpu_exit(unsigned int cpu)
{
	cpumask_t tmp_cpumask;
	struct cpumask *shared_cpu_map;
	int new_cpu;
	int i;
	int l3_domain;
	int level;
	int leaf;

	mutex_lock(&rdtgroup_mutex);

	level = CACHE_LEVEL3;

	l3_domain = per_cpu(cpu_l3_domain, cpu);
	leaf = level_to_leaf(level);
	shared_cpu_map = &cache_domains[leaf].shared_cpu_map[l3_domain];

	cpumask_clear_cpu(cpu, &rdt_l3_cpumask);
	cpumask_clear_cpu(cpu, shared_cpu_map);
	if (cpumask_empty(shared_cpu_map))
		goto out;

	new_cpu = cpumask_first(shared_cpu_map);
	rdt_cpumask_update(&rdt_l3_cpumask, new_cpu, level);

out:
	mutex_unlock(&rdtgroup_mutex);
	return;

	if (cpumask_test_and_clear_cpu(cpu, &rdt_l3_cpumask)) {
		mutex_unlock(&rdtgroup_mutex);
		return;
	}

	cpumask_and(&tmp_cpumask, topology_core_cpumask(cpu), cpu_online_mask);
	cpumask_clear_cpu(cpu, &tmp_cpumask);
	i = cpumask_any(&tmp_cpumask);

	if (i < nr_cpu_ids)
		cpumask_set_cpu(i, &rdt_l3_cpumask);
	mutex_unlock(&rdtgroup_mutex);
}

static int intel_rdt_cpu_notifier(struct notifier_block *nb,
				  unsigned long action, void *hcpu)
{
	unsigned int cpu  = (unsigned long)hcpu;

	switch (action) {
	case CPU_DOWN_FAILED:
	case CPU_ONLINE:
		intel_rdt_cpu_start(cpu);
		break;
	case CPU_DOWN_PREPARE:
		intel_rdt_cpu_exit(cpu);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

/*
 * Initialize per-cpu cpu_l3_domain.
 *
 * cpu_l3_domain numbers are consequtive integer starting from 0.
 * Sets up 1:1 mapping of cpu id and cpu_l3_domain.
 */
static int __init cpu_cache_domain_init(int level)
{
	int i, j;
	int max_cpu_cache_domain = 0;
	int index;
	struct cacheinfo *leaf;
	int *domain;
	struct cpu_cacheinfo *cpu_ci;

	for_each_online_cpu(i) {
		domain = &per_cpu(cpu_l3_domain, i);
		if (*domain == -1) {
			index = get_cache_leaf(level, i);
			if (index < 0)
				return -EINVAL;

			cpu_ci = get_cpu_cacheinfo(i);
			leaf = cpu_ci->info_list + index;
			if (cpumask_empty(&leaf->shared_cpu_map)) {
				WARN(1, "no shared cpu for L2\n");
				return -EINVAL;
			}

			for_each_cpu(j, &leaf->shared_cpu_map) {
				domain = &per_cpu(cpu_l3_domain, j);
				*domain = max_cpu_cache_domain;
			}
			max_cpu_cache_domain++;
		}
	}

	return 0;
}

struct rdt_opts rdt_opts = {
	.cdp_enabled = false,
	.verbose = false,
	.simulate_cat_l3 = false,
};

static bool disable_cat_l3 __initdata;

static int __init rdt_setup(char *str)
{
	char *tok;

	while ((tok = strsep(&str, ",")) != NULL) {
		if (!*tok)
			return -EINVAL;

		if (strcmp(tok, "simulate_cat_l3") == 0) {
			pr_info("Simulate CAT L3\n");
			rdt_opts.simulate_cat_l3 = true;
		} else if (strcmp(tok, "disable_cat_l3") == 0) {
			pr_info("CAT L3 is disabled\n");
			disable_cat_l3 = true;
		} else {
			pr_info("Invalid rdt option\n");
			return -EINVAL;
		}
	}

	return 0;
}
__setup("rscctrl=", rdt_setup);

struct shared_domain *shared_domain;
int shared_domain_num;

static int shared_domain_init(void)
{
	int l3_domain_num = get_domain_num(CACHE_LEVEL3);
	int size;
	int domain;
	struct cpumask *cpumask;
	struct cpumask *shared_cpu_map;
	int cpu;

	if (resource_allocl3_enabled()) {
		shared_domain_num = l3_domain_num;
		cpumask = &rdt_l3_cpumask;
	} else
		return -EINVAL;

	size = shared_domain_num * sizeof(struct shared_domain);
	shared_domain = kzalloc(size, GFP_KERNEL);
	if (!shared_domain)
		return -EINVAL;

	domain = 0;
	for_each_cpu(cpu, cpumask) {
		if (resource_allocl3_enabled())
			shared_domain[domain].l3_domain =
					per_cpu(cpu_l3_domain, cpu);
		else
			shared_domain[domain].l3_domain = -1;

		shared_cpu_map = get_shared_cpu_map(cpu, CACHE_LEVEL3);

		cpumask_copy(&shared_domain[domain].cpumask, shared_cpu_map);

		domain++;
	}
	for_each_online_cpu(cpu) {
		if (resource_allocl3_enabled())
			per_cpu(cpu_shared_domain, cpu) =
					per_cpu(cpu_l3_domain, cpu);
	}

	return 0;
}

static int cconfig_init(int maxid)
{
	int num;
	int domain;
	unsigned long *closmap_block;
	int maxid_size;

	maxid_size = BITS_TO_LONGS(maxid);
	num = maxid_size * shared_domain_num;
	cconfig.closmap = kcalloc(maxid, sizeof(unsigned long *), GFP_KERNEL);
	if (!cconfig.closmap)
		goto out_free;

	closmap_block = kcalloc(num, sizeof(unsigned long), GFP_KERNEL);
	if (!closmap_block)
		goto out_free;

	for (domain = 0; domain < shared_domain_num; domain++)
		cconfig.closmap[domain] = (unsigned long *)closmap_block +
					  domain * maxid_size;

	cconfig.catl3_max_closid = boot_cpu_data.x86_l3_max_closid;;
	cconfig.mbe_max_closid = boot_cpu_data.x86_mbe_max_closid;

	return 0;
out_free:
	kfree(cconfig.closmap);
	kfree(closmap_block);
	return -ENOMEM;
}

static int __init cat_cache_init(int level, int maxid,
				 struct clos_cbm_table ***cctable)
{
	int domain_num;
	int domain;
	int size;
	int ret = 0;
	struct clos_cbm_table *p;

	domain_num = get_domain_num(level);
	size = domain_num * sizeof(struct clos_cbm_table *);
	*cctable = kzalloc(size, GFP_KERNEL);
	if (!*cctable) {
		ret = -ENOMEM;
		goto out;
	}

	size = maxid * domain_num * sizeof(struct clos_cbm_table);
	p = kzalloc(size, GFP_KERNEL);
	if (!p) {
		kfree(*cctable);
		ret = -ENOMEM;
		goto out;
	}
	for (domain = 0; domain < domain_num; domain++)
		(*cctable)[domain] = p + domain * maxid;

	ret = cpu_cache_domain_init(level);
	if (ret) {
		kfree(*cctable);
		kfree(p);
	}
out:
	return ret;
}
static int __init intel_rdt_late_init(void)
{
	struct cpuinfo_x86 *c = &boot_cpu_data;
	u32 maxid;
	int i;
	int ret;

	if (unlikely(disable_cat_l3))
		cat_l3_enabled = false;
	else if (cat_l3_supported(c))
		cat_l3_enabled = true;
	else if (rdt_opts.simulate_cat_l3 &&
		 get_cache_leaf(CACHE_LEVEL3, 0) >= 0)
		cat_l3_enabled = true;
	else
		cat_l3_enabled = false;

	if (mbe_supported(c))
		mbe_enabled = true;

	if (!resource_allocl3_enabled())
		return -ENODEV;

	if (rdt_opts.simulate_cat_l3) {
		boot_cpu_data.x86_l3_max_closid = 16;
		boot_cpu_data.x86_l3_max_cbm_len = 20;
	}
	for_each_online_cpu(i) {
		rdt_cpumask_update(&rdt_l3_cpumask, i, CACHE_LEVEL3);
	}

	maxid = 0;

	/*
	 * MBE and CAT_l3 domain is L3.
	 */
	if (resource_allocl3_enabled()) {
		maxid = hw_max_closid(CACHE_LEVEL3);
		ret = cat_cache_init(CACHE_LEVEL3, maxid, &l3_cctable);
		if (ret)
			goto err;
		}

	ret = shared_domain_init();
	if (ret)
		goto err;

	ret = cconfig_init(maxid);
	if (ret)
		goto err;

	cpu_notifier_register_begin();

	__hotcpu_notifier(intel_rdt_cpu_notifier, 0);

	cpu_notifier_register_done();

	rdtgroup_init();

	static_key_slow_inc(&rdt_enable_key);
	pr_info("Intel cache allocation enabled\n");
	if (cpu_has(c, X86_FEATURE_CDP_L3))
		pr_info("Intel code data prioritization detected\n");

	return 0;
err:
	cat_l3_enabled = false;
	mbe_enabled = false;

	return ret;
}

late_initcall(intel_rdt_late_init);

void rdtgroup_fork(struct task_struct *child)
{
	INIT_LIST_HEAD(&child->rg_list);
	child->rdtgroup = NULL;
}

void rdtgroup_post_fork(struct task_struct *child)
{
	if (!use_rdtgroup_tasks)
		return;

	spin_lock_irq(&rdtgroup_task_lock);
	if (list_empty(&child->rg_list)) {
		struct rdtgroup *rdtgrp = current->rdtgroup;

		list_add_tail(&child->rg_list, &rdtgrp->pset.tasks);
		child->rdtgroup = rdtgrp;
		atomic_inc(&rdtgrp->pset.refcount);
	}
	spin_unlock_irq(&rdtgroup_task_lock);
}
