/*
 * Resource Director Technology(RDT)
 * - Cache Allocation code.
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * 2014-09-10 Written by
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
 * Software Developer Manual, volume 3, section 17.15.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/cpu.h>
#include <asm/intel_rdt.h>

/*
 * ccmap maintains 1:1 mapping between CLOSid and cache bitmask.
 */
static struct clos_cbm_map *ccmap;
static struct rdt_subsys_info rdtss_info;
static DEFINE_MUTEX(rdt_group_mutex);
struct intel_rdt rdt_root_group;

/*
 * Mask of CPUs for writing CBM values. We only need one per-socket.
 */
static cpumask_t rdt_cpumask;

#define rdt_for_each_child(pos_css, parent_ir)		\
	css_for_each_child((pos_css), &(parent_ir)->css)

static void __clos_init(unsigned int closid)
{
	struct clos_cbm_map *ccm = &ccmap[closid];

	lockdep_assert_held(&rdt_group_mutex);

	ccm->clos_refcnt = 1;
}

static int intel_rdt_alloc_closid(struct intel_rdt *ir)
{
	unsigned int id;
	unsigned int maxid;

	lockdep_assert_held(&rdt_group_mutex);

	maxid = boot_cpu_data.x86_rdt_max_closid;
	id = find_next_zero_bit(rdtss_info.closmap, maxid, 0);
	if (id == maxid)
		return -ENOSPC;

	set_bit(id, rdtss_info.closmap);
	__clos_init(id);
	ir->clos = id;

	return 0;
}

static void intel_rdt_free_closid(unsigned int clos)
{
	lockdep_assert_held(&rdt_group_mutex);

	clear_bit(clos, rdtss_info.closmap);
}

static void __clos_get(unsigned int closid)
{
	struct clos_cbm_map *ccm = &ccmap[closid];

	lockdep_assert_held(&rdt_group_mutex);

	ccm->clos_refcnt += 1;
}

static void __clos_put(unsigned int closid)
{
	struct clos_cbm_map *ccm = &ccmap[closid];

	lockdep_assert_held(&rdt_group_mutex);
	WARN_ON(!ccm->clos_refcnt);

	ccm->clos_refcnt -= 1;
	if (!ccm->clos_refcnt)
		intel_rdt_free_closid(closid);
}

static struct cgroup_subsys_state *
intel_rdt_css_alloc(struct cgroup_subsys_state *parent_css)
{
	struct intel_rdt *parent = css_rdt(parent_css);
	struct intel_rdt *ir;

	/*
	 * Cannot return failure on systems with no Cache Allocation
	 * as the cgroup_init does not handle failures gracefully.
	 */
	if (!parent)
		return &rdt_root_group.css;

	ir = kzalloc(sizeof(struct intel_rdt), GFP_KERNEL);
	if (!ir)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&rdt_group_mutex);
	ir->clos = parent->clos;
	__clos_get(ir->clos);
	mutex_unlock(&rdt_group_mutex);

	return &ir->css;
}

static void intel_rdt_css_free(struct cgroup_subsys_state *css)
{
	struct intel_rdt *ir = css_rdt(css);

	mutex_lock(&rdt_group_mutex);
	__clos_put(ir->clos);
	kfree(ir);
	mutex_unlock(&rdt_group_mutex);
}

static inline bool cbm_is_contiguous(unsigned long var)
{
	unsigned long maxcbm = MAX_CBM_LENGTH;
	unsigned long first_bit, zero_bit;

	if (!var)
		return false;

	first_bit = find_next_bit(&var, maxcbm, 0);
	zero_bit = find_next_zero_bit(&var, maxcbm, first_bit);

	if (find_next_bit(&var, maxcbm, zero_bit) < maxcbm)
		return false;

	return true;
}

static int intel_cache_alloc_cbm_read(struct seq_file *m, void *v)
{
	struct intel_rdt *ir = css_rdt(seq_css(m));

	seq_printf(m, "%08lx\n", ccmap[ir->clos].cache_mask);

	return 0;
}

static int validate_cbm(struct intel_rdt *ir, unsigned long cbmvalue)
{
	struct cgroup_subsys_state *css;
	struct intel_rdt *par, *c;
	unsigned long *cbm_tmp;
	int err = 0;

	if (!cbm_is_contiguous(cbmvalue)) {
		pr_err("bitmask should have >= 1 bit and be contiguous\n");
		err = -EINVAL;
		goto out_err;
	}

	par = parent_rdt(ir);
	cbm_tmp = &ccmap[par->clos].cache_mask;
	if (!bitmap_subset(&cbmvalue, cbm_tmp, MAX_CBM_LENGTH)) {
		err = -EINVAL;
		goto out_err;
	}

	rcu_read_lock();
	rdt_for_each_child(css, ir) {
		c = css_rdt(css);
		cbm_tmp = &ccmap[c->clos].cache_mask;
		if (!bitmap_subset(cbm_tmp, &cbmvalue, MAX_CBM_LENGTH)) {
			rcu_read_unlock();
			pr_err("Children's mask not a subset\n");
			err = -EINVAL;
			goto out_err;
		}
	}
	rcu_read_unlock();
out_err:

	return err;
}

static bool cbm_search(unsigned long cbm, int *closid)
{
	int maxid = boot_cpu_data.x86_rdt_max_closid;
	unsigned int i;

	for (i = 0; i < maxid; i++) {
		if (bitmap_equal(&cbm, &ccmap[i].cache_mask, MAX_CBM_LENGTH)) {
			*closid = i;
			return true;
		}
	}

	return false;
}

static void cbmmap_dump(void)
{
	int i;

	pr_debug("CBMMAP\n");
	for (i = 0; i < boot_cpu_data.x86_rdt_max_closid; i++) {
		pr_debug("cache_mask: 0x%x,clos_refcnt: %u\n",
		 (unsigned int)ccmap[i].cache_mask, ccmap[i].clos_refcnt);
	}
}

static void __cpu_cbm_update(void *info)
{
	unsigned int closid = *((unsigned int *)info);

	wrmsrl(CBM_FROM_INDEX(closid), ccmap[closid].cache_mask);
}

/*
 * cbm_update_all() - Update the cache bit mask for all packages.
 */
static inline void cbm_update_all(unsigned int closid)
{
	on_each_cpu_mask(&rdt_cpumask, __cpu_cbm_update, &closid, 1);
}

/*
 * cbm_update_msrs() - Updates all the existing IA32_L3_MASK_n MSRs
 * which are one per CLOSid, except IA32_L3_MASK_0 on the current package.
 * @cpu : the cpu on which the mask is updated.
 */
static inline void cbm_update_msrs(int cpu)
{
	int maxid = boot_cpu_data.x86_rdt_max_closid;
	unsigned int i;

	if (WARN_ON(cpu != smp_processor_id()))
		return;

	for (i = 1; i < maxid; i++) {
		if (ccmap[i].clos_refcnt)
			__cpu_cbm_update(&i);
	}
}

/*
 * intel_cache_alloc_cbm_write() - Validates and writes the
 * cache bit mask(cbm) to the IA32_L3_MASK_n
 * and also store the same in the ccmap.
 *
 * CLOSids are reused for cgroups which have same bitmask.
 * - This helps to use the scant CLOSids optimally.
 * - This also implies that at context switch write
 * to PQR-MSR is done only when a task with a
 * different bitmask is scheduled in.
 */
static int intel_cache_alloc_cbm_write(struct cgroup_subsys_state *css,
				 struct cftype *cft, u64 cbmvalue)
{
	u32 max_cbm = boot_cpu_data.x86_rdt_max_cbm_len;
	struct intel_rdt *ir = css_rdt(css);
	unsigned long cache_mask, max_mask;
	unsigned long *cbm_tmp;
	unsigned int closid;
	ssize_t err = 0;

	if (ir == &rdt_root_group)
		return -EPERM;
	bitmap_set(&max_mask, 0, max_cbm);

	/*
	 * Need global mutex as cbm write may allocate a closid.
	 */
	mutex_lock(&rdt_group_mutex);
	bitmap_and(&cache_mask, (unsigned long *)&cbmvalue, &max_mask, max_cbm);
	cbm_tmp = &ccmap[ir->clos].cache_mask;

	if (bitmap_equal(&cache_mask, cbm_tmp, MAX_CBM_LENGTH))
		goto out;

	err = validate_cbm(ir, cache_mask);
	if (err)
		goto out;

	/*
	 * At this point we are sure to change the cache_mask.Hence release the
	 * reference to the current CLOSid and try to get a reference for
	 * a different CLOSid.
	 */
	__clos_put(ir->clos);

	if (cbm_search(cache_mask, &closid)) {
		ir->clos = closid;
		__clos_get(closid);
	} else {
		err = intel_rdt_alloc_closid(ir);
		if (err)
			goto out;

		ccmap[ir->clos].cache_mask = cache_mask;
		cbm_update_all(ir->clos);
	}
	cbmmap_dump();
out:
	mutex_unlock(&rdt_group_mutex);

	return err;
}

static inline bool intel_rdt_update_cpumask(int cpu)
{
	int phys_id = topology_physical_package_id(cpu);
	struct cpumask *mask = &rdt_cpumask;
	int i;

	for_each_cpu(i, mask) {
		if (phys_id == topology_physical_package_id(i))
			return false;
	}
	cpumask_set_cpu(cpu, mask);

	return true;
}

/*
 * intel_rdt_cpu_start() - If a new package has come up, update all
 * the Cache bitmasks on the package.
 */
static inline void intel_rdt_cpu_start(int cpu)
{
	mutex_lock(&rdt_group_mutex);
	if (intel_rdt_update_cpumask(cpu))
		cbm_update_msrs(cpu);
	mutex_unlock(&rdt_group_mutex);
}

static void intel_rdt_cpu_exit(unsigned int cpu)
{
	int phys_id = topology_physical_package_id(cpu);
	int i;

	mutex_lock(&rdt_group_mutex);
	if (!cpumask_test_and_clear_cpu(cpu, &rdt_cpumask)) {
		mutex_unlock(&rdt_group_mutex);
		return;
	}

	for_each_online_cpu(i) {
		if (i == cpu)
			continue;

		if (phys_id == topology_physical_package_id(i)) {
			cpumask_set_cpu(i, &rdt_cpumask);
			break;
		}
	}
	mutex_unlock(&rdt_group_mutex);
}

static int intel_rdt_cpu_notifier(struct notifier_block *nb,
				  unsigned long action, void *hcpu)
{
	unsigned int cpu  = (unsigned long)hcpu;

	switch (action) {
	case CPU_STARTING:
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

static int __init intel_rdt_late_init(void)
{
	struct cpuinfo_x86 *c = &boot_cpu_data;
	int maxid, max_cbm_len, err = 0, i;
	static struct clos_cbm_map *ccm;
	size_t sizeb;

	if (!cpu_has(c, X86_FEATURE_CAT_L3)) {
		rdt_root_group.css.ss->disabled = 1;
		return -ENODEV;
	}
	maxid = c->x86_rdt_max_closid;
	max_cbm_len = c->x86_rdt_max_cbm_len;

	sizeb = BITS_TO_LONGS(maxid) * sizeof(long);
	rdtss_info.closmap = kzalloc(sizeb, GFP_KERNEL);
	if (!rdtss_info.closmap) {
		err = -ENOMEM;
		goto out_err;
	}

	sizeb = maxid * sizeof(struct clos_cbm_map);
	ccmap = kzalloc(sizeb, GFP_KERNEL);
	if (!ccmap) {
		kfree(rdtss_info.closmap);
		err = -ENOMEM;
		goto out_err;
	}

	set_bit(0, rdtss_info.closmap);
	rdt_root_group.clos = 0;
	ccm = &ccmap[0];
	bitmap_set(&ccm->cache_mask, 0, max_cbm_len);
	ccm->clos_refcnt = 1;

	cpu_notifier_register_begin();
	for_each_online_cpu(i)
		intel_rdt_update_cpumask(i);

	__hotcpu_notifier(intel_rdt_cpu_notifier, 0);

	cpu_notifier_register_done();

	pr_info("Max bitmask length:%u,Max ClosIds: %u\n", max_cbm_len, maxid);
out_err:

	return err;
}

late_initcall(intel_rdt_late_init);

static struct cftype rdt_files[] = {
	{
		.name = "cache_mask",
		.seq_show = intel_cache_alloc_cbm_read,
		.write_u64 = intel_cache_alloc_cbm_write,
		.mode = 0666,
	},
	{ }	/* terminate */
};

struct cgroup_subsys intel_rdt_cgrp_subsys = {
	.css_alloc		= intel_rdt_css_alloc,
	.css_free		= intel_rdt_css_free,
	.legacy_cftypes		= rdt_files,
	.early_init		= 0,
};
