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
#include <asm/intel_rdt.h>

/*
 * ccmap maintains 1:1 mapping between CLOSid and cache bitmask.
 */
static struct clos_cbm_map *ccmap;
static struct rdt_subsys_info rdtss_info;
static DEFINE_MUTEX(rdt_group_mutex);
struct intel_rdt rdt_root_group;

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

static int __init intel_rdt_late_init(void)
{
	struct cpuinfo_x86 *c = &boot_cpu_data;
	static struct clos_cbm_map *ccm;
	int maxid, max_cbm_len, err = 0;
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

	pr_info("Max bitmask length:%u,Max ClosIds: %u\n", max_cbm_len, maxid);
out_err:

	return err;
}

late_initcall(intel_rdt_late_init);

struct cgroup_subsys intel_rdt_cgrp_subsys = {
	.css_alloc		= intel_rdt_css_alloc,
	.css_free		= intel_rdt_css_free,
	.early_init		= 0,
};
