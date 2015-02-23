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

static int __init intel_rdt_late_init(void)
{
	struct cpuinfo_x86 *c = &boot_cpu_data;
	int maxid, max_cbm_len;

	if (!cpu_has(c, X86_FEATURE_CAT_L3))
		return -ENODEV;

	maxid = c->x86_rdt_max_closid;
	max_cbm_len = c->x86_rdt_max_cbm_len;

	pr_info("Max bitmask length:%u,Max ClosIds: %u\n", max_cbm_len, maxid);

	return 0;
}

late_initcall(intel_rdt_late_init);
