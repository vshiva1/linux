#ifndef _RDT_H_
#define _RDT_H_

#ifdef CONFIG_CGROUP_RDT

#include <linux/cgroup.h>
#define MAX_CBM_LENGTH			32
#define IA32_L3_CBM_BASE		0xc90
#define CBM_FROM_INDEX(x)		(IA32_L3_CBM_BASE + x)

struct rdt_subsys_info {
	/* Clos Bitmap to keep track of available CLOSids.*/
	unsigned long *closmap;
};

struct intel_rdt {
	struct cgroup_subsys_state css;
	/* Class of service for the cgroup.*/
	unsigned int clos;
};

struct clos_cbm_map {
	unsigned long cache_mask;
	unsigned int clos_refcnt;
};

/*
 * Return rdt group corresponding to this container.
 */
static inline struct intel_rdt *css_rdt(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct intel_rdt, css) : NULL;
}

static inline struct intel_rdt *parent_rdt(struct intel_rdt *ir)
{
	return css_rdt(ir->css.parent);
}

#endif
#endif
