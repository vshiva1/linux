/*
 * Resource Director Technology(RDT)
 * - User interface for Resource Alloction in RDT.
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * 2016 Written by
 *    Fenghua Yu <fenghua.yu@intel.com>
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

#include <linux/cred.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/init_task.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/magic.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/mount.h>
#include <linux/pagemap.h>
#include <linux/proc_fs.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/percpu-rwsem.h>
#include <linux/string.h>
#include <linux/sort.h>
#include <linux/pid_namespace.h>
#include <linux/idr.h>
#include <linux/vmalloc.h> /* TODO: replace with more sophisticated array */
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/cpumask.h>
#include <linux/cacheinfo.h>
#include <linux/cacheinfo.h>
#include <net/sock.h>
#include <asm/intel_rdt_rdtgroup.h>
#include <asm/intel_rdt.h>

/**
 * kernfs_root - find out the kernfs_root a kernfs_node belongs to
 * @kn: kernfs_node of interest
 *
 * Return the kernfs_root @kn belongs to.
 */
static inline struct kernfs_root *get_kernfs_root(struct kernfs_node *kn)
{
	/* if parent exists, it's always a dir; otherwise, @sd is a dir */
	if (kn->parent)
		kn = kn->parent;
	return kn->dir.root;
}

/*
 * Protects rdtgroup_idr so that IDs can be released without grabbing
 * rdtgroup_mutex.
 */
static DEFINE_SPINLOCK(rdtgroup_idr_lock);

struct percpu_rw_semaphore rdtgroup_threadgroup_rwsem;

#define MAX_CPUMASK_CHAR_IN_HEX	(NR_CPUS/4)

static struct rftype rdtgroup_root_base_files[];

#define RDTGROUP_FILE_NAME_MAX		(MAX_RDTGROUP_TYPE_NAMELEN +	\
					 MAX_RFTYPE_NAME + 2)
static char *rdtgroup_file_name(const struct rftype *rft, char *buf)
{
	strncpy(buf, rft->name, RDTGROUP_FILE_NAME_MAX);
	return buf;
}

/**
 * rdtgroup_file_mode - deduce file mode of a control file
 * @cft: the control file in question
 *
 * S_IRUGO for read, S_IWUSR for write.
 */
static umode_t rdtgroup_file_mode(const struct rftype *rft)
{
	umode_t mode = 0;

	if (rft->read_u64 || rft->read_s64 || rft->seq_show)
		mode |= S_IRUGO;

	if (rft->write_u64 || rft->write_s64 || rft->write) {
		if (rft->flags & RFTYPE_WORLD_WRITABLE)
			mode |= S_IWUGO;
		else
			mode |= S_IWUSR;
	}

	return mode;
}

/* set uid and gid of rdtgroup dirs and files to that of the creator */
static int rdtgroup_kn_set_ugid(struct kernfs_node *kn)
{
	struct iattr iattr = { .ia_valid = ATTR_UID | ATTR_GID,
			       .ia_uid = current_fsuid(),
			       .ia_gid = current_fsgid(), };

	if (uid_eq(iattr.ia_uid, GLOBAL_ROOT_UID) &&
	    gid_eq(iattr.ia_gid, GLOBAL_ROOT_GID))
		return 0;

	return kernfs_setattr(kn, &iattr);
}

struct rdtgroup *root_rdtgrp;
static int rdtgroup_add_file(struct kernfs_node *parent_kn, struct rftype *rft)
{
	char name[RDTGROUP_FILE_NAME_MAX];
	struct kernfs_node *kn;
	struct lock_class_key *key = NULL;
	int ret;

	kn = __kernfs_create_file(parent_kn, rdtgroup_file_name(rft, name),
				  rdtgroup_file_mode(rft), 0, rft->kf_ops, rft,
				  NULL, key);
	if (IS_ERR(kn))
		return PTR_ERR(kn);

	ret = rdtgroup_kn_set_ugid(kn);
	if (ret) {
		kernfs_remove(kn);
		return ret;
	}

	return 0;
}

static void rdtgroup_rm_file(struct kernfs_node *kn, const struct rftype *rft)
{
	char name[RDTGROUP_FILE_NAME_MAX];

	lockdep_assert_held(&rdtgroup_mutex);

	kernfs_remove_by_name(kn, rdtgroup_file_name(rft, name));
}

static int rdtgroup_addrm_files(struct kernfs_node *kn, struct rftype rfts[],
			      bool is_add)
{
	struct rftype *rft, *rft_end = NULL;
	int ret;

	lockdep_assert_held(&rdtgroup_mutex);

restart:
	for (rft = rfts; rft != rft_end && rft->name[0] != '\0'; rft++) {
		if (is_add) {
			ret = rdtgroup_add_file(kn, rft);
			if (ret) {
				pr_warn("%s: failed to add %s, err=%d\n",
					__func__, rft->name, ret);
				rft_end = rft;
				is_add = false;
				goto restart;
			}
		} else {
			rdtgroup_rm_file(kn, rft);
		}
	}
	return 0;
}

static enum resource_type get_kn_res_type(struct kernfs_node *kn)
{
	return RESOURCE_L3;
}

static int rdt_max_closid_show(struct seq_file *seq, void *v)
{
	struct kernfs_open_file *of = seq->private;
	enum resource_type res_type;

	res_type = get_kn_res_type(of->kn);

	switch (res_type) {
	case RESOURCE_L3:
		seq_printf(seq, "%d\n",
			boot_cpu_data.x86_l3_max_closid);
		break;
	default:
		break;
	}

	return 0;
}

static int rdt_max_cbm_len_show(struct seq_file *seq, void *v)
{
	struct kernfs_open_file *of = seq->private;
	enum resource_type res_type;

	res_type = get_kn_res_type(of->kn);
	switch (res_type) {
	case RESOURCE_L3:
		seq_printf(seq, "%d\n",
			boot_cpu_data.x86_l3_max_cbm_len);
		break;
	default:
		break;
	}

	return 0;
}

static int get_shared_domain(int domain, int level)
{
	int sd;

	for_each_cache_domain(sd, 0, shared_domain_num) {
		if (cat_l3_enabled && level == CACHE_LEVEL3) {
			if (shared_domain[sd].l3_domain == domain)
				return sd;
		}
	}

	return -1;
}
static void rdt_info_show_cat(struct seq_file *seq, int level)
{
	int domain;
	int domain_num = get_domain_num(level);
	int closid;
	u64 cbm;
	struct clos_cbm_table **cctable;
	int maxid;
	int shared_domain;
	int cnt;

	if (level == CACHE_LEVEL3)
		cctable = l3_cctable;
	else
		return;

	maxid = cconfig.max_closid;
	for (domain = 0; domain < domain_num; domain++) {
		seq_printf(seq, "domain %d:\n", domain);
		shared_domain = get_shared_domain(domain, level);
		for (closid = 0; closid < maxid; closid++) {
			int dindex, iindex;

			if (test_bit(closid,
			(unsigned long *)cconfig.closmap[shared_domain])) {
				dindex = get_dcbm_table_index(closid);
				cbm = cctable[domain][dindex].cbm;
				cnt = cctable[domain][dindex].clos_refcnt;
				seq_printf(seq, "cbm[%d]=%lx, refcnt=%d\n",
					 dindex, (unsigned long)cbm, cnt);
				if (cdp_enabled) {
					iindex = get_icbm_table_index(closid);
					cbm = cctable[domain][iindex].cbm;
					cnt =
					   cctable[domain][iindex].clos_refcnt;
					seq_printf(seq,
						   "cbm[%d]=%lx, refcnt=%d\n",
						   iindex, (unsigned long)cbm,
						   cnt);
				}
			} else {
				cbm = max_cbm(level);
				cnt = 0;
				dindex = get_dcbm_table_index(closid);
				seq_printf(seq, "cbm[%d]=%lx, refcnt=%d\n",
					 dindex, (unsigned long)cbm, cnt);
				if (cdp_enabled) {
					iindex = get_icbm_table_index(closid);
					seq_printf(seq,
						 "cbm[%d]=%lx, refcnt=%d\n",
						 iindex, (unsigned long)cbm,
						 cnt);
				}
			}
		}
	}
}

static void show_shared_domain(struct seq_file *seq)
{
	int domain;

	seq_puts(seq, "Shared domains:\n");

	for_each_cache_domain(domain, 0, shared_domain_num) {
		struct shared_domain *sd;

		sd = &shared_domain[domain];
		seq_printf(seq, "domain[%d]:", domain);
		if (cat_enabled(CACHE_LEVEL3))
			seq_printf(seq, "l3_domain=%d ", sd->l3_domain);
		seq_printf(seq, "cpumask=%*pb\n",
			   cpumask_pr_args(&sd->cpumask));
	}
}

static int rdt_info_show(struct seq_file *seq, void *v)
{
	show_shared_domain(seq);

	if (cat_l3_enabled) {
		if (rdt_opts.verbose)
			rdt_info_show_cat(seq, CACHE_LEVEL3);
	}

	seq_puts(seq, "\n");

	return 0;
}

static int res_type_to_level(enum resource_type res_type, int *level)
{
	int ret = 0;

	switch (res_type) {
	case RESOURCE_L3:
		*level = CACHE_LEVEL3;
		break;
	case RESOURCE_NUM:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int domain_to_cache_id_show(struct seq_file *seq, void *v)
{
	struct kernfs_open_file *of = seq->private;
	enum resource_type res_type;
	int domain;
	int leaf;
	int level = 0;
	int ret;

	res_type = (enum resource_type)of->kn->parent->priv;

	ret = res_type_to_level(res_type, &level);
	if (ret)
		return 0;

	leaf =	get_cache_leaf(level, 0);

	for (domain = 0; domain < get_domain_num(level); domain++) {
		unsigned int cid;

		cid = cache_domains[leaf].shared_cache_id[domain];
		seq_printf(seq, "%d:%d\n", domain, cid);
	}

	return 0;
}

static struct rftype info_files[] = {
	{
		.name = "info",
		.seq_show = rdt_info_show,
	},
	{ }	/* terminate */
};

/* rdtgroup information files for one cache resource. */
static struct rftype res_info_files[] = {
	{
		.name = "max_closid",
		.seq_show = rdt_max_closid_show,
	},
	{
		.name = "max_cbm_len",
		.seq_show = rdt_max_cbm_len_show,
	},
	{
		.name = "domain_to_cache_id",
		.seq_show = domain_to_cache_id_show,
	},
	{ }	/* terminate */
};

static int info_populate_dir(struct kernfs_node *kn)
{
	struct rftype *rfts;

	rfts = info_files;
	return rdtgroup_addrm_files(kn, rfts, true);
}

static int res_info_populate_dir(struct kernfs_node *kn)
{
	struct rftype *rfts;

	rfts = res_info_files;
	return rdtgroup_addrm_files(kn, rfts, true);
}

static int rdtgroup_populate_dir(struct kernfs_node *kn)
{
	struct rftype *rfts;

	rfts = rdtgroup_root_base_files;
	return rdtgroup_addrm_files(kn, rfts, true);
}

static struct rftype rdtgroup_partition_base_files[];
static int rdtgroup_partition_populate_dir(struct kernfs_node *kn)
{
	struct rftype *rfts;

	rfts = rdtgroup_partition_base_files;

	return rdtgroup_addrm_files(kn, rfts, true);
}

static int rdtgroup_procs_write_permission(struct task_struct *task,
					   struct kernfs_open_file *of)
{
	const struct cred *cred = current_cred();
	const struct cred *tcred = get_task_cred(task);
	int ret = 0;

	/*
	 * even if we're attaching all tasks in the thread group, we only
	 * need to check permissions on one of them.
	 */
	if (!uid_eq(cred->euid, GLOBAL_ROOT_UID) &&
	    !uid_eq(cred->euid, tcred->uid) &&
	    !uid_eq(cred->euid, tcred->suid))
		ret = -EACCES;

	put_cred(tcred);
	return ret;
}

bool use_rdtgroup_tasks;

static void init_rdtgroup_housekeeping(struct rdtgroup *rdtgrp)
{
	init_waitqueue_head(&rdtgrp->offline_waitq);
	rdtgrp->pset.self = rdtgrp;
	INIT_LIST_HEAD(&rdtgrp->pset.task_iters);
}

static LIST_HEAD(rdtgroup_lists);
static void init_rdtgroup_root(struct rdtgroup_root *root)
{
	struct rdtgroup *rdtgrp = &root->rdtgrp;

	INIT_LIST_HEAD(&root->root_list);
	INIT_LIST_HEAD(&rdtgrp->rdtgroup_list);
	list_add_tail(&rdtgrp->rdtgroup_list, &rdtgroup_lists);
	atomic_set(&root->nr_rdtgrps, 1);
	rdtgrp->root = root;
	init_rdtgroup_housekeeping(rdtgrp);
	idr_init(&root->rdtgroup_idr);
}

static DEFINE_IDR(rdtgroup_hierarchy_idr);
static int rdtgroup_init_root_id(struct rdtgroup_root *root)
{
	int id;

	lockdep_assert_held(&rdtgroup_mutex);

	id = idr_alloc_cyclic(&rdtgroup_hierarchy_idr, root, 0, 0, GFP_KERNEL);
	if (id < 0)
		return id;

	root->hierarchy_id = id;
	return 0;
}

static struct kernfs_syscall_ops rdtgroup_kf_syscall_ops;
/* IDR wrappers which synchronize using rdtgroup_idr_lock */
static int rdtgroup_idr_alloc(struct idr *idr, void *ptr, int start, int end,
			    gfp_t gfp_mask)
{
	int ret;

	idr_preload(gfp_mask);
	spin_lock_bh(&rdtgroup_idr_lock);
	ret = idr_alloc(idr, ptr, start, end, gfp_mask & ~__GFP_DIRECT_RECLAIM);
	spin_unlock_bh(&rdtgroup_idr_lock);
	idr_preload_end();
	return ret;
}

/* hierarchy ID allocation and mapping, protected by rdtgroup_mutex */
static void rdtgroup_exit_root_id(struct rdtgroup_root *root)
{
	lockdep_assert_held(&rdtgroup_mutex);

	if (root->hierarchy_id) {
		idr_remove(&rdtgroup_hierarchy_idr, root->hierarchy_id);
		root->hierarchy_id = 0;
	}
}

static struct rdtgroup *rdtgroup_kn_lock_live(struct kernfs_node *kn)
{
	struct rdtgroup *rdtgrp;

	if (kernfs_type(kn) == KERNFS_DIR)
		rdtgrp = kn->priv;
	else
		rdtgrp = kn->parent->priv;

	kernfs_break_active_protection(kn);

	mutex_lock(&rdtgroup_mutex);

	return rdtgrp;
}

static void rdtgroup_kn_unlock(struct kernfs_node *kn)
{
	mutex_unlock(&rdtgroup_mutex);

	kernfs_unbreak_active_protection(kn);
}

static char *res_info_dir_name(enum resource_type res_type, char *name)
{
	switch (res_type) {
	case RESOURCE_L3:
		strncpy(name, "l3", RDTGROUP_FILE_NAME_MAX);
		break;
	default:
		break;
	}

	return name;
}

static int create_res_info(enum resource_type res_type,
			   struct kernfs_node *parent_kn)
{
	struct kernfs_node *kn;
	char name[RDTGROUP_FILE_NAME_MAX];
	int ret;

	res_info_dir_name(res_type, name);
	kn = kernfs_create_dir(parent_kn, name, parent_kn->mode, NULL);
	if (IS_ERR(kn)) {
		ret = PTR_ERR(kn);
		goto out;
	}

	/*
	 * This extra ref will be put in kernfs_remove() and guarantees
	 * that @rdtgrp->kn is always accessible.
	 */
	kernfs_get(kn);

	ret = rdtgroup_kn_set_ugid(kn);
	if (ret)
		goto out_destroy;

	ret = res_info_populate_dir(kn);
	if (ret)
		goto out_destroy;

	kernfs_activate(kn);

	ret = 0;
	goto out;

out_destroy:
	kernfs_remove(kn);
out:
	return ret;

}

static int rdtgroup_create_info_dir(struct kernfs_node *parent_kn,
				    const char *name)
{
	struct kernfs_node *kn;
	int ret;

	if (parent_kn != root_rdtgrp->kn)
		return -EPERM;

	/* create the directory */
	kn = kernfs_create_dir(parent_kn, "info", parent_kn->mode, root_rdtgrp);
	if (IS_ERR(kn)) {
		ret = PTR_ERR(kn);
		goto out;
	}

	ret = info_populate_dir(kn);
	if (ret)
		goto out_destroy;

	if (cat_enabled(CACHE_LEVEL3))
		create_res_info(RESOURCE_L3, kn);

	/*
	 * This extra ref will be put in kernfs_remove() and guarantees
	 * that @rdtgrp->kn is always accessible.
	 */
	kernfs_get(kn);

	ret = rdtgroup_kn_set_ugid(kn);
	if (ret)
		goto out_destroy;

	kernfs_activate(kn);

	ret = 0;
	goto out;

out_destroy:
	kernfs_remove(kn);
out:
	return ret;
}

static int rdtgroup_setup_root(struct rdtgroup_root *root,
			       unsigned long ss_mask)
{
	int ret;

	root_rdtgrp = &root->rdtgrp;

	lockdep_assert_held(&rdtgroup_mutex);

	ret = rdtgroup_idr_alloc(&root->rdtgroup_idr, root_rdtgrp,
				 1, 2, GFP_KERNEL);
	if (ret < 0)
		goto out;

	root_rdtgrp->id = ret;
	root_rdtgrp->ancestor_ids[0] = ret;

	ret = rdtgroup_init_root_id(root);
	if (ret)
		goto cancel_ref;

	root->kf_root = kernfs_create_root(&rdtgroup_kf_syscall_ops,
					   KERNFS_ROOT_CREATE_DEACTIVATED,
					   root_rdtgrp);
	if (IS_ERR(root->kf_root)) {
		ret = PTR_ERR(root->kf_root);
		goto exit_root_id;
	}
	root_rdtgrp->kn = root->kf_root->kn;

	ret = rdtgroup_populate_dir(root->kf_root->kn);
	if (ret)
		goto destroy_root;

	rdtgroup_create_info_dir(root->kf_root->kn, "info_dir");

	/*
	 * Link the root rdtgroup in this hierarchy into all the css_set
	 * objects.
	 */
	WARN_ON(atomic_read(&root->nr_rdtgrps) != 1);

	kernfs_activate(root_rdtgrp->kn);
	ret = 0;
	goto out;

destroy_root:
	kernfs_destroy_root(root->kf_root);
	root->kf_root = NULL;
exit_root_id:
	rdtgroup_exit_root_id(root);
cancel_ref:
out:
	return ret;
}

#define cache_leaves(cpu)       (get_cpu_cacheinfo(cpu)->num_leaves)

struct cache_domain cache_domains[MAX_CACHE_LEAVES];

static int get_shared_cache_id(int cpu, int level)
{
	struct cpuinfo_x86 *c;
	int index_msb;
	struct cpu_cacheinfo *this_cpu_ci;
	struct cacheinfo *this_leaf;

	this_cpu_ci = get_cpu_cacheinfo(cpu);

	this_leaf = this_cpu_ci->info_list + level_to_leaf(level);
	return this_leaf->id;
	return c->apicid >> index_msb;
}

static __init void init_cache_domains(void)
{
	int cpu, domain;
	struct cpu_cacheinfo *this_cpu_ci;
	struct cacheinfo *this_leaf;
	int leaves;
	char buf[MAX_CPUMASK_CHAR_IN_HEX + 1];
	unsigned int level;

	for (leaves = 0; leaves < cache_leaves(0); leaves++) {
		for_each_online_cpu(cpu) {
			struct cpumask *mask;

			this_cpu_ci = get_cpu_cacheinfo(cpu);
			this_leaf = this_cpu_ci->info_list + leaves;
			cache_domains[leaves].level = this_leaf->level;
			mask = &this_leaf->shared_cpu_map;
			cpumap_print_to_pagebuf(false, buf, mask);
			for (domain = 0; domain < MAX_CACHE_DOMAINS; domain++) {
				if (cpumask_test_cpu(cpu,
				&cache_domains[leaves].shared_cpu_map[domain]))
					break;
			}
			if (domain == MAX_CACHE_DOMAINS) {
				domain =
				  cache_domains[leaves].max_cache_domains_num++;

				cache_domains[leaves].shared_cpu_map[domain] =
					*mask;

				level = cache_domains[leaves].level;
				cache_domains[leaves].shared_cache_id[domain] =
					get_shared_cache_id(cpu, level);
			}
		}
	}
}

static ssize_t rdtgroup_tasks_write(struct kernfs_open_file *of,
				  char *buf, size_t nbytes, loff_t off);

DEFINE_SPINLOCK(rdtgroup_task_lock);

void rdtgroup_exit(struct task_struct *tsk)
{

	spin_lock_irq(&rdtgroup_task_lock);
	if (!list_empty(&tsk->rg_list)) {
		struct rdtgroup *rdtgrp = tsk->rdtgroup;

		list_del_init(&tsk->rg_list);
		tsk->rdtgroup = NULL;
		atomic_dec(&rdtgrp->pset.refcount);
	}
	spin_unlock_irq(&rdtgroup_task_lock);
}

static struct rdtgroup *rdtgroup_kn_lock_live(struct kernfs_node *kn);
static void rdtgroup_kn_unlock(struct kernfs_node *kn);
static int rdtgroup_cpus_show(struct seq_file *s, void *v)
{
	struct kernfs_open_file *of = s->private;
	struct rdtgroup *rdtgrp;

	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	seq_printf(s, "%*pb\n", cpumask_pr_args(&rdtgrp->cpu_mask));
	rdtgroup_kn_unlock(of->kn);

	return 0;
}

static ssize_t rdtgroup_cpus_write(struct kernfs_open_file *of,
			char *buf, size_t nbytes, loff_t off)
{
	struct rdtgroup *rdtgrp;
	unsigned long bitmap[BITS_TO_LONGS(NR_CPUS)];
	struct cpumask *cpumask;
	int cpu;
	struct list_head *l;
	struct rdtgroup *r;

	if (!buf)
		return -EINVAL;

	rdtgrp = rdtgroup_kn_lock_live(of->kn);
	if (!rdtgrp)
		return -ENODEV;

	if (list_empty(&rdtgroup_lists))
		goto end;

	__bitmap_parse(buf, strlen(buf), 0, bitmap, nr_cpu_ids);

	cpumask = to_cpumask(bitmap);

	list_for_each(l, &rdtgroup_lists) {
		r = list_entry(l, struct rdtgroup, rdtgroup_list);
		if (r == rdtgrp)
			continue;

		for_each_cpu_and(cpu, &r->cpu_mask, cpumask)
			cpumask_clear_cpu(cpu, &r->cpu_mask);
	}

	cpumask_copy(&rdtgrp->cpu_mask, cpumask);
	for_each_cpu(cpu, cpumask)
		per_cpu(cpu_rdtgroup, cpu) = rdtgrp;

end:
	rdtgroup_kn_unlock(of->kn);

	return nbytes;
}

static struct rftype rdtgroup_partition_base_files[] = {
	{
		.name = "tasks",
		.seq_show = rdtgroup_pidlist_show,
		.write = rdtgroup_tasks_write,
	},
	{
		.name = "cpus",
		.write = rdtgroup_cpus_write,
		.seq_show = rdtgroup_cpus_show,
	},
	{
		.name = "schemas",
		.write = rdtgroup_schemas_write,
		.seq_show = rdtgroup_schemas_show,
	},
	{ }	/* terminate */
};

/* rdtgroup core interface files */
static struct rftype rdtgroup_root_base_files[] = {
	{
		.name = "tasks",
		.seq_show = rdtgroup_pidlist_show,
		.write = rdtgroup_tasks_write,
	},
	{
		.name = "cpus",
		.write = rdtgroup_cpus_write,
		.seq_show = rdtgroup_cpus_show,
	},
	{
		.name = "schemas",
		.write = rdtgroup_schemas_write,
		.seq_show = rdtgroup_schemas_show,
	},
	{ }	/* terminate */
};

static void *rdtgroup_idr_replace(struct idr *idr, void *ptr, int id)
{
	void *ret;

	spin_lock_bh(&rdtgroup_idr_lock);
	ret = idr_replace(idr, ptr, id);
	spin_unlock_bh(&rdtgroup_idr_lock);
	return ret;
}

static int rdtgroup_destroy_locked(struct rdtgroup *rdtgrp)
	__releases(&rdtgroup_mutex) __acquires(&rdtgroup_mutex)
{
	int shared_domain;
	int closid;

	lockdep_assert_held(&rdtgroup_mutex);

	/*
	 * Only migration can raise populated from zero and we're already
	 * holding rdtgroup_mutex.
	 */
	if (rdtgroup_is_populated(rdtgrp))
		return -EBUSY;

	/* free closid occupied by this rdtgroup. */
	for_each_cache_domain(shared_domain, 0, shared_domain_num) {
		closid = rdtgrp->resource.closid[shared_domain];
		closid_put(closid, shared_domain);
	}

	list_del_init(&rdtgrp->rdtgroup_list);

	/*
	 * Remove @rdtgrp directory along with the base files.  @rdtgrp has an
	 * extra ref on its kn.
	 */
	kernfs_remove(rdtgrp->kn);

	return 0;
}

static void rdtgroup_idr_remove(struct idr *idr, int id)
{
	spin_lock_bh(&rdtgroup_idr_lock);
	idr_remove(idr, id);
	spin_unlock_bh(&rdtgroup_idr_lock);
}


static int rdtgroup_mkdir(struct kernfs_node *parent_kn, const char *name,
			umode_t mode)
{
	struct rdtgroup *parent, *rdtgrp;
	struct rdtgroup_root *root;
	struct kernfs_node *kn;
	int level, ret;

	if (parent_kn != root_rdtgrp->kn)
		return -EPERM;

	/* Do not accept '\n' to avoid unparsable situation.
	 */
	if (strchr(name, '\n'))
		return -EINVAL;

	parent = rdtgroup_kn_lock_live(parent_kn);
	if (!parent)
		return -ENODEV;
	root = parent->root;
	level = parent->level + 1;

	/* allocate the rdtgroup and its ID, 0 is reserved for the root */
	rdtgrp = kzalloc(sizeof(*rdtgrp) +
			 sizeof(rdtgrp->ancestor_ids[0]) * (level + 1),
			 GFP_KERNEL);
	if (!rdtgrp) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	/*
	 * Temporarily set the pointer to NULL, so idr_find() won't return
	 * a half-baked rdtgroup.
	 */
	rdtgrp->id = rdtgroup_idr_alloc(&root->rdtgroup_idr, NULL, 2, 0,
					GFP_KERNEL);
	if (rdtgrp->id < 0) {
		ret = -ENOMEM;
		goto out_cancel_ref;
	}

	INIT_LIST_HEAD(&rdtgrp->pset.tasks);

	init_rdtgroup_housekeeping(rdtgrp);
	cpumask_clear(&rdtgrp->cpu_mask);

	rdtgrp->root = root;
	rdtgrp->level = level;

	if (test_bit(RDTGRP_CPUSET_CLONE_CHILDREN, &parent->flags))
		set_bit(RDTGRP_CPUSET_CLONE_CHILDREN, &rdtgrp->flags);

	/* create the directory */
	kn = kernfs_create_dir(parent->kn, name, mode, rdtgrp);
	if (IS_ERR(kn)) {
		ret = PTR_ERR(kn);
		goto out_free_id;
	}
	rdtgrp->kn = kn;

	/*
	 * This extra ref will be put in kernfs_remove() and guarantees
	 * that @rdtgrp->kn is always accessible.
	 */
	kernfs_get(kn);

	atomic_inc(&root->nr_rdtgrps);

	/*
	 * @rdtgrp is now fully operational.  If something fails after this
	 * point, it'll be released via the normal destruction path.
	 */
	rdtgroup_idr_replace(&root->rdtgroup_idr, rdtgrp, rdtgrp->id);

	ret = rdtgroup_kn_set_ugid(kn);
	if (ret)
		goto out_destroy;

	ret = rdtgroup_partition_populate_dir(kn);
	if (ret)
		goto out_destroy;

	kernfs_activate(kn);

	list_add_tail(&rdtgrp->rdtgroup_list, &rdtgroup_lists);
	/* Generate default schema for rdtgrp. */
	ret = get_default_resources(rdtgrp);
	if (ret)
		goto out_destroy;

	ret = 0;
	goto out_unlock;

out_free_id:
	rdtgroup_idr_remove(&root->rdtgroup_idr, rdtgrp->id);
out_cancel_ref:
	kfree(rdtgrp);
out_unlock:
	rdtgroup_kn_unlock(parent_kn);
	return ret;

out_destroy:
	rdtgroup_destroy_locked(rdtgrp);
	goto out_unlock;
}

static int rdtgroup_rmdir(struct kernfs_node *kn)
{
	struct rdtgroup *rdtgrp;
	int cpu;
	int ret = 0;

	rdtgrp = rdtgroup_kn_lock_live(kn);
	if (!rdtgrp)
		return -ENODEV;

	if (!list_empty(&rdtgrp->pset.tasks)) {
		ret = -EBUSY;
		goto out;
	}

	for_each_cpu(cpu, &rdtgrp->cpu_mask)
		per_cpu(cpu_rdtgroup, cpu) = 0;

	ret = rdtgroup_destroy_locked(rdtgrp);

out:
	rdtgroup_kn_unlock(kn);
	return ret;
}

static int
rdtgroup_move_task_all(struct rdtgroup *src_rdtgrp, struct rdtgroup *dst_rdtgrp)
{
	struct list_head *tasks;

	tasks = &src_rdtgrp->pset.tasks;
	while (!list_empty(tasks)) {
		struct task_struct *tsk;
		struct list_head *pos;
		pid_t pid;
		int ret;

		pos = tasks->next;
		tsk = list_entry(pos, struct task_struct, rg_list);
		pid = tsk->pid;
		ret = rdtgroup_move_task(pid, dst_rdtgrp, false, NULL);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Remove all of subdirectories under root.
 */
static int rmdir_all_sub(void)
{
	struct rdtgroup *rdtgrp;
	int cpu;
	int ret = 0;
	struct list_head *l;

	while (!list_is_last(&root_rdtgrp->rdtgroup_list, &rdtgroup_lists)) {
		l = rdtgroup_lists.next;
		if (l == &root_rdtgrp->rdtgroup_list)
			l = l->next;

		rdtgrp = list_entry(l, struct rdtgroup, rdtgroup_list);
		if (rdtgrp == root_rdtgrp)
			continue;

		rdtgroup_move_task_all(rdtgrp, root_rdtgrp);

		for_each_cpu(cpu, &rdtgrp->cpu_mask)
			per_cpu(cpu_rdtgroup, cpu) = 0;

		ret = rdtgroup_destroy_locked(rdtgrp);
		if (ret)
			goto out;
	}

out:
	return ret;
}

/*
 * The default hierarchy.
 */
struct rdtgroup_root rdtgrp_dfl_root;
EXPORT_SYMBOL_GPL(rdtgrp_dfl_root);

static int parse_rdtgroupfs_options(char *data)
{
	char *token, *o = data;
	int nr_opts = 0;

	while ((token = strsep(&o, ",")) != NULL) {
		nr_opts++;

		if (!*token)
			return -EINVAL;
		if (!strcmp(token, "cdp")) {
			/* Enable CDP */
			rdt_opts.cdp_enabled = true;
			continue;
		}
		if (!strcmp(token, "verbose")) {
			rdt_opts.verbose = true;
			continue;
		}
	}

	return 0;
}

static void release_root_closid(void)
{
	int domain;
	int closid;

	if (!root_rdtgrp->resource.valid)
		return;

	for_each_cache_domain(domain, 0, shared_domain_num) {
		/* Put closid in root rdtgrp's domain if valid. */
		closid = root_rdtgrp->resource.closid[domain];
		closid_put(closid, domain);
	}
}

static struct kernfs_syscall_ops rdtgroup_kf_syscall_ops = {
	.mkdir			= rdtgroup_mkdir,
	.rmdir			= rdtgroup_rmdir,
};

static void setup_task_rg_lists(struct rdtgroup *rdtgrp, bool enable)
{
	struct task_struct *p, *g;

	spin_lock_irq(&rdtgroup_task_lock);
	if (enable)
		INIT_LIST_HEAD(&rdtgrp->pset.tasks);
	use_rdtgroup_tasks = enable;

	/*
	 * We need tasklist_lock because RCU is not safe against
	 * while_each_thread(). Besides, a forking task that has passed
	 * rdtgroup_post_fork() without seeing use_task_css_set_links = 1
	 * is not guaranteed to have its child immediately visible in the
	 * tasklist if we walk through it with RCU.
	 */
	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		WARN_ON_ONCE(enable ? !list_empty(&p->rg_list) :
			     list_empty(&p->rg_list));

		/*
		 * We should check if the process is exiting, otherwise
		 * it will race with rdtgroup_exit() in that the list
		 * entry won't be deleted though the process has exited.
		 * Do it while holding siglock so that we don't end up
		 * racing against rdtgroup_exit().
		 */
		spin_lock_irq(&p->sighand->siglock);
		if (!(p->flags & PF_EXITING)) {
			if (enable) {
				list_add_tail(&p->rg_list, &rdtgrp->pset.tasks);
				p->rdtgroup = rdtgrp;
				atomic_inc(&rdtgrp->pset.refcount);
			} else {
				list_del_init(&p->rg_list);
				p->rdtgroup = NULL;
				atomic_dec(&rdtgrp->pset.refcount);
			}
		}
		spin_unlock_irq(&p->sighand->siglock);
	} while_each_thread(g, p);
	read_unlock(&tasklist_lock);
	spin_unlock_irq(&rdtgroup_task_lock);
}

/*
 * The default hierarchy always exists but is hidden until mounted for the
 * first time.  This is for backward compatibility.
 */
static bool rdtgrp_dfl_root_visible;

bool rdtgroup_mounted;

static struct dentry *rdt_mount(struct file_system_type *fs_type,
			 int flags, const char *unused_dev_name,
			 void *data)
{
	struct super_block *pinned_sb = NULL;
	struct rdtgroup_root *root;
	struct dentry *dentry;
	int ret;
	bool new_sb;

	/*
	 * The first time anyone tries to mount a rdtgroup, enable the list
	 * linking tasks and fix up all existing tasks.
	 */
	if (rdtgroup_mounted)
		return ERR_PTR(-EBUSY);

	rdt_opts.cdp_enabled = false;
	rdt_opts.verbose = false;
	cdp_enabled = false;

	ret = parse_rdtgroupfs_options(data);
	if (ret)
		goto out_mount;

	if (rdt_opts.cdp_enabled) {
		cdp_enabled = true;
		cconfig.max_closid >>= cdp_enabled;
		pr_info("CDP is enabled\n");
	}

	init_msrs(cdp_enabled);

	rdtgrp_dfl_root_visible = true;
	root = &rdtgrp_dfl_root;

	ret = get_default_resources(&root->rdtgrp);
	if (ret)
		return ERR_PTR(-ENOSPC);

out_mount:
	dentry = kernfs_mount(fs_type, flags, root->kf_root,
			      RDTGROUP_SUPER_MAGIC,
			      &new_sb);
	if (IS_ERR(dentry) || !new_sb)
		goto out_unlock;

	/*
	 * If @pinned_sb, we're reusing an existing root and holding an
	 * extra ref on its sb.  Mount is complete.  Put the extra ref.
	 */
	if (pinned_sb) {
		WARN_ON(new_sb);
		deactivate_super(pinned_sb);
	}

	setup_task_rg_lists(&root->rdtgrp, true);

	cpumask_clear(&root->rdtgrp.cpu_mask);
	rdtgroup_mounted = true;

	return dentry;

out_unlock:
	return ERR_PTR(ret);
}

static void rdt_kill_sb(struct super_block *sb)
{
	int ret;

	mutex_lock(&rdtgroup_mutex);

	ret = rmdir_all_sub();
	if (ret)
		goto out_unlock;

	setup_task_rg_lists(root_rdtgrp, false);
	release_root_closid();
	root_rdtgrp->resource.valid = false;

	/* Restore max_closid to original value. */
	cconfig.max_closid <<= cdp_enabled;

	kernfs_kill_sb(sb);
	rdtgroup_mounted = false;
out_unlock:

	mutex_unlock(&rdtgroup_mutex);
}

static struct file_system_type rdt_fs_type = {
	.name = "rscctrl",
	.mount = rdt_mount,
	.kill_sb = rdt_kill_sb,
};

static ssize_t rdtgroup_file_write(struct kernfs_open_file *of, char *buf,
				 size_t nbytes, loff_t off)
{
	struct rftype *rft = of->kn->priv;

	if (rft->write)
		return rft->write(of, buf, nbytes, off);

	return -EINVAL;
}

static void *rdtgroup_seqfile_start(struct seq_file *seq, loff_t *ppos)
{
	return seq_rft(seq)->seq_start(seq, ppos);
}

static void *rdtgroup_seqfile_next(struct seq_file *seq, void *v, loff_t *ppos)
{
	return seq_rft(seq)->seq_next(seq, v, ppos);
}

static void rdtgroup_seqfile_stop(struct seq_file *seq, void *v)
{
	seq_rft(seq)->seq_stop(seq, v);
}

static int rdtgroup_seqfile_show(struct seq_file *m, void *arg)
{
	struct rftype *rft = seq_rft(m);

	if (rft->seq_show)
		return rft->seq_show(m, arg);
	return 0;
}

static struct kernfs_ops rdtgroup_kf_ops = {
	.atomic_write_len	= PAGE_SIZE,
	.write			= rdtgroup_file_write,
	.seq_start		= rdtgroup_seqfile_start,
	.seq_next		= rdtgroup_seqfile_next,
	.seq_stop		= rdtgroup_seqfile_stop,
	.seq_show		= rdtgroup_seqfile_show,
};

static struct kernfs_ops rdtgroup_kf_single_ops = {
	.atomic_write_len	= PAGE_SIZE,
	.write			= rdtgroup_file_write,
	.seq_show		= rdtgroup_seqfile_show,
};

static void rdtgroup_exit_rftypes(struct rftype *rfts)
{
	struct rftype *rft;

	for (rft = rfts; rft->name[0] != '\0'; rft++) {
		/* free copy for custom atomic_write_len, see init_cftypes() */
		if (rft->max_write_len && rft->max_write_len != PAGE_SIZE)
			kfree(rft->kf_ops);
		rft->kf_ops = NULL;

		/* revert flags set by rdtgroup core while adding @cfts */
		rft->flags &= ~(__RFTYPE_ONLY_ON_DFL | __RFTYPE_NOT_ON_DFL);
	}
}

static int rdtgroup_init_rftypes(struct rftype *rfts)
{
	struct rftype *rft;

	for (rft = rfts; rft->name[0] != '\0'; rft++) {
		struct kernfs_ops *kf_ops;

		if (rft->seq_start)
			kf_ops = &rdtgroup_kf_ops;
		else
			kf_ops = &rdtgroup_kf_single_ops;

		/*
		 * Ugh... if @cft wants a custom max_write_len, we need to
		 * make a copy of kf_ops to set its atomic_write_len.
		 */
		if (rft->max_write_len && rft->max_write_len != PAGE_SIZE) {
			kf_ops = kmemdup(kf_ops, sizeof(*kf_ops), GFP_KERNEL);
			if (!kf_ops) {
				rdtgroup_exit_rftypes(rfts);
				return -ENOMEM;
			}
			kf_ops->atomic_write_len = rft->max_write_len;
		}

		rft->kf_ops = kf_ops;
	}

	return 0;
}

static struct list_head rdtgroups;

struct rdtgroup_root rdtgrp_dfl_root;
/*
 * rdtgroup_init - rdtgroup initialization
 *
 * Register rdtgroup filesystem, and initialize any subsystems that didn't
 * request early init.
 */
int __init rdtgroup_init(void)
{
	WARN_ON(percpu_init_rwsem(&rdtgroup_threadgroup_rwsem));
	WARN_ON(rdtgroup_init_rftypes(rdtgroup_root_base_files));

	WARN_ON(rdtgroup_init_rftypes(res_info_files));
	WARN_ON(rdtgroup_init_rftypes(info_files));

	WARN_ON(rdtgroup_init_rftypes(rdtgroup_partition_base_files));
	mutex_lock(&rdtgroup_mutex);

	init_rdtgroup_root(&rdtgrp_dfl_root);
	WARN_ON(rdtgroup_setup_root(&rdtgrp_dfl_root, 0));

	mutex_unlock(&rdtgroup_mutex);

	WARN_ON(sysfs_create_mount_point(fs_kobj, "rscctrl"));
	WARN_ON(register_filesystem(&rdt_fs_type));
	init_cache_domains();

	INIT_LIST_HEAD(&rdtgroups);

	return 0;
}
