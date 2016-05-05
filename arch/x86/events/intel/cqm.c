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

static unsigned int cqm_l3_scale; /* supposedly cacheline size */

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

/*
 * A cache groups is a group of perf_events with the same target (thread,
 * cgroup, CPU or system-wide). Each cache group receives has one RMID.
 * Cache groups are protected by cqm_mutex.
 */
static LIST_HEAD(cache_groups);
static DEFINE_MUTEX(cqm_mutex);

struct pkg_data **cqm_pkgs_data;

static inline bool __valid_pkg_id(u16 pkg_id)
{
	return pkg_id < topology_max_packages();
}

/* Init cqm pkg_data for @cpu 's package. */
static int pkg_data_init_cpu(int cpu)
{
	struct pkg_data *pkg_data;
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	u16 pkg_id = topology_physical_package_id(cpu);

	if (cqm_pkgs_data[pkg_id])
		return 0;


	pkg_data = kmalloc_node(sizeof(struct pkg_data),
				GFP_KERNEL, cpu_to_node(cpu));
	if (!pkg_data)
		return -ENOMEM;

	pkg_data->max_rmid = c->x86_cache_max_rmid;

	/* Does hardware has more rmids than this driver can handle? */
	if (WARN_ON(pkg_data->max_rmid >= INVALID_RMID))
		pkg_data->max_rmid = INVALID_RMID - 1;

	if (c->x86_cache_occ_scale != cqm_l3_scale) {
		pr_err("Multiple LLC scale values, disabling\n");
		kfree(pkg_data);
		return -EINVAL;
	}

	pkg_data->prmids_by_rmid = kmalloc_node(
		sizeof(struct prmid *) * (1 + pkg_data->max_rmid),
		GFP_KERNEL, cpu_to_node(cpu));

	if (!pkg_data) {
		kfree(pkg_data);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&pkg_data->free_prmids_pool);

	mutex_init(&pkg_data->pkg_data_mutex);
	raw_spin_lock_init(&pkg_data->pkg_data_lock);

	/* XXX: Chose randomly*/
	pkg_data->rotation_cpu = cpu;

	cqm_pkgs_data[pkg_id] = pkg_data;
	return 0;
}

static int intel_cqm_setup_pkg_prmid_pools(u16 pkg_id)
{
	int r;
	unsigned long flags;
	struct prmid *prmid;
	struct pkg_data *pkg_data = cqm_pkgs_data[pkg_id];

	if (!__valid_pkg_id(pkg_id))
		return -EINVAL;

	for (r = 0; r <= pkg_data->max_rmid; r++) {

		prmid = kmalloc_node(sizeof(struct prmid), GFP_KERNEL,
				     cpu_to_node(pkg_data->rotation_cpu));
		if (!prmid)
			goto fail;

		atomic64_set(&prmid->last_read_value, 0L);
		atomic64_set(&prmid->last_read_time, 0L);
		INIT_LIST_HEAD(&prmid->pool_entry);
		prmid->rmid = r;

		/* Lock needed if called during CPU hotplug. */
		raw_spin_lock_irqsave_nested(
			&pkg_data->pkg_data_lock, flags, pkg_id);
		pkg_data->prmids_by_rmid[r] = prmid;


		/* RMID 0 is special and makes the root of rmid hierarchy. */
		raw_spin_unlock_irqrestore(&pkg_data->pkg_data_lock, flags);
	}
	return 0;
fail:
	while (!list_empty(&pkg_data->free_prmids_pool)) {
		prmid = list_first_entry(&pkg_data->free_prmids_pool,
					 struct prmid, pool_entry);
		list_del(&prmid->pool_entry);
		kfree(pkg_data->prmids_by_rmid[prmid->rmid]);
		kfree(prmid);
	}
	return -ENOMEM;
}


/*
 * Determine if @a and @b measure the same set of tasks.
 *
 * If @a and @b measure the same set of tasks then we want to share a
 * single RMID.
 */
static bool __match_event(struct perf_event *a, struct perf_event *b)
{
	/* Per-cpu and task events don't mix */
	if ((a->attach_state & PERF_ATTACH_TASK) !=
	    (b->attach_state & PERF_ATTACH_TASK))
		return false;

#ifdef CONFIG_CGROUP_PERF
	if (a->cgrp != b->cgrp)
		return false;
#endif

	/* If not task event, it's a a cgroup or a non-task cpu event. */
	if (!(b->attach_state & PERF_ATTACH_TASK))
		return true;

	/*
	 * Events that target same task are placed into the same cache group.
	 */
	if (a->hw.target == b->hw.target)
		return true;

	/*
	 * Are we an inherited event?
	 */
	if (b->parent == a)
		return true;

	return false;
}

static struct pmu intel_cqm_pmu;

/*
 * Find a group and setup RMID.
 *
 * If we're part of a group, we use the group's monr.
 */
static int
intel_cqm_setup_event(struct perf_event *event, struct perf_event **group)
{
	struct perf_event *iter;


	list_for_each_entry(iter, &cache_groups, hw.cqm_event_groups_entry) {
		if (__match_event(iter, event)) {
			*group = iter;
			return 0;
		}
	}
	return 0;
}

/* Read current package immediately and remote pkg (if any) from cache. */
static void intel_cqm_event_read(struct perf_event *event)
{
}

static void intel_cqm_event_start(struct perf_event *event, int mode)
{
	if (!(event->hw.state & PERF_HES_STOPPED))
		return;

	event->hw.state &= ~PERF_HES_STOPPED;
}

static void intel_cqm_event_stop(struct perf_event *event, int mode)
{
	if (event->hw.state & PERF_HES_STOPPED)
		return;

	event->hw.state |= PERF_HES_STOPPED;
}

static int intel_cqm_event_add(struct perf_event *event, int mode)
{
	event->hw.state = PERF_HES_STOPPED;

	return 0;
}

static inline bool cqm_group_leader(struct perf_event *event)
{
	return !list_empty(&event->hw.cqm_event_groups_entry);
}

static void intel_cqm_event_destroy(struct perf_event *event)
{
	struct perf_event *group_other = NULL;

	mutex_lock(&cqm_mutex);
	/*
	 * If there's another event in this group...
	 */
	if (!list_empty(&event->hw.cqm_event_group_entry)) {
		group_other = list_first_entry(&event->hw.cqm_event_group_entry,
					       struct perf_event,
					       hw.cqm_event_group_entry);
		list_del(&event->hw.cqm_event_group_entry);
	}
	/*
	 * And we're the group leader..
	 */
	if (!cqm_group_leader(event))
		goto exit;

	/*
	 * If there was a group_other, make that leader, otherwise
	 * destroy the group and return the RMID.
	 */
	if (group_other) {
		/* Update monr reference to group head. */
		list_replace(&event->hw.cqm_event_groups_entry,
			     &group_other->hw.cqm_event_groups_entry);
		goto exit;
	}

	/*
	 * Event is the only event in cache group.
	 */

	list_del(&event->hw.cqm_event_groups_entry);

exit:
	mutex_unlock(&cqm_mutex);
}

static int intel_cqm_event_init(struct perf_event *event)
{
	struct perf_event *group = NULL;
	int ret;

	if (event->attr.type != intel_cqm_pmu.type)
		return -ENOENT;

	if (event->attr.config & ~QOS_EVENT_MASK)
		return -EINVAL;

	/* unsupported modes and filters */
	if (event->attr.exclude_user   ||
	    event->attr.exclude_kernel ||
	    event->attr.exclude_hv     ||
	    event->attr.exclude_idle   ||
	    event->attr.exclude_host   ||
	    event->attr.exclude_guest  ||
	    event->attr.sample_period) /* no sampling */
		return -EINVAL;

	INIT_LIST_HEAD(&event->hw.cqm_event_groups_entry);
	INIT_LIST_HEAD(&event->hw.cqm_event_group_entry);

	event->destroy = intel_cqm_event_destroy;

	mutex_lock(&cqm_mutex);


	/* Will also set rmid */
	ret = intel_cqm_setup_event(event, &group);
	if (ret) {
		mutex_unlock(&cqm_mutex);
		return ret;
	}

	if (group) {
		list_add_tail(&event->hw.cqm_event_group_entry,
				&group->hw.cqm_event_group_entry);
	} else {
		list_add_tail(&event->hw.cqm_event_groups_entry,
				&cache_groups);
	}

	mutex_unlock(&cqm_mutex);

	return 0;
}

EVENT_ATTR_STR(llc_occupancy, intel_cqm_llc, "event=0x01");
EVENT_ATTR_STR(llc_occupancy.per-pkg, intel_cqm_llc_pkg, "1");
EVENT_ATTR_STR(llc_occupancy.unit, intel_cqm_llc_unit, "Bytes");
EVENT_ATTR_STR(llc_occupancy.scale, intel_cqm_llc_scale, NULL);
EVENT_ATTR_STR(llc_occupancy.snapshot, intel_cqm_llc_snapshot, "1");

static struct attribute *intel_cqm_events_attr[] = {
	EVENT_PTR(intel_cqm_llc),
	EVENT_PTR(intel_cqm_llc_pkg),
	EVENT_PTR(intel_cqm_llc_unit),
	EVENT_PTR(intel_cqm_llc_scale),
	EVENT_PTR(intel_cqm_llc_snapshot),
	NULL,
};

static struct attribute_group intel_cqm_events_group = {
	.name = "events",
	.attrs = intel_cqm_events_attr,
};

PMU_FORMAT_ATTR(event, "config:0-7");
static struct attribute *intel_cqm_formats_attr[] = {
	&format_attr_event.attr,
	NULL,
};

static struct attribute_group intel_cqm_format_group = {
	.name = "format",
	.attrs = intel_cqm_formats_attr,
};

static ssize_t
max_recycle_threshold_show(
	struct device *dev, struct device_attribute *attr, char *page)
{
	ssize_t rv;

	monr_hrchy_acquire_mutexes();
	rv = snprintf(page, PAGE_SIZE - 1, "%u\n",
		      __intel_cqm_max_threshold);
	monr_hrchy_release_mutexes();

	return rv;
}

static ssize_t
max_recycle_threshold_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int bytes;
	int ret;

	ret = kstrtouint(buf, 0, &bytes);
	if (ret)
		return ret;

	/* Mutex waits for rotation logic in all packages to complete. */
	monr_hrchy_acquire_mutexes();

	__intel_cqm_max_threshold = bytes;

	monr_hrchy_release_mutexes();

	return count;
}

static DEVICE_ATTR_RW(max_recycle_threshold);

static struct attribute *intel_cqm_attrs[] = {
	&dev_attr_max_recycle_threshold.attr,
	NULL,
};

static const struct attribute_group intel_cqm_group = {
	.attrs = intel_cqm_attrs,
};

static const struct attribute_group *intel_cqm_attr_groups[] = {
	&intel_cqm_events_group,
	&intel_cqm_format_group,
	&intel_cqm_group,
	NULL,
};

static struct pmu intel_cqm_pmu = {
	.hrtimer_interval_ms = CQM_DEFAULT_ROTATION_PERIOD,
	.attr_groups	     = intel_cqm_attr_groups,
	.task_ctx_nr	     = perf_sw_context,
	.event_init	     = intel_cqm_event_init,
	.add		     = intel_cqm_event_add,
	.del		     = intel_cqm_event_stop,
	.start		     = intel_cqm_event_start,
	.stop		     = intel_cqm_event_stop,
	.read		     = intel_cqm_event_read,
};

static inline void cqm_pick_event_reader(int cpu)
{
	u16 pkg_id = topology_physical_package_id(cpu);
	/* XXX: lock, check if rotation cpu is online, maybe */
	/*
	 * Pick a reader if there isn't one already.
	 */
	if (cqm_pkgs_data[pkg_id]->rotation_cpu != -1)
		cqm_pkgs_data[pkg_id]->rotation_cpu = cpu;
}

static void intel_cqm_cpu_starting(unsigned int cpu)
{
	struct intel_pqr_state *state = &per_cpu(pqr_state, cpu);
	struct cpuinfo_x86 *c = &cpu_data(cpu);
	u16 pkg_id = topology_physical_package_id(cpu);

	state->rmid = 0;
	state->closid = 0;

	/* XXX: lock */
	/* XXX: Make sure this case is handled when hotplug happens. */
	WARN_ON(c->x86_cache_max_rmid != cqm_pkgs_data[pkg_id]->max_rmid);
	WARN_ON(c->x86_cache_occ_scale != cqm_l3_scale);
}

static void intel_cqm_cpu_exit(unsigned int cpu)
{
	/*
	 * Is @cpu a designated cqm reader?
	 */
	u16 pkg_id = topology_physical_package_id(cpu);

	if (cqm_pkgs_data[pkg_id]->rotation_cpu != cpu)
		return;
	/* XXX: do remove unused packages */
	cqm_pkgs_data[pkg_id]->rotation_cpu = cpumask_any_but(
		topology_core_cpumask(cpu), cpu);
}

static int intel_cqm_cpu_notifier(struct notifier_block *nb,
				  unsigned long action, void *hcpu)
{
	unsigned int cpu  = (unsigned long)hcpu;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DOWN_PREPARE:
		intel_cqm_cpu_exit(cpu);
		break;
	case CPU_STARTING:
		pkg_data_init_cpu(cpu);
		intel_cqm_cpu_starting(cpu);
		cqm_pick_event_reader(cpu);
		break;
	}

	return NOTIFY_OK;
}

static const struct x86_cpu_id intel_cqm_match[] = {
	{ .vendor = X86_VENDOR_INTEL, .feature = X86_FEATURE_CQM_OCCUP_LLC },
	{}
};

static int __init intel_cqm_init(void)
{
	char *str, scale[20];
	int i, cpu, ret = 0, min_max_rmid = 0;

	if (!x86_match_cpu(intel_cqm_match))
		return -ENODEV;

	cqm_l3_scale = boot_cpu_data.x86_cache_occ_scale;
	if (WARN_ON(cqm_l3_scale == 0))
		cqm_l3_scale = 1;

	cqm_pkgs_data = kmalloc(
		sizeof(struct pkg_data *) * topology_max_packages(),
		GFP_KERNEL);
	if (!cqm_pkgs_data)
		return -ENOMEM;

	for (i = 0; i < topology_max_packages(); i++)
		cqm_pkgs_data[i] = NULL;

	/*
	 * It's possible that not all resources support the same number
	 * of RMIDs. Instead of making scheduling much more complicated
	 * (where we have to match a task's RMID to a cpu that supports
	 * that many RMIDs) just find the minimum RMIDs supported across
	 * all cpus.
	 *
	 * Also, check that the scales match on all cpus.
	 */
	cpu_notifier_register_begin();

	/* XXX: assert all cpus in pkg have same nr rmids (they should). */
	for_each_online_cpu(cpu) {
		ret = pkg_data_init_cpu(cpu);
		if  (ret)
			goto error;
	}

	/* Select the minimum of the maximum rmids to use as limit for
	 * threshold. XXX: per-package threshold.
	 */
	cqm_pkg_id_for_each_online(i) {
		if (min_max_rmid < cqm_pkgs_data[i]->max_rmid)
			min_max_rmid = cqm_pkgs_data[i]->max_rmid;
		intel_cqm_setup_pkg_prmid_pools(i);
	}

	/*
	 * A reasonable upper limit on the max threshold is the number
	 * of lines tagged per RMID if all RMIDs have the same number of
	 * lines tagged in the LLC.
	 *
	 * For a 35MB LLC and 56 RMIDs, this is ~1.8% of the LLC.
	 */
	__intel_cqm_max_threshold =
		boot_cpu_data.x86_cache_size * 1024 / (min_max_rmid + 1);

	snprintf(scale, sizeof(scale), "%u", cqm_l3_scale);
	str = kstrdup(scale, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto error;
	}

	event_attr_intel_cqm_llc_scale.event_str = str;

	for_each_online_cpu(i) {
		intel_cqm_cpu_starting(i);
		cqm_pick_event_reader(i);
	}

	__perf_cpu_notifier(intel_cqm_cpu_notifier);

	ret = perf_pmu_register(&intel_cqm_pmu, "intel_cqm", -1);
	if (ret)
		goto error;

	cpu_notifier_register_done();

	pr_info("Intel CQM monitoring enabled with at least %u rmids per package.\n",
		min_max_rmid + 1);

	return ret;

error:
	pr_err("Intel CQM perf registration failed: %d\n", ret);
	cpu_notifier_register_done();

	return ret;
}

device_initcall(intel_cqm_init);
