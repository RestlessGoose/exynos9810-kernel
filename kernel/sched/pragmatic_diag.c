// SPDX-License-Identifier: GPL-2.0
/* Temporary Exynos9810 diagnostics. Storage never belongs to a policy. */
#include <linux/pragmatic_diag.h>
#include <linux/pragmatic_profile.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/percpu.h>
#include <linux/cpu.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sched/sysctl.h>
#include <linux/uaccess.h>
#include <linux/module.h>

DEFINE_STATIC_KEY_FALSE(pr_diag_key);
static DEFINE_MUTEX(control_lock);
static bool enabled;
static unsigned long epoch = 1;
static unsigned int dvfs_channel = ~0U;
struct timing { u64 count, total_ns, max_ns, bins[5]; };
struct bank {
	u64 event[PRD_EVENTS];
	struct timing time[PRD_METRICS];
};
typedef struct bank policy_banks[2];
static DEFINE_PER_CPU(policy_banks, banks);
static const char * const event_names[] = {
	"util_callbacks",
	"commit_calculations",
	"desired_changes",
	"effective_changes",
	"duplicate_pending",
	"achieved_suppressed",
	"rate_early",
	"rate_commit",
	"queued_target_updates",
	"inflight_target_updates",
	"forced_updates",
	"retries",
	"reconciliation_attempts",
	"reconciled",
	"work_submissions",
	"util_submissions",
	"cap_submissions",
	"irq_callbacks",
	"queue_accepted",
	"worker_executions",
	"worker_no_change",
	"driver_calls",
	"driver_errors",
	"driver_no_change",
	"request_mismatch",
	"absolute_error_khz_sum",
	"requested_khz_sum",
	"reported_khz_sum",
	"watchdog_runs",
	"boost_runs",
	"limits_calls",
	"qos_callbacks",
	"qos_updates",
	"pe_limit_changes",
	"acme_attempts",
	"acme_errors",
	"acpm_poll_requests",
	"acpm_nonpoll_requests",
	"acpm_sleep_requests",
	"acpm_errors",
	"acpm_deadline_events",
	"acpm_poll_loops",
	"acpm_udelay_calls",
	"acpm_usleep_calls",
	"profile_changes",
	"display_changes",
	"user_profile_changes",
	"up_immediate",
	"up_deferred",
	"deferred_armed",
	"deferred_executed",
	"deferred_canceled",
	"down_requests",
	"pelt_to_1",
	"pelt_to_2",
	"pelt_noops",
	"pelt_coalesced",
	"pelt_superseded",
	"pelt_failures",
};
static const char * const metric_names[] = {
	"worker",
	"driver",
	"outside",
	"dispatch",
	"acme",
	"acpm_poll",
	"acpm_nonpoll",
	"pelt_cutover",
};

/* All writers are bounded CPU-local IRQ-disabled sections. No DVFS locks.
 * ARM64 READ_ONCE(u64) is atomic; live snapshots are not transactional.
 */
void __pr_diag_add(unsigned int cpu, enum pr_diag_event event, u64 value)
{
	unsigned long flags;

	local_irq_save(flags);
	if (smp_load_acquire(&enabled))
		(*this_cpu_ptr(&banks))[cpu >= 4].event[event] += value;
	local_irq_restore(flags);
}

struct pr_diag_stamp __pr_diag_begin(void)
{
	struct pr_diag_stamp s = { 0, 0 };

	if (smp_load_acquire(&enabled)) {
		s.epoch = READ_ONCE(epoch);
		s.ns = ktime_get_ns();
	}
	return s;
}

bool __pr_diag_valid(struct pr_diag_stamp s)
{
	return s.epoch && pr_diag_on() && smp_load_acquire(&enabled) &&
		s.epoch == READ_ONCE(epoch);
}

unsigned long pr_diag_epoch(void)
{
	return READ_ONCE(epoch);
}

void __pr_diag_add_stamp(unsigned int cpu, enum pr_diag_event event, u64 value,
		       struct pr_diag_stamp stamp)
{
	unsigned long flags;

	if (!pr_diag_valid(stamp))
		return;
	local_irq_save(flags);
	if (smp_load_acquire(&enabled) && stamp.epoch == READ_ONCE(epoch))
		(*this_cpu_ptr(&banks))[cpu >= 4].event[event] += value;
	local_irq_restore(flags);
}

void __pr_diag_time(unsigned int cpu, enum pr_diag_metric metric,
		  struct pr_diag_stamp stamp, u64 ns)
{
	struct timing *t;
	unsigned long flags;
	unsigned int bin = ns < 100000 ? 0 : ns < 500000 ? 1 :
			   ns < 1000000 ? 2 : ns < 2000000 ? 3 : 4;

	if (!pr_diag_valid(stamp))
		return;
	local_irq_save(flags);
	if (smp_load_acquire(&enabled) && stamp.epoch == READ_ONCE(epoch)) {
		t = &(*this_cpu_ptr(&banks))[cpu >= 4].time[metric];
		t->count++;
		t->total_ns += ns;
		t->max_ns = max(t->max_ns, ns);
		t->bins[bin]++;
	}
	local_irq_restore(flags);
}

void pr_diag_set_channel(unsigned int channel)
{
	WRITE_ONCE(dvfs_channel, channel);
}

struct pr_diag_ipc __pr_diag_ipc_begin(unsigned int channel,
		const unsigned int *cmd, bool response, bool polling, bool sleep_mode)
{
	struct pr_diag_ipc d = { .stamp = { 0, 0 } };

	/* Exynos9810 CPU CAL IDs b040002/b040003, FREQ_REQ == 0.
	 * Match the registered DVFS channel before inspecting its protocol.
	 */
	if (!pr_diag_on() || channel != READ_ONCE(dvfs_channel) ||
	    !cmd || cmd[2] != 0 || (cmd[0] != 2 && cmd[0] != 3))
		return d;
	d.stamp = pr_diag_begin();
	d.cpu = cmd[0] == 2 ? 0 : 4;
	d.polling = polling && response;
	pr_diag_add_stamp(d.cpu, d.polling ? PRD_ACPM_POLL_REQUESTS :
		    PRD_ACPM_NONPOLL_REQUESTS, 1, d.stamp);
	if (d.polling && sleep_mode)
		pr_diag_add_stamp(d.cpu, PRD_ACPM_SLEEP_REQUESTS, 1, d.stamp);
	return d;
}

void __pr_diag_ipc_end(struct pr_diag_ipc *d, int ret)
{
	if (!pr_diag_valid(d->stamp))
		return;
	pr_diag_time(d->cpu, d->polling ? PRD_TIME_ACPM_POLL :
		     PRD_TIME_ACPM_NONPOLL, d->stamp, ktime_get_ns() - d->stamp.ns);
	pr_diag_add_stamp(d->cpu, PRD_ACPM_POLL_LOOPS, d->loops, d->stamp);
	pr_diag_add_stamp(d->cpu, PRD_ACPM_UDELAY_CALLS, d->delays, d->stamp);
	pr_diag_add_stamp(d->cpu, PRD_ACPM_USLEEP_CALLS, d->sleeps, d->stamp);
	if (ret)
		pr_diag_add_stamp(d->cpu, PRD_ACPM_ERRORS, 1, d->stamp);
}

static int stats_show(struct seq_file *s, void *unused)
{
	unsigned int p, cpu, e, m, b;

	mutex_lock(&control_lock);
	seq_printf(s, "enabled=%u epoch=%lu dvfs_channel=%u\n",
		   enabled, epoch, READ_ONCE(dvfs_channel));
	{
		u64 state = pr_profile_read();
		seq_printf(s, "user_profile=%s active_profile=%s display_state=%s requested_pelt=%u actual_pelt=%u\n",
			pr_profile_name(pr_user_id(state)), pr_profile_name(pr_effective_id(state)),
			pr_display_name(pr_display_id(state)), pr_profile_policy(state)->requested_pelt,
			READ_ONCE(sysctl_sched_pelt_multiplier));
	}
	{
		char status[160];

		pr_pelt_status(status, sizeof(status));
		seq_printf(s, "pelt_controller %s", status);
	}
	seq_puts(s, "units: counts kHz sums ns; elapsed is NOT CPU runtime\n"
		 "hist: <100us 100-<500us 500-<1000us 1-<2ms >=2ms\n");
	for (p = 0; p < 2; p++) {
		seq_printf(s, "policy%u\n", p * 4);
		for (e = 0; e < PRD_EVENTS; e++) {
			u64 sum = 0;
			for_each_possible_cpu(cpu)
				sum += READ_ONCE(per_cpu(banks, cpu)[p].event[e]);
			seq_printf(s, "  %s %llu\n", event_names[e], sum);
			if (e == PRD_ACPM_UDELAY_CALLS)
				seq_printf(s, "  acpm_busy_delay_ns_estimate %llu\n",
					   sum * 10000ULL);
		}
		for (m = 0; m < PRD_METRICS; m++) {
			struct timing t = { 0 };
			for_each_possible_cpu(cpu) {
				struct timing *v = &per_cpu(banks, cpu)[p].time[m];
				t.count += READ_ONCE(v->count);
				t.total_ns += READ_ONCE(v->total_ns);
				t.max_ns = max(t.max_ns, READ_ONCE(v->max_ns));
				for (b = 0; b < 5; b++)
					t.bins[b] += READ_ONCE(v->bins[b]);
			}
			seq_printf(s, "  %s count=%llu total_ns=%llu max_ns=%llu hist=",
				   metric_names[m], t.count, t.total_ns, t.max_ns);
			for (b = 0; b < 5; b++)
				seq_printf(s, "%s%llu", b ? "," : "", t.bins[b]);
			seq_putc(s, '\n');
		}
	}
	mutex_unlock(&control_lock);
	return 0;
}
static int stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, stats_show, NULL);
}
static const struct file_operations stats_fops = {
	.owner = THIS_MODULE, .open = stats_open, .read = seq_read,
	.llseek = seq_lseek, .release = single_release,
};
static int enabled_get(void *data, u64 *value)
{
	*value = READ_ONCE(enabled);
	return 0;
}
static int enabled_set(void *data, u64 value)
{
	if (value > 1)
		return -EINVAL;
	mutex_lock(&control_lock);
	if (!!value != enabled) {
		if (value) {
			/* Old sleeping operations cannot enter a new collection epoch. */
			if (!++epoch)
				epoch++;
			smp_store_release(&enabled, true);
			static_branch_enable(&pr_diag_key);
		} else {
			WRITE_ONCE(enabled, false);
			static_branch_disable(&pr_diag_key);
			/* Drain short per-CPU writers, not sleeping DVFS operations. */
			synchronize_sched();
		}
	}
	mutex_unlock(&control_lock);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(enabled_fops, enabled_get, enabled_set, "%llu\n");
static ssize_t reset_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	unsigned int cpu;
	int ret = 0;

	mutex_lock(&control_lock);
	if (enabled) {
		ret = -EBUSY;
	} else {
		for_each_possible_cpu(cpu)
			memset(&per_cpu(banks, cpu), 0, sizeof(per_cpu(banks, cpu)));
	}
	mutex_unlock(&control_lock);
	return ret ? ret : count;
}
static const struct file_operations reset_fops = {
	.owner = THIS_MODULE, .write = reset_write, .llseek = no_llseek,
};
static int __init pr_diag_init(void)
{
	struct dentry *root = debugfs_create_dir("pragmatic_diag", NULL);
	struct dentry *d;

	BUILD_BUG_ON(ARRAY_SIZE(event_names) != PRD_EVENTS);
	BUILD_BUG_ON(ARRAY_SIZE(metric_names) != PRD_METRICS);
	if (IS_ERR_OR_NULL(root))
		return 0;
	d = debugfs_create_file("enabled", 0600, root, NULL, &enabled_fops);
	if (IS_ERR_OR_NULL(d))
		goto fail;
	d = debugfs_create_file("stats", 0400, root, NULL, &stats_fops);
	if (IS_ERR_OR_NULL(d))
		goto fail;
	d = debugfs_create_file("reset", 0200, root, NULL, &reset_fops);
	if (!IS_ERR_OR_NULL(d))
		return 0;
fail:
	debugfs_remove_recursive(root);
	return 0;
}
/* PRAGMATIC is bool: storage/debugfs live until reboot, not policy exit. */
late_initcall(pr_diag_init);
