// SPDX-License-Identifier: GPL-2.0
/* Global built-in controller; no cpufreq policy or panel lifetime references. */
#include <linux/pragmatic_profile.h>
#include <linux/pragmatic_diag.h>
#include <linux/sched/sysctl.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/kernel.h>

#define PR_PELT_OFF_DELAY_MS 120
static DEFINE_MUTEX(pelt_control_lock);
static DEFINE_RAW_SPINLOCK(pelt_lock);
static struct workqueue_struct *pelt_wq;
static bool auto_requested, auto_enabled, applying;
static unsigned int target = 2;
static unsigned long deadline;
static int last_error;
static u64 last_duration_ns;
static struct pr_diag_stamp last_timing;
static void pr_pelt_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(pelt_work, pr_pelt_work);

static unsigned int pr_pelt_requested(u64 state)
{
	unsigned int id = pr_effective_id(state);

	return id < 3 ? pr_profile_policies[id].requested_pelt : 2;
}

/* pelt_lock held. Refresh from the atomic snapshot, including publication
 * whose notification has not yet acquired this lock. Never extend a deadline
 * for another profile requesting the same multiplier.
 */
static void pr_pelt_refresh(u64 state)
{
	unsigned int next = pr_pelt_requested(state);

	if (next == target)
		return;
	target = next;
	deadline = jiffies;
	if (auto_requested && next == 1 && pr_effective_id(state) == PR_SCREEN_OFF)
		deadline += msecs_to_jiffies(PR_PELT_OFF_DELAY_MS);
}

static void pr_pelt_queue(void)
{
	unsigned long now = jiffies;
	unsigned long delay = time_before(now, deadline) ? deadline - now : 0;

	if (pelt_wq && auto_requested)
		mod_delayed_work(pelt_wq, &pelt_work, delay);
}

void pr_pelt_profile_changed(u64 state)
{
	unsigned long flags;
	unsigned int previous;

	raw_spin_lock_irqsave(&pelt_lock, flags);
	previous = target;
	pr_pelt_refresh(state);
	if (auto_requested) {
		if (previous != target || last_error) {
			if (previous != target && !applying && delayed_work_pending(&pelt_work))
				pr_diag_add(0, PRD_PELT_SUPERSEDED, 1);
			pr_pelt_queue();
		} else {
			pr_diag_add(0, PRD_PELT_COALESCED, 1);
		}
	}
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
}

/* pelt_control_lock held; no spin/panel/rq/governor locks. Timing/counters
 * use the existing opt-in diagnostic epoch, including concurrent reset rules.
 */
static int pr_pelt_apply(enum sched_pelt_action action, unsigned int value)
{
	struct pr_diag_stamp stamp = pr_diag_begin();
	bool changed = false;
	int ret = sched_pelt_auto_control(action, value, &changed);

	if (pr_diag_valid(stamp)) {
		u64 ns = ktime_get_ns() - stamp.ns;

		if (!ret && changed) {
			unsigned long flags;

			pr_diag_time(0, PRD_TIME_PELT, stamp, ns);

			raw_spin_lock_irqsave(&pelt_lock, flags);
			last_duration_ns = ns;
			last_timing = stamp;
			raw_spin_unlock_irqrestore(&pelt_lock, flags);
		}
		if (ret)
			pr_diag_add_stamp(0, PRD_PELT_FAILURES, 1, stamp);
		else if (!changed)
			pr_diag_add_stamp(0, PRD_PELT_NOOPS, 1, stamp);
		else
			pr_diag_add_stamp(0, value == 1 ? PRD_PELT_TO_1 : PRD_PELT_TO_2, 1, stamp);
	}
	return ret;
}

static void pr_pelt_work(struct work_struct *work)
{
	unsigned long flags;
	unsigned int admitted;
	int ret;

	mutex_lock(&pelt_control_lock);
	raw_spin_lock_irqsave(&pelt_lock, flags);
	if (!auto_requested || !auto_enabled || !pelt_wq)
		goto unlock;
	/* Admission linearizes at this latest-snapshot read. A publication
	 * after it is an event DURING the admitted noncancelable transition.
	 */
	pr_pelt_refresh(pr_profile_read());
	if (time_before(jiffies, deadline)) {
		pr_pelt_queue();
		goto unlock;
	}
	admitted = target;
	applying = true;
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
	ret = pr_pelt_apply(SCHED_PELT_UPDATE, admitted);
	raw_spin_lock_irqsave(&pelt_lock, flags);
	last_error = ret;
	applying = false;
	pr_pelt_refresh(pr_profile_read());
	if (target != admitted) {
		pr_diag_add(0, PRD_PELT_SUPERSEDED, 1);
		pr_pelt_queue();
	}
unlock:
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
	mutex_unlock(&pelt_control_lock);
}

bool pr_pelt_auto_enabled(void)
{
	return READ_ONCE(auto_enabled);
}

int pr_pelt_set_auto(bool enable)
{
	unsigned long flags;
	unsigned int admitted;
	int ret = 0;

	mutex_lock(&pelt_control_lock);
	raw_spin_lock_irqsave(&pelt_lock, flags);
	if (!pelt_wq) {
		ret = enable ? (last_error ?: -ENODEV) : 0;
		goto unlock;
	}
	if (enable == auto_enabled && !last_error)
		goto unlock;
	/* Duplicate disable in manual mode must not overwrite a manual 1/4x. */
	if (!enable && !auto_enabled)
		goto unlock;
	auto_requested = enable;
	pr_pelt_refresh(pr_profile_read());
	deadline = jiffies; /* Initial enable synchronizes without debounce. */
	admitted = enable ? target : 2;
	applying = true;
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
	/* Never cancel_work_sync while holding a mutex the worker takes. A
	 * queued/running worker will serialize here and observe the final mode.
	 */
	cancel_delayed_work(&pelt_work);
	ret = pr_pelt_apply(enable ? (auto_enabled ? SCHED_PELT_UPDATE : SCHED_PELT_CLAIM) :
			    SCHED_PELT_RELEASE, admitted);
	raw_spin_lock_irqsave(&pelt_lock, flags);
	last_error = ret;
	applying = false;
	if (!ret)
		WRITE_ONCE(auto_enabled, enable);
	/* On error retain the last completed ownership state. Explicit writes
	 * or a new profile event can retry; there is no periodic retry loop.
	 */
	auto_requested = auto_enabled;
	pr_pelt_refresh(pr_profile_read());
	if (auto_enabled && target != admitted)
		pr_pelt_queue();
unlock:
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
	mutex_unlock(&pelt_control_lock);
	return ret;
}

int pr_pelt_status(char *buf, size_t size)
{
	unsigned long flags;
	const char *phase;
	unsigned int completed, requested;
	int ret;

	raw_spin_lock_irqsave(&pelt_lock, flags);
	completed = READ_ONCE(sysctl_sched_pelt_multiplier);
	requested = pr_pelt_requested(pr_profile_read());
	phase = applying ? (last_error ? "recovering" : !auto_requested ? "disabling" :
		!auto_enabled ? "enabling" : "applying") : last_error ? "error" :
		auto_enabled ? (requested != completed ? "pending" : "automatic") : "external";
	ret = scnprintf(buf, size,
		"%s ready=%u enabled=%u requested=%u completed=%u error=%d last_control_ns=%llu\n",
		phase, !!pelt_wq, auto_enabled, requested,
		completed, last_error, (unsigned long long)(pr_diag_valid(last_timing) ? last_duration_ns : 0));
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
	return ret;
}

void __init pr_pelt_init(void)
{
	struct workqueue_struct *wq;
	unsigned long flags;

	wq = alloc_ordered_workqueue("pragmatic_pelt", WQ_HIGHPRI | WQ_FREEZABLE);
	raw_spin_lock_irqsave(&pelt_lock, flags);
	pelt_wq = wq;
	last_error = wq ? 0 : -ENOMEM;
	pr_pelt_refresh(pr_profile_read());
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
}

/* Only failed sysfs initialization, after automatic ownership was released:
 * auto_pelt is the LAST created attribute, so it cannot have been writable if
 * create_group failed. Gate publishers before draining/freeing.
 */
void __init pr_pelt_init_failed(void)
{
	struct workqueue_struct *wq;
	unsigned long flags;

	raw_spin_lock_irqsave(&pelt_lock, flags);
	wq = pelt_wq;
	pelt_wq = NULL;
	raw_spin_unlock_irqrestore(&pelt_lock, flags);
	cancel_delayed_work_sync(&pelt_work);
	if (wq)
		destroy_workqueue(wq);
}
