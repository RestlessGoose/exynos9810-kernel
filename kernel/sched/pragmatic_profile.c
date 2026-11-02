// SPDX-License-Identifier: GPL-2.0
/* Global immutable policies. No policy pointers, panel callbacks or DVFS here. */
#include <linux/pragmatic_profile.h>
#include <linux/pragmatic_diag.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/sched/sysctl.h>

const struct pr_profile_policy pr_profile_policies[3] = {
	[PR_OPTIMAL] = { 0, 0, 60, 100, 2 },
	[PR_POWER_SAVER] = { 4000, 1, 60, 80, 1 },
	[PR_SCREEN_OFF] = { 8000, 2, 60, 60, 1 },
};
atomic64_t pr_profile_state = ATOMIC64_INIT(PR_PROFILE_INITIAL);
static DEFINE_RAW_SPINLOCK(profile_lock);

const char *pr_profile_name(unsigned int id)
{
	static const char * const names[] = { "optimal", "powersave", "screen_off" };
	return id < ARRAY_SIZE(names) ? names[id] : "unknown";
}
const char *pr_display_name(unsigned int id)
{
	static const char * const names[] = { "normal", "alpm", "off", "unknown" };
	return id < ARRAY_SIZE(names) ? names[id] : "unknown";
}

/* profile_lock held; nonblocking publication/work submission only.
 * Actual governor adoption is demand driven; no asynchronous manager can
 * apply an obsolete snapshot. Both clusters consult the same atomic word.
 */
static void pr_profile_publish(unsigned int user, unsigned int display)
{
	u64 old = pr_profile_read(), generation = pr_profile_generation(old);
	unsigned int effective = display == PR_DISPLAY_NORMAL ? user :
		(display == PR_DISPLAY_ALPM || display == PR_DISPLAY_OFF ?
		 PR_SCREEN_OFF : PR_OPTIMAL);
	u64 next;

	if (effective != pr_effective_id(old)) {
		generation++;
		if (!(generation << 8))
			generation = 1;
		pr_diag_add(0, PRD_PROFILE_CHANGES, 1);
	}
	next = (generation << 8) | (effective << 4) | (display << 2) | user;
	atomic64_set(&pr_profile_state, next);
	pr_pelt_profile_changed(next);
}

void pr_profile_display(enum pr_display_state display)
{
	unsigned long flags;
	u64 old;

	if ((unsigned int)display > PR_DISPLAY_UNKNOWN)
		return;
	raw_spin_lock_irqsave(&profile_lock, flags);
	old = pr_profile_read();
	if (display != pr_display_id(old)) {
		pr_diag_add(0, PRD_DISPLAY_CHANGES, 1);
		pr_profile_publish(pr_user_id(old), display);
	}
	raw_spin_unlock_irqrestore(&profile_lock, flags);
}

static ssize_t profile_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", pr_profile_name(pr_user_id(pr_profile_read())));
}
static ssize_t profile_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int user;
	unsigned long flags;
	u64 old;

	if (sysfs_streq(buf, "optimal") || sysfs_streq(buf, "0"))
		user = PR_OPTIMAL;
	else if (sysfs_streq(buf, "powersave") || sysfs_streq(buf, "1"))
		user = PR_POWER_SAVER;
	else
		return -EINVAL;
	raw_spin_lock_irqsave(&profile_lock, flags);
	old = pr_profile_read();
	if (user != pr_user_id(old)) {
		pr_diag_add(0, PRD_USER_PROFILE_CHANGES, 1);
		pr_profile_publish(user, pr_display_id(old));
	}
	raw_spin_unlock_irqrestore(&profile_lock, flags);
	return count;
}
static ssize_t active_profile_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", pr_profile_name(pr_effective_id(pr_profile_read())));
}
static ssize_t display_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", pr_display_name(pr_display_id(pr_profile_read())));
}
static ssize_t available_profiles_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "optimal powersave\n");
}
static ssize_t requested_pelt_multiplier_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", pr_profile_policy(pr_profile_read())->requested_pelt);
}
static ssize_t pelt_multiplier_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", READ_ONCE(sysctl_sched_pelt_multiplier));
}
static ssize_t pelt_control_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", sched_pelt_auto_owned() ? "automatic" : "external");
}
static ssize_t auto_pelt_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", pr_pelt_auto_enabled());
}
static ssize_t auto_pelt_store(struct kobject *kobj, struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;
	bool enable;

	if (sysfs_streq(buf, "0"))
		enable = false;
	else if (sysfs_streq(buf, "1"))
		enable = true;
	else
		return -EINVAL;
	ret = pr_pelt_set_auto(enable);
	return ret ? ret : (ssize_t)count;
}
static ssize_t pelt_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return pr_pelt_status(buf, PAGE_SIZE);
}
static struct kobj_attribute auto_pelt_attr = __ATTR(auto_pelt, 0644, auto_pelt_show, auto_pelt_store);
static struct kobj_attribute profile_attr = __ATTR(profile, 0644, profile_show, profile_store);
#define PR_ATTR_RO(name) static struct kobj_attribute name##_attr = __ATTR_RO(name)
PR_ATTR_RO(active_profile);
PR_ATTR_RO(display_state);
PR_ATTR_RO(available_profiles);
PR_ATTR_RO(requested_pelt_multiplier);
PR_ATTR_RO(pelt_multiplier);
PR_ATTR_RO(pelt_control);
PR_ATTR_RO(pelt_status);
static struct attribute *profile_attrs[] = {
	&profile_attr.attr, &active_profile_attr.attr, &display_state_attr.attr,
	&available_profiles_attr.attr, &requested_pelt_multiplier_attr.attr,
	&pelt_multiplier_attr.attr, &pelt_control_attr.attr, &pelt_status_attr.attr,
	/* Keep automatic admission last for failed-init lifetime safety. */
	&auto_pelt_attr.attr, NULL,
};
static const struct attribute_group profile_group = { .attrs = profile_attrs };
static int __init pr_profile_init(void)
{
	struct kobject *kobj;
	int ret;

	if (!cpufreq_global_kobject)
		return -ENODEV;
	kobj = kobject_create_and_add("pragmatic", cpufreq_global_kobject);
	if (!kobj)
		return -ENOMEM;
	pr_pelt_init();
	ret = sysfs_create_group(kobj, &profile_group);
	if (ret) {
		pr_pelt_init_failed();
		kobject_put(kobj);
	}
	return ret;
}
/* Governor is bool: manager and sysfs live until reboot, not policy exit. */
late_initcall(pr_profile_init);
