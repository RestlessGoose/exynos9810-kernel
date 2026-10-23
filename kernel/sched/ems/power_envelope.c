// SPDX-License-Identifier: GPL-2.0
/* Exynos 9810 M3 instantaneous envelope + one shared PM-QoS owner. */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#include "ems.h"
#include "cl1_power_model.h"

#define PE_FIRST_CPU          4
#define PE_LAST_CPU           7
#define PE_CPUS               4
#define PE_INTERVAL_MS        10
#define PE_MAX_ELAPSED_MS     100
#define PE_DT_COMPAT          "samsung,exynos9810-cl1-envelope"

struct pe_config {
	u32 busy_enter_pct;
	u32 busy_exit_pct;
	u32 single_khz;
	u32 dual_khz;
	u32 multi_khz;
	u32 power_budget_mw;
	u32 shared_pct;
};

/* The 3 frequency ceilings remain the experimentally conservative OCP guard. */
static struct pe_config pe_cfg = {
	.busy_enter_pct = 25,
	.busy_exit_pct = 15,
	.single_khz = 2314000,
	.dual_khz = 2002000,
	.multi_khz = 1794000,
	.power_budget_mw = 0, /* No measured multicore OCP power threshold yet. */
	.shared_pct = 20,
};

static struct pm_qos_request pe_max_qos_req;
static struct delayed_work pe_work;
static bool pe_busy[PE_CPUS];
static u32 pe_limit_khz = 2314000;
static u64 pe_last_ns;

static void __init pe_read_dts(void)
{
	struct device_node *np;
	u32 value;

	np = of_find_compatible_node(NULL, NULL, PE_DT_COMPAT);
	if (!np)
		return;

	if (!of_property_read_u32(np, "samsung,busy-enter-pct", &value) &&
	    value >= 1 && value <= 100)
		pe_cfg.busy_enter_pct = value;
	if (!of_property_read_u32(np, "samsung,busy-exit-pct", &value) &&
	    value < 100)
		pe_cfg.busy_exit_pct = value;
	if (pe_cfg.busy_exit_pct >= pe_cfg.busy_enter_pct) {
		pe_cfg.busy_enter_pct = 25;
		pe_cfg.busy_exit_pct = 15;
	}

	if (!of_property_read_u32(np, "samsung,single-max-khz", &value) &&
	    value >= 598000 && value <= 2314000)
		pe_cfg.single_khz = value;
	if (!of_property_read_u32(np, "samsung,dual-max-khz", &value) &&
	    value >= 598000 && value <= 2002000)
		pe_cfg.dual_khz = value;
	if (!of_property_read_u32(np, "samsung,multi-max-khz", &value) &&
	    value >= 598000 && value <= 1794000)
		pe_cfg.multi_khz = value;
	/* A bad ordering must never allow an unexpectedly permissive setting. */
	if (pe_cfg.dual_khz > pe_cfg.single_khz)
		pe_cfg.dual_khz = pe_cfg.single_khz;
	if (pe_cfg.multi_khz > pe_cfg.dual_khz)
		pe_cfg.multi_khz = pe_cfg.dual_khz;

	if (!of_property_read_u32(np, "samsung,electrical-budget-mw", &value) &&
	    value <= 10000)
		pe_cfg.power_budget_mw = value;
	if (!of_property_read_u32(np, "samsung,shared-power-pct", &value) &&
	    value <= 100)
		pe_cfg.shared_pct = value;

	of_node_put(np);
}

static unsigned long pe_cpu_util(int cpu)
{
	unsigned long sum;

	if (!cpu_online(cpu))
		return 0;

	/* USS + SSE is the convention used by this EMS port; verify their scale. */
	sum = ml_cpu_util_ratio(cpu, 0) + ml_cpu_util_ratio(cpu, 1);
	return min_t(unsigned long, sum, SCHED_CAPACITY_SCALE);
}

static unsigned int pe_envelope(unsigned long *total_util,
				unsigned long *peak_util)
{
	unsigned int busy_count = 0;
	unsigned long enter = SCHED_CAPACITY_SCALE * pe_cfg.busy_enter_pct / 100;
	unsigned long exit = SCHED_CAPACITY_SCALE * pe_cfg.busy_exit_pct / 100;
	int cpu;

	*total_util = 0;
	*peak_util = 0;
	for (cpu = PE_FIRST_CPU; cpu <= PE_LAST_CPU; cpu++) {
		unsigned long util = pe_cpu_util(cpu);
		int idx = cpu - PE_FIRST_CPU;

		*total_util += util;
		*peak_util = max(*peak_util, util);
		if (!cpu_online(cpu))
			pe_busy[idx] = false;
		else if (pe_busy[idx]) {
			if (util <= exit)
				pe_busy[idx] = false;
		} else if (util >= enter) {
			pe_busy[idx] = true;
		}
		busy_count += pe_busy[idx];
	}

	if (busy_count >= 3)
		return pe_cfg.multi_khz;
	if (busy_count == 2)
		return pe_cfg.dual_khz;
	return pe_cfg.single_khz;
}

/* Optional model-based cap. Actual OCP protection is the count-based cap. */
static unsigned int pe_budget_limit(unsigned int max_khz,
				     unsigned long total_util,
				     unsigned long peak_util)
{
	int i;

	if (!pe_cfg.power_budget_mw || !total_util)
		return max_khz;

	for (i = (int)CL1_POWER_POINTS - 1; i >= 0; i--) {
		u32 freq = cl1_power_table[i].freq_khz;

		if (freq > max_khz)
			continue;
		if (cl1_estimated_power_mw(freq, total_util, peak_util,
					       pe_cfg.shared_pct) <=
		    pe_cfg.power_budget_mw)
			return freq;
	}

	return cl1_power_table[0].freq_khz;
}

static void pe_workfn(struct work_struct *work)
{
	u64 now_ns = ktime_get_ns();
	u64 elapsed_ns = now_ns - pe_last_ns;
	unsigned int dt_ms, freq, cap;
	unsigned long sum_util, peak_util;

	pe_last_ns = now_ns;
	/* Prevent a delayed worker from attributing seconds to a stale util sample. */
	dt_ms = min_t(u64, div_u64(elapsed_ns, 1000000), PE_MAX_ELAPSED_MS);
	if (!dt_ms)
		dt_ms = 1;

	cap = pe_envelope(&sum_util, &peak_util);
	freq = cpufreq_quick_get(PE_FIRST_CPU);
	boost_limiter_step(freq, sum_util, peak_util, dt_ms);
	cap = min(cap, boost_limiter_get_limit());
	cap = pe_budget_limit(cap, sum_util, peak_util);

	if (cap != pe_limit_khz) {
		pm_qos_update_request(&pe_max_qos_req, cap);
		pe_limit_khz = cap;
	}

	schedule_delayed_work(&pe_work, msecs_to_jiffies(PE_INTERVAL_MS));
}

static int __init power_envelope_init(void)
{
	pe_read_dts();
	pe_limit_khz = pe_cfg.single_khz;
	pm_qos_add_request(&pe_max_qos_req, PM_QOS_CLUSTER1_FREQ_MAX,
			   pe_limit_khz);
	pe_last_ns = ktime_get_ns();
	INIT_DELAYED_WORK(&pe_work, pe_workfn);
	schedule_delayed_work(&pe_work, msecs_to_jiffies(PE_INTERVAL_MS));
	pr_info("CL1 envelope v2: single=%u dual=%u multi=%u kHz budget=%u mW\n",
		pe_cfg.single_khz, pe_cfg.dual_khz, pe_cfg.multi_khz,
		pe_cfg.power_budget_mw);
	return 0;
}
late_initcall(power_envelope_init);
