// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 9810 CL1 Power Envelope
 *
 * Electrical/core-count envelope for the M3 cluster.
 *
 * This file is the sole owner of the CL1 max-frequency PM-QoS request.
 * It combines:
 *
 *   1. Instantaneous core-count power envelope
 *   2. Long-term boost/thermal limiter suggestion
 *
 * effective_limit = min(power_envelope, boost_limiter)
 *
 * The envelope itself deliberately contains no thermal history.
 * Time/energy residency is handled by boost_limiter.c.
 */

#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#include "ems.h"

/*
 * CL1 = Exynos M3 CPUs 4-7.
 */
#define PE_FIRST_CPU			4
#define PE_LAST_CPU			7
#define PE_CPU_COUNT			(PE_LAST_CPU - PE_FIRST_CPU + 1)

/*
 * Poll quickly enough to react to additional M3 activity without putting
 * work directly in the scheduler hot path.
 */
#define PE_INTERVAL_MS			10

/*
 * Busy-core detection.
 *
 * 25% was the original threshold. Add a lower exit threshold so a CPU
 * hovering around 25% does not repeatedly enter/leave the busy set.
 */
#define PE_BUSY_ENTER_PCT		25
#define PE_BUSY_EXIT_PCT		15

/*
 * Conservative electrical envelope.
 *
 * These are deliberately below the frequencies where the 9810 has shown
 * undesirable OCP behaviour with several M3 cores active.
 */
#define PE_FREQ_SINGLE			2314000
#define PE_FREQ_DUAL			2002000
#define PE_FREQ_MULTI			1794000

/*
 * boost_limiter.c publishes a cap suggestion only.
 * Power Envelope owns the actual PM-QoS request.
 */
extern unsigned int boost_limiter_get_limit(void);

static struct pm_qos_request pe_max_qos_req;
static struct delayed_work pe_work;

static bool pe_busy_state[PE_CPU_COUNT];
static unsigned int pe_applied_limit = PE_FREQ_SINGLE;

/*
 * Return CPU activity normalized to 0..1024.
 *
 * EMS tracks USS and SSE independently. Using each utilization ratio and
 * combining them gives us a better approximation of actual CPU occupancy
 * than using only one scheduler class.
 */
static unsigned long pe_cpu_activity(int cpu)
{
	unsigned long uss;
	unsigned long sse;
	unsigned long activity;

	if (!cpu_online(cpu))
		return 0;

	uss = ml_cpu_util_ratio(cpu, 0);
	sse = ml_cpu_util_ratio(cpu, 1);

	activity = uss + sse;

	return min_t(unsigned long, activity, SCHED_CAPACITY_SCALE);
}

static bool pe_cpu_is_busy(int cpu)
{
	unsigned long activity;
	unsigned long enter;
	unsigned long exit;
	int idx;

	idx = cpu - PE_FIRST_CPU;

	if (!cpu_online(cpu)) {
		pe_busy_state[idx] = false;
		return false;
	}

	activity = pe_cpu_activity(cpu);

	enter = SCHED_CAPACITY_SCALE * PE_BUSY_ENTER_PCT / 100;
	exit = SCHED_CAPACITY_SCALE * PE_BUSY_EXIT_PCT / 100;

	if (pe_busy_state[idx]) {
		if (activity <= exit)
			pe_busy_state[idx] = false;
	} else {
		if (activity >= enter)
			pe_busy_state[idx] = true;
	}

	return pe_busy_state[idx];
}

static unsigned int pe_get_envelope_limit(void)
{
	int cpu;
	int busy = 0;

	for (cpu = PE_FIRST_CPU; cpu <= PE_LAST_CPU; cpu++) {
		if (pe_cpu_is_busy(cpu))
			busy++;
	}

	if (busy >= 3)
		return PE_FREQ_MULTI;

	if (busy == 2)
		return PE_FREQ_DUAL;

	/*
	 * Zero or one busy M3:
	 *
	 * Keep the cluster capped to the known-safe single-core ceiling.
	 * We intentionally do not expose 2496+ MHz here.
	 */
	return PE_FREQ_SINGLE;
}

static void pe_update_limit(void)
{
	unsigned int envelope_limit;
	unsigned int boost_limit;
	unsigned int effective_limit;

	envelope_limit = pe_get_envelope_limit();
	boost_limit = boost_limiter_get_limit();

	/*
	 * Electrical safety and accumulated thermal budget are independent
	 * constraints. The stricter one wins.
	 */
	effective_limit = min(envelope_limit, boost_limit);

	if (effective_limit == READ_ONCE(pe_applied_limit))
		return;

	pm_qos_update_request(&pe_max_qos_req, effective_limit);
	WRITE_ONCE(pe_applied_limit, effective_limit);
}

static void pe_workfn(struct work_struct *work)
{
	pe_update_limit();

	schedule_delayed_work(&pe_work,
			      msecs_to_jiffies(PE_INTERVAL_MS));
}

static int __init power_envelope_init(void)
{
	int i;

	for (i = 0; i < PE_CPU_COUNT; i++)
		pe_busy_state[i] = false;

	INIT_DELAYED_WORK(&pe_work, pe_workfn);

	/*
	 * Start safe immediately. boost_limiter_get_limit() also defaults
	 * to PE_FREQ_SINGLE / 2314 MHz before its worker starts.
	 */
	pm_qos_add_request(&pe_max_qos_req,
			   PM_QOS_CLUSTER1_FREQ_MAX,
			   PE_FREQ_SINGLE);

	WRITE_ONCE(pe_applied_limit, PE_FREQ_SINGLE);

	pr_info("EMS power envelope: single=%u dual=%u multi=%u kHz\n",
		PE_FREQ_SINGLE, PE_FREQ_DUAL, PE_FREQ_MULTI);

	schedule_delayed_work(&pe_work,
			      msecs_to_jiffies(PE_INTERVAL_MS));

	return 0;
}
late_initcall(power_envelope_init);