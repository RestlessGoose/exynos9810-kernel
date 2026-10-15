// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 9810 CL1 Boost Limiter
 *
 * Three-layer boost control for the Exynos M3 cluster:
 *
 *   1. Short transient boost token
 *
 *      2314 MHz is treated as a latency-response frequency only.
 *      A workload may use it for a small amount of actual high-frequency
 *      residency before being limited to 2106 MHz.
 *
 *   2. Chassis thermal-energy budget
 *
 *      Sustained CL1 power fills a leaky energy bucket. As the bucket fills,
 *      the allowed maximum frequency progressively falls:
 *
 *          2106 -> 2002 -> 1794 -> 1586 MHz
 *
 *   3. Battery/chassis thermal headroom
 *
 *      Battery temperature controls both:
 *
 *        a) the lowest cap the thermal limiter may impose
 *        b) the amount of CL1 dynamic power considered sustainable
 *
 *          < 30 C:
 *              floor       = 1924 MHz
 *              sustainable = 650 mW
 *
 *          30-35 C:
 *              floor       = 1794 MHz
 *              sustainable = 500 mW
 *
 *          35-38 C:
 *              floor       = 1690 MHz
 *              sustainable = 350 mW
 *
 *          >= 38 C:
 *              floor       = 1586 MHz
 *              sustainable = 250 mW
 *
 *      Battery temperature is used as a slow chassis thermal proxy.
 *      The energy bucket remains responsible for fast preventative control.
 *
 * This file never changes cpufreq directly and owns no PM-QoS request.
 * power_envelope.c calls boost_limiter_get_limit() and combines the returned
 * suggestion with its instantaneous multicore electrical envelope.
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#include "ems.h"


#define BL_FIRST_CPU                    4
#define BL_LAST_CPU                     7


/*
 * HZ=100, therefore 20 ms corresponds to two jiffies.
 *
 * This is fast enough for the transient boost-token logic and far faster
 * than any meaningful chassis thermal time constant.
 */
#define BL_INTERVAL_MS                  20


/*
 * Frequency stages.
 *
 * These are maximum-frequency suggestions only. schedutil remains free to
 * select any lower frequency.
 */
#define BL_FREQ_MAX                     2314000
#define BL_FREQ_HIGH                    2106000
#define BL_FREQ_MED                     2002000
#define BL_FREQ_LOW                     1794000
#define BL_FREQ_DEEP                    1586000


/*
 * --------------------------------------------------------------------------
 * 2314 MHz transient boost token
 * --------------------------------------------------------------------------
 *
 * Three 20 ms observations give approximately 60 ms of actual residency
 * above 2106 MHz.
 *
 * Once consumed, 2314 remains unavailable until CL1 spends a meaningful
 * period at very low utilization.
 */
#define BL_SPIKE_BUDGET_MS              60
#define BL_SPIKE_REARM_MS               250

#define BL_SPIKE_ACTIVE_PCT              20
#define BL_SPIKE_REARM_PCT                8


/*
 * --------------------------------------------------------------------------
 * Thermal-energy bucket
 * --------------------------------------------------------------------------
 *
 * Rough synthetic thermal-energy capacity allocated to CL1 boosting.
 */
#define BL_BUCKET_MAX_UJ                6000000ULL


/*
 * Temperature-dependent sustainable dynamic-power allowance.
 *
 * This is not intended to be a literal physical measurement of chassis
 * dissipation. It is the amount of estimated CL1 dynamic power that the
 * controller will tolerate without increasing heat debt.
 *
 * As the chassis gets warmer, progressively less sustained power is allowed.
 */
#define BL_SUSTAINABLE_COOL_MW          650U
#define BL_SUSTAINABLE_WARM_MW          500U
#define BL_SUSTAINABLE_HOT_MW           350U
#define BL_SUSTAINABLE_VHOT_MW          250U


/*
 * Thermal-state entry thresholds.
 */
#define BL_ENTER_HIGH_PCT               15
#define BL_ENTER_MED_PCT                30
#define BL_ENTER_LOW_PCT                50
#define BL_ENTER_DEEP_PCT               70


/*
 * Recovery thresholds.
 *
 * Asymmetric hysteresis prevents repeated cap changes near a boundary.
 */
#define BL_EXIT_HIGH_PCT                 8
#define BL_EXIT_MED_PCT                 20
#define BL_EXIT_LOW_PCT                 40
#define BL_EXIT_DEEP_PCT                60


/*
 * --------------------------------------------------------------------------
 * Battery/chassis temperature control
 * --------------------------------------------------------------------------
 *
 * POWER_SUPPLY_PROP_TEMP is expected in tenths of a degree Celsius.
 *
 * Heating thresholds:
 *
 *      300 = 30.0 C
 *      350 = 35.0 C
 *      380 = 38.0 C
 *
 * Recovery thresholds sit 1 C lower.
 */
#define BL_TEMP_30C                     300
#define BL_TEMP_35C                     350
#define BL_TEMP_38C                     380

#define BL_TEMP_29C                     290
#define BL_TEMP_34C                     340
#define BL_TEMP_37C                     370


/*
 * Reject clearly bogus sensor readings rather than accidentally granting
 * the coldest performance policy because of a broken/early sensor value.
 */
#define BL_TEMP_VALID_MIN              (-200)
#define BL_TEMP_VALID_MAX               800


/*
 * Battery temperature is extremely slow compared with scheduler activity.
 */
#define BL_TEMP_UPDATE_MS               500


/*
 * Temperature-dependent lowest allowed limiter cap.
 */
#define BL_TEMP_FLOOR_COOL              1924000
#define BL_TEMP_FLOOR_WARM              1794000
#define BL_TEMP_FLOOR_HOT               1690000
#define BL_TEMP_FLOOR_VHOT              1586000


struct bl_power_point {
	unsigned int freq_khz;
	unsigned int power_mw;
};


/*
 * Corrected single-M3 incremental power estimates from freqbench.
 *
 * Small low-frequency measurement irregularities are smoothed while
 * retaining the measured overall curve.
 */
static const struct bl_power_point bl_power_table[] = {
	{  598000,  220 },
	{  650000,  225 },
	{  741000,  283 },
	{  858000,  315 },
	{  962000,  336 },
	{ 1066000,  390 },
	{ 1170000,  390 },
	{ 1261000,  395 },
	{ 1469000,  511 },
	{ 1586000,  617 },
	{ 1690000,  706 },
	{ 1794000,  800 },
	{ 1924000,  914 },
	{ 2002000, 1026 },
	{ 2106000, 1147 },
	{ 2314000, 1573 },
};


enum bl_state {
	BL_STATE_MAX = 0,
	BL_STATE_HIGH,
	BL_STATE_MED,
	BL_STATE_LOW,
	BL_STATE_DEEP,
};


enum bl_temp_state {
	BL_TEMP_COOL = 0,
	BL_TEMP_WARM,
	BL_TEMP_HOT,
	BL_TEMP_VHOT,
};


static struct delayed_work bl_work;


/*
 * --------------------------------------------------------------------------
 * Energy-bucket state
 * --------------------------------------------------------------------------
 */

static u64 bl_heat_debt_uj;

static enum bl_state bl_state = BL_STATE_MAX;

static unsigned int bl_thermal_limit_khz = BL_FREQ_MAX;


/*
 * --------------------------------------------------------------------------
 * 2314 MHz transient-token state
 * --------------------------------------------------------------------------
 */

static unsigned int bl_spike_used_ms;
static unsigned int bl_spike_quiet_ms;

static bool bl_spike_armed = true;

static unsigned int bl_spike_limit_khz = BL_FREQ_MAX;


/*
 * --------------------------------------------------------------------------
 * Battery/chassis state
 * --------------------------------------------------------------------------
 *
 * Until the first valid battery reading arrives, use the WARM policy rather
 * than assuming the device is completely cold.
 */

static struct power_supply *bl_battery_psy;

static enum bl_temp_state bl_temp_state = BL_TEMP_WARM;

static unsigned int bl_temp_floor_khz = BL_TEMP_FLOOR_WARM;

static unsigned int bl_sustainable_mw = BL_SUSTAINABLE_WARM_MW;

static unsigned int bl_temp_elapsed_ms;

static bool bl_temp_valid;

static int bl_battery_temp;


/*
 * Return estimated single-M3 incremental dynamic power for the current
 * cluster frequency.
 *
 * If the current value falls between measured OPPs, round upward to remain
 * conservative.
 */
static unsigned int bl_power_for_freq(unsigned int freq)
{
	int i;

	if (freq <= bl_power_table[0].freq_khz)
		return bl_power_table[0].power_mw;

	for (i = 1; i < ARRAY_SIZE(bl_power_table); i++) {
		if (freq <= bl_power_table[i].freq_khz)
			return bl_power_table[i].power_mw;
	}

	return bl_power_table[ARRAY_SIZE(bl_power_table) - 1].power_mw;
}


/*
 * One M3's normalized scheduler occupancy.
 *
 * EMS tracks USS and SSE separately, so account for both classes.
 */
static unsigned long bl_cpu_activity(int cpu)
{
	unsigned long uss;
	unsigned long sse;
	unsigned long activity;

	if (!cpu_online(cpu))
		return 0;

	uss = ml_cpu_util_ratio(cpu, 0);
	sse = ml_cpu_util_ratio(cpu, 1);

	activity = uss + sse;

	return min_t(unsigned long,
		     activity,
		     SCHED_CAPACITY_SCALE);
}


/*
 * Total CL1 occupancy:
 *
 *      1024 ~= one fully busy M3
 *      2048 ~= two
 *      3072 ~= three
 *      4096 ~= four
 */
static unsigned long bl_cluster_activity(void)
{
	unsigned long total;
	int cpu;

	total = 0;

	for (cpu = BL_FIRST_CPU; cpu <= BL_LAST_CPU; cpu++)
		total += bl_cpu_activity(cpu);

	return total;
}


static unsigned long bl_pct_to_util(unsigned int pct)
{
	return SCHED_CAPACITY_SCALE * pct / 100;
}


/*
 * --------------------------------------------------------------------------
 * 2314 MHz transient token
 * --------------------------------------------------------------------------
 *
 * Credit is consumed only while:
 *
 *      actual CL1 frequency > 2106 MHz
 *
 * and meaningful M3 activity exists.
 *
 * Merely having a runnable task does not consume the token if schedutil
 * decides the task does not need the highest OPP.
 */
static void bl_update_spike(unsigned int freq, unsigned long activity)
{
	unsigned long active_threshold;
	unsigned long rearm_threshold;

	active_threshold = bl_pct_to_util(BL_SPIKE_ACTIVE_PCT);
	rearm_threshold = bl_pct_to_util(BL_SPIKE_REARM_PCT);

	if (bl_spike_armed) {
		if (freq > BL_FREQ_HIGH &&
		    activity >= active_threshold) {
			bl_spike_used_ms += BL_INTERVAL_MS;

			if (bl_spike_used_ms >= BL_SPIKE_BUDGET_MS)
				bl_spike_used_ms = BL_SPIKE_BUDGET_MS;

			bl_spike_quiet_ms = 0;
		}

		if (bl_spike_used_ms >= BL_SPIKE_BUDGET_MS) {
			bl_spike_armed = false;
			bl_spike_quiet_ms = 0;

			WRITE_ONCE(bl_spike_limit_khz,
				   BL_FREQ_HIGH);
			return;
		}

		/*
		 * Partially consumed credit is restored only after a genuine
		 * low-utilization period.
		 */
		if (activity <= rearm_threshold) {
			bl_spike_quiet_ms += BL_INTERVAL_MS;

			if (bl_spike_quiet_ms >= BL_SPIKE_REARM_MS) {
				bl_spike_used_ms = 0;
				bl_spike_quiet_ms = 0;
			}
		} else {
			bl_spike_quiet_ms = 0;
		}

		WRITE_ONCE(bl_spike_limit_khz,
			   BL_FREQ_MAX);

		return;
	}


	/*
	 * Token exhausted.
	 *
	 * Stay capped at 2106 until CL1 remains quiet for the entire rearm
	 * interval.
	 */
	WRITE_ONCE(bl_spike_limit_khz,
		   BL_FREQ_HIGH);

	if (activity <= rearm_threshold) {
		bl_spike_quiet_ms += BL_INTERVAL_MS;

		if (bl_spike_quiet_ms >= BL_SPIKE_REARM_MS) {
			bl_spike_armed = true;
			bl_spike_used_ms = 0;
			bl_spike_quiet_ms = 0;

			WRITE_ONCE(bl_spike_limit_khz,
				   BL_FREQ_MAX);
		}
	} else {
		bl_spike_quiet_ms = 0;
	}
}


/*
 * --------------------------------------------------------------------------
 * Temperature state
 * --------------------------------------------------------------------------
 */

static void bl_set_temp_state(enum bl_temp_state state)
{
	bl_temp_state = state;

	switch (state) {
	case BL_TEMP_COOL:
		WRITE_ONCE(bl_temp_floor_khz,
			   BL_TEMP_FLOOR_COOL);

		WRITE_ONCE(bl_sustainable_mw,
			   BL_SUSTAINABLE_COOL_MW);
		break;


	case BL_TEMP_WARM:
		WRITE_ONCE(bl_temp_floor_khz,
			   BL_TEMP_FLOOR_WARM);

		WRITE_ONCE(bl_sustainable_mw,
			   BL_SUSTAINABLE_WARM_MW);
		break;


	case BL_TEMP_HOT:
		WRITE_ONCE(bl_temp_floor_khz,
			   BL_TEMP_FLOOR_HOT);

		WRITE_ONCE(bl_sustainable_mw,
			   BL_SUSTAINABLE_HOT_MW);
		break;


	case BL_TEMP_VHOT:
	default:
		WRITE_ONCE(bl_temp_floor_khz,
			   BL_TEMP_FLOOR_VHOT);

		WRITE_ONCE(bl_sustainable_mw,
			   BL_SUSTAINABLE_VHOT_MW);
		break;
	}
}


/*
 * Initial classification uses the exact requested temperature thresholds.
 *
 * Hysteresis is used after the first valid classification.
 */
static enum bl_temp_state bl_classify_temp(int temp)
{
	if (temp < BL_TEMP_30C)
		return BL_TEMP_COOL;

	if (temp < BL_TEMP_35C)
		return BL_TEMP_WARM;

	if (temp < BL_TEMP_38C)
		return BL_TEMP_HOT;

	return BL_TEMP_VHOT;
}


static void bl_update_temp_state(int temp)
{
	enum bl_temp_state new_state;

	if (!bl_temp_valid) {
		bl_temp_valid = true;

		bl_set_temp_state(bl_classify_temp(temp));

		return;
	}

	new_state = bl_temp_state;

	switch (bl_temp_state) {
	case BL_TEMP_COOL:
		/*
		 * Allow direct jumps to a hotter state if temperature rises
		 * substantially between measurements.
		 */
		if (temp >= BL_TEMP_38C)
			new_state = BL_TEMP_VHOT;
		else if (temp >= BL_TEMP_35C)
			new_state = BL_TEMP_HOT;
		else if (temp >= BL_TEMP_30C)
			new_state = BL_TEMP_WARM;

		break;


	case BL_TEMP_WARM:
		if (temp >= BL_TEMP_38C)
			new_state = BL_TEMP_VHOT;
		else if (temp >= BL_TEMP_35C)
			new_state = BL_TEMP_HOT;
		else if (temp <= BL_TEMP_29C)
			new_state = BL_TEMP_COOL;

		break;


	case BL_TEMP_HOT:
		if (temp >= BL_TEMP_38C)
			new_state = BL_TEMP_VHOT;
		else if (temp <= BL_TEMP_29C)
			new_state = BL_TEMP_COOL;
		else if (temp <= BL_TEMP_34C)
			new_state = BL_TEMP_WARM;

		break;


	case BL_TEMP_VHOT:
	default:
		/*
		 * Likewise, allow direct recovery if it cooled dramatically.
		 */
		if (temp <= BL_TEMP_29C)
			new_state = BL_TEMP_COOL;
		else if (temp <= BL_TEMP_34C)
			new_state = BL_TEMP_WARM;
		else if (temp <= BL_TEMP_37C)
			new_state = BL_TEMP_HOT;

		break;
	}

	if (new_state != bl_temp_state)
		bl_set_temp_state(new_state);
}


/*
 * Read battery temperature in tenths of a degree Celsius.
 */
static int bl_read_battery_temp(int *temp)
{
	union power_supply_propval val;
	int ret;

	if (!bl_battery_psy)
		bl_battery_psy = power_supply_get_by_name("battery");

	if (!bl_battery_psy)
		return -ENODEV;

	ret = power_supply_get_property(bl_battery_psy,
					POWER_SUPPLY_PROP_TEMP,
					&val);

	if (ret)
		return ret;

	/*
	 * Ignore obviously invalid values.
	 *
	 * This is especially useful during early boot if the battery driver
	 * temporarily exposes nonsense before it has fully initialized.
	 */
	if (val.intval < BL_TEMP_VALID_MIN ||
	    val.intval > BL_TEMP_VALID_MAX)
		return -EINVAL;

	*temp = val.intval;

	return 0;
}


static void bl_update_temperature(void)
{
	int temp;
	int ret;

	bl_temp_elapsed_ms += BL_INTERVAL_MS;

	if (bl_temp_elapsed_ms < BL_TEMP_UPDATE_MS)
		return;

	bl_temp_elapsed_ms = 0;

	ret = bl_read_battery_temp(&temp);

	if (ret)
		return;

	WRITE_ONCE(bl_battery_temp, temp);

	bl_update_temp_state(temp);
}


/*
 * --------------------------------------------------------------------------
 * Thermal-energy bucket
 * --------------------------------------------------------------------------
 *
 * This is effectively:
 *
 *   debt += estimated_power - sustainable_allowance
 *
 * integrated over time.
 *
 * The sustainable allowance is now temperature-dependent:
 *
 *      COOL  -> 650 mW
 *      WARM  -> 500 mW
 *      HOT   -> 350 mW
 *      VHOT  -> 250 mW
 *
 * Consequently, hotter chassis states:
 *
 *   - accumulate debt more quickly
 *   - recover debt more slowly
 *
 * which is exactly what we want from a preventative controller.
 */
static void bl_update_bucket(unsigned int freq, unsigned long activity)
{
	unsigned int power_mw;
	unsigned int sustainable_mw;

	u64 input_uj;
	u64 allowance_uj;
	u64 delta;

	input_uj = 0;

	if (freq && activity) {
		power_mw = bl_power_for_freq(freq);

		input_uj = (u64)power_mw *
			   activity *
			   BL_INTERVAL_MS;

		input_uj >>= SCHED_CAPACITY_SHIFT;
	}

	sustainable_mw = READ_ONCE(bl_sustainable_mw);

	/*
	 * mW * ms = uJ
	 */
	allowance_uj = (u64)sustainable_mw *
		       BL_INTERVAL_MS;


	if (input_uj > allowance_uj) {
		delta = input_uj - allowance_uj;

		if (delta >= BL_BUCKET_MAX_UJ - bl_heat_debt_uj)
			bl_heat_debt_uj = BL_BUCKET_MAX_UJ;
		else
			bl_heat_debt_uj += delta;

	} else {
		delta = allowance_uj - input_uj;

		if (delta >= bl_heat_debt_uj)
			bl_heat_debt_uj = 0;
		else
			bl_heat_debt_uj -= delta;
	}
}


static unsigned int bl_bucket_pct(void)
{
	return (unsigned int)
		div64_u64(bl_heat_debt_uj * 100ULL,
			  BL_BUCKET_MAX_UJ);
}


/*
 * --------------------------------------------------------------------------
 * Thermal bucket state machine
 * --------------------------------------------------------------------------
 *
 * Internal thermal cap:
 *
 *      <15%    2314
 *       15%    2106
 *       30%    2002
 *       50%    1794
 *       70%    1586
 *
 * The temperature-dependent floor is applied afterwards.
 */
static void bl_update_thermal_state(void)
{
	unsigned int pct;
	enum bl_state new_state;

	pct = bl_bucket_pct();

	new_state = bl_state;

	switch (bl_state) {
	case BL_STATE_MAX:
		if (pct >= BL_ENTER_HIGH_PCT)
			new_state = BL_STATE_HIGH;

		break;


	case BL_STATE_HIGH:
		if (pct >= BL_ENTER_MED_PCT)
			new_state = BL_STATE_MED;
		else if (pct <= BL_EXIT_HIGH_PCT)
			new_state = BL_STATE_MAX;

		break;


	case BL_STATE_MED:
		if (pct >= BL_ENTER_LOW_PCT)
			new_state = BL_STATE_LOW;
		else if (pct <= BL_EXIT_MED_PCT)
			new_state = BL_STATE_HIGH;

		break;


	case BL_STATE_LOW:
		if (pct >= BL_ENTER_DEEP_PCT)
			new_state = BL_STATE_DEEP;
		else if (pct <= BL_EXIT_LOW_PCT)
			new_state = BL_STATE_MED;

		break;


	case BL_STATE_DEEP:
		if (pct <= BL_EXIT_DEEP_PCT)
			new_state = BL_STATE_LOW;

		break;


	default:
		new_state = BL_STATE_MAX;

		break;
	}


	if (new_state == bl_state)
		return;


	bl_state = new_state;


	switch (bl_state) {
	case BL_STATE_HIGH:
		WRITE_ONCE(bl_thermal_limit_khz,
			   BL_FREQ_HIGH);
		break;


	case BL_STATE_MED:
		WRITE_ONCE(bl_thermal_limit_khz,
			   BL_FREQ_MED);
		break;


	case BL_STATE_LOW:
		WRITE_ONCE(bl_thermal_limit_khz,
			   BL_FREQ_LOW);
		break;


	case BL_STATE_DEEP:
		WRITE_ONCE(bl_thermal_limit_khz,
			   BL_FREQ_DEEP);
		break;


	case BL_STATE_MAX:
	default:
		WRITE_ONCE(bl_thermal_limit_khz,
			   BL_FREQ_MAX);
		break;
	}
}


/*
 * Temperature floor limits how deeply the energy bucket is allowed to
 * throttle.
 *
 * Example:
 *
 *      bucket cap = 1586
 *      battery    = 28 C
 *      temp floor = 1924
 *
 *      effective thermal cap = 1924
 */
static unsigned int bl_get_effective_thermal_limit(void)
{
	unsigned int thermal;
	unsigned int floor;

	thermal = READ_ONCE(bl_thermal_limit_khz);
	floor = READ_ONCE(bl_temp_floor_khz);

	return max(thermal, floor);
}


/*
 * Public cap suggestion consumed by power_envelope.c.
 *
 * Both independent restrictions must be obeyed:
 *
 *      transient 2314 token
 *      energy/temperature control
 *
 * Whichever is stricter wins.
 */
unsigned int boost_limiter_get_limit(void)
{
	unsigned int spike;
	unsigned int thermal;

	spike = READ_ONCE(bl_spike_limit_khz);

	thermal = bl_get_effective_thermal_limit();

	return min(spike, thermal);
}


static void bl_workfn(struct work_struct *work)
{
	unsigned int freq;
	unsigned long activity;

	/*
	 * CL1 shares one cpufreq policy, therefore CPU4 is sufficient to
	 * obtain the cluster's current frequency.
	 */
	freq = cpufreq_quick_get(BL_FIRST_CPU);

	activity = bl_cluster_activity();


	/*
	 * Update temperature first.
	 *
	 * If this 500 ms sample crosses a temperature boundary, the new
	 * sustainable-power allowance is therefore used immediately by this
	 * same bucket sample.
	 */
	bl_update_temperature();


	/*
	 * Fast transient-boost control.
	 */
	bl_update_spike(freq, activity);


	/*
	 * Medium-timescale energy control.
	 */
	bl_update_bucket(freq, activity);

	bl_update_thermal_state();


	schedule_delayed_work(&bl_work,
			      msecs_to_jiffies(BL_INTERVAL_MS));
}


static int __init boost_limiter_init(void)
{
	/*
	 * Energy bucket.
	 */
	bl_heat_debt_uj = 0;

	bl_state = BL_STATE_MAX;

	WRITE_ONCE(bl_thermal_limit_khz,
		   BL_FREQ_MAX);


	/*
	 * 2314 MHz boost token.
	 */
	bl_spike_used_ms = 0;

	bl_spike_quiet_ms = 0;

	bl_spike_armed = true;

	WRITE_ONCE(bl_spike_limit_khz,
		   BL_FREQ_MAX);


	/*
	 * Battery/chassis policy.
	 *
	 * Until a valid reading arrives, start in WARM mode:
	 *
	 *      floor       = 1794 MHz
	 *      sustainable = 500 mW
	 *
	 * This avoids assuming maximum thermal headroom during early boot.
	 */
	bl_battery_psy = NULL;

	bl_temp_valid = false;

	bl_battery_temp = 0;

	bl_temp_state = BL_TEMP_WARM;

	WRITE_ONCE(bl_temp_floor_khz,
		   BL_TEMP_FLOOR_WARM);

	WRITE_ONCE(bl_sustainable_mw,
		   BL_SUSTAINABLE_WARM_MW);


	/*
	 * Force a battery query on the first worker invocation.
	 */
	bl_temp_elapsed_ms = BL_TEMP_UPDATE_MS;


	INIT_DELAYED_WORK(&bl_work,
			  bl_workfn);


	pr_info("EMS boost limiter: spike=%ums rearm=%ums bucket=%llu uJ temp-aware floor/power enabled\n",
		BL_SPIKE_BUDGET_MS,
		BL_SPIKE_REARM_MS,
		BL_BUCKET_MAX_UJ);


	schedule_delayed_work(&bl_work,
			      msecs_to_jiffies(BL_INTERVAL_MS));

	return 0;
}

late_initcall(boost_limiter_init);