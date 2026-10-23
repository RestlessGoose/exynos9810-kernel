// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 9810 M3 transient boost and first-order thermal-headroom proxy.
 * Called by power_envelope.c; this file owns NO worker and NO PM-QoS vote.
 * Battery power measurements are NOT a die-temperature or OCP measurement.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/types.h>

#include "ems.h"
#include "cl1_power_model.h"

#define BL_DT_COMPAT            "samsung,exynos9810-cl1-envelope"
#define BL_STAGES               5
#define BL_TEMP_STATES          4
#define BL_MAX_POWER_MW         12000U

struct bl_config {
	u32 spike_ms;
	u32 rearm_ms;
	u32 spike_active_pct;
	u32 spike_quiet_pct;
	u32 bucket_capacity_uj;
	u32 thermal_tau_ms;
	u32 shared_pct;
	u32 stages_khz[BL_STAGES];
	u32 enter_pct[BL_STAGES - 1];
	u32 exit_pct[BL_STAGES - 1];
	u32 battery_thresholds[BL_TEMP_STATES - 1]; /* 0.1 C */
	u32 battery_hysteresis; /* 0.1 C */
	u32 sustainable_mw[BL_TEMP_STATES];
	u32 temp_ceiling_khz[BL_TEMP_STATES];
};

static struct bl_config bl_cfg = {
	.spike_ms = 40,
	.rearm_ms = 250,
	.spike_active_pct = 20,
	.spike_quiet_pct = 8,
	.bucket_capacity_uj = 6000000,
	.thermal_tau_ms = 5000,
	.shared_pct = 20,
	.stages_khz = { 2314000, 2106000, 2002000, 1794000, 1586000 },
	.enter_pct = { 15, 30, 50, 70 },
	.exit_pct = { 8, 20, 40, 60 },
	.battery_thresholds = { 300, 350, 380 },
	.battery_hysteresis = 10,
	.sustainable_mw = { 650, 500, 350, 250 },
	.temp_ceiling_khz = { 2314000, 2314000, 2002000, 1794000 },
};

static struct power_supply *bl_battery;
static bool bl_has_temperature;
static unsigned int bl_temp_state = 1;  /* startup: warm, not falsely cold */
static unsigned int bl_temp_elapsed_ms = 500;
static unsigned int bl_stage;
static u64 bl_energy_uj;
static unsigned int bl_spike_used_ms;
static unsigned int bl_quiet_ms;
static bool bl_spike_armed = true;
static unsigned int bl_cap_khz = 2314000;

static bool bl_valid_frequencies(const u32 *freqs, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (freqs[i] < 598000 || freqs[i] > 2314000)
			return false;
		if (i && freqs[i] > freqs[i - 1])
			return false;
	}
	return true;
}

static void __init bl_read_dts(void)
{
	struct device_node *np;
	u32 v, tmp[BL_STAGES];
	unsigned int i;
	bool okay;

	np = of_find_compatible_node(NULL, NULL, BL_DT_COMPAT);
	if (!np)
		return;

#define BL_READ_U32(prop, field, low, high) \
	do { \
		if (!of_property_read_u32(np, prop, &v) && \
		    v >= (low) && v <= (high)) \
			bl_cfg.field = v; \
	} while (0)
	BL_READ_U32("samsung,spike-budget-ms", spike_ms, 10U, 500U);
	BL_READ_U32("samsung,spike-rearm-ms", rearm_ms, 20U, 10000U);
	BL_READ_U32("samsung,spike-active-pct", spike_active_pct, 1U, 100U);
	BL_READ_U32("samsung,spike-quiet-pct", spike_quiet_pct, 0U, 50U);
	BL_READ_U32("samsung,thermal-capacity-uj", bucket_capacity_uj, 100000U, 50000000U);
	BL_READ_U32("samsung,thermal-tau-ms", thermal_tau_ms, 500U, 60000U);
	BL_READ_U32("samsung,shared-power-pct", shared_pct, 0U, 100U);
	BL_READ_U32("samsung,battery-temp-hysteresis", battery_hysteresis, 0U, 30U);
#undef BL_READ_U32

	if (bl_cfg.spike_quiet_pct >= bl_cfg.spike_active_pct) {
		bl_cfg.spike_active_pct = 20;
		bl_cfg.spike_quiet_pct = 8;
	}

	if (!of_property_read_u32_array(np, "samsung,thermal-stages-khz",
					 tmp, BL_STAGES) &&
	    bl_valid_frequencies(tmp, BL_STAGES))
		memcpy(bl_cfg.stages_khz, tmp, sizeof(bl_cfg.stages_khz));

	if (!of_property_read_u32_array(np, "samsung,thermal-enter-pct", tmp,
					 BL_STAGES - 1)) {
		okay = true;
		for (i = 0; i < BL_STAGES - 1; i++)
			if (tmp[i] > 100 || (i && tmp[i] <= tmp[i - 1]))
				okay = false;
		if (okay)
			memcpy(bl_cfg.enter_pct, tmp, sizeof(bl_cfg.enter_pct));
	}
	if (!of_property_read_u32_array(np, "samsung,thermal-exit-pct", tmp,
					 BL_STAGES - 1)) {
		okay = true;
		for (i = 0; i < BL_STAGES - 1; i++)
			if (tmp[i] >= bl_cfg.enter_pct[i] ||
			    (i && tmp[i] <= tmp[i - 1]))
				okay = false;
		if (okay)
			memcpy(bl_cfg.exit_pct, tmp, sizeof(bl_cfg.exit_pct));
	}

	/* Keep hysteresis coherent if only one threshold array was overridden. */
	for (i = 0; i < BL_STAGES - 1; i++) {
		if (bl_cfg.exit_pct[i] >= bl_cfg.enter_pct[i]) {
			static const u32 default_enter[] = { 15, 30, 50, 70 };
			static const u32 default_exit[] = { 8, 20, 40, 60 };
			memcpy(bl_cfg.enter_pct, default_enter,
			       sizeof(bl_cfg.enter_pct));
			memcpy(bl_cfg.exit_pct, default_exit,
			       sizeof(bl_cfg.exit_pct));
			break;
		}
	}

	if (!of_property_read_u32_array(np, "samsung,battery-thresholds-deci-c",
					 tmp, BL_TEMP_STATES - 1) &&
	    tmp[0] >= 100 && tmp[0] < tmp[1] &&
	    tmp[1] < tmp[2] && tmp[2] <= 600)
		memcpy(bl_cfg.battery_thresholds, tmp,
		       sizeof(bl_cfg.battery_thresholds));

	if (!of_property_read_u32_array(np, "samsung,sustainable-power-mw",
					 tmp, BL_TEMP_STATES)) {
		okay = true;
		for (i = 0; i < BL_TEMP_STATES; i++)
			if (!tmp[i] || tmp[i] > BL_MAX_POWER_MW ||
			    (i && tmp[i] > tmp[i - 1]))
				okay = false;
		if (okay)
			memcpy(bl_cfg.sustainable_mw, tmp,
			       sizeof(bl_cfg.sustainable_mw));
	}

	if (!of_property_read_u32_array(np, "samsung,battery-ceiling-khz",
					 tmp, BL_TEMP_STATES) &&
	    bl_valid_frequencies(tmp, BL_TEMP_STATES))
		memcpy(bl_cfg.temp_ceiling_khz, tmp, sizeof(bl_cfg.temp_ceiling_khz));

	of_node_put(np);
}

static int bl_battery_temperature(int *temp)
{
	union power_supply_propval value;
	int ret;

	if (!bl_battery)
		bl_battery = power_supply_get_by_name("battery");
	if (!bl_battery)
		return -ENODEV;
	ret = power_supply_get_property(bl_battery, POWER_SUPPLY_PROP_TEMP,
					&value);
	if (ret || value.intval < -200 || value.intval > 800)
		return ret ? ret : -EINVAL;
	*temp = value.intval;
	return 0;
}

static void bl_update_temperature(unsigned int dt_ms)
{
	int temperature;
	unsigned int state;

	bl_temp_elapsed_ms += dt_ms;
	if (bl_temp_elapsed_ms < 500)
		return;
	bl_temp_elapsed_ms %= 500;
	if (bl_battery_temperature(&temperature))
		return;

	state = bl_temp_state;
	if (!bl_has_temperature) {
		state = 0;
		while (state < BL_TEMP_STATES - 1 &&
		       temperature >= (int)bl_cfg.battery_thresholds[state])
			state++;
		bl_has_temperature = true;
	} else {
		while (state < BL_TEMP_STATES - 1 &&
		       temperature >= (int)bl_cfg.battery_thresholds[state])
			state++;
		while (state > 0 &&
		       temperature <= (int)bl_cfg.battery_thresholds[state - 1] -
				      (int)bl_cfg.battery_hysteresis)
			state--;
	}
	bl_temp_state = state;
}

static void bl_update_spike(unsigned int freq, unsigned long util,
			    unsigned int dt_ms)
{
	unsigned long active = SCHED_CAPACITY_SCALE *
				  bl_cfg.spike_active_pct / 100;
	unsigned long quiet = SCHED_CAPACITY_SCALE *
				 bl_cfg.spike_quiet_pct / 100;

	if (bl_spike_armed && freq > bl_cfg.stages_khz[1] && util >= active) {
		bl_spike_used_ms = min(bl_cfg.spike_ms, bl_spike_used_ms + dt_ms);
		bl_quiet_ms = 0;
		if (bl_spike_used_ms >= bl_cfg.spike_ms)
			bl_spike_armed = false;
	}

	if (util <= quiet) {
		bl_quiet_ms = min(bl_cfg.rearm_ms, bl_quiet_ms + dt_ms);
		if (bl_quiet_ms >= bl_cfg.rearm_ms) {
			bl_spike_armed = true;
			bl_spike_used_ms = 0;
		}
	} else if (!bl_spike_armed ||
		   !(freq > bl_cfg.stages_khz[1] && util >= active)) {
		bl_quiet_ms = 0;
	}
}

/*
 * RC proxy: E' = (P_in - P_sustainable) - E / tau.
 * P[mW] * dt[ms] is energy[uJ]. E is bounded above and below.
 * The decay term represents stronger cooling as the thermal headroom
 * has already been consumed. tau is NOT a measured die time constant.
 */
static void bl_update_energy(u32 freq, unsigned long total,
			     unsigned long peak, u32 dt_ms)
{
	u32 input_mw = cl1_estimated_power_mw(freq, total, peak,
						bl_cfg.shared_pct);
	s64 delta_uj = ((s64)input_mw -
			 (s64)bl_cfg.sustainable_mw[bl_temp_state]) * dt_ms;
	u64 decay = div64_u64(bl_energy_uj * dt_ms,
				 bl_cfg.thermal_tau_ms);
	s64 next = (s64)bl_energy_uj + delta_uj - (s64)decay;

	if (next < 0)
		bl_energy_uj = 0;
	else if ((u64)next > bl_cfg.bucket_capacity_uj)
		bl_energy_uj = bl_cfg.bucket_capacity_uj;
	else
		bl_energy_uj = next;
}

static void bl_update_stage(void)
{
	unsigned int pct = div64_u64(bl_energy_uj * 100ULL,
					    bl_cfg.bucket_capacity_uj);

	while (bl_stage < BL_STAGES - 1 &&
	       pct >= bl_cfg.enter_pct[bl_stage])
		bl_stage++;
	while (bl_stage > 0 && pct <= bl_cfg.exit_pct[bl_stage - 1])
		bl_stage--;
}

void boost_limiter_step(unsigned int freq, unsigned long total_util,
			unsigned long peak_util, unsigned int elapsed_ms)
{
	unsigned int thermal_khz, spike_khz;

	/* Sampling staleness protection, even if an outside caller is added. */
	elapsed_ms = min(elapsed_ms, 100U);
	if (!elapsed_ms)
		return;

	bl_update_temperature(elapsed_ms);
	bl_update_spike(freq, total_util, elapsed_ms);
	bl_update_energy(freq, total_util, peak_util, elapsed_ms);
	bl_update_stage();

	/* Independent limits: fast thermal debt AND slow battery/chassis proxy.
	 * A hot battery must never relax a cap imposed by accumulated heat. */
	thermal_khz = min(bl_cfg.stages_khz[bl_stage],
			  bl_cfg.temp_ceiling_khz[bl_temp_state]);
	spike_khz = bl_spike_armed ? bl_cfg.stages_khz[0] :
					 bl_cfg.stages_khz[1];
	WRITE_ONCE(bl_cap_khz, min(spike_khz, thermal_khz));
}

unsigned int boost_limiter_get_limit(void)
{
	return READ_ONCE(bl_cap_khz);
}

static int __init boost_limiter_init(void)
{
	bl_read_dts();
	bl_cap_khz = bl_cfg.stages_khz[0];
	pr_info("CL1 boost v2: token=%ums tau=%ums capacity=%u uJ shared=%u%%\n",
		bl_cfg.spike_ms, bl_cfg.thermal_tau_ms,
		bl_cfg.bucket_capacity_uj, bl_cfg.shared_pct);
	return 0;
}
late_initcall(boost_limiter_init);
