/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _EMS_CL1_POWER_MODEL_H
#define _EMS_CL1_POWER_MODEL_H

#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/sched.h>
#include <linux/types.h>

/*
 * Freqbench-derived incremental BATTERY power of ONE fully loaded M3,
 * relative to a low-frequency housekeeping baseline. Units: kHz, mW.
 * These are NOT regulator current limits or multi-core measurements.
 */
struct cl1_power_point {
	u32 freq_khz;
	u32 power_mw;
};

static const struct cl1_power_point cl1_power_table[] = {
	{  598000,  220 }, {  650000,  225 },
	{  741000,  283 }, {  858000,  315 },
	{  962000,  336 }, { 1066000,  390 },
	{ 1170000,  390 }, { 1261000,  395 },
	{ 1469000,  511 }, { 1586000,  617 },
	{ 1690000,  706 }, { 1794000,  800 },
	{ 1924000,  914 }, { 2002000, 1026 },
	{ 2106000, 1147 }, { 2314000, 1573 },
};

#define CL1_POWER_POINTS ARRAY_SIZE(cl1_power_table)

static inline u32 cl1_one_core_power_mw(u32 freq_khz)
{
	unsigned int i;

	if (freq_khz <= cl1_power_table[0].freq_khz)
		return cl1_power_table[0].power_mw;

	for (i = 1; i < CL1_POWER_POINTS; i++) {
		const struct cl1_power_point *lo = &cl1_power_table[i - 1];
		const struct cl1_power_point *hi = &cl1_power_table[i];

		if (freq_khz <= hi->freq_khz) {
			/* Interpolate the calibrated curve, not a linear f-only model. */
			return lo->power_mw +
				div_u64((u64)(hi->power_mw - lo->power_mw) *
					(freq_khz - lo->freq_khz),
					hi->freq_khz - lo->freq_khz);
		}
	}

	/* Do not extrapolate beyond the highest measured OPP. */
	return cl1_power_table[CL1_POWER_POINTS - 1].power_mw;
}

/*
 * Single-core freqbench cannot identify the shared-vs-per-core split.
 * Heuristic: shared_pct weights the busiest core once; remaining power
 * scales with total normalized utilization. One full core reproduces the
 * measured power; four full cores cost (4 - 3*shared_pct/100) times it.
 * This is a battery-power estimate, never an electrical/OCP guarantee.
 */
static inline u32 cl1_estimated_power_mw(u32 freq_khz,
					unsigned long total_util,
					unsigned long peak_util,
					u32 shared_pct)
{
	u64 weighted_util;
	u32 one_core;

	if (!freq_khz || !total_util)
		return 0;

	shared_pct = min(shared_pct, 100U);
	peak_util = min_t(unsigned long, peak_util, SCHED_CAPACITY_SCALE);
	total_util = min_t(unsigned long, total_util, 4UL * SCHED_CAPACITY_SCALE);
	weighted_util = (u64)(100 - shared_pct) * total_util +
			(u64)shared_pct * peak_util;
	one_core = cl1_one_core_power_mw(freq_khz);

	return div64_u64((u64)one_core * weighted_util,
			  100ULL * SCHED_CAPACITY_SCALE);
}

#endif /* _EMS_CL1_POWER_MODEL_H */
