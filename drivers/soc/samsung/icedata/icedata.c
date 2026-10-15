/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IceCold Engine voltage policy for Exynos9810.
 *
 * ICE deliberately keeps policy out of fvmap.c.  FVMap supplies the domain,
 * frequency and Samsung voltage; this file decides whether to generate a
 * replacement voltage or apply a small fixed offset.
 */

#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/string.h>

#include "icedata.h"

#define ICE_STEP_UV		6250U
#define ICE_SCALE		1000LL

/* Simple global undervolts.  Keep these in regulator-step multiples. */
#define ICE_CP_OFFSET_UV	(-12500)
#define ICE_DISP_OFFSET_UV	(-12500)
#define ICE_INT_OFFSET_UV	(-12500)

/* Existing CL1 undervolt shape, kept unchanged by this refactor. */
#define ICE_CL1_UV_CENTER_MHZ	1794
#define ICE_CL1_UV_WIDTH_MHZ	420
#define ICE_CL1_UV_FULL_MHZ		1261
#define ICE_CL1_UV_FLOOR_MHZ	598

unsigned int ice_asv_scores_cl0;
unsigned int ice_asv_scores_cl1;
unsigned int ice_asv_scores_g3d;
unsigned int ice_asv_scores_mif;
unsigned int ice_asv_scores_cp;
unsigned int ice_asv_scores_disp;

struct ice_opp_point {
	unsigned int freq_mhz;
	unsigned int ref_uv;
	unsigned int spread_uv;
};

struct ice_reference_profile {
	const struct ice_opp_point *points;
	unsigned int count;
	unsigned int reference_asv;
	unsigned int min_voltage_uv;
};

struct ice_fixed_offset {
	const char *domain;
	int offset_uv;
};

/*
 * CPUCL0 reference profile.
 *
 * Samsung's ASV9 column is used as the reference because ASV9 is a much more
 * representative CL0 bin than the unusually strong ASV11 part used during
 * development.  A 9.375 mV/ASV spread reproduces the supplied ASV6-12 table
 * to within one 6.25 mV regulator step at the real OPPs.  The 455 MHz point
 * uses the smaller low-end spread seen in Samsung's table; 299/208 MHz are
 * already pinned to the 581.25 mV floor.
 */
static const struct ice_opp_point ice_cl0_points[] = {
	{ 2002, 1118750,  9375 },
	{ 1950, 1081250,  9375 },
	{ 1794, 1006250,  9375 },
	{ 1690,  962500,  9375 },
	{ 1456,  881250,  9375 },
	{ 1248,  812500,  9375 },
	{ 1053,  762500,  9375 },
	{  949,  731250,  9375 },
	{  832,  700000,  9375 },
	{  715,  668750,  9375 },
	{  598,  637500,  9375 },
	{  455,  593750,  6250 },
	{  299,  581250,     0 },
	{  208,  581250,     0 },
};

static const struct ice_reference_profile ice_cl0_profile = {
	.points			= ice_cl0_points,
	.count			= ARRAY_SIZE(ice_cl0_points),
	.reference_asv		= 9,
	.min_voltage_uv		= 581250,
};

/*
 * G3D reference profile.
 *
 * ASV6 is the useful real-world reference bin.  Across the observed ASV3-7
 * range Samsung's table is exceptionally regular: approximately 12.5 mV per
 * ASV group at every GPU OPP.
 */
static const struct ice_opp_point ice_g3d_points[] = {
	{ 598, 731250, 12500 },
	{ 572, 725000, 12500 },
	{ 546, 712500, 12500 },
	{ 455, 687500, 12500 },
	{ 385, 668750, 12500 },
	{ 338, 650000, 12500 },
	{ 299, 637500, 12500 },
	{ 260, 618750, 12500 },
};

static const struct ice_reference_profile ice_g3d_profile = {
	.points			= ice_g3d_points,
	.count			= ARRAY_SIZE(ice_g3d_points),
	.reference_asv		= 6,
	.min_voltage_uv		= 581250,
};

static const struct ice_fixed_offset ice_fixed_offsets[] = {
	{ "dvfs_cp",   ICE_CP_OFFSET_UV   },
	{ "dvfs_disp_evt1", ICE_DISP_OFFSET_UV },
	{ "dvfs_int",  ICE_INT_OFFSET_UV  },
};

static unsigned long ice_rate_to_mhz(unsigned long rate)
{
	return rate > 10000 ? rate / 1000 : rate;
}

static unsigned int ice_sanitize_asv(unsigned int asv)
{
	return min(asv, 15U);
}

static unsigned int ice_round_voltage(s64 voltage_uv,
				      unsigned int min_voltage_uv)
{
	s64 steps;

	if (voltage_uv < min_voltage_uv)
		voltage_uv = min_voltage_uv;

	steps = div_s64(voltage_uv + ICE_STEP_UV / 2, ICE_STEP_UV);
	voltage_uv = steps * ICE_STEP_UV;

	if (voltage_uv < min_voltage_uv)
		voltage_uv = min_voltage_uv;

	return (unsigned int)voltage_uv;
}

/*
 * Interpolate one compact reference profile.
 *
 * FVMap normally lands exactly on one of these OPPs, but interpolation keeps
 * ICE well behaved if another frequency is added later.
 */
static void ice_profile_at_freq(const struct ice_reference_profile *profile,
				unsigned long freq_mhz,
				s64 *ref_uv, s64 *spread_uv)
{
	const struct ice_opp_point *hi, *lo;
	unsigned int i;
	s64 span, pos;

	if (freq_mhz >= profile->points[0].freq_mhz) {
		*ref_uv = profile->points[0].ref_uv;
		*spread_uv = profile->points[0].spread_uv;
		return;
	}

	if (freq_mhz <= profile->points[profile->count - 1].freq_mhz) {
		*ref_uv = profile->points[profile->count - 1].ref_uv;
		*spread_uv = profile->points[profile->count - 1].spread_uv;
		return;
	}

	for (i = 0; i < profile->count - 1; i++) {
		hi = &profile->points[i];
		lo = &profile->points[i + 1];

		if (freq_mhz > hi->freq_mhz || freq_mhz < lo->freq_mhz)
			continue;

		span = hi->freq_mhz - lo->freq_mhz;
		pos = freq_mhz - lo->freq_mhz;

		*ref_uv = lo->ref_uv +
			div_s64(((s64)hi->ref_uv - lo->ref_uv) * pos, span);
		*spread_uv = lo->spread_uv +
			div_s64(((s64)hi->spread_uv - lo->spread_uv) * pos,
				span);
		return;
	}

	/* Should be unreachable for a valid descending profile. */
	*ref_uv = profile->points[profile->count - 1].ref_uv;
	*spread_uv = profile->points[profile->count - 1].spread_uv;
}

static unsigned int ice_generate_reference_voltage(
		const struct ice_reference_profile *profile,
		unsigned int asv, unsigned long freq_mhz)
{
	s64 ref_uv, spread_uv;
	s64 voltage_uv;

	asv = ice_sanitize_asv(asv);
	ice_profile_at_freq(profile, freq_mhz, &ref_uv, &spread_uv);

	voltage_uv = ref_uv +
		(s64)((int)profile->reference_asv - (int)asv) * spread_uv;

	return ice_round_voltage(voltage_uv, profile->min_voltage_uv);
}

/*
 * CL0 undervolt policy.
 *
 * The score controls the available headroom; the frequency gate fades the
 * offset out near the low-voltage floor.  2002 MHz remains untouched just as
 * in the old ICE table.  Typical examples:
 *
 *   ASV8  -> 18.75 mV at the main OPPs
 *   ASV9  -> 25.00 mV
 *   ASV11 -> 37.50 mV
 */
static unsigned int ice_cl0_uv_steps(unsigned int asv, unsigned long freq_mhz)
{
	unsigned int steps;

	if (freq_mhz > 1950 || freq_mhz <= 455)
		return 0;

	asv = ice_sanitize_asv(asv);
	steps = asv > 5 ? asv - 5 : 0;
	steps = min(steps, 7U);

	if (freq_mhz >= 949)
		return steps;
	if (freq_mhz >= 832)
		return DIV_ROUND_CLOSEST(steps * 3, 4);
	if (freq_mhz >= 715)
		return DIV_ROUND_CLOSEST(steps, 2);
	if (freq_mhz >= 598)
		return DIV_ROUND_CLOSEST(steps, 4);

	return 0;
}

/*
 * G3D undervolt policy derived from the useful ASV3-7 range of the previous
 * ICE table, without retaining the old 8x16 matrix.
 *
 *   ASV3 ->  6.25 mV at 385 MHz and above
 *   ASV6 -> 31.25 mV
 *   ASV7 -> 37.50 mV
 */
static unsigned int ice_g3d_uv_steps(unsigned int asv, unsigned long freq_mhz)
{
	unsigned int steps;

	asv = ice_sanitize_asv(asv);

	if (asv <= 2)
		steps = 0;
	else
		steps = asv - 2;

	/* Preserve the extra headroom historically used by the strong bins. */
	if (asv >= 6)
		steps++;

	steps = min(steps, 6U);

	if (freq_mhz >= 385)
		return steps;
	if (freq_mhz >= 299)
		return steps > 0 ? steps - 1 : 0;
	if (freq_mhz >= 260)
		return steps > 3 ? steps - 3 : 0;

	return 0;
}

/*
 * Existing CPUCL1 generator moved intact out of fvmap.c.
 *
 * The refactor intentionally does not retune M3: ASV9 keeps the voltage
 * behavior already validated on CrownTrail, including the low-end fade and
 * the extra mid-frequency undervolt pocket.
 */
static s64 ice_cl1_undervolt_millisteps(unsigned int asv,
					unsigned long freq_mhz)
{
	s64 global_uv = 8000;
	s64 gate;
	s64 mid_depth = 2000;
	s64 mid_shape = 0;
	s64 diff;

	if (asv < 9)
		global_uv -= (s64)(9 - asv) * 1000 / 3;
	else if (asv > 9)
		global_uv -= (s64)(asv - 9) * 2000 / 3;

	if (global_uv < 5000)
		global_uv = 5000;

	if (freq_mhz >= ICE_CL1_UV_FULL_MHZ) {
		gate = ICE_SCALE;
	} else if (freq_mhz <= ICE_CL1_UV_FLOOR_MHZ) {
		gate = 0;
	} else {
		gate = (s64)(freq_mhz - ICE_CL1_UV_FLOOR_MHZ) * ICE_SCALE;
		gate = div_s64(gate,
			ICE_CL1_UV_FULL_MHZ - ICE_CL1_UV_FLOOR_MHZ);
		gate = div_s64(gate * gate, ICE_SCALE);
	}

	global_uv = div_s64(global_uv * gate, ICE_SCALE);

	if (asv < 9)
		mid_depth -= (s64)(9 - asv) * 500 / 3;
	else if (asv > 9)
		mid_depth -= (s64)(asv - 9) * 250 / 3;

	if (mid_depth < 1000)
		mid_depth = 1000;

	diff = (s64)freq_mhz - ICE_CL1_UV_CENTER_MHZ;
	if (diff < 0)
		diff = -diff;

	if (diff < ICE_CL1_UV_WIDTH_MHZ) {
		mid_shape = ICE_SCALE -
			div_s64(diff * diff * ICE_SCALE,
				(s64)ICE_CL1_UV_WIDTH_MHZ * ICE_CL1_UV_WIDTH_MHZ);
	}

	mid_depth = div_s64(mid_depth * mid_shape, ICE_SCALE);

	return global_uv + mid_depth;
}

static unsigned int ice_generate_cl1(unsigned int asv, unsigned long freq_mhz)
{
	s64 x, x2;
	s64 base, spread, total;
	s64 steps;

	asv = ice_sanitize_asv(asv);
	freq_mhz = clamp(freq_mhz, 598UL, 2704UL);

	x = (s64)(freq_mhz - 598) * ICE_SCALE;
	x = div_s64(x, 2704 - 598);
	x2 = div_s64(x * x, ICE_SCALE);

	/* Existing ASV9 base curve: 96 + 32.38*x + 51.62*x^2 steps. */
	base = 96000;
	base += div_s64(32380 * x, ICE_SCALE);
	base += div_s64(51620 * x2, ICE_SCALE);

	/* Existing frequency-dependent ASV spread: 0.5 -> 2.4 steps/group. */
	spread = 500;
	spread += div_s64(1900 * x2, ICE_SCALE);

	total = base + (s64)(9 - (int)asv) * spread;
	total -= ice_cl1_undervolt_millisteps(asv, freq_mhz);

	steps = div_s64(total + ICE_SCALE / 2, ICE_SCALE);
	if (steps < 0)
		steps = 0;

	return max_t(unsigned int, (unsigned int)steps * ICE_STEP_UV, 600000U);
}

static unsigned int ice_apply_fixed_offset(unsigned int stock_uv, int offset_uv)
{
	s64 voltage_uv = (s64)stock_uv + offset_uv;

	if (voltage_uv < ICE_STEP_UV)
		voltage_uv = ICE_STEP_UV;

	return ice_round_voltage(voltage_uv, ICE_STEP_UV);
}

unsigned int ice_adjust_voltage(const char *domain,
				unsigned long rate,
				unsigned int stock_uv)
{
	unsigned long freq_mhz = ice_rate_to_mhz(rate);
	unsigned int voltage_uv;
	unsigned int uv_steps;
	unsigned int i;

	if (!strcmp(domain, "dvfs_cpucl0")) {
		voltage_uv = ice_generate_reference_voltage(&ice_cl0_profile,
				ice_asv_scores_cl0, freq_mhz);
		uv_steps = ice_cl0_uv_steps(ice_asv_scores_cl0, freq_mhz);
		return ice_round_voltage((s64)voltage_uv -
					 uv_steps * ICE_STEP_UV,
					 ice_cl0_profile.min_voltage_uv);
	}

	if (!strcmp(domain, "dvfs_cpucl1"))
		return ice_generate_cl1(ice_asv_scores_cl1, freq_mhz);

	if (!strcmp(domain, "dvfs_g3d")) {
		voltage_uv = ice_generate_reference_voltage(&ice_g3d_profile,
				ice_asv_scores_g3d, freq_mhz);
		uv_steps = ice_g3d_uv_steps(ice_asv_scores_g3d, freq_mhz);
		return ice_round_voltage((s64)voltage_uv -
					 uv_steps * ICE_STEP_UV,
					 ice_g3d_profile.min_voltage_uv);
	}

	for (i = 0; i < ARRAY_SIZE(ice_fixed_offsets); i++) {
		if (!strcmp(domain, ice_fixed_offsets[i].domain))
			return ice_apply_fixed_offset(stock_uv,
					ice_fixed_offsets[i].offset_uv);
	}

	return stock_uv;
}
