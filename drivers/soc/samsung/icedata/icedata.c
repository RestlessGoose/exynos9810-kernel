// SPDX-License-Identifier: GPL-2.0
/* IceCold Engine - per-frequency, per-ASV undervolt lookup for Exynos9810. */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>

#include "icedata.h"

#define ICE_ASV_GROUPS 16
#define ICE_UV_PER_STEP 6250U

/* Written once by asv_table_init(), before FVMap initialization. */
unsigned int ice_asv_scores_cl0;
unsigned int ice_asv_scores_cl1;
unsigned int ice_asv_scores_g3d;
unsigned int ice_asv_scores_mif;
unsigned int ice_asv_scores_cp;
unsigned int ice_asv_scores_disp;

static const unsigned int ice_cl0_rates[] = {
	2002000, 1950000, 1794000, 1690000, 1456000, 1248000,
	1053000, 949000, 832000, 715000, 598000, 455000,
	299000, 208000,
};

/* Rows follow the frequency list; columns are raw ASV groups 0..15. */
static const u8 ice_cl0_steps[][ICE_ASV_GROUPS] = {
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2002000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 7, 7, 0, 0, 0 }, /* 1950000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 7, 8, 0, 0, 0 }, /* 1794000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 3, 5, 6, 7, 8, 0, 0, 0 }, /* 1690000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 3, 5, 6, 7, 8, 0, 0, 0 }, /* 1456000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 3, 5, 6, 7, 8, 0, 0, 0 }, /* 1248000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 3, 5, 6, 7, 8, 0, 0, 0 }, /* 1053000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 0, 0, 0 }, /* 949000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 2, 3, 5, 7, 7, 0, 0, 0 }, /* 832000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 4, 4, 0, 0, 0 }, /* 715000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 0, 0, 0 }, /* 598000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 455000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 299000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 208000 kHz */
};

static const unsigned int ice_cl1_rates[] = {
	2886000, 2860000, 2704000, 2652000, 2496000, 2314000,
	2106000, 2002000, 1924000, 1794000, 1690000, 1586000,
	1469000, 1261000, 1170000, 1066000, 962000, 858000,
	741000, 650000, 598000,
};

/* Rows follow the frequency list; columns are raw ASV groups 0..15. */
static const u8 ice_cl1_steps[][ICE_ASV_GROUPS] = {
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2886000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2860000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2704000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2652000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2496000 kHz */
	{ 0, 0, 0, 0, 1, 3, 4, 5, 6, 8, 8, 8, 0, 0, 0, 0 }, /* 2314000 kHz */
	{ 0, 0, 0, 0, 1, 4, 6, 7, 8, 9, 9, 9, 0, 0, 0, 0 }, /* 2106000 kHz */
	{ 0, 0, 0, 0, 1, 3, 5, 6, 7, 8, 8, 8, 0, 0, 0, 0 }, /* 2002000 kHz */
	{ 0, 0, 0, 0, 1, 3, 5, 6, 7, 8, 8, 8, 0, 0, 0, 0 }, /* 1924000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 1794000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 1690000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 1586000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 1469000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 1261000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 1170000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 1066000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 6, 6, 7, 7, 7, 0, 0, 0, 0 }, /* 962000 kHz */
	{ 0, 0, 0, 0, 1, 2, 4, 5, 5, 7, 7, 7, 0, 0, 0, 0 }, /* 858000 kHz */
	{ 0, 0, 0, 0, 1, 3, 5, 6, 6, 8, 7, 7, 0, 0, 0, 0 }, /* 741000 kHz */
	{ 0, 0, 0, 0, 1, 4, 6, 7, 7, 9, 8, 8, 0, 0, 0, 0 }, /* 650000 kHz */
	{ 0, 0, 0, 0, 1, 4, 7, 8, 8, 9, 8, 8, 0, 0, 0, 0 }, /* 598000 kHz */
};

static const unsigned int ice_g3d_rates[] = {
	598000, 572000, 546000, 455000, 385000, 338000,
	299000, 260000,
};

/* Rows follow the frequency list; columns are raw ASV groups 0..15. */
static const u8 ice_g3d_steps[][ICE_ASV_GROUPS] = {
	{ 0, 0, 0, 1, 2, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0 }, /* 598000 kHz */
	{ 0, 0, 0, 1, 2, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0 }, /* 572000 kHz */
	{ 0, 0, 0, 1, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0 }, /* 546000 kHz */
	{ 0, 0, 0, 1, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0 }, /* 455000 kHz */
	{ 0, 0, 0, 1, 3, 3, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0 }, /* 385000 kHz */
	{ 0, 0, 0, 1, 2, 3, 4, 5, 7, 0, 0, 0, 0, 0, 0, 0 }, /* 338000 kHz */
	{ 0, 0, 0, 1, 2, 2, 4, 4, 5, 0, 0, 0, 0, 0, 0, 0 }, /* 299000 kHz */
	{ 0, 0, 0, 0, 1, 1, 2, 3, 3, 0, 0, 0, 0, 0, 0, 0 }, /* 260000 kHz */
};

static const unsigned int ice_mif_rates[] = {
	2094000, 2093000, 2002000, 1794000, 1539000, 1352000,
	1014000, 845000, 676000, 546000, 421000,
};

/* Rows follow the frequency list; columns are raw ASV groups 0..15. */
static const u8 ice_mif_steps[][ICE_ASV_GROUPS] = {
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2094000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2093000 kHz */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* 2002000 kHz */
	{ 0, 0, 0, 1, 2, 4, 4, 6, 7, 7, 0, 0, 0, 0, 0, 0 }, /* 1794000 kHz */
	{ 0, 0, 0, 1, 2, 4, 4, 6, 7, 7, 0, 0, 0, 0, 0, 0 }, /* 1539000 kHz */
	{ 0, 0, 0, 1, 2, 4, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0 }, /* 1352000 kHz */
	{ 0, 0, 0, 1, 2, 2, 3, 3, 4, 5, 0, 0, 0, 0, 0, 0 }, /* 1014000 kHz */
	{ 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0 }, /* 845000 kHz */
	{ 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0 }, /* 676000 kHz */
	{ 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0 }, /* 546000 kHz */
	{ 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0 }, /* 421000 kHz */
};

static const unsigned int ice_cp_rates[] = {
	1300000, 950000, 800000, 533000,
};

/* Rows follow the frequency list; columns are raw ASV groups 0..15. */
static const u8 ice_cp_steps[][ICE_ASV_GROUPS] = {
	{ 0, 0, 0, 0, 0, 2, 2, 3, 4, 5, 5, 6, 0, 0, 0, 0 }, /* 1300000 kHz */
	{ 0, 0, 0, 0, 0, 1, 2, 3, 4, 4, 4, 5, 0, 0, 0, 0 }, /* 950000 kHz */
	{ 0, 0, 0, 0, 0, 1, 2, 3, 4, 4, 4, 5, 0, 0, 0, 0 }, /* 800000 kHz */
	{ 0, 0, 0, 0, 0, 0, 1, 2, 2, 3, 3, 4, 0, 0, 0, 0 }, /* 533000 kHz */
};

static const unsigned int ice_disp_rates[] = {
	640000, 534000, 400000, 200000, 134000,
};

/* Rows follow the frequency list; columns are raw ASV groups 0..15. */
static const u8 ice_disp_steps[][ICE_ASV_GROUPS] = {
	{ 0, 0, 0, 0, 0, 2, 2, 3, 3, 3, 3, 3, 0, 0, 0, 0 }, /* 640000 kHz */
	{ 0, 0, 0, 0, 0, 1, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0 }, /* 534000 kHz */
	{ 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 2, 2, 0, 0, 0, 0 }, /* 400000 kHz */
	{ 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 }, /* 200000 kHz */
	{ 0, 0, 0, 0, 0, 2, 2, 3, 3, 3, 3, 3, 0, 0, 0, 0 }, /* 134000 kHz */
};

struct ice_domain_table {
	const unsigned int *rates;
	const u8 (*steps)[ICE_ASV_GROUPS];
	unsigned int count;
	const unsigned int *asv_group;
};

#define ICE_TABLE(name) {                             \
	.rates = ice_##name##_rates,                     \
	.steps = ice_##name##_steps,                     \
	.count = ARRAY_SIZE(ice_##name##_rates),         \
	.asv_group = &ice_asv_scores_##name,             \
}

static const struct ice_domain_table ice_tables[ICE_DOMAIN_COUNT] = {
	[ICE_DOMAIN_CL0] = ICE_TABLE(cl0),
	[ICE_DOMAIN_CL1] = ICE_TABLE(cl1),
	[ICE_DOMAIN_G3D] = ICE_TABLE(g3d),
	[ICE_DOMAIN_MIF] = ICE_TABLE(mif),
	[ICE_DOMAIN_CP] = ICE_TABLE(cp),
	[ICE_DOMAIN_DISP] = ICE_TABLE(disp),
};

/* Resolve the Samsung DVFS name once per domain, not once per OPP. */
enum ice_domain ice_domain_from_name(const char *name)
{
	if (!name)
		return ICE_DOMAIN_NONE;

	if (!strcmp(name, "dvfs_cpucl0"))
		return ICE_DOMAIN_CL0;
	if (!strcmp(name, "dvfs_cpucl1"))
		return ICE_DOMAIN_CL1;
	if (!strcmp(name, "dvfs_g3d"))
		return ICE_DOMAIN_G3D;
	if (!strcmp(name, "dvfs_mif"))
		return ICE_DOMAIN_MIF;
	if (!strcmp(name, "dvfs_cp"))
		return ICE_DOMAIN_CP;
	if (!strcmp(name, "dvfs_disp"))
		return ICE_DOMAIN_DISP;

	return ICE_DOMAIN_NONE;
}

unsigned int ice_get_undervolt_uv(enum ice_domain domain, unsigned int rate)
{
	const struct ice_domain_table *table;
	unsigned int asv, i;

	if (domain < 0 || domain >= ICE_DOMAIN_COUNT)
		return 0;

	table = &ice_tables[domain];
	asv = *table->asv_group;
	if (unlikely(asv >= ICE_ASV_GROUPS))
		return 0;

	/* Match by actual frequency rather than assuming the OPP index. */
	for (i = 0; i < table->count; i++) {
		if (table->rates[i] == rate)
			return (unsigned int)table->steps[i][asv] * ICE_UV_PER_STEP;
	}

	/* An unknown OPP remains at Samsung's voltage. */
	return 0;
}
