/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SAMSUNG_ICEDATA_H__
#define __SAMSUNG_ICEDATA_H__

#include <linux/types.h>

/*
 * Exynos9810 ASV groups captured during the early CAL/ASV init path.
 *
 * MIF/CP/DISP are intentionally retained even though the current ICE policy
 * only needs the CL0/CL1/G3D scores.  Keeping all six values preserves the
 * existing early-boot ASV plumbing and leaves room for future profiles.
 */
extern unsigned int ice_asv_scores_cl0;
extern unsigned int ice_asv_scores_cl1;
extern unsigned int ice_asv_scores_g3d;
extern unsigned int ice_asv_scores_mif;
extern unsigned int ice_asv_scores_cp;
extern unsigned int ice_asv_scores_disp;

/*
 * Apply the ICE policy to a single FVMap rate/voltage entry.
 *
 * Generated domains:
 *   dvfs_cpucl0 - compact ASV9 reference profile
 *   dvfs_cpucl1 - existing tested M3 curve/undervolt model
 *   dvfs_g3d    - compact ASV6 reference profile
 *
 * Offset-only domains retain Samsung's FVMap and receive a fixed global
 * voltage offset.
 */
unsigned int ice_adjust_voltage(const char *domain,
				unsigned long rate,
				unsigned int stock_uv);

#endif /* __SAMSUNG_ICEDATA_H__ */
