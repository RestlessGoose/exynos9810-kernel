/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CROWNTRAIL_ICE_H
#define _CROWNTRAIL_ICE_H

/* A table entry is the number of 6250 uV steps to subtract. */
enum ice_domain {
	ICE_DOMAIN_NONE = -1,
	ICE_DOMAIN_CL0,
	ICE_DOMAIN_CL1,
	ICE_DOMAIN_G3D,
	ICE_DOMAIN_MIF,
	ICE_DOMAIN_CP,
	ICE_DOMAIN_DISP,
	ICE_DOMAIN_COUNT,
};

/* Set by the Exynos9810 ASV initialization; these remain raw fuse groups. */
extern unsigned int ice_asv_scores_cl0;
extern unsigned int ice_asv_scores_cl1;
extern unsigned int ice_asv_scores_g3d;
extern unsigned int ice_asv_scores_mif;
extern unsigned int ice_asv_scores_cp;
extern unsigned int ice_asv_scores_disp;

enum ice_domain ice_domain_from_name(const char *name);
/* Returns a positive reduction in microvolts, or zero if not matched. */
unsigned int ice_get_undervolt_uv(enum ice_domain domain, unsigned int rate);

#endif /* _CROWNTRAIL_ICE_H */
