/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PRAGMATIC_PROFILE_H
#define _LINUX_PRAGMATIC_PROFILE_H
#include <linux/types.h>
#include <linux/atomic.h>

enum pr_profile_id { PR_OPTIMAL, PR_POWER_SAVER, PR_SCREEN_OFF };
enum pr_display_state { PR_DISPLAY_NORMAL, PR_DISPLAY_ALPM,
	PR_DISPLAY_OFF, PR_DISPLAY_UNKNOWN };
struct pr_profile_policy {
	unsigned int hold_us, small_up_steps, urgent_pct, recharge_pct;
	unsigned int requested_pelt;
};
/* One word publishes preference, display, effective policy and generation.
 * Generation changes only when the effective policy changes.
 */
#define PR_PROFILE_INITIAL ((1ULL << 8) | (PR_DISPLAY_UNKNOWN << 2))
static inline unsigned int pr_user_id(u64 s) { return s & 3; }
static inline unsigned int pr_display_id(u64 s) { return (s >> 2) & 3; }
static inline unsigned int pr_effective_id(u64 s) { return (s >> 4) & 3; }
static inline u64 pr_profile_generation(u64 s) { return s >> 8; }
#ifdef CONFIG_CPU_FREQ_GOV_PRAGMATIC
extern atomic64_t pr_profile_state;
extern const struct pr_profile_policy pr_profile_policies[3];
static inline u64 pr_profile_read(void) { return atomic64_read(&pr_profile_state); }
static inline const struct pr_profile_policy *pr_profile_policy(u64 s)
{
	return &pr_profile_policies[pr_effective_id(s)];
}
void pr_profile_display(enum pr_display_state state);
void pr_pelt_profile_changed(u64 state);
void pr_pelt_init(void);
void pr_pelt_init_failed(void);
int pr_pelt_set_auto(bool enable);
bool pr_pelt_auto_enabled(void);
int pr_pelt_status(char *buf, size_t size);
const char *pr_profile_name(unsigned int id);
const char *pr_display_name(unsigned int id);
#else
static inline void pr_profile_display(enum pr_display_state s) { }
#endif
#endif
