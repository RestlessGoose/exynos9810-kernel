// SPDX-License-Identifier: GPL-2.0
/*
 * CrownTrail Pragmatic governor for Samsung Exynos 9810 (Linux 4.9).
 *
 * Scheduler/EMS inputs and Samsung slow cpufreq transition protocol are
 * deliberately compatible with cpufreq_schedutil.c.  Frequency selection,
 * M3 turbo credit, and the former CL1 boost limiter live here instead.
 *
 * Experimental: review and build against the exact downstream kernel tree.
 */
#define pr_fmt(fmt) "pragmatic: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/errno.h>
#include <linux/ems.h>
#include <linux/init.h>
#include <linux/irq_work.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <trace/events/power.h>

#include "sched.h"
#include "tune.h"
#include "ems/ems.h"
#include "ems/cl1_power_model.h"

#ifdef CONFIG_FREQVAR_TUNE
unsigned long freqvar_boost_vector(int cpu, unsigned long util);
#endif

#define PR_CL1_FIRST_CPU        4
#define PR_CL1_LAST_CPU         7
#define PR_CL1_CPUS             4
#define PR_STAGES               5
#define PR_TEMP_STATES          4
#define PR_BOOST_FAST_MS        10
#define PR_BOOST_ACTIVE_MS      40
#define PR_CL0_HEADROOM_PCT      115
#define PR_CL1_TURBO_URGENT_PCT  99
#define PR_CL1_TOP_PREMIUM_MW    360
#define PR_CL1_URGENT_PREMIUM_MW 160
#define PR_MAX_OPPS             32
#define PR_BOOST_SLOW_MS        200
#define PR_MAX_ELAPSED_MS       100
#define PR_DT_COMPAT            "samsung,exynos9810-cl1-envelope"

/* All capacities/ratios are fixed point; scheduler callbacks cannot sleep. */
struct pragmatic_policy;
struct pragmatic_cpu {
	struct update_util_data update_util;
	struct pragmatic_policy *pp;
	unsigned long util, max;
	unsigned int flags;
	u64 last_update;
	u32 iowait_boost, iowait_boost_max;
	bool iowait_pending;
};

struct pr_boost_config {
	u32 spike_ms, top_burst_ms, rearm_ms, spike_active_pct, spike_quiet_pct;
	u32 bucket_capacity_uj, thermal_tau_ms, shared_pct;
	u32 stages_khz[PR_STAGES];
	u32 enter_pct[PR_STAGES - 1], exit_pct[PR_STAGES - 1];
	u32 battery_thresholds[PR_TEMP_STATES - 1];
	u32 battery_hysteresis;
	u32 sustainable_mw[PR_TEMP_STATES];
	u32 temp_ceiling_khz[PR_TEMP_STATES];
	u32 turbo_base_khz, turbo_capacity_uj, turbo_recharge_mw;
};

struct pr_boost_state {
	struct pr_boost_config cfg;
	/* A nondeferrable, one-shot ceiling for a 2314 MHz pulse. */
	struct delayed_work spike_watchdog;
	/* Regular accounting stays deferrable to protect deep idle. */
	struct delayed_work work;
	raw_spinlock_t state_lock;
	struct power_supply *battery;
	bool temperature_valid, spike_armed;
	u32 temp_state, temp_elapsed_ms;
	u32 spike_used_ms, quiet_ms;
	u32 stage;
	u64 energy_uj, turbo_credit_uj, last_ns;
	u32 cap_khz;
};

struct pr_opp {
	u32 freq_khz;
	u32 power_mw;
};

struct pragmatic_policy {
	struct cpufreq_policy *policy;
	/* Static OPP cache: no allocations or power-curve scans in sched context. */
	struct pr_opp opps[PR_MAX_OPPS];
	u32 opp_count;
	raw_spinlock_t update_lock;
	struct irq_work irq_work;
	struct kthread_work work;
	struct kthread_worker worker;
	struct task_struct *thread;
	struct mutex work_lock;
	bool work_in_progress, limits_changed, started;
	unsigned int next_freq;
	u64 last_freq_update_time;
	unsigned int up_rate_us, down_rate_us;
	bool cl1;
	struct pr_boost_state boost;
};

static DEFINE_PER_CPU(struct pragmatic_cpu, pragmatic_cpu);

/* One boost state per CL1 cpufreq policy; no independent PM-QoS vote. */
static void pr_boost_defaults(struct pr_boost_config *c)
{
	*c = (struct pr_boost_config) {
		.spike_ms = 40, .top_burst_ms = 16, .rearm_ms = 250,
		.spike_active_pct = 20, .spike_quiet_pct = 8,
		.bucket_capacity_uj = 6000000, .thermal_tau_ms = 5000,
		.shared_pct = 20,
		.stages_khz = { 2314000, 2106000, 2002000, 1794000, 1586000 },
		.enter_pct = { 15, 30, 50, 70 },
		.exit_pct = { 8, 20, 40, 60 },
		.battery_thresholds = { 300, 350, 380 },
		.battery_hysteresis = 10,
		.sustainable_mw = { 650, 500, 350, 250 },
		.temp_ceiling_khz = { 2314000, 2314000, 2002000, 1794000 },
		.turbo_base_khz = 1794000,
		/* Roughly 40 ms of full-load 2314-vs-1794 MHz incremental energy. */
		.turbo_capacity_uj = 32000,
		/* Replenish 32 mJ over ~267 ms when not spending turbo. */
		.turbo_recharge_mw = 120,
	};
}

static bool pr_valid_descending(const u32 *v, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (v[i] < 598000 || v[i] > 2314000)
			return false;
		if (i && v[i] > v[i - 1])
			return false;
	}
	return true;
}

/* Existing v2 board DTS properties continue to work on Note9/S9/S9+. */
static void pr_boost_parse_dt(struct pr_boost_config *c)
{
	struct device_node *np;
	u32 value, v[PR_STAGES];
	unsigned int i;
	bool good;

	pr_boost_defaults(c);
	np = of_find_compatible_node(NULL, NULL, PR_DT_COMPAT);
	if (!np)
		return;

#define PR_DT_U32(prop, member, lo, hi) \
	do { \
		if (!of_property_read_u32(np, prop, &value) && \
		    value >= (lo) && value <= (hi)) \
			c->member = value; \
	} while (0)
	PR_DT_U32("samsung,spike-budget-ms", spike_ms, 10U, 500U);
	/* Pragmatic-specific stricter ceiling on the highest M3 OPP. */
	PR_DT_U32("samsung,pragmatic-top-burst-ms", top_burst_ms, 8U, 40U);
	PR_DT_U32("samsung,spike-rearm-ms", rearm_ms, 20U, 10000U);
	PR_DT_U32("samsung,spike-active-pct", spike_active_pct, 1U, 100U);
	PR_DT_U32("samsung,spike-quiet-pct", spike_quiet_pct, 0U, 50U);
	PR_DT_U32("samsung,thermal-capacity-uj", bucket_capacity_uj, 100000U, 50000000U);
	PR_DT_U32("samsung,thermal-tau-ms", thermal_tau_ms, 500U, 60000U);
	PR_DT_U32("samsung,shared-power-pct", shared_pct, 0U, 100U);
	PR_DT_U32("samsung,battery-temp-hysteresis", battery_hysteresis, 0U, 30U);
	PR_DT_U32("samsung,pragmatic-turbo-base-khz", turbo_base_khz, 1586000U, 2106000U);
	PR_DT_U32("samsung,pragmatic-turbo-capacity-uj", turbo_capacity_uj, 5000U, 200000U);
	PR_DT_U32("samsung,pragmatic-turbo-recharge-mw", turbo_recharge_mw, 10U, 2000U);
#undef PR_DT_U32

	if (c->spike_quiet_pct >= c->spike_active_pct) {
		c->spike_quiet_pct = 8;
		c->spike_active_pct = 20;
	}
	if (!of_property_read_u32_array(np, "samsung,thermal-stages-khz", v,
					 PR_STAGES) && pr_valid_descending(v, PR_STAGES))
		memcpy(c->stages_khz, v, sizeof(c->stages_khz));

	if (!of_property_read_u32_array(np, "samsung,thermal-enter-pct", v,
					 PR_STAGES - 1)) {
		good = true;
		for (i = 0; i < PR_STAGES - 1; i++)
			if (v[i] > 100 || (i && v[i] <= v[i - 1]))
				good = false;
		if (good)
			memcpy(c->enter_pct, v, sizeof(c->enter_pct));
	}
	if (!of_property_read_u32_array(np, "samsung,thermal-exit-pct", v,
					 PR_STAGES - 1)) {
		good = true;
		for (i = 0; i < PR_STAGES - 1; i++)
			if (v[i] >= c->enter_pct[i] || (i && v[i] <= v[i - 1]))
				good = false;
		if (good)
			memcpy(c->exit_pct, v, sizeof(c->exit_pct));
	}
	for (i = 0; i < PR_STAGES - 1; i++) {
		if (c->exit_pct[i] >= c->enter_pct[i]) {
			const u32 enter[] = { 15, 30, 50, 70 };
			const u32 exit[] = { 8, 20, 40, 60 };
			memcpy(c->enter_pct, enter, sizeof(enter));
			memcpy(c->exit_pct, exit, sizeof(exit));
			break;
		}
	}
	if (!of_property_read_u32_array(np, "samsung,battery-thresholds-deci-c",
					 v, PR_TEMP_STATES - 1) &&
	    v[0] >= 100 && v[0] < v[1] && v[1] < v[2] && v[2] <= 600)
		memcpy(c->battery_thresholds, v, sizeof(c->battery_thresholds));
	if (!of_property_read_u32_array(np, "samsung,sustainable-power-mw", v,
					 PR_TEMP_STATES)) {
		good = true;
		for (i = 0; i < PR_TEMP_STATES; i++)
			if (!v[i] || v[i] > 12000 || (i && v[i] > v[i - 1]))
				good = false;
		if (good)
			memcpy(c->sustainable_mw, v, sizeof(c->sustainable_mw));
	}
	if (!of_property_read_u32_array(np, "samsung,battery-ceiling-khz", v,
					 PR_TEMP_STATES) &&
	    pr_valid_descending(v, PR_TEMP_STATES))
		memcpy(c->temp_ceiling_khz, v, sizeof(c->temp_ceiling_khz));
	of_node_put(np);
}

static void pr_update_battery(struct pr_boost_state *b, u32 dt_ms)
{
	union power_supply_propval reading;
	unsigned int state;
	int temp, ret;

	b->temp_elapsed_ms += dt_ms;
	if (b->temp_elapsed_ms < 500)
		return;
	b->temp_elapsed_ms %= 500;
	if (!b->battery)
		b->battery = power_supply_get_by_name("battery");
	if (!b->battery)
		return;
	ret = power_supply_get_property(b->battery, POWER_SUPPLY_PROP_TEMP,
					&reading);
	if (ret || reading.intval < -200 || reading.intval > 800)
		return;
	temp = reading.intval;
	state = b->temp_state;
	if (!b->temperature_valid) {
		state = 0;
		while (state < PR_TEMP_STATES - 1 &&
		       temp >= (int)b->cfg.battery_thresholds[state])
			state++;
		b->temperature_valid = true;
	} else {
		while (state < PR_TEMP_STATES - 1 &&
		       temp >= (int)b->cfg.battery_thresholds[state])
			state++;
		while (state > 0 &&
		       temp <= (int)b->cfg.battery_thresholds[state - 1] -
				       (int)b->cfg.battery_hysteresis)
			state--;
	}
	b->temp_state = state;
}

/* Keep the legacy board spike budget, but allow Pragmatic to treat the
 * expensive >2106 MHz OPP more strictly without changing older DTS files.
 * A board may shorten either budget; neither can increase the other.
 */
static inline u32 pr_top_burst_ms(const struct pr_boost_config *c)
{
	return min(c->spike_ms, c->top_burst_ms);
}

static void pr_update_spike(struct pr_boost_state *b, u32 freq,
			    unsigned long peak, u32 dt_ms)
{
	const struct pr_boost_config *c = &b->cfg;
	unsigned long active = SCHED_CAPACITY_SCALE * c->spike_active_pct / 100;
	unsigned long quiet = SCHED_CAPACITY_SCALE * c->spike_quiet_pct / 100;

	if (b->spike_armed && freq > c->stages_khz[1] && peak >= active) {
		b->spike_used_ms = min(pr_top_burst_ms(c),
				       b->spike_used_ms + dt_ms);
		b->quiet_ms = 0;
		if (b->spike_used_ms >= pr_top_burst_ms(c))
			b->spike_armed = false;
	}
	if (peak <= quiet) {
		b->quiet_ms = min(c->rearm_ms, b->quiet_ms + dt_ms);
		if (b->quiet_ms >= c->rearm_ms) {
			b->spike_armed = true;
			b->spike_used_ms = 0;
		}
	} else if (!b->spike_armed ||
		   !(freq > c->stages_khz[1] && peak >= active)) {
		b->quiet_ms = 0;
	}
}

static void pr_update_debt(struct pr_boost_state *b, u32 freq,
			   unsigned long total, unsigned long peak, u32 dt_ms)
{
	const struct pr_boost_config *c = &b->cfg;
	u32 input_mw = cl1_estimated_power_mw(freq, total, peak, c->shared_pct);
	s64 injected = ((s64)input_mw - c->sustainable_mw[b->temp_state]) * dt_ms;
	u64 decay = div64_u64(b->energy_uj * dt_ms, c->thermal_tau_ms);
	s64 next = (s64)b->energy_uj + injected - (s64)decay;
	u32 pct;

	if (next < 0)
		b->energy_uj = 0;
	else if ((u64)next > c->bucket_capacity_uj)
		b->energy_uj = c->bucket_capacity_uj;
	else
		b->energy_uj = next;

	pct = div64_u64(b->energy_uj * 100ULL, c->bucket_capacity_uj);
	while (b->stage < PR_STAGES - 1 && pct >= c->enter_pct[b->stage])
		b->stage++;
	while (b->stage > 0 && pct <= c->exit_pct[b->stage - 1])
		b->stage--;
}

static void pr_update_credit(struct pr_boost_state *b, u32 freq,
			     unsigned long total, unsigned long peak, u32 dt_ms)
{
	const struct pr_boost_config *c = &b->cfg;
	u32 actual = cl1_estimated_power_mw(freq, total, peak, c->shared_pct);
	u32 reference = cl1_estimated_power_mw(c->turbo_base_khz,
						      total, peak, c->shared_pct);
	u64 excess = actual > reference ? (u64)(actual - reference) * dt_ms : 0;
	u64 refill = (u64)c->turbo_recharge_mw * dt_ms;

	if (excess) {
		b->turbo_credit_uj = excess >= b->turbo_credit_uj ? 0 :
					b->turbo_credit_uj - excess;
	} else {
		b->turbo_credit_uj = min_t(u64, c->turbo_capacity_uj,
						     b->turbo_credit_uj + refill);
	}
}

static u32 pr_credit_cap(const struct pr_boost_state *b)
{
	u64 credit = b->turbo_credit_uj;
	u64 capacity = b->cfg.turbo_capacity_uj;

	if (!credit)
		return b->cfg.turbo_base_khz;
	if (credit < capacity / 4)
		return max(b->cfg.turbo_base_khz, 1924000U);
	if (credit < capacity / 2)
		return max(b->cfg.turbo_base_khz, 2002000U);
	if (credit < (capacity * 3) / 4)
		return max(b->cfg.turbo_base_khz, 2106000U);
	return 2314000;
}

static unsigned long pr_sample_util(int cpu)
{
	unsigned long util;

	if (!cpu_online(cpu))
		return 0;
	util = ml_cpu_util_ratio(cpu, 0) + ml_cpu_util_ratio(cpu, 1);
	return min_t(unsigned long, util, SCHED_CAPACITY_SCALE);
}

/*
 * Enforce a reduced ceiling even without another scheduler utilization tick.
 * This is necessary when an asynchronous cpufreq transition completes just
 * before the last runnable task goes idle: time-in-state can otherwise grow
 * at 2314 MHz while the deferrable periodic sampler is asleep.
 */
static void pr_force_cap(struct pragmatic_policy *pp, u32 cap)
{
	struct cpufreq_policy *policy = pp->policy;
	unsigned long flags;

	/* QoS or another actor may raise policy->min above our soft cap. */
	cap = max(cap, READ_ONCE(policy->min));
	raw_spin_lock_irqsave(&pp->update_lock, flags);
	if (READ_ONCE(pp->started) &&
	    (pp->next_freq > cap || READ_ONCE(policy->cur) > cap)) {
		pp->next_freq = cpufreq_driver_resolve_freq(policy, cap);
		pp->last_freq_update_time = 0;
		if (!pp->work_in_progress) {
			pp->work_in_progress = true;
			irq_work_queue(&pp->irq_work);
		}
	}
	raw_spin_unlock_irqrestore(&pp->update_lock, flags);
}

/* Nondeferrable only during an actual maximum-turbo pulse. */
static void pr_spike_watchdog_fn(struct work_struct *work)
{
	struct pr_boost_state *b = container_of(to_delayed_work(work),
					 struct pr_boost_state, spike_watchdog);
	struct pragmatic_policy *pp = container_of(b, struct pragmatic_policy, boost);
	unsigned long flags;
	u32 cap;

	if (!READ_ONCE(pp->started))
		return;

	raw_spin_lock_irqsave(&b->state_lock, flags);
	b->spike_armed = false;
	b->spike_used_ms = pr_top_burst_ms(&b->cfg);
	b->quiet_ms = 0;
	cap = min(READ_ONCE(b->cap_khz), b->cfg.stages_khz[1]);
	WRITE_ONCE(b->cap_khz, cap);
	raw_spin_unlock_irqrestore(&b->state_lock, flags);

	pr_force_cap(pp, cap);
}

static void pr_boost_workfn(struct work_struct *work)
{
	struct pr_boost_state *b = container_of(to_delayed_work(work),
						 struct pr_boost_state, work);
	struct pragmatic_policy *pp = container_of(b, struct pragmatic_policy, boost);
	struct cpufreq_policy *policy = pp->policy;
	u64 now_ns = ktime_get_ns();
	u64 diff_ns = now_ns - b->last_ns;
	unsigned long peak = 0, total = 0, flags;
	u32 dt_ms, freq, cap, next_ms;
	int cpu;

	b->last_ns = now_ns;
	dt_ms = min_t(u64, div_u64(diff_ns, 1000000ULL), PR_MAX_ELAPSED_MS);
	if (!dt_ms)
		dt_ms = 1;
	for (cpu = PR_CL1_FIRST_CPU; cpu <= PR_CL1_LAST_CPU; cpu++) {
		unsigned long util = pr_sample_util(cpu);
		total += util;
		peak = max(peak, util);
	}

	freq = cpufreq_quick_get(policy->cpu);
	if (!freq)
		freq = READ_ONCE(policy->cur);

	pr_update_battery(b, dt_ms);
	raw_spin_lock_irqsave(&b->state_lock, flags);
	pr_update_spike(b, freq, peak, dt_ms);
	pr_update_debt(b, freq, total, peak, dt_ms);
	/* Idle-only recharges may use the true elapsed interval: deferrable
	 * work can legitimately sleep much longer than its configured delay.
	 * Never attribute an extended interval to a stale active sample.
	 */
	pr_update_credit(b, freq, total, peak,
			 total ? dt_ms : min_t(u64,
						   div_u64(diff_ns, 1000000ULL),
						   1000ULL));

	cap = min(b->cfg.stages_khz[b->stage],
		  b->cfg.temp_ceiling_khz[b->temp_state]);
	cap = min(cap, b->spike_armed ? b->cfg.stages_khz[0] :
					 b->cfg.stages_khz[1]);
	cap = min(cap, pr_credit_cap(b));
	WRITE_ONCE(b->cap_khz, cap);
	raw_spin_unlock_irqrestore(&b->state_lock, flags);

	/* Enforce a lowered cap even if the cluster has just become idle. */
	pr_force_cap(pp, cap);

	/* Deferrable delayed work avoids forcing a sleeping CPU to wake up. */
	next_ms = (freq > b->cfg.turbo_base_khz && peak) ?
		  PR_BOOST_FAST_MS : (peak ? PR_BOOST_ACTIVE_MS :
						      PR_BOOST_SLOW_MS);
	if (READ_ONCE(pp->started))
		schedule_delayed_work(&b->work, msecs_to_jiffies(next_ms));
}

static inline void pr_boost_init(struct pragmatic_policy *pp)
{
	struct pr_boost_state *b = &pp->boost;

	pr_boost_parse_dt(&b->cfg);
	b->temp_state = 1; /* fail-safe: do not falsely assume a cold battery */
	b->temp_elapsed_ms = 500;
	b->spike_armed = true;
	b->turbo_credit_uj = b->cfg.turbo_capacity_uj;
	b->cap_khz = b->cfg.stages_khz[0];
	raw_spin_lock_init(&b->state_lock);
	INIT_DELAYED_WORK(&b->spike_watchdog, pr_spike_watchdog_fn);
	INIT_DEFERRABLE_WORK(&b->work, pr_boost_workfn);
}

/* Driver's POSTCHANGE callback already feeds freqvar's rate limit tables. */
void pragmatic_update_rate_limit_us(struct cpufreq_policy *policy,
				    int up_us, int down_us)
{
	struct pragmatic_policy *pp;

	if (!policy || !policy->governor_data ||
	    !policy->governor || strcmp(policy->governor->name, "pragmatic"))
		return;
	pp = policy->governor_data;
	if (up_us < 0 || down_us < 0 || up_us > 1000000 || down_us > 1000000)
		return;
	WRITE_ONCE(pp->up_rate_us, up_us);
	WRITE_ONCE(pp->down_rate_us, down_us);
}

static inline bool pr_freq_ready(struct pragmatic_policy *pp, u64 now)
{
	u64 delta = now - pp->last_freq_update_time;
	u32 up = READ_ONCE(pp->up_rate_us);
	u32 down = READ_ONCE(pp->down_rate_us);

	if (unlikely(pp->limits_changed)) {
		pp->limits_changed = false;
		return true;
	}
	return delta >= (u64)min(up, down) * NSEC_PER_USEC;
}

static void pr_cache_opps(struct pragmatic_policy *pp)
{
	struct cpufreq_frequency_table *pos;
	struct cpufreq_policy *policy = pp->policy;

	if (!pp->cl1 || !policy->freq_table)
		return;
	cpufreq_for_each_valid_entry(pos, policy->freq_table) {
		struct pr_opp *opp;

		if (pp->opp_count >= PR_MAX_OPPS) {
			pr_warn("CL1 OPP table exceeds %u entries; using driver mapping\n",
				PR_MAX_OPPS);
			pp->opp_count = 0;
			return;
		}
		opp = &pp->opps[pp->opp_count++];
		opp->freq_khz = pos->frequency;
		opp->power_mw = cl1_one_core_power_mw(opp->freq_khz);
	}
}

static unsigned int pr_select_cl1(struct pragmatic_policy *pp,
				   unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = pp->policy;
	u32 base = arch_scale_freq_invariant() ? policy->max : policy->cur;
	u32 cap = max(policy->min,
		      min(READ_ONCE(pp->boost.cap_khz), policy->max));
	u32 target, best_freq = 0;
	u32 i;
	u64 best_score = U64_MAX;
	u32 urgency = div64_u64((u64)util * 100, max);
	u32 penalty = urgency >= 75 ? 10 : 5;
	const struct pr_boost_state *b = &pp->boost;
	u64 credit = READ_ONCE(b->turbo_credit_uj);
	u32 baseline_power = cl1_one_core_power_mw(b->cfg.turbo_base_khz);
	u32 depleted_pct = credit < b->cfg.turbo_capacity_uj ?
		div64_u64((u64)(b->cfg.turbo_capacity_uj - credit) * 100,
			  b->cfg.turbo_capacity_uj) : 0;

	/* Demand remains utilization-based; cost controls how far to underrun it. */
	target = min_t(u64, cap,
			   div64_u64((u64)base * util * 112ULL, (u64)max * 100ULL));

	for (i = 0; i < pp->opp_count; i++) {
		u32 freq = pp->opps[i].freq_khz;
		u32 power = pp->opps[i].power_mw;
		u64 score;
		u32 gap_pct = 0;

		if (freq < policy->min || freq > cap)
			continue;
		if (freq < target && target)
			gap_pct = (u64)(target - freq) * 100 / target;
		score = power + (u64)penalty * gap_pct * gap_pct;
		/* Shrinking turbo credit penalizes upper OPPs progressively. */
		if (freq > b->cfg.turbo_base_khz)
			score += (u64)(power - baseline_power) * depleted_pct / 100;
		/* The 2314 MHz spike is reserved for effectively saturated demand. */
		if (freq > b->cfg.stages_khz[1])
			score += urgency >= PR_CL1_TURBO_URGENT_PCT ?
				 PR_CL1_URGENT_PREMIUM_MW : PR_CL1_TOP_PREMIUM_MW;
		if (score < best_score ||
		    (score == best_score && freq < best_freq)) {
			best_score = score;
			best_freq = freq;
		}
	}
	if (best_freq)
		return best_freq;
	/* No usable OPP cache or a policy floor higher than the turbo cap.
	 * Ask the driver for the closest valid target rather than maxing CL1.
	 */
	return cpufreq_driver_resolve_freq(policy, target);
}

static unsigned int pr_select_freq(struct pragmatic_policy *pp,
				    unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = pp->policy;
	u32 base, target;

	if (!max)
		return policy->min;
	if (pp->cl1)
		return pr_select_cl1(pp, util, max);

	/* Keep A55 responsive, but reduce max-OPP pinning. 115% headroom
	 * still allows full 1950 MHz at ~87% of normalized CPU capacity.
	 */
	base = arch_scale_freq_invariant() ? policy->max : policy->cur;
	target = div64_u64((u64)base * util * PR_CL0_HEADROOM_PCT,
			   (u64)max * 100);
	return cpufreq_driver_resolve_freq(policy, target);
}

static void pr_get_util(unsigned long *util, unsigned long *max, u64 now)
{
	int cpu = smp_processor_id();
	struct rq *rq = cpu_rq(cpu);
	unsigned long max_cap = arch_scale_cpu_capacity(NULL, cpu);
	unsigned long rt;
	s64 delta;

	sched_avg_update(rq);
	delta = now - rq->age_stamp;
	if (unlikely(delta < 0))
		delta = 0;
	rt = div64_u64(rq->rt_avg, sched_avg_period() + delta);
	rt = (rt * max_cap) >> SCHED_CAPACITY_SHIFT;
	/* Preserve the exact EMS/RT accounting of the known-good schedutil. */
	*util = ml_boosted_cpu_util(cpu) + rt;
	*util += rt;
#ifdef CONFIG_FREQVAR_TUNE
	*util = freqvar_boost_vector(cpu, *util);
#endif
	*util = min(*util, max_cap);
	*max = max_cap;
	part_cpu_active_ratio(util, max, cpu);
}

static void pr_iowait_set(struct pragmatic_cpu *pc, u64 now, unsigned int flags)
{
	struct cpufreq_policy *policy = pc->pp->policy;

	if (!policy->iowait_boost_enable)
		return;
	if (pc->iowait_boost && now - pc->last_update > TICK_NSEC) {
		pc->iowait_boost = 0;
		pc->iowait_pending = false;
	}
	if (flags & SCHED_CPUFREQ_IOWAIT) {
		if (pc->iowait_pending)
			return;
		pc->iowait_pending = true;
		if (pc->iowait_boost)
			pc->iowait_boost = min(pc->iowait_boost << 1,
						  pc->iowait_boost_max);
		else
			pc->iowait_boost = policy->min;
	}
}

static void pr_iowait_apply(struct pragmatic_cpu *pc,
			    unsigned long *util, unsigned long *max)
{
	u32 boost;

	if (!pc->iowait_boost)
		return;
	if (pc->iowait_pending)
		pc->iowait_pending = false;
	else {
		pc->iowait_boost >>= 1;
		if (pc->iowait_boost < pc->pp->policy->min) {
			pc->iowait_boost = 0;
			return;
		}
	}
	boost = pc->iowait_boost;
	if ((u64)*util * pc->iowait_boost_max < (u64)*max * boost) {
		*util = boost;
		*max = pc->iowait_boost_max;
	}
}

static unsigned int pr_shared_freq(struct pragmatic_policy *pp, u64 now)
{
	struct cpufreq_policy *policy = pp->policy;
	unsigned long util = 0, max = 1;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus) {
		struct pragmatic_cpu *pc = &per_cpu(pragmatic_cpu, cpu);
		unsigned long j_util, j_max;

		if (now - pc->last_update > TICK_NSEC) {
			pc->iowait_boost = 0;
			pc->iowait_pending = false;
			continue;
		}
		if (pc->flags & SCHED_CPUFREQ_DL)
			return pp->cl1 ? min(policy->cpuinfo.max_freq,
						 READ_ONCE(pp->boost.cap_khz)) :
						 policy->cpuinfo.max_freq;
		j_util = pc->util;
		j_max = pc->max;
		if ((u64)j_util * max >= (u64)j_max * util) {
			util = j_util;
			max = j_max;
		}
		pr_iowait_apply(pc, &util, &max);
	}
	return pr_select_freq(pp, util, max);
}

static void pr_commit(struct pragmatic_policy *pp, u64 now, unsigned int next)
{
	u64 elapsed = now - pp->last_freq_update_time;
	u32 delay = next > pp->next_freq ? READ_ONCE(pp->up_rate_us) :
						 READ_ONCE(pp->down_rate_us);

	if (next != pp->next_freq &&
	    elapsed < (u64)delay * NSEC_PER_USEC)
		return;
	if (next == pp->next_freq && !pp->limits_changed)
		return;

	pp->next_freq = next;
	pp->last_freq_update_time = now;
	if (!pp->work_in_progress) {
		pp->work_in_progress = true;
		irq_work_queue(&pp->irq_work);
	}
}

static void pr_update(struct update_util_data *hook, u64 now, unsigned int flags)
{
	struct pragmatic_cpu *pc = container_of(hook, struct pragmatic_cpu,
						    update_util);
	struct pragmatic_policy *pp = pc->pp;
	unsigned long util, max;
	unsigned int next;

	pr_get_util(&util, &max, now);
	raw_spin_lock(&pp->update_lock);
	pc->util = util;
	pc->max = max;
	pc->flags = flags;
	pr_iowait_set(pc, now, flags);
	pc->last_update = now;
	if (pr_freq_ready(pp, now)) {
		next = pr_shared_freq(pp, now);
		pr_commit(pp, now, next);
	}
	raw_spin_unlock(&pp->update_lock);
}

static void pr_work(struct kthread_work *work)
{
	struct pragmatic_policy *pp = container_of(work,
						  struct pragmatic_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->update_lock, flags);
	freq = pp->next_freq;
	pp->work_in_progress = false;
	raw_spin_unlock_irqrestore(&pp->update_lock, flags);

	mutex_lock(&pp->work_lock);
	/* A queued request can outlive the cap used by the scheduler.
	 * Recheck the live M3 ceiling immediately before the slow transition.
	 */
	if (pp->cl1)
		freq = min(freq, max(READ_ONCE(pp->policy->min),
				READ_ONCE(pp->boost.cap_khz)));
	/* We must arm the one-shot timer after the driver has completed its
	 * transition, not merely when the scheduler requests 2314 MHz.
	 */
	__cpufreq_driver_target(pp->policy, freq, CPUFREQ_RELATION_L);
	if (pp->cl1 && READ_ONCE(pp->started)) {
		struct pr_boost_state *b = &pp->boost;
		if (READ_ONCE(pp->policy->cur) > b->cfg.stages_khz[1])
			schedule_delayed_work(&b->spike_watchdog,
					msecs_to_jiffies(pr_top_burst_ms(&b->cfg)));
		else
			cancel_delayed_work(&b->spike_watchdog);
	}
	mutex_unlock(&pp->work_lock);
}

static void pr_irq_work(struct irq_work *work)
{
	struct pragmatic_policy *pp = container_of(work,
						  struct pragmatic_policy, irq_work);
	kthread_queue_work(&pp->worker, &pp->work);
}

static int pr_init(struct cpufreq_policy *policy)
{
	struct pragmatic_policy *pp;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	int ret;

	if (policy->governor_data)
		return -EBUSY;
	pp = kzalloc(sizeof(*pp), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;
	pp->policy = policy;
	pp->cl1 = cpumask_test_cpu(PR_CL1_FIRST_CPU, policy->related_cpus);
	pp->up_rate_us = 5000;
	pp->down_rate_us = 3000;
	raw_spin_lock_init(&pp->update_lock);
	mutex_init(&pp->work_lock);
	kthread_init_work(&pp->work, pr_work);
	kthread_init_worker(&pp->worker);
	init_irq_work(&pp->irq_work, pr_irq_work);
	if (pp->cl1) {
		pr_boost_init(pp);
		pr_cache_opps(pp);
	}

	pp->thread = kthread_create(kthread_worker_fn, &pp->worker,
				  "pragmatic:%d", cpumask_first(policy->related_cpus));
	if (IS_ERR(pp->thread)) {
		ret = PTR_ERR(pp->thread);
		goto fail;
	}
	ret = sched_setscheduler_nocheck(pp->thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(pp->thread);
		goto fail;
	}
	kthread_bind_mask(pp->thread, policy->related_cpus);
	wake_up_process(pp->thread);
	policy->governor_data = pp;
	return 0;
fail:
	mutex_destroy(&pp->work_lock);
	kfree(pp);
	return ret;
}

static void pr_exit(struct cpufreq_policy *policy)
{
	struct pragmatic_policy *pp = policy->governor_data;

	policy->governor_data = NULL;
	if (pp->cl1 && pp->boost.battery)
		power_supply_put(pp->boost.battery);
	kthread_flush_worker(&pp->worker);
	kthread_stop(pp->thread);
	mutex_destroy(&pp->work_lock);
	kfree(pp);
}

static int pr_start(struct cpufreq_policy *policy)
{
	struct pragmatic_policy *pp = policy->governor_data;
	unsigned int cpu;

	pp->work_in_progress = false;
	pp->limits_changed = true;
	WRITE_ONCE(pp->started, true);
	pp->next_freq = policy->cur;
	pp->last_freq_update_time = 0;
	for_each_cpu(cpu, policy->cpus) {
		struct pragmatic_cpu *pc = &per_cpu(pragmatic_cpu, cpu);
		memset(pc, 0, sizeof(*pc));
		pc->pp = pp;
		pc->iowait_boost_max = policy->cpuinfo.max_freq;
	}
	if (pp->cl1) {
		pp->boost.last_ns = ktime_get_ns();
		schedule_delayed_work(&pp->boost.work,
				      msecs_to_jiffies(PR_BOOST_FAST_MS));
	}
	for_each_cpu(cpu, policy->cpus) {
		struct pragmatic_cpu *pc = &per_cpu(pragmatic_cpu, cpu);
		cpufreq_add_update_util_hook(cpu, &pc->update_util, pr_update);
	}
	return 0;
}

static void pr_stop(struct cpufreq_policy *policy)
{
	struct pragmatic_policy *pp = policy->governor_data;
	unsigned int cpu;

	WRITE_ONCE(pp->started, false);
	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);
	synchronize_sched();
	if (pp->cl1) {
		cancel_delayed_work_sync(&pp->boost.work);
		cancel_delayed_work_sync(&pp->boost.spike_watchdog);
	}
	irq_work_sync(&pp->irq_work);
	kthread_cancel_work_sync(&pp->work);
}

static void pr_limits(struct cpufreq_policy *policy)
{
	struct pragmatic_policy *pp = policy->governor_data;
	unsigned long flags;

	mutex_lock(&pp->work_lock);
	cpufreq_policy_apply_limits(policy);
	mutex_unlock(&pp->work_lock);
	raw_spin_lock_irqsave(&pp->update_lock, flags);
	pp->limits_changed = true;
	raw_spin_unlock_irqrestore(&pp->update_lock, flags);
}

static struct cpufreq_governor pragmatic_gov = {
	.name = "pragmatic",
	.owner = THIS_MODULE,
	.init = pr_init,
	.exit = pr_exit,
	.start = pr_start,
	.stop = pr_stop,
	.limits = pr_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_PRAGMATIC
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &pragmatic_gov;
}
#endif

static int __init pragmatic_register(void)
{
	pr_info("Exynos9810 governor with integrated M3 turbo and thermal debt\n");
	return cpufreq_register_governor(&pragmatic_gov);
}
fs_initcall(pragmatic_register);
