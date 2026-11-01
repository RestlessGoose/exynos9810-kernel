/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PRAGMATIC_DIAG_H
#define _LINUX_PRAGMATIC_DIAG_H
#include <linux/types.h>
#include <linux/jump_label.h>
#include <linux/ktime.h>

enum pr_diag_event {
	PRD_UTIL_CALLBACKS,
	PRD_COMMIT_CALCULATIONS,
	PRD_DESIRED_CHANGES,
	PRD_EFFECTIVE_CHANGES,
	PRD_DUPLICATE_PENDING,
	PRD_ACHIEVED_SUPPRESSED,
	PRD_RATE_EARLY,
	PRD_RATE_COMMIT,
	PRD_QUEUED_TARGET_UPDATES,
	PRD_INFLIGHT_TARGET_UPDATES,
	PRD_FORCED_UPDATES,
	PRD_RETRIES,
	PRD_RECONCILIATION_ATTEMPTS,
	PRD_RECONCILED,
	PRD_WORK_SUBMISSIONS,
	PRD_UTIL_SUBMISSIONS,
	PRD_CAP_SUBMISSIONS,
	PRD_IRQ_CALLBACKS,
	PRD_QUEUE_ACCEPTED,
	PRD_WORKER_EXECUTIONS,
	PRD_WORKER_NO_CHANGE,
	PRD_DRIVER_CALLS,
	PRD_DRIVER_ERRORS,
	PRD_DRIVER_NO_CHANGE,
	PRD_REQUEST_MISMATCH,
	PRD_ABSOLUTE_ERROR_KHZ_SUM,
	PRD_REQUESTED_KHZ_SUM,
	PRD_REPORTED_KHZ_SUM,
	PRD_WATCHDOG_RUNS,
	PRD_BOOST_RUNS,
	PRD_LIMITS_CALLS,
	PRD_QOS_CALLBACKS,
	PRD_QOS_UPDATES,
	PRD_PE_LIMIT_CHANGES,
	PRD_ACME_ATTEMPTS,
	PRD_ACME_ERRORS,
	PRD_ACPM_POLL_REQUESTS,
	PRD_ACPM_NONPOLL_REQUESTS,
	PRD_ACPM_SLEEP_REQUESTS,
	PRD_ACPM_ERRORS,
	PRD_ACPM_DEADLINE_EVENTS,
	PRD_ACPM_POLL_LOOPS,
	PRD_ACPM_UDELAY_CALLS,
	PRD_ACPM_USLEEP_CALLS,
	PRD_EVENTS
};
enum pr_diag_metric {
	PRD_TIME_WORKER,
	PRD_TIME_DRIVER,
	PRD_TIME_OUTSIDE,
	PRD_TIME_DISPATCH,
	PRD_TIME_ACME,
	PRD_TIME_ACPM_POLL,
	PRD_TIME_ACPM_NONPOLL,
	PRD_METRICS
};
struct pr_diag_stamp { u64 ns; unsigned long epoch; };
struct pr_diag_ipc {
	struct pr_diag_stamp stamp;
	unsigned int cpu;
	u64 loops, delays, sleeps;
	bool polling;
};
#ifdef CONFIG_CPU_FREQ_GOV_PRAGMATIC
DECLARE_STATIC_KEY_FALSE(pr_diag_key);
void __pr_diag_add(unsigned int cpu, enum pr_diag_event event, u64 value);
struct pr_diag_stamp __pr_diag_begin(void);
bool __pr_diag_valid(struct pr_diag_stamp stamp);
unsigned long pr_diag_epoch(void);
void __pr_diag_add_stamp(unsigned int cpu, enum pr_diag_event event, u64 value,
		       struct pr_diag_stamp stamp);
void __pr_diag_time(unsigned int cpu, enum pr_diag_metric metric,
		  struct pr_diag_stamp stamp, u64 ns);
void pr_diag_set_channel(unsigned int channel);
struct pr_diag_ipc __pr_diag_ipc_begin(unsigned int channel, const unsigned int *cmd,
				   bool response, bool polling, bool sleep_mode);
void __pr_diag_ipc_end(struct pr_diag_ipc *d, int ret);
static inline bool pr_diag_on(void)
{
	return static_branch_unlikely(&pr_diag_key);
}
static inline bool pr_diag_valid(struct pr_diag_stamp s)
{
	return s.epoch && pr_diag_on() && __pr_diag_valid(s);
}
static inline void pr_diag_add_stamp(unsigned int cpu, enum pr_diag_event e,
				     u64 value, struct pr_diag_stamp s)
{
	if (s.epoch && pr_diag_on())
		__pr_diag_add_stamp(cpu, e, value, s);
}
static inline void pr_diag_time(unsigned int cpu, enum pr_diag_metric m,
			       struct pr_diag_stamp s, u64 ns)
{
	if (s.epoch && pr_diag_on())
		__pr_diag_time(cpu, m, s, ns);
}
static inline struct pr_diag_ipc pr_diag_ipc_begin(unsigned int channel,
		const unsigned int *cmd, bool response, bool polling, bool sleep_mode)
{
	if (pr_diag_on())
		return __pr_diag_ipc_begin(channel, cmd, response, polling, sleep_mode);
	return (struct pr_diag_ipc) { .stamp = { 0, 0 } };
}
static inline void pr_diag_ipc_end(struct pr_diag_ipc *d, int ret)
{
	if (d->stamp.epoch && pr_diag_on())
		__pr_diag_ipc_end(d, ret);
}
static inline void pr_diag_add(unsigned int cpu, enum pr_diag_event event, u64 value)
{
	if (pr_diag_on())
		__pr_diag_add(cpu, event, value);
}
static inline struct pr_diag_stamp pr_diag_begin(void)
{
	if (pr_diag_on())
		return __pr_diag_begin();
	return (struct pr_diag_stamp) { 0, 0 };
}
#else
static inline bool pr_diag_on(void) { return false; }
static inline void pr_diag_add(unsigned int c, enum pr_diag_event e, u64 v) { }
static inline struct pr_diag_stamp pr_diag_begin(void)
{ return (struct pr_diag_stamp) { 0, 0 }; }
static inline bool pr_diag_valid(struct pr_diag_stamp s) { return false; }
static inline unsigned long pr_diag_epoch(void) { return 0; }
static inline void pr_diag_add_stamp(unsigned int c, enum pr_diag_event e,
		u64 v, struct pr_diag_stamp s) { }
static inline void pr_diag_time(unsigned int c, enum pr_diag_metric m,
			       struct pr_diag_stamp s, u64 n) { }
static inline void pr_diag_set_channel(unsigned int c) { }
static inline struct pr_diag_ipc pr_diag_ipc_begin(unsigned int c,
		const unsigned int *cmd, bool r, bool p, bool w)
{ return (struct pr_diag_ipc) { .stamp = { 0, 0 } }; }
static inline void pr_diag_ipc_end(struct pr_diag_ipc *d, int r) { }
#endif
#endif
