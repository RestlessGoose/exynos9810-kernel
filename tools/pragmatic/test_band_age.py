#!/usr/bin/env python3
"""Host tests of extracted CFS band clock/publication code and reference histories.

No claim of kernel scheduling, lockdep, hotplug or real-device validation.
The reference model checks accounting-domain and quantization invariants; actual
helpers and throttle callbacks run with pthread mutexes standing in for raw locks.
"""
from pathlib import Path
import random
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def function(text, name):
    start = re.search(r'^[\w \t*]*\b' + re.escape(name) +
                      r'\([^;{}]*\)\s*\{', text, re.M).start()
    start = text.rfind('\n', 0, start) + 1
    opening = text.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


sched = (ROOT / 'kernel/sched/sched.h').read_text()
fair = (ROOT / 'kernel/sched/fair.c').read_text()
band = (ROOT / 'kernel/sched/ems/band.c').read_text()
helpers = sched[sched.index('struct cfs_band_time {'):sched.index('#define RQCF_REQ_SKIP')]
# The first band-clock occurrence is a declaration. Extract the two definitions.
clock_at = fair.index('static inline u64 cfs_rq_band_clock(struct cfs_rq *cfs_rq)\n{')
clock = function(fair[clock_at:], 'cfs_rq_band_clock')
throttle = function(fair, 'tg_throttle_down')
unthrottle = function(fair, 'tg_unthrottle_up')
stale = function(band, 'band_task_stale')
sync_throttle = function(fair, 'sync_throttle')
prelude = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <pthread.h>
typedef uint64_t u64;
typedef int64_t s64;
#define CONFIG_SCHED_EMS 1
#define CONFIG_FAIR_GROUP_SCHED 1
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define unlikely(x) (x)
#define raw_spin_lock_init(x) assert(!pthread_mutex_init(x,NULL))
#define raw_spin_lock_irqsave(x,f) do { (f)=0; assert(!pthread_mutex_lock(x)); } while (0)
#define raw_spin_unlock_irqrestore(x,f) do { (void)(f); assert(!pthread_mutex_unlock(x)); } while (0)
#define rcu_read_lock() do {} while(0)
#define rcu_read_unlock() do {} while(0)
struct cfs_rq;
struct cfs_band_age { pthread_mutex_t lock; u64 time,version; struct cfs_rq *domain; };
struct sched_avg { u64 last_update_time; struct cfs_band_age band_age; };
struct rq { u64 clock_task,clock_task_mult,clock; };
struct cfs_rq { struct sched_avg avg; struct rq *rq; int throttle_count;
 u64 throttled_clock_task,throttled_clock_task_time,throttled_clock,
 band_throttled_start,band_throttled_time; };
struct task_group { struct cfs_rq *cfs_rq[1]; struct task_group *parent; };
struct sched_entity { struct sched_avg avg; struct cfs_rq *cfs_rq; };
struct task_struct { struct sched_entity se; };
#define rq_of(q) ((q)->rq)
#define cpu_of(r) (0)
#define rq_clock_task(r) ((r)->clock_task)
#define rq_clock_task_mult(r) ((r)->clock_task_mult)
#define rq_clock(r) ((r)->clock)
static struct rq *test_rq;
#define cpu_rq(cpu) (test_rq)
#define cfs_bandwidth_used() (true)
static unsigned long out_of_time=100000000;
'''
main = r'''
static struct sched_avg concurrent;
static struct cfs_rq a,b;
static void *writer(void *unused) {
 (void)unused;
 for (u64 i=1;i<=200000;i++) cfs_band_age_set(&concurrent,&a,2*i,i);
 return NULL;
}
int main(void) {
 struct rq rq={0};
 struct cfs_rq q={.rq=&rq}; struct task_group tg={.cfs_rq={&q}};
 struct task_struct p={.se={.cfs_rq=&q}};
 cfs_band_age_init(&q.avg); cfs_band_age_init(&p.se.avg);
 assert(!band_task_stale(&p)); /* no sample is fresh */
 cfs_band_age_set(&p.se.avg,&q,2000000,1000000);
 cfs_band_age_set(&q.avg,&q,202000002,101000001);
 assert(band_task_stale(&p));
 /* No elapsed time: neither changing a slope nor a different scaled
    interval spanning a slope transition may change raw freshness. */
 cfs_band_age_set(&q.avg,&q,400000000,101000001);
 assert(band_task_stale(&p));
 cfs_band_age_set(&q.avg,&q,400000000,101000000);
 assert(!band_task_stale(&p));
 /* Invalidated class state cannot inherit a foreign RT origin. */
 cfs_band_age_invalidate(&p.se.avg);
 p.se.avg.last_update_time=123456789;
 assert(!band_task_stale(&p));
 /* Attachment/migration rebases both samples, not a scaled division. */
 p.se.cfs_rq=&b; cfs_band_age_init(&b.avg);
 cfs_band_age_set(&p.se.avg,&q,123,1);
 assert(!band_task_stale(&p));
 cfs_band_age_set(&b.avg,&b,1000,9000000000ULL);
 struct cfs_band_time t=cfs_band_age_read(&b.avg);
 cfs_band_age_set(&p.se.avg,&b,t.scaled,t.time);
 assert(!band_task_stale(&p));
 cfs_band_age_set(&b.avg,&b,200000000,9100000001ULL);
 assert(band_task_stale(&p));
 cfs_band_age_reset(&p.se.avg); assert(!band_task_stale(&p));
 /* Actual nested throttle callbacks freeze a CFS group's raw clock.
    Multiplier changes inside the frozen interval do not alter age. */
 rq.clock_task=100; rq.clock_task_mult=200;
 tg_throttle_down(&tg,&rq);
 rq.clock_task=150; rq.clock_task_mult=300;
 tg_throttle_down(&tg,&rq);
 rq.clock_task=200; rq.clock_task_mult=350;
 assert(cfs_rq_band_clock(&q)==100);
 tg_unthrottle_up(&tg,&rq); assert(cfs_rq_band_clock(&q)==100);
 rq.clock_task=250; rq.clock_task_mult=400;
 tg_unthrottle_up(&tg,&rq); assert(cfs_rq_band_clock(&q)==100);
 rq.clock_task=300; assert(cfs_rq_band_clock(&q)==150);
 /* sync_throttle's inherited count starts exclusion at child creation. */
 struct cfs_rq child={.rq=&rq};
 struct task_group child_tg={.cfs_rq={&child},.parent=&tg};
 q.throttle_count=2; test_rq=&rq;
 sync_throttle(&child_tg,0);
 assert(child.throttle_count==2 && child.band_throttled_start==300);
 assert(cfs_rq_band_clock(&child)==300);
 rq.clock_task=400; assert(cfs_rq_band_clock(&child)==300);
 /* Serialized publication must never tear a tuple. */
 cfs_band_age_init(&concurrent); pthread_t thread;
 assert(!pthread_create(&thread,NULL,writer,NULL));
 for (int i=0;i<200000;i++) {
  t=cfs_band_age_read(&concurrent);
  if(t.domain) assert(t.scaled==2*t.time && t.domain==&a);
 }
 assert(!pthread_join(thread,NULL));
 t=cfs_band_age_read(&concurrent);
 cfs_band_age_set(&concurrent,&b,10,5);
 cfs_band_age_set(&concurrent,&a,t.scaled,t.time);
 assert(t.version!=cfs_band_age_read(&concurrent).version); /* ABA */
 puts("PASS extracted helpers: freshness, initialization/class invalidation, migration, nested throttling, 400000 concurrent reads/writes, ABA detection");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='crowntrail-band-age-') as directory:
    source = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    source.write_text(prelude + helpers + clock + throttle + unthrottle + sync_throttle + stale + main)
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-O2',
                    '-pthread', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

# Independent piecewise accounting reference: raw time excludes the UNION of
# nested throttle intervals, not the sum of each ancestor's excluded interval.
rng = random.Random(9810)
for shifts in [(0,), (1,), (1, 0), (0, 1), (1, 0, 1, 0)]:
    raw = scaled = excluded_raw = excluded_scaled = 0
    count = 0
    frozen = None
    expected = 0
    previous = 0
    for i in range(20000):
        shift = shifts[(i // 1000) % len(shifts)]
        if count and rng.randrange(7) == 0:
            count -= 1
            if not count:
                excluded_raw += raw - frozen[0]
                excluded_scaled += scaled - frozen[1]
        elif count < 4 and rng.randrange(11) == 0:
            if not count:
                frozen = (raw, scaled)
            count += 1
        dt = rng.randrange(0, 100001)
        raw += dt
        scaled += dt << shift
        if not count:
            expected += dt
        observed = (frozen[0] if count else raw) - excluded_raw
        assert observed == expected and observed >= previous
        previous = observed

# Quantization equivalence at a fixed slope: each PELT publication is <1024
# scaled ns behind its raw sampling event. Pair differences differ by <1024
# raw ns at 1x and <512 raw ns at 2x. Compare strict > threshold predicates.
threshold = 100000000
max_error = {}
for shift in (0, 1):
    worst = 0
    for _ in range(100000):
        origin = rng.randrange(0, 1024)
        t1 = rng.randrange(1, 1000000000)
        t2 = t1 + threshold + rng.randrange(-2000, 2001)
        scaled1 = origin + (((t1 << shift) - origin) >> 10 << 10)
        scaled2 = scaled1 + (((t2 << shift) - scaled1) >> 10 << 10)
        error = abs((scaled2-scaled1) - ((t2-t1) << shift))
        assert error < 1024
        old = scaled2-scaled1 > threshold << shift
        new = t2-t1 > threshold
        if old != new:
            assert abs(t2-t1-threshold) < (1024 >> shift)
        worst = max(worst, error)
    max_error[1 << shift] = worst
print('PASS piecewise/nested-throttle model: 100000 steps; constant slope: 200000 cases; max scaled rounding error ns:', max_error)
print('LIMIT: host models and extracted helpers do not validate kernel lockdep, scheduling, hotplug, or real task-placement behavior')

# Execute both the original and modified PELT recurrence with the repository's
# real decay tables and accumulator. Only the unchanged EMS multi-load callback
# is stubbed: it is not part of the band timestamp calculation.
baseline = subprocess.check_output(
    ['git', 'show', '5ee636305a6bb1cf8957fbf6b3c38d30382ec411:kernel/sched/fair.c'],
    cwd=ROOT, text=True)
common = (function(fair, 'decay_load') + '\n' +
          function(fair, '__accumulate_sum') + '\n' +
          'static inline u32 ' + function(fair, 'accumulate_sum') + '\n')
old_update = 'static inline int ' + function(baseline, '___update_load_avg')
old_update = old_update.replace('___update_load_avg(', 'old_update(')
new_update = 'static inline int ' + function(fair, '___update_load_avg')
new_update = new_update.replace('___update_load_avg(', 'new_update(')
math_prelude = prelude.replace('typedef int64_t s64;', 'typedef int64_t s64; typedef uint32_t u32;')
math_prelude = math_prelude.replace('u64 last_update_time; struct cfs_band_age band_age;',
    'u64 last_update_time, load_sum; u32 util_sum, period_contrib; unsigned long load_avg,util_avg; struct cfs_band_age band_age;')
math_prelude = math_prelude.replace('struct cfs_rq { struct sched_avg avg;',
    'struct cfs_rq { u64 runnable_load_sum, runnable_load_avg; struct sched_avg avg;')
math_prelude = math_prelude.replace('static unsigned long out_of_time=100000000;', '').replace('static struct rq *test_rq;', '')
math_prelude += r'''
#define likely(x) (x)
#define SCHED_CAPACITY_SHIFT 10
#define cap_scale(v,s) ((v)*(s)>>10)
#define arch_scale_freq_capacity(sd,cpu) (768UL)
#define arch_scale_cpu_capacity(sd,cpu) ((cpu)?1024UL:512UL)
#define mul_u64_u32_shr(a,b,s) ((u64)(((__uint128_t)(a)*(b))>>(s)))
#define div_u64(a,b) ((a)/(b))
#define update_multi_load(...) do {} while(0)
struct rt_rq { int unused; };
'''
lifecycle = r'''
static bool attach_age_load=true;
#define sched_feat(x) (attach_age_load)
#define set_tg_cfs_propagate(...) do {} while(0)
#define attach_entity_multi_load(...) do {} while(0)
#define detach_entity_multi_load(...) do {} while(0)
#define cfs_rq_util_change(...) do {} while(0)
#define sub_positive(p,v) do { if (*(p)>(v)) *(p)-=(v); else *(p)=0; } while(0)
static struct cfs_band_time cfs_rq_load_time(struct cfs_rq *q) {
 return cfs_band_age_read(&q->avg);
}
''' + ('static int ' + function(fair, '__update_load_avg_blocked_se')).replace(
    '___update_load_avg(', 'new_update(') + function(fair, 'set_task_rq_fair') + \
    function(fair, 'attach_entity_load_avg') + function(fair, 'detach_entity_load_avg')
math_main = r'''
static void equal_avg(struct sched_avg *a,struct sched_avg *b) {
 assert(a->last_update_time==b->last_update_time);
 assert(a->load_sum==b->load_sum && a->load_avg==b->load_avg);
 assert(a->util_sum==b->util_sum && a->util_avg==b->util_avg);
 assert(a->period_contrib==b->period_contrib);
}
int main(void) {
 struct sched_entity entity={0}; struct cfs_rq src={0},dst={0};
 cfs_band_age_init(&entity.avg); cfs_band_age_init(&src.avg); cfs_band_age_init(&dst.avg);
 cfs_band_age_set(&src.avg,&src,200000000,100000000);
 cfs_band_age_set(&dst.avg,&dst,9000000000ULL,8000000000ULL);
 set_task_rq_fair(&entity,NULL,&src);
 assert(!cfs_band_age_read(&entity.avg).domain && !entity.avg.last_update_time);
 attach_entity_load_avg(&src,&entity);
 assert(cfs_band_age_read(&entity.avg).time==100000000);
 detach_entity_load_avg(&src,&entity);
 assert(cfs_band_age_read(&entity.avg).domain==&src);
 set_task_rq_fair(&entity,&src,&dst);
 struct cfs_band_time migrated=cfs_band_age_read(&entity.avg);
 assert(migrated.domain==&dst && migrated.time==8000000000ULL && migrated.scaled==9000000000ULL);
 attach_age_load=false;
 set_task_rq_fair(&entity,&dst,&src);
 assert(!cfs_band_age_read(&entity.avg).domain);
 attach_entity_load_avg(&src,&entity);
 assert(cfs_band_age_read(&entity.avg).domain==&src);
 attach_age_load=true;
 for(unsigned int shift=0;shift<=1;shift++) {
  struct sched_avg old={0},new={0}; struct cfs_rq q1={0},q2={0};
  cfs_band_age_init(&new); u64 raw=0, scaled=0;
  for (unsigned int i=0;i<100000;i++) {
   u64 dt=(i*7919ULL)%5000000;
   raw+=dt;scaled+=dt<<shift;
   unsigned long weight=(i/200)%2?0:1024; int running=(i/100)%2;
   int x=old_update(scaled,shift,&old,weight,running,&q1,NULL);
   int y=new_update(scaled,shift,&new,weight,running,&q2,NULL,raw,&q2);
   assert(x==y);equal_avg(&old,&new);
   assert(q1.runnable_load_sum==q2.runnable_load_sum);
   assert(q1.runnable_load_avg==q2.runnable_load_avg);
   struct cfs_band_time t=cfs_band_age_read(&new);
   if(t.domain) assert(t.time<=raw && t.scaled<=scaled);
  }
 }
 puts("PASS extracted attachment/detachment/migration/ATTACH_AGE_LOAD paths; real PELT recurrence: 200000 old/new ramp-up, ramp-down and blocked updates at constant 1x/2x; all numeric sums, averages and timestamps identical");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='crowntrail-band-pelt-') as directory:
    source = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    source.write_text(math_prelude + (ROOT / 'kernel/sched/sched-pelt.h').read_text() +
                      helpers + common + old_update + new_update + lifecycle + math_main)
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-O2', '-pthread', str(source),
                    '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

# Concrete old misclassifications for an interval straddling a slope change.
for segments, current_shift in [([(40000000, 1), (40000000, 0)], 0),
                                ([(60000000, 0), (60000000, 1)], 1)]:
    elapsed = sum(dt for dt, _ in segments)
    scaled_elapsed = sum(dt << shift for dt, shift in segments)
    assert (scaled_elapsed > threshold << current_shift) != (elapsed > threshold)
    # Changing just the selected multiplier cannot change new age.
    assert (elapsed > threshold) == (sum(dt for dt, _ in segments) > threshold)
print('PASS explicit 2x->1x false-stale and 1x->2x false-fresh reproductions')
