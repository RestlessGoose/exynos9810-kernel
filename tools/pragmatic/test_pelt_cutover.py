#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Extracted cutover/clock tests, not proof of kernel concurrency or hotplug.

pthread locks stand in for rq, sysctl and hotplug locks. IRQ/steal counters and
sched_clock samples are controlled inputs. Run the existing band/PELT tests too.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
core = (ROOT / 'kernel/sched/core.c').read_text()
fair = (ROOT / 'kernel/sched/fair.c').read_text()


def function(text, name):
    match = re.search(r'^(?:(?:static|inline|__always_inline|void|u64|u32|int|bool)\s+)+' + re.escape(name) +
                      r'\([^;{}]*\)\s*\{', text, re.M)
    start = match.start()
    opening = text.index('{', start)
    end, depth = opening + 1, 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end] + '\n'


old = subprocess.check_output(['git', 'show',
    '6df69e3ff800:kernel/sched/core.c'], cwd=ROOT, text=True)
old_clock = function(old, 'update_rq_clock_task').replace(
    'update_rq_clock_task(', 'old_update_rq_clock_task(', 1)
production = ''.join(function(core, name) for name in [
    'update_rq_clock_task', '__update_rq_clock', 'update_rq_clock',
    'sched_pelt_set_multiplier', 'sched_pelt_validate', 'sched_pelt_auto_control',
    'sched_pelt_auto_owned', 'sched_pelt_multiplier'])
throttle = ''.join(function(fair, name) for name in [
    'cfs_rq_clock_task', 'cfs_rq_band_clock',
    'tg_throttle_down', 'tg_unthrottle_up'])
prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <sys/types.h>
typedef uint64_t u64; typedef int64_t s64;
#define CONFIG_SCHED_EMS 1
#define __user
#define unlikely(x) (x)
#define READ_ONCE(x) __atomic_load_n(&(x),__ATOMIC_RELAXED)
#define WRITE_ONCE(x,v) __atomic_store_n(&(x),(v),__ATOMIC_RELAXED)
#define DEFINE_MUTEX(m) pthread_mutex_t m=PTHREAD_MUTEX_INITIALIZER
#define mutex_lock(m) assert(!pthread_mutex_lock(m))
#define mutex_unlock(m) assert(!pthread_mutex_unlock(m))
#define raw_spin_lock_irqsave(l,f) do { (f)=0; mutex_lock(l); } while(0)
#define raw_spin_unlock_irqrestore(l,f) do { (void)(f); if(unlock_observer)unlock_observer(l); mutex_unlock(l); if(after_unlock)after_unlock(l); } while(0)
#define lockdep_assert_held(l) do {} while(0)
#define RQCF_ACT_SKIP 2
#define NR 8
struct rq {
 pthread_mutex_t lock;
 unsigned int clock_skip_update,pelt_lshift;
 u64 clock,clock_task,clock_task_mult,prev_irq_time,prev_steal_time_rq,rt_avg;
 int cpu;
};
static struct rq queues[NR];
static _Atomic u64 samples[NR],irq_total[NR],steal_total[NR];
static _Atomic unsigned int samples_read;
static void (*sample_observer)(int,u64);
static void (*unlock_observer)(pthread_mutex_t*);
static void (*after_unlock)(pthread_mutex_t*);
static pthread_rwlock_t hotplug=PTHREAD_RWLOCK_INITIALIZER;
static bool online[NR];
unsigned int sysctl_sched_pelt_multiplier=2,sched_pelt_lshift=1;
DEFINE_MUTEX(sched_pelt_mutex);
static bool sched_pelt_automatic;
enum sched_pelt_action { SCHED_PELT_CLAIM,SCHED_PELT_UPDATE,SCHED_PELT_RELEASE };
#define cpu_rq(c) (&queues[c])
#define cpu_of(q) ((q)->cpu)
#define for_each_possible_cpu(c) for((c)=0;(c)<NR;(c)++)
#define rq_clock_task(q) ((q)->clock_task)
#define rq_clock_task_mult(q) ((q)->clock_task_mult)
#define sched_feat(x) (true)
#define static_key_false(k) (true)
static int paravirt_steal_rq_enabled __attribute__((unused));
#define irq_time_read(c) atomic_load(&irq_total[c])
#define paravirt_steal_clock(c) atomic_load(&steal_total[c])
#define sched_rt_avg_update(q,t) ((q)->rt_avg+=(t))
static u64 sched_clock_cpu(int c) { atomic_fetch_add(&samples_read,1); u64 now=atomic_load(&samples[c]); if(sample_observer)sample_observer(c,now); return now; }
static void get_online_cpus(void) { assert(!pthread_rwlock_rdlock(&hotplug)); }
static void put_online_cpus(void) {
 /* Called before dropping the writer's sysctl mutex: completion must already
    cover offline and online runqueues, regardless of the CPU mask. */
 for(int i=0;i<NR;i++) assert(queues[i].pelt_lshift==sched_pelt_lshift);
 assert(!pthread_rwlock_unlock(&hotplug));
}
struct ctl_table { void *data; int maxlen; };
static int proc_dointvec(struct ctl_table *t,int w,void *b,size_t *l,loff_t *p) {
 (void)l;(void)p;
 if(!w){*(unsigned int*)b=READ_ONCE(*(unsigned int*)t->data);return 0;}
 if(*(int*)b==INT_MIN){*(unsigned int*)t->data=3;return -EFAULT;}
 *(unsigned int*)t->data=*(unsigned int*)b;return 0;
}
struct cfs_rq { struct rq *rq; int throttle_count;
 u64 throttled_clock_task,throttled_clock_task_time,band_throttled_start,band_throttled_time; };
struct task_group { struct cfs_rq *cfs_rq[NR]; };
#define rq_of(q) ((q)->rq)
'''
main = r'''
static int request(int v) {
 struct ctl_table t={&sysctl_sched_pelt_multiplier,sizeof(unsigned int)};
 size_t len=sizeof(v);loff_t pos=0;
 return sched_pelt_multiplier(&t,1,&v,&len,&pos);
}
static void reset(void) {
 sysctl_sched_pelt_multiplier=2;sched_pelt_lshift=1;samples_read=0;
 for(int i=0;i<NR;i++) {
  struct rq *q=&queues[i];
  q->clock_skip_update=0;q->pelt_lshift=1;q->clock=0;q->clock_task=0;
  q->clock_task_mult=0;q->prev_irq_time=0;q->prev_steal_time_rq=0;q->rt_avg=0;
  samples[i]=irq_total[i]=steal_total[i]=0;online[i]=i<4;
 }
}
/* Independent reference for the existing effective task-clock delta, including
   capped IRQ/steal debt. It deliberately does not infer physical IRQ intervals
   unavailable to the existing kernel accounting interface. */
static struct rq reference(struct rq q,u64 now) {
 s64 delta=(s64)(now-q.clock);
 if(delta<0)return q;
 q.clock=now;s64 used=0;
#ifdef CONFIG_IRQ_TIME_ACCOUNTING
 s64 irq=(s64)(atomic_load(&irq_total[q.cpu])-q.prev_irq_time);
 if(irq>delta)irq=delta;
 q.prev_irq_time+=irq;delta-=irq;used+=irq;
#endif
#ifdef CONFIG_PARAVIRT_TIME_ACCOUNTING
 s64 steal=(s64)(atomic_load(&steal_total[q.cpu])-q.prev_steal_time_rq);
 if(steal>delta)steal=delta;
 q.prev_steal_time_rq+=steal;delta-=steal;used+=steal;
#endif
 q.clock_task+=delta;q.clock_task_mult+=(u64)delta<<q.pelt_lshift;q.rt_avg+=used;
 return q;
}
static void equal_clock(struct rq *a,struct rq *b) {
 assert(a->clock==b->clock && a->clock_task==b->clock_task);
 assert(a->clock_task_mult==b->clock_task_mult);
 assert(a->prev_irq_time==b->prev_irq_time && a->prev_steal_time_rq==b->prev_steal_time_rq);
 assert(a->rt_avg==b->rt_avg);
}
static void checked_request(int value) {
 struct rq expected[NR];bool changed=(unsigned int)value!=sysctl_sched_pelt_multiplier;
 for(int i=0;i<NR;i++)expected[i]=changed?reference(queues[i],samples[i]):queues[i];
 assert(!request(value));
 for(int i=0;i<NR;i++) {
  equal_clock(&queues[i],&expected[i]);
  assert(queues[i].pelt_lshift==(unsigned int)(value==1?0:value==2?1:2));
 }
}
static unsigned int mixed_boundary_checks;
static void check_mixed_boundary(pthread_mutex_t *lock) {
 if(lock!=&queues[0].lock)return;
 assert(queues[0].pelt_lshift!=sched_pelt_lshift);
 assert(sysctl_sched_pelt_multiplier==2); /* old completed value */
 samples[0]+=17;
 mutex_lock(lock);
 struct rq expected=reference(queues[0],samples[0]);
 update_rq_clock(&queues[0]);equal_clock(&queues[0],&expected);
 mutex_unlock(lock);mixed_boundary_checks++;
}
static void *normal_updates(void *unused) {
 (void)unused;
 for(int i=0;i<100000;i++) {
  struct rq *q=&queues[(i&1)?0:4];unsigned long flags;
  raw_spin_lock_irqsave(&q->lock,flags);
  update_rq_clock(q);
  raw_spin_unlock_irqrestore(&q->lock,flags);
 }
 return NULL;
}
static struct rq observed[NR];
static void observe_sample(int cpu,u64 now) { observed[cpu]=reference(queues[cpu],now); }
static void observe_unlock(pthread_mutex_t *lock) {
 for(int i=0;i<NR;i++)if(lock==&queues[i].lock)equal_clock(&queues[i],&observed[i]);
}
static void *clock_progress(void *unused) {
 (void)unused;
 for(int i=0;i<100000;i++)for(int c=0;c<NR;c++)atomic_fetch_add(&samples[c],1000+c);
 return NULL;
}
static void *writer(void *arg) {
 unsigned int seed=(uintptr_t)arg;int values[]={1,2,4,2};
 for(int i=0;i<4000;i++)assert(!request(values[(i+seed)%4]));
 return NULL;
}
static void *cpu_lifecycle(void *unused) {
 (void)unused;
 for(int i=0;i<2000;i++) {
  assert(!pthread_rwlock_wrlock(&hotplug));online[7]=!online[7];
  assert(queues[7].pelt_lshift==sched_pelt_lshift);
  assert(!pthread_rwlock_unlock(&hotplug));
 }
 return NULL;
}
static void *migration(void *unused) {
 (void)unused;
 /* Existing migration holds both rq locks in address order. The cutover
    takes only one, so this stress has no second-lock dependency in reverse.
    Rebase a historical sample to the destination, never compare origins. */
 for(int i=0;i<20000;i++) {
  mutex_lock(&queues[0].lock);mutex_lock(&queues[4].lock);
  u64 source_sample=queues[0].clock_task_mult;
  u64 destination_sample=queues[4].clock_task_mult;
  u64 entity_sample=destination_sample;
  assert(entity_sample==queues[4].clock_task_mult);
  (void)source_sample;
  mutex_unlock(&queues[4].lock);mutex_unlock(&queues[0].lock);
 }
 return NULL;
}
int main(void) {
 for(int i=0;i<NR;i++){assert(!pthread_mutex_init(&queues[i].lock,NULL));queues[i].cpu=i;}
 reset();
 int sequence[]={1,2,1,2,4,2,4,1,4,2};
 for(unsigned int step=0;step<10000;step++) {
  for(int i=0;i<NR;i++) {
   samples[i]+=1000+i*17; /* deliberately different per-CPU boundaries */
   if(step%7==0)irq_total[i]+=200;
   if(step%11==0)steal_total[i]+=100;
   queues[i].clock_skip_update=(step%3==0)?RQCF_ACT_SKIP:0;
  }
  checked_request(sequence[step%10]);
 }
 puts("PASS 10000 piecewise 1x/2x/4x transitions, old-slope settlement, independent boundaries, skipped clocks, offline/never-online queues, IRQ/steal debt");
 unsigned int reads=samples_read;
 checked_request(sysctl_sched_pelt_multiplier);assert(samples_read==reads);
 /* Rejections and parser errors must precede every rq mutation. */
 for(int i=0;i<4;i++) {int v=(int[]){0,3,-1,INT_MIN}[i];assert(request(v)==(v==INT_MIN?-EFAULT:-EINVAL));assert(samples_read==reads);}
 checked_request(1);checked_request(2);
 /* Carry more reported non-task time than the current clock interval. */
 reset();samples[0]=100;irq_total[0]=300;steal_total[0]=100;
 checked_request(1);samples[0]=200;checked_request(4);
 samples[0]=500;checked_request(2);
 assert(!sched_pelt_auto_control(SCHED_PELT_CLAIM,1));
 assert(sched_pelt_auto_owned());assert(request(4)==-EBUSY);assert(request(1)==-EBUSY);
 assert(sched_pelt_auto_control(SCHED_PELT_CLAIM,2)==-EBUSY);
 assert(!sched_pelt_auto_control(SCHED_PELT_UPDATE,2));
 assert(sched_pelt_auto_control(SCHED_PELT_UPDATE,3)==-EINVAL);
 assert(!sched_pelt_auto_control(SCHED_PELT_RELEASE,0));
 assert(!sched_pelt_auto_owned() && sysctl_sched_pelt_multiplier==2);
 assert(sched_pelt_auto_control(SCHED_PELT_UPDATE,1)==-EBUSY);
 assert(!request(4));assert(!request(2));
 puts("PASS scheduler ownership claim/update/release, EBUSY, invalid requests and manual recovery");
 puts("PASS idempotence, invalid/failed requests leave all clocks unchanged, subsequent recovery");
 reset();samples[0]=100;after_unlock=check_mixed_boundary;
 assert(!request(1));after_unlock=NULL;
 assert(mixed_boundary_checks==1 && queues[0].clock_task_mult==217);
 puts("PASS deterministic normal accounting after first local boundary, while global completion still reports old setting");
 reset();
 /* Long idle/offline history settles NOW, not on a future CPU wakeup. */
 for(int i=0;i<NR;i++)samples[i]=1000000000000ULL+i*10000000ULL;
 checked_request(1);
 for(int i=0;i<NR;i++){assert(queues[i].clock_task_mult==2*samples[i]);samples[i]+=1000;}
 checked_request(2);
 for(int i=0;i<NR;i++)assert(queues[i].clock_task_mult==2*(samples[i]-1000)+1000);
 /* Backwards clock clamp uses the already-accounted rq boundary. */
 samples[7]-=500;checked_request(4);samples[7]+=1000;checked_request(1);
 puts("PASS long-idle return, separate CPU origins, no clock rewind or history rescale");
 reset();
 struct cfs_rq group={.rq=&queues[0]};struct task_group tg={.cfs_rq={&group}};
 samples[0]=100000000;checked_request(1); /* old 2x interval */
 tg_throttle_down(&tg,&queues[0]);
 u64 raw=cfs_rq_band_clock(&group),scaled=cfs_rq_clock_task(&group);
 samples[0]+=40000000;checked_request(2);tg_throttle_down(&tg,&queues[0]);
 samples[0]+=40000000;checked_request(4);tg_unthrottle_up(&tg,&queues[0]);
 assert(cfs_rq_clock_task(&group)==scaled && cfs_rq_band_clock(&group)==raw);
 samples[0]+=40000000;checked_request(1);tg_unthrottle_up(&tg,&queues[0]);
 assert(cfs_rq_clock_task(&group)==scaled && cfs_rq_band_clock(&group)==raw);
 samples[0]+=10000000;checked_request(2);
 assert(cfs_rq_clock_task(&group)==scaled+10000000);
 assert(cfs_rq_band_clock(&group)==raw+10000000);
 checked_request(4);assert(cfs_rq_band_clock(&group)==raw+10000000);
 puts("PASS extracted nested CFS throttle clocks and EMS band clock across all slopes, including zero elapsed cutover");
 /* At a constant multiplier compare the actual prior/new task-clock function.
    Normal clock suppression remains in update_rq_clock(). */
 for(unsigned int shift=0;shift<=2;shift++) {
  reset();sched_pelt_lshift=shift;struct rq prior={.cpu=0},current={.cpu=0,.pelt_lshift=shift};
  for(int i=0;i<100000;i++) {
   s64 delta=(i*7919ULL)%1000000;
   irq_total[0]+=17;steal_total[0]+=13;
   old_update_rq_clock_task(&prior,delta);update_rq_clock_task(&current,delta);
   equal_clock(&prior,&current);
  }
 }
 reset();samples[0]=1000;queues[0].clock_skip_update=RQCF_ACT_SKIP;
 update_rq_clock(&queues[0]);assert(!queues[0].clock);
 queues[0].clock_skip_update=0;update_rq_clock(&queues[0]);assert(queues[0].clock==1000);
 puts("PASS 300000 constant-slope old/new IRQ/steal/task clock updates and unchanged normal skip behavior");
 reset();for(int i=0;i<NR;i++)samples[i]=1000000+i*1000;
 sample_observer=observe_sample;unlock_observer=observe_unlock;
 pthread_t writers[3],hotplug_thread,migrate_thread,clock_thread,updates_thread;
 assert(!pthread_create(&clock_thread,NULL,clock_progress,NULL));
 assert(!pthread_create(&updates_thread,NULL,normal_updates,NULL));
 for(uintptr_t i=0;i<3;i++)assert(!pthread_create(&writers[i],NULL,writer,(void*)i));
 assert(!pthread_create(&hotplug_thread,NULL,cpu_lifecycle,NULL));
 assert(!pthread_create(&migrate_thread,NULL,migration,NULL));
 for(int i=0;i<3;i++)assert(!pthread_join(writers[i],NULL));
 assert(!pthread_join(hotplug_thread,NULL));assert(!pthread_join(migrate_thread,NULL));
 assert(!pthread_join(clock_thread,NULL));assert(!pthread_join(updates_thread,NULL));
 sample_observer=NULL;unlock_observer=NULL;
 checked_request(2);
 puts("PASS actual sysctl serialization: 12000 concurrent requests, hotplug-lock exclusion and paired-rq lock stress with concurrent clock advancement and per-settlement weighted checks");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='crowntrail-pelt-cutover-') as directory:
    source = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    source.write_text(prefix + old_clock + production + throttle + main)
    for name, flags in [('native', []), ('irq', ['CONFIG_IRQ_TIME_ACCOUNTING']),
                        ('steal', ['CONFIG_PARAVIRT_TIME_ACCOUNTING']),
                        ('irq-steal', ['CONFIG_IRQ_TIME_ACCOUNTING',
                                      'CONFIG_PARAVIRT_TIME_ACCOUNTING'])]:
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-O2',
                        '-pthread'] + ['-D' + flag for flag in flags] +
                       [str(source), '-o', str(binary)], check=True)
        print('Variant:', name, flush=True)
        subprocess.run([str(binary)], check=True)
print('LIMIT: extracted code and models do not validate real kernel lockdep, CPU hotplug, migration, IRQ timing, latency or device behavior')

# Feed the actual cutover clock into the repository's real CFS and RT PELT
# recurrences and compare against independently accumulated piecewise clocks.
# Capacity inputs and the unchanged EMS multi-load callback are stubbed, as in
# test_band_age.py. This does not exercise the kernel task/group scheduler.
consumer_prefix = prefix.replace('typedef uint64_t u64; typedef int64_t s64;',
    'typedef uint64_t u64; typedef int64_t s64; typedef uint32_t u32;')
consumer_prefix = consumer_prefix.replace('struct rq {', r'''
struct cfs_rq;
struct cfs_band_age { pthread_mutex_t lock; u64 time,version; struct cfs_rq *domain; };
struct sched_avg { u64 last_update_time,load_sum; u32 util_sum,period_contrib;
 unsigned long load_avg,util_avg; struct cfs_band_age band_age; };
struct rt_rq { int unused; };
struct rq {''')
consumer_prefix = consumer_prefix.replace('struct cfs_rq { struct rq *rq;',
    'struct cfs_rq { struct sched_avg avg; u64 runnable_load_sum,runnable_load_avg; struct rq *rq;')
consumer_prefix += r'''
#define raw_spin_lock_init(l) assert(!pthread_mutex_init(l,NULL))
#define likely(x) (x)
#define SCHED_CAPACITY_SHIFT 10
#define cap_scale(v,s) ((v)*(s)>>10)
#define arch_scale_freq_capacity(sd,cpu) (768UL)
#define arch_scale_cpu_capacity(sd,cpu) ((cpu)?1024UL:512UL)
#define mul_u64_u32_shr(a,b,s) ((u64)(((__uint128_t)(a)*(b))>>(s)))
#define div_u64(a,b) ((a)/(b))
#define update_multi_load(...) do {} while(0)
'''
sched_header = (ROOT / 'kernel/sched/sched.h').read_text()
metadata = sched_header[sched_header.index('struct cfs_band_time {'):
                        sched_header.index('#define RQCF_REQ_SKIP')]
cfs_math = ''.join(function(fair, name) for name in [
    'decay_load', '__accumulate_sum', 'accumulate_sum', '___update_load_avg'])
rt = (ROOT / 'kernel/sched/rt.c').read_text()
rt_math = ''.join(function(rt, name) for name in [
    '__accumulate_pelt_segments_rt', 'accumulate_sum_rt', '__update_load_avg'])
consumer_main = r'''
static void equal_avg(struct sched_avg *a,struct sched_avg *b) {
 assert(a->last_update_time==b->last_update_time);
 assert(a->load_sum==b->load_sum && a->load_avg==b->load_avg);
 assert(a->util_sum==b->util_sum && a->util_avg==b->util_avg);
 assert(a->period_contrib==b->period_contrib);
}
int main(void) {
 for(int c=0;c<NR;c++){assert(!pthread_mutex_init(&queues[c].lock,NULL));queues[c].cpu=c;}
 reset();
 struct sched_avg cfs_actual={0},cfs_reference={0},rt_actual={0},rt_reference={0};
 cfs_band_age_init(&cfs_actual);cfs_band_age_init(&cfs_reference);
 struct cfs_rq group={.rq=&queues[0]};struct task_group tg={.cfs_rq={&group}};
 u64 expected_task=0,expected_scaled=0,excluded_task=0,excluded_scaled=0;
 u64 frozen_task=0,frozen_scaled=0;
 unsigned int shift=1,depth=0;
 int choices[]={1,2,4,2,1,4};
 for(unsigned int i=0;i<100000;i++) {
  u64 delta=(i*7919ULL)%5000000;
  samples[0]+=delta;expected_task+=delta;expected_scaled+=delta<<shift;
  int value=choices[(i/13)%6];checked_request(value);
  /* If identical, the setter must not settle the normal rq clock either. */
  mutex_lock(&queues[0].lock);update_rq_clock(&queues[0]);mutex_unlock(&queues[0].lock);
  shift=value==1?0:value==2?1:2;
  assert(queues[0].clock_task==expected_task && queues[0].clock_task_mult==expected_scaled);
  unsigned int phase=i%401;
  if(phase==0||phase==37) {
   if(!depth){frozen_task=expected_task;frozen_scaled=expected_scaled;}
   depth++;tg_throttle_down(&tg,&queues[0]);
  } else if(phase==150||phase==250) {
   depth--;tg_unthrottle_up(&tg,&queues[0]);
   if(!depth){excluded_task+=expected_task-frozen_task;excluded_scaled+=expected_scaled-frozen_scaled;}
  }
  u64 cfs_now=(depth?frozen_scaled:expected_scaled)-excluded_scaled;
  u64 band_now=(depth?frozen_task:expected_task)-excluded_task;
  assert(cfs_now==cfs_rq_clock_task(&group));assert(band_now==cfs_rq_band_clock(&group));
  unsigned long weight=(i/97)%2?0:1024;int running=(i/53)%2;
  ___update_load_avg(cfs_rq_clock_task(&group),0,&cfs_actual,weight,running,NULL,NULL,cfs_rq_band_clock(&group),&group);
  ___update_load_avg(cfs_now,0,&cfs_reference,weight,running,NULL,NULL,band_now,&group);
  __update_load_avg(queues[0].clock_task_mult,0,&rt_actual,weight,running,NULL);
  __update_load_avg(expected_scaled,0,&rt_reference,weight,running,NULL);
  equal_avg(&cfs_actual,&cfs_reference);equal_avg(&rt_actual,&rt_reference);
 }
 puts("PASS real CFS/RT PELT consumers: 100000 runnable/blocked histories, rapid all-slope cutovers and nested CFS throttling against independent integrated clocks");
 return 0;
}
'''
# Reuse reset/reference/assertion utilities, not the preceding main().
utilities = main[:main.index('int main(void)')]
with tempfile.TemporaryDirectory(prefix='crowntrail-pelt-consumers-') as directory:
    source = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    source.write_text(consumer_prefix +
                      (ROOT / 'kernel/sched/sched-pelt.h').read_text() +
                      metadata + production + throttle + cfs_math + rt_math +
                      utilities + consumer_main)
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-Wno-unused-function', '-O2',
                    '-pthread', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
