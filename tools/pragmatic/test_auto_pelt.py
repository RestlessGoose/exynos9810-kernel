#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Actual controller/manager/scheduler code with simulated time/workqueues.

Also runs the extracted scheduler cutover suite. Kernel lockdep, freezer,
workqueue concurrency and device latency remain untested.
"""
from pathlib import Path
import re
import runpy
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
cut = runpy.run_path(str(ROOT / 'tools/pragmatic/test_pelt_cutover.py'))
fn = cut['function']
manager = (ROOT / 'kernel/sched/pragmatic_profile.c').read_text()
controller = (ROOT / 'kernel/sched/pragmatic_pelt.c').read_text()
controller = re.sub(r'^#include.*\n', '', controller, flags=re.M)
# Inject failures only BEFORE the real scheduler admits a cutover.
controller = controller.replace('sched_pelt_auto_control(action, value, &changed)',
                                'test_control(action, value, &changed)')
header = (ROOT / 'include/linux/pragmatic_profile.h').read_text()
header = re.sub(r'^#include.*\n', '', header, flags=re.M)
diag = (ROOT / 'include/linux/pragmatic_diag.h').read_text()
diag_enums = diag[diag.index('enum pr_diag_event'):diag.index('struct pr_diag_stamp')]
policies = manager[manager.index('const struct pr_profile_policy'):manager.index('static DEFINE_RAW_SPINLOCK')]
prelude = r'''
#define CONFIG_CPU_FREQ_GOV_PRAGMATIC 1
#define __init
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define DEFINE_RAW_SPINLOCK(l) pthread_mutex_t l=PTHREAD_MUTEX_INITIALIZER
#define scnprintf snprintf
#define WQ_HIGHPRI 1
#define WQ_FREEZABLE 2
#define time_before(a,b) ((long)((a)-(b))<0)
#define msecs_to_jiffies(n) (((n)+3)/4)
typedef struct { _Atomic u64 value; } atomic64_t;
#define ATOMIC64_INIT(v) {v}
static u64 atomic64_read(atomic64_t *p){return atomic_load(&p->value);}
static void atomic64_set(atomic64_t *p,u64 v){atomic_store(&p->value,v);}
static unsigned long jiffies;
struct work_struct { int unused; };
struct delayed_work { struct work_struct work; void (*fn)(struct work_struct*); bool pending; unsigned long due; };
#define DECLARE_DELAYED_WORK(n,f) struct delayed_work n={.fn=f}
struct workqueue_struct { bool destroyed; };
static struct workqueue_struct mock_wq;
static bool alloc_fail;
static unsigned int destroys,submissions;
static struct workqueue_struct *alloc_ordered_workqueue(const char *name,int flags){
 (void)name;(void)flags;if(alloc_fail)return NULL;mock_wq.destroyed=false;return &mock_wq;
}
static bool mod_delayed_work(struct workqueue_struct *q,struct delayed_work *w,unsigned long delay){
 assert(q&&!q->destroyed);bool pending=w->pending;w->pending=true;w->due=jiffies+delay;submissions++;return pending;
}
#define delayed_work_pending(w) ((w)->pending)
static bool cancel_delayed_work(struct delayed_work *w){bool p=w->pending;w->pending=false;return p;}
#define cancel_delayed_work_sync(w) cancel_delayed_work(w)
static void destroy_workqueue(struct workqueue_struct *q){q->destroyed=true;destroys++;}
struct kobject { int unused; };struct kobj_attribute { int unused; };
static bool sysfs_streq(const char *a,const char *b){size_t n=strlen(a);return !strcmp(a,b)||(n&&a[n-1]=='\n'&&n==strlen(b)+1&&!strncmp(a,b,n-1));}
static struct kobject root_kobj, profile_kobj;
static struct kobject *cpufreq_global_kobject=&root_kobj;
static int profile_group, group_calls, kobject_puts, warnings;
static bool kobject_fail, group_fail, group_saw_owner;
static unsigned int group_saw_multiplier;
static int request(int value);
static struct kobject *kobject_create_and_add(const char *name,struct kobject *parent){
 assert(!strcmp(name,"pragmatic")&&parent==&root_kobj);
 return kobject_fail?NULL:&profile_kobj;
}
static int sysfs_create_group(struct kobject *kobj,const void *group){
 assert(kobj==&profile_kobj&&group==&profile_group);
 group_calls++;group_saw_owner=sched_pelt_auto_owned();
 group_saw_multiplier=sysctl_sched_pelt_multiplier;
 if(group_saw_owner)assert(request(4)==-EBUSY);
 return group_fail?-ENOMEM:0;
}
static void kobject_put(struct kobject *kobj){assert(kobj==&profile_kobj);kobject_puts++;}
#define pr_warn(...) do { warnings++; } while(0)
#define WARN_ON_ONCE(c) ({ bool warning=!!(c); assert(!warning); warning; })
struct pr_diag_stamp { u64 ns; unsigned long epoch; };
static bool diag_on;
static u64 timing_clock,clock_reads;
static unsigned long diag_epoch=1;
static u64 diag_events[PRD_EVENTS],timings;
static u64 ktime_get_ns(void){clock_reads++;return timing_clock+=100;}
static struct pr_diag_stamp pr_diag_begin(void){return diag_on?(struct pr_diag_stamp){ktime_get_ns(),diag_epoch}:(struct pr_diag_stamp){0,0};}
static bool pr_diag_valid(struct pr_diag_stamp s){return diag_on&&s.epoch&&s.epoch==diag_epoch;}
static void pr_diag_add(int cpu,int event,u64 n){(void)cpu;if(diag_on)diag_events[event]+=n;}
static void pr_diag_add_stamp(int cpu,int event,u64 n,struct pr_diag_stamp s){if(pr_diag_valid(s))pr_diag_add(cpu,event,n);}
static void pr_diag_time(int cpu,int metric,struct pr_diag_stamp s,u64 ns){(void)cpu;(void)metric;(void)ns;if(pr_diag_valid(s))timings++;}
static bool fail_next;
static int test_control(enum sched_pelt_action action,unsigned int v,bool *changed){
 if(fail_next){fail_next=false;*changed=false;return -EIO;}
 return sched_pelt_auto_control(action,v,changed);
}
'''
manager_code = ''.join(fn(manager, name) for name in [
    'pr_profile_publish', 'pr_profile_display', 'profile_store',
    'auto_pelt_store']) + fn(manager.replace('__init ', ''), 'pr_profile_init')
# Publication expects profile_lock; the production lock order is retained.
manager_code = 'static DEFINE_RAW_SPINLOCK(profile_lock);\n' + manager_code
main = r'''
static int user(const char *s){return profile_store(NULL,NULL,s,strlen(s));}
static int automatic(const char *s){return auto_pelt_store(NULL,NULL,s,strlen(s));}
static void drain(void){int bound=100;while(pelt_work.pending&&!time_before(jiffies,pelt_work.due)){assert(bound--);pelt_work.pending=false;pelt_work.fn(&pelt_work.work);}}
static void expire(void){if(pelt_work.pending)jiffies=pelt_work.due;drain();}
static void expect(unsigned int value,bool enabled){assert(sysctl_sched_pelt_multiplier==value);assert(pr_pelt_auto_enabled()==enabled);assert(sched_pelt_auto_owned()==enabled);}
static void *toggle_mode(void *unused){
 (void)unused;for(int i=0;i<1000;i++)assert(pr_pelt_set_auto(i&1)==0);return NULL;
}
static void *manual_writer(void *unused){
 (void)unused;for(int i=0;i<1000;i++){int ret=request((i&1)?2:4);assert(ret==0||ret==-EBUSY);}return NULL;
}
static unsigned int injected;
static void wake_during_cutover(pthread_mutex_t *lock){
 if(lock!=&queues[0].lock||injected)return;
 injected++;
 char status[180];pr_pelt_status(status,sizeof(status));assert(strstr(status,"applying"));
 pr_profile_display(PR_DISPLAY_NORMAL);
}
static void many_during_cutover(pthread_mutex_t *lock){
 if(lock!=&queues[0].lock||injected)return;
 injected++;user("powersave");pr_profile_display(PR_DISPLAY_ALPM);
 pr_profile_display(PR_DISPLAY_UNKNOWN);pr_profile_display(PR_DISPLAY_OFF);
 user("optimal");pr_profile_display(PR_DISPLAY_NORMAL);
}
static _Atomic bool disable_started;
static void *disable_thread(void *unused){(void)unused;atomic_store(&disable_started,true);assert(automatic("0")==1);return NULL;}
static pthread_t disabler;
static void disable_during_cutover(pthread_mutex_t *lock){
 if(lock!=&queues[0].lock||injected)return;
 injected++;assert(!pthread_create(&disabler,NULL,disable_thread,NULL));
 while(!atomic_load(&disable_started)){} /* host-only interleaving barrier */
 pr_profile_display(PR_DISPLAY_NORMAL);
}
int main(int argc,char **argv){
 for(int i=0;i<NR;i++){assert(!pthread_mutex_init(&queues[i].lock,NULL));queues[i].cpu=i;samples[i]=1000;}
 expect(2,false);
 if(argc>1){
  if(!strcmp(argv[1],"no-cpufreq")){
   cpufreq_global_kobject=NULL;assert(pr_profile_init()==-ENODEV);
   expect(2,false);assert(!group_calls&&!pelt_wq);return 0;
  }
  if(!strcmp(argv[1],"no-kobject")){
   kobject_fail=true;assert(pr_profile_init()==-ENOMEM);
   expect(2,false);assert(!group_calls&&!pelt_wq);return 0;
  }
  if(!strcmp(argv[1],"no-worker")){
   alloc_fail=true;assert(!pr_profile_init());expect(2,false);
   assert(group_calls==1&&!group_saw_owner&&warnings==1&&last_error==-ENOMEM);
   assert(!request(4));assert(automatic("1")==-ENOMEM);return 0;
  }
  if(!strcmp(argv[1],"claim-fail")){
   fail_next=true;assert(!pr_profile_init());expect(2,false);
   assert(group_calls==1&&!group_saw_owner&&warnings==1&&last_error==-EIO);
   assert(!request(4));assert(automatic("1")==1);expect(2,true);return 0;
  }
  if(!strcmp(argv[1],"no-sysfs")){
   pr_profile_display(PR_DISPLAY_OFF);group_fail=true;
   assert(pr_profile_init()==-ENOMEM);expect(2,false);
   assert(group_calls==1&&group_saw_owner&&group_saw_multiplier==1);
   assert(kobject_puts==1&&destroys==1&&!pelt_wq);
   assert(!request(4));return 0;
  }
  if(!strcmp(argv[1],"off-at-boot")){
   pr_profile_display(PR_DISPLAY_OFF);assert(!pr_profile_init());expect(1,true);
   assert(group_saw_owner&&group_saw_multiplier==1);
   assert(automatic("0")==1);expect(2,false);assert(!request(4));return 0;
  }
  if(!strcmp(argv[1],"preclaim-manual")){
   assert(!request(4));assert(!pr_profile_init());expect(2,true);
   assert(group_saw_owner&&group_saw_multiplier==2);
   assert(automatic("0")==1);expect(2,false);return 0;
  }
  assert(0&&"unknown boot scenario");
 }
 assert(!pr_profile_init());expect(2,true);
 assert(group_calls==1&&group_saw_owner&&group_saw_multiplier==2&&!warnings);
 assert(automatic("true")==-EINVAL&&automatic("2")==-EINVAL&&automatic("-1")==-EINVAL);
 assert(request(1)==-EBUSY&&request(2)==-EBUSY&&request(4)==-EBUSY);
 unsigned int reads=samples_read;assert(automatic("1\n")==2);assert(samples_read==reads);
 {struct ctl_table t={&sysctl_sched_pelt_multiplier,sizeof(unsigned int)};unsigned int v=0;size_t n=sizeof(v);loff_t pos=0;assert(!sched_pelt_multiplier(&t,0,&v,&n,&pos)&&v==2);}
 pr_profile_display(PR_DISPLAY_NORMAL);user("powersave");drain();expect(1,true);
 reads=samples_read;unsigned int submissions_before=submissions;pr_profile_display(PR_DISPLAY_ALPM);pr_profile_display(PR_DISPLAY_OFF);drain();expect(1,true);assert(samples_read==reads&&submissions==submissions_before);
 user("optimal");drain();expect(1,true); /* preference retained while off */
 pr_profile_display(PR_DISPLAY_NORMAL);drain();expect(2,true);
 pr_profile_display(PR_DISPLAY_OFF);assert(pelt_work.pending&&time_before(jiffies,pelt_work.due));
 pr_profile_display(PR_DISPLAY_UNKNOWN);drain();expect(2,true);jiffies+=100;drain();expect(2,true);
 pr_profile_display(PR_DISPLAY_OFF);expire();expect(1,true);
 assert(automatic("0")==1);expect(2,false);assert(!request(4));
 assert(automatic("0")==1);expect(4,false); /* duplicate disable preserves manual */
 assert(automatic("1")==1);expect(1,true); /* initial enable while off is immediate */
 assert(automatic("0")==1);expect(2,false);
 fail_next=true;assert(automatic("1")==-EIO);expect(2,false);
 assert(automatic("1")==1);expect(1,true);
 fail_next=true;assert(automatic("0")==-EIO);expect(1,true);
 assert(automatic("0")==1);expect(2,false);
 pr_profile_display(PR_DISPLAY_NORMAL);assert(automatic("1")==1);expect(2,true);
 diag_on=true;
 pr_profile_display(PR_DISPLAY_OFF);pr_profile_display(PR_DISPLAY_NORMAL);drain();expect(2,true);
 assert(diag_events[PRD_PELT_NOOPS]&&diag_events[PRD_PELT_SUPERSEDED]);
 injected=0;after_unlock=wake_during_cutover;
 pr_profile_display(PR_DISPLAY_OFF);expire();after_unlock=NULL;drain();expect(2,true);assert(injected==1);
 injected=0;after_unlock=many_during_cutover;
 pr_profile_display(PR_DISPLAY_OFF);expire();after_unlock=NULL;drain();expect(2,true);assert(injected==1);
 user("powersave");fail_next=true;drain();expect(2,true);assert(last_error==-EIO);
 assert(automatic("1")==1);expect(1,true);assert(!last_error);
 assert(diag_events[PRD_PELT_FAILURES]&&diag_events[PRD_PELT_TO_1]&&diag_events[PRD_PELT_TO_2]&&timings);
 diag_epoch++;char status[180];pr_pelt_status(status,sizeof(status));assert(strstr(status,"last_control_ns=0"));
 diag_on=false;u64 clocks=clock_reads;
 user("optimal");drain();expect(2,true);
 for(int i=0;i<10000;i++){
  pr_profile_display(PR_DISPLAY_ALPM);pr_profile_display(PR_DISPLAY_OFF);
  user(i&1?"powersave":"optimal");pr_profile_display(PR_DISPLAY_UNKNOWN);
  drain();expect(2,true);pr_profile_display(PR_DISPLAY_NORMAL);drain();expect(i&1?1:2,true);
 }
 assert(clock_reads==clocks); /* no timing calls when diagnostics disabled */
 user("optimal");drain();expect(2,true);
 jiffies=ULONG_MAX-5;pr_profile_display(PR_DISPLAY_OFF);assert(time_before(jiffies,pelt_work.due));
 pr_profile_display(PR_DISPLAY_NORMAL);drain();expect(2,true);
 pr_profile_display(PR_DISPLAY_OFF);expire();expect(1,true);
 pr_profile_display(PR_DISPLAY_NORMAL);drain();expect(2,true);
 pthread_t mode_a,mode_b,manual;
 assert(!pthread_create(&mode_a,NULL,toggle_mode,NULL));assert(!pthread_create(&mode_b,NULL,toggle_mode,NULL));
 assert(!pthread_create(&manual,NULL,manual_writer,NULL));
 assert(!pthread_join(mode_a,NULL));assert(!pthread_join(mode_b,NULL));assert(!pthread_join(manual,NULL));
 assert(automatic("0")==1);assert(!request(2));assert(automatic("1")==1);expect(2,true);
 injected=0;disable_started=false;after_unlock=disable_during_cutover;
 pr_profile_display(PR_DISPLAY_OFF);expire();after_unlock=NULL;
 assert(!pthread_join(disabler,NULL));drain();expect(2,false);assert(injected==1);
 assert(!request(1));expect(1,false);assert(!request(4));expect(4,false);
 assert(automatic("0")==1);expect(4,false);assert(!request(2));
 pr_pelt_init_failed();assert(destroys==1&&!pelt_wq&&!pelt_work.pending);
 unsigned int queued=submissions;pr_profile_display(PR_DISPLAY_NORMAL);assert(submissions==queued);
 assert(automatic("1")==-ENODEV);expect(2,false);
 puts("PASS default-on boot, ownership before sysfs visibility, manual writes blocked, failed startup recovery and rollback");
 puts("PASS actual automatic controller/manager/scheduler: mapping, ownership, enable/disable/errors, debounce supersession, mid-cutover publications, racing disable, 10000 rapid display/user cycles, init failure/cleanup, diag-disabled timing and epoch reset");
 puts("LIMIT: workqueue/freezer/hotplug and panel execution are simulated; real kernel concurrency and phone latency remain unverified");
 return 0;
}
'''
# Reuse real scheduler code and its scalar proc parser, not its main program.
utilities = cut['main'][:cut['main'].index('int main(void)')]
with tempfile.TemporaryDirectory(prefix='crowntrail-auto-pelt-') as directory:
    source = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    source.write_text(cut['prefix'] + cut['production'] + diag_enums + prelude +
                      header + policies + controller + manager_code + utilities + main)
    subprocess.run(['cc', '-std=gnu11', '-O2', '-pthread', '-Wall', '-Wextra',
                    '-Werror', '-Wno-unused-parameter', '-Wno-unused-function',
                    str(source), '-o', str(binary)], check=True)
    for scenario in ['no-cpufreq', 'no-kobject', 'no-worker', 'claim-fail',
                     'no-sysfs', 'off-at-boot', 'preclaim-manual']:
        subprocess.run([str(binary), scenario], check=True, timeout=60)
    subprocess.run([str(binary)], check=True, timeout=60)
