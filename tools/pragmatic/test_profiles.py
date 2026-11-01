#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
# Run from the kernel root. Host stubs test state/decision logic, not concurrency.
from pathlib import Path
import re, subprocess, tempfile
out = Path(tempfile.mkdtemp(prefix="pragmatic-profile-tests-"))
def function(src,name):
 m=re.search(r'(?m)^(?:static\s+)?(?:inline\s+)?(?:[\w*]+\s+)+?'+re.escape(name)+r'\s*\([^;{]*\)\s*\{',src)
 assert m,name
 i=m.end();level=1
 while level:level+=(src[i]=='{')-(src[i]=='}');i+=1
 return src[m.start():i]
h=Path('include/linux/pragmatic_profile.h').read_text();h=re.sub(r'^#include.*\n','',h,flags=re.M)
base='''#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
typedef uint64_t u64;typedef uint32_t u32;
#define CONFIG_CPU_FREQ_GOV_PRAGMATIC 1
typedef struct { u64 value; } atomic64_t;
#define ATOMIC64_INIT(n) {n}
static u64 atomic64_read(atomic64_t *p){return p->value;}
static void atomic64_set(atomic64_t *p,u64 v){p->value=v;}
#define pr_diag_add(...) do {} while(0)
#define raw_spin_lock_irqsave(l,f) ((f)=0)
#define raw_spin_unlock_irqrestore(l,f) ((void)(f))
#define READ_ONCE(x) (x)
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
'''+h
manager=Path('kernel/sched/pragmatic_profile.c').read_text()
policies=manager[manager.index('const struct pr_profile_policy pr_profile_policies'):manager.index('static DEFINE_RAW_SPINLOCK')]
# The manager's stores are tested through its actual sysfs parser.
source=base+policies+'''struct kobject{int x;};struct kobj_attribute{int x;};
#define EINVAL 22
static bool sysfs_streq(const char*a,const char*b){size_t n=strlen(a);return (!strcmp(a,b))||(n&&a[n-1]=='\\n'&&n==strlen(b)+1&&!strncmp(a,b,n-1));}
'''
for n in ['pr_profile_publish','pr_profile_display','profile_store']:source+='\n'+function(manager,n)
panel=Path('drivers/video/fbdev/exynos/panel/panel_drv.c').read_text()
source+='''
#define PANEL_STATE_OFF 0
#define PANEL_STATE_ON 1
#define PANEL_STATE_NORMAL 2
#define PANEL_STATE_ALPM 3
#define PANEL_CONNECT 1
#define PANEL_DISPLAY_ON 1
#define PANEL_DISPLAY_OFF 0
struct panel_device{int id;unsigned int pragmatic_state;bool pragmatic_state_ready;struct{int cur_state,disp_on,connect_panel;}state;};
'''+function(panel,'panel_publish_pragmatic_state')+'''
int main(void){
 assert(pr_user_id(pr_profile_read())==PR_OPTIMAL);
 assert(pr_display_id(pr_profile_read())==PR_DISPLAY_UNKNOWN);
 assert(profile_store(NULL,NULL,"powersave\\n",10)==10);
 assert(pr_effective_id(pr_profile_read())==PR_OPTIMAL);
 pr_profile_display(PR_DISPLAY_NORMAL);assert(pr_effective_id(pr_profile_read())==PR_POWER_SAVER);
 pr_profile_display(PR_DISPLAY_ALPM);u64 gen=pr_profile_generation(pr_profile_read());
 assert(pr_effective_id(pr_profile_read())==PR_SCREEN_OFF);
 pr_profile_display(PR_DISPLAY_OFF);assert(pr_profile_generation(pr_profile_read())==gen);
 assert(pr_display_id(pr_profile_read())==PR_DISPLAY_OFF);
 assert(profile_store(NULL,NULL,"optimal",7)==7);assert(pr_user_id(pr_profile_read())==PR_OPTIMAL);
 assert(pr_profile_generation(pr_profile_read())==gen);assert(pr_effective_id(pr_profile_read())==PR_SCREEN_OFF);
 u64 state=pr_profile_read();assert(profile_store(NULL,NULL,"screen_off",10)==-EINVAL);assert(state==pr_profile_read());
 for(int i=0;i<100000;i++){
  pr_profile_display(PR_DISPLAY_NORMAL);assert(pr_effective_id(pr_profile_read())==PR_OPTIMAL);
  pr_profile_display(PR_DISPLAY_ALPM);pr_profile_display(PR_DISPLAY_OFF);
  profile_store(NULL,NULL,"powersave",9);assert(pr_effective_id(pr_profile_read())==PR_SCREEN_OFF);
  pr_profile_display(PR_DISPLAY_NORMAL);assert(pr_effective_id(pr_profile_read())==PR_POWER_SAVER);
  profile_store(NULL,NULL,"optimal",7);
 }
 struct panel_device p={.id=0,.pragmatic_state=PR_DISPLAY_UNKNOWN,.pragmatic_state_ready=true,.state={PANEL_STATE_NORMAL,0,PANEL_CONNECT}};
 panel_publish_pragmatic_state(&p);assert(p.pragmatic_state==PR_DISPLAY_UNKNOWN);
 p.state.disp_on=1;panel_publish_pragmatic_state(&p);assert(pr_display_id(pr_profile_read())==PR_DISPLAY_NORMAL);
 p.state.cur_state=PANEL_STATE_ALPM;panel_publish_pragmatic_state(&p);assert(pr_display_id(pr_profile_read())==PR_DISPLAY_ALPM);
 p.state.cur_state=PANEL_STATE_ON;p.state.disp_on=0;panel_publish_pragmatic_state(&p);assert(pr_display_id(pr_profile_read())==PR_DISPLAY_OFF);
 p.state.cur_state=99;panel_publish_pragmatic_state(&p);assert(pr_display_id(pr_profile_read())==PR_DISPLAY_UNKNOWN);
 p.id=1;p.state.cur_state=PANEL_STATE_NORMAL;p.state.disp_on=1;panel_publish_pragmatic_state(&p);assert(pr_display_id(pr_profile_read())==PR_DISPLAY_UNKNOWN);
 p.id=0;p.pragmatic_state_ready=false;panel_publish_pragmatic_state(&p);assert(pr_display_id(pr_profile_read())==PR_DISPLAY_UNKNOWN);
 puts("PASS: actual manager/parser/panel mapping, 100000 rapid display/user cycles, unknown, initial, secondary panel and AOD distinction");
}
'''
(out/'manager-test.c').write_text(source)
subprocess.run(['cc','-std=gnu11','-O2',str(out/'manager-test.c'),'-o',str(out/'manager-test')],check=True);subprocess.run([str(out/'manager-test')],check=True)
# Real commit/filter/timer code, with minimal deterministic host scheduler stubs.
old=subprocess.check_output(['git','show','ee98b70197799fdb95c8589fe550114f239a7a4d:kernel/sched/cpufreq_pragmatic.c'],text=True)
src=Path('kernel/sched/cpufreq_pragmatic.c').read_text()
assert function(src,'pr_effective_freq')==function(old,'pr_effective_freq')
assert function(src,'pr_select_cl1')==function(old,'pr_select_cl1')
assert function(src,'pr_select_freq')==function(old,'pr_select_freq')
assert function(src,'pr_freq_ready')==function(old,'pr_freq_ready')
prefix='\n#include <stdint.h>\n#include <stdbool.h>\n#include <assert.h>\n#include <stdio.h>\ntypedef uint64_t u64; typedef uint32_t u32;\n#define READ_ONCE(x) (x)\n#define unlikely(x) (x)\n#define min(a,b) ((a)<(b)?(a):(b))\n#define max(a,b) ((a)>(b)?(a):(b))\n#define NSEC_PER_USEC 1000ULL\n#define PM_QOS_CLUSTER0_FREQ_MIN 0\n#define PM_QOS_CLUSTER0_FREQ_MAX 1\n#define PM_QOS_CLUSTER1_FREQ_MIN 2\n#define PM_QOS_CLUSTER1_FREQ_MAX 3\n#define CPUFREQ_RELATION_L 0\nstruct opp { unsigned int frequency; };\nstruct cpufreq_policy { unsigned int cur,min,max; struct opp *freq_table; };\nstruct pragmatic_policy { struct cpufreq_policy *policy; bool cl1,limits_changed,work_in_progress,transition_in_progress; unsigned int retry_delay_us,next_freq,up_rate_us,down_rate_us; u64 last_freq_update_time; int irq_work; };\nstatic int qos[4]={0,3000,0,3000}, queues;\nstatic int pm_qos_request(int n) { return qos[n]; }\nstatic int cpufreq_frequency_table_target(struct cpufreq_policy *p,unsigned n,int r) { (void)r;n=max(p->min,min(p->max,n)); for(int i=0;i<3;i++) if(p->freq_table[i].frequency>=n)return i;return 2; }\nstatic void irq_work_queue(int *w) { (void)w;queues++; }\n\n'
prefix=prefix.replace('struct opp','struct cpufreq_frequency_table').replace('struct cpufreq_policy {','struct cpufreq_policy { unsigned int cpu,cpus;').replace('struct pragmatic_policy {','''struct timer_list {bool pending;unsigned long expires;};
struct pr_diag_stamp {u64 ns;unsigned long epoch;};
struct pragmatic_policy { struct pr_diag_stamp diag_queued; unsigned long diag_epoch;unsigned int diag_desired,diag_effective;bool diag_reconcile;
struct timer_list profile_timer;u64 profile_state,profile_release_generation;unsigned long profile_deadline;unsigned int profile_target,profile_util_pct;bool profile_pending,profile_urgent,profile_releasing,started;
''')
# Avoid redefining base types already supplied in prefix.
mini=base[base.index('#define CONFIG_CPU_FREQ_GOV_PRAGMATIC'):];mini=mini.replace('#define READ_ONCE(x) (x)\n','')
c='#include <limits.h>\n'+prefix+mini+policies+'''
static unsigned long jiffies;
static u64 timer_now;
static bool diagnostic;
static struct { unsigned int nr_running; } runqueues[2];
#define cpu_rq(c) (&runqueues[c])
#define for_each_cpu(c,mask) for((c)=0;(c)<2;(c)++) if((mask)&(1U<<(c)))
#define time_after_eq(a,b) ((long)((a)-(b))>=0)
#define time_before(a,b) ((long)((a)-(b))<0)
#define usecs_to_jiffies(u) (((unsigned long)(u)+3999)/4000)
#define cpufreq_for_each_valid_entry(pos,table) for((pos)=(table);(pos)->frequency!=~0U;(pos)++)
static int del_timer(struct timer_list*t){t->pending=false;return 0;}
static int mod_timer(struct timer_list*t,unsigned long expires){t->pending=true;t->expires=expires;return 0;}
static u64 local_clock(void){return timer_now;}
static bool pr_diag_on(void){return diagnostic;}
static unsigned long pr_diag_epoch(void){return 1;}
static struct pr_diag_stamp pr_diag_begin(void){return (struct pr_diag_stamp){0,1};}
'''
for n in ['pr_cancel_deferred','pr_sync_profile','pr_effective_freq','pr_minor_up','pr_defer_up','pr_commit','pr_profile_timer_fn','pr_force_cap']:c+='\n'+function(src,n)
c+='\n'+function(old,'pr_commit').replace('pr_commit','pr_commit_base',1)
c+='\n'+function(old,'pr_force_cap').replace('pr_force_cap','pr_force_cap_base',1)
c+='''
static unsigned int rng=12345;
static unsigned int random32(void){rng=rng*1664525+1013904223;return rng;}
static struct cpufreq_frequency_table table[]={{1000},{2000},{3000},{~0U}};
static struct cpufreq_policy p={.cur=1000,.min=1000,.max=3000,.freq_table=table,.cpu=0,.cpus=3};
static struct pragmatic_policy fresh(unsigned int id){
 atomic64_set(&pr_profile_state,(1ULL<<8)|(id<<4));
 qos[0]=qos[2]=0;qos[1]=qos[3]=3000;jiffies=100;timer_now=20000000;queues=0;
 p.cur=1000;p.min=1000;p.max=3000;runqueues[0].nr_running=1;
 return (struct pragmatic_policy){.policy=&p,.started=true,.profile_state=pr_profile_read(),.profile_util_pct=20,.next_freq=1000,.up_rate_us=5000,.down_rate_us=3000};
}
int main(void){
 for(int d=0;d<2;d++)for(int i=0;i<200000;i++){
  struct pragmatic_policy a=fresh(PR_OPTIMAL),b;u64 now=random32()%200000000;unsigned int next=random32()%4001;int qa,qb;
  diagnostic=d;p.cur=(1+random32()%3)*1000;a.next_freq=random32()%4001;
  a.work_in_progress=!!((random32()>>16)&1);a.transition_in_progress=!!((random32()>>16)&1);a.limits_changed=!!((random32()>>16)&1);
  a.retry_delay_us=random32()%2?0:100000;a.last_freq_update_time=random32()%100000000;
  for(int j=0;j<4;j++)qos[j]=(int)(random32()%5)*1000-1000;
  b=a;queues=0;pr_commit_base(&a,now,next);qa=queues;queues=0;pr_commit(&b,now,next);qb=queues;
  assert(a.next_freq==b.next_freq&&a.work_in_progress==b.work_in_progress&&a.retry_delay_us==b.retry_delay_us&&a.limits_changed==b.limits_changed&&a.last_freq_update_time==b.last_freq_update_time&&qa==qb);
  queues=0;pr_force_cap_base(&a,next);qa=queues;queues=0;pr_force_cap(&b,next);qb=queues;
  assert(a.next_freq==b.next_freq&&a.work_in_progress==b.work_in_progress&&a.last_freq_update_time==b.last_freq_update_time&&qa==qb);
 }
 struct pragmatic_policy a=fresh(PR_POWER_SAVER);
 assert(!pr_commit(&a,timer_now,2000)&&a.profile_pending&&!queues&&a.next_freq==1000);
 unsigned long deadline=a.profile_deadline;
 assert(!pr_commit(&a,timer_now+1000,2000)&&a.profile_deadline==deadline);
 assert(!pr_commit(&a,timer_now+2000,1000)&&!a.profile_pending&&!a.profile_timer.pending);
 a=fresh(PR_POWER_SAVER);a.profile_urgent=true;assert(pr_commit(&a,timer_now,2000)&&queues==1);
 a=fresh(PR_POWER_SAVER);a.profile_util_pct=60;assert(pr_commit(&a,timer_now,2000));
 a=fresh(PR_POWER_SAVER);assert(pr_commit(&a,timer_now,3000));
 a=fresh(PR_POWER_SAVER);qos[0]=2000;assert(pr_commit(&a,timer_now,1000)&&a.next_freq==2000);
 a=fresh(PR_POWER_SAVER);a.limits_changed=true;assert(pr_commit(&a,timer_now,2000));
 a=fresh(PR_SCREEN_OFF);assert(!pr_commit(&a,timer_now,3000));assert(a.profile_deadline==102);
 jiffies=101;pr_profile_timer_fn((unsigned long)&a);assert(a.profile_pending&&!queues); /* older invocation vs newer deadline */
 jiffies=102;timer_now+=8000000;pr_profile_timer_fn((unsigned long)&a);assert(!a.profile_pending&&queues==1&&a.next_freq==3000); /* no further util callback needed */
 a=fresh(PR_SCREEN_OFF);pr_commit(&a,timer_now,2000);jiffies=102;runqueues[0].nr_running=0;pr_profile_timer_fn((unsigned long)&a);assert(!a.profile_pending&&!queues);
 a=fresh(PR_SCREEN_OFF);pr_commit(&a,timer_now,2000);atomic64_set(&pr_profile_state,(2ULL<<8));jiffies=102;pr_profile_timer_fn((unsigned long)&a);assert(!a.profile_pending&&!queues);
 a=fresh(PR_SCREEN_OFF);pr_commit(&a,timer_now,2000);a.started=false;jiffies=102;pr_profile_timer_fn((unsigned long)&a);assert(!a.profile_pending&&!queues);
 a=fresh(PR_POWER_SAVER);a.profile_releasing=true;a.profile_release_generation=1;atomic64_set(&pr_profile_state,(2ULL<<8)|(PR_SCREEN_OFF<<4));assert(!pr_commit(&a,timer_now,2000)&&!queues);
 a=fresh(PR_SCREEN_OFF);pr_commit(&a,timer_now,2000);atomic64_set(&pr_profile_state,(2ULL<<8));assert(pr_commit(&a,timer_now+1000,2000)&&!a.profile_pending&&!a.profile_timer.pending&&queues==1); /* wake to Optimal */
 a=fresh(PR_SCREEN_OFF);pr_commit(&a,timer_now,2000);unsigned long saved=a.profile_deadline;atomic64_set(&pr_profile_state,pr_profile_read()|PR_POWER_SAVER);assert(!pr_commit(&a,timer_now+1000,2000)&&a.profile_pending&&a.profile_deadline==saved); /* preference changes while off */
 a=fresh(PR_SCREEN_OFF);pr_commit(&a,timer_now,2000);qos[0]=2000;assert(pr_commit(&a,timer_now+1000,2000)&&!a.profile_pending&&queues==1); /* floor while held */
 a=fresh(PR_SCREEN_OFF);pr_commit(&a,timer_now,2000);pr_force_cap(&a,1000);assert(!a.profile_pending&&!a.profile_timer.pending&&!queues); /* cap without a hardware transition */
 a=fresh(PR_SCREEN_OFF);a.next_freq=3000;a.work_in_progress=true;assert(pr_commit(&a,timer_now,2000)&&a.next_freq==2000&&!a.profile_pending&&!queues); /* reduce an admitted target while hardware still low */
 a=fresh(PR_POWER_SAVER);jiffies=ULONG_MAX;pr_commit(&a,timer_now,2000);assert(a.profile_deadline==0);jiffies=0;timer_now+=4000000;pr_profile_timer_fn((unsigned long)&a);assert(queues==1);
 puts("PASS: 400000 Optimal equivalence cases; filter/down/urgent/QoS/limits, bounded release, idle cancellation, generations, stale timer, stopped state, jiffies wrap");
}
'''
(out/'governor-test.c').write_text(c)
subprocess.run(['cc','-std=gnu11','-O2',str(out/'governor-test.c'),'-o',str(out/'governor-test')],check=True);subprocess.run([str(out/'governor-test')],check=True)

# Preserve Optimal's credit arithmetic; conservative modes only reduce refill.
credit_prefix = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
typedef uint64_t u64;typedef uint32_t u32;
#define min_t(t,a,b) ((t)(a)<(t)(b)?(t)(a):(t)(b))
static u64 div64_u64(u64 a,u64 b){return a/b;}
struct pr_boost_config {u32 turbo_base_khz,shared_pct,turbo_recharge_mw,turbo_capacity_uj;};
struct pr_boost_state {struct pr_boost_config cfg;u64 turbo_credit_uj;};
static u32 cl1_estimated_power_mw(u32 f,unsigned long t,unsigned long p,u32 shared){return f/1000+t+p+shared;}
'''
credit_c = credit_prefix + function(src, 'pr_update_credit') + function(old, 'pr_update_credit').replace('pr_update_credit', 'pr_update_credit_base', 1)
credit_c += r'''
int main(void) {
 for(unsigned int credit=0;credit<=32000;credit+=125)for(unsigned int freq=1000000;freq<=2400000;freq+=100000)for(unsigned int dt=0;dt<=1000;dt+=125){
  struct pr_boost_state a={.cfg={1794000,20,120,32000},.turbo_credit_uj=credit},b=a,c=a,d=a;
  pr_update_credit_base(&a,freq,100,80,dt);
  pr_update_credit(&b,freq,100,80,dt,100);
  pr_update_credit(&c,freq,100,80,dt,80);
  pr_update_credit(&d,freq,100,80,dt,60);
  assert(a.turbo_credit_uj==b.turbo_credit_uj);
  assert(d.turbo_credit_uj<=c.turbo_credit_uj&&c.turbo_credit_uj<=b.turbo_credit_uj);
  if(freq>1794000)assert(d.turbo_credit_uj==b.turbo_credit_uj);
 }
 puts("PASS: 34695 credit cases, Optimal exact, conservative refill monotonic, debit unchanged");
}
'''
(out/'credit-test.c').write_text(credit_c)
subprocess.run(['cc','-std=gnu11','-O2',str(out/'credit-test.c'),'-o',str(out/'credit-test')],check=True)
subprocess.run([str(out/'credit-test')],check=True)
print('Host artifacts:', out)
