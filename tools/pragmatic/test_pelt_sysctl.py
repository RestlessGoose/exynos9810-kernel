#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
# Host test of actual handler code; parser/user-copy and kernel locking are not simulated.
from pathlib import Path
import subprocess
import tempfile, re
out=Path(tempfile.mkdtemp(prefix="pelt-sysctl-test-"))
def handler(s):
 a=s.index("int sched_pelt_multiplier(");b=s.index("\n}\n",a)+2;return s[a:b]
old=subprocess.check_output(["git","show","5ee636305a6bb1cf8957fbf6b3c38d30382ec411:kernel/sched/core.c"],text=True)
new=Path("kernel/sched/core.c").read_text()
prefix=r'''
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <assert.h>
#define __user
#define DEFINE_MUTEX(m) pthread_mutex_t m=PTHREAD_MUTEX_INITIALIZER
#define mutex_lock(m) pthread_mutex_lock(m)
#define mutex_unlock(m) pthread_mutex_unlock(m)
#define READ_ONCE(x) __atomic_load_n(&(x),__ATOMIC_RELAXED)
#define WRITE_ONCE(x,v) __atomic_store_n(&(x),(v),__ATOMIC_RELAXED)
struct ctl_table {void*data;int maxlen;};
unsigned int sysctl_sched_pelt_multiplier=2,sched_pelt_lshift=1;
static unsigned int observed,invalid_exposures;
static int proc_dointvec(struct ctl_table*t,int w,void*b,size_t*l,loff_t*p){
 if(!w){*(int*)b=READ_ONCE(*(unsigned int*)t->data);return 0;}
 int v=*(int*)b;
 __atomic_store_n((int*)t->data,v,__ATOMIC_RELAXED);
 unsigned int x=READ_ONCE(sysctl_sched_pelt_multiplier);
 WRITE_ONCE(observed,x);
 if(x!=1&&x!=2&&x!=4)__atomic_fetch_add(&invalid_exposures,1,__ATOMIC_RELAXED);
 return 0;
}
'''
# This harness tests candidate validation only. The separate cutover harness
# executes the real all-runqueue helper and clock-accounting implementation.
prefix += r'''
static void sched_pelt_set_multiplier(unsigned int value,unsigned int shift){
 assert(value==1||value==2||value==4);
 WRITE_ONCE(sched_pelt_lshift,shift);
 WRITE_ONCE(sysctl_sched_pelt_multiplier,value);
}
'''
c=prefix+handler(old).replace('int sched_pelt_multiplier(', 'int old_handler(',1)+handler(new)
c+=r'''
static int call(int (*fn)(struct ctl_table*,int,void*,size_t*,loff_t*),int w,int v){
 struct ctl_table t={&sysctl_sched_pelt_multiplier,sizeof(unsigned int)};size_t n=sizeof(v);loff_t p=0;
 return fn(&t,w,&v,&n,&p);
}
static void*writer(void*unused){int vals[]={1,2,3,4,-1,2,0};for(int i=0;i<10000;i++)for(int j=0;j<7;j++)call(sched_pelt_multiplier,1,vals[j]);return NULL;}
int main(void){
 assert(call(old_handler,1,3)==-EINVAL);assert(observed==3&&sysctl_sched_pelt_multiplier==2&&sched_pelt_lshift==1);
 puts("CONFIRMED old defect: parser exposes rejected 3 before validation restores 2");
 invalid_exposures=0;
 int vals[]={0,3,-1,INT_MAX};for(int i=0;i<4;i++){assert(call(sched_pelt_multiplier,1,vals[i])==-EINVAL);assert(observed==2&&sysctl_sched_pelt_multiplier==2&&sched_pelt_lshift==1);}
 assert(!invalid_exposures);
 for(int v=1;v<=4;v*=2){assert(!call(sched_pelt_multiplier,1,v));assert(sysctl_sched_pelt_multiplier==(unsigned int)v&&sched_pelt_lshift==(unsigned int)(v==1?0:v==2?1:2));}
 assert(!call(sched_pelt_multiplier,1,2));assert(!call(sched_pelt_multiplier,0,0));assert(sysctl_sched_pelt_multiplier==2);
 pthread_t a,b;pthread_create(&a,NULL,writer,NULL);pthread_create(&b,NULL,writer,NULL);pthread_join(a,NULL);pthread_join(b,NULL);
 assert(!invalid_exposures);assert(!call(sched_pelt_multiplier,1,2));assert(sysctl_sched_pelt_multiplier==2&&sched_pelt_lshift==1);
 puts("PASS proposed fix: valid reads/writes, rejected values, 140000 serialized writer calls; no invalid public value observed");
 puts("LIMIT: parser is a scalar-store stub; this does not test real user-copy faults, rq cutover, automatic ownership or scheduler concurrency");
}
'''
(out/'sysctl-test.c').write_text(c)
subprocess.run(['cc','-std=gnu11','-O2','-pthread',str(out/'sysctl-test.c'),'-o',str(out/'sysctl-test')],check=True)
r=subprocess.run([str(out/'sysctl-test')],capture_output=True,text=True,check=True);print(r.stdout,end='');(out/'sysctl-test.log').write_text(r.stdout)
