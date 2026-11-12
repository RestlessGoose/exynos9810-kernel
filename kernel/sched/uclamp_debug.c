// SPDX-License-Identifier: GPL-2.0
/* Read-only snapshots; no counters or added scheduler hot-path work. */
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include "sched.h"

static int uclamp_debug_show(struct seq_file *s, void *unused)
{
	int cpu;

	seq_printf(s, "active %u\nscale %lu\n", uclamp_is_used(),
		   (unsigned long)SCHED_CAPACITY_SCALE);
	seq_puts(s, "cpu rq_min rq_max capacity\n");
	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		seq_printf(s, "%d %u %u %lu\n", cpu,
			   (unsigned int)READ_ONCE(rq->uclamp[UCLAMP_MIN].value),
			   (unsigned int)READ_ONCE(rq->uclamp[UCLAMP_MAX].value),
			   (unsigned long)capacity_orig_of(cpu));
	}
	return 0;
}

static int uclamp_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, uclamp_debug_show, NULL);
}

static const struct file_operations uclamp_debug_fops = {
	.open = uclamp_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init uclamp_debug_init(void)
{
	debugfs_create_file("sched_uclamp", 0400, NULL, NULL,
			    &uclamp_debug_fops);
	return 0;
}
late_initcall(uclamp_debug_init);
#endif
