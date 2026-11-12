/*
 * Exynos input booster. Input callbacks record owned events; ordered workers
 * own the generic and top-edge request state and delayed releases.
 */
#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/input_booster.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/suspend.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>
#include <linux/pragmatic_profile.h>

#define IB_QUEUE_SIZE 256
#define IB_MAX_INSTANCES 128
/* All shipped DTS head/tail windows are shorter; this only caps a lost OFF. */
#define IB_MAX_HOLD_MS 10000
#define IB_TOP_MAX_KHZ 2314000
#define IB_TOP_COOLDOWN_NS (100ULL * NSEC_PER_MSEC)

struct ib_top_slot {
	int x, y;
	bool pending, x_seen, y_seen;
};

struct ib_event {
	u32 source;
	u32 generation;
	u16 type;
	u16 key;
	bool on;
	bool reset_source;
};

struct ib_handle {
	struct input_handle handle;
	spinlock_t lock;
	unsigned long *slots;
	unsigned int nslots;
	unsigned int slot;
	unsigned int fingers;
	unsigned long keys[BITS_TO_LONGS(KEY_CNT)];
	u8 key_type[KEY_CNT];
	bool scan_pending;
	bool pen_pending;
	int pen_value;
	u32 source;
	struct ib_top_slot *top_slots;
	bool top_touchscreen;
	u32 top_generation;
	int top_xmin, top_xmax, top_ymin, top_ymax;
};

struct ib_instance {
	struct list_head list;
	struct delayed_work timeout;
	struct delayed_work failsafe;
	u32 source;
	u16 type;
	u16 key;
	bool tail;
	bool released;
	u32 head_time;
	u32 tail_time;
	int head[IB_VENDOR_RES_COUNT];
	int tail_values[IB_VENDOR_RES_COUNT];
};

static struct t_ib_device_tree policies[MAX_DEVICE_TYPE_NUM];
static bool policy_present[MAX_DEVICE_TYPE_NUM];
static struct device *policy_devices[MAX_DEVICE_TYPE_NUM];
static struct class *ib_class;
static DEFINE_MUTEX(policy_lock);
static DEFINE_MUTEX(enable_lock);
static struct workqueue_struct *ib_wq;
static bool handler_registered; /* enable_lock */
static bool shutting_down;	/* enable_lock */
static struct work_struct event_work;
static struct work_struct clear_work;
static LIST_HEAD(instances); /* ordered workqueue only */
static unsigned int instance_count;
static DEFINE_SPINLOCK(event_lock);
static struct ib_event events[IB_QUEUE_SIZE];
static unsigned int event_head, event_tail;
static bool event_overflow;
static atomic_t next_source = ATOMIC_INIT(0);
static int applied[IB_VENDOR_RES_COUNT]; /* ordered workqueue only */
static bool booster_enabled;
static u32 enable_generation;
static unsigned int debug_level;
static unsigned int level_value = 2;
static unsigned int send_event;
static struct dentry *diag_dir;
static bool diag_enabled;
static atomic64_t recognized = ATOMIC64_INIT(0);
static atomic64_t submitted = ATOMIC64_INIT(0);
static atomic64_t work_runs = ATOMIC64_INIT(0);
static atomic64_t applied_requests = ATOMIC64_INIT(0);
static atomic64_t releases = ATOMIC64_INIT(0);
static atomic64_t queue_overflows = ATOMIC64_INIT(0);
static atomic64_t failsafe_releases = ATOMIC64_INIT(0);

/* A separate request lets the DTS booster retain ownership of its own QoS. */
static DEFINE_SPINLOCK(top_lock);
static DEFINE_MUTEX(top_control_lock);
static struct workqueue_struct *top_wq;
static struct work_struct top_apply_work;
static struct delayed_work top_release_work;
static struct pm_qos_request top_qos, top_little_qos;
static bool top_suspended;
static unsigned int top_stage;
struct ib_top_config {
	unsigned int big1, little1, ms1, big2, little2, ms2;
	unsigned int profile, spike_enable, spike_khz, spike_ms, plateau_ms,
		decay_ms, hold_ms;
};
static struct ib_top_config top_config = {
	.big1 = 2106000,
	.little1 = 1690000,
	.ms1 = 220,
	.big2 = 1586000,
	.little2 = 1456000,
	.ms2 = 300,
	.profile = 1,
	.spike_enable = 0,
	.spike_khz = 2314000,
	.spike_ms = 20,
	.plateau_ms = 150,
	.decay_ms = 250,
	.hold_ms = 60,
};
#define IB_TOP_POINTS 64
struct ib_top_point {
	unsigned int ms, big, little, stage;
};
struct ib_top_plan {
	unsigned int count;
	struct ib_top_point p[IB_TOP_POINTS];
};
static struct ib_top_plan top_plan, top_run;
static unsigned int top_run_index;
static atomic64_t top_transitions = ATOMIC64_INIT(0);
static unsigned int top_little_at_apply;
static bool top_enabled, top_pending, top_active;
static unsigned int top_region = 8, top_active_freq, top_active_little,
		    top_reach_target;
static u64 top_detect_ns, top_request_ns, top_applied_ns, top_reached_ns;
static u64 top_last_release_ns, top_stage2_ns;
static unsigned int top_freq_at_apply;
static atomic_t top_devices = ATOMIC_INIT(0);
static atomic_t top_generation = ATOMIC_INIT(1);
static atomic64_t top_triggers = ATOMIC64_INIT(0);
static atomic64_t top_applies = ATOMIC64_INIT(0);
static atomic64_t top_releases = ATOMIC64_INIT(0);
static atomic64_t top_reaches = ATOMIC64_INIT(0);
static atomic64_t top_cooldown_skips = ATOMIC64_INIT(0);

static int ib_top_transition(struct notifier_block *nb, unsigned long event,
			     void *data)
{
	struct cpufreq_freqs *freqs = data;
	unsigned long flags;

	if (event != CPUFREQ_POSTCHANGE || freqs->cpu < 4 || freqs->cpu > 7)
		return NOTIFY_DONE;
	spin_lock_irqsave(&top_lock, flags);
	if (top_active && top_stage && READ_ONCE(top_request_ns) &&
	    !top_reached_ns && freqs->new >= top_reach_target) {
		top_reached_ns = ktime_get_ns();
		atomic64_inc(&top_reaches);
	}
	spin_unlock_irqrestore(&top_lock, flags);
	return NOTIFY_OK;
}

static struct notifier_block top_transition_nb = {
	.notifier_call = ib_top_transition,
};

/* Deadlines are relative to detection, so worker latency cannot extend a stage.
 */
static unsigned long ib_top_delay(u64 deadline)
{
	u64 now = ktime_get_ns();

	return deadline > now ? nsecs_to_jiffies(deadline - now) + 1 : 0;
}

/* Both works run on the same ordered queue. No old release survives a run. */
static void ib_top_step(bool initial)
{
	unsigned long flags;
	unsigned int i, actual_khz;
	u64 elapsed;
	struct ib_top_point point;

	elapsed = div_u64(ktime_get_ns() - READ_ONCE(top_detect_ns),
			  NSEC_PER_MSEC);
	i = top_run_index;
	while (i + 1 < top_run.count && top_run.p[i + 1].ms <= elapsed)
		i++;
	point = top_run.p[i];
	spin_lock_irqsave(&top_lock, flags);
	top_run_index = i;
	top_stage = point.stage;
	top_active_freq = point.big;
	top_active_little = point.little;
	if (initial) {
		top_reached_ns = 0;
		top_reach_target = point.big;
	}
	if (initial)
		top_request_ns = ktime_get_ns();
	else
		top_stage2_ns = ktime_get_ns();
	spin_unlock_irqrestore(&top_lock, flags);
	pm_qos_update_request(&top_qos, point.big);
	pm_qos_update_request(&top_little_qos, point.little);
	if (initial)
		WRITE_ONCE(top_applied_ns, ktime_get_ns());
	actual_khz = cpufreq_quick_get(4);
	WRITE_ONCE(top_freq_at_apply, actual_khz);
	WRITE_ONCE(top_little_at_apply, cpufreq_quick_get(0));
	if (initial)
		atomic64_inc(&top_applies);
	else if (point.stage)
		atomic64_inc(&top_transitions);
	spin_lock_irqsave(&top_lock, flags);
	if (!point.stage) {
		top_active = false;
		top_last_release_ns = ktime_get_ns();
	} else if (actual_khz >= top_reach_target && !top_reached_ns) {
		top_reached_ns = ktime_get_ns();
		atomic64_inc(&top_reaches);
	}
	spin_unlock_irqrestore(&top_lock, flags);
	if (!point.stage) {
		atomic64_inc(&top_releases);
		return;
	}
	queue_delayed_work(
		top_wq, &top_release_work,
		ib_top_delay(READ_ONCE(top_detect_ns) +
			     (u64)top_run.p[i + 1].ms * NSEC_PER_MSEC));
}

static void ib_top_release(struct work_struct *work)
{
	ib_top_step(false);
}

static void ib_top_apply(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&top_lock, flags);
	if (!top_pending || !READ_ONCE(top_enabled) ||
	    READ_ONCE(top_suspended)) {
		top_pending = false;
		spin_unlock_irqrestore(&top_lock, flags);
		return;
	}
	top_pending = false;
	top_run = top_plan;
	top_run_index = 0;
	if (ktime_get_ns() - top_detect_ns >=
	    (u64)top_run.p[top_run.count - 1].ms * NSEC_PER_MSEC) {
		spin_unlock_irqrestore(&top_lock, flags);
		return;
	}
	top_active = true;
	top_stage2_ns = 0;
	spin_unlock_irqrestore(&top_lock, flags);
	ib_top_step(true);
}

static void ib_top_trigger(void)
{
	unsigned long flags;
	u64 now;

	if (!READ_ONCE(top_enabled) || !READ_ONCE(booster_enabled))
		return;
#ifdef CONFIG_CPU_FREQ_GOV_PRAGMATIC
	if (pr_display_id(pr_profile_read()) != PR_DISPLAY_NORMAL)
		return;
#endif
	spin_lock_irqsave(&top_lock, flags);
	/* One finite window; a held finger or rapid contacts cannot extend it.
	 */
	if (!READ_ONCE(top_enabled) || !READ_ONCE(booster_enabled) ||
	    READ_ONCE(top_suspended) || top_pending || top_active) {
		spin_unlock_irqrestore(&top_lock, flags);
		return;
	}
	now = ktime_get_ns();
	if (top_last_release_ns &&
	    now - top_last_release_ns < IB_TOP_COOLDOWN_NS) {
		atomic64_inc(&top_cooldown_skips);
		spin_unlock_irqrestore(&top_lock, flags);
		return;
	}
	top_pending = true;
	top_detect_ns = now;
	atomic64_inc(&top_triggers);
	spin_unlock_irqrestore(&top_lock, flags);
	queue_work(top_wq, &top_apply_work);
}

static void ib_top_decide(struct ib_handle *bh, struct ib_top_slot *slot)
{
	s64 y, height;

	if (!slot->x_seen || !slot->y_seen)
		return;
	/* Decide once from this contact's initial coordinates, never from
	 * motion.
	 */
	slot->pending = false;
	y = (s64)slot->y - bh->top_ymin;
	height = (s64)bh->top_ymax - bh->top_ymin + 1;
	if (slot->x >= bh->top_xmin && slot->x <= bh->top_xmax && y >= 0 &&
	    y < height && y * 100 < height * READ_ONCE(top_region))
		ib_top_trigger();
}

static void ib_top_event(struct ib_handle *bh, unsigned int type,
			 unsigned int code, int value)
{
	struct ib_top_slot *slot;
	unsigned int i;
	u32 generation;

	if (!bh->top_touchscreen || !READ_ONCE(top_enabled))
		return;
	generation = atomic_read(&top_generation);
	if (bh->top_generation != generation) {
		memset(bh->top_slots, 0, sizeof(*bh->top_slots) * bh->nslots);
		bh->top_generation = generation;
	}
	if (type == EV_SYN && code == SYN_REPORT) {
		/* The input core suppresses unchanged MT coordinates on slot
		 * reuse.
		 */
		for (i = 0; i < bh->nslots; i++) {
			slot = &bh->top_slots[i];
			if (!slot->pending)
				continue;
			if (!slot->x_seen) {
				slot->x = input_mt_get_value(
					&bh->handle.dev->mt->slots[i],
					ABS_MT_POSITION_X);
				slot->x_seen = true;
			}
			if (!slot->y_seen) {
				slot->y = input_mt_get_value(
					&bh->handle.dev->mt->slots[i],
					ABS_MT_POSITION_Y);
				slot->y_seen = true;
			}
			ib_top_decide(bh, slot);
		}
		return;
	}
	if (type != EV_ABS || bh->slot >= bh->nslots)
		return;
	slot = &bh->top_slots[bh->slot];
	if (code == ABS_MT_TRACKING_ID) {
		slot->pending = value >= 0;
		slot->x_seen = slot->y_seen = false;
		return;
	}
	if (!slot->pending)
		return;
	if (code == ABS_MT_POSITION_X) {
		slot->x = value;
		slot->x_seen = true;
	} else if (code == ABS_MT_POSITION_Y) {
		slot->y = value;
		slot->y_seen = true;
	} else {
		return;
	}
	ib_top_decide(bh, slot);
}

static void ib_top_disable(void)
{
	unsigned long flags;
	bool active;

	WRITE_ONCE(top_enabled, false);
	atomic_inc(&top_generation);
	spin_lock_irqsave(&top_lock, flags);
	top_pending = false;
	spin_unlock_irqrestore(&top_lock, flags);
	cancel_work_sync(&top_apply_work);
	cancel_delayed_work_sync(&top_release_work);
	spin_lock_irqsave(&top_lock, flags);
	active = top_active;
	top_active = false;
	top_stage = 0;
	top_active_freq = top_active_little = 0;
	spin_unlock_irqrestore(&top_lock, flags);
	if (active) {
		pm_qos_update_request(&top_qos, 0);
		pm_qos_update_request(&top_little_qos, 0);
		spin_lock_irqsave(&top_lock, flags);
		top_last_release_ns = ktime_get_ns();
		spin_unlock_irqrestore(&top_lock, flags);
		atomic64_inc(&top_releases);
	}
}

#define IB_COUNT(counter)                                                      \
	do {                                                                   \
		if (READ_ONCE(diag_enabled))                                   \
			atomic64_inc(&(counter));                              \
	} while (0)

static void ib_recompute(void)
{
	struct ib_instance *item;
	int next[IB_VENDOR_RES_COUNT] = {0};
	int changes[IB_VENDOR_RES_COUNT] = {0};
	int i;

	list_for_each_entry(item, &instances, list) {
		for (i = 0; i < IB_VENDOR_RES_COUNT; i++) {
			int value = item->tail ? item->tail_values[i] : item->head[i];

			if (value > next[i])
				next[i] = value;
		}
	}
	for (i = 0; i < IB_VENDOR_RES_COUNT; i++) {
		if (next[i] == applied[i])
			continue;
		if (next[i]) {
			changes[i] = next[i];
			IB_COUNT(applied_requests);
		} else {
			ib_release_booster(i);
			IB_COUNT(releases);
		}
		applied[i] = next[i];
	}
	ib_set_booster(changes);
}

static struct ib_instance *ib_find(const struct ib_event *ev)
{
	struct ib_instance *item;

	list_for_each_entry(item, &instances, list)
		if (item->source == ev->source && item->type == ev->type &&
		    item->key == ev->key)
			return item;
	return NULL;
}

enum ib_remove_origin {
	IB_EXTERNAL,
	IB_STAGE_TIMEOUT,
	IB_FAILSAFE_TIMEOUT,
};

static void ib_remove(struct ib_instance *item, enum ib_remove_origin origin)
{
	list_del(&item->list);
	instance_count--;
	if (origin != IB_STAGE_TIMEOUT)
		cancel_delayed_work(&item->timeout);
	if (origin != IB_FAILSAFE_TIMEOUT)
		cancel_delayed_work(&item->failsafe);
	kfree(item);
}

static void ib_clear(u32 source)
{
	struct ib_instance *item, *tmp;

	list_for_each_entry_safe(item, tmp, &instances, list)
		if (!source || item->source == source)
			ib_remove(item, IB_EXTERNAL);
	ib_recompute();
}

static void ib_failsafe_timeout(struct work_struct *work)
{
	struct ib_instance *item = container_of(to_delayed_work(work),
						struct ib_instance, failsafe);

	IB_COUNT(failsafe_releases);
	ib_remove(item, IB_FAILSAFE_TIMEOUT);
	ib_recompute();
}

static void ib_timeout(struct work_struct *work)
{
	struct ib_instance *item = container_of(to_delayed_work(work),
						struct ib_instance, timeout);

	if (!item->tail && item->tail_time) {
		item->tail = true;
		ib_recompute();
		if (item->released)
			queue_delayed_work(ib_wq, &item->timeout,
				msecs_to_jiffies(item->tail_time));
	} else {
		ib_remove(item, IB_STAGE_TIMEOUT);
		ib_recompute();
	}
}

static void ib_process(const struct ib_event *ev)
{
	struct ib_instance *item;
	struct t_ib_device_tree *policy;
	int i;

	if (ev->reset_source) {
		ib_clear(ev->source);
		return;
	}
	if (ev->type >= MAX_DEVICE_TYPE_NUM || !policy_present[ev->type])
		return;
	item = ib_find(ev);
	if (!ev->on) {
		if (!item || item->released)
			return;
		item->released = true;
		if (item->tail)
			queue_delayed_work(ib_wq, &item->timeout,
				msecs_to_jiffies(item->tail_time));
		return;
	}
	if (!READ_ONCE(booster_enabled) ||
	    ev->generation != READ_ONCE(enable_generation))
		return;
	if (item) {
		if (!item->released)
			return;
		cancel_delayed_work(&item->timeout);
		cancel_delayed_work(&item->failsafe);
	} else {
		if (instance_count >= IB_MAX_INSTANCES)
			return;
		item = kzalloc(sizeof(*item), GFP_KERNEL);
		if (!item)
			return;
		INIT_DELAYED_WORK(&item->timeout, ib_timeout);
		INIT_DELAYED_WORK(&item->failsafe, ib_failsafe_timeout);
		item->source = ev->source;
		item->type = ev->type;
		item->key = ev->key;
		list_add_tail(&item->list, &instances);
		instance_count++;
	}
	policy = &policies[ev->type];
	mutex_lock(&policy_lock);
	item->head_time = policy->head_time;
	item->tail_time = policy->tail_time;
	for (i = 0; i < IB_VENDOR_RES_COUNT; i++) {
		item->head[i] = policy->res[i].head_value;
		item->tail_values[i] = policy->res[i].tail_value;
	}
	mutex_unlock(&policy_lock);
	item->tail = false;
	item->released = false;
	ib_recompute();
	queue_delayed_work(ib_wq, &item->timeout,
			msecs_to_jiffies(item->head_time));
	queue_delayed_work(ib_wq, &item->failsafe,
			msecs_to_jiffies(IB_MAX_HOLD_MS));
}

static void ib_event_worker(struct work_struct *work)
{
	struct ib_event ev;
	unsigned long flags;
	bool overflow, have_event;
	int processed = 0;

	IB_COUNT(work_runs);
	while (processed++ < 64) {
		spin_lock_irqsave(&event_lock, flags);
		overflow = event_overflow;
		event_overflow = false;
		have_event = event_tail != event_head;
		if (!have_event && !overflow) {
			spin_unlock_irqrestore(&event_lock, flags);
			break;
		}
		if (have_event) {
			ev = events[event_tail];
			event_tail = (event_tail + 1) % IB_QUEUE_SIZE;
		}
		spin_unlock_irqrestore(&event_lock, flags);
		if (overflow)
			ib_clear(0);
		if (have_event)
			ib_process(&ev);
	}
	spin_lock_irqsave(&event_lock, flags);
	if (event_tail != event_head || event_overflow)
		queue_work(ib_wq, &event_work);
	spin_unlock_irqrestore(&event_lock, flags);
}

static void ib_queue(const struct ib_event *ev)
{
	unsigned long flags;
	unsigned int next;

	if (!ev->reset_source)
		IB_COUNT(recognized);
	spin_lock_irqsave(&event_lock, flags);
	next = (event_head + 1) % IB_QUEUE_SIZE;
	if (next == event_tail) {
		/* Missing an OFF must never leave a boost held indefinitely. */
		event_head = event_tail = 0;
		event_overflow = true;
		IB_COUNT(queue_overflows);
		next = 1;
	}
	events[event_head] = *ev;
	event_head = next;
	IB_COUNT(submitted);
	queue_work(ib_wq, &event_work);
	spin_unlock_irqrestore(&event_lock, flags);
}

static void ib_emit(struct ib_handle *bh, int type, int key, bool on)
{
	struct ib_event ev = {
		.source = bh->source, .generation = READ_ONCE(enable_generation),
		.type = type, .key = key, .on = on,
	};

	if (on && !READ_ONCE(booster_enabled))
		return;
	ib_queue(&ev);
}

static int ib_key_type(struct ib_handle *bh, unsigned int code)
{
	switch (code) {
	case KEY_BACK:
	case KEY_HOMEPAGE:
	case KEY_RECENT:
		return TOUCH_KEY;
	case KEY_VOLUMEUP:
	case KEY_VOLUMEDOWN:
	case KEY_POWER:
	case KEY_WINK:
		return KEY;
	case BTN_TOOL_PEN:
		return HOVER;
	case BTN_TOUCH:
		return -1;
	case BTN_LEFT:
	case BTN_RIGHT:
	case BTN_MIDDLE:
		return bh->scan_pending ? MOUSE : -1;
	default:
		return bh->scan_pending ? KEYBOARD : -1;
	}
}

static void ib_input_event(struct input_handle *handle,
			  unsigned int type, unsigned int code, int value)
{
	struct ib_handle *bh = handle->private;
	unsigned long flags;
	int kind;

	spin_lock_irqsave(&bh->lock, flags);
	if (bh->pen_pending) {
		if (type == EV_ABS && code == ABS_PRESSURE) {
			if (bh->pen_value == 1 &&
			    !test_and_set_bit(BTN_TOUCH, bh->keys)) {
				bh->key_type[BTN_TOUCH] = SPEN;
				ib_emit(bh, SPEN, BTN_TOUCH, true);
			} else if (bh->pen_value == 0 &&
				   test_and_clear_bit(BTN_TOUCH, bh->keys)) {
				ib_emit(bh, SPEN, BTN_TOUCH, false);
			}
		}
		bh->pen_pending = false;
	}
	if (type == EV_KEY && code == BTN_TOUCH &&
	    test_bit(BTN_TOOL_PEN, handle->dev->keybit) &&
	    test_bit(ABS_PRESSURE, handle->dev->absbit) &&
	    (value == 0 || value == 1)) {
		bh->pen_pending = true;
		bh->pen_value = value;
		bh->scan_pending = false;
		goto out;
	}
	if (type == EV_ABS && code == ABS_MT_SLOT) {
		bh->slot = value >= 0 && value < bh->nslots ? value : bh->nslots;
		bh->scan_pending = false;
	} else if (type == EV_ABS && code == ABS_MT_TRACKING_ID &&
		   bh->slot < bh->nslots) {
		if (value >= 0 && !test_and_set_bit(bh->slot, bh->slots)) {
			if (++bh->fingers == 1)
				ib_emit(bh, TOUCH, 0, true);
			else if (bh->fingers == 2)
				ib_emit(bh, MULTI_TOUCH, 0, true);
		} else if (value < 0 && test_and_clear_bit(bh->slot, bh->slots)) {
			if (bh->fingers == 2)
				ib_emit(bh, MULTI_TOUCH, 0, false);
			else if (bh->fingers == 1)
				ib_emit(bh, TOUCH, 0, false);
			bh->fingers--;
		}
		bh->scan_pending = false;
	} else if (type == EV_MSC && code == MSC_SCAN) {
		bh->scan_pending = true;
	} else if (type == EV_KEY && code < KEY_CNT) {
		if (value == 1 && !test_bit(code, bh->keys)) {
			kind = ib_key_type(bh, code);
			if (kind >= 0 && policy_present[kind]) {
				set_bit(code, bh->keys);
				bh->key_type[code] = kind;
				ib_emit(bh, kind, code, true);
			}
		} else if (value == 0 && test_and_clear_bit(code, bh->keys)) {
			ib_emit(bh, bh->key_type[code], code, false);
		}
		bh->scan_pending = false;
	} else if (type == EV_SYN && code == SYN_REPORT) {
		bh->scan_pending = false;
	} else {
		bh->scan_pending = false;
	}
out:
	ib_top_event(bh, type, code, value);
	spin_unlock_irqrestore(&bh->lock, flags);
}

static void ib_input_start(struct input_handle *handle)
{
	struct ib_handle *bh = handle->private;
	struct ib_event ev = { .source = bh->source, .reset_source = true };
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);
	if (bh->slots)
		bitmap_zero(bh->slots, bh->nslots);
	bitmap_zero(bh->keys, KEY_CNT);
	bh->fingers = bh->slot = 0;
	bh->scan_pending = false;
	bh->pen_pending = false;
	if (bh->top_slots)
		memset(bh->top_slots, 0,
		       sizeof(*bh->top_slots) * bh->nslots);
	ib_queue(&ev);
	spin_unlock_irqrestore(&bh->lock, flags);
}

static bool ib_supported_device(struct input_dev *dev)
{
	if (dev->mt && test_bit(EV_ABS, dev->evbit) &&
	    test_bit(ABS_MT_TRACKING_ID, dev->absbit))
		return true;
	if (!test_bit(EV_KEY, dev->evbit))
		return false;
	if (test_bit(KEY_BACK, dev->keybit) ||
	    test_bit(KEY_HOMEPAGE, dev->keybit) ||
	    test_bit(KEY_RECENT, dev->keybit) ||
	    test_bit(KEY_VOLUMEUP, dev->keybit) ||
	    test_bit(KEY_VOLUMEDOWN, dev->keybit) ||
	    test_bit(KEY_POWER, dev->keybit) ||
	    test_bit(KEY_WINK, dev->keybit) ||
	    test_bit(BTN_TOOL_PEN, dev->keybit))
		return true;
	/* The existing keyboard/mouse scenarios require a scan code. */
	return test_bit(EV_MSC, dev->evbit) &&
	       test_bit(MSC_SCAN, dev->mscbit);
}

static int ib_input_connect(struct input_handler *handler,
			    struct input_dev *dev, const struct input_device_id *id)
{
	struct ib_handle *bh;
	int err;

	if (!ib_supported_device(dev))
		return -ENODEV;
	bh = kzalloc(sizeof(*bh), GFP_KERNEL);
	if (!bh)
		return -ENOMEM;
	spin_lock_init(&bh->lock);
	bh->source = atomic_inc_return(&next_source);
	if (dev->mt && test_bit(ABS_MT_TRACKING_ID, dev->absbit)) {
		bh->nslots = dev->mt->num_slots;
		bh->slots = kcalloc(BITS_TO_LONGS(bh->nslots),
				    sizeof(unsigned long), GFP_KERNEL);
		if (!bh->slots) {
			err = -ENOMEM;
			goto free_handle;
		}
		if (dev->name && !strcmp(dev->name, "sec_touchscreen") &&
		    test_bit(INPUT_PROP_DIRECT, dev->propbit) &&
		    test_bit(ABS_MT_POSITION_X, dev->absbit) &&
		    test_bit(ABS_MT_POSITION_Y, dev->absbit)) {
			bh->top_xmin = dev->absinfo[ABS_MT_POSITION_X].minimum;
			bh->top_xmax = dev->absinfo[ABS_MT_POSITION_X].maximum;
			bh->top_ymin = dev->absinfo[ABS_MT_POSITION_Y].minimum;
			bh->top_ymax = dev->absinfo[ABS_MT_POSITION_Y].maximum;
			if (bh->top_xmax > bh->top_xmin &&
			    bh->top_ymax > bh->top_ymin) {
				bh->top_slots = kcalloc(bh->nslots,
					       sizeof(*bh->top_slots),
					       GFP_KERNEL);
				bh->top_touchscreen = !!bh->top_slots;
			}
		}
	}
	bh->handle.dev = dev;
	bh->handle.handler = handler;
	bh->handle.name = "input_booster";
	bh->handle.private = bh;
	err = input_register_handle(&bh->handle);
	if (err)
		goto free_slots;
	err = input_open_device(&bh->handle);
	if (err)
		goto unregister;
	if (bh->top_touchscreen)
		atomic_inc(&top_devices);
	return 0;

unregister:
	input_unregister_handle(&bh->handle);
free_slots:
	kfree(bh->top_slots);
	kfree(bh->slots);
free_handle:
	kfree(bh);
	return err;
}

static void ib_input_disconnect(struct input_handle *handle)
{
	struct ib_handle *bh = handle->private;
	struct ib_event ev = { .source = bh->source, .reset_source = true };

	input_close_device(handle);
	input_unregister_handle(handle);
	if (bh->top_touchscreen)
		atomic_dec(&top_devices);
	ib_queue(&ev);
	kfree(bh->top_slots);
	kfree(bh->slots);
	kfree(bh);
}

static const struct input_device_id ib_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler ib_handler = {
	.event = ib_input_event,
	.connect = ib_input_connect,
	.disconnect = ib_input_disconnect,
	.start = ib_input_start,
	.name = "input_booster",
	.id_table = ib_ids,
};

/* Called with enable_lock held. No worker takes that mutex. */
static void ib_disable_locked(void)
{
	WRITE_ONCE(booster_enabled, false);
	WRITE_ONCE(enable_generation, enable_generation + 1);
	if (top_wq) {
		mutex_lock(&top_control_lock);
		ib_top_disable();
		mutex_unlock(&top_control_lock);
	}
	if (handler_registered) {
		input_unregister_handler(&ib_handler);
		handler_registered = false;
	}
	/* No event producers remain; drain any self-requeued event work. */
	flush_workqueue(ib_wq);
	queue_work(ib_wq, &clear_work);
	flush_work(&clear_work);
}

/* enable_lock held; publish availability only after registration succeeds. */
static int ib_enable_locked(void)
{
	int err;

	if (READ_ONCE(booster_enabled))
		return 0;
	err = input_register_handler(&ib_handler);
	if (err)
		return err;
	handler_registered = true;
	WRITE_ONCE(enable_generation, enable_generation + 1);
	WRITE_ONCE(booster_enabled, true);
	return 0;
}

static ssize_t enable_show(struct class *class, struct class_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(booster_enabled));
}

static ssize_t enable_store(struct class *class, struct class_attribute *attr,
			    const char *buf, size_t count)
{
	bool value;
	int err = 0;

	if (kstrtobool(buf, &value))
		return -EINVAL;
	mutex_lock(&enable_lock);
	if (shutting_down) {
		err = -ESHUTDOWN;
	} else if (value != READ_ONCE(booster_enabled)) {
		if (value) {
			err = ib_enable_locked();
		} else {
			ib_disable_locked();
		}
	}
	mutex_unlock(&enable_lock);
	return err ? err : count;
}
static CLASS_ATTR(enable_event, 0644, enable_show, enable_store);

static ssize_t debug_show(struct class *class, struct class_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(debug_level));
}

static ssize_t debug_store(struct class *class, struct class_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	WRITE_ONCE(debug_level, value);
	return count;
}
static CLASS_ATTR(debug_level, 0644, debug_show, debug_store);

static ssize_t send_show(struct class *class, struct class_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(send_event));
}

static ssize_t send_store(struct class *class, struct class_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	WRITE_ONCE(send_event, value);
	return count;
}
static CLASS_ATTR(sendevent, 0644, send_show, send_store);

static ssize_t ib_values_show(struct device *dev, char *buf, bool head)
{
	struct t_ib_device_tree *p = dev_get_drvdata(dev);
	ssize_t len;
	int i;

	mutex_lock(&policy_lock);
	len = sprintf(buf, "%u", head ? p->head_time : p->tail_time);
	for (i = 0; i < IB_VENDOR_RES_COUNT; i++)
		len += sprintf(buf + len, " %d", head ? p->res[i].head_value :
							 p->res[i].tail_value);
	len += sprintf(buf + len, " 0\n"); /* reserved seventh resource */
	mutex_unlock(&policy_lock);
	return len;
}

static ssize_t ib_values_store(struct device *dev, const char *buf,
			       size_t count, bool head)
{
	struct t_ib_device_tree *p = dev_get_drvdata(dev);
	int values[MAX_RES_COUNT], time, n, i;

	if (sscanf(buf, "%d%n", &time, &n) != 1 || time < 0)
		return -EINVAL;
	buf += n;
	for (i = 0; i < MAX_RES_COUNT; i++) {
		if (sscanf(buf, "%d%n", &values[i], &n) != 1 || values[i] < 0)
			return -EINVAL;
		buf += n;
	}
	if (values[IB_VENDOR_RES_COUNT] || sscanf(buf, "%d", &i) == 1)
		return -EINVAL;
	mutex_lock(&policy_lock);
	if (head)
		p->head_time = time;
	else
		p->tail_time = time;
	for (i = 0; i < IB_VENDOR_RES_COUNT; i++) {
		if (head)
			p->res[i].head_value = values[i];
		else
			p->res[i].tail_value = values[i];
	}
	mutex_unlock(&policy_lock);
	return count;
}

static ssize_t head_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return ib_values_show(dev, buf, true);
}
static ssize_t head_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	return ib_values_store(dev, buf, count, true);
}
static DEVICE_ATTR(head, 0644, head_show, head_store);

static ssize_t tail_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return ib_values_show(dev, buf, false);
}
static ssize_t tail_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	return ib_values_store(dev, buf, count, false);
}
static DEVICE_ATTR(tail, 0644, tail_show, tail_store);

static ssize_t level_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(level_value));
}
static ssize_t level_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || value > 2)
		return -EINVAL;
	WRITE_ONCE(level_value, value);
	return count;
}
static DEVICE_ATTR(level, 0644, level_show, level_store);

/* Build a bounded immutable schedule in process context, never on touch. */
static int ib_top_point(struct ib_top_plan *plan, unsigned int ms,
			unsigned int big, unsigned int little,
			unsigned int stage)
{
	unsigned int i;

	for (i = 0; i < plan->count && plan->p[i].ms < ms; i++)
		;
	if (i < plan->count && plan->p[i].ms == ms) {
		if (big)
			plan->p[i].big =
				plan->p[i].big ? min(plan->p[i].big, big) : big;
		if (little)
			plan->p[i].little =
				plan->p[i].little
					? min(plan->p[i].little, little)
					: little;
		plan->p[i].stage = stage;
		return 0;
	}
	if (plan->count == IB_TOP_POINTS)
		return -E2BIG;
	memmove(&plan->p[i + 1], &plan->p[i],
		(plan->count - i) * sizeof(plan->p[0]));
	plan->p[i] = (struct ib_top_point){ms, big, little, stage};
	plan->count++;
	return 0;
}

/* cpufreq_cpu_get() rejects an offline representative CPU. A sibling can
 * still hold the live cluster policy during CPU hotplug.
 */
static struct cpufreq_policy *ib_top_policy(unsigned int cpu)
{
	struct cpufreq_policy *policy;
	unsigned int first = cpu < 4 ? 0 : 4;

	for (cpu = first; cpu < first + 4 && cpu < nr_cpu_ids; cpu++) {
		policy = cpufreq_cpu_get(cpu);
		if (policy)
			return policy;
	}
	return NULL;
}

static int ib_top_build(const struct ib_top_config *cfg,
			struct ib_top_plan *plan)
{
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *pos;
	unsigned int cpu, start, end, base, offset, i;
	int err = 0;

	memset(plan, 0, sizeof(*plan));
	if (cfg->profile &&
	    (cfg->big2 >= cfg->big1 || cfg->little2 > cfg->little1 ||
	     (cfg->spike_enable && cfg->spike_khz < cfg->big1)))
		return -EINVAL;
	base = cfg->spike_enable ? cfg->spike_ms : 0;
	if (cfg->profile) {
		if (base)
			ib_top_point(plan, 0, cfg->spike_khz, cfg->little1, 1);
		ib_top_point(plan, base, cfg->big1, cfg->little1, 2);
		base += cfg->plateau_ms;
		ib_top_point(plan, base, cfg->big1, cfg->little1, 3);
	} else {
		ib_top_point(plan, 0, cfg->big1, cfg->little1, 1);
		ib_top_point(plan, cfg->ms1, cfg->big2, cfg->little2, 2);
	}
	for (cpu = 0; cpu <= 4; cpu += 4) {
		policy = ib_top_policy(cpu);
		if (!policy)
			return -EAGAIN;
		start = cpu ? cfg->big1 : cfg->little1;
		end = cpu ? cfg->big2 : cfg->little2;
		if (cpufreq_frequency_table_get_index(policy, start) < 0 ||
		    cpufreq_frequency_table_get_index(policy, end) < 0 ||
		    (cpu && cfg->profile && cfg->spike_enable &&
		     cpufreq_frequency_table_get_index(policy, cfg->spike_khz) <
			     0)) {
			cpufreq_cpu_put(policy);
			return -EINVAL;
		}
		if (cfg->profile && start > end) {
			cpufreq_for_each_valid_entry(pos, policy->freq_table) {
				unsigned int freq = pos->frequency;

				if (freq >= start || freq < end)
					continue;
				/* Crossing times follow frequency distance, not
				 * table index.
				 */
				offset = DIV_ROUND_UP_ULL((u64)(start - freq) *
								  cfg->decay_ms,
							  start - end);
				err = ib_top_point(plan, base + offset,
						   cpu ? freq : 0,
						   cpu ? 0 : freq, 3);
				if (err)
					break;
			}
		}
		cpufreq_cpu_put(policy);
		if (err)
			return err;
	}
	if (cfg->profile) {
		err = ib_top_point(plan, base + cfg->decay_ms, cfg->big2,
				   cfg->little2, 4);
		if (!err)
			err = ib_top_point(plan,
					   base + cfg->decay_ms + cfg->hold_ms,
					   0, 0, 0);
	} else {
		err = ib_top_point(plan, cfg->ms1 + cfg->ms2, 0, 0, 0);
	}
	if (err)
		return err;
	for (i = 1; i + 1 < plan->count; i++) {
		if (!plan->p[i].big)
			plan->p[i].big = plan->p[i - 1].big;
		if (!plan->p[i].little)
			plan->p[i].little = plan->p[i - 1].little;
	}
	return 0;
}

static int ib_top_validate(void)
{
	struct ib_top_plan plan;
	unsigned long flags;
	int err = ib_top_build(&top_config, &plan);

	if (err)
		return err;
	spin_lock_irqsave(&top_lock, flags);
	top_plan = plan;
	spin_unlock_irqrestore(&top_lock, flags);
	return 0;
}

/* top_control_lock held; no enabled state without a complete validated plan. */
static int ib_top_enable_locked(void)
{
	int err;

	if (!top_wq)
		return -ENODEV;
	if (READ_ONCE(top_enabled))
		return 0;
	err = ib_top_validate();
	if (err)
		return err;
	atomic_inc(&top_generation);
	WRITE_ONCE(top_enabled, true);
	return 0;
}

static ssize_t top_enable_show(struct class *class,
			       struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(top_enabled));
}

static ssize_t top_enable_store(struct class *class,
				struct class_attribute *attr, const char *buf,
				size_t count)
{
	bool enable;
	int err;

	if (kstrtobool(buf, &enable))
		return -EINVAL;
	mutex_lock(&top_control_lock);
	if (!top_wq) {
		mutex_unlock(&top_control_lock);
		return -ENODEV;
	}
	err = 0;
	if (enable)
		err = ib_top_enable_locked();
	else if (READ_ONCE(top_enabled))
		ib_top_disable();
	mutex_unlock(&top_control_lock);
	return err ? err : count;
}
static CLASS_ATTR(top_edge_enable, 0644, top_enable_show, top_enable_store);

static ssize_t top_region_show(struct class *class,
			       struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(top_region));
}

static ssize_t top_region_store(struct class *class,
				struct class_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int value;

	if (kstrtouint(buf, 0, &value) || !value || value > 25)
		return -EINVAL;
	WRITE_ONCE(top_region, value);
	return count;
}
static CLASS_ATTR(top_edge_region_percent, 0644, top_region_show,
		  top_region_store);

static int ib_top_set(unsigned int *field, unsigned int value, int cpu)
{
	struct ib_top_config cfg;
	struct ib_top_plan plan;
	struct cpufreq_policy *policy;
	unsigned long flags;
	unsigned int *candidate;
	int err;

	if ((cpu >= 0 &&
	     (!value || value > (cpu ? IB_TOP_MAX_KHZ : 1950000))) ||
	    (cpu == -1 && (!value || value > 1000)) || (cpu == -2 && value > 1))
		return -EINVAL;
	if (cpu >= 0) {
		policy = ib_top_policy(cpu);
		if (!policy)
			return -EAGAIN;
		err = cpufreq_frequency_table_get_index(policy, value);
		cpufreq_cpu_put(policy);
		if (err < 0)
			return -EINVAL;
	}
	mutex_lock(&top_control_lock);
	cfg = top_config;
	candidate = (unsigned int *)((char *)&cfg +
				     ((char *)field - (char *)&top_config));
	*candidate = value;
	err = ib_top_build(&cfg, &plan);
	if (!err) {
		spin_lock_irqsave(&top_lock, flags);
		top_config = cfg;
		top_plan = plan;
		spin_unlock_irqrestore(&top_lock, flags);
	}
	mutex_unlock(&top_control_lock);
	return err;
}

#define IB_TOP_ATTR(name, field, cpu) \
static ssize_t name##_show(struct class *c, struct class_attribute *a, \
			   char *buf) \
{ \
	return sprintf(buf, "%u\n", READ_ONCE(top_config.field)); \
} \
static ssize_t name##_store(struct class *c, struct class_attribute *a, \
			    const char *buf, size_t count) \
{ \
	unsigned int value; \
	int err; \
	\
	if (kstrtouint(buf, 0, &value)) \
		return -EINVAL; \
	err = ib_top_set(&top_config.field, value, cpu); \
	return err ? err : count; \
} \
static CLASS_ATTR(name, 0644, name##_show, name##_store)

IB_TOP_ATTR(top_edge_stage1_big_khz, big1, 4);
IB_TOP_ATTR(top_edge_stage1_little_khz, little1, 0);
IB_TOP_ATTR(top_edge_stage1_ms, ms1, -1);
IB_TOP_ATTR(top_edge_stage2_big_khz, big2, 4);
IB_TOP_ATTR(top_edge_stage2_little_khz, little2, 0);
IB_TOP_ATTR(top_edge_stage2_ms, ms2, -1);
IB_TOP_ATTR(top_edge_profile, profile, -2);
IB_TOP_ATTR(top_edge_spike_enable, spike_enable, -2);
IB_TOP_ATTR(top_edge_spike_khz, spike_khz, 4);
IB_TOP_ATTR(top_edge_spike_ms, spike_ms, -1);
IB_TOP_ATTR(top_edge_plateau_ms, plateau_ms, -1);
IB_TOP_ATTR(top_edge_decay_ms, decay_ms, -1);
IB_TOP_ATTR(top_edge_hold_ms, hold_ms, -1);

static ssize_t top_stats_show(struct class *class, struct class_attribute *attr,
			      char *buf)
{
	return scnprintf(
		buf, PAGE_SIZE,
		"requested_big_khz %u\nrequested_little_khz %u\n"
		"input_handler_enabled %u\ntouchscreens %d\n"
		"top_edge_enabled %u\ntriggers %lld\ncooldown_skips %lld\n"
		"requests_applied %lld\n"
		"requests_completed %lld\nfrequency_reached %lld\n"
		"stage %u\ndetected_ns %llu\nrequest_begin_ns %llu\n"
		"request_applied_ns %llu\nfrequency_reached_ns %llu\n"
		"frequency_at_apply_khz %u\ncurrent_big_khz %u\n"
		"effective_big_min_khz %d\neffective_big_max_khz %d\n"
		"little_at_apply_khz %u\neffective_little_min_khz %d\n"
		"stage_transitions %lld\nstage2_request_ns %llu\n"
		"released_ns %llu\n",
		READ_ONCE(top_active_freq), READ_ONCE(top_active_little),
		READ_ONCE(booster_enabled), atomic_read(&top_devices),
		READ_ONCE(top_enabled), (long long)atomic64_read(&top_triggers),
		(long long)atomic64_read(&top_cooldown_skips),
		(long long)atomic64_read(&top_applies),
		(long long)atomic64_read(&top_releases),
		(long long)atomic64_read(&top_reaches), READ_ONCE(top_stage),
		(unsigned long long)READ_ONCE(top_detect_ns),
		(unsigned long long)READ_ONCE(top_request_ns),
		(unsigned long long)READ_ONCE(top_applied_ns),
		(unsigned long long)READ_ONCE(top_reached_ns),
		READ_ONCE(top_freq_at_apply), cpufreq_quick_get(4),
		pm_qos_request(PM_QOS_CLUSTER1_FREQ_MIN),
		pm_qos_request(PM_QOS_CLUSTER1_FREQ_MAX),
		READ_ONCE(top_little_at_apply),
		pm_qos_request(PM_QOS_CLUSTER0_FREQ_MIN),
		(long long)atomic64_read(&top_transitions),
		(unsigned long long)READ_ONCE(top_stage2_ns),
		(unsigned long long)READ_ONCE(top_last_release_ns));
}
static CLASS_ATTR(top_edge_stats, 0444, top_stats_show, NULL);

static ssize_t top_schedule_show(struct class *class,
				 struct class_attribute *attr, char *buf)
{
	struct ib_top_plan plan;
	unsigned long flags;
	unsigned int i;
	ssize_t len = 0;

	spin_lock_irqsave(&top_lock, flags);
	plan = top_plan;
	spin_unlock_irqrestore(&top_lock, flags);
	len += scnprintf(buf, PAGE_SIZE, "ms big_khz little_khz phase\n");
	for (i = 0; i < plan.count; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u %u %u %u\n",
				 plan.p[i].ms, plan.p[i].big, plan.p[i].little,
				 plan.p[i].stage);
	return len;
}
static CLASS_ATTR(top_edge_schedule, 0444, top_schedule_show, NULL);

static struct class_attribute *top_attrs[] = {
	&class_attr_top_edge_enable,
	&class_attr_top_edge_region_percent,
	&class_attr_top_edge_stage1_big_khz,
	&class_attr_top_edge_stage1_little_khz,
	&class_attr_top_edge_stage1_ms,
	&class_attr_top_edge_stage2_big_khz,
	&class_attr_top_edge_stage2_little_khz,
	&class_attr_top_edge_stage2_ms,
	&class_attr_top_edge_schedule,
	&class_attr_top_edge_profile,
	&class_attr_top_edge_spike_enable,
	&class_attr_top_edge_spike_khz,
	&class_attr_top_edge_spike_ms,
	&class_attr_top_edge_plateau_ms,
	&class_attr_top_edge_decay_ms,
	&class_attr_top_edge_hold_ms,
	&class_attr_top_edge_stats,
};

static int ib_top_pm(struct notifier_block *nb, unsigned long event,
		     void *unused)
{
	bool enabled;

	if (event == PM_SUSPEND_PREPARE || event == PM_HIBERNATION_PREPARE) {
		mutex_lock(&top_control_lock);
		WRITE_ONCE(top_suspended, true);
		enabled = READ_ONCE(top_enabled);
		ib_top_disable();
		WRITE_ONCE(top_enabled, enabled);
		mutex_unlock(&top_control_lock);
	} else if (event == PM_POST_SUSPEND || event == PM_POST_HIBERNATION) {
		WRITE_ONCE(top_suspended, false);
	}
	return NOTIFY_OK;
}
static struct notifier_block top_pm_nb = {.notifier_call = ib_top_pm};

static int ib_top_init(void)
{
	int i, err;

	top_wq = alloc_ordered_workqueue("ib_top_edge", WQ_HIGHPRI);
	if (!top_wq)
		return -ENOMEM;
	INIT_WORK(&top_apply_work, ib_top_apply);
	INIT_DELAYED_WORK(&top_release_work, ib_top_release);
	pm_qos_add_request(&top_qos, PM_QOS_CLUSTER1_FREQ_MIN, 0);
	pm_qos_add_request(&top_little_qos, PM_QOS_CLUSTER0_FREQ_MIN, 0);
	err = cpufreq_register_notifier(&top_transition_nb,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (err)
		goto remove_qos;
	err = register_pm_notifier(&top_pm_nb);
	if (err)
		goto remove_transition;
	for (i = 0; i < ARRAY_SIZE(top_attrs); i++) {
		err = class_create_file(ib_class, top_attrs[i]);
		if (err)
			goto remove_attrs;
	}
	return 0;

remove_attrs:
	while (i--)
		class_remove_file(ib_class, top_attrs[i]);
	unregister_pm_notifier(&top_pm_nb);
remove_transition:
	cpufreq_unregister_notifier(&top_transition_nb,
				    CPUFREQ_TRANSITION_NOTIFIER);
remove_qos:
	pm_qos_remove_request(&top_qos);
	pm_qos_remove_request(&top_little_qos);
	destroy_workqueue(top_wq);
	top_wq = NULL;
	return err;
}

static void ib_top_exit(void)
{
	int i;

	if (!top_wq)
		return;
	for (i = 0; i < ARRAY_SIZE(top_attrs); i++)
		class_remove_file(ib_class, top_attrs[i]);
	unregister_pm_notifier(&top_pm_nb);
	mutex_lock(&top_control_lock);
	ib_top_disable();
	mutex_unlock(&top_control_lock);
	cpufreq_unregister_notifier(&top_transition_nb,
				    CPUFREQ_TRANSITION_NOTIFIER);
	pm_qos_remove_request(&top_qos);
	pm_qos_remove_request(&top_little_qos);
	destroy_workqueue(top_wq);
	top_wq = NULL;
}

static int ib_diag_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "enabled %u\nrecognized_input_events %lld\nsubmitted_events %lld\n"
		   "work_runs %lld\napplied_resource_updates %lld\nresource_releases %lld\n"
		   "queue_overflows %lld\nfailsafe_releases %lld\n",
		   READ_ONCE(diag_enabled),
		   (long long)atomic64_read(&recognized),
		   (long long)atomic64_read(&submitted),
		   (long long)atomic64_read(&work_runs),
		   (long long)atomic64_read(&applied_requests),
		   (long long)atomic64_read(&releases),
		   (long long)atomic64_read(&queue_overflows),
		   (long long)atomic64_read(&failsafe_releases));
	return 0;
}
static int ib_diag_open(struct inode *inode, struct file *file)
{
	return single_open(file, ib_diag_show, NULL);
}
static const struct file_operations ib_diag_fops = {
	.owner = THIS_MODULE,
	.open = ib_diag_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t ib_diag_reset(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	if (READ_ONCE(diag_enabled))
		return -EBUSY;
	atomic64_set(&recognized, 0);
	atomic64_set(&submitted, 0);
	atomic64_set(&work_runs, 0);
	atomic64_set(&applied_requests, 0);
	atomic64_set(&releases, 0);
	atomic64_set(&queue_overflows, 0);
	atomic64_set(&failsafe_releases, 0);
	return count;
}
static const struct file_operations ib_diag_reset_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = ib_diag_reset,
	.llseek = no_llseek,
};

static int ib_parse_dt(int *release_values)
{
	struct device_node *root, *child, *resource, *res;
	const char *str;
	static const char * const resource_names[IB_VENDOR_RES_COUNT] = {
		"cluster1", "cluster0", "mif", "int", "hmp", "ucc",
	};
	char values[100], *cursor, *token;
	int count, i, value, ret = 0;
	u32 type, pair[2];

	root = of_find_compatible_node(NULL, NULL, "input_booster");
	if (!root)
		return -ENODEV;
	ret = of_property_read_string(root, "max_resource_count", &str);
	if (ret || kstrtoint(str, 10, &count) || count != MAX_RES_COUNT) {
		ret = -EINVAL;
		goto out_root;
	}
	ret = of_property_read_string(root, "ib_release_values", &str);
	if (ret || strlcpy(values, str, sizeof(values)) >= sizeof(values)) {
		ret = -EINVAL;
		goto out_root;
	}
	cursor = values;
	for (i = 0; i < MAX_RES_COUNT; i++) {
		token = strsep(&cursor, ",");
		if (!token || kstrtoint(token, 10, &value) || value < 0) {
			ret = -EINVAL;
			goto out_root;
		}
		if (i < IB_VENDOR_RES_COUNT)
			release_values[i] = value;
		else if (value) {
			ret = -EINVAL;
			goto out_root;
		}
	}
	if (cursor) {
		ret = -EINVAL;
		goto out_root;
	}
	for_each_child_of_node(root, child) {
		ret = of_property_read_u32(child, "input_booster,type", &type);
		if (ret || type >= MAX_DEVICE_TYPE_NUM || policy_present[type]) {
			ret = -EINVAL;
			goto out_child;
		}
		ret = of_property_read_string(child, "input_booster,label",
					     &policies[type].label);
		ret |= of_property_read_u32(child, "input_booster,head_time",
					    &policies[type].head_time);
		ret |= of_property_read_u32(child, "input_booster,tail_time",
					    &policies[type].tail_time);
		if (ret)
			goto out_child;
		policies[type].type = type;
		resource = of_get_child_by_name(child, "inputbooster,resource");
		if (!resource) {
			ret = -EINVAL;
			goto out_child;
		}
		i = 0;
		for_each_child_of_node(resource, res) {
			if (i >= IB_VENDOR_RES_COUNT ||
			    of_property_read_string(res, "resource,label", &str) ||
			    strcmp(str, resource_names[i]) ||
			    of_property_read_u32_array(res, "resource,value", pair, 2) ||
			    pair[0] > INT_MAX || pair[1] > INT_MAX) {
				of_node_put(res);
				ret = -EINVAL;
				break;
			}
			policies[type].res[i].head_value = pair[0];
			policies[type].res[i].tail_value = pair[1];
			i++;
		}
		of_node_put(resource);
		if (ret || i != IB_VENDOR_RES_COUNT) {
			ret = -EINVAL;
			goto out_child;
		}
		policy_present[type] = true;
	}
	goto out_root;

out_child:
	of_node_put(child);
out_root:
	of_node_put(root);
	return ret;
}

static void ib_clear_worker(struct work_struct *work)
{
	ib_clear(0);
}

void input_booster_exit(void)
{
	int i;

	if (!ib_wq)
		return;
	mutex_lock(&enable_lock);
	shutting_down = true;
	ib_disable_locked();
	mutex_unlock(&enable_lock);
	debugfs_remove_recursive(diag_dir);
	ib_top_exit();
	if (ib_class) {
		for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++)
			if (policy_devices[i]) {
				device_remove_file(policy_devices[i], &dev_attr_head);
				device_remove_file(policy_devices[i], &dev_attr_tail);
				device_remove_file(policy_devices[i], &dev_attr_level);
				device_unregister(policy_devices[i]);
			}
		class_remove_file(ib_class, &class_attr_enable_event);
		class_remove_file(ib_class, &class_attr_debug_level);
		class_remove_file(ib_class, &class_attr_sendevent);
		class_destroy(ib_class);
		ib_class = NULL;
	}
	destroy_workqueue(ib_wq);
	input_booster_exit_vendor();
	ib_wq = NULL;
}

void input_booster_init(void)
{
	int release_values[IB_VENDOR_RES_COUNT] = { 0 };
	int i, err;

	err = ib_parse_dt(release_values);
	if (err) {
		pr_err("input_booster: invalid or missing device tree (%d)\n", err);
		return;
	}
	ib_wq = alloc_ordered_workqueue("input_booster", WQ_HIGHPRI);
	if (!ib_wq)
		return;
	INIT_WORK(&event_work, ib_event_worker);
	INIT_WORK(&clear_work, ib_clear_worker);
	input_booster_init_vendor(release_values);
	ib_class = class_create(THIS_MODULE, "input_booster");
	if (IS_ERR(ib_class)) {
		ib_class = NULL;
		goto err_vendor;
	}
	err = class_create_file(ib_class, &class_attr_enable_event);
	if (err) {
		class_destroy(ib_class);
		ib_class = NULL;
		goto err_vendor;
	}
	class_create_file(ib_class, &class_attr_debug_level);
	class_create_file(ib_class, &class_attr_sendevent);
	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (!policy_present[i])
			continue;
		policy_devices[i] = device_create(ib_class, NULL, 0,
					   &policies[i], "%s", policies[i].label);
		if (IS_ERR(policy_devices[i])) {
			policy_devices[i] = NULL;
			continue;
		}
		device_create_file(policy_devices[i], &dev_attr_head);
		device_create_file(policy_devices[i], &dev_attr_tail);
		device_create_file(policy_devices[i], &dev_attr_level);
	}
	err = ib_top_init();
	if (err)
		pr_warn("input_booster: top-edge boost unavailable (%d)\n",
			err);
	diag_dir = debugfs_create_dir("input_booster_diag", NULL);
	if (!IS_ERR_OR_NULL(diag_dir)) {
		debugfs_create_bool("enabled", 0600, diag_dir, &diag_enabled);
		debugfs_create_file("stats", 0400, diag_dir, NULL, &ib_diag_fops);
		debugfs_create_file("reset", 0200, diag_dir, NULL,
				    &ib_diag_reset_fops);
	} else {
		diag_dir = NULL;
	}
	return;
err_vendor:
	flush_work(&event_work);
	destroy_workqueue(ib_wq);
	input_booster_exit_vendor();
	ib_wq = NULL;
}

/* ACME registers its policies at device_initcall. Enable defaults only after
 * those calls and booster resource initialization, before starting userspace.
 */
static int __init ib_enable_boot_defaults(void)
{
	int err = 0;

	mutex_lock(&enable_lock);
	if (!ib_wq || shutting_down) {
		err = -ENODEV;
		goto out;
	}
	err = ib_enable_locked();
	if (err)
		goto out;
	mutex_lock(&top_control_lock);
	err = ib_top_enable_locked();
	mutex_unlock(&top_control_lock);
out:
	mutex_unlock(&enable_lock);
	if (err)
		pr_warn("input_booster: boot defaults incomplete (%d)\n", err);
	return err;
}
late_initcall(ib_enable_boot_defaults);
