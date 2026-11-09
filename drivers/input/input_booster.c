/* Exynos input booster.  The input callback only records small, owned events;
 * one ordered worker owns all request state and delayed releases.
 */
#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/input_booster.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define IB_QUEUE_SIZE 256
#define IB_MAX_INSTANCES 128
/* All shipped DTS head/tail windows are shorter; this only caps a lost OFF. */
#define IB_MAX_HOLD_MS 10000

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
static bool shutting_down; /* enable_lock */
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

#define IB_COUNT(counter) do { if (READ_ONCE(diag_enabled)) atomic64_inc(&(counter)); } while (0)

static void ib_recompute(void)
{
	struct ib_instance *item;
	int next[IB_VENDOR_RES_COUNT] = { 0 };
	int changes[IB_VENDOR_RES_COUNT] = { 0 };
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
	return 0;

unregister:
	input_unregister_handle(&bh->handle);
free_slots:
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
	ib_queue(&ev);
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
	if (handler_registered) {
		input_unregister_handler(&ib_handler);
		handler_registered = false;
	}
	/* No event producers remain; drain any self-requeued event work. */
	flush_workqueue(ib_wq);
	queue_work(ib_wq, &clear_work);
	flush_work(&clear_work);
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
			err = input_register_handler(&ib_handler);
			if (!err) {
				handler_registered = true;
				WRITE_ONCE(enable_generation,
					   enable_generation + 1);
				WRITE_ONCE(booster_enabled, true);
			}
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
