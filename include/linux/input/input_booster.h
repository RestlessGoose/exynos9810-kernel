#if !defined(CONFIG_INPUT_BOOSTER)
#ifndef _SEC_INPUT_BOOSTER_H
#define _SEC_INPUT_BOOSTER_H

#include <linux/types.h>

/* The Exynos backend implements six resources; the DT has a reserved seventh. */
#define IB_VENDOR_RES_COUNT 6
#define MAX_RES_COUNT 7

enum ib_device_type {
	KEY,
	TOUCH_KEY,
	TOUCH,
	MULTI_TOUCH,
	KEYBOARD,
	MOUSE,
	MOUSH_WHEEL,
	HOVER,
	SPEN,
	KEY_TWO,
	MAX_DEVICE_TYPE_NUM,
};

struct t_ib_res_info {
	int head_value;
	int tail_value;
};

struct t_ib_device_tree {
	const char *label;
	u32 type;
	u32 head_time;
	u32 tail_time;
	struct t_ib_res_info res[IB_VENDOR_RES_COUNT];
};

void input_booster_init(void);
void input_booster_exit(void);
void input_booster_init_vendor(int *release_val);
void input_booster_exit_vendor(void);
void ib_set_booster(int *qos_values);
void ib_release_booster(int res_id);

#endif
#endif /* !CONFIG_INPUT_BOOSTER */
