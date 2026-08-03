#include <linux/module.h>
#include <linux/init.h>
#include <linux/forcer.h>
#include <dt-bindings/clock/exynos9810.h>
#include <soc/samsung/cal-if.h>

unsigned int custom_disp_freq, custom_disp_voltage = 0;
unsigned int back_disp_freq, back_disp_voltage = 0;
static unsigned int disp_freqlist[5] = {640000, 534000, 400000, 200000, 134000};
unsigned int disp_voltlist[5] = {0, 0, 0, 0, 0};

void forcer_handle_display_on(void) {
    custom_disp_freq = 640000;
    custom_disp_voltage = disp_voltlist[0];

    fvmap_patch(DVFS_DISP_EVT1, back_disp_freq, custom_disp_voltage);

    cal_dfs_set_rate(ACPM_DVFS_DISP, custom_disp_freq);
}

void forcer_handle_display_off(void) {
    custom_disp_freq = 134000;
    custom_disp_voltage = disp_voltlist[4];

    fvmap_patch(DVFS_DISP_EVT1, back_disp_freq, custom_disp_voltage);

    cal_dfs_set_rate(ACPM_DVFS_DISP, custom_disp_freq);
}

void forcer_handle_display_alpm(void) {
    custom_disp_freq = 200000;
    custom_disp_voltage = disp_voltlist[3];

    fvmap_patch(DVFS_DISP_EVT1, back_disp_freq, custom_disp_voltage);

    cal_dfs_set_rate(ACPM_DVFS_DISP, custom_disp_freq);
}

unsigned long cal_dfs_check_forcer(unsigned int id) {
    unsigned long ret = 0;

    switch (id) {
	/* LITTLE */
	case ACPM_DVFS_DISP:
        ret = custom_disp_freq;
        break;
    default:
        break;
    }

    return ret;
}

void fill_voltlist(void) {
    int i;
    for (i = 0; i < 5; i++) {
        disp_voltlist[i] = fvmap_read(DVFS_DISP_EVT1, READ_VOLT, disp_freqlist[i]);
    }
}

static int __init forcer_init(void)
{
    fill_voltlist();
    back_disp_freq = 200000;
    custom_disp_freq = 200000;
    back_disp_voltage = disp_voltlist[3];
    custom_disp_voltage = disp_voltlist[3];
    return 0;
}

module_init(forcer_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos 9810 DISP freq/volt Forcer");