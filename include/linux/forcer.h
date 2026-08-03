enum dvfs_id {
	DVFS_MIF = 0,
	DVFS_INT,
	DVFS_CPUCL0,
	DVFS_CPUCL1,
	DVFS_G3D,
	DVFS_INTCAM,
	DVFS_FSYS0,
	DVFS_CAM,
	DVFS_DISP_EVT1,
	DVFS_AUD,
	DVFS_IVA,
	DVFS_SCORE,
	DVFS_CP,
};

#ifdef CONFIG_FORCER
extern unsigned long cal_dfs_check_forcer(unsigned int id);
extern void forcer_handle_display_on(void);
extern void forcer_handle_display_off(void);
extern void forcer_handle_display_alpm(void);
#endif