/*
 * Platform data for MAX98925
 *
 * Copyright 2011-2012 Maxim Integrated Products
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __SOUND_MAX98925_PDATA_H__
#define __SOUND_MAX98925_PDATA_H__

#include <sound/maxim_dsm.h>

#define MAX98925_I2C_ADDRESS	(0x62 >> 1)

struct max98925_dsp_cfg {
	const char *name;
	u8 tx_dither_en;
	u8 rx_dither_en;
	u8 meas_dc_block_en;
	u8 rx_flt_mode;
};

/* MAX98925 volume step */
#define MAX98925_VSTEP_0		0
#define MAX98925_VSTEP_1		1
#define MAX98925_VSTEP_2		2
#define MAX98925_VSTEP_3		3
#define MAX98925_VSTEP_4		4
#define MAX98925_VSTEP_5		5
#define MAX98925_VSTEP_6		6
#define MAX98925_VSTEP_7		7
#define MAX98925_VSTEP_8		8
#define MAX98925_VSTEP_9		9
#define MAX98925_VSTEP_10		10
#define MAX98925_VSTEP_11		11
#define MAX98925_VSTEP_12		12
#define MAX98925_VSTEP_13		13
#define MAX98925_VSTEP_14		14
#define MAX98925_VSTEP_15		15
#define MAX98925_VSTEP_MAX		MAX98925_VSTEP_15

#ifdef CONFIG_SND_SOC_MAXIM_DSM_CAL
extern struct class *g_class;
#else
struct class *g_class;
#endif /* CONFIG_SND_SOC_MAXIM_DSM_CAL */

struct max98925_volume_step_info {
	int length;
	int vol_step;
	int adc_thres;
	int boost_step[MAX98925_VSTEP_MAX + 1];
	bool adc_status;
};

/*
 * codec platform data.
 * This definition should be changed,
 * if platform_info of device tree is changed.
 */
#ifdef CONFIG_SND_SOC_MAXIM_DSM
#define MAX98925_PINFO_SZ	PARAM_OFFSET_MAX
#else
#define MAX98925_PINFO_SZ	6
#endif /* CONFIG_SND_SOC_MAXIM_DSM */

struct max98925_pdata {
	int sysclk;
	u32 spk_vol;
	u32 vmon_slot;
	u32 capture_active;
	u32 playback_active:1;
	bool i2c_pull_up;
#ifdef USE_MAX98925_IRQ
	int irq;
#endif
	uint32_t pinfo[MAX98925_PINFO_SZ];
	struct max98925_volume_step_info vstep;
	const uint32_t *reg_arr;
	uint32_t reg_arr_len;
	int sub_reg;
	int sub_bus;
	const uint32_t *regcfg;
	uint32_t regcfg_sz;
	bool nodsm;
};

void max98925_spk_enable(int enable);
#endif
