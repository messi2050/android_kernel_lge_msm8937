/*
 * EXTERNAL DSV MFD Driver
 *
 * Copyright 2014 LG Electronics Inc,
 *
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_EXTERNAL_DSV_H__
#define __MFD_EXTERNAL_DSV_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define MAX_REGISTERS 0x04

#define DW8768_POSITIVE_OUTPUT_CONTROL_REG	0X00
#define DW8768_NEGATIVE_OUTPUT_CONTROL_REG	0X01
#define DW8768_DISCHARGE_STATUS_CONTROL_REG 0x03
#define DW8768_ENABLE_REG 					0x05
#define DW8768_KNOCK_ON_CONTROL_REG 		0x07

#define SM5107_POSCNTL 	0x00
#define SM5107_NEGCNTL 	0x01
#define SM5107_CONTROL 	0x03
#define SM5107_CTRL_SET 0xFF

typedef enum {
	DSV_MODE_NORMAL = 0,
	DSV_MODE_LPWG,
	DSV_MODE_LPWG_ALWAYS_ON,
	DSV_MODE_LPWG_TOGGLE,
	DSV_MODE_SLEEP,
	DSV_MODE_POSITIVE_ONLY_ON,
	DSV_MODE_NORMAL_WITHOUT_DELAY,
	DSV_MODE_POWER_OFF,
	DSV_MODE_POWER_OFF_HIZ,
	DSV_MODE_OUTPUT_VOLTAGE,
	DSV_MODE_MAX_NUM
} dsv_mode;

typedef enum {
	EXT_DSV_ACCESS_LEVEL_UNPRIVILEGED = 0,
	EXT_DSV_ACCESS_LEVEL_PRIVILEGED
} dsv_access_level;

#ifdef EXT_DSV_PRIVILEGED
extern int ext_dsv_chip_enable(int enable);
extern int ext_dsv_register_set(u8 address, u8 value);
extern void ext_dsv_set_access_level(int level);
#define ext_dsv_mode_change ext_dsv_mode_change_privileged
#else
#define ext_dsv_mode_change ext_dsv_mode_change_unprivileged
#endif
extern int ext_dsv_mode_change(int mode);

#endif
