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

#ifndef __MFD_EXT_DSV_SM5107_H__
#define __MFD_EXT_DSV_SM5107_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define MAX_REGISTERS 0x04

#define SM5107_CONTROL 0x03
#define SM5107_CTRL_SET 0xFF

struct ext_dsv_platform_data {
	const char *name;
};

typedef enum {
	MODE_NORMAL,
	MODE_KNOCKON,
	POWER_OFF,
} dsv_mode;

struct ext_dsv {
	struct device *dev;
	struct regmap *regmap;
	struct ext_dsv_platform_data *pdata;
};

extern int ext_dsv_mode_change(int mode);
int ext_dsv_register_set(u8 address, u8 value);
#endif
