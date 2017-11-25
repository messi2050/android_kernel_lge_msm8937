/*
 * Copyright(c) 2016, LG Electronics. All rights reserved.
 *
 * e-pack i2c device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __E_PACK__
#define __E_PACK__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/power_supply.h>
#include <linux/switch.h>

enum epack_status{
	EPACK_ABSENT,
	EPACK_POWER_LOW,
	EPACK_POWER_OK,
	EPACK_STATUS_MAX,
};

enum vbus_source_type{
	NONE,
	FROM_EPACK,
	FROM_USB_PORT,
};

struct epack_dev_data {
	struct i2c_client *client;
	int last_status;
	int vbus_src;

	int polling_time_snd;
	int polling_time_pwr;
	int id_gpio;
	int pack_power_gpio;
	int usb_power_gpio;
	int ovp_sw_pcon_gpio;

	unsigned int hwver;
	unsigned int fwver;
	unsigned int bin_fwver[2];
	unsigned int devid;
	unsigned int devconfig[2];
	bool force_update;

	int slavemode;

	int batt_voltage;
	int batt_temp;
	int chg_status;
	int fault_status;
	int output_status;
	struct mutex irq_lock;
	struct mutex i2c_lock;
	struct workqueue_struct *wq;
	struct delayed_work audio_work;
	struct delayed_work pwr_monitor_work;
	struct delayed_work notify_epack_ready_work;
	struct delayed_work notify_epack_unready_work;
	struct delayed_work notify_vbus_src_work;
	struct power_supply *usb_psy;
	struct switch_dev sdev;
};

int get_epack_status(void);
int get_vbus_source(void);
void epack_audio_work_func(struct work_struct *upgrade_work);
void epack_pwr_monitor_work(struct work_struct *upgrade_work);
int epack_firmware_update(struct i2c_client *client, struct device *dev,int flag, char* fwpath);
bool check_fw_update(unsigned int fwver,unsigned int hwver);
int get_fw_ver_from_file(char* fwpath, struct device *dev);

#define MAX_RETRY_COUNT 5
#define epack_log(fmt, args...) 	printk(KERN_ERR "[Epack] " fmt, ##args)
#endif //__E_PACK__
