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

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <linux/input/epack_core.h>
#include "cmd.h"

void epack_pwr_monitor_work(struct work_struct *pwr_monitor_work)
{
	struct epack_dev_data *epack = container_of(to_delayed_work(pwr_monitor_work)
					,struct epack_dev_data, pwr_monitor_work);

	bool rc;

	rc = cmd_get_pack_voltage(epack,&(epack->batt_voltage));
	rc &= cmd_get_pack_temp(epack,&(epack->batt_temp));
	rc &= cmd_get_pack_chg_status(epack,&(epack->chg_status));
	rc &= cmd_get_pack_fault_status(epack,&(epack->fault_status));
	rc &= cmd_get_pack_output_status(epack,&(epack->output_status));

	pr_err("[EpackMonitor] rc=%d V:%dmV T:%d chg:0x%x fault:0x%x output:0x%x \n"
			,rc,epack->batt_voltage,epack->batt_temp
			,epack->chg_status,epack->fault_status,epack->output_status);

	schedule_delayed_work(&epack->pwr_monitor_work,msecs_to_jiffies(epack->polling_time_pwr));

	return;
}
