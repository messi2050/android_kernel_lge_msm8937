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

#include <linux/switch.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/epack_core.h>
#include <linux/input/epack_audio.h>

static int epack_state = 0;

void epack_audio_work_func(struct work_struct *audio_work)
{
	struct epack_dev_data *epack = container_of(to_delayed_work(audio_work), struct epack_dev_data, audio_work);
	static int count = 0;

	pr_err("[%s] status : %d, count : %d\n",__func__,get_epack_status(),count++);

	schedule_delayed_work(&epack->audio_work, msecs_to_jiffies(13000));

	return;
}

void audio_input_init(struct i2c_client *client , struct epack_dev_data *data)
{
	int err;
	pr_debug("[%s] enter\n", __func__);
 	data->sdev.name = EPACK_STATE;
	err = switch_dev_register(&data->sdev);
	if (err < 0) {
        pr_err("[%s] Failed to register switch device\n", __func__);
        switch_dev_unregister(&data->sdev);
	}
}

void audio_input_set_sdev_name(struct epack_dev_data *data, int status)
{
	pr_debug("[%s] enter\n", __func__);

	epack_state = status;

	if (status)
		switch_set_state(&data->sdev, EPACK_ENABLE);
	else
		switch_set_state(&data->sdev, EPACK_DISABLE);

	pr_info("[%s] sdev.name: %s, epack state: %d\n", __func__, data->sdev.name, epack_state);
}

int audio_get_epack_state(void)
{
	return epack_state;
}

