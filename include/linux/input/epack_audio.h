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
#ifndef __E_PACK_AUDIO__
#define __E_PACK_AUDIO__

#include <linux/switch.h>
#include <linux/input/epack_core.h>

void audio_input_init(struct i2c_client *client , struct epack_dev_data *data);
void audio_input_set_sdev_name(struct epack_dev_data *data, int status);
int audio_get_epack_state(void);

#define EPACK_STATE		"epack"

enum {
	EPACK_ENABLE   = (1 << 7),
	EPACK_DISABLE = 0,
};

#endif //__E_PACK__
