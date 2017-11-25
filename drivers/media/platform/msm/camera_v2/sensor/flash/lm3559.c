/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#define FLASH_NAME "ti,lm3559"

#define LM3559_DBG(fmt, args...) pr_err(fmt, ##args)

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3559_i2c_driver;

#if defined(CONFIG_MACH_MSM8916_M209_TRF_US) || defined(CONFIG_MACH_MSM8916_M209_TRF_US_VZW)
static struct msm_camera_i2c_reg_array lm3559_init_array[] = {
	{0x10, 0x00},
	{0x80, 0x09},
	{0xA0, 0x12},
	{0xB0, 0x77}, //900mA total
};
#else
static struct msm_camera_i2c_reg_array lm3559_init_array[] = {
	{0x10, 0x00},
	{0x80, 0x09},
	{0xA0, 0x09},
	{0xB0, 0x11}, //1012.5mA total
};
#endif

static struct msm_camera_i2c_reg_array lm3559_off_array[] = {
	{0x10, 0x00},
};

static struct msm_camera_i2c_reg_array lm3559_release_array[] = {
	{0x10, 0x00},
};

static struct msm_camera_i2c_reg_array lm3559_low_array[] = {
	{0x10, 0x1A},
};

static struct msm_camera_i2c_reg_array lm3559_high_array[] = {
	{0x10, 0x1B},
};


static const struct of_device_id lm3559_i2c_trigger_dt_match[] = {
	{.compatible = "ti,lm3559"},
	{}
};

MODULE_DEVICE_TABLE(of, lm3559_i2c_trigger_dt_match);
static const struct i2c_device_id lm3559_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value > LED_OFF) {
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3559_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

int msm_flash_lm3559_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3559_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return rc;
}

int msm_flash_lm3559_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	/* LGE_CHANGE_S, Fix to NULL pointer dereference, 2016-01-14, hongs.lee@lge.com */
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	/* LGE_CHANGE_E, Fix to NULL pointer dereference, 2016-01-14, hongs.lee@lge.com */
	
	LM3559_DBG("%s:%d called\n", __func__, __LINE__);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	return 0;
}

int msm_flash_lm3559_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	/* LGE_CHANGE_S, Fix to NULL pointer dereference, 2016-01-14, hongs.lee@lge.com */
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	/* LGE_CHANGE_S, Fix to NULL pointer dereference, 2016-01-14, hongs.lee@lge.com */

	LM3559_DBG("%s:%d called\n", __func__, __LINE__);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_lm3559_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint16_t flag;
	LM3559_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;


	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
				fctrl->flash_i2c_client, fctrl->reg_setting->low_setting);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		} else {
			fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
				fctrl->flash_i2c_client, 0xD0, &flag, MSM_CAMERA_I2C_BYTE_DATA);
			pr_info("%s flag = 0x%x\n", __func__, flag);
		}
	}


	return rc;
}

int msm_flash_lm3559_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint16_t flag;
	LM3559_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);


	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		} else {
			fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
				fctrl->flash_i2c_client, 0xD0, &flag, MSM_CAMERA_I2C_BYTE_DATA);
			pr_info("%s flag = 0x%x\n", __func__, flag);
		}
	}

	return rc;
}
static int msm_flash_lm3559_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	pr_err("[dongsu] probe lm3559 %s entry\n", __func__);
	if (!id) {
		pr_err("msm_flash_lm3559_i2c_probe: id is NULL");
		id = lm3559_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

	if (!rc)
		msm_lm3559_torch_create_classdev(&(client->dev),NULL);
	return rc;
}

static int msm_flash_lm3559_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	LM3559_DBG("%s entry\n", __func__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	return rc;
}


static struct i2c_driver lm3559_i2c_driver = {
	.id_table = lm3559_i2c_id,
	.probe  = msm_flash_lm3559_i2c_probe,
	.remove = msm_flash_lm3559_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3559_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_lm3559_init(void)
{
	LM3559_DBG("%s entry\n", __func__);
	return i2c_add_driver(&lm3559_i2c_driver);
}

static void __exit msm_flash_lm3559_exit(void)
{
	LM3559_DBG("%s entry\n", __func__);
	i2c_del_driver(&lm3559_i2c_driver);
	return;
}


static struct msm_camera_i2c_client lm3559_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3559_init_setting = {
	.reg_setting = lm3559_init_array,
	.size = ARRAY_SIZE(lm3559_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3559_off_setting = {
	.reg_setting = lm3559_off_array,
	.size = ARRAY_SIZE(lm3559_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3559_release_setting = {
	.reg_setting = lm3559_release_array,
	.size = ARRAY_SIZE(lm3559_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3559_low_setting = {
	.reg_setting = lm3559_low_array,
	.size = ARRAY_SIZE(lm3559_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3559_high_setting = {
	.reg_setting = lm3559_high_array,
	.size = ARRAY_SIZE(lm3559_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3559_regs = {
	.init_setting = &lm3559_init_setting,
	.off_setting = &lm3559_off_setting,
	.low_setting = &lm3559_low_setting,
	.high_setting = &lm3559_high_setting,
	.release_setting = &lm3559_release_setting,
};

static struct msm_flash_fn_t lm3559_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3559_led_init,
	.flash_led_release = msm_flash_lm3559_led_release,
	.flash_led_off = msm_flash_lm3559_led_off,
	.flash_led_low = msm_flash_lm3559_led_low,
	.flash_led_high = msm_flash_lm3559_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3559_i2c_client,
	.reg_setting = &lm3559_regs,
	.func_tbl = &lm3559_func_tbl,
};

module_init(msm_flash_lm3559_init);
module_exit(msm_flash_lm3559_exit);
MODULE_DESCRIPTION("lm3559 FLASH");
MODULE_LICENSE("GPL v2");
