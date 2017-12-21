/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_EEPROM_H
#define MSM_EEPROM_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_spi.h"
#include "msm_camera_io_util.h"
#include "msm_camera_dt_util.h"

struct msm_eeprom_ctrl_t;

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define PROPERTY_MAXSIZE 32

#define VENDOR_ID_ADDR 0x700 /*LGE_CHANGE, define vendor id address, 2016-12-23, jungryoul.choi@lge.com */

struct msm_eeprom_ctrl_t {
	struct platform_device *pdev;
	struct mutex *eeprom_mutex;

	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *eeprom_v4l2_subdev_ops;
	enum msm_camera_device_type_t eeprom_device_type;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_master;
	enum i2c_freq_mode_t i2c_freq_mode;

	struct msm_camera_i2c_client i2c_client;
	struct msm_eeprom_board_info *eboard_info;
	uint32_t subdev_id;
	int32_t userspace_probe;
	struct msm_eeprom_memory_block_t cal_data;
	uint8_t is_supported;
/*LGE_CHANGE_S, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
    uint8_t AAT_Checksum;
	enum camb_position_t position;
/*LGE_CHANGE_E, Make EEPROM CheckSum Property for AAT Mode, 2016-02-23, seungmin.hong@lge.com*/
#if defined(CONFIG_MSM_OTP)
	struct list_head link;
#endif
};

//EEPROM Code Refinement, Camera-Driver@lge.com, 2015-06-11
typedef enum {
	BigEndian,
	LittleEndian,
} Endian;

//Module Selector
int32_t msm_eeprom_checksum_imtech(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_cowell(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_sunny(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_s5k5e2_eeprom_checksum(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_ofilm(struct msm_eeprom_ctrl_t *e_ctrl);

//Module CheckSum routine
int32_t msm_eeprom_checksum_cowell_hi841(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_cowell_hi842(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_cowell_mn34153(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_imtech_ov8858(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_imtech_t4kb3(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_imtech_hi841(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_v0d(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_v0d_t4ka3(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_sunny_ov13850(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_sunny_hi1332(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_ofilm_hi1332(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_at24c16d(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_hi1332(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_imx258(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_hi842(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_imx234(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_m24c32_hi553(struct msm_eeprom_ctrl_t *e_ctrl);

//Helper function for arithmetic shifted addition / just accumulation
uint32_t shiftedSum (struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr, Endian endian);
uint32_t accumulation (struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr);
#endif
