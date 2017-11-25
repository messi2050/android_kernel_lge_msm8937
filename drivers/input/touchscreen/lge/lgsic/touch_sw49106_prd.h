/* production_test.h
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw49106.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* production test */
#define tc_test_mode_ctl 		(0xC6E)
#define cmd_test_exit 			(0x0000)
#define cmd_test_enter 			(0x0001)

#define tc_tsp_test_ctl			(0xC04)
#define tc_tsp_test_sts			(0x265)
#define tc_tsp_test_pf_result   (0x266)
#define tc_tsp_test_off_info	(0x2FB)
#define tc_tsp_test_data_offset	(0x0AF)
#define tc_tsp_data_access_addr (0xFD1)
#define tune_result_offset      (0x289)


#define RAWDATA_OFFSET			(0xE00)
#define rawdata_ctl_read		(0x2A4)
#define rawdata_ctl_write		(0xC49)

//Product test
#define Serial_Data_Offset		(0x0AF)// [bringup] (0x007B) // tc_tsp_data_access_addr
#define DATA_BASE_ADDR 			(0xFD1)//  [bringup] (0x301) // tc_tsp_data_access_addr
//#define m1_raw_data_offset		(0x02D7)// [bringup] (0x0287)
#define m2_raw_data_offset		(0x0286)// [bringup] (0x0287)
//#define prod_open1_open2_offset (0x02D7)// [bringup] (0x0287)
#define prod_open3_short_offset	(0x0288)// [bringup] (0x0288)
#define M1_M2_RAWDATA_TEST_CNT_MAX	(2)
#define M1_M2_RAWDATA_TEST_CNT	(1)
#define LINE_FILTER_OPTION		(0x40000)

//Firmware debugging
#define AIT_RAW_DATA_OFFSET 		(0xF88)//(0xB85)// [bringup] ((0xA8C)
#define DELTA_DATA_OFFSET   		(0x1110)//(0xDC5)// [bringup] (0xD95)
#define LABLE_DATA_OFFSET   		(0x1200)//(0xF19)// [bringup] (0xE83)
#define AIT_BASE_DATA_OFFSET    	(0x104C)//(0xCA5)// [bringup] (0xC0F)
#define AIT_DEBUG_BUF_DATA_OFFSET			(0xC3F)//(0xB0F)// [bringup] (0xBCF)
#define FILTERED_DELTA_DATA_OFFSET 			(0x7FD)
#define ADDR_CMD_REG_SIC_IMAGECTRL_TYPE		(0x0C6C)
#define ADDR_CMD_REG_SIC_GETTER_READYSTATUS	(0x0C64)

struct sw49106_test_off {
	u16 offset0;
	u16 offset1;
} __packed;

struct sw49106_test_off_info {
	struct sw49106_test_off m1_m2_raw;
	struct sw49106_test_off frame0_1;
	struct sw49106_test_off frame2_short;
	struct sw49106_test_off os_result;
} __packed;

/* tune code */
#define tc_tune_code_size		260
#define tc_total_ch_size		32
#define TSP_TUNE_CODE_L_GOFT_OFFSET		0
#define TSP_TUNE_CODE_L_M1_OFT_OFFSET		2
#define TSP_TUNE_CODE_L_G1_OFT_OFFSET		(TSP_TUNE_CODE_L_M1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_L_G2_OFT_OFFSET	(TSP_TUNE_CODE_L_G1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_L_G3_OFT_OFFSET		(TSP_TUNE_CODE_L_G2_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_GOFT_OFFSET		(TSP_TUNE_CODE_L_G3_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_M1_OFT_OFFSET		(TSP_TUNE_CODE_R_GOFT_OFFSET + 2)
#define TSP_TUNE_CODE_R_G1_OFT_OFFSET		(TSP_TUNE_CODE_R_M1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_G2_OFT_OFFSET		(TSP_TUNE_CODE_R_G1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_G3_OFT_OFFSET		(TSP_TUNE_CODE_R_G2_OFT_OFFSET + tc_total_ch_size)
#define PATH_SIZE		64
#define BURST_SIZE		512
#define RAWDATA_SIZE		2
#define ROW_SIZE		28
#define SHORT_ROW_SIZE		tc_total_ch_size
#define COL_SIZE		14
#define M1_COL_SIZE		2
#define SHORT_COL_SIZE		4
#define LOG_BUF_SIZE		256
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define DEBUG_ROW_SIZE		ROW_SIZE
#define DEBUG_COL_SIZE		COL_SIZE

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

//AIT IT_IMAGE CMD
enum {
	CMD_NONE = 0,
	CMD_RAWDATA,
	CMD_BASE_DATA,
	CMD_DELTADATA,
	CMD_LABELDATA,
	CMD_FILTERED_DELTA,    // Not used
	CMD_RESERVED,			// Not used
	CMD_DEBUGDATA,
	DONT_USE_CMD = 0xEE,
	CMD_WAIT = 0xFF,
};

enum {
	NO_TEST = 0,
	OPEN_SHORT_ALL_TEST,
	OPEN_NODE_TEST,		//type_temp = 0x2;
	SHORT_NODE_TEST,	//type_temp = 0x3;
	U3_M2_RAWDATA_TEST,	//type_temp = 0x5;
	U3_M1_RAWDATA_TEST,	//type_temp = 0x6;
	U0_M2_RAWDATA_TEST,	//type_temp = 0x5;
	U0_M1_RAWDATA_TEST,	//type_temp = 0x6;
	U3_JITTER_TEST, 	//type_temp = 0xC;
	U0_JITTER_TEST, 	//type_temp = 0xC;
	U3_BLU_JITTER_TEST,	//type_temp = 0xC;
};

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

typedef enum
{
    IT_NONE = 0,
    IT_ALGORITHM_RAW_IMAGE,
    IT_BASELINE_E_IMAGE,
    IT_BASELINE_O_IMAGE,
    IT_DELTA_IMAGE,
    IT_LABEL_IMAGE,
    IT_FILTERED_DELTA_IMAGE,
    IT_WAIT = 0xFF
} eImageType_t;

typedef enum
{
    RS_READY    = 0xA0,
    RS_NONE     = 0x05,
    RS_LOG      = 0x77,
    RS_IMAGE	= 0xAA
} eProtocolReadyStatus_t;

extern void touch_msleep(unsigned int msecs);
int sw49106_prd_register_sysfs(struct device *dev);

/* For BLU test. We need to control backlight level. */
#if defined(CONFIG_TOUCHSCREEN_MTK)
extern unsigned int mt_get_bl_brightness(void);
extern int mt65xx_leds_brightness_set(int, int);
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern int mdss_fb_get_bl_brightness_extern(void);
extern void mdss_fb_set_bl_brightness_extern(int);
#endif

#endif

