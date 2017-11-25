/* touch_ft8707.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hyokmin.kwon@lge.com
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

#ifndef LGE_TOUCH_FT8707_H
#define LGE_TOUCH_FT8707_H

//#include "touch_ft8707_watch.h"
//#include "touch_ft8707_asc.h"	/* ASC */

/* debug info */
#define DEBUG_BUF_SIZE 512

struct ft8707_touch_debug {
	u32 padding[3];	/* packet structure 4+4+4+12*10+12+112 */
	u8 protocol_ver;
	u8 reserved_1;
	u32 frame_cnt;
	u8 rn_max_bfl;
	u8 rn_max_afl;
	u8 rn_min_bfl;
	u8 rn_min_afl;
	u8 rn_max_afl_x;
	u8 rn_max_afl_y;
	s8 lf_oft[18];
	u8 seg1_cnt:4;
	u8 seg2_cnt:4;
	u8 seg1_thr;
	u8 rebase[8];
	u8 rn_pos_cnt;
	u8 rn_neg_cnt;
	u8 rn_pos_sum;
	u8 rn_neg_sum;
	u8 rn_stable;
	u8 track_bit[10];
	u8 rn_max_tobj[12];
	u8 palm[8];
	u8 noise_detect[8];
	u8 reserved_2[21];
	u32 ic_debug[3];
	u32 ic_debug_info;
} __packed;

#if 0
/* report packet */
struct ft8707_touch_data {
	u8 tool_type:4;
	u8 event:4;
	s8 track_id;
	u16 x;
	u16 y;
	u8 pressure;
	s8 angle;
	u16 width_major;
	u16 width_minor;
} __packed;
#endif

struct ft8707_touch_data {
	u8 xh;
	u8 xl;
	u8 yh;
	u8 yl;
	u8 weight;
	u8 area;
} __packed;


struct ft8707_touch_info {
	u32 ic_status;
	u32 device_status;
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 palm_bit:16;
	struct ft8707_touch_data data[10];
	/* debug info */
	struct ft8707_touch_debug debug;
} __packed;



/* Definitions for FTS */

#define FTS_PACKET_LENGTH 	32

#define MAX_TAP_COUNT 		12

#define FT5X06_ID			0x55
#define FT5X16_ID			0x0A
#define FT5X36_ID			0x14
#define FT6X06_ID			0x06
#define FT6X36_ID			0x36

#define FT5316_ID			0x0A
#define FT5306I_ID			0x55

#define LEN_FLASH_ECC_MAX 		0xFFFE

#define FTS_MAX_POINTS			10

#define FTS_WORKQUEUE_NAME		"fts_wq"

#define FTS_DEBUG_DIR_NAME		"fts_debug"

#define FTS_INFO_MAX_LEN		512
#define FTS_FW_NAME_MAX_LEN		50

#define FTS_REG_ID				0xA3	// Chip Selecting (High)
#define FTS_REG_ID_LOW			0x9F	// Chip Selecting (Low)
#define FTS_REG_FW_VER			0xA6 	// Firmware Version (Major)
#define FTS_REG_FW_VENDOR_ID	0xA8	// Focaltech's Panel ID
#define FTS_REG_LIB_VER_H		0xA1 	// LIB Version Info (High)
#define FTS_REG_LIB_VER_L		0xA2	// LIB Version Info (Low)
#define FTS_REG_FW_VER_MINOR	0xB2	// Firmware Version (Minor)
#define FTS_REG_FW_VER_SUB_MINOR	0xB3	// Firmware Version (Sub-Minor)


#define FTS_REG_POINT_RATE		0x88

#define FTS_FACTORYMODE_VALUE	0x40
#define FTS_WORKMODE_VALUE		0x00




#define FTS_META_REGS			3
#define FTS_ONE_TCH_LEN			6
#define FTS_TCH_LEN(x)			(FTS_META_REGS + FTS_ONE_TCH_LEN * x)

#define FTS_PRESS				0x7F
#define FTS_MAX_ID				0x0F
#define FTS_TOUCH_P_NUM			2
#define FTS_TOUCH_X_H_POS		3
#define FTS_TOUCH_X_L_POS		4
#define FTS_TOUCH_Y_H_POS		5
#define FTS_TOUCH_Y_L_POS		6
#define FTS_TOUCH_PRE_POS		7
#define FTS_TOUCH_AREA_POS		8
#define FTS_TOUCH_POINT_NUM		2
#define FTS_TOUCH_EVENT_POS		3
#define FTS_TOUCH_ID_POS		5

#define FTS_TOUCH_DOWN			0
#define FTS_TOUCH_UP			1
#define FTS_TOUCH_CONTACT		2

#define POINT_READ_BUF			(3 + FTS_ONE_TCH_LEN * FTS_MAX_POINTS)


/* charger status */
#define SPR_CHARGER_STS			(0xC50)

#define CONNECT_NONE			(0x00)
#define CONNECT_USB			(0x01)
#define CONNECT_TA			(0x02)
#define CONNECT_OTG			(0x03)
#define CONNECT_WIRELESS		(0x10)

enum {
	SW_RESET = 0,
	HW_RESET,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	ABS_MODE = 0,
	KNOCK_1,
	KNOCK_2,
	SWIPE_DOWN,
	SWIPE_UP,
	CUSTOM_DEBUG = 200,
	KNOCK_OVERTAP = 201,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};


enum {
	TCI_CTRL_SET = 0,
	TCI_CTRL_CONFIG_COMMON,
	TCI_CTRL_CONFIG_TCI_1,
	TCI_CTRL_CONFIG_TCI_2,
};


#if 0
struct ft8707_version {
	u8 build : 4;
	u8 major : 4;
	u8 minor;
};

struct ft8707_ic_info {
	struct ft8707_version version;
	u8 product_id[8];
	u8 image_version[2];
	u8 image_product_id[8];
	u8 revision;
	u32 wfr;
	u32 cg;
	u32 fpc;
};
#endif 

struct ft8707_version {
	u8 major;
	u8 minor;
	u8 sub_minor;
};

struct ft8707_ic_info {
	struct ft8707_version version;
	u8 info_valid;
	u8 chip_id;
	u8 chip_id_low;
	u8 is_official;
	u8 fw_version;
	u8 fw_vendor_id;
	u8 lib_version_high;
	u8 lib_version_low;
	u8 is_official_bin;
	u8 fw_version_bin;
};

enum {
	TC_STATE_ACTIVE = 0,
	TC_STATE_LPWG,
	TC_STATE_DEEP_SLEEP,
	TC_STATE_POWER_OFF,
};

struct ft8707_data {
	struct device *dev;
	struct kobject kobj;
	struct ft8707_touch_info info;
	struct ft8707_ic_info ic_info;
//	struct ft8707_asc_info asc;	/* ASC */
	struct workqueue_struct *wq_log;
	u8 lcd_mode;
	u8 prev_lcd_mode;
	u8 driving_mode;
	u8 u3fake;
//	struct watch_data watch;
//	struct swipe_ctrl swipe;
//	struct mutex spi_lock;
	struct mutex rw_lock;
	struct mutex fb_lock;
	struct delayed_work font_download_work;
	struct delayed_work fb_notify_work;
	struct delayed_work debug_info_work;
	u32 charger;
	u32 earjack;
	u32 frame_cnt;
	u8 tci_debug_type;
//	u8 swipe_debug_type;
	atomic_t block_watch_cfg;
	atomic_t init;
	u8 state;
	u8 chip_rev; // 0 : Cut 1.1 , 1 : Cut 1.0
	u8 en_i2c_lpwg;
	struct notifier_block fb_notif;
};


#ifdef CONFIG_LGE_MIPI_DSI_LGD_K7J_FHD_VIDEO_INCELL_LCD_PANEL

extern void lge_incell_lcd_external_api(int type, int enable);

#define LCD_RESET_H 	lge_incell_lcd_external_api(2, 1)
#define LCD_RESET_L 	lge_incell_lcd_external_api(2, 0)
#define LCD_DSV_ON 		lge_incell_lcd_external_api(1, 1)
#define LCD_DSV_OFF 	lge_incell_lcd_external_api(1, 0)
#define LCD_VDDI_ON 	lge_incell_lcd_external_api(0, 1)
#define LCD_VDDI_OFF 	lge_incell_lcd_external_api(0, 0)

#else

#define LCD_RESET_H
#define LCD_RESET_L
#define LCD_DSV_ON
#define LCD_DSV_OFF
#define LCD_VDDI_ON
#define LCD_VDDI_OFF

#endif


#define TCI_MAX_NUM			2
#define SWIPE_MAX_NUM			2
#define TCI_DEBUG_MAX_NUM			16
#define SWIPE_DEBUG_MAX_NUM			8
#define DISTANCE_INTER_TAP		(0x1 << 1) /* 2 */
#define DISTANCE_TOUCHSLOP		(0x1 << 2) /* 4 */
#define TIMEOUT_INTER_TAP_LONG		(0x1 << 3) /* 8 */
#define MULTI_FINGER			(0x1 << 4) /* 16 */
#define DELAY_TIME			(0x1 << 5) /* 32 */
#define TIMEOUT_INTER_TAP_SHORT		(0x1 << 6) /* 64 */
#define PALM_STATE			(0x1 << 7) /* 128 */
#define TAP_TIMEOVER			(0x1 << 8) /* 256 */
#define TCI_DEBUG_ALL (DISTANCE_INTER_TAP | DISTANCE_TOUCHSLOP |\
	TIMEOUT_INTER_TAP_LONG | MULTI_FINGER | DELAY_TIME |\
	TIMEOUT_INTER_TAP_SHORT | PALM_STATE | TAP_TIMEOVER)

static inline struct ft8707_data *to_ft8707_data(struct device *dev)
{
	return (struct ft8707_data *)touch_get_device(to_touch_core(dev));
}

static inline struct ft8707_data *to_ft8707_data_from_kobj(struct kobject *kobj)
{
	return (struct ft8707_data *)container_of(kobj,
			struct ft8707_data, kobj);
}

int ft8707_reg_read(struct device *dev, u16 addr, void *data, int size);
int ft8707_reg_write(struct device *dev, u16 addr, void *data, int size);
int ft8707_ic_info(struct device *dev);
int ft8707_tc_driving(struct device *dev, int mode);
int ft8707_irq_abs(struct device *dev);
int ft8707_irq_lpwg(struct device *dev, int tci);
int ft8707_irq_handler(struct device *dev);
int ft8707_check_status(struct device *dev);
int ft8707_debug_info(struct device *dev, int mode);
int ft8707_report_tci_fr_buffer(struct device *dev);


static inline int ft8707_read_value(struct device *dev,
					u16 addr, u32 *value)
{
	return ft8707_reg_read(dev, addr, value, sizeof(*value));
}

static inline int ft8707_write_value(struct device *dev,
					 u16 addr, u32 value)
{
	return ft8707_reg_write(dev, addr, &value, sizeof(value));
}

//void ft8707_xfer_msg_ready(struct device *dev, u8 msg_cnt);

#endif /* LGE_TOUCH_FT8707_H */
