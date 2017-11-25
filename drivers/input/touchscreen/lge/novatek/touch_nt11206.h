/* touch_nt11206.h
 *
 * Copyright (C) 2015 LGE.
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

#ifndef  LGE_DEVICE_NT11206_H
#define LGE_TOUCH_NT11206_H

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#define NVT_TOUCH_CTRL_DRIVER 		1
#define NVT_TOUCH_EXT_PROC 			1

#define IC_ID 						0x26

#define SECTOR_NUM 					32
#define SECTOR_SIZE 				4096
#define NT_PAGE_NUM 				512
#define NT_PAGE_SIZE 				256

#define MAX_FINGER_NUM          	10
#define FINGER_EVENT_LEN        	6
#define FINGER_STATUS_MASK      	0x07
#define BOOTLOADER_SLAVE_ADDR 		0x62
#define SLAVE_ADDR 					0x01
#define READ_COOR_ADDR 				0x00
#define MODE_CHANGE_OFFSET 			0x50
#define MODE_CHANGE_NORMAL_OFFSET 	0x51
#define MODE_CHANGE_CEHCK 			0x70

#define ADDR_CMD 					0xFF
#define UPLOAD_SRAM_FLASH_CMD 		0x02
#define ADDR_L_CHECKSUM 	 		0x00
#define CHIP_ID_OFFSET 				0x00
#define DEEP_SLEEP_OFFSET 			0x50
#define FW_INFO_OFFSET_1 			0x00
#define FW_INFO_OFFSET_2 			0x16
#define I2C_DRV_ABL_OFFSET 			0x14
#define I2C_DEBOUNCE_OFFSET 		0x15
#define SHM_TRG_OFFSET 				0x3E

#define FW_IMG_INFO_START 			0xE000

#define NT_MIN(x, y)  (((x) < (y)) ? (x) : (y))

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"
#endif

#define I2C_TANSFER_LENGTH  64

#define MODE_CHANGE_NORMAL_MODE 	0x00
#define TEST_MODE_1 				0x21    //before algo.
#define TEST_MODE_2 				0x22    //after algo.
#define AUTORC_OFF 					0x24    // AutoRC Off

#define RAW_PIPE0_ADDR  0x10528
#define RAW_PIPE1_ADDR  0x13528
#define BASELINE_ADDR   0x11054
#define DIFF_PIPE0_ADDR 0x10A50
#define DIFF_PIPE1_ADDR 0x13A50
#define XDATA_SECTOR_SIZE   256

#define LOG_BUF_SIZE (PAGE_SIZE * 2)

#define SELF_TEST_FILE_PATH "/data/touch/touch_self_test.txt"
#define NORMAL_SELF_TEST_FILE_PATH "/sdcard/touch_self_test.txt"
#define RAWDATA_FILE_PATH "/data/touch/touch_rawdata.txt"
#define DELTA_FILE_PATH "/data/touch/touch_delta.txt"
#define BASELINE_FILE_PATH "/data/touch/touch_baseline.txt"
#define MAX_LOG_FILE_SIZE 	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT 	4

enum {
	 POWER_OFF_MODE = 0,
	 DEEP_SLEEP_MODE,
	 GESTURE_MODE,
     NORMAL_MODE,
	 FW_UPGRADE_MODE,
};

enum {
	 FINGER_NONE = 0,
     FINGER_DOWN,
     FINGER_MOVE,
     FINGER_UP,
};

struct nt11206_touch_data {
	u8 tool_type:4;
	u8 event:4;
	s8 track_id;
	u16 x;
	u16 y;
	u8 pressure;
	u8 angle;
	u16 width_major;
	u16 width_minor;
} __packed;

struct nt11206_touch_info {
	u32 device_status;
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	struct nt11206_touch_data data[10];
} __packed;

struct nt11206_fw_info {
	u8 fw_ver;
	u8 fw_ver_bar;
	u8 x_axis_num;
	u8 y_axis_num;
	u16 resolution_x;
	u16 resolution_y;
	u8 common_fw_ver;
	u8 max_touch_num;
	u8 b_num;
	u8 custom_id;
	u8 sensor_id;
	u8 mutual_touch_threshold;
};

struct swipe_info {
	u8	distance;
	u8	ratio_thres;
	u8	ratio_distance;
	u8	ratio_period;
	u16	min_time;
	u16	max_time;
	struct active_area area;
};

struct tp_event {
     u16 x;
     u16 y;
     u16 id;
     u16 pressure;
	 u8 area;
     u8 status;
};

struct nt11206_data {
	struct device *dev;
	struct kobject kobj;
	struct nt11206_touch_info info;
	struct nt11206_fw_info fw;
	struct delayed_work font_download_work;
	struct wake_lock lpwg_wake_lock;
	u8 fingers[MAX_FINGER_NUM];
	u32 charger;
	int boot_mode;
	u8 tci_debug_type;
	u8 swipe_debug_type;
	u8 mode_state;
	u8 resume_state;
	u8 tci_debug;
};

#if NVT_TOUCH_CTRL_DRIVER
struct nvt_flash_data{
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	char *str;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_MCU_NO_BOOT = 0,
	RESET_MCU_TO_IDLE,
	RESET_MCU_BOOT,
} SW_RESET_MODE;

static inline struct nt11206_data *to_nt11206_data(struct device *dev)
{
	return (struct nt11206_data *)touch_get_device(to_touch_core(dev));
}

static inline struct nt11206_data *to_nt11206_data_from_kobj(struct kobject *kobj)
{
	return (struct nt11206_data *)container_of(kobj,
			struct nt11206_data, kobj);
}
int nt11206_reg_read(struct device *dev, u16 addr, void *data, int size);
int nt11206_reg_write(struct device *dev, u16 addr, void *data, int size);
void nvt_set_i2c_debounce(struct device *dev);
int nt11206_bootloader_write(struct device *dev, u8 *data, u8 len);
int nt11206_bootloader_read(struct device *dev, u8 *buf, u8 len);
void nvt_change_mode(struct device *dev, u8 mode);
void nvt_get_fw_info(struct device *dev);
u8 nvt_get_fw_pipe(struct device *dev);
void nvt_read_mdata(struct device *dev, u32 xdata_addr);
void nvt_get_mdata(__s32 *buf, u8 *m_x_num, u8 *m_y_num);
int nt11206_touch_register_sysfs(struct device *dev);
u8 nvt_check_fw_status(struct device *dev);
u8 nvt_clear_fw_status(struct device *dev);
void nt11206_hw_reset(struct device *dev);
int nt11206_selftest(struct device *dev, char* buf, u8 mode);
void nt11206_interrupt_control(struct device *dev, int on_off);
void log_file_size_check(struct device *dev, char *fname);
void write_file(char *filename, char *data, int time);
void large_mdelay(unsigned int msec);
int nt11206_check_baseline(struct device *dev);
int nt11206_power(struct device *dev, int ctrl);
int nt11206_sw_reset(struct device *dev, SW_RESET_MODE mode);
#endif
