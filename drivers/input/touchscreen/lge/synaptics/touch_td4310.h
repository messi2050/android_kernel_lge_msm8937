/* touch_synaptics.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
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

#ifndef LGE_TOUCH_SYNAPTICS_H
#define LGE_TOUCH_SYNAPTICS_H

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/firmware.h>
#include <linux/input/lge_touch_notify.h>
#include <linux/spi/spi.h>
#include <linux/pm_qos.h>

#define MAX_POINT_SIZE_FOR_LPWG	256
#define FW_VER_INFO_NUM			4
#define MAX_NUM_OF_FINGERS		10
#define PAGE_SELECT_REG			0xff
#define PAGES_TO_SERVICE		10
#define LPWG_MAX_BUFFER			10

#define MASK_16BIT	(0xFFFF)
#define MASK_8BIT	(0xFF)
#define MASK_7BIT	(0x7F)
#define MASK_6BIT	(0x3F)
#define MASK_5BIT	(0x1F)
#define MASK_4BIT	(0x0F)
#define MASK_3BIT	(0x07)
#define MASK_2BIT	(0x03)
#define MASK_1BIT	(0x01)

#define PAGE_SELECT_LEN		(2)
#define ADDRESS_WORD_LEN	(2)

#define PLATFORM_DRIVER_NAME "synaptics_dsx"

#define DEFAULT_PAGE			0x00
#define COMMON_PAGE			(d->f01.page)
#define FINGER_PAGE			(d->f12.page)
#define ANALOG_PAGE			(d->f54.page)
#define FLASH_PAGE			(d->f34.page)
#define LPWG_PAGE			(d->f51.page)

/* RMI_DEVICE_CONTROL */
#define DEVICE_CONTROL_REG		(d->f01.dsc.control_base + 0)
#define DEVICE_CONTROL_NORMAL_OP	0x00
#define DEVICE_CONTROL_SLEEP		0x01
#define DEVICE_CONTROL_NOSLEEP		0x04
#define DEVICE_CONTROL_CONFIGURED	0x80

//#define INTERRUPT_ENABLE_REG		(d->f01.dsc.control_base + 1) //Synaptics reccomended to not use
#define DOZE_INTERVAL_REG		(d->f01.dsc.control_base + 2)

#define DEVICE_STATUS_REG		(d->f01.dsc.data_base + 0)
#define INTERRUPT_STATUS_REG		(d->f01.dsc.data_base + 1)
#define INTERRUPT_MASK_FLASH		(1 << 0)
#define INTERRUPT_MASK_STATUS		(1 << 1)
#define INTERRUPT_MASK_ABS0			(1 << 2)
#define INTERRUPT_MASK_BUTTON		(1 << 4)
#define INTERRUPT_MASK_CUSTOM		(1 << 5)
#define INTERRUPT_MASK_LPWG		INTERRUPT_MASK_CUSTOM
/* [bringup] RMI_DEVICE_STATUS */
#define STATUS_CODE_MASK                0x0F
#define NORMAL_MASK_STATUS              0x00
#define RESET_MASK_STATUS               0x01
#define INVALID_CONF_MASK_STATUS        0x02
#define DEVICE_FAILURE_MASK_STATUS      0x03
#define CONF_CRC_FAILURE_MASK_STATUS    0x04
#define FW_CRC_FAILURE_MASK_STATUS      0x05
#define CRC_PROGRESS_MASK_STATUS        0x06
#define GUEST_CRC_FAILURE_MASK_STATUS   0x07
#define EXT_AFE_FAILURE_MASK_STATUS     0x08
#define DISPLAY_FAILURE_MASK_STATUS     0x09
#define FLASH_PROG_MASK_STATUS          (1<<6)
#define DEVICE_UNCONF_MASK_STATUS       (1<<7)

#define FINGER_DATA_REG				(d->f12.dsc.data_base + 0)
#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS			(0x01)
#define F12_STYLUS_STATUS			(0x02)
#define F12_PALM_STATUS				(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)
#define F12_MAX_OBJECT				(0x06)
#define FINGER_REPORT_DATA			(d->f12_reg.ctrl[15])
#define FINGER_REPORT_REG			(d->f12_reg.ctrl[20])

#define LPWG_STATUS_REG				(d->f51.dsc.data_base)
#define LPWG_STATUS_DOUBLETAP		(1 << 0)
#define LPWG_STATUS_PASSWORD		(1 << 1)
#define LPWG_DATA_REG				(d->f51.dsc.data_base + 1)
#define LPWG_OVER_TAPCOUNT			(d->f51.dsc.data_base + 73)

#define LPWG_TAPCOUNT_REG		(d->f51.dsc.control_base)
#define LPWG_MIN_INTERTAP_REG		(d->f51.dsc.control_base + 1)
#define LPWG_MAX_INTERTAP_REG		(d->f51.dsc.control_base + 2)
#define LPWG_TOUCH_SLOP_REG		(d->f51.dsc.control_base + 3)
#define LPWG_TAP_DISTANCE_REG		(d->f51.dsc.control_base + 4)
#define LPWG_INTERRUPT_DELAY_REG	(d->f51.dsc.control_base + 6)
#define LPWG_BLKSIZ				7 	/* 4-page */
//#define LPWG_PARTIAL_REG		(LPWG_INTERRUPT_DELAY_REG2 + 35)

#define LPWG_TAPCOUNT_REG2		(d->f51.dsc.control_base + 7)
#define LPWG_MIN_INTERTAP_REG2		(d->f51.dsc.control_base + 8)
#define LPWG_MAX_INTERTAP_REG2		(d->f51.dsc.control_base + 9)
#define LPWG_TOUCH_SLOP_REG2		(d->f51.dsc.control_base + 10)
#define LPWG_TAP_DISTANCE_REG2		(d->f51.dsc.control_base + 11)
#define LPWG_INTERRUPT_DELAY_REG2	(d->f51.dsc.control_base + 13)

#define LPWG_TCI1_FAIL_COUNT_REG	(d->f51.dsc.data_base + 49)
#define LPWG_TCI1_FAIL_INDEX_REG	(d->f51.dsc.data_base + 50)
#define LPWG_TCI1_FAIL_BUFFER_REG	(d->f51.dsc.data_base + 51)
#define LPWG_TCI2_FAIL_COUNT_REG	(d->f51.dsc.data_base + 61)
#define LPWG_TCI2_FAIL_INDEX_REG	(d->f51.dsc.data_base + 62)
#define LPWG_TCI2_FAIL_BUFFER_REG	(d->f51.dsc.data_base + 63)

/* STYLUS Control Reg */
#define STYLUS_CTRL_REG		(d->f51.dsc.control_base + 21)
#define STYLUS_MODE_ENABLE	0x08

/* Real-Time LPWG Fail Reason Ctrl */
#define NUM_OF_EACH_FINGER_DATA		8
#define MAX_NUM_OF_FAIL_REASON		2
// F51_CUSTOM_DATA31 (Multitap Fail Reason Real-Time Interrupt)
#define LPWG_FAIL_REASON_REALTIME_INT	(d->f51.dsc.data_base + 74)
// F51_CUSTOM_CTRL06.00 (MultiTap Fail Real-Time Interrupt Enable)
#define LPWG_FAIL_INT_ENABLE_REG	(d->f51.dsc.control_base + 15)

#define REPORT_TYPE_REG		(d->f54.dsc.data_base)
#define FIFO_INDEX_LSB_REG		(d->f54.dsc.data_base + 1)
#define FIFO_INDEX_MSB_REG		(d->f54.dsc.data_base + 2)
#define REPORT_DATA_REG		(d->f54.dsc.data_base + 3)
#define INTERFERENCE_METRIC_LSB_REG	(d->f54.dsc.data_base + 4)
#define INTERFERENCE_METRIC_MSB_REG	(d->f54.dsc.data_base + 5)
#define CURRENT_NOISE_STATUS_REG	(d->f54.dsc.data_base + 8)
#define CID_IM_REG			(d->f54.dsc.data_base + 10)
#define FREQ_SCAN_IM_REG		(d->f54.dsc.data_base + 11)

#define DYNAMIC_SENSING_CTRL_REG	(d->f54.dsc.control_base + 33)
// F54_ANALOG_CTRL91(Transcap/Feedback Capacitance) - Electrode Open
#define ANALOG_CONTROL2		(d->f54.dsc.control_base + 36)
// F54_ANALOG_CTRL99 (Integration Duration) - AMP/Electrode Open
#define WAVEFORM_DURATION_CTRL_REG	(d->f54.dsc.control_base + 41)
// F54_ANALOG_CTRL103 (TDDI CBC Enable) - AMP Short
#define TDDI_CBC_ENABLE_CTRL_REG	(d->f54.dsc.control_base + 44)
// F54_ANALOG_CTRL182 (CBC TIMING CONTROL) - Electrode Open
#define CBC_TIMING_CONTROL		(d->f54.dsc.control_base + 46)

#define ANALOG_COMMAND_REG		(d->f54.dsc.command_base)

/* Flash Memory Management */
#define FLASH_CONFIG_ID_REG		(d->f34.dsc.control_base)
/* Manufacturer ID */
#define MANUFACTURER_ID_REG		(d->f01.dsc.query_base)
/* CUSTOMER_FAMILY QUERY */
#define CUSTOMER_FAMILY_REG		(d->f01.dsc.query_base + 2)
/* FW revision */
#define FW_REVISION_REG			(d->f01.dsc.query_base + 3)
/* Product ID */
#define PRODUCT_ID_REG			(d->f01.dsc.query_base + 11)
#define DEVICE_COMMAND_REG		(d->f01.dsc.command_base)

#define F35_ERROR_CODE_OFFSET		0
#define F35_FLASH_STATUS_OFFSET  	5
#define F35_CHUNK_NUM_LSB_OFFSET	0
#define F35_CHUNK_NUM_MSB_OFFSET	1
#define F35_CHUNK_DATA_OFFSET		2
#define F35_CHUNK_COMMAND_OFFSET	18

#define F35_CHUNK_SIZE				16
#define F35_ERASE_ALL_WAIT_MS 		8000 // [bringup] 2000
#define F35_RESET_WAIT_MS			250

#define PDT_START			0x00E9
#define PDT_END				0x00DD

enum{
	CONNECT_NONE = 0,
	CONNECT_USB,
	CONNECT_TA,
	CONNECT_OTG,
};

enum{
	NOISE_DISABLE = 0,
	NOISE_ENABLE,
};

enum {
	ABS_STOP = 0,
	ABS_SENSING,
	LPWG_STOP,
	LPWG_SENSING,
	PARTIAL_LPWG_SENSING,
};

enum {
    FAIL_DISTANCE_INTER_TAP = 1,
    FAIL_DISTANCE_TOUCHSLOP,
    FAIL_TIMEOUT_INTER_TAP,
    FAIL_MULTI_FINGER,
    FAIL_DELAY_TIME,
    FAIL_PALM_STATE,
    FAIL_ACTIVE_AREA,
    FAIL_TAP_COUNT,
};

/* stylus mode */
enum {
	STYLUS_OFF= 0,
	STYLUS_ON,
};

/* Added for IC Initialization state*/
enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	PANEL_RESET_NEED = 0,
	PANEL_RESET_MAX = 3,
};

/* Added for ESD Initialization state*/
enum {
	ESD_RECOVERY_NEED = 0,
	ESD_RECOVERY_DONE,
};
/* Added for IC Configured state */
enum {
	IC_CONFIGURED_NEED = 0,
	IC_CONFIGURED_DONE,
};

enum {
	SW_RESET = 0,
	HW_RESET,
};

enum {
	TCI_REPORT_NOT_SET = -1,
	TCI_REPORT_DISABLE,
	TCI_REPORT_ENABLE,
};

struct td4310_f12_reg {
	u8 *ctrl;
	u8 *data;
};

struct td4310_version {
	u8 build : 4;
	u8 major : 4;
	u8 minor;
};

struct td4310_touch_data {
	u8 type;
	u8 x_lsb;
	u8 x_msb;
	u8 y_lsb;
	u8 y_msb;
	u8 z;
	u8 wx;
	u8 wy;
} __packed;

struct td4310_touch_info {
	u8 device_status;
	u8 irq_status;
	u8 touch_cnt:4;
	struct td4310_touch_data data[10];
} __packed;


struct td4310_ic_info {
	struct td4310_version version;
	u8 raws[4];
	u8 product_id[10];
	struct td4310_version img_version;
	u8 img_raws[4];
	u8 img_product_id[10];
	u8 revision;
	u8 family;
	u32 fw_ver_addr;
	u32 fw_pid_addr;
	u8 bootloader_type;
};

struct td4310_noise_ctrl {
	u8 noise_log;
	u8 check_noise;
	u8 cnt;
	u8 cns_avg;
	u8 im_avg;
	u8 cid_im_avg;
	u8 freq_scan_im_avg;
	unsigned long im_sum;
	unsigned long cns_sum;
	unsigned long cid_im_sum;
	unsigned long freq_scan_im_sum;
};

struct function_descriptor {
	u8 query_base;
	u8 command_base;
	u8 control_base;
	u8 data_base;
	u8 int_source_count;
	u8 fn_number;
};

struct td4310_function {
	struct function_descriptor dsc;
	u8 page;
};

struct td4310_rmidev_exp_fn {
	int (*init)(struct device *dev);
	void (*remove)(struct device *dev);
	void (*reset)(struct device *dev);
	void (*reinit)(struct device *dev);
	void (*early_suspend)(struct device *dev);
	void (*suspend)(struct device *dev);
	void (*resume)(struct device *dev);
	void (*late_resume)(struct device *dev);
	void (*attn)(struct device *dev,
			unsigned char intr_mask);
};

struct td4310_rmidev_exp_fhandler {
	struct td4310_rmidev_exp_fn *exp_fn;
	bool insert;
	bool initialized;
	bool remove;
};

struct td4310_state_info {
	atomic_t charger;
	atomic_t init;
	atomic_t config;
	atomic_t scan_pdt;
	atomic_t fw_recovery;
	atomic_t esd_recovery;
	atomic_t stylus_mode;
	atomic_t panel_reset_flag;
};

struct td4310_data {
	struct td4310_function f01;
	struct td4310_function f12;
	struct td4310_function f34;
	struct td4310_function f51;
	struct td4310_function f54;
	struct td4310_function f55;
	struct td4310_function fdc;
	struct td4310_f12_reg f12_reg;
	struct td4310_touch_info info;
	struct td4310_ic_info ic_info;
	struct td4310_noise_ctrl noise;
	u8 uBL_addr;/* Micro BL mode slave address */
	u8 curr_page;
	u8 object_report;
	u8 max_num_of_fingers;
	u8 lpwg_fail_reason;
	struct delayed_work fb_notify_work;
	struct pm_qos_request pm_qos_req;
	struct td4310_state_info state;
};

static inline struct td4310_data *to_td4310_data(struct device *dev)
{
	return (struct td4310_data *)touch_get_device(to_touch_core(dev));
}

bool td4310_is_product(struct td4310_data *d,
					const char *product_id, size_t len);
bool td4310_is_img_product(struct td4310_data *d,
					const char *product_id, size_t len);
int td4310_set_page(struct device *dev, u8 page);
int td4310_init(struct device *dev);
int td4310_ic_info(struct device * dev);
int td4310_read(struct device *dev, u8 addr, void *data, int size);
int td4310_write(struct device *dev, u8 addr, void *data, int size);
int td4310_force_update(struct device *dev);
void td4310_reset_ctrl(struct device *dev, int ctrl);
void td4310_rmidev_function(struct td4310_rmidev_exp_fn *exp_fn, bool insert);
irqreturn_t touch_irq_handler(int irq, void *dev_id);
irqreturn_t touch_irq_thread(int irq, void *dev_id);

/* extern function */
extern int FirmwareUpgrade(struct device *dev, const struct firmware *fw);
extern int FirmwareRecovery(struct device *dev, const struct firmware *fw);

#endif /* LGE_TOUCH_SYNAPTICS_H */
