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

/* Start - kiwoo.han 2016-8-20 */
//#define DUALIZATION_DISPLAY_MODULE
/* End - kiwoo.han 2016-8-20 */

#define MAX_POINT_SIZE_FOR_LPWG		256
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

#define PAGE_SELECT_LEN (2)
#define ADDRESS_WORD_LEN (2)

#define PLATFORM_DRIVER_NAME "synaptics_dsx"

#define DEFAULT_PAGE			0x00
#define COMMON_PAGE			(d->f01.page)
#define FINGER_PAGE			(d->f12.page)
#define ANALOG_PAGE			(d->f54.page)
#define FLASH_PAGE			(d->f34.page)
#define LPWG_PAGE			(d->f51.page)

#define DEVICE_CONTROL_REG		(d->f01.dsc.control_base + 0)
#define DEVICE_CONTROL_NORMAL_OP	0x00
#define DEVICE_CONTROL_SLEEP		0x01
#define DEVICE_CONTROL_NOSLEEP	0x04
#define DEVICE_CONTROL_CONFIGURED	0x80

#define INTERRUPT_ENABLE_REG		(d->f01.dsc.control_base + 1)
#define DOZE_INTERVAL_REG		(d->f01.dsc.control_base + 2)

#define DEVICE_STATUS_REG		(d->f01.dsc.data_base + 0)
#define INTERRUPT_STATUS_REG		(d->f01.dsc.data_base + 1)
#define DOZE_DATA_REG			(d->f01.dsc.data_base + 3)
#define INTERRUPT_MASK_FLASH		(1 << 0)
#define INTERRUPT_MASK_STATUS		(1 << 1)
#define INTERRUPT_MASK_ABS0		(1 << 2)
#define INTERRUPT_MASK_BUTTON		(1 << 4)
#define INTERRUPT_MASK_CUSTOM		(1 << 5)
#define INTERRUPT_MASK_LPWG		INTERRUPT_MASK_CUSTOM

#define FINGER_DATA_REG			(d->f12.dsc.data_base + 0)
#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS		(0x01)
#define F12_STYLUS_STATUS		(0x02)
#define F12_PALM_STATUS			(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)
#define F12_MAX_OBJECT			(0x06)

#define LPWG_STATUS_REG			(d->f51.dsc.data_base)
#define LPWG_STATUS_DOUBLETAP		(1 << 0)
#define LPWG_STATUS_PASSWORD		(1 << 1)
#define LPWG_DATA_REG			(d->f51.dsc.data_base + 1)
#define LPWG_OVER_TAPCOUNT		(d->f51.dsc.data_base + 73)

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

/* Real-Time LPWG Fail Reason Ctrl */
#define NUM_OF_EACH_FINGER_DATA		8
#define MAX_NUM_OF_FAIL_REASON		2
#define LPWG_FAIL_REASON_REALTIME_INT	(d->f51.dsc.data_base + 74) // F51_CUSTOM_DATA31 (Multitap Fail Reason Real-Time Interrupt)
#define LPWG_FAIL_INT_ENABLE_REG		(d->f51.dsc.control_base + 15) // F51_CUSTOM_CTRL06.00 (MultiTap Fail Real-Time Interrupt Enable)

#define REPORT_TYPE_REG				(d->f54.dsc.data_base)
#define FIFO_INDEX_LSB_REG			(d->f54.dsc.data_base + 1)
#define FIFO_INDEX_MSB_REG			(d->f54.dsc.data_base + 2)
#define REPORT_DATA_REG				(d->f54.dsc.data_base + 3)
#define INTERFERENCE_METRIC_LSB_REG	(d->f54.dsc.data_base + 4)
#define INTERFERENCE_METRIC_MSB_REG	(d->f54.dsc.data_base + 5)
#define CURRENT_NOISE_STATUS_REG		(d->f54.dsc.data_base + 8)
#define CID_IM_REG					(d->f54.dsc.data_base + 10)
#define FREQ_SCAN_IM_REG				(d->f54.dsc.data_base + 11)

#define DYNAMIC_SENSING_CTRL_REG		(d->f54.dsc.control_base + 33)
#define WAVEFORM_DURATION_CTRL_REG		(d->f54.dsc.control_base + 41) // F54_ANALOG_CTRL99 (Integration Duration) - AMP Open
#define TDDI_CBC_ENABLE_CTRL_REG		(d->f54.dsc.control_base + 44) // F54_ANALOG_CTRL103 (TDDI CBC Enable) - AMP Short
#define ABS_DOZE_CTRL_REG			(d->f54.dsc.control_base + 87)

#define ANALOG_COMMAND_REG		(d->f54.dsc.command_base)

/* ESD Detect Control Register */
#define ESD_GENERAL_CTRL_REG		(d->f51.dsc.control_base + 21)

#define FINGER_REPORT_REG		(d->f12_reg.ctrl[20])

/* Flash Memory Management */
#define FLASH_CONFIG_ID_REG		(d->f34.dsc.control_base)
#define FLASH_STATUS_REG			(d->f34.dsc.data_base)
#define PARTITION_ID_REG			(d->f34.dsc.data_base + 1)
#define BLOCK_OFFSET_REG			(d->f34.dsc.data_base + 2)
#define TRANSFER_LENGTH_REG		(d->f34.dsc.data_base + 3)
#define PROGRAMING_CMD_REG		(d->f34.dsc.data_base + 4)
#define PAYLOAD_REG				(d->f34.dsc.data_base + 5)
#define FLASH_PROPERTY_REG		(d->f34.dsc.query_base + 3)

/* RMI_DEVICE_CONTROL */
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
#define F35_CHUNK_NUM_LSB_OFFSET	0
#define F35_CHUNK_NUM_MSB_OFFSET	1
#define F35_CHUNK_DATA_OFFSET		2
#define F35_CHUNK_COMMAND_OFFSET	18

#define F35_CHUNK_SIZE		16
#define F35_ERASE_ALL_WAIT_MS 2000
#define F35_RESET_WAIT_MS	250

#define PDT_START		0x00e9
//#define PDT_END		0x00D0
#define PDT_END		0x00C0

#ifdef CONFIG_LGE_TOUCH_TOVIS_SYNAPTICS_TD4100
#define DRUMMING_ACCEL_TH		0x0b
#define IME_DRUMMING_ACCEL_TH		0x06
#else /*SF3*/
#define DRUMMING_ACCEL_TH			0x0a
#define IME_DRUMMING_ACCEL_TH			0x04
#endif
#define SMALL_FINGER_AMPLITUDE_TH		0x0d
#define IME_SMALL_FINGER_AMPLITUDE_TH	0x1a
#define FINGER_LANDING_MAX_FRAME_LPWG	0x01

enum{
	NOISE_DISABLE = 0,
	NOISE_ENABLE,
};

enum {
	SW_RESET = 0,
	HW_RESET,
};

enum {
	ABS_STOP = 0,
	ABS_SENSING,
	WAKEUP_STOP,
	WAKEUP_SENSING,
	PARTIAL_WAKEUP_SENSING,
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

enum {
	ESD_RECOVERY_END = 0,
	ESD_RECOVERY_RUN,
};

enum {
	INIT_END = 0,
	INIT_RUN,
};

struct synaptics_state_info {
	atomic_t recovery; /* check esd recovery operation */
	atomic_t init; /* check synaptics init status */
};

struct synaptics_f12_reg {
	u8 ctrl[32];
	u8 data[16];
};

struct synaptics_f51_reg {
	u8 lpwg_status_reg;
	u8 lpwg_data_reg;
	u8 lpwg_tapcount_reg;
	u8 lpwg_min_intertap_reg;
	u8 lpwg_max_intertap_reg;
	u8 lpwg_touch_slop_reg;
	u8 lpwg_tap_distance_reg;
	u8 lpwg_interrupt_delay_reg;
	u8 lpwg_tapcount_reg2;
	u8 lpwg_min_intertap_reg2;
	u8 lpwg_max_intertap_reg2;
	u8 lpwg_touch_slop_reg2;
	u8 lpwg_tap_distance_reg2;
	u8 lpwg_interrupt_delay_reg2;
	u8 overtap_cnt_reg;
	u8 request_reset_reg;
	u8 lpwg_partial_reg;
	u8 lpwg_fail_count_reg;
	u8 lpwg_fail_index_reg;
	u8 lpwg_fail_reason_reg;
	u8 lpwg_adc_offset_reg;
	u8 lpwg_adc_fF_reg1;
	u8 lpwg_adc_fF_reg2;
	u8 lpwg_adc_fF_reg3;
	u8 lpwg_adc_fF_reg4;
};

struct synaptics_f54_reg {
	u8 interference__metric_LSB;
	u8 interference__metric_MSB;
	u8 current_noise_status;
	u8 cid_im;
	u8 freq_scan_im;
	u8 incell_statistic;
};

struct synaptics_version {
	u8 build : 4;
	u8 major : 4;
	u8 minor;
};

struct synaptics_ic_info {
	struct synaptics_version version;
	u8 raws[4];
	u8 product_id[10];
	struct synaptics_version img_version;
	u8 img_raws[4];
	u8 img_product_id[10];
	u8 revision;
	u8 family;
	u32 fw_ver_addr;
	u32 fw_pid_addr;
};

struct synaptics_noise_ctrl {
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

struct synaptics_function {
	struct function_descriptor dsc;
	u8 page;
};

struct synaptics_object {
	u8 type;
	u8 x_lsb;
	u8 x_msb;
	u8 y_lsb;
	u8 y_msb;
	u8 z;
	u8 wx;
	u8 wy;
};

struct synaptics_rmidev_exp_fn {
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

struct synaptics_rmidev_exp_fhandler {
	struct synaptics_rmidev_exp_fn *exp_fn;
	bool insert;
	bool initialized;
	bool remove;
};

struct synaptics_data {
	struct synaptics_function f01;
	struct synaptics_function f11;
	struct synaptics_function f12;
	struct synaptics_function f1a;
	struct synaptics_function f34;
	struct synaptics_function f51;
	struct synaptics_function f54;
	struct synaptics_function f55;
	struct synaptics_f12_reg f12_reg;
	struct synaptics_f51_reg f51_reg;
	struct synaptics_f54_reg f54_reg;
	struct synaptics_ic_info ic_info;
	struct synaptics_noise_ctrl noise;
	struct synaptics_state_info state;

	u8 curr_page;
	u8 object_report;
	u8 num_of_fingers;
	u8 irq_mask;
	u8 power_state;
	bool need_scan_pdt;
	u8 prev_lcd_mode;
	u8 lpwg_fail_reason;
	struct pm_qos_request pm_qos_req;
	struct delayed_work fb_notify_work;
	bool need_fw_recovery;
	bool use_palm_filter;
	u16 palm_mask;
	struct point palm_point[MAX_NUM_OF_FINGERS];
	struct touch_data old_tdata[MAX_NUM_OF_FINGERS];
	struct delayed_work palm_filter_work;
};

static inline struct synaptics_data *to_synaptics_data(struct device *dev)
{
	return (struct synaptics_data *)touch_get_device(to_touch_core(dev));
}

bool synaptics_is_product(struct synaptics_data *d,
					const char *product_id, size_t len);
bool synaptics_is_img_product(struct synaptics_data *d,
					const char *product_id, size_t len);
int synaptics_set_page(struct device *dev, u8 page);
int synaptics_init(struct device *dev);
int synaptics_ic_info(struct device * dev);
int synaptics_read(struct device *dev, u8 addr, void *data, int size);
int synaptics_write(struct device *dev, u8 addr, void *data, int size);
int TD4100_ForceUpdate(struct device *dev);
int synaptics_esd_detect_enable(struct device *dev, int value);
void synaptics_reset_ctrl(struct device *dev, int ctrl);
void synaptics_rmidev_function(struct synaptics_rmidev_exp_fn *exp_fn, bool insert);
irqreturn_t touch_irq_handler(int irq, void *dev_id);
irqreturn_t touch_irq_thread(int irq, void *dev_id);

/* extern function */
extern int FirmwareUpgrade(struct device *dev, const struct firmware *fw);
extern int FirmwareRecovery(struct device *dev, const struct firmware *fw);
#if defined(CONFIG_LGE_DISPLAY_SF3_LGD_TD4100) /*only use SF3*/
extern int ext_dsv_mode_change(int mode);
#endif

/* check recovery mode */
extern int check_recovery_boot;

#if defined (CONFIG_LGE_TOUCH_TOVIS_PANEL)
extern bool check_panel_type;
#endif

/* for quick cover */
#if defined(CONFIG_LGE_TOUCH_HALL_IC_COVER)
extern int cradle_smart_cover_status(void);
#endif

#endif /* LGE_TOUCH_SYNAPTICS_H */
