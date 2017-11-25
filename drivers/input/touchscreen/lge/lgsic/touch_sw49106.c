/* touch_sw49106.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: BSP-TOUCH@lge.com
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



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw49106.h"
#include "touch_sw49106_abt.h"
#include "touch_sw49106_prd.h"

// Definitions for Debugging Failure Reason in LPWG
enum {
	TCI_DEBUG_DISABLE = 0,
	TCI_DEBUG_ENABLE,
};

static const char *debug_type[] = {
	"TCI Debug Disable",
	"TCI Debug Enable",
};
#define TCI_FAIL_NUM 10
static const char const *tci_debug_str[TCI_FAIL_NUM] = {
	"SUCCESS",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"MINTIMEOUT_INTER_TAP",
	"MAXTIMEOUT_INTER_TAP",
	"LONGPRESS_TIME_OUT",
	"MULTI_FINGER",
	"DELAY_TIME",/* It means Over Tap */
	"PALM_STATE",
	"OUTOF_AREA",
};

#define IC_DEBUG_INFO_NUM 15
static const char *ic_debug_info_str[] = {
	[0] = "NONE",
	[1] = "DBG_TG_FAULT",
	[2] = "DBG_ESD_FAULT",
	[3] = "DBG_WATDOG_TIMEOUT",
	[4] = "DBG_TC_DRV_MISMATCH",
	[5] = "DBG_TC_INVALID_TIME_DRV_REQ",
	[6] = "DBG_AFE_TUNE_FAIL",
	[7] = "DBG_DBG_MSG_FULL",
	[8] = "DBG_PRE_MA_OVF_ERR",
	[9] = "DBG_ADC_OVF_ERR",
	[10] = "DBG_CM3_FAULT",
	[11] = "DBG_DBG_TC_DRV_START_SKIP",
	[12] = "DBG_DBG_TEST_MSG1",
	[13] = "DBG_DBG_TEST_MSG2",
	[14] = "DBG_DBG_TEST_MSG3",
};
#ifdef SW49106_TCL_OFF_VIA_MIPI
//extern int set_touch_osc(int enable);
extern void lge_panel_enter_deep_sleep(void);
extern void lge_panel_exit_deep_sleep(void);
#endif

static int sw49106_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry);

#ifdef SW49106_ESD_SKIP_WHILE_TOUCH_ON
static int finger_cnt = 0;

bool sw49106_check_finger(void)
{
	return finger_cnt==0? false:true;
}
EXPORT_SYMBOL(sw49106_check_finger);
#endif


int sw49106_xfer_msg(struct device *dev, struct touch_xfer_msg *xfer)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	int ret = 0;
	int i = 0;

	mutex_lock(&d->spi_lock);

	for (i = 0; i < xfer->msg_count; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		if (rx->size) {
			tx->data[0] = ((rx->size > 4) ? 0x20 : 0x00);
			tx->data[0] |= ((rx->addr >> 8) & 0x0f);
			tx->data[1] = (rx->addr & 0xff);
			tx->data[2] = 0;
			tx->data[3] = 0;
			rx->size += R_HEADER_SIZE;
		} else {
			if (tx->size > (MAX_XFER_BUF_SIZE - W_HEADER_SIZE)) {
				TOUCH_E("buffer overflow\n");
				mutex_unlock(&d->spi_lock);
				return -EOVERFLOW;
			}

			tx->data[0] = ((tx->size == 1) ? 0x60 : 0x40);
			tx->data[0] |= ((tx->addr >> 8) & 0x0f);
			tx->data[1] = (tx->addr  & 0xff);
			memcpy(&tx->data[W_HEADER_SIZE], tx->buf, tx->size);
			tx->size += W_HEADER_SIZE;
		}
	}

	ret = touch_bus_xfer(dev, xfer);
	if (ret) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->spi_lock);
		return ret;
	}

	for (i = 0; i < xfer->msg_count; i++) {
		rx = &xfer->data[i].rx;

		if (rx->size) {
			memcpy(rx->buf, rx->data + R_HEADER_SIZE,
				(rx->size - R_HEADER_SIZE));
		}
	}

	mutex_unlock(&d->spi_lock);

	return 0;
}

int sw49106_reg_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

// [bringup] start
/*
 * I2C Burst Read workaround
 * 0x200 : Single read - 0x200 (Real-time HW register)
 *         Burst read  - 0x201 (Mirroring resgiser for burst read in IRQ)
 * 0x201 : Change to 0x202 for single read
 *
 * Mirroring : 0x200 ~ 0x240 => 0x201 ~ 0x241
 *
 */
/* Not used in SW49106 */ 
//	if (addr == tc_ic_status) {			//0x200 to 0x201 for burst read
//		addr += !!(size > 4);
//	} else {
//		addr += !!(addr == tc_status);	//0x201 to 0x202
//	}
// [bringup] end

	mutex_lock(&d->spi_lock);
	ts->tx_buf[0] = ((size > 4) ? 0x20 : 0x00);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr & 0xff);
	//ts->tx_buf[2] = 0;
	//ts->tx_buf[3] = 0;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_HEADER_SIZE;
	msg.rx_buf = ts->rx_buf;
	msg.rx_size = R_HEADER_SIZE + size;
	msg.bits_per_word = 8;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->spi_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[R_HEADER_SIZE], size);
	mutex_unlock(&d->spi_lock);
	return 0;
}

int sw49106_reg_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->spi_lock);
	ts->tx_buf[0] = ((size > 4) ? 0x60 : 0x40);
	ts->tx_buf[0] |= ((addr >> 8) & 0x0f);
	ts->tx_buf[1] = (addr  & 0xff);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_HEADER_SIZE + size;
	msg.rx_buf = NULL;
	msg.rx_size = 0;
	msg.bits_per_word = 8;

	memcpy(&ts->tx_buf[W_HEADER_SIZE], data, size);

	ret = touch_bus_write(dev, &msg);
	mutex_unlock(&d->spi_lock);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		return ret;
	}

	return 0;
}


static int sw49106_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;

		if (*blank == FB_BLANK_UNBLANK)
			TOUCH_I("FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			TOUCH_I("FB_BLANK\n");
	}

	return 0;
}
static int sw49106_sw_reset(struct device *dev)
{
       struct touch_core_data *ts = to_touch_core(dev);
       struct sw49106_data *d = to_sw49106_data(dev);
       int ret = 0;
       /******************************************************
       * Siliconworks does not recommend to use SW reset    *
       * due to its limitation in stability in lg4894.      *
       ******************************************************/
       TOUCH_I("%s : SW Reset\n", __func__);
       sw49106_tc_driving(dev, LCD_MODE_STOP);
       sw49106_write_value(dev, sys_rst_ctl, 7);
       touch_msleep(20);
       sw49106_write_value(dev, sys_rst_ctl, 0);
       /* Boot Start */
       //sw49106_write_value(dev, sys_boot_ctl, 1);
       /* firmware boot done check */
       //ret = sw49106_condition_wait(dev, tc_flash_dn_sts, NULL,
       //                                  FLASH_BOOTCHK_VALUE, 0xFFFFFFFF, 10, 200);
       //if (ret < 0) {
       //         TOUCH_E("failed : \'boot check\'\n");
       //         return -EPERM;
       //}
       atomic_set(&d->init, IC_INIT_NEED);
       queue_delayed_work(ts->wq, &ts->init_work,
                                      msecs_to_jiffies(ts->caps.sw_reset_delay));

       return ret;
}

int sw49106_hw_reset(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);

	TOUCH_I("%s : HW Reset(%d)\n", __func__, mode);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_msleep(1);

	touch_gpio_direction_output(ts->reset_pin, 1);
	atomic_set(&d->init, IC_INIT_NEED);

	if (mode == HW_RESET_ASYNC){
		queue_delayed_work(ts->wq, &ts->init_work,
				msecs_to_jiffies(ts->caps.hw_reset_delay));
	} else if(mode == HW_RESET_SYNC) {
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	} else {
		TOUCH_E("%s Invalid HW reset mode!!\n", __func__);
	}

	return 0;
}

int sw49106_reset_ctrl(struct device *dev, int ctrl)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct sw49106_data *d = to_sw49106_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	default :
	case SW_RESET:
		sw49106_sw_reset(dev);
		break;

	case HW_RESET_ASYNC:
	case HW_RESET_SYNC:
		sw49106_hw_reset(dev, ctrl);
		break;
	}

	//atomic_set(&d->watch.state.rtc_status, RTC_CLEAR);

	return 0;
}

static int sw49106_power(struct device *dev, int ctrl)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct sw49106_data *d = to_sw49106_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		//touch_gpio_direction_output(ts->reset_pin, 0);
		//touch_power_vio(dev, 0);
		//touch_power_vdd(dev, 0);
		//touch_msleep(1);
		//atomic_set(&d->watch.state.font_status, FONT_EMPTY);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		//touch_power_vdd(dev, 1);
		//touch_power_vio(dev, 1);
		//touch_gpio_direction_output(ts->reset_pin, 1);
		break;


	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	case POWER_HW_RESET:
		TOUCH_I("%s, reset\n", __func__);
		sw49106_reset_ctrl(dev, HW_RESET_ASYNC);
		break;
	}

	return 0;
}

static void sw49106_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 6;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 6;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

static void sw49106_get_swipe_info(struct device *dev)
{
	struct sw49106_data *d = to_sw49106_data(dev);

	d->swipe.info[SWIPE_L].distance = 5;
	d->swipe.info[SWIPE_L].ratio_thres = 100;
	d->swipe.info[SWIPE_L].ratio_distance = 2;
	d->swipe.info[SWIPE_L].ratio_period = 5;
	d->swipe.info[SWIPE_L].min_time = 0;
	d->swipe.info[SWIPE_L].max_time = 150;
	d->swipe.info[SWIPE_L].area.x1 = 401;	/* 0 */
	d->swipe.info[SWIPE_L].area.y1 = 0;	/* 2060 */
	d->swipe.info[SWIPE_L].area.x2 = 1439;
	d->swipe.info[SWIPE_L].area.y2 = 159;	/* 2559 */

	d->swipe.info[SWIPE_R].distance = 5;
	d->swipe.info[SWIPE_R].ratio_thres = 100;
	d->swipe.info[SWIPE_R].ratio_distance = 2;
	d->swipe.info[SWIPE_R].ratio_period = 5;
	d->swipe.info[SWIPE_R].min_time = 0;
	d->swipe.info[SWIPE_R].max_time = 150;
	d->swipe.info[SWIPE_R].area.x1 = 401;
	d->swipe.info[SWIPE_R].area.y1 = 0;
	d->swipe.info[SWIPE_R].area.x2 = 1439;
	d->swipe.info[SWIPE_R].area.y2 = 159;

	d->swipe.mode = SWIPE_LEFT_BIT | SWIPE_RIGHT_BIT;
}

int sw49106_ic_info(struct device *dev)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	u32 version = 0;
	u32 revision = 0;
	u32 bootmode = 0;
	u32 lcdrevision = 0;
	u32 product[2] = {0};
	char rev_str[32] = {0};

	ret = sw49106_reg_read(dev, tc_version, &version, sizeof(version));
	if (ret < 0) {
		TOUCH_D(BASE_INFO, "version : %x\n", version);
		return ret;
	}

	ret = sw49106_reg_read(dev, info_chip_revision, &revision, sizeof(revision));
	ret = sw49106_reg_read(dev, tc_product_id1, &product[0], sizeof(product));
	ret = sw49106_reg_read(dev, spr_boot_st, &bootmode, sizeof(bootmode));
	ret = sw49106_reg_read(dev, info_lcd_revision, &lcdrevision, sizeof(lcdrevision));

	d->fw.version[0] = ((version >> 8) & 0xFF);
	d->fw.version[1] = version & 0xFF;
	d->fw.revision = revision & 0xFF;
	d->fw.lcd_fpcb_revision = lcdrevision;
	memcpy(&d->fw.product_id[0], &product[0], sizeof(product));

	if (d->fw.revision == 0xFF)
		snprintf(rev_str, 32, "revision: Flash Erased(0xFF)");
	else
		snprintf(rev_str, 32, "revision: %d", d->fw.revision);

	TOUCH_D(BASE_INFO, "version : v%d.%02d, chip : %d, protocol : %d\n" \
		"[Touch] %s\n" \
		"[Touch] product id : %s\n" \
		"[Touch] flash boot : %s, %s, crc : %s\n",
		d->fw.version[0], d->fw.version[1],
		(version >> 16) & 0xFF, (version >> 24) & 0xFF, rev_str,  d->fw.product_id,
		(bootmode >> 0 & 0x1) ? "BUSY" : "idle",
		(bootmode >> 1 & 0x1) ? "done" : "booting",
		(bootmode >> 2 & 0x1) ? "ERROR" : "ok");

	TOUCH_D(BASE_INFO, "lcd fpcb revision : %d\n", d->fw.lcd_fpcb_revision);
	if (((((version >> 16) & 0xFF) != VCHIP_VAL) || (((version >> 24) & 0x0F) != VPROTO_VAL))
		&& (atomic_read(&ts->state.core) != CORE_PROBE)) {
		TOUCH_I("FW is in abnormal state because of ESD or something.\n");
		sw49106_reset_ctrl(dev, HW_RESET_ASYNC);
		//sw49106_power(dev, POWER_OFF);
		//sw49106_power(dev, POWER_ON);
		//touch_msleep(ts->caps.hw_reset_delay);
	}

	return ret;
}
static void sw49106_setup_q_sensitivity(struct device *dev, int enable)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);

	d->q_sensitivity = enable; // 1=enable touch, 0=disable touch

	#ifdef SW49106_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP)
	#endif
	sw49106_reg_write(dev, Q_TOUCH_SENSE, &d->q_sensitivity, sizeof(u32));

	TOUCH_I("%s : %s(%d)\n", __func__,
			(d->q_sensitivity) ? "SENSITIVE" : "NORMAL", (d->q_sensitivity));
}

static int sw49106_get_tci_data(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	u8 i = 0;
	u32 rdata[MAX_LPWG_CODE];

	if (!count)
		return 0;

	ts->lpwg.code_num = count;

	memcpy(&rdata, d->info.data, sizeof(u32) * count);

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = rdata[i] & 0xffff;
		ts->lpwg.code[i].y = (rdata[i] >> 16) & 0xffff;

		if (ts->lpwg.mode >= LPWG_PASSWORD)
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int sw49106_get_swipe_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	u32 rdata[3];
	int count = 1;

	/* swipe_info */
	/* start (X, Y), end (X, Y), time = 2bytes * 5 = 10 bytes */
	memcpy(&rdata, d->info.data, sizeof(u32) * 3);

	TOUCH_I("Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
			rdata[0] & 0xffff, rdata[0] >> 16,
			rdata[1] & 0xffff, rdata[1] >> 16,
			rdata[2] & 0xffff);

	ts->lpwg.code_num = count;
	ts->lpwg.code[0].x = rdata[1] & 0xffff;
	ts->lpwg.code[0].x = rdata[1]  >> 16;

	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static void set_debug_reason(struct device *dev, int onoff)
{
	u32 wdata = 0;
	wdata = onoff;

	TOUCH_I("TCI Debug:%s\n", debug_type[wdata]);

	sw49106_reg_write(dev, LPWG_DEBUG_CTRL, &wdata, sizeof(wdata));
}

static int sw49106_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data[7];

	if ((d->tci_debug_type != 0) && (ts->lpwg.mode==LPWG_DOUBLE_TAP))
		set_debug_reason(dev, d->tci_debug_type);

	lpwg_data[0] = ts->tci.mode;
	lpwg_data[1] = info1->tap_count | (info2->tap_count << 16);
	lpwg_data[2] = info1->min_intertap | (info2->min_intertap << 16);
	lpwg_data[3] = info1->max_intertap | (info2->max_intertap << 16);
	lpwg_data[4] = info1->touch_slop | (info2->touch_slop << 16);
	lpwg_data[5] = info1->tap_distance | (info2->tap_distance << 16);
	lpwg_data[6] = info1->intr_delay | (info2->intr_delay << 16);

	return sw49106_reg_write(dev, TCI_ENABLE_W,
			&lpwg_data[0], sizeof(lpwg_data));
}

#if 0 // LPWG_PASSWORD disable for MH
static int sw49106_tci_password(struct device *dev)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	if (d->tci_debug_type != 0)
		set_debug_reason(dev, d->tci_debug_type);

	return sw49106_tci_knock(dev);
}
#endif

static int sw49106_tci_active_area(struct device *dev,
		u32 x1, u32 y1, u32 x2, u32 y2)
{
	int ret = 0, i;
	u32 active_area[4] = {x1+ACT_SENSELESS_AREA_W, y1+ACT_SENSELESS_AREA_W, \
						  x2-ACT_SENSELESS_AREA_W, y2-ACT_SENSELESS_AREA_W};

	TOUCH_I("%s: x1[%d], y1[%d], x2[%d], y2[%d]\n", __func__,
				active_area[0], active_area[1], active_area[2], active_area[3]);

	for (i=0; i < sizeof(active_area)/sizeof(u32); i++)
		active_area[i] = (active_area[i]) | (active_area[i] << 16);

	ret = sw49106_reg_write(dev, ACT_AREA_X1_W,
			&active_area[0], sizeof(u32));
	ret = sw49106_reg_write(dev, ACT_AREA_Y1_W,
			&active_area[1], sizeof(u32));
	ret = sw49106_reg_write(dev, ACT_AREA_X2_W,
			&active_area[2], sizeof(u32));
	ret = sw49106_reg_write(dev, ACT_AREA_Y2_W,
			&active_area[3], sizeof(u32));

	return ret;
}

static int sw49106_tci_control(struct device *dev, int type)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	struct tci_info *info2 = &ts->tci.info[TCI_2];
	u32 lpwg_data;
	int ret = 0;
	TOUCH_I("%s: type= %d\n",__func__,type);
	switch (type) {
	case ENABLE_CTRL:
		lpwg_data = ts->tci.mode;
		ret = sw49106_reg_write(dev, TCI_ENABLE_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case TAP_COUNT_CTRL:
		lpwg_data = info1->tap_count | (info2->tap_count << 16);
		ret = sw49106_reg_write(dev, TAP_COUNT_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case MIN_INTERTAP_CTRL:
		lpwg_data = info1->min_intertap | (info2->min_intertap << 16);
		ret = sw49106_reg_write(dev, MIN_INTERTAP_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case MAX_INTERTAP_CTRL:
		lpwg_data = info1->max_intertap | (info2->max_intertap << 16);
		ret = sw49106_reg_write(dev, MAX_INTERTAP_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case TOUCH_SLOP_CTRL:
		lpwg_data = info1->touch_slop | (info2->touch_slop << 16);
		ret = sw49106_reg_write(dev, TOUCH_SLOP_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case TAP_DISTANCE_CTRL:
		lpwg_data = info1->tap_distance | (info2->tap_distance << 16);
		ret = sw49106_reg_write(dev, TAP_DISTANCE_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case INTERRUPT_DELAY_CTRL:
		lpwg_data = info1->intr_delay | (info2->intr_delay << 16);
		ret = sw49106_reg_write(dev, INT_DELAY_W,
				&lpwg_data, sizeof(lpwg_data));
		break;

	case ACTIVE_AREA_CTRL:
		ret = sw49106_tci_active_area(dev, 0, 0, 1080, 2160); /*  [bringup] 1080 X 2160 */
		/*
				ts->tci.area.x1,
				ts->tci.area.y1,
				ts->tci.area.x2,
				ts->tci.area.y2);
		*/
		break;

	case ACTIVE_AREA_RESET_CTRL:
		/*
		ret = sw49106_tci_active_area(dev,
				(65 | 65 << 16),
				(1374 | 1374 << 16),
				(65 | 65 << 16),
				(2494 | 2494 << 16));
		*/
		break;

	default:
		break;
	}

	return ret;
}

static int sw49106_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];
	int ret = 0;

#ifdef SW49106_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("Not Ready, Need to turn on clock\n");
		return 0;
	}
#endif

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		ret = sw49106_tci_control(dev, ACTIVE_AREA_CTRL);
		ret = sw49106_tci_knock(dev);
		break;
#if 0 // LPWG_PASSWORD disable for MH
	case LPWG_PASSWORD:
		ts->tci.mode = 0x01 | (0x01 << 16);
		info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
		info1->tap_distance = 7;

		ret = sw49106_tci_control(dev, ACTIVE_AREA_CTRL);
		ret = sw49106_tci_password(dev);
		break;


	case LPWG_PASSWORD_ONLY:
		TOUCH_I("sw49106_lpwg_control LPWG_PASSWORD_ONLY\n");
		ts->tci.mode = 0x01 << 16;
		info1->intr_delay = 0;
		info1->tap_distance = 10;

		ret = sw49106_tci_control(dev, ACTIVE_AREA_CTRL);
		ret = sw49106_tci_password(dev);
		break;
#endif
	default:
		ts->tci.mode = 0;
		ret = sw49106_tci_control(dev, ENABLE_CTRL);
		break;
	}

	TOUCH_I("sw49106_lpwg_control mode = %d\n", mode);

	return ret;
}

static int sw49106_swipe_active_area(struct device *dev)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	struct swipe_info *left = &d->swipe.info[SWIPE_L];
	struct swipe_info *right = &d->swipe.info[SWIPE_R];
	u32 active_area[4] = {0x0, };
	int ret = 0;

	active_area[0] = (right->area.x1) | (left->area.x1 << 16);
	active_area[1] = (right->area.y1) | (left->area.y1 << 16);
	active_area[2] = (right->area.x2) | (left->area.x2 << 16);
	active_area[3] = (right->area.y2) | (left->area.y2 << 16);

	ret = sw49106_reg_write(dev, SWIPE_ACT_AREA_X1_W,
			active_area, sizeof(active_area));

	return ret;
}

static int sw49106_swipe_control(struct device *dev, int type)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	struct swipe_info *left = &d->swipe.info[SWIPE_L];
	struct swipe_info *right = &d->swipe.info[SWIPE_R];
	u32 swipe_data = 0;
	int ret = 0;

	switch (type) {
	case SWIPE_ENABLE_CTRL:
		swipe_data = d->swipe.mode;
		ret = sw49106_reg_write(dev, SWIPE_ENABLE_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_DISABLE_CTRL:
		swipe_data = 0;
		ret = sw49106_reg_write(dev, SWIPE_ENABLE_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_DIST_CTRL:
		swipe_data = (right->distance) | (left->distance << 16);
		ret = sw49106_reg_write(dev, SWIPE_DIST_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_THR_CTRL:
		swipe_data = (right->ratio_thres) | (left->ratio_thres << 16);
		ret = sw49106_reg_write(dev, SWIPE_RATIO_THR_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_PERIOD_CTRL:
		swipe_data = (right->ratio_period) | (left->ratio_period << 16);
		ret = sw49106_reg_write(dev, SWIPE_RATIO_PERIOD_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_RATIO_DIST_CTRL:
		swipe_data = (right->ratio_distance) |
				(left->ratio_distance << 16);
		ret = sw49106_reg_write(dev, SWIPE_RATIO_DIST_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_TIME_MIN_CTRL:
		swipe_data = (right->min_time) | (left->min_time << 16);
		ret = sw49106_reg_write(dev, SWIPE_TIME_MIN_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_TIME_MAX_CTRL:
		swipe_data = (right->max_time) | (left->max_time << 16);
		ret = sw49106_reg_write(dev, SWIPE_TIME_MAX_W,
				&swipe_data, sizeof(swipe_data));
		break;
	case SWIPE_AREA_CTRL:
		ret = sw49106_swipe_active_area(dev);
		break;
	default:
		break;
	}

	return ret;
}

static int sw49106_swipe_mode(struct device *dev, u8 lcd_mode)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	struct swipe_info *left = &d->swipe.info[SWIPE_L];
	struct swipe_info *right = &d->swipe.info[SWIPE_R];
	u32 swipe_data[11] = {0x0, };
	int ret = 0;

	if (!d->swipe.mode)
		return ret;

	if (lcd_mode != LCD_MODE_U2) {
		ret = sw49106_swipe_control(dev, SWIPE_DISABLE_CTRL);
		TOUCH_I("swipe disable\n");
	} else {
		swipe_data[0] = d->swipe.mode;
		swipe_data[1] = (right->distance) | (left->distance << 16);
		swipe_data[2] = (right->ratio_thres) | (left->ratio_thres << 16);
		swipe_data[3] = (right->ratio_distance) |
					(left->ratio_distance << 16);
		swipe_data[4] = (right->ratio_period) | (left->ratio_period << 16);
		swipe_data[5] = (right->min_time) | (left->min_time << 16);
		swipe_data[6] = (right->max_time) | (left->max_time << 16);
		swipe_data[7] = (right->area.x1) | (left->area.x1 << 16);
		swipe_data[8] = (right->area.y1) | (left->area.y1 << 16);
		swipe_data[9] = (right->area.x2) | (left->area.x2 << 16);
		swipe_data[10] = (right->area.y2) | (left->area.y2 << 16);

		ret = sw49106_reg_write(dev, SWIPE_ENABLE_W,
			&swipe_data[0], sizeof(swipe_data));

		TOUCH_I("swipe enable\n");
	}

	return ret;
}


static int sw49106_clock(struct device *dev, bool onoff)
{
#ifdef SW49106_TCL_OFF_VIA_MIPI
	struct touch_core_data *ts = to_touch_core(dev);

//	set_touch_osc(onoff); //0 : osc off / 1 : osc on
	if (onoff) {
		lge_panel_exit_deep_sleep();
		atomic_set(&ts->state.sleep, IC_NORMAL);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	} else {
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		lge_panel_enter_deep_sleep();
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
	}
	TOUCH_I("lge_panel_sleep_status = %s\n", (onoff == 0) ? "0 (deep)" : "1 (not deep)");
#else
	/* sw49106 needs to control touch oscillator clock via MIPI script. */
#endif
	return 0;
}

/*
 * BIT2 : 6LHB mode
 *
 * Sensing Type |  U3     | U3_PARTIAL | U3_QUICKCOVER
 * 6LHB         |  0x185  | 0x385      | 0x585
 * V-Blank      |  0x181  | 0x381      | 0x581
 */
int sw49106_tc_driving(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	u32 ctrl = 0;
	u32 rdata;

#ifdef SW49106_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("Not Ready, Need to turn on clock\n");
		return 0;
	}
#endif

	d->driving_mode = mode;

	switch (mode) {
	case LCD_MODE_U0:
		ctrl = 0x01;
		break;

	case LCD_MODE_U2:
		return 0;
		ctrl = 0x101;
		break;

	case LCD_MODE_U3:
		ctrl = 0x181; // [bringup]
		break;

	case LCD_MODE_U3_PARTIAL:
		return 0;
	//	ctrl = 0x381; [bringup]
	//	break;

	case LCD_MODE_U3_QUICKCOVER:
		ctrl = 0x581; //[bringup]
		break;

	case LCD_MODE_STOP:
		ctrl = 0x02;
		break;
	}

	/* When quick cover is in CLOSE state, set 1 in 10th bit of tc_drive_ctl.*/
	if(0) { //avoid build error for ts warning
		if (ts->lpwg.qcover == HOLE_NEAR)
			ctrl |= (1 << 10);
	}

	/* swipe set */
	if(0)	// [bringup]
		sw49106_swipe_mode(dev, mode);

	TOUCH_I("sw49106_tc_driving = 0x%04X, 0x%x\n", mode,ctrl);
	sw49106_reg_read(dev, spr_subdisp_st, &rdata, sizeof(rdata));
	TOUCH_I("DDI Display Mode = 0x%08X\n", rdata);
	sw49106_reg_write(dev, tc_drive_ctl, &ctrl, sizeof(ctrl));
	touch_msleep(20);


	return 0;
}

static void sw49106_deep_sleep(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
//	u32 setvalue = 0;
	
	sw49106_tc_driving(dev, LCD_MODE_STOP);
	if (!atomic_read(&ts->state.incoming_call)) {	/* Idle status */
		sw49106_write_value(dev, 0x008, 0);
	//	sw49106_reg_read(dev, 0x008, &setvalue, sizeof(setvalue));
	//	TOUCH_I("0x008 value = 0x%04X\n", setvalue);

		sw49106_clock(dev, 0);
	} else {					/* Ringing or Offhook status*/
		TOUCH_I("Avoid deep sleep during Call\n");
	}
}

static void sw49106_debug_tci(struct device *dev)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	u32 rdata = 0;
	u32 rdata1 = 0;
	u16 bit_mask = 0;

	if (!d->tci_debug_type)
		return;

	sw49106_reg_read(dev, LPWG_DEBUG_FAIL_STATUS, &rdata1, sizeof(rdata1));
	sw49106_reg_read(dev, TCI_DEBUG_FAILREASON_BUFFER, &rdata, sizeof(rdata));
	TOUCH_I("[LPWG_DEBUG_FAIL_STATUS=[0x%x] TCI_DEBUG_FAILREASON_BUFFER = [0x%x]\n", rdata1, rdata);
	if (rdata1&0x1) {
		bit_mask = (rdata & 0xffff);
		if (bit_mask < TCI_FAIL_NUM)
			TOUCH_I("[TCI_ON] TCI_DEBUG_FAILREASON_BUFFER = [0x%x]%s\n", rdata, tci_debug_str[bit_mask]);
	}

	if (rdata1&(0x1<<1)) {
		bit_mask = (rdata >> 16) & 0xffff;
		if (bit_mask < TCI_FAIL_NUM)
			TOUCH_I("[TCI_CODE] TCI_DEBUG_FAILREASON_BUFFER = [0x%x]%s\n", rdata, tci_debug_str[bit_mask]);
	}
}

static void sw49106_debug_swipe(struct device *dev)
{
	return;
}


static int sw49106_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}

#ifdef SW49106_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.hw_reset) == LCD_EVENT_TOUCH_RESET_START) {
		TOUCH_I("Touch IC is being reset. skip lpwg mode update\n");
		return 0;
	}
#endif

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			sw49106_lpwg_control(dev, LPWG_DOUBLE_TAP);
			sw49106_tc_driving(dev, d->lcd_mode);
			return 0;
		}
		if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg_mode\n");
#ifdef SW49106_TCL_OFF_VIA_MIPI
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				TOUCH_I("Not Ready, Need to turn on clock\n");
				return 0;
			}
#endif

#if 0 // not yet non RT debug
		sw49106_debug_tci(dev);
#endif
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("suspend sensor == PROX_NEAR\n");
			if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP){
				sw49106_deep_sleep(dev);
			}
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			TOUCH_I("Qcover == HOLE_NEAR\n");

			/* Deep Sleep same as Prox near  */
			if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP){
				sw49106_deep_sleep(dev);
			}
		} else {
			/* knock on/code */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				sw49106_clock(dev, 1);

			sw49106_lpwg_control(dev, ts->lpwg.mode);
			if (ts->lpwg.mode == LPWG_NONE &&
					d->lcd_mode == LCD_MODE_U0) {
				/* knock on/code disable */
				TOUCH_I("LCD_MODE_U0 - DeepSleep\n");
				sw49106_deep_sleep(dev);
			} else {
				sw49106_tc_driving(dev, d->lcd_mode);//needed to check
			}
		}
		return 0;
	}

	touch_report_all_event(ts);
	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("resume ts->lpwg.screen on\n");
		sw49106_lpwg_control(dev, LPWG_NONE);
		if (ts->lpwg.qcover == HOLE_NEAR){
			sw49106_tc_driving(dev, LCD_MODE_U3_QUICKCOVER);
			}
		else {
			sw49106_tc_driving(dev, d->lcd_mode);
		}

	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* wake up */
		TOUCH_I("resume ts->lpwg.mode == LPWG_NONE\n");
		sw49106_tc_driving(dev, LCD_MODE_STOP);
	}else if (ts->lpwg.sensor == PROX_NEAR) {
		TOUCH_I("resume ts->lpwg.sensor == PROX_NEAR\n");
		sw49106_deep_sleep(dev);
	} else {
		/* partial */
		TOUCH_I("resume Partial-Do not set for MH\n");

		//TBD active area for HOLE NEAR and FAR check
		/*
		if (ts->lpwg.qcover == HOLE_NEAR)
			sw49106_lpwg_control(dev, LPWG_NONE);
		else
			sw49106_lpwg_control(dev, ts->lpwg.mode);
		sw49106_tc_driving(dev, LCD_MODE_U3_PARTIAL);
		*/
	}

	return 0;
}

static int sw49106_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->tci.area.x1 = value[0];
		ts->tci.area.x2 = value[1];
		ts->tci.area.y1 = value[2];
		ts->tci.area.y2 = value[3];
		TOUCH_I("LPWG_ACTIVE_AREA: x0[%d], x1[%d], x2[%d], x3[%d]\n",
			value[0], value[1], value[2], value[3]);
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[TCI_2].tap_count = value[0];
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		break;

	case LPWG_UPDATE_ALL:
		if ( (ts->lpwg.screen == 1 && value[1] == 0 &&
			ts->lpwg.sensor == PROX_FAR) ||
			(ts->lpwg.qcover == 1 && value[3] == 0) )
			sw49106_setup_q_sensitivity(dev, 0);

		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];

		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg.mode,
			ts->lpwg.screen ? "ON" : "OFF",
			ts->lpwg.sensor ? "FAR" : "NEAR",
			ts->lpwg.qcover ? "CLOSE" : "OPEN");

		sw49106_lpwg_mode(dev);

		break;

	case LPWG_REPLY:
		break;

	}

	return 0;
}

static void sw49106_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
	int wireless_state = atomic_read(&ts->state.wireless);

	TOUCH_TRACE();

	d->charger = 0;
	/* wire */
	if (charger_state == CONNECT_INVALID)
		d->charger = CONNECT_NONE;
	else if ((charger_state == CONNECT_DCP)
			|| (charger_state == CONNECT_PROPRIETARY))
		d->charger = CONNECT_TA;
	else if (charger_state == CONNECT_HUB)
		d->charger = CONNECT_OTG;
	else
		d->charger = CONNECT_USB;

	/* wireless */
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try I2C\n");
		return;
	}
	sw49106_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u32));
}

static void sw49106_lcd_mode(struct device *dev, u32 mode)
{
	struct sw49106_data *d = to_sw49106_data(dev);

	TOUCH_I("lcd_mode: %d (prev: %d)\n", mode, d->lcd_mode);
	
	if (mode == LCD_MODE_U2_UNBLANK)
		mode = LCD_MODE_U2;

	d->lcd_mode = mode;
}

static int sw49106_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	sw49106_connect(dev);
	return 0;
}

static int sw49106_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	sw49106_connect(dev);
	return 0;
}

static int sw49106_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}

static int sw49106_debug_tool(struct device *dev, u32 value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (value == DEBUG_TOOL_ENABLE) {
		ts->driver->irq_handler = sw49106_sic_abt_irq_handler;
	} else {
		ts->driver->irq_handler = sw49106_irq_handler;
	}

	return 0;
}

#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
int lge_mdss_report_touchintpin_keep_low(void);
// [ESD] workaround code for COF type ============================= //
// If the state of int pin is keeped  low during 2 sec, notify to Display
//====================================================== //
static void sw49106_int_pin_work_func(struct work_struct *int_pin_work)
{
	struct sw49106_data *d =
			container_of(to_delayed_work(int_pin_work),
				struct sw49106_data, int_pin_work);
	struct touch_core_data *ts = to_touch_core(d->dev);
	int i =0;
	int int_pin_state = 1;
	int mfts_mode = 0;

	mfts_mode = touch_boot_mode_check(d->dev);

	if (mfts_mode >= MINIOS_MFTS_FOLDER) {
		return;
	}

	TOUCH_I("%s\n", __func__);
	for(i=0 ; i<20; i++) {
		int_pin_state = gpio_get_value(ts->int_pin);
		touch_msleep(100);
		if(int_pin_state == 1) {
			return;
		}
	}

	if((int_pin_state==0) && (d->lcd_mode == LCD_MODE_U3)){
		TOUCH_I("%s report touchintpin_keep_low to display\n", __func__);
		lge_mdss_report_touchintpin_keep_low();
	}
}
#endif

static void sw49106_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct sw49106_data *d =
			container_of(to_delayed_work(fb_notify_work),
				struct sw49106_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U0 || d->lcd_mode == LCD_MODE_U2)
		ret = FB_SUSPEND;
	else
		ret = FB_RESUME;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

static int sw49106_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
	case NOTIFY_TOUCH_RESET:
		if(atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_1)
			ret = 1;
		else
			ret = 0;
		TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
		//atomic_set(&d->watch.state.font_status, FONT_EMPTY);
		//atomic_set(&d->block_watch_cfg, BLOCKED);
		break;
	case LCD_EVENT_TOUCH_RESET_START:
		atomic_set(&ts->state.hw_reset, event);

		TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
		touch_gpio_direction_output(ts->reset_pin, 0);
		break;
	case LCD_EVENT_TOUCH_RESET_END:
		atomic_set(&ts->state.hw_reset, event);

		TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");
		touch_gpio_direction_output(ts->reset_pin, 1);

		queue_delayed_work(ts->wq, &ts->init_work,
					msecs_to_jiffies(ts->caps.hw_reset_delay));
		break;
	case LCD_EVENT_LCD_MODE:
		TOUCH_I("LCD_EVENT_LCD_MODE!\n");
		sw49106_lcd_mode(dev, *(u32 *)data);
		queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = sw49106_usb_status(dev, *(u32 *)data);
		break;
	case NOTIFY_WIRELEES:
		TOUCH_I("NOTIFY_WIRELEES!\n");
		ret = sw49106_wireless_status(dev, *(u32 *)data);
		break;
	case NOTIFY_EARJACK:
		TOUCH_I("NOTIFY_EARJACK!\n");
		ret = sw49106_earjack_status(dev, *(u32 *)data);
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
		ret = sw49106_reg_write(dev, REG_IME_STATE,
			(u32*)data, sizeof(u32));
		break;
	case NOTIFY_DEBUG_TOOL:
		ret = sw49106_debug_tool(dev, *(u32 *)data);
		TOUCH_I("NOTIFY_DEBUG_TOOL!\n");
		break;
	case NOTIFY_CALL_STATE:
		/* Notify TouchIC only for GSM call and idle state */
		if (*(u32*)data >= INCOMING_CALL_IDLE && *(u32*)data <= INCOMING_CALL_OFFHOOK) {
		TOUCH_I("NOTIFY_CALL_STATE!\n");
		ret = sw49106_reg_write(dev, REG_CALL_STATE,
			(u32*)data, sizeof(u32));
		}
		break;
	case NOTIFY_QMEMO_STATE:
		TOUCH_I("NOTIFY_QMEMO_STATE!\n");
		ret = sw49106_reg_write(dev, REG_QMEMO_STATE,
			(u32*)data, sizeof(u32));
		break;
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static void sw49106_init_works(struct sw49106_data *d)
{
	//INIT_DELAYED_WORK(&d->font_download_work, sw49106_font_download);
	INIT_DELAYED_WORK(&d->fb_notify_work, sw49106_fb_notify_work_func);
#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
	INIT_DELAYED_WORK(&d->int_pin_work, sw49106_int_pin_work_func);
#endif
}

static void sw49106_init_locks(struct sw49106_data *d)
{
	mutex_init(&d->spi_lock);
}

static int sw49106_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate synaptics data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	//touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);
//	touch_gpio_set_pull(ts->int_pin, 1);
if (0) {
	touch_gpio_init(ts->maker_id_pin, "touch_make_id");
	touch_gpio_direction_input(ts->maker_id_pin);
}
	/******************************************************
	 * Display driver does control the power in sw49106 IC *
	 * due to its design from 1-chip. Here we skip the    *
	 * control.                                           *
	 ******************************************************/
	//touch_power_init(dev);
	touch_bus_init(dev, MAX_XFER_BUF_SIZE);

	sw49106_init_works(d);
	sw49106_init_locks(d);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		touch_gpio_init(ts->reset_pin, "touch_reset");
		touch_gpio_direction_output(ts->reset_pin, 1);
		/* Deep Sleep */
		sw49106_deep_sleep(dev);
		return 0;
	}

	sw49106_get_tci_info(dev);
	sw49106_get_swipe_info(dev);

	d->lcd_mode = LCD_MODE_U3;
	d->tci_debug_type = TCI_DEBUG_ENABLE;
	sw49106_sic_abt_probe();

	return 0;
}

static int sw49106_remove(struct device *dev)
{
	TOUCH_TRACE();
	sw49106_sic_abt_remove();
	//sw49106_watch_remove(dev);

	return 0;
}

static int sw49106_shutdown(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int sw49106_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	u8 dev_major = d->fw.version[0];
	u8 dev_minor = d->fw.version[1];
	u32 bin_ver_offset = *((u32 *)&fw->data[0xe8]);
	u32 bin_pid_offset = *((u32 *)&fw->data[0xf0]);
	char pid[12] = {0};
	u8 bin_major;
	u8 bin_minor;
	int update = 0;

	if ((bin_ver_offset > FLASH_FW_SIZE) || (bin_pid_offset > FLASH_FW_SIZE)) {
		TOUCH_I("INVALID OFFSET\n");
		return -1;
	}

	bin_major = fw->data[bin_ver_offset];
	bin_minor = fw->data[bin_ver_offset + 1];
	memcpy(pid, &fw->data[bin_pid_offset], 8);

	if (ts->force_fwup) {
		update = 1;
	} else if (bin_major != dev_major) {
		update = 1;
	} else {
		if (bin_minor != dev_minor)
		update = 1;
	}
/*
	if(dev_major == 0 && dev_minor == 0){
		TOUCH_E("Touch FW cannot be 0.0!! Check your panel connection!!\n");
		update = 0;
	}
*/
	TOUCH_I(
		"bin-ver: %d.%02d (%s), dev-ver: %d.%02d -> update: %d, force_fwup: %d\n",
		bin_major, bin_minor, pid, dev_major, dev_minor,
		update, ts->force_fwup);

	return update;
}

static int sw49106_condition_wait(struct device *dev,
				    u16 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;

	do {
		touch_msleep(delay);
		sw49106_read_value(dev, addr, &data);

		if ((data & mask) == expect) {
			if (value)
				*value = data;
			TOUCH_I(
				"%d, addr[%04x] data[%08x], mask[%08x], expect[%08x]\n",
				retry, addr, data, mask, expect);
			return 0;
		}
	} while (--retry);

	if (value)
		*value = data;

	TOUCH_I("%s addr[%04x], expect[%x], mask[%x], data[%x]\n",
		__func__, addr, expect, mask, data);

	return -EPERM;
}
//-----Added for FW Upgrade---------
int specific_header_verify(unsigned char *header, int i)
{
	t_cfg_s_header_def *head = (t_cfg_s_header_def *)header;
	char tmp[8] = {0, };

	if (head->cfg_specific_info1.b.chip_rev <= 0
		&& head->cfg_specific_info1.b.chip_rev > 10) {
		TOUCH_I("Invalid Chip revision id %8.8X\n",
			head->cfg_specific_info1.b.chip_rev);
		return -2;
	}

	memset(tmp, 0, 8);
	memcpy((void*)tmp, (void *)&head->cfg_model_name, 4);

	TOUCH_I("==================== SPECIFIC #%d =====================\n",
						i +1);
	TOUCH_I("chip_rev           : %d\n",
					head->cfg_specific_info1.b.chip_rev);
	TOUCH_I("fpcb_id            : %d\n",
					head->cfg_specific_info1.b.fpcb_id);
	TOUCH_I("lcm_id             : %d\n",
					head->cfg_specific_info1.b.lcm_id);
	TOUCH_I("model_id           : %d\n",
					head->cfg_specific_info1.b.model_id);
	TOUCH_I("model_name         : %s\n", tmp);
	TOUCH_I("lot_id             : %d\n",
					head->cfg_specific_info2.b.lot_id);
	TOUCH_I("ver                : %d\n",
					head->cfg_specific_version);

	return 1;
}

int common_header_verify(t_cfg_info_def *header)
{
	t_cfg_info_def *head = (t_cfg_info_def *)header;
	t_cfg_c_header_def *common_head =
		(t_cfg_c_header_def *)(header + sizeof(t_cfg_info_def));

	if (head->cfg_magic_code != CFG_MAGIC_CODE) {
		TOUCH_I("Invalid CFG_MAGIC_CODE. %8.8X\n",
			head->cfg_magic_code);
		return -1;
	}

	if (head->cfg_chip_id != CFG_CHIP_ID) {
		TOUCH_I("Invalid Chip ID. (49106 != %d)\n",
			head->cfg_chip_id);
		return -2;
	}

	if (head->cfg_struct_version <= 0) {
		TOUCH_I("Invalid cfg_struct_version. %8.8X\n",
			head->cfg_struct_version);
		return -3;
	}

	if (head->cfg_specific_cnt <= 0) {
		TOUCH_I("No Specific Data. %8.8X\n",
			head->cfg_specific_cnt);
		return -4;
	}

	if (head->cfg_size.b.common_cfg_size > CFG_C_MAX_SIZE) {
		TOUCH_I("Over CFG COMMON MAX Size (%d). %8.8X\n",
			CFG_C_MAX_SIZE, head->cfg_size.b.common_cfg_size);
		return -5;
	}

	if (head->cfg_size.b.specific_cfg_size > CFG_S_MAX_SIZE) {
		TOUCH_I("Over CFG SPECIFIC MAX Size (%d). %8.8X\n",
			CFG_S_MAX_SIZE, head->cfg_size.b.specific_cfg_size);
		return -6;
	}

	TOUCH_I("==================== COMMON ====================\n");
	TOUCH_I("magic code         : 0x%8.8X\n", head->cfg_magic_code);
	TOUCH_I("chip id            : %d\n", head->cfg_chip_id);
	TOUCH_I("struct_ver         : %d\n", head->cfg_struct_version);
	TOUCH_I("specific_cnt       : %d\n", head->cfg_specific_cnt);
	TOUCH_I("cfg_c size         : %d\n", head->cfg_size.b.common_cfg_size);
	TOUCH_I("cfg_s size         : %d\n",
					head->cfg_size.b.specific_cfg_size);
	TOUCH_I("date               : 0x%8.8X\n", head->cfg_global_date);
	TOUCH_I("time               : 0x%8.8X\n", head->cfg_global_time);
	TOUCH_I("common_ver         : %d\n", common_head->cfg_common_ver);

	return 1;
}

static int sw49106_img_binary_verify(unsigned char *imgBuf)
{
	unsigned char *specific_ptr;
	unsigned char *cfg_buf_base = &imgBuf[FLASH_FW_SIZE];
	int i;
	t_cfg_info_def *head = (t_cfg_info_def *)cfg_buf_base;

	u32 *fw_crc = (u32 *)&imgBuf[FLASH_FW_SIZE -4];
	u32 *fw_size = (u32 *)&imgBuf[FLASH_FW_SIZE -8];

	if (*fw_crc == 0x0
		|| *fw_crc == 0xFFFFFFFF
		|| *fw_size > FLASH_FW_SIZE) {
		TOUCH_I("Firmware Size Invalid READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC Invalid READ : 0x%X\n", *fw_crc);
		return E_FW_CODE_SIZE_ERR;
	} else {
		TOUCH_I("Firmware Size READ : 0x%X\n", *fw_size);
		TOUCH_I("Firmware CRC READ : 0x%X\n", *fw_crc);
	}

	if (common_header_verify(head) < 0) {
		TOUCH_I("No Common CFG! Firmware Code Only\n");
		return E_FW_CODE_ONLY_VALID;
	}

	specific_ptr = cfg_buf_base + head->cfg_size.b.common_cfg_size;
	for (i = 0; i < head->cfg_specific_cnt; i++) {
		if (specific_header_verify(specific_ptr, i) < 0) {
			TOUCH_I("specific CFG invalid!\n");
			return -2;
		}
		specific_ptr += head->cfg_size.b.specific_cfg_size;
	}

	return E_FW_CODE_AND_CFG_VALID;
}

//-------------------------------
static int sw49106_fw_upgrade(struct device *dev,
			     const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 *fwdata = (u8 *) fw->data;
	u32 data;
	u32 conf_dn_addr;
	u32 conf_specific_dn_index;
	u32 cfg_c_size;
	u32 cfg_s_size;
	t_cfg_info_def *head;
	int ret;
	int i = 0;
	int img_check_result;

	//--------- Binary Check Verification Start-----------------------
		TOUCH_I("%s - Checking FW Image before flashing\n", __func__);
		if(fw->size > FLASH_SIZE) {
			TOUCH_I("%s - FW Image Size(0x%x) is not correct\n", __func__,fw->size);
			return -EPERM;
		} else {
			TOUCH_I("%s - FW Image Size(0x%x) FLASH_FW_SIZE= 0x%x\n",__func__,fw->size,FLASH_FW_SIZE);
		}
	if(0) {//CFG area deleted
			img_check_result = sw49106_img_binary_verify((unsigned char*)fwdata);

			switch (img_check_result) {
				case E_FW_CODE_AND_CFG_VALID:
					break;
				case E_FW_CODE_CFG_ERR:
				case E_FW_CODE_SIZE_ERR:
				case E_FW_CODE_ONLY_VALID:
				default:
					TOUCH_I("%s - FW Image Verification fail!!\n", __func__);
					return -EPERM;
			}
			TOUCH_I("%s - FW Image Verification success!!\n", __func__);
		}
	//-----------Binary Check Verification End-------------------------
	/* enable SPI between RAM and ROM */
	sw49106_write_value(dev, 0x15, 0);

	//---------------------------------------------------
	/* Reset Touch CM3 core and put system on hold */
	sw49106_write_value(dev, sys_rst_ctl, 2);

	/* sram write enable */
	sw49106_write_value(dev, sys_sram_ctl, 3);

	/* Write F/W Code to CODE SRAM (80KB) */
	for (i = 0 ; i < FLASH_FW_SIZE ; i += MAX_RW_SIZE) {

		/* Set code sram base address write */
		sw49106_write_value(dev, spr_code_offset, i / 4);

		/* firmware image download to code sram */
		sw49106_reg_write(dev, code_access_addr, &fwdata[i], MAX_RW_SIZE);
	}
	/* sram write disable */
	sw49106_write_value(dev, sys_sram_ctl, 0);
		
	/* Release Touch CM3 core reset*/
	sw49106_write_value(dev, sys_rst_ctl, 0);

	/* Start CM3 Boot after Code Dump */
	sw49106_write_value(dev, sys_boot_ctl, 1);

	/* Check F/W Boot Done Status */
	ret = sw49106_condition_wait(dev, tc_flash_dn_sts, NULL,
				    FLASH_BOOTCHK_VALUE, 0xFFFFFFFF, 10, 200);
	if (ret < 0) {
		TOUCH_E("failed : \'boot check\'\n");
		return -EPERM;
	} else {
		TOUCH_I("success : boot check\n");
	}
	//---------------------------------------------------

	//--------------F/W Code Flash Download Start---------
		/* Dump F/W Code with Flash DMA */
		sw49106_write_value(dev,tc_flash_dn_ctl,(FLASH_KEY_CODE_CMD << 16) | 1);
		touch_msleep(ts->caps.hw_reset_delay);
	
		/* Check F/W Code Flash Download Status */
		ret = sw49106_condition_wait(dev, tc_flash_dn_sts, &data,
		                 FLASH_CODE_DNCHK_VALUE, 0xFFFFFFFF, 10, 200);
		if (ret < 0) {
			TOUCH_E("failed : \'code check\'\n");
			return -EPERM;
		} else {
		TOUCH_I("success : code check\n");
	    }
	//--------------F/W Code Flash Download End---------

if(0) { //blocked CFG, OTPM Download as per vendor suggestion
	if (img_check_result == E_FW_CODE_AND_CFG_VALID) {
			head = (t_cfg_info_def *)&fwdata[FLASH_FW_SIZE];

			cfg_c_size = head->cfg_size.b.common_cfg_size;
			cfg_s_size = head->cfg_size.b.specific_cfg_size;

			/* conf base address read */
			sw49106_reg_read(dev, tc_confdn_base_addr, (u8 *)&data, sizeof(u32));
			conf_dn_addr =  ((data) & 0xFFFF);
			conf_specific_dn_index= ((data >> 16) & 0xFFFF);
			TOUCH_I("conf_dn_addr : %08x data: %08x conf_specific_dn_index : %08x  \n",
		  			conf_specific_dn_index, conf_dn_addr, data);
		    if (conf_specific_dn_index == 0 ||
					((conf_specific_dn_index * cfg_s_size) >
					 (fw->size - FLASH_FW_SIZE - cfg_c_size))) {
				TOUCH_I("Invalid Specific CFG Index => 0x%8.8X\n",
					conf_specific_dn_index);
				return -EPERM;
		    }
		    if (conf_dn_addr >= (0x1600) || conf_dn_addr < (0x8C0)) {
			TOUCH_E("failed : \'conf base invalid \'\n");
			return -EPERM;
		    }
		    //--------------Config Data Flash Download Start-------------------
			/* cfg_c sram base address write */
			sw49106_write_value(dev, spr_data_offset,conf_dn_addr);

			/* Conf data download to conf sram */
			sw49106_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE],cfg_c_size);

			/* cfg_s sram base address write */
			sw49106_write_value(dev, spr_data_offset,conf_dn_addr + cfg_c_size/4);
		
			// CFG Specific Download to CFG Download buffer (SRAM)
			sw49106_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE + cfg_c_size +
						(conf_specific_dn_index - 1) * cfg_s_size], cfg_s_size);
		

			/* Conf Download Start */
			sw49106_write_value(dev,tc_flash_dn_ctl,(FLASH_KEY_CONF_CMD << 16) | 2);

			/* Conf check */
			ret = sw49106_condition_wait(dev, tc_flash_dn_sts,&data,
			               FLASH_CONF_DNCHK_VALUE,0xFFFFFFFF, 10, 200);
			if (ret < 0) {
				TOUCH_E("failed : \'cfg check\'\n");
				return -EPERM;
			} else {
				TOUCH_I("success : cfg_check\n");
			}

			ret = specific_header_verify(&fwdata[FLASH_FW_SIZE + cfg_c_size + (conf_specific_dn_index - 1)*cfg_s_size],
							conf_specific_dn_index - 1);
			if(ret < 0) {
				TOUCH_I("specific header invalid!\n");
				return -EPERM;
			}
		    //--------------Config Data Flash down End-------------------

		    //------------------OTPM Download Start-----------------
			/* cfg_s sram base address write */
			sw49106_write_value(dev, spr_data_offset,conf_dn_addr);
					
			// OTPM Data Download to (SRAM)
			sw49106_reg_write(dev, data_access_addr, &fwdata[FLASH_FW_SIZE +
						FLASH_CONF_SIZE], FLASH_OTP_SIZE);
		
			/* OTPM Download Start */
			sw49106_write_value(dev,tc_flash_dn_ctl,(FLASH_KEY_OTPM_CMD << 16) | 4);
					
			/*Check OTPM Data Flash Download Status */
			ret = sw49106_condition_wait(dev, tc_flash_dn_sts,&data,
			            FLASH_OTP_DNCHK_VALUE,0xFFFFFFFF, 10, 200);
			if (ret < 0) {
				TOUCH_E("failed : \'OTPM check\'\n");
				return -EPERM;
			} else {
				TOUCH_I("success : OTPM check\n");
			}
		    //------------------OTPM Download End------------------
	    }
}

	TOUCH_I("===== Firmware download Okay =====\n");

	return 0;
}

static int sw49106_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
    struct sw49106_data *d = to_sw49106_data(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;
	int i = 0;

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n",
			&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		/*if (d->fw.revision <= REV_6) {*/
			memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		/*} else if (d->fw.revision == REV_8) {
			memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath));
		} else if (d->fw.revision == REV_9) {
			memcpy(fwpath, ts->def_fwpath[2], sizeof(fwpath));
		}*/

		TOUCH_I("get fwpath from def_fwpath : %s, lcd_revision: %d\n",
					fwpath, d->fw.lcd_fpcb_revision);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);

		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	if (sw49106_fw_compare(dev, fw)) {
		ret = -EINVAL;
		touch_msleep(200);
		for (i = 0; i < 2 && ret; i++)
			ret = sw49106_fw_upgrade(dev, fw);
	} else {
		release_firmware(fw);
		return 0;
	}
	if (!ret) {
		d->err_cnt = 0;
		TOUCH_I("FW upgrade retry err_cnt clear\n");
	}

	release_firmware(fw);

	return ret;
}

static int sw49106_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		sw49106_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
	}

	if (atomic_read(&d->init) == IC_INIT_DONE)
		sw49106_lpwg_mode(dev);
	else /* need init */
		ret = 1;

	return ret;
}

static int sw49106_resume(struct device *dev)
{
#if 0 // FW-upgrade not working at MFTS mode
	struct touch_core_data *ts = to_touch_core(dev);
	int mfts_mode = 0;
#endif

	TOUCH_TRACE();

#if 0 // FW-upgrade not working at MFTS mode
	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		sw49106_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		sw49106_ic_info(dev);
		if (sw49106_upgrade(dev) == 0) {
			sw49106_power(dev, POWER_OFF);
			sw49106_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
	}
#endif
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		sw49106_deep_sleep(dev);
		return -EPERM;
	}

	return 0;
}

static int sw49106_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	u32 data = 1;
	int ret = 0;

	TOUCH_TRACE();

#ifdef SW49106_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("TC clock is off. Turn it on before init\n");
		sw49106_clock(dev, 1);
	}
#endif

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("fb_notif change\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = sw49106_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}
//MH sw49016 code start SPI disable

	sw49106_write_value(dev, SERIAL_SPI_EN, 0);
	TOUCH_I("Serial Control Addr=[%04x] val=[%04x] \n", SERIAL_SPI_EN, 0);
	sw49106_write_value(dev, SERIAL_I2C_EN, 1);
	TOUCH_I("Serial Control Addr=[%04x] val=[%04x] \n", SERIAL_I2C_EN, 1);
	sw49106_write_value(dev, SPI_TATTN_OPT, 3);
	TOUCH_I("Serial Control Addr=[%04x] val=[%04x] \n", SPI_TATTN_OPT, 3);

//MH sw49106 code end

	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);

	if (atomic_read(&ts->state.debug_tool) == DEBUG_TOOL_ENABLE)
		sw49106_sic_abt_init(dev);
	ret = sw49106_ic_info(dev);
	if (ret) {
		TOUCH_E("sw49106_ic_info failed, ret:%d\n", ret);
		return ret;
	}

	ret = sw49106_reg_write(dev, tc_device_ctl, &data, sizeof(data));
	if (ret) {
		TOUCH_E("failed to write \'tc_device_ctrl\', ret:%d\n", ret);
		return ret;
	}

	ret = sw49106_reg_write(dev, tc_interrupt_ctl, &data, sizeof(data));
	if (ret) {
		TOUCH_E("failed to write \'tc_interrupt_ctrl\', ret:%d\n", ret);
		return ret;
	}
	ret = sw49106_reg_write(dev, SPR_CHARGER_STS, &d->charger, sizeof(u32));
	if (ret)
		TOUCH_E("failed to write \'spr_charger_sts\', ret:%d\n", ret);

	data = atomic_read(&ts->state.ime);
	ret = sw49106_reg_write(dev, REG_IME_STATE, &data, sizeof(data));
	if (ret)
		TOUCH_E("failed to write \'reg_ime_state\', ret:%d\n", ret);

	TOUCH_D(QUICKCOVER,"%s : %s(%d)\n", __func__,
			(d->q_sensitivity) ? "SENSITIVE" : "NORMAL", (d->q_sensitivity));

	ret = sw49106_reg_write(dev, Q_TOUCH_SENSE, &d->q_sensitivity, sizeof(u32));
	if (ret)
		TOUCH_E("failed to write \'QCOVER_SENSITIVITY\', ret:%d\n", ret);

	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);

	ret = sw49106_lpwg_mode(dev);
	if (ret)
		TOUCH_E("failed to lpwg_control, ret:%d", ret);

	return 0;
}

/* (1 << 5)|(1 << 6)|(1 << 7)|(1 << 9)|(1 << 10) */
/*#define INT_RESET_CLR_BIT   0x6C0*/  /*Global reset use only*/
/* (1 << 10) */  /* Global Reset bit */
#define INT_GLOBAL_RESET_CLR_BIT 0x400
/* (1 << 6)|(1 << 7)|(1 << 9) */  /* Touch HW Reset bit */
#define INT_HW_RESET_CLR_BIT 0x2C0
/* LG4894 not set */
#define INT_SW_RESET_CLR_BIT 0x0
/* (1 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_LOGGING_CLR_BIT     0x50A000
/* LG4894 not set */
#define INT_FWU_CLR_BIT      0x0
/* (1 << 5) |(1 << 6) |(1 << 7)|(0 << 9)|(0 << 10)|(0 << 13)|(1 << 15)|(1 << 20)|(1 << 22) */
#define INT_NORMAL_MASK                  0x5080E0
#define IC_DEBUG_SIZE              16       /* byte */
/* During tumble, IC status is abnormal due to I2C error occurs and requires HW reset */
#define ABNORMAL_IC_BIT	0x6F000C0

int sw49106_check_status(struct device *dev)
{
           struct sw49106_data *d = to_sw49106_data(dev);
           int ret = 0;
           u32 ic_status = d->info.ic_status; //0x200
           u32 status = d->info.device_status; //0x201
           u32 debugging_num = 0;
           u32 debugging_mask = 0x0;
           u32 status_mask = 0x0;
           int checking_log_flag = 0;
           const int checking_log_size = 1024;
           char *checking_log = NULL;
           int length = 0;

           status_mask = status ^ INT_NORMAL_MASK; //mask 1 is OK bit
           debugging_mask = ((status >> 16) & 0xF);
           if ((status_mask & INT_GLOBAL_RESET_CLR_BIT) || (ic_status & (1 << 0)) || (ic_status & (1 << 3)) || (ic_status & (1 << 6)) || (ic_status & (1 << 7))) {
                     TOUCH_I("%s : Need Global Reset, status = %x, ic_status = %x\n",
                                __func__, status, ic_status);
                     ret = -ERESTART;
           } else if ((status_mask & INT_HW_RESET_CLR_BIT)
                                           || (ic_status & (1 << 5))) {
                     TOUCH_I("%s : Need Touch HW Reset, status = %x, ic_status = %x\n",
                                __func__, status, ic_status);
                     ret = -EHWRESET;
           } else if (status_mask & INT_SW_RESET_CLR_BIT) {
                     TOUCH_I("%s : Need Touch SW Reset, status = %x, ic_status = %x\n",
                                __func__, status, ic_status);
                     ret = -ESWRESET;
           } else if ((status_mask & INT_LOGGING_CLR_BIT) || (debugging_mask == 0x4)) {
		if(ABNORMAL_IC_BIT == status){
                     TOUCH_I("%s : Need Touch HW reset, status = %x, ic_status = %x\n",
                                __func__, status, ic_status);
                     ret = -EHWRESET;
		}
		else{
                     TOUCH_I("%s : Need Logging, status = %x, ic_status = %x\n",
                                __func__, status, ic_status);
                     ret = -ERANGE;
		}
           } else if (status_mask & INT_FWU_CLR_BIT) {
                     if (d->err_cnt >= 3) {
                                ret = -ERANGE;
                     } else {
                                d->err_cnt++;
                                ret = -EUPGRADE;
                     }
                     TOUCH_I("%s : Need FW Upgrade, status = %x, ic_status = %x err_cnt = %d %s\n",
                                __func__, status, ic_status, d->err_cnt,
                                d->err_cnt>=3 ? " skip upgrade":"");
           }

           if (ret != 0) {
                     checking_log = kzalloc(sizeof(*checking_log) * checking_log_size, GFP_KERNEL);
                     if(checking_log == NULL) {
                                TOUCH_E("Failed to allocate mem for checking_log\n");
                                ret = -ENOMEM;
                                goto error;
                     }
                     if (ic_status & (1 << 0)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[IC_STATUS] [0]ESD detection");
                     }
                     if (ic_status & (1 << 3)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[IC_STATUS] [3]Watchdog Exception");
                     }
                     if (ic_status & (1 << 5)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[IC_STATUS] [5]CM3 Fault Status");
                     }
		     if (ic_status & (1 << 6)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[IC_STATUS] [6]MIPI ESD Error (DIC MIPI ERROR)");
                     }
                     if (ic_status & (1 << 7)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[IC_STATUS] [7]Check Sum Error (DIC Check Sum ERROR)");
                     }
                     if (!(status & (1 << 5))) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [5]Device_ctl not Set");
                     }
                     if (!(status & (1 << 6))) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [6]Code CRC Invalid");
                     }
                     if (!(status & (1 << 7))) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [7]CFG CRC Invalid");
                     }
                     if (status & (1 << 9)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [9]Abnormal status Detected");
                     }
                     if (status & (1 << 10)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [10]System Error Detected");
                     }
                     if (status & (1 << 13)) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [13]Display mode Mismatch");
                     }
                     if (!(status & (1 << 15))) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [15]Interrupt_Pin Invalid");
                     }
                     if (!(status & (1 << 20))) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [20]Touch interrupt status Invalid");
                     }
                     if (!(status & (1 << 22))) {
                                checking_log_flag = 1;
                                length += snprintf(checking_log + length,
                                          checking_log_size - length,
                                          "[TC_STATUS] [22]TC driving Invalid");
                     }

                     if (checking_log_flag) {
                                TOUCH_E("%s, status = %x, ic_status = %x\n",
                                                     checking_log, status, ic_status);
                     }
                     if(checking_log != NULL)
                                kfree(checking_log);
           }

/*
*        0x3 abnormal Error
*        0x4 debugging INT
*/
           if (debugging_mask == 0x3 || debugging_mask == 0x4) {
                     sw49106_reg_read(dev, ic_debug_info_addr, &d->info.debug[0].ic_debug_info,
                                sizeof(d->info.debug[0].ic_debug_info));
                     debugging_num = d->info.debug[0].ic_debug_info;
                     if (debugging_num < IC_DEBUG_INFO_NUM )
                                TOUCH_E("[IC_DEBUG_INFO] [%d]%s \n", debugging_num,
                                          ic_debug_info_str[debugging_num]);
           }
error:
           return ret;
}


int sw49106_debug_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	struct sw49106_touch_debug *debug = d->info.debug;
	int ret = 0;
	int id = 0;
	int i = 0;

	u16 debug_change_mask = 0;
	u16 press_mask = 0;
	id = DEBUG_PROTOCOL_PACKET_NUM - 1;

	debug_change_mask = ts->old_mask ^ ts->new_mask;
	press_mask = ts->new_mask & debug_change_mask;

	/* check protocol ver */
	if (debug[id].protocol_ver < 0)
		return ret;

	/* check debugger status */
	if ((atomic_read(&ts->state.earjack) == EARJACK_DEBUG) ||
		(gpio_get_value(126) < 1))
			return ret;

	if (debug_change_mask && press_mask) {
		u8 buf[DEBUG_BUF_SIZE] = {0,};
		ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
				"[%d] %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				ts->tcount - 1,
				debug[id].protocol_ver,
				debug[id].frame_cnt,
				debug[id].rn_max_bfl,
				debug[id].rn_max_afl,
				debug[id].rn_min_bfl,
				debug[id].rn_min_afl,
				debug[id].rn_max_afl_x,
				debug[id].rn_max_afl_y,
				debug[id].seg1_cnt,
				debug[id].seg2_cnt,
				debug[id].seg1_thr,
				debug[id].rn_pos_cnt,
				debug[id].rn_neg_cnt,
				debug[id].rn_pos_sum,
				debug[id].rn_neg_sum,
				debug[id].rn_stable
			       );

		for (i = 0 ; i < ts->tcount ; i++) {
			if (i < 1)
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"[Touch]	tb:");
			ret += snprintf(buf + ret, DEBUG_BUF_SIZE - ret,
					"%2d ",	debug[id].track_bit[i]);
		}

		for (i = 0 ; i < sizeof(debug[id].rn_max_tobj) ; i++) {
			if (debug[id].rn_max_tobj[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" to:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug[id].rn_max_tobj[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].rebase) ; i++) {
			if (debug[id].rebase[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" re:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug[id].rebase[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].noise_detect) ; i++) {
			if (debug[id].noise_detect[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" nd:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2d ",
						debug[id].noise_detect[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].lf_oft) ; i++) {
			if (debug[id].lf_oft[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" lf:");
				ret += snprintf(buf + ret,
						DEBUG_BUF_SIZE - ret,
						"%2x ",	debug[id].lf_oft[i]);
			} else {
				break;
			}
		}

		for (i = 0 ; i < sizeof(debug[id].palm) ; i++) {
			if (debug[id].palm[i] > 0) {
				if (i < 1)
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							" pa:");
					ret += snprintf(buf + ret,
							DEBUG_BUF_SIZE - ret,
							"%2d ",
							debug[id].palm[i]);
				} else {
					break;
				}
			}
		TOUCH_I("%s\n", buf);
	}
	return ret;
}
static int sw49106_irq_abs_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	struct sw49106_touch_data *data = d->info.data;
	struct touch_data *tdata;
	u32 touch_count = 0;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	touch_count = d->info.touch_cnt;
	ts->new_mask = 0;

	/* check q cover status */
	if (d->driving_mode == LCD_MODE_U3_QUICKCOVER && !d->q_sensitivity) {
		TOUCH_I("Interrupt in Qcover closed\n");
		ts->is_cancel = 1;
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	/* check if palm detected */
	if (data[0].track_id == PALM_ID) {
		if (data[0].event == TOUCHSTS_DOWN) {
			ts->is_cancel = 1;
			TOUCH_I("Palm Detected\n");
		} else if (data[0].event == TOUCHSTS_UP) {
			ts->is_cancel = 0;
			TOUCH_I("Palm Released\n");
		}
		ts->tcount = 0;
		ts->intr_status = TOUCH_IRQ_FINGER;
		return ret;
	}

	for (i = 0; i < touch_count; i++) {
		if (data[i].track_id >= MAX_FINGER)
			continue;

		if (data[i].event == TOUCHSTS_DOWN
			|| data[i].event == TOUCHSTS_MOVE) {
			ts->new_mask |= (1 << data[i].track_id);
			tdata = ts->tdata + data[i].track_id;

			tdata->id = data[i].track_id;
			tdata->type = data[i].tool_type;
			tdata->x = data[i].x;
			tdata->y = data[i].y;
			tdata->pressure = data[i].pressure;
			tdata->width_major = data[i].width_major;
			tdata->width_minor = data[i].width_minor;

			if (data[i].width_major == data[i].width_minor)
				tdata->orientation = 1;
			else
				tdata->orientation = (s8)data[i].angle;

			finger_index++;

			TOUCH_D(ABS,
				"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
					tdata->id,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);
		}
	}

#ifdef SW49106_ESD_SKIP_WHILE_TOUCH_ON
	if (finger_index != finger_cnt) {
		TOUCH_D(ABS, "finger cnt changed from %d to %d\n", finger_cnt, finger_index);
		finger_cnt = finger_index;
	}
#endif

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

int sw49106_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);

	/* check if touch cnt is valid */
	if (d->info.touch_cnt == 0 || d->info.touch_cnt > ts->caps.max_id) {
		TOUCH_I("%s : touch cnt is invalid - %d\n",
			__func__, d->info.touch_cnt);
		return -ERANGE;
	}

	return sw49106_irq_abs_data(dev);
}

int sw49106_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	int ret = 0;

	if (d->info.wakeup_type == KNOCK_1) {
		if (ts->lpwg.mode != LPWG_NONE) {
			sw49106_get_tci_data(dev,
				ts->tci.info[TCI_1].tap_count);
			ts->intr_status = TOUCH_IRQ_KNOCK;
		}
	} else if (d->info.wakeup_type == KNOCK_2) {
		if (ts->lpwg.mode >= LPWG_PASSWORD) {
			sw49106_get_tci_data(dev,
				ts->tci.info[TCI_2].tap_count);
			ts->intr_status = TOUCH_IRQ_PASSWD;
		}
	} else if (d->info.wakeup_type == SWIPE_LEFT) {
		TOUCH_I("SWIPE_LEFT\n");
		sw49106_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_LEFT;
	} else if (d->info.wakeup_type == SWIPE_RIGHT) {
		TOUCH_I("SWIPE_RIGHT\n");
		sw49106_get_swipe_data(dev);
		ts->intr_status = TOUCH_IRQ_SWIPE_RIGHT;
	} else if (d->info.wakeup_type == KNOCK_OVERTAP) {
		TOUCH_I("LPWG wakeup_type is Overtap\n");
		sw49106_get_tci_data(dev, 1);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if (d->info.wakeup_type == CUSTOM_DEBUG) {
		TOUCH_I("LPWG wakeup_type is CUSTOM_DEBUG\n");
		sw49106_debug_tci(dev);
		sw49106_debug_swipe(dev);
	} else {
		TOUCH_I("LPWG wakeup_type is not support type![%d]\n",
			d->info.wakeup_type);
	}

	return ret;
}

int sw49106_irq_handler(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	int ret = 0;

	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_2) {
		ret = sw49106_reg_read(dev, tc_ic_status, &d->info,
				sizeof(d->info));
		if(ret < 0) {
			goto error;
		}
	} else {
		ret = sw49106_reg_read(dev, tc_ic_status, &d->info,
				sizeof(d->info) - sizeof(d->info.debug));
		if(ret < 0) {
#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
			if(d->lcd_mode == LCD_MODE_U3) {
				queue_delayed_work(ts->wq, &d->int_pin_work, 0);
			}
#endif
			goto error;
		}
	}

	ret = sw49106_check_status(dev);
	if (ret < 0)
		goto error;

	if (d->info.wakeup_type == ABS_MODE)
		ret = sw49106_irq_abs(dev);
	else
		ret = sw49106_irq_lpwg(dev);

	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_2)
		sw49106_debug_info(dev);
error:
	return ret;
}

static ssize_t store_reg_ctrl(struct device *dev,
				const char *buf, size_t count)
{
	char command[6] = {0};
	u32 reg = 0;
	int value = 0;
	u32 data = 1;
	u16 reg_addr;

	if (sscanf(buf, "%5s %x %x", command, &reg, &value) <= 0)
		return count;

	reg_addr = reg;
	if (!strcmp(command, "write")) {
		data = value;
		if (sw49106_reg_write(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else if (!strcmp(command, "read")) {
		if (sw49106_reg_read(dev, reg_addr, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", reg_addr);
		else
			TOUCH_I("reg[%x] = 0x%x\n", reg_addr, data);
	} else {
		TOUCH_D(BASE_INFO, "Usage\n");
		TOUCH_D(BASE_INFO, "Write reg value\n");
		TOUCH_D(BASE_INFO, "Read reg\n");
	}
	return count;
}

static ssize_t show_tci_debug(struct device *dev, char *buf)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	int ret = 0;
	u32 rdata = -1;

	if (sw49106_reg_read(dev, LPWG_DEBUG_CTRL,
				(u8 *)&rdata, sizeof(rdata)) < 0) {
		TOUCH_I("Fail to Read TCI Debug Reason type\n");
		return ret;
	}

	ret = snprintf(buf + ret, PAGE_SIZE,
			"Read TCI Debug Reason type[IC] = %s\n",
			debug_type[(rdata & 0x1) ? 1 : 0]);					
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Read TCI Debug Reason type[Driver] = %s\n",
			debug_type[d->tci_debug_type]);
	TOUCH_I("Read TCI Debug Reason type = %s\n",
			debug_type[d->tci_debug_type]);

	return ret;
}

static ssize_t store_tci_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 1 || value < 0) {
		TOUCH_I("SET TCI debug , 0(disable), 1(enable) only\n");
		return count;
	}

	d->tci_debug_type = (u8)value;
	TOUCH_I("SET TCI Debug = %s\n", debug_type[value]);

	return count;
}

static ssize_t show_swipe_debug(struct device *dev, char *buf)
{
	return 0;
}

static ssize_t show_pinstate(struct device *dev, char *buf)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	ret = snprintf(buf, PAGE_SIZE, "RST:%d, INT:%d\n",
			gpio_get_value(ts->reset_pin), gpio_get_value(ts->int_pin));
	TOUCH_I("%s() buf:%s",__func__, buf);	
	return ret;
}
static ssize_t store_swipe_debug(struct device *dev,
						const char *buf, size_t count)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 1 || value < 0) {
		TOUCH_I("SET SWIPE debug , 0(disable), 1(enable) only\n");
		return count;
	}

	d->swipe_debug_type = (u8)value;
	TOUCH_I("Write SWIPE Debug reason type = %s\n", debug_type[value]);

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	sw49106_reset_ctrl(dev, value);

//	sw49106_init(dev);

	return count;
}

static ssize_t store_q_sensitivity(struct device *dev, const char *buf,
								size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct sw49106_data *d = to_sw49106_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	mutex_lock(&ts->lock);

	TOUCH_D(QUICKCOVER, "%s: change sensitivity %d -> %d", __func__, d->q_sensitivity, (value));
	d->q_sensitivity = (value); // 1=enable touch, 0=disable touch

	#ifdef SW49106_TCL_OFF_VIA_MIPI
	if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP)
	#endif
	sw49106_reg_write(dev, Q_TOUCH_SENSE, &d->q_sensitivity, sizeof(u32));

	mutex_unlock(&ts->lock);

	TOUCH_I("%s : %s(%d)\n", __func__,
			(d->q_sensitivity) ? "SENSITIVE" : "NORMAL", (d->q_sensitivity));

	return count;
}
static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(tci_debug, show_tci_debug, store_tci_debug);
static TOUCH_ATTR(swipe_debug, show_swipe_debug, store_swipe_debug);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(q_sensitivity, NULL, store_q_sensitivity);
static TOUCH_ATTR(pinstate, show_pinstate, NULL);


static struct attribute *sw49106_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_tci_debug.attr,
	&touch_attr_swipe_debug.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_q_sensitivity.attr,
	&touch_attr_pinstate.attr,
	NULL,
};

static const struct attribute_group sw49106_attribute_group = {
	.attrs = sw49106_attribute_list,
};

static int sw49106_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &sw49106_attribute_group);
	if (ret < 0)
		TOUCH_E("sw49106 sysfs register failed\n");

	//sw49106_watch_register_sysfs(dev);
	sw49106_prd_register_sysfs(dev);
	sw49106_sic_abt_register_sysfs(&ts->kobj);

	return 0;
}

static int sw49106_get_cmd_version(struct device *dev, char *buf)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	int offset = 0;
	int ret = 0;
	u32 rdata[4] = {0};

	ret = sw49106_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf + offset, PAGE_SIZE - offset, "version : v%d.%02d\n",
		d->fw.version[0], d->fw.version[1]);

	if (d->fw.revision == 0xFF) {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"revision : Flash Erased(0xFF)\n");
	} else {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"revision : %d\n", d->fw.revision);
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
		"product id : [%s]\n\n", d->fw.product_id);

	sw49106_reg_read(dev, info_lot_num, (u8 *)&rdata, sizeof(rdata));
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "lot : %d\n", rdata[0]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "serial : 0x%X\n", rdata[1]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "date : 0x%X 0x%X\n",
		rdata[2], rdata[3]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset, "date : %04d.%02d.%02d " \
		"%02d:%02d:%02d Site%d\n",
		rdata[2] & 0xFFFF, (rdata[2] >> 16 & 0xFF), (rdata[2] >> 24 & 0xFF),
		rdata[3] & 0xFF, (rdata[3] >> 8 & 0xFF), (rdata[3] >> 16 & 0xFF),
		(rdata[3] >> 24 & 0xFF));

	return offset;
}

static int sw49106_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct sw49106_data *d = to_sw49106_data(dev);
	int offset = 0;
	int ret = 0;

	ret = sw49106_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf, PAGE_SIZE, "v%d.%02d\n",
		d->fw.version[0], d->fw.version[1]);

	return offset;
}

static int sw49106_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int sw49106_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = sw49106_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = sw49106_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static struct touch_driver touch_driver = {
	.probe = sw49106_probe,
	.remove = sw49106_remove,
	.shutdown = sw49106_shutdown,
	.suspend = sw49106_suspend,
	.resume = sw49106_resume,
	.init = sw49106_init,
	.irq_handler = sw49106_irq_handler,
	.power = sw49106_power,
	.upgrade = sw49106_upgrade,
	.lpwg = sw49106_lpwg,
	.notify = sw49106_notify,
	.register_sysfs = sw49106_register_sysfs,
	.set = sw49106_set,
	.get = sw49106_get,
};

#define MATCH_NAME			"lge,sw49106"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{ },
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();

	if (lge_get_panel_type() != LGD_INCELL_SW49106) {
		TOUCH_I("%s, sw49106 not found.\n", __func__);
		return 0;
	}

	TOUCH_I("%s, sw49106 found!\n", __func__);

	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("BSP-TOUCH@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
