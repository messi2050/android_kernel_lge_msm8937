/* touch_synaptics.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_td4100.h"
#include "touch_td4100_prd.h"

/*
 * PLG632 - SF3(LGD Panel)
 * PLG635 - LV7
 * PLG640 - LV5
 * PLG643 - SF3(TOVIS Panel)
 */

const char *f_str[] = {
	"ERROR",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME",
	"PALM_STATE",
	"ACTIVE_AREA"
};

static struct synaptics_rmidev_exp_fhandler rmidev_fhandler;

/* touch irq handle according to display power down(shutdown) in mfts */
bool mfts_check_shutdown = false;

bool synaptics_is_product(struct synaptics_data *d,
				const char *product_id, size_t len)
{
	return strncmp(d->ic_info.product_id, product_id, len)
			? false : true;
}

bool synaptics_is_img_product(struct synaptics_data *d,
				const char *product_id, size_t len)
{
	return strncmp(d->ic_info.img_product_id, product_id, len)
			? false : true;
}

int synaptics_read(struct device *dev, u8 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	return 0;
}

int synaptics_write(struct device *dev, u8 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size+1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		return ret;
	}

	return 0;
}

void synaptics_reset_ctrl(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 wdata = 0x01;

	TOUCH_TRACE();

	switch (ctrl) {
	default:
	case SW_RESET:
		TOUCH_I("%s : SW Reset\n", __func__);
		synaptics_write(dev, DEVICE_COMMAND_REG, &wdata, sizeof(u8));
		touch_msleep(ts->caps.sw_reset_delay);
		break;
	case HW_RESET:
		TOUCH_I("%s : HW Reset\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(1);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(ts->caps.hw_reset_delay);
		break;
	}
}

static int synaptics_abs_enable(struct device *dev, bool enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val;
	int ret;

	TOUCH_TRACE();

	ret = synaptics_read(dev, INTERRUPT_ENABLE_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read interrupt enable - ret:%d\n", ret);
		return ret;
	}

	if (enable)
		val |= INTERRUPT_MASK_ABS0;
	else
		val &= ~INTERRUPT_MASK_ABS0;


	ret = synaptics_write(dev, INTERRUPT_ENABLE_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write interrupt enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - enable:%d\n", __func__, enable);

	return 0;
}

static int synaptics_abs_check(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 val;
	int ret;

	ret = synaptics_read(dev, INTERRUPT_ENABLE_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read interrupt enable - ret:%d\n", ret);
		return ret;
	}
	TOUCH_D(ABS,"%s - ABS interrupt: %d\n", __func__,(val & INTERRUPT_MASK_ABS0));
	return (val & INTERRUPT_MASK_ABS0);
}

int synaptics_set_page(struct device *dev, u8 page)
{
	int ret = synaptics_write(dev, PAGE_SELECT_REG, &page, 1);

	if (ret >= 0)
		to_synaptics_data(dev)->curr_page = page;

	return ret;
}

static void synaptics_check_fail_reason(char* reason)
{
	int i = 0;

	TOUCH_TRACE();

	for (i = 0; i < MAX_NUM_OF_FAIL_REASON; i++) {
		switch (reason[i]) {
		case FAIL_DISTANCE_INTER_TAP:
			TOUCH_I("LPWG FAIL REASON = FAIL_DISTANCE_INTER_TAP\n");
			break;
		case FAIL_DISTANCE_TOUCHSLOP:
			TOUCH_I("LPWG FAIL REASON = FAIL_DISTANCE_TOUCHSLOP\n");
			break;
		case FAIL_TIMEOUT_INTER_TAP:
			TOUCH_I("LPWG FAIL REASON = FAIL_TIMEOUT_INTER_TAP\n");
			break;
		case FAIL_MULTI_FINGER:
			TOUCH_I("LPWG FAIL REASON = FAIL_MULTI_FINGER\n");
			break;
		case FAIL_DELAY_TIME:
			TOUCH_I("LPWG FAIL REASON = FAIL_DELAY_TIME\n");
			break;
		case FAIL_PALM_STATE:
			TOUCH_I("LPWG FAIL REASON = FAIL_PALM_STATE\n");
			break;
		case FAIL_ACTIVE_AREA:
			TOUCH_I("LPWG FAIL REASON = FAIL_ACTIVE_AREA\n");
			break;
		case FAIL_TAP_COUNT:
			TOUCH_I("LPWG FAIL REASON = FAIL_TAP_COUNT\n");
			break;
		default:
			TOUCH_I("LPWG FAIL REASON = Unknown Fail Reason\n");
			break;
		}
	}
}

static int synaptics_get_f12(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 query_5_data[5];
	u8 query_8_data[3];
	u8 ctrl_23_data[2];
	u8 ctrl_8_data[14];

	u32 query_5_present = 0;
	u16 query_8_present = 0;

	u8 offset;
	int i;
	int ret;

	TOUCH_TRACE();

	ret = synaptics_read(dev, d->f12.dsc.query_base + 5,
			     query_5_data, sizeof(query_5_data));

	if (ret < 0) {
		TOUCH_E("faied to get query5 (ret: %d)\n", ret);
		return ret;
	}

	query_5_present = (query_5_present << 8) | query_5_data[4];
	query_5_present = (query_5_present << 8) | query_5_data[3];
	query_5_present = (query_5_present << 8) | query_5_data[2];
	query_5_present = (query_5_present << 8) | query_5_data[1];
	TOUCH_I("qeury_5_present=0x%08X [%02X %02X %02X %02X %02X]\n",
			query_5_present, query_5_data[0], query_5_data[1],
			query_5_data[2], query_5_data[3], query_5_data[4]);

	for (i = 0, offset = 0; i < 32; i++) {
		d->f12_reg.ctrl[i] = d->f12.dsc.control_base + offset;

		if (query_5_present & (1 << i)) {
			TOUCH_I("f12_reg.ctrl[%d]=0x%02X (0x%02x+%d)\n",
					i, d->f12_reg.ctrl[i],
					d->f12.dsc.control_base, offset);
			offset++;
		}
	}

	ret = synaptics_read(dev, d->f12.dsc.query_base + 8,
			query_8_data, sizeof(query_8_data));

	if (ret < 0) {
		TOUCH_E("faied to get query8 (ret: %d)\n", ret);
		return ret;
	}

	query_8_present = (query_8_present << 8) | query_8_data[2];
	query_8_present = (query_8_present << 8) | query_8_data[1];
	TOUCH_I("qeury_8_present=0x%08X [%02X %02X %02X]\n",
			query_8_present, query_8_data[0],
			query_8_data[1], query_8_data[2]);

	for (i = 0, offset = 0; i < 16; i++) {
		d->f12_reg.data[i] = d->f12.dsc.data_base + offset;

		if (query_8_present & (1 << i)) {
			TOUCH_I("d->f12_reg.data[%d]=0x%02X (0x%02x+%d)\n",
					i, d->f12_reg.data[i],
					d->f12.dsc.data_base, offset);
			offset++;
		}
	}

	ret = synaptics_read(dev, d->f12_reg.ctrl[23],
			     ctrl_23_data, sizeof(ctrl_23_data));

	if (ret < 0) {
		TOUCH_E("faied to get f12_ctrl32_data (ret: %d)\n", ret);
		return ret;
	}

	d->object_report = ctrl_23_data[0];
	d->num_of_fingers = min_t(u8, ctrl_23_data[1], (u8) MAX_NUM_OF_FINGERS);

	TOUCH_I("object_report[0x%02X], num_of_fingers[%d]\n",
			d->object_report, d->num_of_fingers);

	ret = synaptics_read(dev, d->f12_reg.ctrl[8],
			     ctrl_8_data, sizeof(ctrl_8_data));

	if (ret < 0) {
		TOUCH_E("faied to get f12_ctrl8_data (ret: %d)\n", ret);
		return ret;
	}

	TOUCH_I("ctrl_8-sensor_max_x[%d], sensor_max_y[%d]\n",
			((u16)ctrl_8_data[0] << 0) |
			((u16)ctrl_8_data[1] << 8),
			((u16)ctrl_8_data[2] << 0) |
			((u16)ctrl_8_data[3] << 8));

	return 0;
}

static int synaptics_page_description(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	struct function_descriptor dsc;
	u8 page;

	unsigned short pdt;
	int ret;
	/* TD4100 fw recovery - slave addr change to 0x2c */
	struct i2c_client *client = to_i2c_client(dev);
	u32 backup_slave_addr = client->addr;

	TOUCH_TRACE();

	d->need_fw_recovery = false;
	memset(&d->f01, 0, sizeof(struct synaptics_function));
	memset(&d->f11, 0, sizeof(struct synaptics_function));
	memset(&d->f12, 0, sizeof(struct synaptics_function));
	memset(&d->f1a, 0, sizeof(struct synaptics_function));
	memset(&d->f34, 0, sizeof(struct synaptics_function));
	memset(&d->f51, 0, sizeof(struct synaptics_function));
	memset(&d->f54, 0, sizeof(struct synaptics_function));
	memset(&d->f55, 0, sizeof(struct synaptics_function));

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		ret = synaptics_set_page(dev, page);

		if (ret < 0) {
			TOUCH_E("faied to set page %d (ret: %d)\n", page, ret);
			/* TD4100 FW recovery - slave addr change to 0x2c */
			TOUCH_E("I2C error - change slave addr to 0x2c\n");
			client->addr = 0x2c;
			synaptics_set_page(dev, page);
		}

		for (pdt = PDT_START; pdt > PDT_END; pdt -= sizeof(dsc)) {
			ret = synaptics_read(dev, pdt, &dsc, sizeof(dsc));

			if (!dsc.fn_number)
				break;

			TOUCH_I("dsc - %02x, %02x, %02x, %02x, %02x, %02x\n",
				dsc.query_base, dsc.command_base,
				dsc.control_base, dsc.data_base,
				dsc.int_source_count, dsc.fn_number);

			switch (dsc.fn_number) {
			case 0x01:
				d->f01.dsc = dsc;
				d->f01.page = page;
				break;

			case 0x11:
				d->f11.dsc = dsc;
				d->f11.page = page;
				break;

			case 0x12:
				d->f12.dsc = dsc;
				d->f12.page = page;
				synaptics_get_f12(dev);
				break;

			case 0x1a:
				d->f1a.dsc = dsc;
				d->f1a.page = page;
				break;

			case 0x34:
				d->f34.dsc = dsc;
				d->f34.page = page;
				break;

			/* TD4100 FW recovery Added */
			case 0x35:
				TOUCH_E("F35 detected - Need FW recovery\n");
				d->need_fw_recovery = true;
				break;

			case 0x51:
				d->f51.dsc = dsc;
				d->f51.page = page;
				break;

			case 0x54:
				d->f54.dsc = dsc;
				d->f54.page = page;
				break;

			default:
				break;
			}
		}
	}

	client->addr = backup_slave_addr;

	TOUCH_I("common[%dP:0x%02x] finger_f12[%dP:0x%02x] flash[%dP:0x%02x] analog[%dP:0x%02x] lpwg[%dP:0x%02x]\n",
		d->f01.page, d->f01.dsc.fn_number,
		d->f12.page, d->f12.dsc.fn_number,
		d->f34.page, d->f34.dsc.fn_number,
		d->f54.page, d->f54.dsc.fn_number,
		d->f51.page, d->f51.dsc.fn_number);

	if (!(d->f01.dsc.fn_number &&
	      d->f12.dsc.fn_number &&
	      d->f34.dsc.fn_number &&
	      d->f54.dsc.fn_number &&
	      d->f51.dsc.fn_number))
		return -EINVAL;

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret) {
		TOUCH_E("failed to set page %d (ret: %d)\n", 0, ret);
		return ret;
	}

	return 0;
}

int TD4100_ForceUpdate(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 data = 0;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("synaptics_set_page error\n");
		goto FAIL;
	}

	data = 0x04; /* Force Update */
	ret = synaptics_write(dev, ANALOG_COMMAND_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("ANALOG_COMMAND_REG write error\n");
		goto FAIL;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("synaptics_set_page set error\n");
		goto FAIL;
	}

	return ret;

FAIL:
	TOUCH_E("TD4100_ForceUpdate error !!!\n");
	synaptics_set_page(dev, DEFAULT_PAGE);
	return ret;
}

static int synaptics_get_product_id(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	ret = synaptics_read(dev, PRODUCT_ID_REG,
			d->ic_info.product_id, 6);

	if (ret < 0) {
		TOUCH_I("[%s] PRODUCT_ID_REG read error...\n", __func__);
		return ret;
	}

	TOUCH_I("[%s] IC_product_id: %s\n",
			__func__, d->ic_info.product_id);

	return 0;
}

static void synaptics_clear_palm_filter_data(struct synaptics_data *d)
{
	TOUCH_TRACE();

	if (!(synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6)))
		return;

	TOUCH_I("%s : clear palm_mask & palm_point\n", __func__);

	d->palm_mask = 0;
	memset(d->palm_point, 0, sizeof(struct point) * MAX_NUM_OF_FINGERS);
	return;
}

static void synaptics_palm_filter(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u16 old_mask = ts->old_mask;
	u16 new_mask = ts->new_mask;
	u16 temp_mask = new_mask;
	u16 change_mask = old_mask ^ new_mask;
	u16 press_mask = new_mask & change_mask;
	u16 release_mask = old_mask & change_mask;
	uint delay = 50;
	uint pen_z = 20;
	uint small_finger_z = 30;
	uint palm_z = 255;
	uint palm_w = 20;
	uint palm_dst = 150;
	uint jump_dst = 375;
	int dx = 0;
	int dy = 0;
	int i = 0;
	int j = 0;

	TOUCH_TRACE();

	if (!(synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6)) || !d->use_palm_filter)
		return;

	/* check jumpy ghost */
	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if ((new_mask & (1 << i)) &&
				(old_mask & (1 << i)) &&
				(ts->tdata[i].pressure == pen_z)) {
			dx = ts->tdata[i].x - d->old_tdata[i].x;
			dy = ts->tdata[i].y - d->old_tdata[i].y;

			if ((uint)((dx * dx) + (dy * dy)) >
					(jump_dst * jump_dst)) {
				TOUCH_D(ABS, "%s : id[%d](%4d,%4d,%4d) is jumpy ghost - old_tdata[%d](%4d,%4d,%4d)(d:%4d)\n",
						__func__,
						i,
						ts->tdata[i].x,
						ts->tdata[i].y,
						ts->tdata[i].pressure,
						i,
						d->old_tdata[i].x,
						d->old_tdata[i].y,
						d->old_tdata[i].pressure,
						jump_dst);

				memcpy(&(ts->tdata[i]), &(d->old_tdata[i]),
						sizeof(struct touch_data));
				temp_mask &= ~(1 << i);
				TOUCH_D(ABS, "%s: new_mask = 0x%04X , temp_mask = 0x%04X\n",
						__func__, new_mask, temp_mask);
			}
		}
	}

	if (temp_mask != new_mask) {
		ts->new_mask = temp_mask;
		new_mask = ts->new_mask;
		change_mask = old_mask ^ new_mask;
		press_mask = new_mask & change_mask;
		release_mask = old_mask & change_mask;
	}

	/* check pen z value change */
	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if ((new_mask & (1 << i)) &&
				(ts->tdata[i].pressure == palm_z) &&
				(d->old_tdata[i].pressure == pen_z)) {
			TOUCH_D(ABS, "%s : id[%d](%4d,%4d,%4d) is changed to small finger - old_tdata[%d](%4d,%4d,%4d)\n",
					__func__,
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure,
					i,
					d->old_tdata[i].x,
					d->old_tdata[i].y,
					d->old_tdata[i].pressure);

			ts->tdata[i].pressure = small_finger_z;
		}
	}

	/* clear all palm_mask when tcount is 0 */
	if ((ts->tcount == 0) && (d->palm_mask)) {
		mod_delayed_work(ts->wq, &d->palm_filter_work,
				msecs_to_jiffies(delay));
		TOUCH_I("%s : tcount is 0, clear all palm_mask after %ums\n",
				__func__, delay);
	}

	/* update palm_mask */
	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if ((press_mask & (1 << i)) && (d->palm_mask & (1 << i)) &&
				((ts->tdata[i].width_major < palm_w) ||
				(ts->tdata[i].pressure < palm_z))) {
			dx = ts->tdata[i].x - d->palm_point[i].x;
			dy = ts->tdata[i].y - d->palm_point[i].y;

			if ((uint)((dx * dx) + (dy * dy)) >
					(palm_dst * palm_dst)) {
				d->palm_mask &= ~(1 << i);
				TOUCH_I("%s : prev palm region[%d](%4d,%4d)(d:%4d)\n",
						__func__,
						i,
						d->palm_point[i].x,
						d->palm_point[i].y,
						palm_dst);
				TOUCH_I("%s : id[%d](%4d,%4d,%4d) is not palm - palm_mask id[%d] is cleared! palm_mask = 0x%04X\n",
						__func__,
						i,
						ts->tdata[i].x,
						ts->tdata[i].y,
						ts->tdata[i].pressure,
						i,
						d->palm_mask);
			}
		}

		if ((new_mask & (1 << i)) && !(d->palm_mask & (1 << i)) &&
				((ts->tdata[i].width_major >= palm_w) ||
				(ts->tdata[i].pressure >= palm_z))) {
			d->palm_mask |= (1 << i);
			TOUCH_I("%s : id[%d](%4d,%4d,%4d) is palm - palm_mask id[%d] set! palm_mask = 0x%04X\n",
					__func__,
					i,
					ts->tdata[i].x,
					ts->tdata[i].y,
					ts->tdata[i].pressure,
					i,
					d->palm_mask);
		}
	}

	/* update palm_point */
	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if ((new_mask & (1 << i)) && (d->palm_mask & (1 << i)) &&
				((ts->tdata[i].width_major >= palm_w) ||
				(ts->tdata[i].pressure >= palm_z))) {
			d->palm_point[i].x = ts->tdata[i].x;
			d->palm_point[i].y = ts->tdata[i].y;
		}
	}

	/* change palm area touch z value & cancel clearing palm_mask */
	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if (new_mask & (1 << i)) {
			for (j = 0; j < MAX_NUM_OF_FINGERS; j++) {
				if (d->palm_mask & (1 << j)) {
					dx = ts->tdata[i].x -
						d->palm_point[j].x;
					dy = ts->tdata[i].y -
						d->palm_point[j].y;

					if ((uint)((dx * dx) + (dy * dy)) <
							(palm_dst * palm_dst)) {
						ts->tdata[i].pressure = 255;

						TOUCH_D(ABS, "%s : id[%d](%4d,%4d) is in palm region[%d](%4d,%4d)(d:%4d)\n",
								__func__,
								i,
								ts->tdata[i].x,
								ts->tdata[i].y,
								j,
								d->palm_point[j].x,
								d->palm_point[j].y,
								palm_dst);

						if (delayed_work_pending(&d->palm_filter_work)) {
							cancel_delayed_work_sync(&d->palm_filter_work);
							TOUCH_I("%s; cancel clearing palm_mask\n",
									__func__);
						}
					}
				}
			}
		}
	}

	/* store old_tdata */
	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if ((new_mask & (1 << i))) {
			memcpy(&(d->old_tdata[i]), &(ts->tdata[i]),
					sizeof(struct touch_data));
		} else {
			memset(&(d->old_tdata[i]), 0,
					sizeof(struct touch_data));
		}
	}
	return;
}

static int synaptics_update_pen_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int qmemo_status = atomic_read(&ts->state.qmemo);
	int pmemo_status = atomic_read(&ts->state.pmemo);
	int ime_status = atomic_read(&ts->state.ime);
	int fb_status = atomic_read(&ts->state.fb);
	bool enable = true;
	u8 buffer = 0;
	u8 pen_mode_ctrl_reg = ESD_GENERAL_CTRL_REG;
	u8 pen_mode_flag = 0x08;
	int debug_option_mask = atomic_read(&ts->state.debug_option_mask);
	u16 palm_filter_enable_flag = 0x0001;
	bool palm_filter_enable = false;

	TOUCH_TRACE();

	if (!(synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6))) {
		TOUCH_I("%s : skip pen mode reg ctrl\n", __func__);
		return ret;
	}

	if (enable && (qmemo_status == QMEMO_NOT_RUNNING) &&
			(pmemo_status == PMEMO_NOT_RUNNING)) {
		TOUCH_I("%s : QMEMO/PMEMO is not running, disable pen mode\n", __func__);
		enable = false;
	}

	if (enable && (ime_status != IME_OFF)) {
		TOUCH_I("%s : IME ON, disable pen mode\n", __func__);
		enable = false;
	}

	if (enable && (fb_status == FB_SUSPEND)) {
		TOUCH_I("%s : FB SUSPEND, disable pen mode\n", __func__);
		enable = false;
	}

	ret = synaptics_set_page(dev, LPWG_PAGE);
	if (ret < 0) {
		TOUCH_E("Write LPWG_PAGE error\n");
		return ret;
	}

	ret = synaptics_read(dev, pen_mode_ctrl_reg, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read pen_mode_ctrl_reg - ret:%d\n", ret);
		return ret;
	}

	if (enable)
		buffer |= pen_mode_flag;
	else
		buffer &= ~pen_mode_flag;

	ret = synaptics_write(dev, pen_mode_ctrl_reg, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to write pen_mode_ctrl_reg- ret:%d\n", ret);
		return ret;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("Write DEFAULT_PAGE error\n");
		return ret;
	}

	TOUCH_I("%s : pen mode %s\n", __func__, (enable ? "ON" : "OFF"));

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		debug_option_mask |= palm_filter_enable_flag;
		atomic_set(&ts->state.debug_option_mask, debug_option_mask);
	}

	palm_filter_enable = ((debug_option_mask & palm_filter_enable_flag)
			&& enable);

	TOUCH_I("%s : palm filter %s\n", __func__,
			(palm_filter_enable ? "ON" : "OFF"));
	synaptics_clear_palm_filter_data(d);
	d->use_palm_filter = palm_filter_enable;

	return ret;
}

static int synaptics_change_ime_status(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ime_status = 0;
	u8 buffer[18] = {0,};
	u8 buf[7] = {0,};

	ime_status = atomic_read(&ts->state.ime);

	TOUCH_I("[IME_STATUS] : %d\n", ime_status);

	ret = synaptics_read(dev, d->f12_reg.ctrl[10], buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read i2c - ret:%d\n", ret);
		return ret;
	}

	if (ime_status) {
		buffer[3] = IME_DRUMMING_ACCEL_TH;
	} else {
		if (synaptics_is_product(d, "PLG640", 6))
			buffer[3] = DRUMMING_ACCEL_TH - 0x04;
		else
			buffer[3] = DRUMMING_ACCEL_TH;
	}

	ret = synaptics_write(dev, d->f12_reg.ctrl[10], buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to write i2c - ret:%d\n", ret);
		return ret;
	}

	if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6)) { /* Use only SF3 */
		ret = synaptics_read(dev, d->f12_reg.ctrl[15], buf, sizeof(buf));
		if (ret < 0) {
			TOUCH_E("failed to read i2c - ret:%d\n", ret);
			return ret;
		}
		if (ime_status)
			buf[1] = IME_SMALL_FINGER_AMPLITUDE_TH;
		else
			buf[1] = SMALL_FINGER_AMPLITUDE_TH;

		ret = synaptics_write(dev, d->f12_reg.ctrl[15], buf, sizeof(buf));
		if (ret < 0) {
			TOUCH_E("failed to write i2c - ret:%d\n", ret);
			return ret;
		}
	}

	return ret;
}

static int synaptics_change_landing_frame(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 buffer[19] = {0,};

	TOUCH_TRACE();

	ret = synaptics_read(dev, d->f12_reg.ctrl[11], buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read i2c - ret:%d\n", ret);
		return ret;
	}

	buffer[8] = FINGER_LANDING_MAX_FRAME_LPWG;	// lpwg landing max frame is '1'

	ret = synaptics_write(dev, d->f12_reg.ctrl[11], buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to write i2c - ret:%d\n", ret);
		return ret;
	}

	return ret;
}

static int synaptics_closed_cover_set(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 buffer;

	TOUCH_TRACE();

	if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6))	/* SF3 don't use closed cover */
		return ret;

	ret = synaptics_read(dev, d->f12_reg.ctrl[26], &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read i2c - ret:%d\n", ret);
		return ret;
	}

	if (ts->lpwg.qcover)
		buffer |= 0x02;
	else
		buffer &= 0xFD;

	ret = synaptics_write(dev, d->f12_reg.ctrl[26], &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read i2c - ret:%d\n", ret);
		return ret;
	}

	return ret;
}

int synaptics_ic_info(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);
	u8 status = 0;
	int ret;

	TOUCH_TRACE();

	if (d->need_scan_pdt == true) {
		ret = synaptics_page_description(dev);
		if (ret < 0) {
			TOUCH_E("firmware page description read error... : (%d)\n", ret);
			if (d->need_fw_recovery) {
				TOUCH_I("Need to FW Recovery. Keep initialize\n");
				goto skip_error;
			}
			TOUCH_E("F35 was not detected. Check Device status\n");
			ret = synaptics_read(dev, DEVICE_STATUS_REG, &status, sizeof(status));
			if((status & 0xC0) == 0x40) {  // bit 6 : check bootloader mode
				TOUCH_E("[0x%x] bootloader mode. Keep initialize\n", status);
				goto skip_error;
			}
			TOUCH_E("Touch IC(TD4100) is no response\n");
			d->need_scan_pdt = false;
			return ret;
		}
		/* After prd.c porting
		SCAN_PDT(dev);
		*/
	}

skip_error: /*in order to execute fw upgrade function*/

	d->need_scan_pdt = false;

	synaptics_get_product_id(dev);
	ret = synaptics_read(dev, FLASH_CONFIG_ID_REG,
			d->ic_info.raws, sizeof(d->ic_info.raws));
	if (ret < 0)
		TOUCH_E("Customer Defined Config ID read error : %d\n", ret);
	ret = synaptics_read(dev, CUSTOMER_FAMILY_REG,
			&(d->ic_info.family), sizeof(d->ic_info.family));
	if (ret < 0)
		TOUCH_E("Customer Family Query read error : %d\n", ret);
	ret = synaptics_read(dev, FW_REVISION_REG,
			&(d->ic_info.revision), sizeof(d->ic_info.revision));
	if (ret < 0)
		TOUCH_E("Firmware Revision Query read error : %d\n", ret);
	if(atomic_read(&ts->state.ime))
		ret = synaptics_change_ime_status(dev);
	synaptics_closed_cover_set(dev);

	d->ic_info.version.major = (d->ic_info.raws[3] & 0x80 ? 1 : 0);
	d->ic_info.version.minor = (d->ic_info.raws[3] & 0x7F);

	TOUCH_I("=======================\n");
	TOUCH_I(" IC_Version = v%d.%02d\n", d->ic_info.version.major, d->ic_info.version.minor);
	TOUCH_I(" Customer Family = %d\n", d->ic_info.family);
	TOUCH_I(" F/W Revision = %d\n", d->ic_info.revision);
	TOUCH_I(" Product ID = %s\n", d->ic_info.product_id);
	TOUCH_I("=======================\n");

	return 0;
}

static int synaptics_set_configured(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 dev_status = 0;
	u8 dev_ctrl_data =0;
	int ret = 0;

	ret = synaptics_read(dev, DEVICE_STATUS_REG, &dev_status, sizeof(dev_status));
	ret |= synaptics_read(dev, DEVICE_CONTROL_REG, &dev_ctrl_data, sizeof(dev_ctrl_data));
	if (ret < 0) {
		TOUCH_E("failed to read device status - ret:%d\n", ret);
		return ret;
	}

	if (dev_status == 0x00) {
		TOUCH_I("[%s] Already set configured. DCR bit : 0x%x\n", __func__, dev_ctrl_data);
		return 0;
	} else if (dev_status != 0x81) {
		TOUCH_E("dev_status is 0x%x\n", dev_status);
	}

	/* Display Reset dev_status 0x81 -> Set configured */
	if (dev_status == 0x81) {
		TOUCH_I("Set configured. dev_status : 0x%x\n", dev_status);
		dev_ctrl_data = 0x80; // Set Configured
		ret = synaptics_write(dev, DEVICE_CONTROL_REG, &dev_ctrl_data, sizeof(dev_ctrl_data));
		ret |= synaptics_read(dev, DEVICE_STATUS_REG, &dev_status, sizeof(dev_status));
		if (ret < 0) {
			TOUCH_E("Set configured error\n");
			return ret;
		}
		TOUCH_I("[%s] device_status bit cleared : 0x%x\n", __func__, dev_status);
	}

	return 0;
}

static int synaptics_sleep_control(struct device *dev, u8 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 val;
	int ret;

	TOUCH_TRACE();

	ret = synaptics_read(dev, DEVICE_CONTROL_REG, &val, sizeof(val));

	if (ret < 0) {
		TOUCH_E("failed to read finger report enable - ret:%d\n", ret);
		return ret;
	}

	val &= 0xf8;

	if (mode) {
		val |= 1;
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
	} else {
		val |= 0;
		atomic_set(&ts->state.sleep, IC_NORMAL);
	}

	ret = synaptics_write(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write finger report enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - mode:%d\n", __func__, mode);

	return 0;
}

static int synaptics_lpwg_debug(struct device *dev, int num)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 count = 0;
	u8 index = 0;
	u8 buf = 0;
	u8 i = 0;
	u8 addr = 0;
	u8 offset = num ? LPWG_MAX_BUFFER + 2 : 0;

	TOUCH_TRACE();

	synaptics_set_page(dev, LPWG_PAGE);

	synaptics_read(dev, LPWG_TCI1_FAIL_COUNT_REG + offset, &count, sizeof(count));
	synaptics_read(dev, LPWG_TCI1_FAIL_INDEX_REG + offset, &index, sizeof(index));

	for (i = 1; i <= count; i++) {
		addr = LPWG_TCI1_FAIL_BUFFER_REG + offset +
			((index + LPWG_MAX_BUFFER - i) % LPWG_MAX_BUFFER);
		synaptics_read(dev, addr, &buf, sizeof(buf));
		TOUCH_I("TCI(%d)-Fail[%d/%d] : %s\n", num, count - i + 1, count,
			(buf > 0 && buf < 8) ? f_str[buf] : f_str[0]);

		if (i == LPWG_MAX_BUFFER)
			break;
	}

	synaptics_set_page(dev, DEFAULT_PAGE);

	return 0;
}

/*
 *  Fail Reason
 *  Error Type			value
 *  1 Distance_Inter_Tap	(1U << 0)
 *  2 Distance TouchSlop	(1U << 1)
 *  3 Timeout Inter Tap		(1U << 2)
 *  4 Multi Finger			(1U << 3)
 *  5 Delay Time			(1U << 4)
 *  6 Palm State			(1U << 5)
 *  7 Active Area			(1U << 6)
 *  8 Tap Count			(1U << 7)
 */
static int synaptics_lpwg_fail_control(struct device *dev, u16 value)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 buffer[2] = {0};
	int ret = 0;

	TOUCH_TRACE();

	synaptics_set_page(dev, LPWG_PAGE);

	ret = synaptics_read(dev, LPWG_FAIL_INT_ENABLE_REG, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read LPWG_FAIL_INT_ENABLE_REG - ret:%d\n", ret);
		goto error;
	}
	buffer[0] = (value >> 8) & 0xFF;
	buffer[1] = value & 0xFF;
	if (synaptics_write(dev, LPWG_FAIL_INT_ENABLE_REG, buffer, sizeof(buffer)) < 0) {
		TOUCH_I("LPWG_FAIL_INT_ENABLE_REG write error\n");
		goto error;
	} else {
		TOUCH_I("LPWG_FAIL_INT_ENABLE_REG write success\n");
	}

	synaptics_set_page(dev, DEFAULT_PAGE);

	return 0;
error:
	TOUCH_E("%s, %d : LPWG fail interrupt control error\n", __func__, __LINE__);
	synaptics_set_page(dev, DEFAULT_PAGE);
	return 0;
}

static int synaptics_tci_report_enable(struct device *dev, bool enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val[3];
	int ret;

	TOUCH_TRACE();

	ret = synaptics_read(dev, FINGER_REPORT_REG, val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read finger report enable - ret:%d\n", ret);
		return ret;
	}

	val[2] &= 0xfc;

	if (enable)
		val[2] |= 0x2;

	ret = synaptics_write(dev, FINGER_REPORT_REG, val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write finger report enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - enable:%d\n", __func__, enable);

	return 0;
}

static void synaptics_sensing_cmd(struct device *dev, int type)
{
	TOUCH_TRACE();

	TOUCH_I("%s sensing type = %d\n", __func__, type);

	switch (type) {
	case ABS_STOP:
		synaptics_abs_enable(dev, false);
		break;
	case ABS_SENSING:
		synaptics_abs_enable(dev, true);
		break;
	case WAKEUP_STOP:
		synaptics_tci_report_enable(dev, false);
		break;
	case WAKEUP_SENSING:
		synaptics_abs_enable(dev, false);
		synaptics_tci_report_enable(dev, true);
		break;
	case PARTIAL_WAKEUP_SENSING:
		break;
	default:
		break;
	}
}

int synaptics_update_esd_mode(struct device *dev, int value)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 buffer = 0;
	u8 esd_detect_flag = 0x04;

	TOUCH_TRACE();

	if (!(synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6))) {
		/*Skip esd detect reg ctrl. only use SF3*/
		TOUCH_I("%s : skip esd detect reg ctrl\n", __func__);
		return ret;
	}

	ret = synaptics_set_page(dev, LPWG_PAGE);
	if (ret < 0) {
		TOUCH_E("Write LPWG_PAGE error\n");
		return ret;
	}

	ret = synaptics_read(dev, ESD_GENERAL_CTRL_REG, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read ESD_GENERAL_CTRL_REG - ret:%d\n", ret);
		return ret;
	}

	if (value) { /* ESD Enable */
		TOUCH_I("ESD Detect Control Enable!\n");
		buffer |= esd_detect_flag;
	} else { /* ESD Disable */
		TOUCH_I("ESD Detect Control Disable!\n");
		buffer &= ~esd_detect_flag;
	}

	ret = synaptics_write(dev, ESD_GENERAL_CTRL_REG, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to write ESD_GENERAL_CTRL_REG - ret:%d\n", ret);
		return ret;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("Write DEFAULT_PAGE error\n");
		return ret;
	}

	return ret;
}

static int synaptics_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct tci_info *info;

	u8 lpwg_data[7];
	int ret;
	u8 tci_reg[2] = {LPWG_TAPCOUNT_REG, LPWG_TAPCOUNT_REG2};
	int i = 0;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	for (i = 0; i < 2; i++) {
		if ((ts->tci.mode & (1 << i)) == 0x0) {

			TOUCH_I("[TCI %d]: 0\n",i+1);

			lpwg_data[0] = 0;
			synaptics_write(dev, tci_reg[i],
							lpwg_data, sizeof(u8));
		} else {
			info = &ts->tci.info[i];

			TOUCH_I("[TCI %d]: tap_count(%4d), intertap_time(%4d/%4d), touch_slop(%4d), tap_distance(%4d), intr_delay(%4d)\n",
					i+1, info->tap_count, info->min_intertap, info->max_intertap,
					info->touch_slop, info->tap_distance, info->intr_delay);

			lpwg_data[0] = ((info->tap_count << 3) | 1);
			lpwg_data[1] = info->min_intertap;
			lpwg_data[2] = info->max_intertap;
			lpwg_data[3] = info->touch_slop;
			lpwg_data[4] = info->tap_distance;
			lpwg_data[6] = (info->intr_delay << 1 | 1);
			synaptics_write(dev, tci_reg[i],
					lpwg_data, sizeof(lpwg_data));
		}
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	return ret;
}

static int synaptics_tci_password(struct device *dev)
{
	TOUCH_TRACE();
	return synaptics_tci_knock(dev);
}

static int synaptics_tci_active_area(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 buffer[8];

	TOUCH_TRACE();
	TOUCH_I("ACTIVE_AREA SET: x1[%4d], y1[%4d], x2[%4d], y2[%4d]\n",
		ts->tci.area.x1, ts->tci.area.y1, ts->tci.area.x2, ts->tci.area.y2);

	buffer[0] = (ts->tci.area.x1 >> 0) & 0xff;
	buffer[1] = (ts->tci.area.x1 >> 8) & 0xff;
	buffer[2] = (ts->tci.area.y1 >> 0) & 0xff;
	buffer[3] = (ts->tci.area.y1 >> 8) & 0xff;
	buffer[4] = (ts->tci.area.x2 >> 0) & 0xff;
	buffer[5] = (ts->tci.area.x2 >> 8) & 0xff;
	buffer[6] = (ts->tci.area.y2 >> 0) & 0xff;
	buffer[7] = (ts->tci.area.y2 >> 8) & 0xff;

	synaptics_write(dev, d->f12_reg.ctrl[18], buffer, sizeof(buffer));

	return 0;
}

static void synaptics_tci_area_set(struct device *dev, int cover_status)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (cover_status == QUICKCOVER_CLOSE) {
		ts->tci.area.x1 = 179;
		ts->tci.area.y1 = 144;
		ts->tci.area.x2 = 1261;
		ts->tci.area.y2 = 662;
		synaptics_tci_active_area(dev);
		TOUCH_I("LPWG Active Area - QUICKCOVER_CLOSE\n");
	} else {
		ts->tci.area.x1 = 60;
		ts->tci.area.y1 = 60;
		ts->tci.area.x2 = 660;
		ts->tci.area.y2 = 1220;
		synaptics_tci_active_area(dev);
		TOUCH_I("LPWG Active Area - NORMAL\n");
	}
}

static int synaptics_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];

	TOUCH_TRACE();

	TOUCH_I("%s : %d\n", __func__, mode);

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;
		info1->tap_distance = 10;
		synaptics_tci_knock(dev);
		synaptics_change_landing_frame(dev);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x03;
		info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
		info1->tap_distance = 7;
		synaptics_tci_password(dev);
		synaptics_change_landing_frame(dev);
		break;

	case LPWG_PASSWORD_ONLY:
		ts->tci.mode = 0x02;
		info1->intr_delay = 0;
		info1->tap_distance = 10;
		synaptics_tci_password(dev);
		synaptics_change_landing_frame(dev);
		break;

	default:
		ts->tci.mode = 0;
		synaptics_tci_knock(dev);
		break;
	}

	return 0;
}

static int synaptics_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	TOUCH_TRACE();

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			/* Force knock on set in minios mode */
			TOUCH_I("MiniOS boot mode. change lpwg.mode set : 1\n");
			ts->lpwg.mode = 1;
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_tci_area_set(dev, QUICKCOVER_OPEN);
			synaptics_lpwg_control(dev, ts->lpwg.mode);
			synaptics_sensing_cmd(dev, WAKEUP_SENSING);
			/* Skip ForceUpdate in miniOS */
			TOUCH_I("MiniOS boot mode. lpwg.mode restoration : 0\n");
			ts->lpwg.mode = 0;
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_sensing_cmd(dev, WAKEUP_STOP);
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			synaptics_lpwg_debug(dev, TCI_1);
			synaptics_lpwg_debug(dev, TCI_2);
		} else if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep\n",
					__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_lpwg_control(dev, LPWG_NONE);
			synaptics_sensing_cmd(dev, WAKEUP_SENSING);
			touch_msleep(100);
			synaptics_sleep_control(dev, IC_DEEP_SLEEP);
#if defined(CONFIG_LGE_DISPLAY_SF3_LGD_TD4100) /*only use SF3*/
			touch_msleep(20);
			if (atomic_read(&ts->state.incoming_call)) /* Ringing | Offhook */
				TOUCH_I("No DSV disable control for incoming call\n");
			else /* Idle */
				ext_dsv_mode_change(1); /*DSV_ENREG disable - Discharge*/
#endif
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
					__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_lpwg_control(dev, LPWG_NONE);
			synaptics_sensing_cmd(dev, WAKEUP_SENSING);
			touch_msleep(100);
			synaptics_sleep_control(dev, IC_DEEP_SLEEP);
#if defined(CONFIG_LGE_DISPLAY_SF3_LGD_TD4100) /*only use SF3*/
			touch_msleep(20);
			if (atomic_read(&ts->state.incoming_call)) /* Ringing | Offhook */
				TOUCH_I("No DSV disable control for incoming call\n");
			else /* Idle */
				ext_dsv_mode_change(1); /*DSV_ENREG disable - Discharge*/
#endif
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on */
			TOUCH_I("%s(%d) - knock on by hole\n",
				__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_tci_area_set(dev, QUICKCOVER_CLOSE);
			synaptics_lpwg_control(dev, LPWG_DOUBLE_TAP);
			synaptics_sensing_cmd(dev, WAKEUP_SENSING);
			TD4100_ForceUpdate(dev);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock mode %d, screen %d, proxy %d, qcover %d\n",
				__func__, __LINE__,
				ts->lpwg.mode, ts->lpwg.screen, ts->lpwg.sensor, ts->lpwg.qcover);

			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				synaptics_sleep_control(dev, IC_NORMAL);
#if defined(CONFIG_LGE_DISPLAY_SF3_LGD_TD4100) /*only use SF3*/
				ext_dsv_mode_change(0); /*DSV_ENREG enable - Charge*/
#endif
			}
			synaptics_tci_area_set(dev, QUICKCOVER_OPEN);
			synaptics_lpwg_control(dev, ts->lpwg.mode);

			if (d->lpwg_fail_reason)
				synaptics_lpwg_fail_control(dev, 0xFFFF);

			synaptics_sensing_cmd(dev, WAKEUP_SENSING);
			TD4100_ForceUpdate(dev);
		}
		return 0;
	}

	/* resume */
	touch_report_all_event(ts);
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("%s(%d) - normal\n",
				__func__, __LINE__);
		synaptics_lpwg_control(dev, LPWG_NONE);
		synaptics_sensing_cmd(dev, ABS_SENSING);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
		synaptics_lpwg_control(dev, LPWG_NONE);
		synaptics_sensing_cmd(dev, ABS_STOP);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		synaptics_lpwg_control(dev, LPWG_NONE);
		synaptics_sensing_cmd(dev, ABS_STOP);
	} else {
		/* partial */
		TOUCH_I("%s - partial is not ready\n", __func__);
		if (ts->lpwg.qcover == HOLE_NEAR)
			synaptics_tci_area_set(dev, QUICKCOVER_CLOSE);
		else
			synaptics_tci_area_set(dev, QUICKCOVER_OPEN);
		synaptics_lpwg_control(dev, LPWG_DOUBLE_TAP);
		synaptics_sensing_cmd(dev, PARTIAL_WAKEUP_SENSING);
	}

	return 0;
}

static void synaptics_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 0;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 0;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

static int synaptics_remove(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	TOUCH_TRACE();
	pm_qos_remove_request(&d->pm_qos_req);

	if (rmidev_fhandler.initialized
		&& rmidev_fhandler.insert) {
		rmidev_fhandler.exp_fn->remove(dev);
		rmidev_fhandler.initialized = false;
	}

	return 0;
}

static int synaptics_get_status(struct device *dev, u8 *device, u8 *interrupt)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 status[2] = {0, };
	int ret;

	TOUCH_TRACE();

	ret = synaptics_read(dev, DEVICE_STATUS_REG,
			&status, sizeof(status));

	if (ret < 0) {
		TOUCH_E("failed to read device status - ret:%d\n", ret);
		return ret;
	}

	TOUCH_D(TRACE, "status[device:%02x, interrupt:%02x]\n",
		status[0], status[1]);

	if (device)
		*device = status[0];

	if (interrupt)
		*interrupt = status[1];

	return ret;
}

static int synaptics_irq_clear(struct device *dev)
{

	TOUCH_TRACE();

	return synaptics_get_status(dev, NULL, NULL);

}

static int synaptics_noise_log(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 buffer[2] = {0};
	u8 buf_lsb = 0, buf_msb = 0, cns = 0;
	u16 im = 0, cid_im = 0, freq_scan_im = 0;

	TOUCH_TRACE();

	synaptics_set_page(dev, ANALOG_PAGE);
	synaptics_read(dev, INTERFERENCE_METRIC_LSB_REG, &buf_lsb, sizeof(buf_lsb));
	synaptics_read(dev, INTERFERENCE_METRIC_MSB_REG, &buf_msb, sizeof(buf_msb));

	im = (buf_msb << 8) | buf_lsb;
	d->noise.im_sum += im;

	synaptics_read(dev, CURRENT_NOISE_STATUS_REG, &cns, sizeof(cns));
	d->noise.cns_sum += cns;

	synaptics_read(dev, CID_IM_REG, buffer, sizeof(buffer));
	cid_im = (buffer[1] << 8) | buffer[0];
	d->noise.cid_im_sum += cid_im;

	synaptics_read(dev, FREQ_SCAN_IM_REG, buffer, sizeof(buffer));
	freq_scan_im = (buffer[1] << 8) | buffer[0];
	d->noise.freq_scan_im_sum += freq_scan_im;
	synaptics_set_page(dev, DEFAULT_PAGE);

	d->noise.cnt++;

	if (d->noise.noise_log == NOISE_ENABLE) {
		if (ts->old_mask != ts->new_mask) {
			TOUCH_I("Curr : CNS[%5d] IM[%5d] CID_IM[%5d] FREQ_SCAN_IM[%5d]\n",
					cns, im, cid_im, freq_scan_im);
		}
	}

	if (ts->new_mask == 0 || (d->noise.im_sum >= ULONG_MAX
				|| d->noise.cns_sum >= ULONG_MAX
				|| d->noise.cid_im_sum >= ULONG_MAX
				|| d->noise.freq_scan_im_sum >= ULONG_MAX
				|| d->noise.cnt >= UINT_MAX)) {
		if (d->noise.noise_log == NOISE_ENABLE) {
			TOUCH_I("Aver : CNS[%5lu] IM[%5lu] CID_IM[%5lu] FREQ_SCAN_IM[%5lu] (cnt:%u)\n",
				d->noise.cns_sum / d->noise.cnt,
				d->noise.im_sum / d->noise.cnt,
				d->noise.cid_im_sum / d->noise.cnt,
				d->noise.freq_scan_im_sum / d->noise.cnt,
				d->noise.cnt);
		}

		d->noise.im_avg = d->noise.im_sum / d->noise.cnt;
		d->noise.cns_avg = d->noise.cns_sum / d->noise.cnt;
		d->noise.cid_im_avg = d->noise.cid_im_sum / d->noise.cnt;
		d->noise.freq_scan_im_avg = d->noise.freq_scan_im_sum / d->noise.cnt;
	}

	if (ts->old_mask == 0 && ts->new_mask != 0) {
		d->noise.cnt = d->noise.im_sum = d->noise.cns_sum =
			d->noise.cid_im_sum = d->noise.freq_scan_im_sum = 0;
		d->noise.im_avg = d->noise.cns_avg =
			d->noise.cid_im_avg = d->noise.freq_scan_im_avg = 0;
	}

	return 0;
}

static int synaptics_get_object_count(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 object_to_read = d->num_of_fingers;
	u8 buf[2] = {0,};
	u16 object_attention = 0;
	int ret;

	TOUCH_TRACE();

	ret = synaptics_read(dev, d->f12_reg.data[15],
			(u8 *) buf, sizeof(buf));

	if (ret < 0) {
		TOUCH_E("%s, %d : get object_attention data failed\n",
			__func__, __LINE__);
		return ret;
	}

	object_attention = (((u16)((buf[1] << 8) & 0xFF00) | (u16)((buf[0])&0xFF)));

	for (; object_to_read > 0; object_to_read--) {
		if (object_attention & (0x1 << (object_to_read - 1)))
			break;
	}
	TOUCH_D(ABS, "object_to_read: %d\n", object_to_read);

	return object_to_read;
}

static int synaptics_irq_status(struct device *dev, u8 dev_status)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int mfts_mode = 0;

	TOUCH_TRACE();

	mfts_mode = touch_boot_mode_check(dev);

	/* check for set configured end of TD4100 initialized */
	if (atomic_read(&d->state.init) == INIT_END) {
		 /* ASIC Reset (Unknown Reset) Status */
		if (dev_status == 0x81) {
			/* Verify for ESD Recovery is in progress */
			if (atomic_read(&d->state.recovery) == ESD_RECOVERY_END)
				goto ESD_RECOVERY;
			else
				TOUCH_I("ESD Recovery is not over yet : 0x%x\n", dev_status);
		}
	}

	/* Abnormal Display Status */
	if ((dev_status & 0x0F) == 0x09) {
		/* Verify for ESD Recovery is in progress */
		if (atomic_read(&d->state.recovery) == ESD_RECOVERY_END)
			goto ESD_RECOVERY;
		else
			TOUCH_I("ESD Recovery is not over yet : 0x%x\n", dev_status);
	}

	return 0;

ESD_RECOVERY:
	if (mfts_mode >= MINIOS_MFTS_FOLDER) {
		TOUCH_I("[%s] MFTS boot mode. Do not carry out the recovery action (0x%x)\n", __func__, dev_status);
		return 0;
	}
	TOUCH_I("###### ESD Detected !!!(0x%x) - Call ESD Recovery Func ######\n", dev_status);
	atomic_set(&d->state.recovery, ESD_RECOVERY_RUN); /* ESD recovery operation Start Flag */
	return -ERESTART;
}

static int synaptics_irq_abs_data(struct device *dev, int object_to_read)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct synaptics_object objects[MAX_NUM_OF_FINGERS];
	struct synaptics_object *obj;
	struct touch_data *tdata;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	TOUCH_TRACE();

	ts->new_mask = 0;

	if (object_to_read == 0) {
		goto end;
	}

	ret = synaptics_read(dev, FINGER_DATA_REG,
			     objects, sizeof(*obj) * object_to_read);
	if (ret < 0) {
		TOUCH_E("faied to read finger data\n");
		goto end;
	}

	for (i = 0; i < object_to_read; i++) {
		obj = objects + i;

		if (obj->type == F12_NO_OBJECT_STATUS)
			continue;

		if (obj->type > F12_MAX_OBJECT)
			TOUCH_D(ABS, "id : %d, type : %d\n",
				i, obj->type);

		if (obj->type == F12_FINGER_STATUS || obj->type == F12_PALM_STATUS) {
			ts->new_mask |= (1 << i);
			tdata = ts->tdata + i;

			tdata->id = i;
			tdata->type = obj->type;
			tdata->x = obj->x_lsb | obj->x_msb << 8;
			tdata->y = obj->y_lsb | obj->y_msb << 8;
			tdata->pressure = obj->z;

			if (obj->wx > obj->wy) {
				tdata->width_major = obj->wx;
				tdata->width_minor = obj->wy;
				tdata->orientation = 0;
			} else {
				tdata->width_major = obj->wy;
				tdata->width_minor = obj->wx;
				tdata->orientation = 1;
			}

			finger_index++;

#if defined(CONFIG_LGE_TOUCH_HALL_IC_COVER)
			if(synaptics_is_product(d, "PLG635", 6) ||
					synaptics_is_product(d, "PLG640", 6)) {
				if (cradle_smart_cover_status() && ts->use_qcover) {
					if (tdata->y > 265) {
						TOUCH_I("Skip irq while quickcover is closed!\n");
						ts->intr_status = TOUCH_IRQ_NONE;
						return ret;
					}
				}
			}
#endif
			TOUCH_D(ABS,
				"tdata [id:%2d t:%d x:%4d y:%4d z:%3d-%3d,%3d,%3d]\n",
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

	if (d->noise.check_noise == NOISE_ENABLE
		|| d->noise.noise_log == NOISE_ENABLE) {
		synaptics_noise_log(dev);
	}

end:
	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	if ((synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6)) && d->use_palm_filter)
		synaptics_palm_filter(dev);

	return ret;
}

static int synaptics_irq_abs(struct device *dev)
{
	u8 object_to_read = 0;
	int ret = 0;

	TOUCH_TRACE();

	object_to_read = synaptics_get_object_count(dev);

	if (object_to_read < 0) {
		TOUCH_E("faied to read object count\n");
		return ret;
	}

	return synaptics_irq_abs_data(dev, object_to_read);
}

static int synaptics_tci_getdata(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u32 buffer[12];
	int i = 0;
	int ret;

	TOUCH_TRACE();

	ts->lpwg.code_num = count;

	if (!count)
		return 0;

	ret = synaptics_read(dev, LPWG_DATA_REG,
					 buffer, sizeof(u32) * count);

	if (ret < 0)
		return ret;

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = buffer[i] & 0xffff;
		ts->lpwg.code[i].y = (buffer[i] >> 16) & 0xffff;

		/* temp - there is not overtap point */
		if (buffer[i] == 0) {
			ts->lpwg.code[i].x = 1;
			ts->lpwg.code[i].y = 1;
		}

		if ((ts->lpwg.mode >= LPWG_PASSWORD) &&
				(ts->role.hide_coordinate))
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int synaptics_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 status;
	u8 buffer;
	u8 fail_buffer;
	char reason[NUM_OF_EACH_FINGER_DATA];
	int ret;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0) {
		synaptics_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	ret = synaptics_read(dev, LPWG_STATUS_REG, &status, 1);
	if (ret < 0) {
		synaptics_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	if (status & LPWG_STATUS_DOUBLETAP) {
		synaptics_tci_getdata(dev, ts->tci.info[TCI_1].tap_count);
		ts->intr_status = TOUCH_IRQ_KNOCK;
	} else if (status & LPWG_STATUS_PASSWORD) {
		synaptics_tci_getdata(dev, ts->tci.info[TCI_2].tap_count);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if(d->lpwg_fail_reason) {
		TOUCH_I("LPWG Fail Detected\n");
		synaptics_read(dev, LPWG_FAIL_REASON_REALTIME_INT, &fail_buffer, 1);
		reason[0] = fail_buffer & 0x0F;
		reason[1] = (fail_buffer & 0xF0) >> 4;
		if (reason != NULL) {
			TOUCH_I("Fail-Reason TCI1 : [%d], TCI2 : [%d]\n", reason[0], reason[1]);
			synaptics_check_fail_reason(reason);
		} else {
			TOUCH_E("LPWG Real-Time-Interrupt fail buffer is NULL\n");
		}
	} else {
		/* Overtab */
		synaptics_read(dev, LPWG_OVER_TAPCOUNT, &buffer, 1);
		if (buffer > ts->tci.info[TCI_2].tap_count) {
			synaptics_tci_getdata(dev, ts->tci.info[TCI_2].tap_count + 1);
			ts->intr_status = TOUCH_IRQ_PASSWD;
			TOUCH_I("knock code fail to over tap count = %d\n", buffer);
		}
	}

	return synaptics_set_page(dev, DEFAULT_PAGE);
}

static int synaptics_irq_handler(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 irq_status;
	u8 dev_status;
	int ret = 0;
	int retry = 0;
	int mfts_mode = 0;

	TOUCH_TRACE();

	mfts_mode = touch_boot_mode_check(dev);

	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		if (mfts_check_shutdown) {
			TOUCH_I("Unknown IRQ. Do not handling\n");
			return ret;
		}
	}

	pm_qos_update_request(&d->pm_qos_req, 10);

	do {
		ret = synaptics_get_status(dev, &dev_status, &irq_status);
		if (ret < 0) {
			TOUCH_E("synaptics get status error : %d [retry : %d]\n", ret, retry+1);
			touch_msleep(30);
		} else {
			break;
		}
	} while((retry++) < 5);

	if (retry >= 5) {
		TOUCH_E("abnormal device status\n");
		return ret;
	}

	ret = synaptics_irq_status(dev, dev_status);
	if (ret == 0) {
		if (irq_status & INTERRUPT_MASK_ABS0){
			ret = synaptics_irq_abs(dev);
		} else if (irq_status & INTERRUPT_MASK_LPWG) {
			ret = synaptics_irq_lpwg(dev);
		}
	}
	pm_qos_update_request(&d->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	return ret;
}

void synaptics_rmidev_function(struct synaptics_rmidev_exp_fn *exp_fn,
		bool insert)
{
	TOUCH_TRACE();

	rmidev_fhandler.insert = insert;

	if (insert) {
		rmidev_fhandler.exp_fn = exp_fn;
		rmidev_fhandler.insert = true;
		rmidev_fhandler.remove = false;
	} else {
		rmidev_fhandler.exp_fn = NULL;
		rmidev_fhandler.insert = false;
		rmidev_fhandler.remove = true;
	}

	return;
}

static int synaptics_rmidev_init(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	if (rmidev_fhandler.insert) { //TODO DEBUG_OPTION_2
		ret = rmidev_fhandler.exp_fn->init(dev);

		if (ret < 0) {
			TOUCH_I("%s : Failed to init rmi_dev settings\n", __func__);
		} else {
			rmidev_fhandler.initialized = true;
		}
	}

	return 0;
}

int synaptics_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&d->state.init) == INIT_END)
		atomic_set(&d->state.init, INIT_RUN);

	ret = synaptics_ic_info(dev);
	if (ret < 0) {
		TOUCH_E("synaptics_ic_info failed %d\n", ret);
		return ret;
	}
	synaptics_set_configured(dev);
	synaptics_abs_enable(dev, true);
	atomic_set(&ts->state.sleep, IC_NORMAL);

	synaptics_lpwg_mode(dev);
	synaptics_irq_clear(dev);
	synaptics_rmidev_init(dev);
	atomic_set(&d->state.init, INIT_END);
	atomic_set(&d->state.recovery, ESD_RECOVERY_END);

	if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6))
		synaptics_update_pen_mode(dev);

	return 0;
}

static int synaptics_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		if (atomic_read(&ts->state.core) == CORE_PROBE) {
			TOUCH_I("%s, off\n", __func__);
			touch_gpio_direction_output(ts->reset_pin, 0);
			touch_power_vio(dev, 0);
			touch_power_vdd(dev, 0);
			touch_msleep(1);
		}
		break;

	case POWER_ON:
		if (atomic_read(&ts->state.core) == CORE_PROBE) {
			TOUCH_I("%s, on\n", __func__);
			touch_power_vdd(dev, 1);
			touch_power_vio(dev, 1);
			touch_gpio_direction_output(ts->reset_pin, 1);
		}
		break;

	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	}

	return 0;
}

static int synaptics_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int mfts_mode = 0;

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		return -EPERM;
	}

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		mfts_check_shutdown = false;
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		return 0;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
	}

	/* Clear Interrupt status register if Interrupt is disabled */
	if (!synaptics_abs_check(dev))
		synaptics_irq_clear(dev);

	 /* Exception handling for Display On(0x81) packet. */
	if (atomic_read(&d->state.init) == INIT_END)
		atomic_set(&d->state.init, INIT_RUN);

	if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6)) {
		synaptics_update_esd_mode(dev, 0);
		synaptics_update_pen_mode(dev);
	}

	synaptics_lpwg_mode(dev);

	return 0;
}

static int synaptics_resume(struct device *dev)
{
	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		/* Deep Sleep */
		synaptics_sleep_control(dev, 1);
		return -EPERM;
	}

	touch_interrupt_control(dev, INTERRUPT_ENABLE);

	return 0;
}

static int synaptics_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int *value = (int *)param;

	TOUCH_TRACE();

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
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];
		TOUCH_I("LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg.mode,
			ts->lpwg.screen ? "ON" : "OFF",
			ts->lpwg.sensor ? "FAR" : "NEAR",
			ts->lpwg.qcover ? "CLOSE" : "OPEN");
		synaptics_lpwg_mode(dev);
		synaptics_closed_cover_set(dev);
		break;
	}

	return 0;
}

static int synaptics_bin_fw_version(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	const struct firmware *fw = NULL;
	int rc = 0;

	TOUCH_TRACE();

#if defined (CONFIG_LGE_TOUCH_TOVIS_PANEL)
	if(check_panel_type == true)
		rc = request_firmware(&fw, ts->def_fwpath[2], dev);
	else
		rc = request_firmware(&fw, ts->def_fwpath[0], dev);
#else
	rc = request_firmware(&fw, ts->def_fwpath[0], dev);
#endif

	if (rc != 0) {
		TOUCH_E("[%s] request_firmware() failed %d\n", __func__, rc);
		return -EIO;
	}

	memcpy(d->ic_info.img_product_id, &fw->data[d->ic_info.fw_pid_addr], 6);
	memcpy(d->ic_info.img_raws, &fw->data[d->ic_info.fw_ver_addr], 4);
	d->ic_info.img_version.major = (d->ic_info.img_raws[3] & 0x80 ? 1 : 0);
	d->ic_info.img_version.minor = (d->ic_info.img_raws[3] & 0x7F);

	release_firmware(fw);

	return rc;
}

static char *synaptics_productcode_parse(unsigned char *product)
{
	static char str[128] = {0};
	int len = 0;
	char inch[2] = {0};
	char paneltype = 0;
	char version[2] = {0};
	const char *str_panel[]
		= { "ELK", "Suntel", "Tovis", "Innotek", "JDI", "LGD", };
	const char *str_ic[] = { "Synaptics", };
	int i;

	TOUCH_TRACE();

	i = (product[0] & 0xF0) >> 4;
	if (i < 6)
		len += snprintf(str + len, sizeof(str) - len,
				"%s\n", str_panel[i]);
	else
		len += snprintf(str + len, sizeof(str) - len,
				"Unknown\n");

	i = (product[0] & 0x0F);
	if (i < 5 && i != 1)
		len += snprintf(str + len, sizeof(str) - len,
				"%dkey\n", i);
	else
		len += snprintf(str + len, sizeof(str) - len,
				"Unknown\n");

	i = (product[1] & 0xF0) >> 4;
	if (i < 1)
		len += snprintf(str + len, sizeof(str) - len,
				"%s\n", str_ic[i]);
	else
		len += snprintf(str + len, sizeof(str) - len,
				"Unknown\n");

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += snprintf(str + len, sizeof(str) - len,
			"%d.%d\n", inch[0], inch[1]);

	paneltype = (product[2] & 0x0F);
	len += snprintf(str + len, sizeof(str) - len,
			"PanelType %d\n", paneltype);

	version[0] = ((product[3] & 0x80) >> 7);
	version[1] = (product[3] & 0x7F);
	len += snprintf(str + len, sizeof(str) - len,
			"version : v%d.%02d\n", version[0], version[1]);

	return str;
}

static int synaptics_get_cmd_version(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int offset = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = synaptics_ic_info(dev);
	ret += synaptics_bin_fw_version(dev);

	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf + offset, PAGE_SIZE - offset,
				"\n======== Firmware Info ========\n");
	/* IC_Info */
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"ic_version RAW = %02X %02X %02X %02X\n",
			d->ic_info.raws[0], d->ic_info.raws[1],
			d->ic_info.raws[2], d->ic_info.raws[3]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"=== ic_fw_version info ===\n%s",
			synaptics_productcode_parse(d->ic_info.raws));
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"IC_product_id[%s]\n", d->ic_info.product_id);
	if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG635", 6)
			|| synaptics_is_product(d, "PLG640", 6) || synaptics_is_product(d, "PLG643", 6))
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch IC : TD4100\n\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch product ID read fail\n");

	/* Image_Info */
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"img_version RAW = %02X %02X %02X %02X\n",
			d->ic_info.img_raws[0], d->ic_info.img_raws[1],
			d->ic_info.img_raws[2], d->ic_info.img_raws[3]);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"=== img_version info ===\n%s",
			synaptics_productcode_parse(d->ic_info.img_raws));
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Img_product_id[%s]\n", d->ic_info.img_product_id);
	if (synaptics_is_img_product(d, "PLG632", 6) || synaptics_is_img_product(d, "PLG635", 6)
			|| synaptics_is_img_product(d, "PLG640", 6) || synaptics_is_img_product(d, "PLG643", 6))
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch IC : TD4100\n\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch product ID read fail\n");

	return offset;
}

static int synaptics_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int offset = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = synaptics_ic_info(dev);
	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	offset = snprintf(buf + offset, PAGE_SIZE - offset,
			"v%d.%02d(0x%X/0x%X/0x%X/0x%X)\n",
			d->ic_info.version.major,
			d->ic_info.version.minor,
			d->ic_info.raws[0],
			d->ic_info.raws[1],
			d->ic_info.raws[2],
			d->ic_info.raws[3]);

	return offset;
}

static int synaptics_debug_option(struct device *dev, u32 *data)
{
	u32 chg_mask = data[0];
	u32 enable = data[1];

	TOUCH_TRACE();

	switch (chg_mask) {
	case DEBUG_OPTION_0:
		TOUCH_I("Debug Option 0 %s\n", enable ? "Enable" : "Disable");
		break;
	default:
		TOUCH_E("Not supported debug option\n");
		break;
	}

	return 0;
}

static int synaptics_notify(struct device *dev, ulong event, void *data)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s event=0x%x", __func__, (unsigned int)event);

	switch (event) {
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		ret = synaptics_debug_option(dev, (u32 *)data);
		break;
	case NOTIFY_IME_STATE:
		TOUCH_I("NOTIFY_IME_STATE!\n");
		ret = synaptics_change_ime_status(dev);
		if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6))
			synaptics_update_pen_mode(dev);
		break;
	case NOTIFY_QMEMO_STATE:
		TOUCH_I("NOTIFY_QMEMO_STATE!\n");
		if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6))
			ret = synaptics_update_pen_mode(dev);
		break;
	case NOTIFY_PMEMO_STATE:
		TOUCH_I("NOTIFY_PMEMO_STATE!\n");
		if (synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6))
			ret = synaptics_update_pen_mode(dev);
		break;
	default:
		TOUCH_E("%lu is not supported\n", event);
		break;
	}

	return ret;
}

static ssize_t show_pen_support(struct device *dev, char *buf)
{
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", 1);

	return ret;
}

static ssize_t store_reg_ctrl(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	u32 reg = 0;
	int offset = 0;
	u32 value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%5s %d %x %d %x ",
				command, &page, &reg, &offset, &value) <= 0)
		return count;

	if ((offset < 0) || (offset > 49)) {
		TOUCH_E("invalid offset[%d]\n", offset);
		return count;
	}

	mutex_lock(&ts->lock);
	synaptics_set_page(dev, page);
	if (!strcmp(command, "write")) {
		synaptics_read(dev, reg, buffer, offset + 1);
		buffer[offset] = (u8)value;
		synaptics_write(dev, reg, buffer, offset + 1);
	} else if (!strcmp(command, "read")) {
		synaptics_read(dev, reg, buffer, offset + 1);
		TOUCH_I("page[%d] reg[%x] offset[%d] = 0x%x\n",
				page, reg, offset, buffer[offset]);
	} else {
		TOUCH_E("Usage\n");
		TOUCH_E("Write page reg offset value\n");
		TOUCH_E("Read page reg offset\n");
	}
	synaptics_set_page(dev, DEFAULT_PAGE);
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t show_check_noise(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int offset = 0;

	TOUCH_TRACE();

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Test Count : %d\n", d->noise.cnt);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Current Noise State : %d\n", d->noise.cns_avg);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Interference Metric : %d\n", d->noise.im_avg);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"CID IM : %d\n", d->noise.cid_im_avg);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Freq Scan IM : %d\n", d->noise.freq_scan_im_avg);

	return offset;
}

static ssize_t store_check_noise(struct device *dev,
		const char *buf, size_t count)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if ((d->noise.check_noise == NOISE_DISABLE)
		&& (value == NOISE_ENABLE)) {
		d->noise.check_noise = NOISE_ENABLE;
	} else if ((d->noise.check_noise == NOISE_ENABLE)
			&& (value == NOISE_DISABLE)) {
		d->noise.check_noise = NOISE_DISABLE;
	} else {
		TOUCH_I("Already enabled check_noise\n");
		TOUCH_I("check_noise = %d, value = %d\n",
				d->noise.check_noise, value);
		return count;
	}

	TOUCH_I("check_noise = %s\n", (d->noise.check_noise == NOISE_ENABLE)
			? "NOISE_CHECK_ENABLE" : "NOISE_CHECK_DISABLE");

	return count;
}

static ssize_t show_noise_log(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int offset = 0;

	TOUCH_TRACE();

	offset += snprintf(buf + offset, PAGE_SIZE - offset, "%d\n",
				d->noise.noise_log);

	TOUCH_I("noise_log = %s\n", (d->noise.noise_log == NOISE_ENABLE)
			? "NOISE_LOG_ENABLE" : "NOISE_LOG_DISABLE");

	return offset;
}

static ssize_t store_noise_log(struct device *dev,
		const char *buf, size_t count)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if ((d->noise.noise_log == NOISE_DISABLE)
		&& (value == NOISE_ENABLE)) {
		d->noise.noise_log = NOISE_ENABLE;
	} else if ((d->noise.noise_log == NOISE_ENABLE)
			&& (value == NOISE_DISABLE)) {
		d->noise.noise_log = NOISE_DISABLE;
	} else {
		TOUCH_I("Already enabled noise_log\n");
		TOUCH_I("noise_log = %d, value = %d\n",
				d->noise.noise_log, value);
		return count;
	}

	TOUCH_I("noise_log = %s\n", (d->noise.noise_log == NOISE_ENABLE)
			? "NOISE_LOG_ENABLE" : "NOISE_LOG_DISABLE");

	return count;
}

static int synaptics_recovery(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct i2c_client *client = to_i2c_client(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	u32 backup_slave_addr = client->addr;
	int ret = 0;

	TOUCH_TRACE();

#if defined (CONFIG_LGE_TOUCH_TOVIS_PANEL)
	if(check_panel_type == true)
		memcpy(fwpath, ts->def_fwpath[3], sizeof(fwpath));
	else
		memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath));
#else
	memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath));
#endif

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}
	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);
	if (ret < 0) {
		TOUCH_E(
			"fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);
		return ret;
	}
	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	client->addr = 0x2c;
	FirmwareRecovery(dev, fw);
	client->addr = backup_slave_addr;

	release_firmware(fw);

	return ret;
}

static ssize_t show_fw_recovery(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int offset = 0;

	TOUCH_TRACE();
	TOUCH_I("skip fw recovery\n");
	return offset;

	atomic_set(&ts->state.core, CORE_UPGRADE);
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	synaptics_recovery(dev);

	d->need_scan_pdt = true;
	/* init force_upgrade */
	ts->force_fwup = 0;
	ts->test_fwpath[0] = '\0';

	synaptics_reset_ctrl(dev, SW_RESET);
	synaptics_init(dev);
	atomic_set(&ts->state.core, CORE_NORMAL);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s\n",
				"FirmwareRecovery Finish");

	return offset;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;
	touch_interrupt_control(dev, INTERRUPT_DISABLE);
	synaptics_reset_ctrl(dev, value);
	synaptics_init(dev);
	touch_interrupt_control(dev, INTERRUPT_ENABLE);
	return count;
}

static ssize_t show_lpwg_fail_reason(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret += sprintf(buf+ret, "LPWG_FAIL_REASON : [%d]\n", d->lpwg_fail_reason);

	return ret;
}

static ssize_t store_lpwg_fail_reason(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int value = 0;

	TOUCH_TRACE();

	if (ts->lpwg.screen == 0) {
		TOUCH_I("LCD is off. Try after LCD On \n");
		return count;
	}

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	d->lpwg_fail_reason = value;

	return count;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(pen_support, show_pen_support, NULL);
static TOUCH_ATTR(ts_noise, show_check_noise, store_check_noise);
static TOUCH_ATTR(ts_noise_log_enable, show_noise_log, store_noise_log);
static TOUCH_ATTR(fw_recovery, show_fw_recovery, NULL);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(lpwg_fail_reason, show_lpwg_fail_reason, store_lpwg_fail_reason);

static struct attribute *td4100_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_pen_support.attr,
	&touch_attr_ts_noise.attr,
	&touch_attr_ts_noise_log_enable.attr,
	&touch_attr_fw_recovery.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_lpwg_fail_reason.attr,
	NULL,
};

static const struct attribute_group td4100_attribute_group = {
	.attrs = td4100_attribute_list,
};

static int synaptics_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();


	ret = sysfs_create_group(&ts->kobj, &td4100_attribute_group);
	if (ret < 0)
		TOUCH_E("td4100 sysfs register failed\n");

	td4100_prd_register_sysfs(dev);

	return 0;
}

static int synaptics_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int synaptics_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = synaptics_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = synaptics_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static int synaptics_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct synaptics_version *device = &d->ic_info.version;
	struct synaptics_version *binary = NULL;
	int update = 0;

	TOUCH_TRACE();

	memcpy(d->ic_info.img_product_id, &fw->data[d->ic_info.fw_pid_addr], 6);
	memcpy(d->ic_info.img_raws, &fw->data[d->ic_info.fw_ver_addr], 4);
	d->ic_info.img_version.major = (d->ic_info.img_raws[3] & 0x80 ? 1 : 0);
	d->ic_info.img_version.minor = (d->ic_info.img_raws[3] & 0x7F);
	binary = &d->ic_info.img_version;

	if (ts->force_fwup) {
		update = 1;
	} else if (binary->major != device->major) {
		update = 1;
	} else {
		if (binary->minor != device->minor)
			update = 1;
		else if (binary->build > device->build)
			update = 1;
	}

	TOUCH_I("%s : binary[%d.%02d.%d] device[%d.%02d.%d]" \
		" -> update: %d, force: %d\n", __func__,
		binary->major, binary->minor, binary->build,
		device->major, device->minor, device->build,
		update, ts->force_fwup);

	return update;
}
static int synaptics_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	u8 status = 0;
	int ret = 0;

	TOUCH_TRACE();

	if((check_recovery_boot == LGE_RECOVERY_BOOT) || (lge_get_laf_mode() == LGE_LAF_MODE_LAF) || (touch_boot_mode() == TOUCH_CHARGER_MODE))
	{
		TOUCH_I("recovery mode booting fw upgrade skip!!\n");
		return -EPERM;
	}

	if (atomic_read(&d->state.init) == INIT_END)
		atomic_set(&d->state.init, INIT_RUN);


	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		/* TD4100 fw_recovery */
		if (d->need_fw_recovery) {
			d->need_fw_recovery = false;
			if (synaptics_recovery(dev) < 0) {
				TOUCH_E("fw_recovery fail\n");
				return -EPERM;
			}
			ts->force_fwup = 1;
			touch_msleep(20);
		}

#if defined (CONFIG_LGE_TOUCH_TOVIS_PANEL)
	if(check_panel_type == true)
		memcpy(fwpath, ts->def_fwpath[2], sizeof(fwpath));
	else
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
#else
	memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
#endif

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

	ret = synaptics_read(dev, DEVICE_STATUS_REG,
			&status, sizeof(status));

	if (ret < 0) {
		TOUCH_E("failed to read device status - ret:%d\n", ret);
	} else {
		TOUCH_I("status[device:%02x]\n",status);
		if((status & 0xC0) == 0x40){  // bit 6 : check bootloader mode
			TOUCH_I("bootloader mode. force fw upgrade!\n");
			ts->force_fwup = 1;
		}
	}

	if (synaptics_fw_compare(dev, fw) == 1) {
		d->need_scan_pdt = true;
		ret = FirmwareUpgrade(dev, fw);
	} else {
		release_firmware(fw);
		atomic_set(&d->state.init, INIT_END);

		return -EPERM;
	}

	synaptics_reset_ctrl(dev, SW_RESET);
	release_firmware(fw);
	ts->force_fwup = 0;
	atomic_set(&d->state.init, INIT_END);


	return 0;
}

static void synaptics_palm_filter_work_func(struct work_struct *palm_filter_work)
{
	struct synaptics_data *d =
		container_of(to_delayed_work(palm_filter_work),
				struct synaptics_data, palm_filter_work);

	TOUCH_TRACE();

	if (!(synaptics_is_product(d, "PLG632", 6) || synaptics_is_product(d, "PLG643", 6)))
		return;

	synaptics_clear_palm_filter_data(d);

	return;
}

static void synaptics_init_works(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	TOUCH_TRACE();

	INIT_DELAYED_WORK(&d->palm_filter_work,
			synaptics_palm_filter_work_func);
}

static int synaptics_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate synaptics data\n");
		return -ENOMEM;
	}

	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	synaptics_init_works(dev);

	synaptics_init_tci_info(dev);
	pm_qos_add_request(&d->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	d->need_scan_pdt = true;
	d->ic_info.fw_pid_addr = 0x10;
	d->ic_info.fw_ver_addr = 0x1d100;
	/*Real-Time fail reason - default disable*/
	d->lpwg_fail_reason  = 0;

	return 0;
}

static struct touch_driver touch_driver = {
	.probe = synaptics_probe,
	.remove = synaptics_remove,
	.suspend = synaptics_suspend,
	.resume = synaptics_resume,
	.init = synaptics_init,
	.upgrade = synaptics_upgrade,
	.irq_handler = synaptics_irq_handler,
	.power = synaptics_power,
	.lpwg = synaptics_lpwg,
	.notify = synaptics_notify,
	.register_sysfs = synaptics_register_sysfs,
	.set = synaptics_set,
	.get = synaptics_get,
};


#define MATCH_NAME			"synaptics,TD4100"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
	//.bits_per_word = 8,
	//.spi_mode = SPI_MODE_3,
	//.max_freq = (5 * 1000000),

};

#if defined (CONFIG_LGE_TOUCH_TOVIS_PANEL)
bool check_panel_type = false;
#endif
static int __init touch_device_init(void)
{
	TOUCH_TRACE();

#if defined (CONFIG_LGE_TOUCH_TOVIS_PANEL)
	if (lge_get_panel_type() == LV5_LGD) {
		TOUCH_I("TD4100__LGD panel\n");
		return 0;
	}
	else if (lge_get_panel_type() == SF3_TOVIS)
	{
		TOUCH_I("TD4100__SF3_TOVIS panel\n");
		check_panel_type = true;
	}
#endif

	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("hoyeon.jang@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
