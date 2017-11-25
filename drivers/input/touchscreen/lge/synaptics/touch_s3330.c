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
#include <linux/syscalls.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_s3330.h"
#include "touch_s3330_prd.h"
#include "touch_s3330_f54_test.h"

/*
 * PLG591 - PH2
 * PLG636 - LV9
 */

const char *f_str[] = {
	"ERROR",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME"
};

static struct synaptics_exp_fhandler rmidev_fhandler;

bool synaptics_is_product(struct synaptics_data *d,
				const char *product_id, size_t len)
{
	return strncmp(d->fw.product_id, product_id, len)
			? false : true;
}

bool synaptics_is_img_product(struct synaptics_data *d,
				const char *product_id, size_t len)
{
	return strncmp(d->fw.img_product_id, product_id, len)
			? false : true;
}

int synaptics_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->io_lock);

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->io_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	mutex_unlock(&d->io_lock);

	return 0;
}

int synaptics_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->io_lock);

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size+1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->io_lock);
		return ret;
	}

	mutex_unlock(&d->io_lock);

	return 0;
}

static int synaptics_irq_enable(struct device *dev, bool enable)
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
		val |= (INTERRUPT_MASK_ABS0 | INTERRUPT_MASK_CUSTOM);
	else
		val &= ~INTERRUPT_MASK_ABS0;


	ret = synaptics_write(dev, INTERRUPT_ENABLE_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write interrupt enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("write interrupt : enable:%d, val:%02X\n", enable, val);

	return 0;
}

int synaptics_set_page(struct device *dev, u8 page)
{
	int ret = synaptics_write(dev, PAGE_SELECT_REG, &page, 1);

	if (ret >= 0)
		to_synaptics_data(dev)->curr_page = page;

	return ret;
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

	TOUCH_TRACE();

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
			return ret;
		}

		for (pdt = PDT_START; pdt > PDT_END; pdt -= sizeof(dsc)) {
			ret = synaptics_read(dev, pdt, &dsc, sizeof(dsc));

			if (ret < 0) {
				TOUCH_E("read descrptore %d (ret: %d)\n",
					pdt, ret);
				return ret;
			}

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

	TOUCH_D(BASE_INFO,
		"common[%dP:0x%02x] finger_f12[%dP:0x%02x] flash[%dP:0x%02x] analog[%dP:0x%02x] lpwg[%dP:0x%02x]\n",
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

static int synaptics_get_product_id(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	u8 temp_pid[11] = {0,};

	TOUCH_TRACE();

	ret = synaptics_read(dev, PRODUCT_ID_REG,
			d->fw.product_id, sizeof(d->fw.product_id) - 1);

	if (ret < 0) {
		TOUCH_I("[%s]read error...\n", __func__);
		return ret;
	}

	TOUCH_I("[%s] IC_product_id: %s\n",
			__func__, d->fw.product_id);

	if (synaptics_is_product(d, "S3330", 5)) {
		ret = synaptics_read(dev, FLASH_CONFIG_ID_REG,
				temp_pid, sizeof(temp_pid) - 1);

		if (ret < 0) {
			TOUCH_I("[%s]read error...\n", __func__);
			return ret;
		}

		memset(d->fw.product_id, 0,
				sizeof(d->fw.product_id));
		memcpy(d->fw.product_id, &temp_pid[4], 6);

		TOUCH_I("[%s] Product_ID_Reset ! , addr = 0x%x, P_ID = %s\n",
			__func__, FLASH_CONFIG_ID_REG, d->fw.product_id);
	} else {
		return -EINVAL;
	}

	return 0;
}

int synaptics_ic_info(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret;

	TOUCH_TRACE();

	if (d->need_scan_pdt == true) {
		ret = synaptics_page_description(dev);
		if (ret < 0) {
			TOUCH_E("firmware page description read error... : (%d)\n", ret);
			TOUCH_E("Touch IC(S3330) is no response\n");
			d->need_scan_pdt = false;
			return ret;
		}
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	synaptics_get_product_id(dev);
	ret = synaptics_read(dev, FLASH_CONFIG_ID_REG,
			d->fw.version, sizeof(d->fw.version));
	if (ret < 0)
		TOUCH_E("Customer Configuration ID read error : %d\n", ret);
	ret = synaptics_read(dev, CUSTOMER_FAMILY_REG,
			&(d->fw.family), sizeof(d->fw.family));
	if (ret < 0)
		TOUCH_E("Customer Family Query read error : %d\n", ret);
	ret = synaptics_read(dev, FW_REVISION_REG,
			&(d->fw.revision), sizeof(d->fw.revision));
	if (ret < 0)
		TOUCH_E("Firmware Revision Query read error : %d\n", ret);

	d->fw.info.major = (d->fw.version[3] & 0x80 ? 1 : 0);
	d->fw.info.minor = (d->fw.version[3] & 0x7F);

	TOUCH_I("=======================\n");
	TOUCH_I("IC_Version = v%d.%02d\n", d->fw.info.major, d->fw.info.minor);
	TOUCH_I("Customer Family = %d\n", d->fw.family);
	TOUCH_I("F/W Revision = %d\n", d->fw.revision);
	TOUCH_I("Product ID = %s\n", d->fw.product_id);
	TOUCH_I("=======================\n");

	return 0;
}

static int synaptics_sleep_control(struct device *dev, u8 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 val;
	int ret;

	TOUCH_TRACE();

	touch_msleep(50);
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

	touch_msleep(50);
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
	int ret = 0;
	u8 count = 0;
	u8 index = 0;
	u8 buf = 0;
	u8 i = 0;
	u8 addr = 0;
	u8 offset = num ? LPWG_MAX_BUFFER + 2 : 0;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	ret = synaptics_read(dev, LPWG_TCI1_FAIL_COUNT_REG + offset, &count, sizeof(count));
	if (ret < 0) {
		TOUCH_E("failed to read LPWG_TCI1_FAIL_COUNT_REG. - ret:%d\n", ret);
		return ret;
	}
	ret = synaptics_read(dev, LPWG_TCI1_FAIL_INDEX_REG + offset, &index, sizeof(index));
	if (ret < 0) {
		TOUCH_E("failed to read LPWG_TCI1_FAIL_INDEX_REG. - ret:%d\n", ret);
		return ret;
	}

	for (i = 1; i <= count; i++) {
		addr = LPWG_TCI1_FAIL_BUFFER_REG + offset +
			((index + LPWG_MAX_BUFFER - i) % LPWG_MAX_BUFFER);
		ret = synaptics_read(dev, addr, &buf, sizeof(buf));
		if (ret < 0) {
			TOUCH_E("failed to read for LPWG_DEBUG. - ret:%d\n", ret);
			return ret;
		}
		TOUCH_I("TCI(%d)-Fail[%d/%d] : %s\n", num, count - i + 1, count,
			(buf > 0 && buf < 6) ? f_str[buf] : f_str[0]);

		if (i == LPWG_MAX_BUFFER)
			break;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	return 0;
}

int synaptics_tci_report_enable(struct device *dev, bool enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val[3];
	int ret;

	TOUCH_TRACE();

	synaptics_irq_enable(dev, enable ? false : true);

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

static int synaptics_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct tci_info *info;

	u8 lpwg_data[7];
	int ret = 0;
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
			ret = synaptics_write(dev, tci_reg[i], lpwg_data, sizeof(u8));
			if (ret < 0) {
				TOUCH_E("failed to write tci0_reg - ret:%d\n", ret);
				return ret;
			}
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
			ret = synaptics_write(dev, tci_reg[i], lpwg_data, sizeof(lpwg_data));
			if (ret < 0) {
				TOUCH_E("failed to write tci1_reg - ret:%d\n", ret);
				return ret;
			}
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
	int ret = 0;

	TOUCH_TRACE();

	buffer[0] = (ts->lpwg.area[0].x >> 0) & 0xff;
	buffer[1] = (ts->lpwg.area[0].x >> 8) & 0xff;
	buffer[2] = (ts->lpwg.area[0].y >> 0) & 0xff;
	buffer[3] = (ts->lpwg.area[0].y >> 8) & 0xff;
	buffer[4] = (ts->lpwg.area[1].x >> 0) & 0xff;
	buffer[5] = (ts->lpwg.area[1].x >> 8) & 0xff;
	buffer[6] = (ts->lpwg.area[1].y >> 0) & 0xff;
	buffer[7] = (ts->lpwg.area[1].y >> 8) & 0xff;

	ret = synaptics_write(dev, d->f12_reg.ctrl[18], buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to write tci_active_area - ret:%d\n", ret);
		return ret;
	}

	return 0;
}

static int synaptics_cover_set(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 buf_1 = 0, buf_2 = 0;
	u8 official = 0;
	u8 version = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (synaptics_is_product(d, "PLG636", 6)) {
		/* LV9 is not support cover mode */
                return 0;
	}

	if (ts->factory_boot)
		return ret;

	official = d->fw.version[3] & 0x80 ? 1 : 0;
	version = d->fw.version[3] & 0x7F;

	if (((official == 1) && (version < 5)) || ((official == 0) && (version < 18))) {
		TOUCH_I("Not support f/w to cover setting\n");
		return ret;
	}

	ret = synaptics_read(dev, OBJECT_REPORT_ENABLE_REG, &buf_1, sizeof(buf_1));
	if (ret < 0) {
		TOUCH_E("failed to read OBJECT_REPORT_ENABLE_REG\n");
		return ret;
	}

	ret = synaptics_read(dev, FEATURE_ENABLE_REG, &buf_2, sizeof(buf_2));
	if (ret < 0) {
		TOUCH_E("failed to read FEATURE_ENABLE_REG\n");
		return ret;
	}

	if (ts->lpwg.qcover == QUICKCOVER_CLOSE) {
		TOUCH_I("QUICKCOVER_CLOSE\n");
		buf_1 |= 0x20;
		buf_2 |= 0x03;
	} else {
		TOUCH_I("QUICKCOVER_OPEN\n");
		buf_1 &= 0xDF;
		buf_2 &= 0xFC;
	}

	ret = synaptics_write(dev, OBJECT_REPORT_ENABLE_REG, &buf_1, sizeof(buf_1));
	if (ret < 0) {
		TOUCH_E("failed to write OBJECT_REPORT_ENABLE_REG\n");
		return ret;
	}

	ret = synaptics_write(dev, FEATURE_ENABLE_REG, &buf_2, sizeof(buf_2));
	if (ret < 0) {
		TOUCH_E("failed to write FEATURE_ENABLE_REG\n");
		return ret;
	}

	return 0;
}

static int synaptics_glove_set(struct device *dev, int mode_enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 buf_1 = 0, buf_2 = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (synaptics_is_product(d, "PLG591", 6)) {
		/* not support glove mode */
                return 0;
	}

	ret = synaptics_read(dev,OBJECT_REPORT_ENABLE_REG, &buf_1, sizeof(buf_1));
	if (ret < 0) {
		TOUCH_E("failed to read OBJECT_REPORT_ENABLE_REG\n");
		return ret;
	}

	ret = synaptics_read(dev,FEATURE_ENABLE_REG, &buf_2, sizeof(buf_2));
	if (ret < 0) {
		TOUCH_E("failed to read FEATURE_ENABLE_REG\n");
		return ret;
	}

        if (mode_enable == 1) {
                buf_1 |= 0x20;
                buf_2 |= 0x01;
        } else {
                buf_1 &= 0xDF;
                buf_2 &= 0xFE;
        }

	ret = synaptics_write(dev, OBJECT_REPORT_ENABLE_REG, &buf_1, sizeof(buf_1));
	if (ret < 0) {
		TOUCH_E("failed to write Gloved Finger Report Enable\n");
		return ret;
	}

	ret = synaptics_write(dev,FEATURE_ENABLE_REG, &buf_2, sizeof(buf_2));
	if (ret < 0) {
		TOUCH_E("failed to write Enable Gloved Finger Detection\n");
		return ret;
	}

	TOUCH_I("Glove Feature %s!!!\n", (mode_enable == 1)
			? "Enable" : "Disable");
	return 0;
}

static int synaptics_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct tci_info *info1 = &ts->tci.info[TCI_1];

	TOUCH_TRACE();
	TOUCH_I("synaptics_lpwg_control mode = %d\n", mode);

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		info1->intr_delay = 0;
		info1->tap_distance = 10;
		synaptics_tci_knock(dev);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x03;
		info1->intr_delay = ts->tci.double_tap_check ? 68 : 0;
		info1->tap_distance = 7;
		synaptics_tci_password(dev);
		break;

	case LPWG_PASSWORD_ONLY:
		ts->tci.mode = 0x02;
		info1->intr_delay = 0;
		info1->tap_distance = 10;
		synaptics_tci_password(dev);
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
	int ret = 0;
	u32 data = atomic_read(&d->glove_support);

	TOUCH_TRACE();

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			/* Force knock on set in minios mode */
			TOUCH_I("MiniOS boot mode. change lpwg.mode set : 1\n");
			ts->lpwg.mode = 1;
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_lpwg_control(dev, ts->lpwg.mode);
			ts->lpwg.mode = 0;
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP){
				synaptics_sleep_control(dev, IC_NORMAL);
			}
			synaptics_tci_report_enable(dev, false);
			synaptics_lpwg_debug(dev, TCI_1);
			synaptics_lpwg_debug(dev, TCI_2);
		} else if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep - LPWG Disable */
			TOUCH_I("%s(%d) - deep sleep by knock on disable\n",
				__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_lpwg_control(dev, LPWG_NONE);
			synaptics_sleep_control(dev, IC_DEEP_SLEEP);
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep - Proxy Near */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
				__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
			synaptics_lpwg_control(dev, LPWG_NONE);
			synaptics_sleep_control(dev, IC_DEEP_SLEEP);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* folio cover - LPWG Disable */
			TOUCH_I("%s(%d) - qcover close\n",
				__func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP)
				synaptics_sleep_control(dev, IC_NORMAL);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock %d, screen %d, proxy %d, qcover %d\n",
				__func__, __LINE__,
				ts->lpwg.mode, ts->lpwg.screen, ts->lpwg.sensor, ts->lpwg.qcover);
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP){
				synaptics_sleep_control(dev, IC_NORMAL);
			}
			synaptics_lpwg_control(dev, ts->lpwg.mode);
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
		synaptics_cover_set(dev);
		ret = synaptics_glove_set(dev, data);
		if(ret < 0) {
                        TOUCH_I("%s : failed to set glove mode\n", __func__);
		}
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		TOUCH_I("%s - wake up is not ready\n", __func__);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else {
		/* partial */
		TOUCH_I("%s(%d) - partial mode(knock %d, screen %d, proxy %d, qcover %d)\n",
				__func__, __LINE__,
				ts->lpwg.mode, ts->lpwg.screen, ts->lpwg.sensor, ts->lpwg.qcover);
		TOUCH_I("%s - partial is not ready\n", __func__);
		synaptics_lpwg_control(dev, ts->lpwg.mode);
	}

	return 0;
}

static void synaptics_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 4;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 4;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 0;
}

static int synaptics_remove(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	TOUCH_TRACE();
	pm_qos_remove_request(&d->pm_qos_req);

	if (rmidev_fhandler.initialized
		&& rmidev_fhandler.inserted) {
		rmidev_fhandler.exp_fn->remove(dev);
		rmidev_fhandler.initialized = false;
	}

	return 0;
}

static int synaptics_get_status(struct device *dev, u8 *device, u8 *interrupt)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 dev_status;
	u8 irq_status;
	int ret;

	TOUCH_TRACE();

	ret = synaptics_read(dev, DEVICE_STATUS_REG,
			&dev_status, sizeof(dev_status));

	if (ret < 0) {
		TOUCH_E("failed to read device status - ret:%d\n", ret);
		return ret;
	}

	ret = synaptics_read(dev, INTERRUPT_STATUS_REG,
			&irq_status, sizeof(irq_status));

	if (ret < 0) {
		TOUCH_E("failed to read interrupt status - ret:%d\n", ret);
		return ret;
	}

	TOUCH_D(TRACE, "status[device:%02x, interrupt:%02x]\n",
		dev_status, irq_status);

	if (device)
		*device = dev_status;

	if (interrupt)
		*interrupt = irq_status;

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
	int i = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to ANALOG_PAGE\n");
		return ret;
	}

	ret = synaptics_read(dev, INTERFERENCE_METRIC_LSB_REG, &buf_lsb, sizeof(buf_lsb));
	if (ret < 0) {
		TOUCH_E("failed to read INTERFERENCE_METRIC_LSB_REG - ret:%d\n", ret);
		return ret;
	}
	ret = synaptics_read(dev, INTERFERENCE_METRIC_MSB_REG, &buf_msb, sizeof(buf_msb));
	if (ret < 0) {
		TOUCH_E("failed to read INTERFERENCE_METRIC_MSB_REG - ret:%d\n", ret);
		return ret;
	}

	im = (buf_msb << 8) | buf_lsb;
	d->noise.im_sum += im;

	ret = synaptics_read(dev, CURRENT_NOISE_STATUS_REG, &cns, sizeof(cns));
	if (ret < 0) {
		TOUCH_E("failed to read CURRENT_NOISE_STATUS_REG - ret:%d\n", ret);
		return ret;
	}
	d->noise.cns_sum += cns;

	ret = synaptics_read(dev, CID_IM_REG, buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read CID_IM_REG - ret:%d\n", ret);
		return ret;
	}
	cid_im = (buffer[1] << 8) | buffer[0];
	d->noise.cid_im_sum += cid_im;

	ret = synaptics_read(dev, FREQ_SCAN_IM_REG, buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read FREQ_SCAN_IM_REG - ret:%d\n", ret);
		return ret;
	}
	freq_scan_im = (buffer[1] << 8) | buffer[0];
	d->noise.freq_scan_im_sum += freq_scan_im;

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	d->noise.cnt++;

	if (d->noise.noise_log == NOISE_ENABLE) {
		if (ts->old_mask != ts->new_mask) {
			TOUCH_I("Curr : CNS[%5d] IM[%5d] CID_IM[%5d] FREQ_SCAN_IM[%5d]\n",
					cns, im, cid_im, freq_scan_im);
		}
	}

	if ((i < MAX_NUM_OF_FINGERS && ts->new_mask == 0)
				|| (d->noise.im_sum >= ULONG_MAX
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

static void synaptics_palm_log(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u16 old_mask = d->old_pmask;
	u16 new_mask = d->new_pmask;
	u16 press_mask = 0;
	u16 release_mask = 0;
	u16 change_mask = 0;
	int i;

	TOUCH_TRACE();

	change_mask = old_mask ^ new_mask;
	press_mask = new_mask & change_mask;
	release_mask = old_mask & change_mask;

	TOUCH_D(ABS, "pmask [new: %04x, old: %04x]\n",
			new_mask, old_mask);
	TOUCH_D(ABS, "pmask [change: %04x, press: %04x, release: %04x]\n",
			change_mask, press_mask, release_mask);

	if(!change_mask)
		goto end;

	for (i = 0; i < MAX_FINGER; i++) {
		if (new_mask & (1 << i)) {
			if (press_mask & (1 << i)) {
				TOUCH_I("%d palm detected:<%d>\n", d->pcount, i);
			}
		} else if (release_mask & (1 << i)) {
			TOUCH_I("palm released:<%d>\n", i);
		}
	}

end:
	d->old_pmask = new_mask;
	return ;
}

static int synaptics_get_object_count(struct device *dev, u8 *object)
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

	for (; object_to_read > 0 ;) {
		if (object_attention & (0x1 << (object_to_read - 1)))
			break;
		else
			object_to_read--;
	}

	*object = object_to_read;
	return 0;
}

static int synaptics_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct synaptics_object objects[MAX_NUM_OF_FINGERS];
	struct synaptics_object *obj;
	struct touch_data *tdata;
	u8 i;
	u8 finger_index;
	u8 object_to_read = 0;
	int ret;

	TOUCH_TRACE();

	ts->new_mask = 0;
	d->new_pmask = 0;
	d->pcount = 0;

	ret = synaptics_get_object_count(dev, &object_to_read);

	if (ret < 0) {
		TOUCH_E("faied to read object count\n");
		return ret;
	}

	TOUCH_D(ABS, "object_to_read: %d\n", object_to_read);

	if (object_to_read > 0) {
		ret = synaptics_read(dev, FINGER_DATA_REG,
				     objects, sizeof(*obj) * object_to_read);
		if (ret < 0) {
			TOUCH_E("faied to read finger data\n");
			return ret;
		}

		finger_index = 0;

		for (i = 0; i < object_to_read; i++) {
			obj = objects + i;

			if (obj->type == F12_NO_OBJECT_STATUS)
				continue;

			if (obj->type > F12_MAX_OBJECT)
				TOUCH_D(ABS, "id : %d, type : %d\n",
					i, obj->type);

			if (synaptics_is_product(d, "PLG636", 6) && obj->type == F12_PALM_STATUS) {
				d->new_pmask |= (1 << i);
				d->pcount++;
				continue;
			}

			if (obj->type == F12_FINGER_STATUS
					|| obj->type == F12_PALM_STATUS || obj->type == F12_GLOVED_FINGER_STATUS) {
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

		ts->tcount = finger_index;
	}

	if (d->noise.check_noise == NOISE_ENABLE
		|| d->noise.noise_log == NOISE_ENABLE) {
		synaptics_noise_log(dev);
	}

	ts->intr_status |= TOUCH_IRQ_FINGER;

	if (synaptics_is_product(d, "PLG636", 6)) {
		synaptics_palm_log(dev);
	}
	return 0;
}

static int synaptics_tci_getdata(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u32 buffer[12];
	int i = 0;
	int ret;

	TOUCH_TRACE();

	if (!count)
		return 0;

	ts->lpwg.code_num = count;

	ret = synaptics_read(dev, LPWG_DATA_REG,
					 buffer, sizeof(u32) * count);

	if (ret < 0)
		return ret;

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = buffer[i] & 0xffff;
		ts->lpwg.code[i].y = (buffer[i] >> 16) & 0xffff;

		if ((ts->lpwg.mode >= LPWG_PASSWORD) &&
				ts->role.hide_coordinate)
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}

	ts->lpwg.code[i].x = -1;
	ts->lpwg.code[i].y = -1;

	return 0;
}

static int synaptics_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 status;
	u8 buffer;
	int ret = 0;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	ret = synaptics_read(dev, LPWG_STATUS_REG, &status, 1);
	if (ret < 0) {
		TOUCH_E("failed to read LPWG_STATUS_REG\n");
		return ret;
	}

	if (status & LPWG_STATUS_DOUBLETAP) {
		synaptics_tci_getdata(dev, ts->tci.info[TCI_1].tap_count);
		ts->intr_status |= TOUCH_IRQ_KNOCK;
	} else if (status & LPWG_STATUS_PASSWORD) {
		synaptics_tci_getdata(dev, ts->tci.info[TCI_2].tap_count);
		ts->intr_status |= TOUCH_IRQ_PASSWD;
	} else {
		/* Overtab */
		ret = synaptics_read(dev, LPWG_OVER_TAPCOUNT, &buffer, 1);
		if (ret < 0) {
			TOUCH_E("failed to read LPWG_OVER_TAPCOUNT\n");
			return ret;
		}
		if (buffer > ts->tci.info[TCI_2].tap_count) {
			ts->lpwg.code_num = 1;
			synaptics_tci_getdata(dev, ts->lpwg.code_num);
			ts->intr_status |= TOUCH_IRQ_PASSWD;
			TOUCH_I("knock code fail to over tap count = %d\n", buffer);
		}
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	return ret;
}

static int synaptics_irq_handler(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 dev_status;
	u8 irq_status;
	int ret = 0;
	int retry = 0;

	TOUCH_TRACE();

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

	if (irq_status & INTERRUPT_MASK_ABS0)
		ret = synaptics_irq_abs(dev);
	else if (irq_status & INTERRUPT_MASK_LPWG)
		ret = synaptics_irq_lpwg(dev);

	pm_qos_update_request(&d->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	return ret;
}

void synaptics_rmidev_function(struct synaptics_exp_fn *rmidev_fn,
		bool insert)
{
	TOUCH_TRACE();

	rmidev_fhandler.inserted = insert;

	if (insert) {
		rmidev_fhandler.exp_fn = rmidev_fn;
		rmidev_fhandler.inserted = true;
		rmidev_fhandler.initialized = false;
	} else {
		rmidev_fhandler.exp_fn = NULL;
		rmidev_fhandler.inserted = false;
		rmidev_fhandler.initialized = true;
	}

	return;
}

static int synaptics_rmidev_init(struct device *dev)
{
	int ret = 0;

	TOUCH_TRACE();

	if (rmidev_fhandler.inserted) {
		ret = rmidev_fhandler.exp_fn->init(dev);

		if (ret < 0)
			TOUCH_E("Failed to rmi_dev init\n");
		else
			rmidev_fhandler.initialized = true;
	}

	return 0;
}

int synaptics_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = synaptics_ic_info(dev);
	if (ret < 0) {
		TOUCH_E("synaptics_ic_info failed %d\n", ret);
		return ret;
	}

	synaptics_lpwg_mode(dev);

	synaptics_irq_enable(dev, true);
	synaptics_irq_clear(dev);

	if (!ts->factory_boot)
		synaptics_rmidev_init(dev);

	return 0;
}

static int synaptics_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_power_vio(dev, 0);
		touch_power_vdd(dev, 0);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		touch_power_vdd(dev, 1);
		touch_power_vio(dev, 1);
		touch_gpio_direction_output(ts->reset_pin, 1);
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

	int mfts_mode = lge_get_mfts_mode();

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		return 0;
	}

	if (mfts_mode && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS!!!\n", __func__);
		return 0;
	}

	synaptics_lpwg_mode(dev);

	return 0;
}

static int synaptics_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int mfts_mode = lge_get_mfts_mode();

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		/* Deep Sleep */
		synaptics_sleep_control(dev, 1);
		return -EPERM;
	}

	if (mfts_mode && !ts->role.mfts_lpwg) {
		TOUCH_I("%s, MFTS Touch power on\n", __func__);
		d->need_scan_pdt = true;
	}
/*
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(ts->caps.hw_reset_delay);
*/
	return 0;
}

static int synaptics_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int *value = (int *)param;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->lpwg.area[0].x = value[0];
		ts->lpwg.area[0].y = value[2];
		ts->lpwg.area[1].x = value[1];
		ts->lpwg.area[1].y = value[3];
		TOUCH_I("LPWG AREA (%d,%d)-(%d,%d)\n",
			ts->lpwg.area[0].x, ts->lpwg.area[0].y,
			ts->lpwg.area[1].x, ts->lpwg.area[1].y);
		synaptics_tci_active_area(dev);
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
		break;
	}

	return 0;
}

static int synaptics_bin_fw_version(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	const struct firmware *fw = NULL;
	int rc = 0;

	TOUCH_TRACE();

	rc = request_firmware(&fw, d->fw.def_fw, dev);
	if (rc != 0) {
		TOUCH_E("[%s] request_firmware() failed %d\n", __func__, rc);
		return -EIO;
	}

	memcpy(d->fw.img_product_id, &fw->data[d->fw.fw_pid_addr], 6);
	memcpy(d->fw.img_version, &fw->data[d->fw.fw_ver_addr], 4);
	d->fw.img_info.major = (d->fw.img_version[3] & 0x80 ? 1 : 0);
	d->fw.img_info.minor = (d->fw.img_version[3] & 0x7F);

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
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int offset = 0;
	int ret = 0;

	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);

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

	if (d->fw.version[0] > 0x50) {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"ic_version[%s]\n", d->fw.version);
	} else {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"ic_version RAW = %02X %02X %02X %02X\n",
				d->fw.version[0], d->fw.version[1],
				d->fw.version[2], d->fw.version[3]);
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"=== ic_fw_version info ===\n%s",
				synaptics_productcode_parse(d->fw.version));
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"IC_product_id[%s]\n", d->fw.product_id);

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6))
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch IC : s3330\n\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch product ID read fail\n");

	if (!ts->factory_boot) {
		if (d->fw.img_version[0] > 0x50) {
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"img_version[%s]\n", d->fw.img_version);
		} else {
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"img_version RAW = %02X %02X %02X %02X\n",
					d->fw.img_version[0], d->fw.img_version[1],
					d->fw.img_version[2], d->fw.img_version[3]);
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
					"=== img_version info ===\n%s",
					synaptics_productcode_parse(d->fw.img_version));
		}

		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"Img_product_id[%s]\n", d->fw.img_product_id);

		if (synaptics_is_img_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6))
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"Touch IC : s3330\n\n");
		else
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"Touch product ID read fail\n");
	}
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
			d->fw.info.major,
			d->fw.info.minor,
			d->fw.version[0],
			d->fw.version[1],
			d->fw.version[2],
			d->fw.version[3]);

	return offset;
}

static int synaptics_notify(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int mfts_mode = lge_get_mfts_mode();

	TOUCH_TRACE();

	switch (event) {
	case LCD_EVENT_TOUCH_LPWG_ON:
		d->power_state = POWER_SLEEP;
		if (mfts_mode) {
			TOUCH_I("%s, MFTS LCD Sleep\n", __func__);
			if (ts->role.mfts_lpwg)
				synaptics_tci_report_enable(dev, true);
			else
				TOUCH_I("%s, MFTS Touch power off\n", __func__);
		} else {
			TOUCH_I("%s, LCD Sleep\n", __func__);
			synaptics_tci_report_enable(dev, true);
		}

		break;

	case LCD_EVENT_TOUCH_LPWG_OFF:
		TOUCH_I("%s, LCD_EVENT_TOUCH_LPWG_OFF\n", __func__);
		d->power_state = POWER_WAKE;
		break;
	}

	return 0;
}

static ssize_t show_pen_support(struct device *dev, char *buf)
{
	int ret = 0;
	struct synaptics_data *d = to_synaptics_data(dev);

	TOUCH_TRACE();

	if (synaptics_is_product(d, "PLG591", 6)) {
		/* support pen */
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", 1);
	}
	else if (synaptics_is_product(d, "PLG636", 6)) {
		/* not support pen */
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", 0);
	}

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
	int ret = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%5s %d %x %d %x ",
				command, &page, &reg, &offset, &value) <= 0)
		return count;

	if ((offset < 0) || (offset > 49)) {
		TOUCH_E("invalid offset[%d]\n", offset);
		return count;
	}

	mutex_lock(&ts->lock);
	ret = synaptics_set_page(dev, page);
	if (ret < 0) {
		TOUCH_E("failed to set page to ANLOG_PAGE\n");
		mutex_unlock(&ts->lock);
		return ret;
	}

	if (!strcmp(command, "write")) {
		ret = synaptics_read(dev, reg, buffer, offset + 1);
		if (ret < 0) {
			TOUCH_E("failed to read reg_ctrl_write_reg - ret:%d\n", ret);
			mutex_unlock(&ts->lock);
			return ret;
		}
		buffer[offset] = (u8)value;
		ret = synaptics_write(dev, reg, buffer, offset + 1);
		if (ret < 0) {
			TOUCH_E("failed to write reg_ctrl_write_reg - ret:%d\n", ret);
			mutex_unlock(&ts->lock);
			return ret;
		}
	} else if (!strcmp(command, "read")) {
		ret = synaptics_read(dev, reg, buffer, offset + 1);
		if (ret < 0) {
			TOUCH_E("failed to read reg_ctrl_read_reg - ret:%d\n", ret);
			mutex_unlock(&ts->lock);
			return ret;
		}
		TOUCH_I("page[%d] reg[%x] offset[%d] = 0x%x\n",
				page, reg, offset, buffer[offset]);
	} else {
		TOUCH_E("Usage\n");
		TOUCH_E("Write page reg offset value\n");
		TOUCH_E("Read page reg offset\n");
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		mutex_unlock(&ts->lock);
		return ret;
	}
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

static ssize_t show_glove_state(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (synaptics_is_product(d, "PLG591", 6)) {
		/* not support glove mode */
                ret = snprintf(buf, PAGE_SIZE, "Not Support Glove mode\n");
                return ret;
	}

	value = atomic_read(&d->glove_support);

	ret = snprintf(buf, PAGE_SIZE, "%s : %s(%d)\n", __func__,
			(value == 0 ? "OFF" : "ON"), value);

	return ret;
}

static ssize_t store_glove_state(struct device *dev,
		const char *buf, size_t count)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int value = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (synaptics_is_product(d, "PLG591", 6)) {
		/* not support glove mode */
                TOUCH_I("Not Support Glove Mode\n");
                return count;
	}

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value >= 0 && value <= 1) {
		if (atomic_read(&d->glove_support) == value)
			return count;

		atomic_set(&d->glove_support, value);
		ret = synaptics_glove_set(dev, value);
		if(ret < 0) {
			TOUCH_I("%s : failed to set glove mode\n", __func__);
			return count;
		}
                /*
		ret = touch_blocking_notifier_call(NOTIFY_GLOVE_STATE,
			&d->glove_support);
                */
		TOUCH_I("%s : %s(%d), ret = %d\n",
			__func__, (value == 0 ? "OFF" : "ON"), value, ret);
	} else {
		TOUCH_I("%s : Unknown %d\n", __func__, value);
	}

	return count;
}

static ssize_t show_glove_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int fd = 0;
	char *fname = "/data/touch/touch_glove_test.txt";
	char file_str[256] ={0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int ret=0;
	u8 object_to_read = 0;
	u8 finger_cnt = 0;
	u8 i=0;
	struct synaptics_object objects[MAX_NUM_OF_FINGERS];
	struct synaptics_object *obj;

	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);

	if(atomic_read(&d->glove_support)!=1){
		TOUCH_I("Disable Glove Mode. Can not glove test.\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Disble Glove Mode. Can not glove test.\n");
		return ret;
	}

	if (d->power_state == POWER_SLEEP) {
		TOUCH_I("LCD off!!. Can not glove test.\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not glove_test.\n");
		return ret;
	}

	d->glove_delta_max = 0;
	d->glove_delta_min = 255;

	mutex_lock(&ts->lock);
	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	if (synaptics_is_product(d, "PLG636", 6)) {
		ret = synaptics_get_object_count(dev, &object_to_read);
		TOUCH_I("object_cnt=%d, ret=%d\n", object_to_read,ret);
		if(object_to_read > 0){
			ret = synaptics_read(dev, FINGER_DATA_REG,
					     objects, sizeof(*obj) * object_to_read);
			if (ret < 0) {
				TOUCH_E("faied to read finger data\n");
				mutex_unlock(&ts->lock);
				return ret;
			}

			for(i=0; i<object_to_read; i++){
				obj = objects + i;
				if (obj->type == F12_FINGER_STATUS || obj->type == F12_GLOVED_FINGER_STATUS)
					finger_cnt++;
			}
		}
		else if (object_to_read < 0) {
			TOUCH_E("faied to read object count\n");
			mutex_unlock(&ts->lock);
			return ret;
		}
		TOUCH_I("finger_cnt=%d, ret=%d\n", finger_cnt,ret);

		set_fs(KERNEL_DS);

		if (fname) {
			fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
			sys_chmod(fname, 0666);
		} else {
			TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
			set_fs(old_fs);
			mutex_unlock(&ts->lock);
			return 0;
		}

		if (fd >= 0) {
			ret = snprintf(file_str, sizeof(file_str), "\n======== delta ========\n");
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1),&my_date);
			ret += snprintf(file_str+ret, sizeof(file_str)-ret, "[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);

			ret = F54Test(dev, eRT_Normalized16BitImageReport, 1, buf);
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "Tx line of glove box = %d ~ %d\nRx line of glove box = %d ~ %d\n",
				       d->glove_box_point[0], d->glove_box_point[1], d->glove_box_point[2], d->glove_box_point[3]);
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "glove_delta_max: %d, glove_delta_min: %d\n", d->glove_delta_max, d->glove_delta_min);

			sys_write(fd, file_str, strlen(file_str));
			sys_write(fd, buf, strlen(buf));
			sys_close(fd);
		} else {
			TOUCH_E("File open failed. fd = %d\n", fd);
			set_fs(old_fs);
			mutex_unlock(&ts->lock);
			return 0;
		}
		set_fs(old_fs);

		ret = snprintf(buf, PAGE_SIZE, "%s, max=%d, min=%d\n",(finger_cnt==1 ? "Pass" : "Fail"), d->glove_delta_max, (d->glove_delta_min <0 ? 0 : d->glove_delta_min));
	} else {
		TOUCH_E("Do not support glove mode\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
		mutex_unlock(&ts->lock);
		return ret;
	}
	mutex_unlock(&ts->lock);

	return ret;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(pen_support, show_pen_support, NULL);
static TOUCH_ATTR(ts_noise, show_check_noise, store_check_noise);
static TOUCH_ATTR(ts_noise_log_enable, show_noise_log, store_noise_log);
static TOUCH_ATTR(glove_support, show_glove_state, store_glove_state);
static TOUCH_ATTR(glove_test, show_glove_test, NULL);

static struct attribute *s3330_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_pen_support.attr,
	&touch_attr_ts_noise.attr,
	&touch_attr_ts_noise_log_enable.attr,
	&touch_attr_glove_support.attr,
	&touch_attr_glove_test.attr,
	NULL,
};

static const struct attribute_group s3330_attribute_group = {
	.attrs = s3330_attribute_list,
};

static int synaptics_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &s3330_attribute_group);
	if (ret < 0)
		TOUCH_E("s3330 sysfs register failed\n");

	s3330_prd_register_sysfs(dev);

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
	TOUCH_I("%s : cmd %d\n", __func__, cmd);

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
	struct synaptics_version *device = &d->fw.info;
	struct synaptics_version *binary = NULL;
	int update = 0;
	int mfts_mode = lge_get_mfts_mode();

	TOUCH_TRACE();

	memcpy(d->fw.img_product_id, &fw->data[d->fw.fw_pid_addr], 6);
	memcpy(d->fw.img_version, &fw->data[d->fw.fw_ver_addr], 4);
	d->fw.img_info.major = (d->fw.img_version[3] & 0x80 ? 1 : 0);
	d->fw.img_info.minor = (d->fw.img_version[3] & 0x7F);
	binary = &d->fw.img_info;

	TOUCH_I("img_product_id : %s\n", d->fw.img_product_id);

	if (mfts_mode == 1) {
		TOUCH_I("need not f/w upgrade in mfts_mode\n");
		return 0;
	}

	if (ts->force_fwup) {
		update = 1;
	} else if (binary->major != device->major) {
			update = 1;
	} else {
		if (binary->minor != device->minor)
			update = 1;
	}

	TOUCH_I("%s : binary[%d.%02d] device[%d.%02d]" \
		" -> update: %d, force: %d\n", __func__,
		binary->major, binary->minor,
		device->major, device->minor,
		update, ts->force_fwup);

	return update;
}

static int synaptics_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int update = 0;
	char fwpath[256] = {0};
	const struct firmware *fw = NULL;

	TOUCH_TRACE();

	if (!ts->force_fwup) {
		ts->test_fwpath[0] = '\0';
		d->fw.def_fw = ts->def_fwpath[0];
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n",
				&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, d->fw.def_fw, sizeof(fwpath));
		TOUCH_I("ic_product_id = %s\n", d->fw.product_id);
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
	update = synaptics_fw_compare(dev, fw);

	if (update) {
		d->need_scan_pdt = true;
		ret = FirmwareUpgrade(dev, fw);

		if (ret < 0) {
			release_firmware(fw);
			return ret;
		}
	} else {
		TOUCH_I("need not fw version upgrade\n");
	}

	release_firmware(fw);

	return 0;
}

static int synaptics_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate synaptics data\n");
		return -ENOMEM;
	}

	mutex_init(&d->io_lock);

	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	synaptics_init_tci_info(dev);
	pm_qos_add_request(&d->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	d->need_scan_pdt = true;
	d->fw.fw_pid_addr = 0x8f4;
	d->fw.fw_ver_addr = 0x8f0;

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


#define MATCH_NAME			"synaptics,s3330"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = touch_match_ids,
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();

#ifndef CONFIG_LGE_TOUCH_SYNAPTICS_S3330_LV9
	if (lge_get_panel_type() != PH2_JDI && lge_get_panel_type() != LV9_JDI_NT35596)
		return 0;
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
