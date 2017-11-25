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
#include <soc/qcom/lge/board_lge.h>


/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_td4300.h"
#include "touch_td4300_prd.h"
#include "touch_td4300_f54_test.h"

/*
 * PLG610 - LEMON/L5
 */

const char *f_str[] = {
	"ERROR",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME",
	"PALM STATE",
	"ACTIVE AREA",
	"TAP COUNT"
};

#define USE_DEBUG_I2C_LOG		0
#define UB_I2C_ADDR			0x2c

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

#if USE_DEBUG_I2C_LOG
static int i2c_log = 0;
static int page_num = 0;
#endif

int synaptics_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

#if USE_DEBUG_I2C_LOG
	if(i2c_log) {
		if (addr != 0xff)
			printk("[Touch_I2C] READ [0x%02x%02x] => %d bytes\n", page_num, addr, size);
	}
#endif

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);

	mutex_unlock(&d->rw_lock);

	return 0;
}

int synaptics_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

#if USE_DEBUG_I2C_LOG
	if(i2c_log) {
		if (addr == 0xff) {
			page_num = ((u8*)data)[0];
		}
		else {
			printk("[Touch_I2C] WRITE [0x%02x%02x] <= ", page_num, addr);
			for (ret = 0 ; ret < size ; ret ++) {
				printk("[0x%02x]", ((u8*)data)[ret]);
			}
			printk("\n");
		}
	}
#endif

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size+1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	mutex_unlock(&d->rw_lock);
	return 0;
}

int synaptics_read_r(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct i2c_client *i2c = to_i2c_client(dev);
	struct touch_bus_msg msg;
	int ret = 0;
	u8 ori_addr;

	mutex_lock(&d->rw_lock);

	ori_addr = i2c->addr;
	i2c->addr = UB_I2C_ADDR;

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		i2c->addr = ori_addr;
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);

	i2c->addr = ori_addr;
	mutex_unlock(&d->rw_lock);
	return 0;
}

int synaptics_write_r(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct i2c_client *i2c = to_i2c_client(dev);
	struct touch_bus_msg msg;
	int ret = 0;
	u8 ori_addr;

	mutex_lock(&d->rw_lock);


	ori_addr = i2c->addr;
	i2c->addr = UB_I2C_ADDR;

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size+1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		i2c->addr = ori_addr;
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	i2c->addr = ori_addr;
	mutex_unlock(&d->rw_lock);
	return 0;
}


int synaptics_read_p(struct device *dev, u8 page, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	// Select Page
	ts->tx_buf[0] = PAGE_SELECT_REG;
	ts->tx_buf[1] = page;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 2;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);
	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	// Read data from addr
	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);

	mutex_unlock(&d->rw_lock);
	return 0;
}

int synaptics_write_p(struct device *dev, u8 page, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->rw_lock);

	// Select Page
	ts->tx_buf[0] = PAGE_SELECT_REG;
	ts->tx_buf[1] = page;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 2;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);
	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	// Write data to addr
	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size+1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		mutex_unlock(&d->rw_lock);
		return ret;
	}

	mutex_unlock(&d->rw_lock);
	return 0;
}

static int synaptics_irq_enable(struct device *dev, bool enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val;
	int ret;

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

u8 synaptics_read_reg(struct device *dev, u8 page, u16 addr)
{
	u8 data;
	int ret;

	synaptics_set_page(dev, page);

	ret = synaptics_read(dev, addr, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		synaptics_set_page(dev, DEFAULT_PAGE);
		return 0;
	}

	TOUCH_I("Reg [0x%02X%02X] = [0x%02X]\n", page, addr, data);

	synaptics_set_page(dev, DEFAULT_PAGE);

	return data;
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

	ret = synaptics_set_page(dev, 0);

	if (ret) {
		TOUCH_E("faied to set page %d (ret: %d)\n", 0, ret);
		return ret;
	}

	return 0;
}

static int synaptics_get_product_id(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
//	u8 temp_pid[11] = {0,};

	TOUCH_TRACE();

	ret = synaptics_read(dev, PRODUCT_ID_REG,
			d->fw.product_id, sizeof(d->fw.product_id) - 1);

	if (ret < 0) {
		TOUCH_I("[%s]read error...\n", __func__);
		d->is_pid_read = 0;
		return ret;
	}

	d->is_pid_read = 1;

	TOUCH_I("[%s] IC_product_id: %s\n",
			__func__, d->fw.product_id);

	return 0;
}

int synaptics_ic_info(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret;

	if (d->need_scan_pdt == true) {
		ret = synaptics_page_description(dev);
		//SCAN_PDT(dev); // Needless since F54.c not used
		d->need_scan_pdt = false;
	}

	synaptics_set_page(dev, DEFAULT_PAGE);

	ret = synaptics_get_product_id(dev);
	ret = synaptics_read(dev, FLASH_CONFIG_ID_REG,
			d->fw.version, sizeof(d->fw.version) - 1);
	ret = synaptics_read(dev, CUSTOMER_FAMILY_REG,
			&(d->fw.family), sizeof(d->fw.family));
	ret = synaptics_read(dev, FW_REVISION_REG,
			&(d->fw.revision), sizeof(d->fw.revision));

	if (ret < 0) {
		TOUCH_I("[%s]read error...\n", __func__);
		return ret;
	}

	TOUCH_I("ic_version = V%d.%02d\n", d->fw.version[3] & 0x80 ? 1 : 0,
			d->fw.version[3] & 0x7F);
	TOUCH_I("CUSTOMER_FAMILY_REG = %d\n", d->fw.family);
	TOUCH_I("FW_REVISION_REG = %d\n", d->fw.revision);
	TOUCH_I("PRODUCT ID = %s\n", d->fw.product_id);

	return 0;
}

int synaptics_set_configured(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 data;
	int ret;

	data = 0x80; // Set Configured
	ret = synaptics_write(dev, DEVICE_CONTROL_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
	}

	return ret;
}

int synaptics_set_lpwg_pmic_delay(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 data;
	int ret;

	// This reg.(0x0464) is available only for PLG610 since f/w version V0.21
	// need to be changed from 0x28 to 0x14 for only LEMON/L5 (PMIC En H-period get shorter)
	// need not FORCE UPDATE since f/w version V1.06

	ret = synaptics_set_page(dev, LPWG_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto RET;
	}

	ret = synaptics_read(dev, LPWG_PMIC_DELAY_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto RET;
	}

	if (data == 0x14) {
		TOUCH_I("Need not to set LPWG PMIC Delay (Already applied)\n");
		goto RET;
	}
	else if (data != 0x28) {
		TOUCH_I("Cannot set LPWG PMIC Delay (Not applicable F/W)\n"); // F51_CUSTOM_QUERY00(0x08)
		goto RET;
	}

	data = 0x14;
	ret = synaptics_write(dev, LPWG_PMIC_DELAY_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto RET;
	}

	TOUCH_I("Set LPWG PMIC Delay !!\n");

RET:

	synaptics_set_page(dev, DEFAULT_PAGE);

	return ret;
}

int synaptics_force_update(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret, retry = 0;
	u8 data;

	//TOUCH_I("synaptics_force_update\n");

	ret = synaptics_set_page(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	data = 0x04; // Force Update
	ret = synaptics_write(dev, ANALOG_COMMAND_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	// Waiting for update complete
	do {
		touch_msleep(5);
		ret = synaptics_read(dev, ANALOG_COMMAND_REG, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		if ((data & 0x04) == 0x00) // Force update bit cleared
			break;
	} while ((retry++) < 40);

	if (retry >= 40) {
		TOUCH_E("force_update time out!!\n");
		ret = -EPERM;
		goto FAIL;
	}
	else {
		TOUCH_I("force_update complete : %d ms\n", (retry+1)*5);
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	return ret;

FAIL:

	synaptics_set_page(dev, DEFAULT_PAGE);

	return ret;
}

int synaptics_dynamic_sensing_ctrl(struct device *dev, int value)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 data;

	TOUCH_I("synaptics_dynamic_sensing_ctrl : %d\n", value);

	ret = synaptics_set_page(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	data = value << 1; // Dynamic Sensing Select
	ret = synaptics_write(dev, DYNAMIC_SENSING_CTRL_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		synaptics_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	// Force update
	ret = synaptics_force_update(dev);
	if (ret < 0) {
		TOUCH_E("Force update failed\n");
		synaptics_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	return ret;
}

int synaptics_set_no_sleep(struct device *dev, int mode)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val;
	int ret;

	ret = synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	ret = synaptics_read(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read device control - ret:%d\n", ret);
		return ret;
	}

	val &= 0xfb; // ~(0x04)

	if (mode)
		val |= 0x04;
	else
		val |= 0;

	ret = synaptics_write(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write device control - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - mode:%d\n", __func__, mode);

	return 0;
}

static int synaptics_sleep_control(struct device *dev, u8 mode)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val;
	int ret;

	ret = synaptics_read(dev, DEVICE_CONTROL_REG, &val, sizeof(val));

	if (ret < 0) {
		TOUCH_E("failed to read device control - ret:%d\n", ret);
		return ret;
	}

	val &= 0xf8;

	if (mode)
		val |= 1;
	else
		val |= 0;

	ret = synaptics_write(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write device control - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - mode:%d\n", __func__, mode);

	return 0;
}

int synaptics_lpwg_debug(struct device *dev, int num)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 count = 0;
	u8 index = 0;
	u8 buf = 0;
	u8 i = 0;
	u8 addr = 0;
	u8 offset = num ? LPWG_MAX_BUFFER + 2 : 0;

	synaptics_set_page(dev, LPWG_PAGE);

	synaptics_read(dev, LPWG_TCI1_FAIL_COUNT_REG + offset, &count, sizeof(count));
	synaptics_read(dev, LPWG_TCI1_FAIL_INDEX_REG + offset, &index, sizeof(index));

	for (i = 1; i <= count; i++) {
		addr = LPWG_TCI1_FAIL_BUFFER_REG + offset +
			((index + LPWG_MAX_BUFFER - i) % LPWG_MAX_BUFFER);
		synaptics_read(dev, addr, &buf, sizeof(buf));
		TOUCH_I("TCI(%d)-Fail[%d/%d] : %s (%d)\n", num, count - i + 1, count,
			(buf > 0 && buf < 9) ? f_str[buf] : f_str[0], buf);

		if (i == LPWG_MAX_BUFFER)
			break;
	}

	synaptics_set_page(dev, DEFAULT_PAGE);

	return 0;
}

static int synaptics_tci_report_enable(struct device *dev, bool enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val[3];
	int ret;

	ret = synaptics_read(dev, FINGER_REPORT_REG, val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read finger report enable - ret:%d\n", ret);
		return ret;
	}

	val[2] &= 0xfc;

	if (enable)
		val[2] |= 0x2;

	if (enable)
		synaptics_irq_enable(dev, false);

	ret = synaptics_write(dev, FINGER_REPORT_REG, val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write finger report enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - enable:%d\n", __func__, enable);

	if (!enable)
		synaptics_irq_enable(dev, true);

	return 0;
}

static int synaptics_tci_clear(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 status;
	u8 buffer;
	int ret;

	ret = synaptics_set_page(dev, LPWG_PAGE);
	if (ret < 0)
		return ret;

	ret = synaptics_read(dev, LPWG_STATUS_REG, &status, 1);
	if (ret < 0)
		return ret;

	if (status & 0x03) {
		ret = synaptics_read(dev, LPWG_DATA_REG, &buffer, 1);
		if (ret < 0)
			return ret;
		TOUCH_I("TCI Cleared!!\n");
	}

	return synaptics_set_page(dev, DEFAULT_PAGE);
}


#define TCI_1_INT_DELAY		68		// 680ms for double tab check
#define TCI_2_INT_DELAY		30		// 300ms for overtab (680~700ms??)

static int synaptics_tci_knock(struct device *dev, u8 value)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct tci_info *info;

	u8 lpwg_data[7];
	int ret;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	TOUCH_I("0 : %d,%d,%d,%d,%d,%d,%d\n",
		lpwg_data[0],
		lpwg_data[1],
		lpwg_data[2],
		lpwg_data[3],
		lpwg_data[4],
		lpwg_data[5],
		lpwg_data[6]);

	info = &ts->tci.info[0];

	info->tap_count &= 0x0f;
	info->intr_delay = 0;
	info->tap_distance = 10;

	lpwg_data[0] = ((info->tap_count << 3) | (value ? 1 : 0));
	lpwg_data[1] = info->min_intertap;
	lpwg_data[2] = info->max_intertap;
	lpwg_data[3] = info->touch_slop;
	lpwg_data[4] = info->tap_distance;
	lpwg_data[6] = (info->intr_delay << 1 | 0);

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG2,
			      &lpwg_data[0], sizeof(u8));

	TOUCH_I("1 : %d\n", lpwg_data[0]);

	lpwg_data[0] &= 0xfe;

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG2,
			      lpwg_data, sizeof(u8));
	if (ret < 0) {
		TOUCH_E("failed to write LPWG_TAPCOUNT_REG2\n");
		return ret;
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
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct tci_info *info;

	u8 lpwg_data[7];
	int ret;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	TOUCH_I("0: %d,%d,%d,%d,%d,%d,%d\n",
		lpwg_data[0],
		lpwg_data[1],
		lpwg_data[2],
		lpwg_data[3],
		lpwg_data[4],
		lpwg_data[5],
		lpwg_data[6]);

	info = &ts->tci.info[0];

	info->tap_count &= 0x0f;
	info->intr_delay = ts->tci.double_tap_check ? TCI_1_INT_DELAY : 0;
	info->tap_distance = 7;

	lpwg_data[0] = ((info->tap_count << 3) | 1);
	lpwg_data[1] = info->min_intertap;
	lpwg_data[2] = info->max_intertap;
	lpwg_data[3] = info->touch_slop;
	lpwg_data[4] = info->tap_distance;
	if(info->intr_delay == 0)
		lpwg_data[6] = (info->intr_delay << 1);
	else
		lpwg_data[6] = (info->intr_delay << 1 | 1);

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG2,
			      lpwg_data, sizeof(lpwg_data));

	TOUCH_I("1: %d,%d,%d,%d,%d,%d,%d\n",
		lpwg_data[0],
		lpwg_data[1],
		lpwg_data[2],
		lpwg_data[3],
		lpwg_data[4],
		lpwg_data[5],
		lpwg_data[6]);

	info = &ts->tci.info[1];
	info->tap_count &= 0x0f;
	lpwg_data[0] = ((info->tap_count << 3) | 1);
	lpwg_data[1] = info->min_intertap;
	lpwg_data[2] = info->max_intertap;
	lpwg_data[3] = info->touch_slop;
	lpwg_data[4] = info->tap_distance;
	lpwg_data[6] = (info->intr_delay << 1 | 1);

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG2,
			      lpwg_data, sizeof(lpwg_data));

	if (ret < 0) {
		TOUCH_E("failed to write LPWG_TAPCOUNT_REG2\n");
		return ret;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	return ret;
}

// Done in F/W
#if 0
static int synaptics_tci_active_area(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 buffer[8];

#if 0
	buffer[0] = (0 >> 0) & 0xff;
	buffer[1] = (0 >> 8) & 0xff;
	buffer[2] = (0 >> 0) & 0xff;
	buffer[3] = (0 >> 8) & 0xff;
	buffer[4] = (1080 >> 0) & 0xff;
	buffer[5] = (1080 >> 8) & 0xff;
	buffer[6] = (1920 >> 0) & 0xff;
	buffer[7] = (1920 >> 8) & 0xff;
#else
	buffer[0] = (ts->lpwg.area[0].x >> 0) & 0xff;
	buffer[1] = (ts->lpwg.area[0].x >> 8) & 0xff;
	buffer[2] = (ts->lpwg.area[0].y >> 0) & 0xff;
	buffer[3] = (ts->lpwg.area[0].y >> 8) & 0xff;
	buffer[4] = (ts->lpwg.area[1].x >> 0) & 0xff;
	buffer[5] = (ts->lpwg.area[1].x >> 8) & 0xff;
	buffer[6] = (ts->lpwg.area[1].y >> 0) & 0xff;
	buffer[7] = (ts->lpwg.area[1].y >> 8) & 0xff;
#endif

	synaptics_write(dev, d->f12_reg.ctrl[18], buffer, sizeof(buffer));

	return 0;
}
#endif

static int synaptics_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_I("synaptics_lpwg_control mode = %d\n", mode);

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		synaptics_tci_clear(dev); // When TCI int was not processed in previous(data was not read), TCI is pending state (f/w behavior), so we need dummy-reading
		ts->tci.mode = 0x01;
		synaptics_tci_knock(dev, ts->tci.mode);
		break;

	case LPWG_PASSWORD:
		synaptics_tci_clear(dev); // When TCI int was not processed in previous(data was not read), TCI is pending state (f/w behavior), so we need dummy-reading
		ts->tci.mode = 0x02;
		synaptics_tci_password(dev);
		break;

	default:
		ts->tci.mode = 0;
		synaptics_tci_knock(dev, ts->tci.mode);
		break;
	}

	return 0;
}

int synaptics_cover_control(struct device *dev, int mode)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret;
	u8 data;

	TOUCH_I("synaptics_cover_control mode = %d\n", mode);

	switch (mode) {
	case QUICKCOVER_CLOSE:
		ret = synaptics_set_page(dev, DEFAULT_PAGE);
		data = 0x02;
		ret = synaptics_write(dev, FEATURE_EN_REG, &data, 1);
		break;

	case QUICKCOVER_OPEN:
	default:
		ret = synaptics_set_page(dev, DEFAULT_PAGE);
		data = 0x00;
		ret = synaptics_write(dev, FEATURE_EN_REG, &data, 1);
		break;

	}
	return ret;
}


#define USE_DSV_TOGGLE_MODE			1
#define USE_HW_RESET				0
#define USE_TCI_DEBUG				1
#define LPWG_OFF_DELAY				55		// 50
#define LPWG_SCAN_DELAY				220

static int synaptics_lpwg_mode(struct device *dev, int screen_mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 next_state;
	int ret = 0, mfts_mode = 0;

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		return 0;
	}

	TOUCH_I(
		"synaptics_lpwg_mode [%s]: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
		screen_mode ? "ON" : "OFF",
		ts->lpwg.mode, 
		ts->lpwg.screen ? "ON" : "OFF",
		ts->lpwg.sensor ? "FAR" : "NEAR",
		ts->lpwg.qcover ? "CLOSE" : "OPEN");

#if 0
	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Not Ready, Need IC init\n");
		return 0;
	}
#endif

	// Check MFTS mode to use POWER_OFF state (No use PMIC En)
	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		if (d->state == TC_STATE_ACTIVE && screen_mode == 0) {
			d->esd_recovery_en = 0;
			next_state = TC_STATE_POWER_OFF;
			touch_interrupt_control(dev, INTERRUPT_DISABLE);
			TOUCH_I("Call DSV OFF\n"); // DSV
			LCD_DSV_OFF; // DSV OFF(Disable) in case of MFTS in Display
			touch_msleep(22); // 10 + 10
			TOUCH_I("Call LCD RESET 0\n"); // LCD Reset
			LCD_RESET_L;
			touch_msleep(1);
			touch_gpio_direction_output(ts->reset_pin, 0); // Touch Reset
			touch_msleep(11); // 10
			TOUCH_I("Call VDDIO OFF\n"); // VDDIO
			LCD_VDDI_OFF;
			touch_msleep(11);
		} else if (d->state == TC_STATE_POWER_OFF && screen_mode == 1) {
			next_state = TC_STATE_ACTIVE;
			d->esd_recovery_en = 0;
			touch_gpio_direction_output(ts->reset_pin, 0); // Ensure Reset is LOW
			LCD_RESET_L;
			touch_msleep(1);
			TOUCH_I("Call VDDIO ON\n"); // VDDIO
			LCD_VDDI_ON;
			touch_msleep(11); // 10
			TOUCH_I("Call DSV ON\n"); // DSV
			LCD_DSV_ON; // DSV Enable (Always ON in POR)
			touch_msleep(33); // 10 + 10 + 10
			touch_gpio_direction_output(ts->reset_pin, 1); // Touch Reset
			touch_msleep(1);
			TOUCH_I("Call LCD RESET 1\n"); // LCD Reset
			LCD_RESET_H;
			touch_msleep(165); // 150
			// Since f/w version is running can be changed, we need to re-scan pdt immediately to do something.
			// Scanning pdt can be done twice(here and resume-init), but we don't care it.
			synaptics_page_description(dev);
			touch_interrupt_control(dev, INTERRUPT_ENABLE);
			synaptics_irq_clear(dev);
		} else
			next_state = d->state;
		goto RET;
	}

	// NORMAL Case
	if (d->state == TC_STATE_ACTIVE) {
		if (screen_mode == 0) {
			if (ts->lpwg.sensor == 1 && ts->lpwg.mode != LPWG_NONE && ts->lpwg.qcover == 0) {
				next_state = TC_STATE_LPWG;
#if USE_DSV_TOGGLE_MODE
				TOUCH_I("Call DSV OFF\n");
				LCD_DSV_OFF;
#endif
				ret = synaptics_set_lpwg_pmic_delay(dev); // For only LEMON/l5
				//ret = synaptics_sleep_control(dev, 0); // Need not (Sleep status is cleared after scan start)
				ret = synaptics_lpwg_control(dev, ts->lpwg.mode);
				ret = synaptics_tci_report_enable(dev, true);
				touch_msleep(LPWG_SCAN_DELAY); // Delay to start scan (80/150~180ms)
				//ret = synaptics_force_update(dev); // Need not since all F51 config is dynamic (V1.06)
			}
			else {
				next_state = TC_STATE_DEEP_SLEEP;
#if USE_DSV_TOGGLE_MODE
				TOUCH_I("Call DSV OFF\n");
				LCD_DSV_OFF;
#endif
				ret = synaptics_set_lpwg_pmic_delay(dev); // For only LEMON/l5
				//ret = synaptics_sleep_control(dev, 0); // Need not (Sleep status is cleared after scan start)
				ret = synaptics_lpwg_control(dev, LPWG_NONE);
				ret = synaptics_tci_report_enable(dev, true);
				touch_msleep(LPWG_SCAN_DELAY); // Delay to start scan (80/150~180ms)
				ret = synaptics_sleep_control(dev, 1);
#if !USE_DSV_TOGGLE_MODE
				TOUCH_I("Call DSV OFF\n");
				LCD_DSV_OFF;
#endif
			}
		}
		else {
			next_state = TC_STATE_ACTIVE; // Do nothing
		}
	}
	else if (d->state == TC_STATE_LPWG) {
		if (screen_mode == 0) {
			if (ts->lpwg.sensor == 0 || ts->lpwg.mode == LPWG_NONE || ts->lpwg.qcover != 0) {
				next_state = TC_STATE_DEEP_SLEEP;
				ret = synaptics_lpwg_control(dev, LPWG_NONE);
				ret = synaptics_tci_report_enable(dev, true);
				ret = synaptics_sleep_control(dev, 1);
#if !USE_DSV_TOGGLE_MODE
				TOUCH_I("Call DSV OFF\n");
				LCD_DSV_OFF;
#endif
			}
			else {
				next_state = TC_STATE_LPWG; // Do nothing
			}
		}
		else {
			next_state = TC_STATE_ACTIVE;
#if USE_DSV_TOGGLE_MODE
			TOUCH_I("Call DSV ON\n");
			LCD_DSV_ON;
			touch_msleep(12);
#endif
#if USE_HW_RESET
			ret = synaptics_hw_reset(dev);
#else
			ret = synaptics_lpwg_control(dev, LPWG_NONE);
			ret = synaptics_tci_report_enable(dev, false);
#if USE_TCI_DEBUG
			ret = synaptics_lpwg_debug(dev, 0);
			ret = synaptics_lpwg_debug(dev, 1);
#endif
			touch_msleep(LPWG_OFF_DELAY);
#endif
		}
	} 
	else if (d->state == TC_STATE_DEEP_SLEEP) {
		if (screen_mode == 0) {
			if (ts->lpwg.sensor == 1 && ts->lpwg.mode != LPWG_NONE && ts->lpwg.qcover == 0) {
				next_state = TC_STATE_LPWG;
#if !USE_DSV_TOGGLE_MODE
				TOUCH_I("Call DSV ON\n");
				LCD_DSV_ON;
				touch_msleep(12);
#endif
				ret = synaptics_set_lpwg_pmic_delay(dev); // For only LEMON/l5
				ret = synaptics_sleep_control(dev, 0);
				ret = synaptics_lpwg_control(dev, ts->lpwg.mode);
				ret = synaptics_tci_report_enable(dev, true);
				//ret = synaptics_force_update(dev); // Need not since all F51 config is dynamic (V1.06)
			}
			else {
				next_state = TC_STATE_DEEP_SLEEP; // Do nothing
			}
		}
		else {
			next_state = TC_STATE_ACTIVE;
#if USE_DSV_TOGGLE_MODE || !USE_DSV_TOGGLE_MODE
			TOUCH_I("Call DSV ON\n");
			LCD_DSV_ON;
			touch_msleep(12);
#endif
#if USE_HW_RESET
			ret = synaptics_hw_reset(dev);
#else
			ret = synaptics_sleep_control(dev, 0);
			ret = synaptics_lpwg_control(dev, LPWG_NONE);
			ret = synaptics_tci_report_enable(dev, false);
#if USE_TCI_DEBUG
			ret = synaptics_lpwg_debug(dev, 0);
			ret = synaptics_lpwg_debug(dev, 1);
#endif
			touch_msleep(LPWG_OFF_DELAY);
#endif
		}
	}
	else {
		next_state = d->state;
	}

	if (next_state == TC_STATE_ACTIVE && d->cover_state != ts->lpwg.qcover) {
#ifdef CONFIG_MACH_MSM8937_L5_DCM_JP
		// Cover Setting for L5 Only
		ret = synaptics_cover_control(dev, ts->lpwg.qcover ? QUICKCOVER_CLOSE : QUICKCOVER_OPEN);
#endif
		d->cover_state = ts->lpwg.qcover;
	}

RET:

	TOUCH_I("State changed from [%d] to [%d]\n", d->state, next_state);

	d->state = next_state;

	return ret;
}

int synaptics_init_reg(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
#ifdef CONFIG_MACH_MSM8937_L5_DCM_JP
	struct synaptics_data *d = to_synaptics_data(dev);
#endif
	int ret = 0;

	ret = synaptics_set_lpwg_pmic_delay(dev); // Set is done in LPWG entering, too (no need force update since V1.06)

#ifdef CONFIG_MACH_MSM8937_L5_DCM_JP
	// Cover Setting for L5 Only
	if (d->cover_state == QUICKCOVER_CLOSE) {
		ret |= synaptics_cover_control(dev, QUICKCOVER_CLOSE);
	}		
#endif

	return ret;
}

static void synaptics_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[0].tap_count = 2;
	ts->tci.info[0].min_intertap = 6;
	ts->tci.info[0].max_intertap = 70;
	ts->tci.info[0].touch_slop = 100;
	ts->tci.info[0].tap_distance = 10;
	ts->tci.info[0].intr_delay = 0;

	ts->tci.info[1].min_intertap = 6;
	ts->tci.info[1].max_intertap = 70;
	ts->tci.info[1].touch_slop = 100;
	ts->tci.info[1].tap_distance = 255;
	ts->tci.info[1].intr_delay = TCI_2_INT_DELAY;
}

static int synaptics_remove(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	d->esd_recovery_en = 0;

	if (rmidev_fhandler.initialized
		&& rmidev_fhandler.inserted) {
		rmidev_fhandler.exp_fn->remove(dev);
		rmidev_fhandler.initialized = false;
	}

	cancel_delayed_work_sync(&d->monitor_work);
	cancel_delayed_work_sync(&d->esd_recovery_work);

	return 0;
}

static int synaptics_get_status(struct device *dev, u8 *device, u8 *interrupt)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 dev_status;
	u8 irq_status;
	int ret;

	ret = synaptics_read(dev, DEVICE_STATUS_REG,
			&dev_status, sizeof(dev_status));

	if (ret < 0) {
		TOUCH_E("failed to read device status - ret:%d\n", ret);

		touch_msleep(100);

		ret = synaptics_read(dev, DEVICE_STATUS_REG,
				&dev_status, sizeof(dev_status));

		if (ret < 0) {
			TOUCH_E("failed to read device status AGAIN - ret:%d\n", ret);
			return ret;
		}
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

#define ESD_RECOVERY_WORK_DELAY		1

int synaptics_irq_status(struct device *dev, u8 status)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	if (status) {
		TOUCH_I("Status Interrupt [0x%02X] => Set configured!!!\n", status);
		synaptics_set_configured(dev);
		touch_msleep(30);
	}
	else {
		TOUCH_I("Status Interrupt [0x%02X]!!!\n", status);
		return 0;
	}

	if (d->esd_recovery_en) {
		if (status == 0x81) { // Unknown reset
			TOUCH_I("Status Interrupt [Reset] => ESD Recovery!!!\n");
			schedule_delayed_work(&d->esd_recovery_work, msecs_to_jiffies(ESD_RECOVERY_WORK_DELAY));
		}
		else if ((status & 0x0F) == 0x09) { // Display Failure
			TOUCH_I("Status Interrupt [Display Failure] => ESD Recovery!!!\n");
			schedule_delayed_work(&d->esd_recovery_work, msecs_to_jiffies(ESD_RECOVERY_WORK_DELAY));
		}
	}

	if ((status & 0xC0) == 0x80) {
		TOUCH_I("Init Regs.!!!\n");
		synaptics_init_reg(dev);
	}

	return 0;
}

int synaptics_irq_clear(struct device *dev)
{
	u8 dev_status = 0;
	u8 irq_status = 0;
	int ret;

	ret = synaptics_get_status(dev, &dev_status, &irq_status);

	if (irq_status & INTERRUPT_MASK_STATUS)
		ret = synaptics_irq_status(dev, dev_status);

	return ret;

//	return synaptics_get_status(dev, NULL, NULL);
}

int synaptics_soft_reset(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 data, esd_backup;

#if 1
	if (d->state != TC_STATE_ACTIVE) {
		TOUCH_E("synaptics_soft_reset called in NOT ACTIVE!!!\n"); // Need to check
		return -EPERM;
	}
#endif

	TOUCH_I("synaptics_soft_reset\n");

	esd_backup = d->esd_recovery_en;
	d->esd_recovery_en = 0;

	data = 0x01;
	ret = synaptics_write(dev, DEVICE_COMMAND_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		d->esd_recovery_en = esd_backup;
		return ret;
	}

	touch_msleep(400);

	synaptics_irq_clear(dev); // To set configured for reset

	d->esd_recovery_en = esd_backup;

	return ret;
}

#define HW_RESET_DELAY	160 // 150+10

int synaptics_hw_reset(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 esd_backup;

	touch_interrupt_control(dev, INTERRUPT_DISABLE);

	TOUCH_I("synaptics_hw_reset\n");

	esd_backup = d->esd_recovery_en;
	d->esd_recovery_en = 0;

	TOUCH_I("%s : Display & Touch Reset\n", __func__);
	LCD_RESET_L;
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(12);
	touch_gpio_direction_output(ts->reset_pin, 1);
	LCD_RESET_H;
	touch_msleep(HW_RESET_DELAY);

	synaptics_irq_clear(dev); // To set configured for reset

	d->esd_recovery_en = esd_backup;

	touch_interrupt_control(dev, INTERRUPT_ENABLE);

	synaptics_irq_clear(dev);

	return ret;
}

#define NOISE_LOG_PERIOD_NO_CONTACT		100
#define NOISE_LOG_REPORT_COUNT			24

static int synaptics_noise_log(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 buffer[10] = {0};
	u8 cns = 0, freq_sel = 0;
	u16 im = 0, report_rate = 0/*, freq_scan_im = 0*/;
	int ret = 0;

	static u8 prev_cns, prev_freq_sel;
	static u16 prev_im, prev_report_rate, im_max = 0, im_min = U16_MAX;
	static int count = 0, im_sum = 0;

	synaptics_set_page(dev, ANALOG_PAGE);

	synaptics_read(dev, INTERFERENCE_METRIC_LSB_REG, buffer, 9);
	im = (buffer[1] << 8) | buffer[0];	// Interference Metric
	report_rate = (buffer[3] << 8) | buffer[2]; // Report Rate
	cns = buffer[4]; // Current Noise State
	//freq_scan_im = (buffer[7] << 8) | buffer[6]; // Freq Scan IM
	freq_sel = buffer[8]; // Sense Frequency Selection

	synaptics_set_page(dev, DEFAULT_PAGE);

	count++;
	im_sum += im;
	if (im > im_max)
		im_max = im;
	if (im < im_min)
		im_min = im;

	if ((ts->old_mask != 0 && ts->new_mask == 0) ||
		(ts->old_mask == 0 && ts->new_mask != 0) ||
		count >= NOISE_LOG_REPORT_COUNT ||
		cns != prev_cns || 
		freq_sel != prev_freq_sel ||
		im > (prev_im<<1) ||
		im < (prev_im>>1)) {
			TOUCH_I("=== CNS[%1d] FQ[%1d] RR[%3d] IM[%5d] MAX_IM[%5d] AVR_IM[%5d] MIN_IM[%5d] CNT[%2d]\n", cns, freq_sel, report_rate, im, im_max, im_sum/count, im_min, count);
			count = 0;
			im_sum = 0;
			im_max = 0;
			im_min = U16_MAX;
			ret = 1;
	}

	if (ts->new_mask == 0)
		schedule_delayed_work(&d->monitor_work, msecs_to_jiffies(NOISE_LOG_PERIOD_NO_CONTACT));
	else if (ts->old_mask == 0 && ts->new_mask != 0)
		cancel_delayed_work(&d->monitor_work);

	prev_cns = cns;
	prev_freq_sel = freq_sel;
	prev_report_rate = report_rate;
	prev_im = im;

	return ret;
}

static int synaptics_get_object_count(struct device *dev, u8 *object)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 object_to_read = d->num_of_fingers;
	u8 buf[2] = {0,};
	u16 object_attention = 0;
	int ret;

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

#define USE_PALM_CANCEL

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
#ifdef USE_PALM_CANCEL
	static int is_palm = 0;
#endif

	ts->new_mask = 0;
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

#ifdef USE_PALM_CANCEL
			if (obj->type == F12_PALM_STATUS) {
				if (!is_palm) {
					TOUCH_I("Palm Detected\n");
					is_palm = 1;
					ts->is_cancel = 1;
					ts->new_mask = 0;
					ts->tcount = 0;
					ts->intr_status |= TOUCH_IRQ_FINGER;
					goto RET;
				} 
				else {
					ts->new_mask = 0;
					ts->tcount = 0;
					goto RET;
				}
			}
#endif
			if (obj->type == F12_FINGER_STATUS) {
				ts->new_mask |= (1 << i);
				tdata = ts->tdata + i;

				tdata->id = i;
				tdata->type = obj->type;
				tdata->x = obj->x_lsb | obj->x_msb << 8;
				tdata->y = obj->y_lsb | obj->y_msb << 8;
#ifdef USE_PALM_CANCEL
				if (obj->z == 255)
					tdata->pressure = 254;
				else
					tdata->pressure = obj->z;
#else
				tdata->pressure = obj->z;
#endif

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

#ifdef USE_PALM_CANCEL
	if (is_palm) {
		TOUCH_I("Palm Released\n");
		is_palm = 0;
	}
#endif

	ts->intr_status |= TOUCH_IRQ_FINGER;

RET:

	if (d->noise.noise_log == NOISE_ENABLE) {
		synaptics_noise_log(dev);
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

		if (ts->role.hide_coordinate) {
			TOUCH_I("LPWG data XXX, XXX\n");
		}
		else {
			TOUCH_I("LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
		}
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
	int ret;

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0)
		return ret;

	ret = synaptics_read(dev, LPWG_STATUS_REG, &status, 1);
	if (ret < 0)
		return ret;

	if (status & LPWG_STATUS_DOUBLETAP) {
		synaptics_tci_getdata(dev, ts->tci.info[0].tap_count);
		ts->intr_status |= TOUCH_IRQ_KNOCK;
	} else if (status & LPWG_STATUS_PASSWORD) {
		synaptics_tci_getdata(dev, ts->tci.info[1].tap_count);
		ts->intr_status |= TOUCH_IRQ_PASSWD;
	} else {
		/* Overtab */
		synaptics_read(dev, LPWG_OVER_TAPCOUNT, &buffer, 1);
		if (buffer > ts->tci.info[1].tap_count) {
			ts->lpwg.code_num = 1;
			synaptics_tci_getdata(dev, ts->lpwg.code_num);
			ts->intr_status |= TOUCH_IRQ_PASSWD;
			TOUCH_I("knock code fail to over tap count = %d\n", buffer);
		}
	}

	return synaptics_set_page(dev, DEFAULT_PAGE);
}

static int synaptics_irq_handler(struct device *dev)
{
	u8 dev_status = 0;
	u8 irq_status = 0;
	int ret;

	TOUCH_TRACE();

	ret = synaptics_get_status(dev, &dev_status, &irq_status);

	if (irq_status & INTERRUPT_MASK_STATUS)
		ret = synaptics_irq_status(dev, dev_status);

	if (irq_status & INTERRUPT_MASK_ABS0)
		ret = synaptics_irq_abs(dev);

	if (irq_status & INTERRUPT_MASK_LPWG)
		ret = synaptics_irq_lpwg(dev);

	return ret;
}

void synaptics_rmidev_function(struct synaptics_exp_fn *rmidev_fn,
		bool insert)
{
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
	synaptics_ic_info(dev);

	synaptics_lpwg_mode(dev, 1);	// Screen ON

	synaptics_irq_enable(dev, true);
	synaptics_irq_clear(dev);

	synaptics_rmidev_init(dev);

	return 0;
}

static int synaptics_power(struct device *dev, int ctrl)
{
	//struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off (Actually Do Nothing)\n", __func__);
		//touch_gpio_direction_output(ts->reset_pin, 0);
		//touch_power_vio(dev, 0);
		//touch_power_vdd(dev, 0);
		break;

	case POWER_ON:
		TOUCH_I("%s, on (Actually Do Nothing)\n", __func__);
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
	}

	return 0;
}

static int synaptics_suspend(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		return -EPERM;
	}

	return 0;
}

static int synaptics_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		// Activate IC is done in fb callback EARLY BLANK (POWER_OFF => ACTIVE)
		// Return 0 to do init in touch_resume
		if (d->state == TC_STATE_ACTIVE) {
			d->need_scan_pdt = true;
			return 0;
		}			
		else {
			return -EPERM;
		}		
	}
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		/* Deep Sleep */
		synaptics_sleep_control(dev, 1);
		return -EPERM;
	}

	return -EPERM;
}

static int synaptics_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int *value = (int *)param;

	mutex_lock(&d->fb_lock);	// fb lock is done in fb noti callback
	mutex_lock(&ts->lock);		// ts lock is removed in touch core

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
		//synaptics_tci_active_area(dev); // Just use default value applied lpwg active area
		break;

	case LPWG_TAP_COUNT:
		ts->tci.info[1].tap_count = value[0];
		TOUCH_I("LPWG_TAP_COUNT: [%d]\n", ts->tci.info[1].tap_count);
		break;

	case LPWG_DOUBLE_TAP_CHECK:
		ts->tci.double_tap_check = value[0];
		TOUCH_I("LPWG_DOUBLE_TAP_CHECK: [%d]\n", ts->tci.double_tap_check);
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

		// Screen change is done in fb callback, not here
		if((ts->lpwg.screen == 1 && d->state != TC_STATE_ACTIVE) || (ts->lpwg.screen == 0 && d->state == TC_STATE_ACTIVE)) {
			TOUCH_I("Screen state changed(but, not updated yet) => Ignore LPWG_UPDATE_ALL\n");
		}
		else {
			synaptics_lpwg_mode(dev, ts->lpwg.screen);
		}
		break;
	}

	mutex_unlock(&ts->lock);	// ts lock is removed in touch core
	mutex_unlock(&d->fb_lock); 	// fb lock is done in fb noti callback

	return 0;
}

static int synaptics_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;
	struct synaptics_data *d = container_of(self, struct synaptics_data, fb_notif);
	//struct touch_core_data *ts = to_touch_core(d->dev);
	int event_data;

	if (d->ignore_fb_cb)
		return 0;

	if (event != FB_EVENT_BLANK && event != FB_EARLY_EVENT_BLANK && event != FB_R_EARLY_EVENT_BLANK) {
		return 0;
	}

	if (ev && ev->data) {
		event_data = *((int *)ev->data);
		if (event == FB_EVENT_BLANK) {
			if (event_data == FB_BLANK_POWERDOWN) {
				TOUCH_I("FB_EVENT_BLANK / FB_BLANK_POWERDOWN\n");
				// Power Seq. is done by noti. LCD_EVENT_TOUCH_LPWG_ON
				//synaptics_lpwg_mode(d->dev, 0);
				mutex_unlock(&d->fb_lock);
			} 
			else if (event_data == FB_BLANK_UNBLANK) {
				mutex_unlock(&d->fb_lock);
			}
		}
		else if (event == FB_EARLY_EVENT_BLANK) {
			if (event_data == FB_BLANK_UNBLANK) {
				mutex_lock(&d->fb_lock);
				TOUCH_I("FB_EARLY_EVENT_BLANK / FB_BLANK_UNBLANK\n");
				// Power Seq. is done by noti. LCD_EVENT_TOUCH_LPWG_OFF
				//synaptics_lpwg_mode(d->dev, 1);
			} 
			else if (event_data == FB_BLANK_POWERDOWN) {
				mutex_lock(&d->fb_lock);
			}
		}
		else if (event == FB_R_EARLY_EVENT_BLANK) {
			if (event_data == FB_BLANK_POWERDOWN) {
				TOUCH_I("FB_R_EARLY_EVENT_BLANK / FB_BLANK_POWERDOWN\n");
				mutex_unlock(&d->fb_lock);
			} 
			else if (event_data == FB_BLANK_UNBLANK) {
				TOUCH_I("FB_R_EARLY_EVENT_BLANK / FB_BLANK_UNBLANK\n");
				// Power Seq. is done by LCD_EVENT_TOUCH_LPWG_ON
				//synaptics_lpwg_mode(d->dev, 0);
				mutex_unlock(&d->fb_lock);
			}
		}
	}

	return 0;
}

static int synaptics_bin_fw_version(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	const struct firmware *fw = NULL;
	const u8 *firmware = NULL;
	int rc = 0;

	rc = request_firmware(&fw, d->fw.def_fw, dev);
	if (rc != 0) {
		TOUCH_E("[%s] request_firmware() failed %d, %s\n", __func__, rc, d->fw.def_fw);
		return -EIO;
	}

	firmware = fw->data;

	memcpy(d->fw.img_product_id,
			&firmware[d->fw.fw_pid_addr], 6);
	memcpy(d->fw.img_version,
			&firmware[d->fw.fw_ver_addr], 4);

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
	//struct touch_core_data *ts = to_touch_core(dev);
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

	if (synaptics_is_product(d, "PLG610", 6))
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch IC : TD4300\n\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch product ID read fail\n");

	if (NORMAL_BOOT == touch_boot_mode_check(dev)) {
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

		if (synaptics_is_img_product(d, "PLG610", 6))
			offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"Touch IC : TD4300\n\n");
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

	ret = synaptics_ic_info(dev);

	if (ret < 0) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Read Fail Touch IC Info\n");
		return offset;
	}

	if (d->fw.version[0] > 0x50) {
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"%s\n", d->fw.version);
	} else {
		offset = snprintf(buf + offset, PAGE_SIZE - offset,
					"v%d.%02d(0x%X/0x%X/0x%X/0x%X)\n",
					(d->fw.version[3] & 0x80 ? 1 : 0),
					d->fw.version[3] & 0x7F,
					d->fw.version[0],
					d->fw.version[1],
					d->fw.version[2],
					d->fw.version[3]);
	}

	return offset;
}

static int synaptics_notify(struct device *dev, ulong event, void *data)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;

	switch (event) {
	case LCD_EVENT_TOUCH_LPWG_ON:
		TOUCH_I("LCD_EVENT_TOUCH_LPWG_ON notified\n");
		ret = synaptics_lpwg_mode(dev, 0); // In MFTS/Mini OS, LPWG Mode is Zero!!!
		break;

	case LCD_EVENT_TOUCH_LPWG_OFF:
		TOUCH_I("LCD_EVENT_TOUCH_LPWG_OFF notified\n");
		ret = synaptics_lpwg_mode(dev, 1);
		break;

	default:
		TOUCH_E("Notification[%lu] is not supported\n", event);
		break;
	}

	return ret;
}

static void synaptics_monitor_work_func(struct work_struct *monitor_work)
{
	struct synaptics_data *d = container_of(to_delayed_work(monitor_work), struct synaptics_data, monitor_work);
	struct touch_core_data *ts = to_touch_core(d->dev);

	mutex_lock(&ts->lock);
	synaptics_noise_log(ts->dev);
	mutex_unlock(&ts->lock);
}

static void synaptics_esd_recovery_work_func(struct work_struct *esd_recovery_work)
{
	struct synaptics_data *d = container_of(to_delayed_work(esd_recovery_work), struct synaptics_data, esd_recovery_work);
	struct touch_core_data *ts = to_touch_core(d->dev);

	mutex_lock(&d->fb_lock);
	mutex_lock(&ts->lock);

	if (!d->esd_recovery_en) {
		TOUCH_E("esd_recovery_en is disabled\n");
		goto RET;
	}

	if (d->state != TC_STATE_ACTIVE || atomic_read(&ts->state.fb) != FB_RESUME) {
		TOUCH_E("esd_recovery_work is called in NOT TC_STATE_ACTIVE or NOT FB_RESUME\n");
		goto RET;
	}

	TOUCH_I("esd_recovery_work start\n");

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	d->ignore_fb_cb = 1;
	LCD_RECOVERY;	// Should be blocked !!!
	d->ignore_fb_cb = 0;

	d->esd_recovery_en = 0;
	synaptics_irq_clear(ts->dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	synaptics_irq_clear(ts->dev);
	d->esd_recovery_en = 1;

	TOUCH_I("esd_recovery_work end\n");

RET:

	mutex_unlock(&ts->lock);
	mutex_unlock(&d->fb_lock);
}

int synaptics_set_tuning(struct device *dev)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	//struct synaptics_data *d = to_synaptics_data(dev);
	//u8 data[5];
	int ret = 0;

	TOUCH_I("Set new tuning\n");
#if 0
	// ALGM
	data[0] = 0x01; // ALGM En
	data[1] = 0x09; // Min Comp Factor
	ret = synaptics_write(dev, ALGM_CONTROL_REG, data, 2);

	// 2D Ctrl
	data[0] = 40; // Noise Floor
	data[1] = 50; // Min Peak Amp
	ret = synaptics_write(dev, F12_CONTROL_10_REG, data, 2);
#endif
	synaptics_force_update(dev);

	return ret;
}

static ssize_t store_reg_ctrl(struct device *dev,
		const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct i2c_client *i2c = to_i2c_client(dev);
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	int reg = 0;
	int offset = 0;
	int value = 0;

	if (sscanf(buf, "%5s %x %x %x %x ",
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
		TOUCH_I("page[0x%x] reg[0x%x] offset[0x%x] = 0x%x\n",
				page, reg, offset, buffer[offset]);
	} else if (!strcmp(command, "addr")) {
		i2c->addr = page;
		TOUCH_I("i2c slave address = 0x%x\n", page);
	} 
#if USE_DEBUG_I2C_LOG
	else if (!strcmp(command, "log")) {
		i2c_log = page;
		page_num = 0;
	}
#endif
	else if (!strcmp(command, "esd")) {
		struct synaptics_data *d = to_synaptics_data(dev);
		schedule_delayed_work(&d->esd_recovery_work, 0);
	}
	else if (!strcmp(command, "reset")) {
		synaptics_soft_reset(dev);
	}
	else if (!strcmp(command, "tune")) {
		synaptics_set_tuning(dev);
	}
	else {
		TOUCH_E("Usage\n");
		TOUCH_E("write page reg offset value\n");
		TOUCH_E("read page reg offset\n");
	}
	synaptics_set_page(dev, DEFAULT_PAGE);
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	mutex_lock(&ts->lock);

	touch_interrupt_control(dev, INTERRUPT_DISABLE);

	if (value == 1) {
		TOUCH_I("%s : Display & Touch Reset\n", __func__);
		LCD_RESET_L;
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(12);
		touch_gpio_direction_output(ts->reset_pin, 1);
		LCD_RESET_H;
		touch_msleep(300);
	}
#if 0
	else {
		TOUCH_I("%s : Touch Reset\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_msleep(5);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(300);
	}
#endif
	synaptics_init(dev);

	touch_interrupt_control(dev, INTERRUPT_ENABLE);

	mutex_unlock(&ts->lock);

	return count;
}

static ssize_t show_noise_log(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int offset = 0;

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

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if ((d->noise.noise_log == NOISE_DISABLE)
		&& (value == NOISE_ENABLE)) {
		d->noise.noise_log = NOISE_ENABLE;
		schedule_delayed_work(&d->monitor_work, msecs_to_jiffies(NOISE_LOG_PERIOD_NO_CONTACT));
	} else if ((d->noise.noise_log == NOISE_ENABLE)
			&& (value == NOISE_DISABLE)) {
		d->noise.noise_log = NOISE_DISABLE;
		cancel_delayed_work(&d->monitor_work);
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

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(noise_log_enable, show_noise_log, store_noise_log);

static struct attribute *td4300_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_noise_log_enable.attr,
	NULL,
};

static const struct attribute_group td4300_attribute_group = {
	.attrs = td4300_attribute_list,
};

static int synaptics_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &td4300_attribute_group);
	if (ret < 0)
		TOUCH_E("td4300 sysfs register failed\n");

	td4300_prd_register_sysfs(dev);

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

static int synaptics_fw_compare(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

//	int i = 0;
	int update = 0;

	TOUCH_I("ic_product_id : %s [V%d.%02d]\n", d->fw.product_id, (d->fw.version[3] & 0x80)>>7, d->fw.version[3] & 0x7F);
	TOUCH_I("img_product_id : %s [V%d.%02d]\n", d->fw.img_product_id, (d->fw.img_version[3] & 0x80)>>7, d->fw.img_version[3] & 0x7F);

	if (ts->force_fwup) {
		update = 1;
		TOUCH_I("F/W Compare : Forced to upgrade!!\n");
	}
	else {
		if (!d->is_pid_read) {
			update = 1; // In case of uBL mode, pid cannot be read correctly
			TOUCH_I("F/W Compare : Product ID is not read, uBL mode??!!\n");
		}
		else {
			if (!strncmp("PLG610", d->fw.product_id, 6) || !strncmp("TD4300", d->fw.product_id, 6)) {
				if (d->fw.version[3] != d->fw.img_version[3]) {
					TOUCH_I("F/W Compare : Version is different, Need to upgrade!!\n");
					update = 1;
				}
				else {
					TOUCH_I("F/W Compare : Version is same, Skip upgrade!!\n");
				}
			}
			else {
				TOUCH_I("F/W Compare : Invalid product_id in IC, Skip upgrade!!\n");
			}
		}	
	}

	return update;
}

static int synaptics_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	const struct firmware *fw = NULL;
	const u8 *firmware = NULL;
	char fwpath[256] = {0};
	int ret = 0;
	//unsigned char command = 0x01;
	//unsigned char zero = 0x00;
	int update = 0;
	TOUCH_TRACE();

	// Addr in img for PLG610
	d->fw.fw_pid_addr = 0x10;		// PLG610
	d->fw.fw_ver_addr = 0x1D100;	// PLG610
	d->fw.def_fw = ts->def_fwpath[0];

#if 0
	if (ts->force_fwup == 0) {
		TOUCH_I("Upgrade in probe disabled\n");
		return -EPERM;
	}
#endif

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n",
			&ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from def_fwpath : %s\n", fwpath);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	if (fwpath == NULL) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	d->esd_recovery_en = 1;

	ret = request_firmware(&fw, fwpath, dev);

	if (ret < 0) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n",
			fwpath, ret);

		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	firmware = fw->data;

	memcpy(d->fw.img_product_id,
			&firmware[d->fw.fw_pid_addr], 6);
	memcpy(d->fw.img_version,
			&firmware[d->fw.fw_ver_addr], 4);

	update = synaptics_fw_compare(dev);

	if (update) {
		ret = FirmwareUpgrade(dev, fw);
		if (ret < 0) {
			TOUCH_E("F/W UPGRADE FAIL !!!\n");

			release_firmware(fw);

			// Try to recovery mode
			TOUCH_I("F/W Recovery with fwpath[%s]\n", ts->def_fwpath[1]);
			ret = request_firmware(&fw, ts->def_fwpath[1], dev);
			if (ret < 0) {
				TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
				goto FAIL;
			}

			ret = FirmwareRecovery(dev, fw);
			if (ret < 0) {
				TOUCH_E("F/W RECOVERY FAIL !!!\n");
				release_firmware(fw);
				goto FAIL;
			}
		}
	} else {
		TOUCH_I("need not upgrade f/w\n");
		release_firmware(fw);
		return -EPERM;
	}

	TOUCH_I("f/w upgrade finished\n");

	release_firmware(fw);

	d->need_scan_pdt = true;

	// Reset is done already in f/w updating.
	// So, we need to re-scan pdt immediately to do something.
	// Scanning pdt can be done twice(here and init), but we don't care it.
	synaptics_page_description(dev);

FAIL:

	/*sw reset*/
	TOUCH_I("Soft Reset !!!\n");
	ret = synaptics_soft_reset(dev);
	if (ret < 0)
		TOUCH_E("reset fail\n");

	return -EPERM;
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

	d->dev = dev;
	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	//touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);	// in lk, LCD/Touch power on seq. is done.

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	mutex_init(&d->rw_lock);
	mutex_init(&d->fb_lock);

	TOUCH_I("lge_get_boot_mode = %d\n", lge_get_boot_mode());

	synaptics_init_tci_info(dev);

	TOUCH_I("synaptics fb_notif register\n");
	d->fb_notif.notifier_call = synaptics_fb_notifier_callback;
	d->fb_notif.priority = ts->fb_notif.priority + 1; // To call synaptics fb notif in advance
	fb_register_client(&d->fb_notif);

	d->state = TC_STATE_ACTIVE;
	d->cover_state = 0; // Open
	d->is_pid_read = 0;
	d->esd_recovery_en = 0;
	d->ignore_fb_cb = 0;

	d->need_scan_pdt = true;

	INIT_DELAYED_WORK(&d->monitor_work, synaptics_monitor_work_func);
	INIT_DELAYED_WORK(&d->esd_recovery_work, synaptics_esd_recovery_work_func);

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


#define MATCH_NAME			"synaptics,td4300"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
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

	if (touch_get_device_type() != TYPE_TD4300 ) {
		TOUCH_I("%s, td4300 not found\n", __func__);
		return 0;
	}

	TOUCH_I("%s, TD4300 found !!!\n", __func__);

	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("hyokmin.kwon@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
