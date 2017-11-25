/* touch_td4310.c
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
#include "touch_td4310.h"
#include "touch_td4310_prd.h"

const char *f_str[] = {
	"ERROR",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP",
	"MULTI_FINGER",
	"DELAY_TIME"
};

static struct td4310_rmidev_exp_fhandler rmidev_fhandler;

bool td4310_is_product(struct td4310_data *d,
				const char *product_id, size_t len)
{
	return strncmp(d->ic_info.product_id, product_id, len)
			? false : true;
}

bool td4310_is_img_product(struct td4310_data *d,
				const char *product_id, size_t len)
{
	return strncmp(d->ic_info.img_product_id, product_id, len)
			? false : true;
}

int td4310_read(struct device *dev, u8 addr, void *data, int size)
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

int td4310_write(struct device *dev, u8 addr, void *data, int size)
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

void td4310_reset_ctrl(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	u8 wdata = 0x01;
	int ret = 0;

	TOUCH_TRACE();

	switch (ctrl) {
	default:
	case SW_RESET:
		TOUCH_I("%s : SW Reset\n", __func__);
		ret = td4310_write(dev, DEVICE_COMMAND_REG, &wdata, sizeof(u8));
		if(ret < 0)
			TOUCH_E("td4310_write failed, ret = %d\n",ret);
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

int td4310_set_page(struct device *dev, u8 page)
{
	int ret = td4310_write(dev, PAGE_SELECT_REG, &page, 1);

	if (ret >= 0)
		to_td4310_data(dev)->curr_page = page;

	return ret;
}

static void td4310_check_fail_reason(char* reason)
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

static int td4310_get_f12_reg(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	u8 query_4_data = 0; 		/* Size Of Control Presence */
	u8 *query_5_data = NULL; 	/* Control Register Presence */
	u8 query_7_data = 0; 		/* Size Of Data Presence */
	u8 *query_8_data = NULL; 	/* Data Register Presence */

	u8 ctrl_23_data[2]; 		/* Object Report, Max Number of Objects */
	u8 ctrl_8_data[14];			/* Maximum XY Coordinate */

	u8 offset;
	int i;
	int ret;

	TOUCH_TRACE();

	/* ------------- F12 Control Reg Start ------------- */
	ret = td4310_read(dev, d->f12.dsc.query_base + 4,
			     &query_4_data, sizeof(query_4_data));
	if (ret < 0) {
		TOUCH_E("faied to get query4 (ret: %d)\n", ret);
		goto error;
	}

	if (query_4_data != 0) {
		query_5_data = kzalloc(query_4_data, GFP_KERNEL);
		if (!query_5_data) {
			TOUCH_E("failed to allocate query_5_data\n");
			goto error;
		}
		if (d->f12_reg.ctrl != NULL) {
			devm_kfree(dev, d->f12_reg.ctrl);
		}
		d->f12_reg.ctrl = devm_kzalloc(dev, query_4_data * 8, GFP_KERNEL);
		if (!d->f12_reg.ctrl) {
			TOUCH_E("failed to allocate d->f12_reg.ctrl\n");
			goto error;
		}
	}

	ret = td4310_read(dev, d->f12.dsc.query_base + 5,
			     query_5_data, query_4_data);
	if (ret < 0) {
		TOUCH_E("faied to get query5 (ret: %d)\n", ret);
		goto error;
	}

	for (i = 1; i < query_4_data; i++) {
		TOUCH_I("qeury_5_data[%d] = 0x%x", i, query_5_data[i]);
	}

	for (i = 0, offset = 0; i < query_4_data * 8; i++) {
		// (i/8) --> array element change, (i%8) --> bit pattern change
		if (query_5_data[(i/8)+1] & (1 << (i%8))) {
			d->f12_reg.ctrl[i] = d->f12.dsc.control_base + offset;
			TOUCH_I("f12_reg.ctrl[%d]=0x%02X (0x%02x+%d)\n",
					i, d->f12_reg.ctrl[i],
					d->f12.dsc.control_base, offset);
			offset++;
		}
	}
	/* ------------- F12 Control Reg End ------------- */

	/* ------------- F12 Data Reg Start ------------- */
	ret = td4310_read(dev, d->f12.dsc.query_base + 7,
			     &query_7_data, sizeof(query_7_data));
	if (ret < 0) {
		TOUCH_E("faied to get query7 (ret: %d)\n", ret);
		goto error;
	}

	if (query_7_data != 0) {
		query_8_data = kzalloc(query_7_data, GFP_KERNEL);
		if (!query_8_data) {
			TOUCH_E("failed to allocate query_8_data\n");
			goto error;
		}
		if (d->f12_reg.data != NULL) {
			devm_kfree(dev, d->f12_reg.data);
		}
		d->f12_reg.data = devm_kzalloc(dev, query_7_data * 8, GFP_KERNEL);
		if (!d->f12_reg.data) {
			TOUCH_E("failed to allocate d->f12_reg.data\n");
			goto error;
		}
	}

	ret = td4310_read(dev, d->f12.dsc.query_base + 8,
			     query_8_data, query_7_data);
	if (ret < 0) {
		TOUCH_E("faied to get query8 (ret: %d)\n", ret);
		return ret;
	}

	for (i = 1; i < query_7_data; i++) {
		TOUCH_I("qeury_8_data[%d] = 0x%x", i, query_8_data[i]);
	}

	for (i = 0, offset = 0; i < query_7_data * 8; i++) {
		// (i/8) --> array element change, (i%8) --> bit pattern change
		if (query_8_data[(i/8)+1] & (1 << (i%8))) {
			d->f12_reg.data[i] = d->f12.dsc.data_base + offset;
			TOUCH_I("f12_reg.data[%d]=0x%02X (0x%02x+%d)\n",
					i, d->f12_reg.data[i],
					d->f12.dsc.data_base, offset);
			offset++;
		}
	}
	/* ------------- F12 Data Reg End ------------- */

	ret = td4310_read(dev, d->f12_reg.ctrl[23],
			     ctrl_23_data, sizeof(ctrl_23_data));
	if (ret < 0) {
		TOUCH_E("faied to get f12_reg.ctrl[23] data (ret: %d)\n", ret);
		return ret;
	}

	d->object_report = ctrl_23_data[0];
	d->max_num_of_fingers = min_t(u8, ctrl_23_data[1], (u8) MAX_NUM_OF_FINGERS);

	TOUCH_I("object_report[0x%02X], max_num_of_fingers[%d]\n",
			d->object_report, d->max_num_of_fingers);

	ret = td4310_read(dev, d->f12_reg.ctrl[8],
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

	if (query_5_data)
		kfree(query_5_data);
	if (query_8_data)
		kfree(query_8_data);

	return 0;

error:
	if (query_5_data)
		kfree(query_5_data);
	if (d->f12_reg.ctrl)
		devm_kfree(dev, d->f12_reg.ctrl);
	if (query_8_data)
		kfree(query_8_data);
	if (d->f12_reg.data)
		devm_kfree(dev, d->f12_reg.data);
	return -ENOMEM;
}

static int td4310_page_description(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	struct function_descriptor dsc;
	u8 page;

	unsigned short pdt;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	u32 backup_slave_addr = client->addr;

	TOUCH_TRACE();

	atomic_set(&d->state.fw_recovery, false);
	memset(&d->f01, 0, sizeof(struct td4310_function));
	memset(&d->f12, 0, sizeof(struct td4310_function));
	memset(&d->f34, 0, sizeof(struct td4310_function));
	memset(&d->f51, 0, sizeof(struct td4310_function));
	memset(&d->f54, 0, sizeof(struct td4310_function));
	memset(&d->f55, 0, sizeof(struct td4310_function));
	memset(&d->fdc, 0, sizeof(struct td4310_function));

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		ret = td4310_set_page(dev, page);

		if (ret < 0) {
			TOUCH_E("faied to set page %d (ret: %d)\n", page, ret);
			/* TD4310 FW recovery - slave addr change to uBL mode slave addr*/
			TOUCH_E("I2C error - change slave addr to 0x%x\n",d->uBL_addr);
			client->addr = d->uBL_addr;
			td4310_set_page(dev, page);
		}

		for (pdt = PDT_START; pdt >= PDT_END; pdt -= sizeof(dsc)) {
			ret = td4310_read(dev, pdt, &dsc, sizeof(dsc));

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

			case 0x12:
				d->f12.dsc = dsc;
				d->f12.page = page;
				ret = td4310_get_f12_reg(dev);
				if (ret < 0) {
					TOUCH_E("failed to get f12 register, ret = %d\n", ret);
					return ret;
				}
				break;

			case 0x34:
				d->f34.dsc = dsc;
				d->f34.page = page;
				break;

			/* td4310 FW recovery Added */
			case 0x35:
				if(to_touch_core(dev)->role.use_fw_recovery) {
					TOUCH_E("F35 detected - Need FW recovery\n");
					atomic_set(&d->state.fw_recovery, true);
					break;
				}

			case 0x51:
				d->f51.dsc = dsc;
				d->f51.page = page;
				break;

			case 0x54:
				d->f54.dsc = dsc;
				d->f54.page = page;
				break;

			case 0x55:
				d->f55.dsc = dsc;
				d->f55.page = page;
				break;

			case 0xdc:
				d->fdc.dsc = dsc;
				d->fdc.page = page;
				break;

			default:
				break;
			}
		}
	}

	TOUCH_I("common[%dP:0x%02x] finger_f12[%dP:0x%02x] flash[%dP:0x%02x] analog[%dP:0x%02x] lpwg[%dP:0x%02x]\n",
		d->f01.page, d->f01.dsc.fn_number,
		d->f12.page, d->f12.dsc.fn_number,
		d->f34.page, d->f34.dsc.fn_number,
		d->f54.page, d->f54.dsc.fn_number,
		d->f51.page, d->f51.dsc.fn_number);

	client->addr = backup_slave_addr;
	if(atomic_read(&d->state.fw_recovery)) {
		return -1;
	}

	ret = td4310_set_page(dev, DEFAULT_PAGE);

	if (ret) {
		TOUCH_E("faied to set page %d (ret: %d)\n", 0, ret);
		return ret;
	}

	return 0;
}

int td4310_force_update(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	int ret = 0;
	int retry = 0;
	u8 data = 0;

	TOUCH_TRACE();

	ret = td4310_set_page(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("td4310_set_page error\n");
		goto FAIL;
	}

	data = 0x04; /* Force Update */
	ret = td4310_write(dev, ANALOG_COMMAND_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("ANALOG_COMMAND_REG write error\n");
		goto FAIL;
	}

	/* Waiting for update complete */
	do {
		touch_msleep(5);
		ret = td4310_read(dev, ANALOG_COMMAND_REG, &data, 1);
		if (ret < 0) {
			TOUCH_E("ANALOG_COMMAND_REG read error\n");
			goto FAIL;
		}
		if ((data & 0x04) == 0x00) { /* Force update bit cleared */
			TOUCH_I("Force update bit cleared (data:0x%x)\n", data);
			break;
		}
	} while ((retry++) < 40);

	if (retry >= 40) {
		TOUCH_E("force_update time out!!\n");
		ret = -EPERM;
		goto FAIL;
	} else {
		TOUCH_I("force_update complete : %d ms\n", (retry+1)*5);
	}

	ret = td4310_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("td4310_set_page set error\n");
		goto FAIL;
	}

	return ret;

FAIL:
	TOUCH_E("td4310_force_update error !!!\n");
	td4310_set_page(dev, DEFAULT_PAGE);
	return ret;
}

static int td4310_get_product_id(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	ret = td4310_read(dev, PRODUCT_ID_REG,
			d->ic_info.product_id, sizeof(d->ic_info.product_id));

	if (ret < 0) {
		TOUCH_I("%s - read error...\n", __func__);
		return ret;
	}

	TOUCH_I("%s - IC_product_id: %s\n",
			__func__, d->ic_info.product_id);

	return 0;
}

int td4310_ic_info(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);

	int ret;

	TOUCH_TRACE();

	if (atomic_read(&d->state.scan_pdt) == true) {
		ret = td4310_page_description(dev);
		if (ret < 0) {
			TOUCH_I("%s - page description failed\n", __func__);
			return ret;
		}
		atomic_set(&d->state.scan_pdt, false);
	}

	ret = td4310_get_product_id(dev);
	ret |= td4310_read(dev, FLASH_CONFIG_ID_REG,
			d->ic_info.raws, sizeof(d->ic_info.raws));
	ret |= td4310_read(dev, CUSTOMER_FAMILY_REG,
			&(d->ic_info.family), sizeof(d->ic_info.family));
	ret |= td4310_read(dev, FW_REVISION_REG,
			&(d->ic_info.revision), sizeof(d->ic_info.revision));

	if (ret < 0) {
		TOUCH_I("%s - read error...\n", __func__);
		atomic_set(&d->state.scan_pdt, true);
		ret = td4310_page_description(dev);
		atomic_set(&d->state.scan_pdt, false);
		return ret;
	}

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

static int td4310_set_configured(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	u8 dev_status = 0;
	u8 dev_ctrl_data =0;
	int ret = 0;

	ret = td4310_read(dev, DEVICE_STATUS_REG, &dev_status, sizeof(dev_status));
	ret |= td4310_read(dev, DEVICE_CONTROL_REG, &dev_ctrl_data, sizeof(dev_ctrl_data));
	if (ret < 0) {
		TOUCH_E("failed to read device reg - ret : %d\n", ret);
		return ret;
	}

	if (dev_status == 0x00) {
		TOUCH_I("%s - Already set configured. Device Ctrl Reg : 0x%x\n", __func__, dev_ctrl_data);
		return 0;
	}

	/* After Reset -> dev_status 0x81 : Need to set configured */
	if (dev_status == 0x81) {
		TOUCH_I("%s - Need to set configured. dev_status : 0x%x\n", __func__, dev_status);
		dev_ctrl_data = 0x80; // Set Configured bit
		ret = td4310_write(dev, DEVICE_CONTROL_REG, &dev_ctrl_data, sizeof(dev_ctrl_data));
		ret |= td4310_read(dev, DEVICE_STATUS_REG, &dev_status, sizeof(dev_status));
		if (ret < 0) {
			TOUCH_E("failed to read/write device status - ret : %d\n", ret);
			return ret;
		}
		TOUCH_I("%s - device_status bit cleared : 0x%x\n", __func__, dev_status);
	}
	atomic_set(&d->state.config, IC_CONFIGURED_DONE);
	return 0;
}

static int td4310_sleep_control(struct device *dev, u8 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	u8 val;
	int ret;

	TOUCH_TRACE();

	ret = td4310_read(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read device control reg - ret:%d\n", ret);
		return ret;
	}

	val &= 0xf8; // Clear No Sleep/Sleep Mode bit (3 bit)

	if (mode) {
		val |= DEVICE_CONTROL_SLEEP;
		atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
	} else {
		val |= DEVICE_CONTROL_NORMAL_OP;
		atomic_set(&ts->state.sleep, IC_NORMAL);
	}

	ret = td4310_write(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write device control reg - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - %s\n", __func__, mode ? "IC_DEEP_SLEEP" : "IC_NORMAL");

	return 0;
}

/*
static int td4310_lpwg_debug(struct device *dev, int num)
{
	struct td4310_data *d = to_td4310_data(dev);

	u8 count = 0;
	u8 index = 0;
	u8 buf = 0;
	u8 i = 0;
	u8 addr = 0;
	u8 offset = num ? LPWG_MAX_BUFFER + 2 : 0;

	TOUCH_TRACE();

	td4310_set_page(dev, LPWG_PAGE);

	td4310_read(dev, LPWG_TCI1_FAIL_COUNT_REG + offset, &count, sizeof(count));
	td4310_read(dev, LPWG_TCI1_FAIL_INDEX_REG + offset, &index, sizeof(index));

	for (i = 1; i <= count; i++) {
		addr = LPWG_TCI1_FAIL_BUFFER_REG + offset +
			((index + LPWG_MAX_BUFFER - i) % LPWG_MAX_BUFFER);
		td4310_read(dev, addr, &buf, sizeof(buf));
		TOUCH_I("TCI(%d)-Fail[%d/%d] : %s\n", num, count - i + 1, count,
			(buf > 0 && buf < 6) ? f_str[buf] : f_str[0]);

		if (i == LPWG_MAX_BUFFER)
			break;
	}

	td4310_set_page(dev, DEFAULT_PAGE);

	return 0;
}
*/
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
static int td4310_lpwg_fail_control(struct device *dev, u16 value)
{
	struct td4310_data *d = to_td4310_data(dev);
	u8 buffer[2] = {0};
	int ret = 0;

	TOUCH_TRACE();

	td4310_set_page(dev, LPWG_PAGE);

	ret = td4310_read(dev, LPWG_FAIL_INT_ENABLE_REG, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read LPWG_FAIL_INT_ENABLE_REG - ret:%d\n", ret);
		goto error;
	}

	buffer[0] = (value >> 8) & 0xFF;
	buffer[1] = value & 0xFF;

	if (td4310_write(dev, LPWG_FAIL_INT_ENABLE_REG, buffer, sizeof(buffer)) < 0) {
		TOUCH_I("LPWG_FAIL_INT_ENABLE_REG write error\n");
		goto error;
	} else {
		TOUCH_I("LPWG_FAIL_INT_ENABLE_REG write success\n");
	}

	td4310_set_page(dev, DEFAULT_PAGE);

	return 0;
error:
	TOUCH_E("%s - %d : LPWG fail interrupt control error\n", __func__, __LINE__);
	td4310_set_page(dev, DEFAULT_PAGE);
	return 0;
}

static int td4310_tci_report_enable(struct device *dev, bool enable)
{
	struct td4310_data *d = to_td4310_data(dev);

	u8 val[3] = {0,};
	int ret = 0;

	TOUCH_TRACE();

	if (d->f12_reg.ctrl != NULL) {
		ret = td4310_read(dev, FINGER_REPORT_REG, val, sizeof(val));
		if (ret < 0) {
			TOUCH_E("failed to read finger report enable - ret:%d\n", ret);
			return ret;
		}

		val[2] &= 0xfc;

		if (enable)
			val[2] |= 0x2;

		ret = td4310_write(dev, FINGER_REPORT_REG, val, sizeof(val));
		if (ret < 0) {
			TOUCH_E("failed to write finger report enable - ret:%d\n", ret);
			return ret;
		}
	} else {
		TOUCH_E("f12_reg.ctrl is not allocated\n");
		atomic_set(&d->state.scan_pdt,true);
	}

	return ret;
}

static int td4310_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	struct tci_info *info;

	u8 lpwg_data[7];
	int ret;
	u8 tci_reg[2] = {LPWG_TAPCOUNT_REG, LPWG_TAPCOUNT_REG2};
	int i = 0;

	TOUCH_TRACE();

	if(ts->lpwg.mode == LPWG_PASSWORD) {
		//Make sure (MultiTap Interrupt Delay Time < MultiTap Maximum InterTap Time 2)
		ts->tci.info[TCI_1].intr_delay = ts->tci.double_tap_check ? 68 : 0;
	} else {
		ts->tci.info[TCI_1].intr_delay = 0;
	}

	ret = td4310_set_page(dev, LPWG_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	for (i = 0; i < 2; i++) {
		if ((ts->tci.mode & (1 << i)) == 0x0) {

			TOUCH_I("%s - TCI %s disable\n", __func__, i?"KNOCK-CODE":"KNOCK-ON  ");

			lpwg_data[0] = 0;
			td4310_write(dev, tci_reg[i], lpwg_data, sizeof(u8));
		} else {
			info = &ts->tci.info[i];

			TOUCH_I("%s - TCI %s tap_count(%d), intertap_time(%d/%d), " \
					"touch_slop(%d), tap_distance(%d), intr_delay(%d)\n",
					__func__, i?"KNOCK-CODE":"KNOCK-ON  ", info->tap_count,
					info->min_intertap,	info->max_intertap,	info->touch_slop,
					info->tap_distance, info->intr_delay);

			lpwg_data[0] = ((info->tap_count << 3) | 1);
			lpwg_data[1] = info->min_intertap;
			lpwg_data[2] = info->max_intertap;
			lpwg_data[3] = info->touch_slop;
			lpwg_data[4] = info->tap_distance;
			lpwg_data[6] = (info->intr_delay << 1 | 1);
			td4310_write(dev, tci_reg[i], lpwg_data, sizeof(lpwg_data));
		}
	}

	ret = td4310_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	return ret;
}

static int td4310_lpwg_control(struct device *dev, int mode, int tci_control)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	TOUCH_I("%s - mode = %d\n", __func__, mode);

	switch (mode) {
	case LPWG_NONE:
		ts->tci.mode = 0x00;
		td4310_tci_knock(dev);
		break;

	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		td4310_tci_knock(dev);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x03;
		td4310_tci_knock(dev);
		break;

	case LPWG_PASSWORD_ONLY:
		ts->tci.mode = 0x02;
		td4310_tci_knock(dev);
		break;

	case LPWG_PARTIAL:
		ts->tci.mode = 0x01;
		td4310_tci_knock(dev);
		//TBD: LPWG_PARTIAL_REG SETTING
		break;

	default:
		TOUCH_E("Unknown lpwg control case = %d\n", mode);
		break;
	}

	if (tci_control == TCI_REPORT_ENABLE) {
		TOUCH_I("%s - TCI_REPORT_ENABLE\n", __func__);
		td4310_tci_report_enable(dev, true);
	} else if (tci_control == TCI_REPORT_DISABLE) {
		TOUCH_I("%s - TCI_REPORT_DISABLE\n", __func__);
		td4310_tci_report_enable(dev, false);
	} else if (tci_control == TCI_REPORT_NOT_SET) {
		TOUCH_I("%s - TCI_REPORT_NOT_SET\n", __func__);
	} else {
		TOUCH_E("Unknown tci control case = %d\n", tci_control);
	}

	return 0;
}

static int td4310_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);

	TOUCH_TRACE();

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			/* Forced lpwg set in minios suspend mode */
			td4310_lpwg_control(dev, LPWG_DOUBLE_TAP, TCI_REPORT_ENABLE);
			return 0;
		}

		if (ts->lpwg.screen) {
			TOUCH_I("%s %d line - skip lpwg\n", __func__, __LINE__);
			// td4310_lpwg_debug(dev, TCI_1); // Change real-time fail reason
			// td4310_lpwg_debug(dev, TCI_2);
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			TOUCH_I("%s %d line - deep sleep by prox\n", __func__, __LINE__);
			if (atomic_read(&ts->state.sleep) == IC_NORMAL)
				td4310_sleep_control(dev, IC_DEEP_SLEEP);
		} else {
			/* Knock On Case */
			TOUCH_I("%s %d line - " \
				"knock mode %d, screen %d, proxy %d, qcover %d\n",
				__func__, __LINE__,
				ts->lpwg.mode, ts->lpwg.screen,
				ts->lpwg.sensor, ts->lpwg.qcover);

			if (d->lpwg_fail_reason)
				td4310_lpwg_fail_control(dev, 0xFFFF);

			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				/* IC_DEEP_SLEEP -> LPWG case */
				if (ts->lpwg.mode > LPWG_NONE)
					td4310_sleep_control(dev, IC_NORMAL);

				return 0;
			}

			td4310_lpwg_control(dev, ts->lpwg.mode, TCI_REPORT_ENABLE);

			if (ts->lpwg.mode == LPWG_NONE) {
				if (atomic_read(&ts->state.sleep) == IC_NORMAL)
					td4310_sleep_control(dev, IC_DEEP_SLEEP);
			}
		}
		return 0;
	}

	/* resume */
	touch_report_all_event(ts);
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("%s %d line - normal\n", __func__, __LINE__);
		td4310_lpwg_control(dev, LPWG_NONE, TCI_REPORT_DISABLE);
		td4310_sleep_control(dev, IC_NORMAL);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s %d line - wake up on screen off and prox\n", __func__, __LINE__);
		if (atomic_read(&ts->state.sleep) == IC_NORMAL)
			td4310_sleep_control(dev, IC_DEEP_SLEEP);
	} else {
		/* partial */
		TOUCH_I("%s %d line - partial is not ready\n", __func__, __LINE__);
		td4310_lpwg_control(dev, LPWG_PARTIAL, TCI_REPORT_NOT_SET);
	}

	return 0;
}

static void td4310_init_tci_info(struct device *dev)
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

static int td4310_remove(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);

	TOUCH_TRACE();

	pm_qos_remove_request(&d->pm_qos_req);

	if (rmidev_fhandler.initialized
		&& rmidev_fhandler.insert) {
		rmidev_fhandler.exp_fn->remove(dev);
		rmidev_fhandler.initialized = false;
	}

	return 0;
}

static int td4310_get_status(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	u8 status[2] = {0, };
	int ret;
	u8 retry = 0;

	TOUCH_TRACE();

	/* Clear td4310_touch_info data members */
	memset(&(d->info),0,sizeof(struct td4310_touch_info));

	do {
		ret = td4310_read(dev, DEVICE_STATUS_REG,&status, sizeof(status));
		if (ret < 0) {
			TOUCH_E("failed to read device and irq status - ret:%d [retry : %d]\n", ret, retry);
			touch_msleep(30);
		} else {
			break;
		}
	} while((retry++) < 10);

	if (retry >= 10) {
		if(atomic_read(&d->state.esd_recovery) == ESD_RECOVERY_DONE) {
			TOUCH_I("###### Retry failed !!! - Call ESD Recovery ######\n");
			atomic_set(&d->state.esd_recovery, ESD_RECOVERY_NEED);
			atomic_set(&d->state.scan_pdt, true);
			atomic_set(&d->state.init, IC_INIT_NEED);
			return -ERESTART;
		}
	}

	TOUCH_D(TRACE, "status[device:%02x, interrupt:%02x]\n",
		status[0], status[1]);

	if(&(d->info) != NULL) {
		d->info.device_status  = status[0];
		d->info.irq_status     = status[1];
	} else {
		TOUCH_E("&(d->info) is not allocated\n");
	}

	return ret;
}

static int td4310_irq_clear(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	u8 status = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = td4310_read(dev, INTERRUPT_STATUS_REG, &status, sizeof(status));
	if (ret < 0) {
		TOUCH_E("failed to read device status - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - Touch irq cleared\n", __func__);

	return ret;
}

static int td4310_noise_log(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);

	u8 buffer[2] = {0};
	u8 buf_lsb = 0, buf_msb = 0, cns = 0;
	u16 im = 0, cid_im = 0, freq_scan_im = 0;

	TOUCH_TRACE();

	td4310_set_page(dev, ANALOG_PAGE);
	td4310_read(dev, INTERFERENCE_METRIC_LSB_REG, &buf_lsb, sizeof(buf_lsb));
	td4310_read(dev, INTERFERENCE_METRIC_MSB_REG, &buf_msb, sizeof(buf_msb));

	im = (buf_msb << 8) | buf_lsb;
	d->noise.im_sum += im;

	td4310_read(dev, CURRENT_NOISE_STATUS_REG, &cns, sizeof(cns));
	d->noise.cns_sum += cns;

	td4310_read(dev, CID_IM_REG, buffer, sizeof(buffer));
	cid_im = (buffer[1] << 8) | buffer[0];
	d->noise.cid_im_sum += cid_im;

	td4310_read(dev, FREQ_SCAN_IM_REG, buffer, sizeof(buffer));
	freq_scan_im = (buffer[1] << 8) | buffer[0];
	d->noise.freq_scan_im_sum += freq_scan_im;
	td4310_set_page(dev, DEFAULT_PAGE);

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

static int td4310_get_finger_count(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);

	u8 touch_cnt = d->max_num_of_fingers;
	u8 buf[2] = {0,};
	u16 touch_attention = 0;
	int ret;

	TOUCH_TRACE();

	if (d->f12_reg.data != NULL) {
		ret = td4310_read(dev, FINGER_REPORT_DATA, (u8 *) buf, sizeof(buf));
		if (ret < 0) {
			TOUCH_E("%s, %d : get touch_attention data failed\n",
					__func__, __LINE__);
			return ret;
		}

		touch_attention = (((u16)((buf[1] << 8) & 0xFF00) | (u16)((buf[0])&0xFF)));

		for (; touch_cnt > 0; touch_cnt--) {
			if (touch_attention & (0x1 << (touch_cnt - 1)))
				break;
		}
		TOUCH_D(ABS, "touch_cnt: %d\n", touch_cnt);
	} else {
		TOUCH_E("f12_reg.data is not allocated\n");
		atomic_set(&d->state.scan_pdt,true);
		return -ENOMEM;
	}

	return touch_cnt;
}
/* [bringup] */
static int td4310_check_status(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);
	bool checking_log_flag = false;
	const int checking_log_size = 1024;
	char *checking_log = NULL;
	int length = 0;
	u8 status = (d->info.device_status & STATUS_CODE_MASK);
	int ret = 0;

	TOUCH_TRACE();
	/* Check Device Status, 0x00 -> No Error */
	if(d->info.device_status) {
		TOUCH_I("%s : Need Logging, dev_status : 0x%x\n",__func__,d->info.device_status);

		/* Flash Program Status Bit Check */
		if (d->info.device_status & FLASH_PROG_MASK_STATUS) {
			if(atomic_read(&ts->state.core) == CORE_NORMAL) {
				TOUCH_I("##### F/W UPGRADE #####\n");
				atomic_set(&d->state.scan_pdt, true);
				atomic_set(&d->state.init, IC_INIT_NEED);
				ret = -EUPGRADE;
			} else {
				TOUCH_I("###### IC F/W UPGRADE - In Progress.....\n");
			}
		}

		/* Unconfigured Status Bit Check */
		if (d->info.device_status & DEVICE_UNCONF_MASK_STATUS) {
			if(atomic_read(&d->state.config) == IC_CONFIGURED_DONE) {
				TOUCH_I("##### Unconfigured #####\n");
				atomic_set(&d->state.config, IC_CONFIGURED_NEED);
				ret = -ESWRESET;
			} else {
				TOUCH_I("###### IC Configuration - In Progress.....\n");
			}
		}

	    /* ESD Status Check */
		if ((d->info.device_status) == 0x45 || (d->info.device_status) == 0x89 ||
			(d->info.device_status) == 0x81 || (d->info.device_status) == 0x09){
			if(atomic_read(&d->state.esd_recovery) == ESD_RECOVERY_DONE) {
				TOUCH_I("###### ESD Detected !!! - Call ESD Recovery ######\n");
				atomic_set(&d->state.scan_pdt, true);
				atomic_set(&d->state.init, IC_INIT_NEED);
				atomic_set(&d->state.esd_recovery, ESD_RECOVERY_NEED);
				ret = -ERESTART;
			} else {
				TOUCH_I("###### ESD Recovery - In Progress.....\n");
			}
		}
	}

	if(ret != 0) {
		checking_log = kzalloc(sizeof(*checking_log) * checking_log_size, GFP_ATOMIC);
		if(checking_log == NULL) {
			TOUCH_E("Failed to allocate mem for checking_log\n");
			ret = -ENOMEM;
			goto error;
		}
		if (status == RESET_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x01]Device reset occured last time ");
		}
		if (status == INVALID_CONF_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x02]Invalid Configuration!! ");
		}
		if (status == DEVICE_FAILURE_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x03]Device Failure!! ");
		}
		if (status == CONF_CRC_FAILURE_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x04]Configuration CRC Fail!! ");
		}
		if (status == FW_CRC_FAILURE_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x05]F/W CRC Fail!! ");
		}
		if (status == CRC_PROGRESS_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x06]CRC in progress!! ");
		}
		if (status == GUEST_CRC_FAILURE_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x07]Guest Code Failed!! ");
		}
		if (status == EXT_AFE_FAILURE_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x08]External DDIC AFE Failed!! ");
		}
		if (status == DISPLAY_FAILURE_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x09]Display Device Failed!! ");
		}
		if (d->info.device_status & FLASH_PROG_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[6th bit]Flash Prog bit set!! ");
		}
		if (d->info.device_status & DEVICE_UNCONF_MASK_STATUS) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[7th bit]Device Configuration Lost!! ");
		}
		if (status >= 0x0A) {
			checking_log_flag = true;
			length += snprintf(checking_log + length,
					checking_log_size - length,
					"[0x0A ~ 0x0F]Unknown Status: To be verified!! ");
		}
		if (checking_log_flag) {
			TOUCH_E("%s, device_status = %x irq_status = %x\n",
					checking_log, d->info.device_status, d->info.irq_status);
		}
	}

	if(checking_log != NULL)
		kfree(checking_log);

	return ret;

error:
	return ret;
}

static int td4310_irq_abs_data(struct device *dev, int object_to_read)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	struct touch_data *tdata;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;

	TOUCH_TRACE();

	ts->new_mask = 0;

	if (d->info.touch_cnt == 0) {
		goto end;
	}

	ret = td4310_read(dev, FINGER_DATA_REG,
			d->info.data, sizeof(*(d->info.data)) * d->info.touch_cnt);
	if (ret < 0) {
		TOUCH_E("faied to read finger data\n");
		goto end;
	}

	for (i = 0; i < d->info.touch_cnt; i++) {

		if (d->info.data[i].type == F12_NO_OBJECT_STATUS)
			continue;

		if (d->info.data[i].type > F12_MAX_OBJECT)
			TOUCH_D(ABS, "id : %d, type : %d\n",
					i, d->info.data[i].type);

		if (d->info.data[i].type == F12_FINGER_STATUS) {
			ts->new_mask |= (1 << i);
			tdata = ts->tdata + i;

			tdata->id = i;
			tdata->type = d->info.data[i].type;
			tdata->x = d->info.data[i].x_lsb | d->info.data[i].x_msb << 8;
			tdata->y = d->info.data[i].y_lsb | d->info.data[i].y_msb << 8;
			tdata->pressure = d->info.data[i].z;

			if (d->info.data[i].wx > d->info.data[i].wy) {
				tdata->width_major = d->info.data[i].wx;
				tdata->width_minor = d->info.data[i].wy;
				tdata->orientation = 0;
			} else {
				tdata->width_major = d->info.data[i].wy;
				tdata->width_minor = d->info.data[i].wx;
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

	if (d->noise.check_noise == NOISE_ENABLE
			|| d->noise.noise_log == NOISE_ENABLE) {
		td4310_noise_log(dev);
	}

end:
	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	return ret;
}

static int td4310_irq_abs(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	d->info.touch_cnt = td4310_get_finger_count(dev);
	if (d->info.touch_cnt < 0) {
		TOUCH_E("faied to read touch_cnt\n");
		return ret;
	}

	return td4310_irq_abs_data(dev, d->info.touch_cnt);
}
static int td4310_update_stylus_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	int ret = 0;
	u8 buffer = 0;
	u8 check_stylus_mode = ((atomic_read(&ts->state.qmemo) == QMEMO_RUNNING) ||
							(atomic_read(&ts->state.pmemo) == PMEMO_RUNNING));

	TOUCH_TRACE();

	TOUCH_I("%s : Previous Stylus State = %s\n",
				__func__, (atomic_read(&d->state.stylus_mode) ? "ON" : "OFF"));

	if (check_stylus_mode) {
		TOUCH_I("%s : QMEMO/PMEMO running, enable stylus mode\n", __func__);
		atomic_set(&d->state.stylus_mode, STYLUS_ON);
	} else {
		TOUCH_I("%s : QMEMO/PMEMO not running, disable stylus mode\n", __func__);
		atomic_set(&d->state.stylus_mode, STYLUS_OFF);
	}

	if ( check_stylus_mode && (atomic_read(&ts->state.fb) == FB_SUSPEND) ) {
		TOUCH_I("%s : FB SUSPEND, disable stylus mode\n", __func__);
		atomic_set(&d->state.stylus_mode, STYLUS_OFF);
	}

	ret = td4310_set_page(dev, LPWG_PAGE);
	if (ret < 0) {
		TOUCH_E("Write LPWG_PAGE error\n");
		return ret;
	}

	ret = td4310_read(dev, STYLUS_CTRL_REG, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read STYLUS_CTRL_REG - ret:%d\n", ret);
		return ret;
	}

	if (atomic_read(&d->state.stylus_mode) == STYLUS_ON)
		buffer |= STYLUS_MODE_ENABLE;
	else
		buffer &= ~STYLUS_MODE_ENABLE;

	ret = td4310_write(dev, STYLUS_CTRL_REG, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to write STYLUS_CTRL_REG- ret:%d\n", ret);
		return ret;
	}

	/* To make sure the register value for debug*/
	ret = td4310_read(dev, STYLUS_CTRL_REG, &buffer, sizeof(buffer));
	if (ret < 0) {
		TOUCH_E("failed to read STYLUS_CTRL_REG - ret:%d\n", ret);
		return ret;
	}

	ret = td4310_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("Write DEFAULT_PAGE error\n");
		return ret;
	}

	if( !!(buffer & STYLUS_MODE_ENABLE) == atomic_read(&d->state.stylus_mode) ) {
		TOUCH_I("%s : Current Stylus State = %s\n",
					__func__, (atomic_read(&d->state.stylus_mode) ? "ON" : "OFF"));
	} else {
		TOUCH_E("STYLUS_CTRL_REG not updated properly, value = 0x%x\n", buffer);
	}

	return ret;
}


static int td4310_tci_getdata(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	u32 buffer[12];
	int i = 0;
	int ret;

	TOUCH_TRACE();

	ts->lpwg.code_num = count;

	if (!count)
		return 0;

	ret = td4310_read(dev, LPWG_DATA_REG,
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

		if ((ts->lpwg.mode == LPWG_PASSWORD) &&
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

static int td4310_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	u8 status;
	u8 buffer;
	u8 fail_buffer;
	char reason[NUM_OF_EACH_FINGER_DATA];
	int ret;

	TOUCH_TRACE();

	ret = td4310_set_page(dev, LPWG_PAGE);

	if (ret < 0) {
		td4310_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	ret = td4310_read(dev, LPWG_STATUS_REG, &status, 1);
	if (ret < 0) {
		td4310_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	if (status & LPWG_STATUS_DOUBLETAP) {
		td4310_tci_getdata(dev, ts->tci.info[TCI_1].tap_count);
		ts->intr_status = TOUCH_IRQ_KNOCK;
	} else if (status & LPWG_STATUS_PASSWORD) {
		td4310_tci_getdata(dev, ts->tci.info[TCI_2].tap_count);
		ts->intr_status = TOUCH_IRQ_PASSWD;
	} else if(d->lpwg_fail_reason) {
		TOUCH_I("LPWG Fail Detected\n");
		td4310_read(dev, LPWG_FAIL_REASON_REALTIME_INT, &fail_buffer, 1);
		reason[0] = fail_buffer & 0x0F;
		reason[1] = (fail_buffer & 0xF0) >> 4;
		if (reason != NULL) {
			TOUCH_I("Fail-Reason TCI1 : [%d], TCI2 : [%d]\n", reason[0], reason[1]);
			td4310_check_fail_reason(reason);
		} else {
			TOUCH_E("LPWG Real-Time-Interrupt fail buffer is NULL\n");
		}
	} else {
		/* Overtab */
		td4310_read(dev, LPWG_OVER_TAPCOUNT, &buffer, 1);
		if (buffer > ts->tci.info[TCI_2].tap_count) {
			td4310_tci_getdata(dev, ts->tci.info[TCI_2].tap_count + 1);
			ts->intr_status = TOUCH_IRQ_PASSWD;
			TOUCH_I("knock code fail to over tap count = %d\n", buffer);
		}
	}

	return td4310_set_page(dev, DEFAULT_PAGE);
}

static int td4310_irq_handler(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	pm_qos_update_request(&d->pm_qos_req, 10);

	ret = td4310_get_status(dev);
	if (ret < 0) {
		TOUCH_E("td4310_get_status failed, ret : %d\n", ret);
		pm_qos_update_request(&d->pm_qos_req, PM_QOS_DEFAULT_VALUE);
		return ret;
	}

	ret = td4310_check_status(dev);
	if (ret == 0) {
		if (d->info.irq_status & INTERRUPT_MASK_ABS0) {
			ret = td4310_irq_abs(dev);
		}
		else if (d->info.irq_status & INTERRUPT_MASK_LPWG) {
			ret = td4310_irq_lpwg(dev);
		}
	} else {
		TOUCH_E("td4310_check_status failed, ret = %d\n",ret);
	}

	pm_qos_update_request(&d->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	return ret;
}

void td4310_rmidev_function(struct td4310_rmidev_exp_fn *exp_fn,
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

static int td4310_rmidev_init(struct device *dev)
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

int td4310_reset(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);

	TOUCH_I("%s : HW Reset(%d)\n", __func__, mode);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	if(mode == SW_RESET) {
		td4310_reset_ctrl(dev,SW_RESET);
	} else if(mode == HW_RESET) {
		td4310_reset_ctrl(dev,HW_RESET);
	} else {
		TOUCH_E(" Unknown mode: (%d)\n", mode);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		return 0;
	}
	atomic_set(&d->state.init, IC_INIT_NEED);

	mod_delayed_work(ts->wq, &ts->init_work, 0);

	return 0;
}

static int td4310_power(struct device *dev, int ctrl)
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

    case POWER_SW_RESET:
		TOUCH_I("%s, sw reset\n", __func__);
		td4310_reset(dev, SW_RESET);
		break;

	case POWER_HW_RESET:
		TOUCH_I("%s, hw reset\n", __func__);
		td4310_reset(dev, HW_RESET);
		break;

	default:
		TOUCH_I("%s, Unknown Power ctrl!!\n", __func__);

	}

	return 0;
}

static int td4310_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		return -EPERM;
	}

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		return 0;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
	}

	if (atomic_read(&d->state.init) == IC_INIT_DONE) {
		td4310_lpwg_mode(dev);
		td4310_update_stylus_mode(dev);
	} else {
		ret = 1;
	}

	return ret;
}

static int td4310_resume(struct device *dev)
{
	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("%s : Charger mode!!!\n", __func__);
		/* Deep Sleep */
		td4310_sleep_control(dev, 1);
		return -EPERM;
	}

	touch_interrupt_control(dev, INTERRUPT_DISABLE);
	//td4310_reset_ctrl(dev, SW_RESET);//[TBD]

	return 0;
}

static int td4310_lpwg(struct device *dev, u32 code, void *param)
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
		td4310_lpwg_mode(dev);
		break;
	}

	return 0;
}

static int td4310_bin_fw_version(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);

	const struct firmware *fw = NULL;
	int rc = 0;

	TOUCH_TRACE();

	rc = request_firmware(&fw, ts->def_fwpath[0], dev);
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

static char *td4310_productcode_parse(unsigned char *product)
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

static int td4310_get_cmd_version(struct device *dev, char *buf)
{
	struct td4310_data *d = to_td4310_data(dev);
	int offset = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = td4310_ic_info(dev);
	ret += td4310_bin_fw_version(dev);

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
			td4310_productcode_parse(d->ic_info.raws));
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"IC_product_id[%s]\n", d->ic_info.product_id);
	if (td4310_is_product(d, "PLG639", 6) || td4310_is_product(d, "PLG639", 6))
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch IC : TD4310\n\n");
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
			td4310_productcode_parse(d->ic_info.img_raws));
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Img_product_id[%s]\n", d->ic_info.img_product_id);
	if (td4310_is_img_product(d, "PLG639", 6) || td4310_is_img_product(d, "PLG639", 6))
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch IC : TD4310\n\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Touch product ID read fail\n");
	if(d->ic_info.bootloader_type)
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Bootloader Type : Optimized\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Bootloader Type : Non-Optimized\n");

	return offset;
}

static int td4310_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct td4310_data *d = to_td4310_data(dev);
	int offset = 0;
	int ret = 0;

	TOUCH_TRACE();

	ret = td4310_ic_info(dev);
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

static void td4310_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	int charger_state = atomic_read(&ts->state.connect);

	TOUCH_TRACE();

	atomic_set(&d->state.charger, CONNECT_NONE);

	if (charger_state == CONNECT_INVALID)
		atomic_set(&d->state.charger, CONNECT_NONE);
	else if ((charger_state == CONNECT_DCP)
			|| (charger_state == CONNECT_PROPRIETARY))
		atomic_set(&d->state.charger, CONNECT_TA);
	else if (charger_state == CONNECT_HUB)
		atomic_set(&d->state.charger, CONNECT_OTG);
	else
		atomic_set(&d->state.charger, CONNECT_USB);

	/* code for TA simulator */
	if (atomic_read(&ts->state.debug_option_mask)
			& DEBUG_OPTION_4) {
		TOUCH_I("TA Simulator mode, Set CONNECT_TA\n");
		atomic_set(&d->state.charger, CONNECT_TA);
	}

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, atomic_read(&d->state.charger));
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try I2C\n");
		return;
	}

	// TBD : implement if needed for TD4310
}

static int td4310_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	td4310_connect(dev);
	return 0;
}

static int td4310_debug_option(struct device *dev, u32 *data)
{
	u32 chg_mask = data[0];
	u32 enable = data[1];

	TOUCH_TRACE();

	switch (chg_mask) {
	case DEBUG_OPTION_0:
		TOUCH_I("Debug Option 0 %s\n", enable ? "Enable" : "Disable");
		break;
	case DEBUG_OPTION_4:
		TOUCH_I("TA Simulator mode %s\n",
			enable ? "Enable" : "Disable");
		td4310_connect(dev);
		break;
	default:
		TOUCH_E("Not supported debug option\n");
		break;
	}

	return 0;
}

static int td4310_notify(struct device *dev, ulong event, void *data)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s event=0x%x", __func__, (unsigned int)event);

	switch (event) {
	case NOTIFY_TOUCH_RESET:
		ret = NOTIFY_OK;
		TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
		break;
	case NOTIFY_DEBUG_OPTION:
		TOUCH_I("NOTIFY_DEBUG_OPTION!\n");
		ret = td4310_debug_option(dev, (u32 *)data);
		break;
	case NOTIFY_CONNECTION:
		TOUCH_I("NOTIFY_CONNECTION!\n");
		ret = td4310_usb_status(dev, *(u32 *)data);
		break;
	case NOTIFY_QMEMO_STATE:
		TOUCH_I("NOTIFY_QMEMO_STATE!\n");
		ret = td4310_update_stylus_mode(dev);
		break;
	case NOTIFY_PMEMO_STATE:
		TOUCH_I("NOTIFY_PMEMO_STATE!\n");
		ret = td4310_update_stylus_mode(dev);
		break;
	default:
		TOUCH_E("unknown event\n");
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
	td4310_set_page(dev, page);
	if (!strcmp(command, "write")) {
		td4310_read(dev, reg, buffer, offset + 1);
		buffer[offset] = (u8)value;
		td4310_write(dev, reg, buffer, offset + 1);
	} else if (!strcmp(command, "read")) {
		td4310_read(dev, reg, buffer, offset + 1);
		TOUCH_I("page[%d] reg[%x] offset[%d] = 0x%x\n",
				page, reg, offset, buffer[offset]);
	} else {
		TOUCH_E("Usage\n");
		TOUCH_E("Write page reg offset value\n");
		TOUCH_E("Read page reg offset\n");
	}
	td4310_set_page(dev, DEFAULT_PAGE);
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t show_check_noise(struct device *dev, char *buf)
{
	struct td4310_data *d = to_td4310_data(dev);

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
	struct td4310_data *d = to_td4310_data(dev);

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
	struct td4310_data *d = to_td4310_data(dev);

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
	struct td4310_data *d = to_td4310_data(dev);

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

static int td4310_recovery(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	struct i2c_client *client = to_i2c_client(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	u32 backup_slave_addr = client->addr;
	int ret = 0;

	TOUCH_TRACE();

	memcpy(fwpath, ts->def_fwpath[1], sizeof(fwpath));
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

	if (atomic_read(&d->state.fw_recovery) == true) {
		client->addr = d->uBL_addr; // change to uBL mode i2c slave address
	}
	ret = FirmwareRecovery(dev, fw);
	client->addr = backup_slave_addr;

	release_firmware(fw);

	return ret;
}

static ssize_t show_fw_recovery(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	int offset = 0;

	TOUCH_TRACE();

	if(ts->role.use_fw_recovery) {
		atomic_set(&ts->state.core, CORE_UPGRADE);
		mutex_lock(&ts->lock);
		touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

		td4310_recovery(dev);

		atomic_set(&d->state.scan_pdt,true);
		/* init force_upgrade */
		ts->force_fwup = 0;
		ts->test_fwpath[0] = '\0';

		//td4310_reset_ctrl(dev, SW_RESET);
		td4310_init(dev);
		atomic_set(&ts->state.core, CORE_NORMAL);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		mutex_unlock(&ts->lock);

		TOUCH_I("FirmwareRcovery Finish\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s\n",
				"FirmwareRecovery Finish");
	} else {
		TOUCH_I("FirmwareRcovery Failed. role.use_fw_recovery = %d\n",
				ts->role.use_fw_recovery);
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s%d\n",
				"FirmwareRcovery Failed. role.use_fw_recovery =\n",
				ts->role.use_fw_recovery);
	}


	return offset;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_TRACE();

	if (sscanf(buf, "%d", &value) <= 0)
		return count;
	touch_interrupt_control(dev, INTERRUPT_DISABLE);
	td4310_reset_ctrl(dev, value);
	td4310_init(dev);
	touch_interrupt_control(dev, INTERRUPT_ENABLE);
	return count;
}

static ssize_t show_lpwg_fail_reason(struct device *dev, char *buf)
{
	struct td4310_data *d = to_td4310_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret += sprintf(buf+ret, "LPWG_FAIL_REASON : [%d]\n", d->lpwg_fail_reason);

	return ret;
}

static ssize_t store_lpwg_fail_reason(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
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

static ssize_t show_esd_test(struct device *dev, char *buf)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);

	if(atomic_read(&d->state.esd_recovery) == ESD_RECOVERY_DONE) {
		atomic_set(&d->state.scan_pdt, true);
		atomic_set(&d->state.init, IC_INIT_NEED);
		atomic_set(&d->state.esd_recovery, ESD_RECOVERY_NEED);
		TOUCH_I("###### ESD Detected !!! - Call ESD Recovery ######\n");
		queue_delayed_work(ts->wq, &ts->panel_reset_work, 0);
	} else {
		TOUCH_I("###### ESD Recovery - In Progress.....\n");
	}

	return ret;
}

static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(pen_support, show_pen_support, NULL);
static TOUCH_ATTR(ts_noise, show_check_noise, store_check_noise);
static TOUCH_ATTR(ts_noise_log_enable, show_noise_log, store_noise_log);
static TOUCH_ATTR(fw_recovery, show_fw_recovery, NULL);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(lpwg_fail_reason, show_lpwg_fail_reason, store_lpwg_fail_reason);
static TOUCH_ATTR(esd_test, show_esd_test, NULL);

static struct attribute *td4310_attribute_list[] = {
	&touch_attr_reg_ctrl.attr,
	&touch_attr_pen_support.attr,
	&touch_attr_ts_noise.attr,
	&touch_attr_ts_noise_log_enable.attr,
	&touch_attr_fw_recovery.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_lpwg_fail_reason.attr,
	&touch_attr_esd_test.attr,
	NULL,
};

static const struct attribute_group td4310_attribute_group = {
	.attrs = td4310_attribute_list,
};

static int td4310_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &td4310_attribute_group);
	if (ret < 0)
		TOUCH_E("td4310 sysfs register failed\n");

	td4310_prd_register_sysfs(dev);

	return 0;
}

static int td4310_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int td4310_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case CMD_VERSION:
		ret = td4310_get_cmd_version(dev, (char *)output);
		break;

	case CMD_ATCMD_VERSION:
		ret = td4310_get_cmd_atcmd_version(dev, (char *)output);
		break;

	default:
		break;
	}

	return ret;
}

static int td4310_fw_compare(struct device *dev, const struct firmware *fw)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	struct td4310_version *device = &d->ic_info.version;
	struct td4310_version *binary = NULL;
	int update = 0;

	TOUCH_TRACE();

	memcpy(d->ic_info.img_product_id, &fw->data[d->ic_info.fw_pid_addr], 6);
	memcpy(d->ic_info.img_raws, &fw->data[d->ic_info.fw_ver_addr], 4);
	d->ic_info.img_version.major = (d->ic_info.img_raws[3] & 0x80 ? 1 : 0);
	d->ic_info.img_version.minor = (d->ic_info.img_raws[3] & 0x7F);
	binary = &d->ic_info.img_version;
	d->ic_info.bootloader_type = ((fw->data[0x06] & 0x08) ? 1 : 0);

	if (ts->force_fwup) {
		update = 1;
		goto finish;
	}

	if (!ts->role.use_fw_upgrade) {
		update = 0;
		goto finish;
	}

	if ((binary->major == 0) && (device->major == 0)) {
		if (binary->minor > device->minor) {
			update = 1;
		}
		goto finish;
	}

	if (binary->major != device->major) {
		update = 1;
	} else {
		if (binary->minor != device->minor)
			update = 1;
		else if (binary->build > device->build)
			update = 1;
	}

finish:
	TOUCH_I("%s : binary[%d.%02d.%d] device[%d.%02d.%d]" \
			" -> update: %d, force: %d, use_upgrade: %d\n", __func__,
			binary->major, binary->minor, binary->build,
			device->major, device->minor, device->build,
			update, ts->force_fwup, ts->role.use_fw_upgrade);
	TOUCH_I("%s : fw bootloader_type = %s",
			__func__,d->ic_info.bootloader_type ? "Optimized" : "Non-Optimized");

	return update;
}
static int td4310_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = to_td4310_data(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;

	TOUCH_TRACE();

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from default_fwpath:%s\n", ts->def_fwpath[0]);
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

	/* TD4310 fw_recovery if F35 exist*/
	if (atomic_read(&d->state.fw_recovery) == true) {
		ret = td4310_recovery(dev);
		if (ret < 0) {
			TOUCH_E("Firmware Recovery failed\n");
			release_firmware(fw);
			return -EPERM;
		}
		atomic_set(&d->state.scan_pdt,true);
		atomic_set(&d->state.fw_recovery, false);
	} else if (td4310_fw_compare(dev, fw) == 1) {
		atomic_set(&d->state.scan_pdt,true);
		ret = FirmwareUpgrade(dev, fw); //TBD: check all return
		if (ret < 0) {
			TOUCH_E("Firmware Upgrade failed\n");
			release_firmware(fw);
			return ret;
		}
	} else {
		release_firmware(fw);
		return -EPERM;
	}

	td4310_reset_ctrl(dev, SW_RESET);
	release_firmware(fw);

	return 0;
}

int td4310_init(struct device *dev)
{
	struct td4310_data *d = to_td4310_data(dev);
	struct touch_core_data *ts = to_touch_core(dev);
	int ret;

	TOUCH_TRACE();

	atomic_set(&d->state.init, IC_INIT_NEED);

	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, atomic_read(&d->state.charger));

	ret = td4310_ic_info(dev);
	if (ret < 0) {
		TOUCH_E("Failed to get ic info\n");
		if(atomic_read(&d->state.fw_recovery) == true) {
			ret = td4310_recovery(dev);
			if (ret < 0) {
				TOUCH_E("Firmware Recovery failed\n");
				if (atomic_read(&d->state.panel_reset_flag) < PANEL_RESET_MAX) {
					goto panel_reset;
				}
			}
			atomic_set(&d->state.scan_pdt,true);
			atomic_set(&d->state.fw_recovery,false);
			ret = td4310_ic_info(dev);
			if (ret < 0) {
				TOUCH_E("Failed to get ic info\n");
				if (atomic_read(&d->state.panel_reset_flag) < PANEL_RESET_MAX) {
					goto panel_reset;
				}
			}
		}
	}

	td4310_lpwg_mode(dev);
	td4310_irq_clear(dev);
	td4310_set_configured(dev);
	td4310_rmidev_init(dev);

	atomic_set(&d->state.init, IC_INIT_DONE);
	atomic_set(&d->state.esd_recovery, ESD_RECOVERY_DONE);
	atomic_set(&d->state.panel_reset_flag, PANEL_RESET_NEED);

	/* update stylus state */
	ret = td4310_update_stylus_mode(dev);
	return 0;

panel_reset:
	atomic_inc(&d->state.panel_reset_flag);
	mod_delayed_work(ts->wq, &ts->panel_reset_work, 0);
	return 0;
}

static int td4310_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct td4310_data *d = NULL;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate td4310 data\n");
		return -ENOMEM;
	}

	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0); //[bringup] TD4300 is set 1

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	td4310_init_tci_info(dev);
	pm_qos_add_request(&d->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	atomic_set(&d->state.scan_pdt,true);
	d->ic_info.fw_pid_addr = 0x10; //[bringup] need to check
	d->ic_info.fw_ver_addr = 0x1d100;
	d->lpwg_fail_reason = 1;
	d->uBL_addr = 0x2c;/* uBL mode i2c slave address */

	return 0;
}

static struct touch_driver touch_driver = {
	.probe = td4310_probe,
	.remove = td4310_remove,
	.suspend = td4310_suspend,
	.resume = td4310_resume,
	.init = td4310_init,
	.upgrade = td4310_upgrade,
	.irq_handler = td4310_irq_handler,
	.power = td4310_power,
	.lpwg = td4310_lpwg,
	.notify = td4310_notify,
	.register_sysfs = td4310_register_sysfs,
	.set = td4310_set,
	.get = td4310_get,
};

#define MATCH_NAME			"synaptics,TD4310"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{},
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

	if (lge_get_panel_type() != SF3F_TD4310) {
		TOUCH_I("%s, TD4310 not found.\n", __func__);
		return 0;
	}

	TOUCH_I("%s, TD4310 found!\n", __func__);

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
