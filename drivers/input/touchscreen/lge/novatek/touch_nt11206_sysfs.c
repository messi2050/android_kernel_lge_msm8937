/* production_test.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/time.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_nt11206.h"


void write_file(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu\n",
			my_date.tm_mon + 1,my_date.tm_mday,
			my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	TOUCH_I("write open %s, fd : %d\n", (fd >= 0)? "success":"fail",fd);
	if (fd >= 0) {
		if (time > 0) {
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);

}
void log_file_size_check(struct device *dev, char *fname)
{
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;

	set_fs(KERNEL_DS);
	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n",
				__func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n",
			__func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n",
						__func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n",
							__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d",
							fname,
							(i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to rename file [%s] -> [%s]\n",
								__func__, buf1, buf2);
						goto error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}
error:
	set_fs(old_fs);
	return;
}
static ssize_t show_sd(struct device *dev, char *buf)
{
	int ret = 0;
	u8 mode = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	mutex_lock(&ts->lock);
	nt11206_interrupt_control(dev, INTERRUPT_DISABLE);
	ret = nt11206_selftest(dev, buf, mode);
	nt11206_interrupt_control(dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return ret;
}
static ssize_t show_lpwgsd(struct device *dev, char *buf)
{
	int ret = 0;
	u8 mode = 1;
	struct touch_core_data *ts = to_touch_core(dev);

	mutex_lock(&ts->lock);
	nt11206_interrupt_control(dev, INTERRUPT_DISABLE);

	ret = nt11206_selftest(dev, buf, mode);
	nt11206_interrupt_control(dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return ret;
}
static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 x_num=0;
	u8 y_num=0;
	__s32 *xdata = NULL;

	mutex_lock(&ts->lock);
	xdata = (__s32*)kmalloc(sizeof(__s32) * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		mutex_unlock(&ts->lock);
		return -1;
	}
	nvt_change_mode(dev, TEST_MODE_2);

	if(nvt_check_fw_status(dev) != 0) {
		TOUCH_E("FW_STATUS FAIL\n");
		if(xdata)
			kfree(xdata);
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}
	nvt_get_fw_info(dev);

	if(nvt_get_fw_pipe(dev) == 0)
		nvt_read_mdata(dev, DIFF_PIPE0_ADDR);
	else
		nvt_read_mdata(dev, DIFF_PIPE1_ADDR);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);

	if(xdata) {
		memset(xdata, 0, 2048 * sizeof(__s32));
		nvt_get_mdata(xdata, &x_num, &y_num);
	}
	ret = snprintf(buf, PAGE_SIZE, "======== Deltadata ========\n");
	for(i=0; i<y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for(j=0; j<x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d ", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

	if(xdata)
		kfree(xdata);
	mutex_unlock(&ts->lock);
	return ret;
}
static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int i = 0;
	int j = 0;
	__s32* xdata = NULL;
	u8 x_num=0;
	u8 y_num=0;

	mutex_lock(&ts->lock);
	xdata = (__s32*)kmalloc(sizeof(__s32) * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		mutex_unlock(&ts->lock);
		return -1;
	}
	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_check_fw_status(dev) != 0) {
		TOUCH_E("FW_STATUS FAIL\n");
		if(xdata)
			kfree(xdata);
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}
	nvt_get_fw_info(dev);

	if(nvt_get_fw_pipe(dev) == 0)
		nvt_read_mdata(dev, RAW_PIPE0_ADDR);
	else
		nvt_read_mdata(dev, RAW_PIPE1_ADDR);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);
	if(xdata) {
		memset(xdata, 0, 2048 * sizeof(__s32));
		nvt_get_mdata(xdata, &x_num, &y_num);
	}

	for(i=0; i<y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for(j=0; j<x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

	if(xdata)
		kfree(xdata);
	mutex_unlock(&ts->lock);
	return ret;
}
static ssize_t show_baseline(struct device *dev, char *buf)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	__s32* xdata = NULL;
	u8 x_num=0;
	u8 y_num=0;

	xdata = (__s32*)kmalloc(sizeof(__s32) * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		return -1;
	}
	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_check_fw_status(dev) != 0) {
		if(xdata)
			kfree(xdata);
		TOUCH_E("FW_STATUS FAIL\n");
		return -EAGAIN;
	}
	nvt_get_fw_info(dev);

	nvt_read_mdata(dev, BASELINE_ADDR);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);
	if(xdata) {
		memset(xdata, 0, 2048 * sizeof(__s32));
		nvt_get_mdata(xdata, &x_num, &y_num);
	}
	for(i=0; i<y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for(j=0; j<x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	if(xdata)
		kfree(xdata);
	return ret;
}
static ssize_t show_pinstate(struct device *dev, char *buf)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	ret = snprintf(buf, PAGE_SIZE,
			"RST:%d, INT:%d, SCL:%d, SDA:%d, VDD:%d, VIO:%d\n",
			gpio_get_value(ts->reset_pin), gpio_get_value(ts->int_pin),
			gpio_get_value(890 + 11), gpio_get_value(890 + 10),
			gpio_get_value(ts->vdd_pin), gpio_get_value(ts->vio_pin));
	TOUCH_I("%s() buf:%s",__func__, buf);
	return ret;
}

static ssize_t show_rebase(struct device *dev, char *buf)
{
	int ret = 0;
	u8 serial_data[2] = {0x01, 0x47};
	u8 rebase_addr = 0x50;
	u8 rebase_value = 0x23;

	ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
	if (ret < 0) {
		TOUCH_E("%s() serial_data I2C write Fail\n", __func__);
		return ret;
	}
	ret = nt11206_reg_write(dev, rebase_addr, &rebase_value, 1);
	if (ret < 0) {
		TOUCH_E("%s() rebase CMD write Fail\n", __func__);
		return ret;
	}
	return ret;
}
static ssize_t store_read_reg(struct device *dev,
		const char *buf, size_t count)
{
	int value[4] = {0, };
	u8 serial_data[2] = {0, };
	u8 offset = 0;
	u8 *buffer = NULL;
	u8 len = 0;
	int i = 0;
	int ret = -1;

	TOUCH_I("READ REG\n");
	TOUCH_I("Input ADDR_H(HEX) ADDR_L(HEX) OFFSET(HEX) LEN(DEC)\n");
	if (sscanf(buf, "0x%02X 0x%02X 0x%02X %d", &value[0], &value[1], &value[2], &value[3]) <= 0)
		return count;
	len = (u8)value[3];
	buffer = (u8*)kmalloc(len * sizeof(u8), GFP_KERNEL);
	if(buffer == NULL) {
		TOUCH_E("Read Buf Alloc Fail!\n");
		return count;
	}
	serial_data[0] = (u8)value[0];
	serial_data[1] = (u8)value[1];
	ret = nt11206_reg_write(dev, ADDR_CMD, serial_data, 2);
	if(ret < 0) {
		TOUCH_E("serial data I2C write Fail\n");
		if(buffer) {
			kfree(buffer);
		}
		return count;
	}
	offset = (u8)value[2];

	ret = nt11206_reg_read(dev, offset, buffer, len);
	if(ret < 0) {
		TOUCH_E("Reg:0x%02X%02X%02X Read Fail\n", serial_data[0], serial_data[1], offset);
		if(buffer) {
			kfree(buffer);
		}
		return count;
	}
	TOUCH_I("Reg_Addr 0x%02X%02X%02X : ", serial_data[0], serial_data[1], offset);
	for(i = 0;i < len;i++){
		printk("0x%02X ", buffer[i]);
	}
	printk("\n");
	if(buffer) {
		kfree(buffer);
	}
	return count;
}
static ssize_t store_tci_debug(struct device *dev,
		const char *buf, size_t count)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	int value = 0;
	if (sscanf(buf, "%d", &value) <= 0)
		return count;
	d->tci_debug = (u8)value;
	return count;
}
static ssize_t show_tci_debug(struct device *dev, char *buf)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE,
			"tci_debug:%d\n", d->tci_debug);
	return ret;
}
static ssize_t store_power_control(struct device *dev,
		const char *buf, size_t count)
{
	int cmd = 0;
	int ret = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch(cmd) {
		case 0:
			ret = nt11206_power(dev, POWER_OFF);
			break;

		case 1:
			ret = nt11206_power(dev, POWER_ON);
			break;

		case 2:
			ret = nt11206_sw_reset(dev, RESET_MCU_BOOT);
			break;

		case 3:
			nt11206_hw_reset(dev);
			break;

		default:
			TOUCH_I("usage: echo [0|1|2|3] > power_control\n");
			TOUCH_I("0 : power off\n");
			TOUCH_I("1 : power on\n");
			TOUCH_I("2 : sw_reset\n");
			TOUCH_I("3 : hw_reset\n");
			break;
	}

	return count;
}
static ssize_t show_power_control(struct device *dev, char *buf)
{
	int ret = 0;

	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "usage: echo [0|1|2|3] > power_control\n");
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "  0 : power off\n");
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "  1 : power on\n");
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "  2 : sw_reset\n");
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "  3 : hw_reset\n");

	return ret;
}
static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwgsd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(baseline, show_baseline, NULL);
static TOUCH_ATTR(pinstate, show_pinstate, NULL);
static TOUCH_ATTR(rebase, show_rebase, NULL);
static TOUCH_ATTR(reg_read, NULL, store_read_reg);
static TOUCH_ATTR(tci_debug, show_tci_debug, store_tci_debug);
static TOUCH_ATTR(power_control, show_power_control, store_power_control);
static struct attribute *nt11206_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_pinstate.attr,
	&touch_attr_baseline.attr,
	&touch_attr_rebase.attr,
	&touch_attr_reg_read.attr,
	&touch_attr_tci_debug.attr,
	&touch_attr_power_control.attr,
	NULL,
};

static const struct attribute_group nt11206_attribute_group = {
	.attrs = nt11206_attribute_list,
};

int nt11206_touch_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &nt11206_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
