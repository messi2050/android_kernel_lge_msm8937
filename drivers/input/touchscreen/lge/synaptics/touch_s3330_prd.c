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
#include <linux/firmware.h>

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

static char line[98304] = {0};
int UpperImage[TRX_MAX][TRX_MAX];
int LowerImage[TRX_MAX][TRX_MAX];

int Read8BitRegisters(struct device *dev, unsigned short regAddr,
				unsigned char *data, int length)
{
	return synaptics_read(dev, regAddr, data, length);
}

int Write8BitRegisters(struct device *dev, unsigned short regAddr,
				unsigned char *data, int length)
{
	return synaptics_write(dev, regAddr, data, length);
}

static void log_file_size_check(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int mfts_mode = 0;

	TOUCH_TRACE();

	set_fs(KERNEL_DS);

	mfts_mode = lge_get_mfts_mode();

	switch (mfts_mode) {
	case 0:
		if (ts->factory_boot)
			fname = "/data/touch/touch_self_test.txt";
		else
			fname = "/sdcard/touch_self_test.txt";
		break;
	case 1:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mfts_mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n", __func__, fname, file_size);

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
				TOUCH_I("%s : file [%s] exist\n", __func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n", __func__, buf1);
				} else {
					sprintf(buf2, "%s.%d", fname, (i + 1));

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

void write_file(struct device *dev, char *data, int write_time)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int mfts_mode = 0;

	TOUCH_TRACE();

	set_fs(KERNEL_DS);

	mfts_mode = lge_get_mfts_mode();

	switch (mfts_mode) {
	case 0:
		if (ts->factory_boot)
			fname = "/data/touch/touch_self_test.txt";
		else
			fname = "/sdcard/touch_self_test.txt";
		break;
	case 1:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mfts_mode\n", __func__);
		break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time == TIME_INFO_WRITE) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
					sys_tz.tz_minuteswest * 60 * (-1),
					&my_date);
			snprintf(time_string, 64,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static int sdcard_spec_file_read(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	int ret = 0;
	int fd = 0;
	const char *path[2] = {ts->panel_spec, ts->panel_spec_mfts};
	int mfts_mode = 0;
	mm_segment_t old_fs = get_fs();

	TOUCH_TRACE();

	mfts_mode = lge_get_mfts_mode();

	set_fs(KERNEL_DS);

	if (mfts_mode && ts->factory_boot)
		mfts_mode = 1;
	else
		mfts_mode = 0;

	fd = sys_open(path[mfts_mode], O_RDONLY, 0);
	sys_chmod(path[mfts_mode], 0666);
	if (fd >= 0) {
		sys_read(fd, line, sizeof(line));
		sys_close(fd);
		TOUCH_I("%s file existing\n", path[mfts_mode]);
		ret = 1;
	}
	set_fs(old_fs);

	return ret;
}

static int spec_file_read(struct device *dev)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fwlimit = NULL;
	const char *path[2] = { ts->panel_spec, ts->panel_spec_mfts };
	int mfts_mode = 0;

	TOUCH_TRACE();

	mfts_mode = lge_get_mfts_mode();

	if (mfts_mode && ts->factory_boot)
		mfts_mode = 1;
	else
		mfts_mode = 0;

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		TOUCH_I("panel_spec_file name is null\n");
		ret = -1;
		goto error;
	}

	if (request_firmware(&fwlimit, path[mfts_mode], dev) < 0) {
		TOUCH_I("request ihex is failed in normal mode\n");
		ret = -1;
		goto error;
	}

	if (fwlimit->data == NULL) {
		ret = -1;
		TOUCH_I("fwlimit->data is NULL\n");
		goto error;
	}

	strlcpy(line, fwlimit->data, sizeof(line));

error:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

int synaptics_get_limit(struct device *dev, char *breakpoint,
			unsigned char Tx, unsigned char Rx,
			int limit_data[TRX_MAX][TRX_MAX])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int tx_num = 0, rx_num = 0;
	int ret = 0;
	char *found;
	int file_exist = 0;

	TOUCH_TRACE();

	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	file_exist = sdcard_spec_file_read(dev);
	if (!file_exist) {
		ret = spec_file_read(dev);
		if (ret == -1)
			goto error;
	}

	if (line == NULL) {
		ret =  -1;
		goto error;
	}

	found = strnstr(line, breakpoint, sizeof(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_I("failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -1;
		goto error;
	}

	memset(limit_data, 0, (TRX_MAX * TRX_MAX) * 4);

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') &&
					(line[q - p] <= '9'); p++) {
				limit_data[tx_num][rx_num] +=
					((line[q - p] - '0') * cipher);
				cipher *= 10;

			}
			if (line[q - p] == '-') {
				limit_data[tx_num][rx_num] = (-1) *
					(limit_data[tx_num][rx_num]);
			}
			r++;

			if (r % (int)Rx == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;

		if (r == (int)Tx * (int)Rx) {
			TOUCH_I("panel_spec_file scanning is success\n");
			break;
		}
	}

error:
	return ret;
}

static void firmware_version_log(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int mfts_mode = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	mfts_mode = lge_get_mfts_mode();

	if (mfts_mode)
		ret = synaptics_ic_info(dev);

	ret = snprintf(buffer, LOG_BUF_SIZE, "======== Firmware Info ========\n");

	if (d->fw.version[0] > 0x50) {
		ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : %s\n", d->fw.version);
	} else {
		ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d\n",
			((d->fw.version[3] & 0x80) >> 7),
			(d->fw.version[3] & 0x7F));
	}

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"product id : %s\n", d->fw.product_id);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"===============================\n\n");

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static void production_info_log(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int i = 0;
	unsigned char buf;
	unsigned char block_size;
	unsigned char offset[2] = {0,};
	unsigned char length[2] = {0,};
	unsigned char property[5] = {0,};
	unsigned char data[LOG_BUF_SIZE * 2] = {0,};
	unsigned char buffer[LOG_BUF_SIZE] = {0,};

	TOUCH_TRACE();

	ret = snprintf(buffer, LOG_BUF_SIZE, "======== Production Info ========\n");

	synaptics_read(dev, FLASH_PROPERTY_REG, &property, 3);
	block_size = (property[2] << 8) | property[1];

	buf = 0x05;
	synaptics_write(dev, PARTITION_ID_REG, &buf, sizeof(buf));
	synaptics_write(dev, BLOCK_OFFSET_REG, &offset, sizeof(offset));

	length[0] = 0x14;
	length[1] = 0x00;
	synaptics_write(dev, TRANSFER_LENGTH_REG, &length, sizeof(length));

	buf = 0x02;
	synaptics_write(dev, PROGRAMING_CMD_REG, &buf, sizeof(buf));
	touch_msleep(50);

	synaptics_read(dev, PAYLOAD_REG, &data, block_size);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "TP_Flag : %02X\n", data[0]);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "Program_Name : 0x");

	for (i = 0; i < 5; i++)
		ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "%02X", data[i + 1]);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "\nTime : ");
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
				"20%02X/%02X/%02X - %02Xh %02Xm\n",
				data[6], data[7], data[8], data[9], data[10]);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "Board_ID : ");

	for (i = 0; i < 5; i++) {
		if (i == 4)
			ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, " - %02X", data[i + 11]);
		else
			ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret, "%02X", data[i + 11]);
	}

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
				"\n=================================\n\n");

	write_file(dev, buffer, TIME_INFO_SKIP);
}
#if 0
static int synaptics_abs_rawdata_test(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	synaptics_set_page(dev, ANALOG_PAGE);

	mode = 0x01;
	synaptics_write(dev, ABS_DOZE_CTRL_REG, &mode, sizeof(mode));

	synaptics_set_page(dev, COMMON_PAGE);

	mode = 0x00;
	synaptics_write(dev, DEVICE_CONTROL_REG, &mode, sizeof(mode));
	touch_msleep(50);

	ret = F54Test(dev, eRT_AbsRaw, 0, NULL);

	return ret;
}
#endif
static int synaptics_sd(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int lower_limit = 0;
	int upper_limit = 0;
	int rawdata_ret = 0;
	int jitter_ret = 0;
	int trx_short_ret = 0;
	int high_resistance_ret = 0;
	int rawdata_mode = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (synaptics_is_product(d, "PLG591", 6)) {
		rawdata_mode = eRT_RawImageRT3;
	} else if (synaptics_is_product(d, "PLG636", 6)) {
		rawdata_mode = eRT_FullRawCapacitance;
	}

	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	lower_limit = synaptics_get_limit(dev, "LowerImageLimit",
				TxChannelCount, RxChannelCount, LowerImage);
	upper_limit = synaptics_get_limit(dev, "UpperImageLimit",
				TxChannelCount, RxChannelCount, UpperImage);

	if (lower_limit < 0 || upper_limit < 0) {
		TOUCH_E("[Fail] lower_limit = %d, upper_limit = %d\n",
				lower_limit, upper_limit);
		TOUCH_E("[Fail] Can not check the limit of Raw Cap image\n");
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
				"Can not check the limit of Raw Cap\n");
	} else {
		TOUCH_I("[Success] Can check the limit of Raw Cap image\n");

		rawdata_ret = F54Test(dev, rawdata_mode, 0, NULL);
		touch_msleep(30);
		jitter_ret = F54Test(dev, rawdata_mode, 2, NULL);
		touch_msleep(30);
		trx_short_ret = F54Test(dev, eRT_TRexShort, 0, NULL);
		touch_msleep(200);
		high_resistance_ret = F54Test(dev, eRT_HighResistance, 0, NULL);
		touch_msleep(50);
	}
	if(synaptics_is_product(d, "PLG636", 6)) {
		//jitter_ret = 1;
	}

	d->need_scan_pdt = true;
	synaptics_init(dev);
	touch_msleep(30);

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n========RESULT=======\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "Channel Status : %s",
				(trx_short_ret == 1 && high_resistance_ret == 1)
				? "Pass\n" : "Fail ");

	if (trx_short_ret != 1 || high_resistance_ret != 1) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "(%s/%s)\n",
					(trx_short_ret != 1 ? "0" : "1"),
					(high_resistance_ret != 1 ? "0" : "1"));
	}

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "Raw Data : %s",
				(rawdata_ret == 1 && jitter_ret == 1)
				? "Pass\n" : "Fail ");

	if (rawdata_ret != 1 || jitter_ret != 1) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "(%s/%s)\n",
					(rawdata_ret != 1 ? "0" : "1"),
					(jitter_ret != 1 ? "0" : "1"));
	}

	return ret;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not sd.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	/* file create , time log */
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test Start\n");

	firmware_version_log(dev);
	production_info_log(dev);

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		ret = synaptics_sd(dev, buf);
	} else {
		TOUCH_E("Show_sd Test Error!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	write_file(dev, "Show_sd Test End\n\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test End\n");
	log_file_size_check(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not delta_test.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "======== delta ========\n");
		ret = F54Test(dev, eRT_Normalized16BitImageReport, 1, buf);
	} else {
		TOUCH_E("Delta Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts  = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int rawdata_mode = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (synaptics_is_product(d, "PLG591", 6)) {
		rawdata_mode = eRT_RawImageRT3;
	} else if (synaptics_is_product(d, "PLG636", 6)) {
		rawdata_mode = eRT_FullRawCapacitance;
	}
	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not rawdata_test.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "======== rawdata ========\n");
		ret = F54Test(dev, rawdata_mode, 1, buf);
	} else {
		TOUCH_E("Rawdata Jitter Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	synaptics_init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_rawdata_jitter(struct device *dev, char *buf)
{
	struct touch_core_data *ts  = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int rawdata_mode = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (synaptics_is_product(d, "PLG591", 6)) {
		rawdata_mode = eRT_RawImageRT3;
	} else if (synaptics_is_product(d, "PLG636", 6)) {
		rawdata_mode = eRT_FullRawCapacitance;
	}
	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not rawdata_test.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
						"======== rawdata jitter ========\n");

		ret = F54Test(dev, rawdata_mode, 3, buf);
	} else {
		TOUCH_E("Rawdata Jitter Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	synaptics_init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_chstatus(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int trx_short_ret = 0;
	int high_resistance_ret = 0;
	int hybrid_abs_ret = 0;
	int tx_open_ret = 0;
	int rawdata_mode = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (synaptics_is_product(d, "PLG591", 6)) {
		rawdata_mode = eRT_RawImageRT3;
	} else if (synaptics_is_product(d, "PLG636", 6)) {
		rawdata_mode = eRT_FullRawCapacitance;
	}
	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not chstatus test.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		trx_short_ret = F54Test(dev, eRT_TRexShort, 0, NULL);
		high_resistance_ret = F54Test(dev, eRT_HighResistance, 0, NULL);
		hybrid_abs_ret = F54Test(dev, eRT_HybirdRawCap, 0, NULL);
		tx_open_ret = F54Test(dev, rawdata_mode, 4, NULL);

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "========RESULT=======\n");

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "TRex Short result : %s",
					(trx_short_ret == 1) ? "Pass\n" : "Fail\n");

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "TRex Open result : %s",
					(high_resistance_ret == 1 && hybrid_abs_ret == 1
					&& tx_open_ret == 1) ? "Pass\n" : "Fail ");

		if (high_resistance_ret != 1 || hybrid_abs_ret != 1 || tx_open_ret != 1) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "(%s/%s/%s)\n",
						(high_resistance_ret != 1 ? "0" : "1"),
						(hybrid_abs_ret != 1 ? "0" : "1"),
						(tx_open_ret != 1 ? "0" : "1"));
		}
	} else {
		TOUCH_E("Chstatus Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	d->need_scan_pdt = true;
	synaptics_init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_noise(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int noise_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not noise_test.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		noise_ret = F54Test(dev, eRT_Normalized16BitImageReport, 0, NULL);

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "========RESULT=======\n");

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Noise Test result : %s",
					(noise_ret == 1) ? "Pass\n" : "Fail\n");
	} else {
		TOUCH_E("Noise Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	synaptics_init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_hybrid_abs(struct device *dev, char *buf)
{
	struct touch_core_data *ts  = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int hybrid_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not rawdata_test.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		hybrid_ret = F54Test(dev, eRT_HybirdRawCap, 0, buf);

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "========RESULT=======\n");

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Hybrid Abs Test result : %s",
					(hybrid_ret == 1) ? "Pass\n" : "Fail\n");
	} else {
		TOUCH_E("Hybriad Abs Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	synaptics_init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_tx_open(struct device *dev, char *buf)
{
	struct touch_core_data *ts  = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int tx_open_ret = 0;
	int rawdata_mode = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (synaptics_is_product(d, "PLG591", 6)) {
		rawdata_mode = eRT_RawImageRT3;
	} else if (synaptics_is_product(d, "PLG636", 6)) {
		rawdata_mode = eRT_FullRawCapacitance;
	}
	if (d->power_state == POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD off!!!. Can not rawdata_test.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		tx_open_ret = F54Test(dev, rawdata_mode, 4, buf);

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "========RESULT=======\n");

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Tx Open Test result : %s",
					(tx_open_ret == 1) ? "Pass\n" : "Fail\n");
	} else {
		TOUCH_E("Tx Open Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	synaptics_init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
}

static int synaptics_lpwg_sd(struct device *dev, char *buf)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;
	int lpwg_abs_rawdata_ret = 0;
	int lpwg_abs_jitter_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (d->need_scan_pdt) {
		SCAN_PDT(dev);
		d->need_scan_pdt = false;
	}

	touch_msleep(200);
	lpwg_abs_rawdata_ret = F54Test(dev, eRT_AbsRaw, 1, NULL);
	touch_msleep(30);
	lpwg_abs_jitter_ret = F54Test(dev, eRT_AbsRaw, 2, NULL);

	if(synaptics_is_product(d, "PLG636", 6)) {
		//lpwg_abs_jitter_ret = 1;
	}
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "========RESULT=======\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "LPWG RawData : %s",
			(lpwg_abs_rawdata_ret == 1 && lpwg_abs_jitter_ret == 1)
			? "Pass\n" : "Fail ");

	if (lpwg_abs_rawdata_ret != 1 || lpwg_abs_jitter_ret != 1) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "(%s/%s)\n",
					(lpwg_abs_rawdata_ret != 1 ? "0" : "1"),
					(lpwg_abs_jitter_ret != 1 ? "0" : "1"));
	}

	d->need_scan_pdt = true;

	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n", __func__);

	if (d->power_state != POWER_SLEEP) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD on!!!. Can not lpwg_sd.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	/* file create , time log */
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test Start\n");

	if (!ts->factory_boot) {
		firmware_version_log(dev);
		production_info_log(dev);
	}

	if (synaptics_is_product(d, "PLG591", 6) || synaptics_is_product(d, "PLG636", 6)) {
		ret = synaptics_lpwg_sd(dev, buf);
	} else {
		TOUCH_E("Show_lpwg_sd Test Error!!!(product_id error)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "-1\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Read Fail Touch IC Info\n");
	}

	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test End\n");
	log_file_size_check(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return ret;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(rawdata_jitter, show_rawdata_jitter, NULL);
static TOUCH_ATTR(chstatus, show_chstatus, NULL);
static TOUCH_ATTR(noise_test, show_noise, NULL);
static TOUCH_ATTR(hybrid_abs, show_hybrid_abs, NULL);
static TOUCH_ATTR(tx_open, show_tx_open, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_rawdata_jitter.attr,
	&touch_attr_chstatus.attr,
	&touch_attr_noise_test.attr,
	&touch_attr_hybrid_abs.attr,
	&touch_attr_tx_open.attr,
	&touch_attr_lpwg_sd.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int s3330_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
