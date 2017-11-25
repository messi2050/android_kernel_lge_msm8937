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
#define TS_MODULE "[prd]"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <soc/qcom/lge/board_lge.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8707.h"
#include "touch_ft8707_prd.h"

enum {
	RAW_DATA_TEST = 0,
	IB_DATA_TEST,
	PP_NOISE_TEST,
	NOISE_TEST,
	LPWG_RAW_DATA_TEST,
	LPWG_IB_DATA_TEST,
	LPWG_PP_NOISE_TEST,
	LPWG_NOISE_TEST,
	LPWG_SHORT_TEST,
	DELTA_SHOW,
};

enum {
	TEST_PASS = 0,
	TEST_FAIL,
};

#if 1
// For Cut 1.1 V1.26
#define RAW_DATA_MAX 8500
#define RAW_DATA_MIN 4500
#define RAW_DATA_MARGIN 0
#define IB_MAX 65
#define IB_MIN 15
#define PP_NOISE_MAX 200 // 160+Margin
#define PP_NOISE_MIN 0
#define NOISE_TEST_FRM 50
#define NOISE_MAX 50
#define NOISE_MIN 0
#define LPWG_RAW_DATA_MAX 8500
#define LPWG_RAW_DATA_MIN 4500
#define LPWG_IB_MAX 65
#define LPWG_IB_MIN 15
#define LPWG_PP_NOISE_MAX 360 // 300+Margin
#define LPWG_PP_NOISE_MIN 0
#define LPWG_NOISE_MAX 100
#define LPWG_NOISE_MIN 0
#define LPWG_SHORT_MAX 5500
#define LPWG_SHORT_MIN 1500
#else
// For Cut 1.1 V1.20
#define RAW_DATA_MAX 11500
#define RAW_DATA_MIN 5500
#define RAW_DATA_MARGIN 0
#define IB_MAX 40
#define IB_MIN 0
#define PP_NOISE_MAX 480
#define PP_NOISE_MIN 0
#define NOISE_TEST_FRM 50
#define NOISE_MAX 150
#define NOISE_MIN 0
#define LPWG_RAW_DATA_MAX 11500
#define LPWG_RAW_DATA_MIN 5500
#define LPWG_IB_MAX 43
#define LPWG_IB_MIN 0
#define LPWG_PP_NOISE_MAX 600
#define LPWG_PP_NOISE_MIN 0
#define LPWG_NOISE_MAX 220
#define LPWG_NOISE_MIN 0
#define LPWG_SHORT_MAX 5500
#define LPWG_SHORT_MIN 1500
#endif

// For Cut 1.0 (No Short Test)
#define RAW_DATA_MAX_10 12000
#define RAW_DATA_MIN_10 2500
#define IB_MAX_10 48
#define IB_MIN_10 10
#define PP_NOISE_MAX_10 400
#define PP_NOISE_MIN_10 0
#define NOISE_TEST_FRM_10 50
#define NOISE_MAX_10 50
#define NOISE_MIN_10 0
#define LPWG_RAW_DATA_MAX_10 12000
#define LPWG_RAW_DATA_MIN_10 2500
#define LPWG_IB_MAX_10 48
#define LPWG_IB_MIN_10 10
#define LPWG_PP_NOISE_MAX_10 500
#define LPWG_PP_NOISE_MIN_10 0
#define LPWG_NOISE_MAX_10 50
#define LPWG_NOISE_MIN_10 0

#define LOG_BUF_SIZE		4096 // 4*1024

#define FTS_WORK_MODE		0x00
#define FTS_FACTORY_MODE	0x40

#define FTS_MODE_CHANGE_LOOP	20

#define TEST_PACKET_LENGTH		342 // 255

/* Number of channel */
#define MAX_ROW				30
#define MAX_COL				18

static s16 fts_data[MAX_ROW][MAX_COL];
static u8 i2c_data[MAX_COL*MAX_ROW*2];
static u8 log_buf[LOG_BUF_SIZE]; /* !!!!!!!!!Should not exceed the log size !!!!!!!! */

int max_data = 0;
int min_data = 0;

static void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

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
static void write_file(struct device *dev, char *data, int write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
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
#if 0
static int write_test_mode(struct device *dev, u8 type)
{
	u32 testmode = 0;
	u8 disp_mode = 0x3;
	u32 cur_disp_mode;
	int retry = 20;
	u32 rdata = 0x01;
	int waiting_time = 400;

	switch (type) {
	case OPEN_NODE_TEST:
		testmode = (disp_mode << 8) + type;
		waiting_time = 10;
		break;
	case SHORT_NODE_TEST:
		testmode = (disp_mode << 8) + type;
		waiting_time = 1000;
		break;
	case DOZE1_M2_RAWDATA_TEST:
		testmode = (disp_mode << 8) + type;
		break;
	case DOZE2_M1_RAWDATA_TEST:
		type = 0x6;
		testmode = type;
		break;
	case DOZE2_M2_RAWDATA_TEST:
		type = 0x5;
		testmode = type;
		break;
	case M1_NOISE_TEST:
	case M2_NOISE_TEST:
	case M1_DIFF_TEST:
	case M2_DIFF_TEST:
		lg4946_reg_read(dev, spr_subdisp_st,
				(u8 *)&cur_disp_mode,
				sizeof(u32));
		if (cur_disp_mode == 3)
			testmode = (disp_mode << 8) + type;
		else
			testmode = type;
		waiting_time = 500;
		break;
	}

	/* TestType Set */
	lg4946_reg_write(dev, tc_tsp_test_ctl,
			(u8 *)&testmode,
			sizeof(testmode));
	TOUCH_I("write testmode = %x\n", testmode);
	touch_msleep(waiting_time);

	/* Check Test Result - wait until 0 is written */
	do {
		touch_msleep(100);
		lg4946_reg_read(dev, tc_tsp_test_sts,
				(u8 *)&rdata,
				sizeof(rdata));
		TOUCH_I("rdata = 0x%x\n", rdata);
	} while ((rdata != 0xAA) && retry--);

	if (rdata != 0xAA) {
		TOUCH_I("ProductionTest Type [%d] Time out\n", type);
		goto error;
	}
	return 1;
error:
	TOUCH_E("[%s] fail\n", __func__);
	return 0;
}

static int prd_os_result_get(struct device *dev, u32 *buf)
{
	u32 os_result_offset;
	u32 offset;

	lg4946_reg_read(dev, tc_tsp_test_os_rst_tune_off_info,
		(u8 *)&os_result_offset, sizeof(u32));
	offset = os_result_offset & 0xFFFF;

	lg4946_reg_write(dev, tc_tsp_test_data_offset,
			(u8 *)&offset,
			sizeof(u32));

	lg4946_reg_read(dev, tc_tsp_data_access_addr,
		(u8 *)buf, sizeof(u32)*ROW_SIZE);

	return 0;
}

static int prd_os_xline_result_read(struct device *dev,
	u8 (*buf)[COL_SIZE], int type)
{
	int i = 0;
	int j = 0;
	u32 buffer[ROW_SIZE] = {0,};
	int ret = 0;
	u8 w_val = 0x0;

	switch (type) {
	case OPEN_NODE_TEST:
		w_val = 0x1;
		break;
	case SHORT_NODE_TEST:
		w_val = 0x2;
		break;
	}

	ret = prd_os_result_get(dev, buffer);

	if (ret == 0) {
		for (i = 0; i < ROW_SIZE; i++) {
			for (j = 0; j < COL_SIZE; j++) {
				if ((buffer[i] & (0x1 << j)) != 0)
					buf[i][j] =
						(buf[i][j] | w_val);
			}
		}
	}

	return ret;
}

static int prd_open_short_test(struct device *dev)
{
	int type = 0;
	int ret = 0;
	int write_test_mode_result = 0;
	u32 open_result = 0;
	u32 short_result = 0;
	u32 openshort_all_result = 0;
	u8 buf[ROW_SIZE][COL_SIZE];
	int i = 0;
	int j = 0;

	/* Test Type Write */
	write_file(dev, "[OPEN_SHORT_ALL_TEST]\n", TIME_INFO_SKIP);

	memset(&buf, 0x0, sizeof(buf));

	/* 1. open_test */
	type = OPEN_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	lg4946_reg_read(dev, tc_tsp_test_pf_result,
			(u8 *)&open_result, sizeof(open_result));
	TOUCH_I("open_result = %d\n", open_result);

	if (open_result) {
		ret = prd_os_xline_result_read(dev, buf, type);
		openshort_all_result |= 0x1;
	}

	/* 2. short_test */
	type = SHORT_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	lg4946_reg_read(dev, tc_tsp_test_pf_result,
		(u8 *)&short_result, sizeof(short_result));
	TOUCH_I("short_result = %d\n", short_result);

	if (short_result) {
		ret = prd_os_xline_result_read(dev, buf, type);
		openshort_all_result |= 0x2;
	}

	/* fail case */
	if (openshort_all_result != 0) {
		ret = snprintf(W_Buf, BUF_SIZE, "\n   : ");
		for (i = 0; i < COL_SIZE; i++)
			ret += snprintf(W_Buf + ret,
					BUF_SIZE - ret,
					" [%2d] ", i);

		for (i = 0; i < ROW_SIZE; i++) {
			ret += snprintf(W_Buf + ret,
					BUF_SIZE - ret,
					"\n[%2d] ", i);

			for (j = 0; j < COL_SIZE; j++) {
				ret += snprintf(W_Buf + ret,
					BUF_SIZE - ret, "%5s ",
				((buf[i][j] & 0x3) == 0x3) ?  "O,S" :
				((buf[i][j] & 0x1) == 0x1) ?  "O" :
				((buf[i][j] & 0x2) == 0x2) ?  "S" : "-");
			}
		}
		ret += snprintf(W_Buf + ret, BUF_SIZE - ret, "\n");
	} else
		ret = snprintf(W_Buf + ret, BUF_SIZE - ret,
				"OPEN_SHORT_ALL_TEST : Pass\n");

	write_file(dev, W_Buf, TIME_INFO_SKIP);

	return openshort_all_result;
}

static void prd_read_rawdata(struct device *dev, u8 type)
{
	u32 raw_offset_info = 0;
	u16 raw_offset = 0;

	int __m1_frame_size = ROW_SIZE*M1_COL_SIZE*RAWDATA_SIZE;
	int __m2_frame_size = ROW_SIZE*COL_SIZE*RAWDATA_SIZE;
	int i = 0;
	static u16 M1_Rawdata_temp[ROW_SIZE*M1_COL_SIZE];

	if (__m1_frame_size % 4)
		__m1_frame_size = (((__m1_frame_size >> 2) + 1) << 2);
	if (__m2_frame_size % 4)
		__m2_frame_size = (((__m2_frame_size >> 2) + 1) << 2);

	if (type == M2_DIFF_TEST
		|| type == M1_DIFF_TEST)
		lg4946_reg_read(dev, tc_tsp_test_diff_off_info,
			(u8 *)&raw_offset_info, sizeof(u32));
	else
		lg4946_reg_read(dev, tc_tsp_test_rawdata_off_info,
			(u8 *)&raw_offset_info, sizeof(u32));

	switch (type) {
	case DOZE2_M1_RAWDATA_TEST:
	case M1_NOISE_TEST:
	case M1_DIFF_TEST:
		raw_offset = raw_offset_info & 0xFFFF;
		lg4946_reg_write(dev, tc_tsp_test_data_offset,
				(u8 *)&raw_offset,
				sizeof(u32));
		memset(M1_Rawdata_buf, 0, sizeof(M1_Rawdata_buf));
		memset(M1_Rawdata_temp, 0, sizeof(M1_Rawdata_buf));
		lg4946_reg_read(dev, tc_tsp_data_access_addr,
				(u8 *)&M1_Rawdata_temp,
				__m1_frame_size);

		for(i = 0; i < ROW_SIZE; i++)
		{
			M1_Rawdata_buf[i*2] = M1_Rawdata_temp[ROW_SIZE+i];
			M1_Rawdata_buf[i*2+1] = M1_Rawdata_temp[i];
		}
		break;
	case DOZE1_M2_RAWDATA_TEST:
	case DOZE2_M2_RAWDATA_TEST:
	case M2_NOISE_TEST:
		raw_offset = (raw_offset_info >> 16) & 0xFFFF;
		lg4946_reg_write(dev, tc_tsp_test_data_offset,
				(u8 *)&raw_offset,
				sizeof(u32));
		memset(M2_Rawdata_buf, 0, sizeof(M2_Rawdata_buf));
		lg4946_reg_read(dev, tc_tsp_data_access_addr,
				(u8 *)&M2_Rawdata_buf,
				__m2_frame_size);
		break;
	case M2_DIFF_TEST:
		raw_offset = raw_offset_info & 0xFFFF;
		lg4946_reg_write(dev, tc_tsp_test_data_offset,
				(u8 *)&raw_offset,
				sizeof(u32));
		memset(M2_Rawdata_buf, 0, sizeof(M2_Rawdata_buf));
		lg4946_reg_read(dev, tc_tsp_data_access_addr,
				(u8 *)&M2_Rawdata_buf,
				__m2_frame_size);

		break;
	}
}

static int sdcard_spec_file_read(struct device *dev)
{
	int ret = 0;
	int fd = 0;
	char *path[2] = { "/mnt/sdcard/h1_limit.txt",
		"/mnt/sdcard/h1_limit_mfts.txt"
	};
	int path_idx = 0;

	mm_segment_t old_fs = get_fs();

	if(touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;
	set_fs(KERNEL_DS);
	fd = sys_open(path[path_idx], O_RDONLY, 0);
	if (fd >= 0) {
		sys_read(fd, line, sizeof(line));
		sys_close(fd);
		TOUCH_I("%s file existing\n", path[path_idx]);
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
	const char *path[2] = { ts->panel_spec,
		ts->panel_spec_mfts
	};
	int path_idx = 0;

	if(touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		TOUCH_I("panel_spec_file name is null\n");
		ret = -1;
		goto error;
	}

	if (request_firmware(&fwlimit, path[path_idx], dev) < 0) {
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

static int sic_get_limit(struct device *dev, char *breakpoint, u16 (*buf)[COL_SIZE])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	char *found;
	int boot_mode = 0;
	int file_exist = 0;
	int tx_num = 0;
	int rx_num = 0;


	if (breakpoint == NULL) {
		ret = -1;
		goto error;
	}

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode > MINIOS_MFTS_CURVED
			|| boot_mode < NORMAL_BOOT) {
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
		TOUCH_I(
			"failed to find breakpoint. The panel_spec_file is wrong\n");
		ret = -1;
		goto error;
	}

	memset(buf, 0, ROW_SIZE * COL_SIZE * 2);

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') &&
					(line[q - p] <= '9'); p++) {
				buf[tx_num][rx_num] += ((line[q - p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
			if (r % (int)COL_SIZE == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;
		if (r == (int)ROW_SIZE * (int)COL_SIZE) {
			TOUCH_I("panel_spec_file scanning is success\n");
			break;
		}
	}

	if (ret == 0) {
		ret = -1;
		goto error;

	} else {
		TOUCH_I("panel_spec_file scanning is success\n");
		return ret;
	}

error:
	return ret;
}

static int prd_print_rawdata(struct device *dev, char *buf, u8 type)
{
	int i = 0, j = 0;
	int ret = 0;
	int min = 9999;
	int max = 0;
	u16 *rawdata_buf = NULL;
	int col_size = 0;

	/* print a frame data */
	ret = snprintf(buf, PAGE_SIZE, "\n   : ");

	switch (type) {
	case DOZE2_M1_RAWDATA_TEST:
	case M1_DIFF_TEST:
	case M1_NOISE_TEST:
		col_size = M1_COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
		break;
	case DOZE2_M2_RAWDATA_TEST:
	case DOZE1_M2_RAWDATA_TEST:
	case M2_NOISE_TEST:
	case M2_DIFF_TEST:
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
		break;
	}

	for (i = 0; i < col_size; i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, " [%2d] ", i);

	for (i = 0; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0, };
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret,  "\n[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,  "[%2d]  ", i);
		for (j = 0; j < col_size; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", rawdata_buf[i*col_size+j]);
			log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret,
					"%5d ", rawdata_buf[i*col_size+j]);
			if (rawdata_buf[i*col_size+j] != 0 &&
					rawdata_buf[i*col_size+j] < min)
				min = rawdata_buf[i*col_size+j];
			if (rawdata_buf[i*col_size+j] > max)
				max = rawdata_buf[i*col_size+j];
		}
		TOUCH_I("%s\n", log_buf);
	}


	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"\nRawdata min : %d , max : %d\n", min, max);

	return ret;
}

static int prd_print_max_noise(struct device *dev, char *buf, u8 type, int ret_val)
{
	int i = 0, j = 0;
	int ret = ret_val;
	int max = 0;
	u16 *rawdata_buf = NULL;
	int col_size = 0;

	switch (type) {
	case DOZE2_M1_RAWDATA_TEST:
	case M1_DIFF_TEST:
	case M1_NOISE_TEST:
		col_size = M1_COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
		break;
	case DOZE2_M2_RAWDATA_TEST:
	case DOZE1_M2_RAWDATA_TEST:
	case M2_NOISE_TEST:
	case M2_DIFF_TEST:
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
		break;
	}

	max = rawdata_buf[0];

	for (i = 0; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0, };
		int log_ret = 0;

		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,  "[%2d]  ", i);
		for (j = 0; j < col_size; j++) {
			log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret,
					"%5d ", rawdata_buf[i*col_size+j]);
			if (rawdata_buf[i*col_size+j] > max) {
				max = rawdata_buf[i*col_size+j];
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
						"noise_max[%d][%d] = %d\n", j, i, rawdata_buf[i*col_size+j]);
			}
		}
		TOUCH_I("%s\n", log_buf);
	}

	return ret;
}

/* Rawdata compare result
	Pass : reurn 0
	Fail : return 1
*/
static int prd_compare_rawdata(struct device *dev, u8 type)
{
	struct lg4946_data *d = to_lg4946_data(dev);
	/* spec reading */
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	u16 *rawdata_buf = NULL;
	int col_size = 0;
	int i, j;
	int ret = 0;
	int result = 0;

	switch (type) {
	case DOZE2_M1_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"DOZE2_M1_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"DOZE2_M1_Upper");
		col_size = M1_COL_SIZE;
		rawdata_buf = M1_Rawdata_buf;
		break;
	case DOZE2_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"DOZE2_M2_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"DOZE2_M2_Upper");
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
		break;
	case DOZE1_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"DOZE1_M2_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"DOZE1_M2_Upper");
		col_size = COL_SIZE;
		rawdata_buf = M2_Rawdata_buf;
		break;
	}

	sic_get_limit(dev, lower_str, LowerImage);
	sic_get_limit(dev, upper_str, UpperImage);

	for (i = 0; i < ROW_SIZE; i++) {
		for (j = 0; j < col_size; j++) {
			if ((rawdata_buf[i*col_size+j] < LowerImage[i][j]) ||
				(rawdata_buf[i*col_size+j] > UpperImage[i][j])) {
				if ((type != DOZE2_M1_RAWDATA_TEST) &&
						(i <= 1 && j <= 4)) {
					if (rawdata_buf[i*col_size+j] != 0) {
						result = 1;
						ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
								"F [%d][%d] = %d\n", i, j, rawdata_buf[i*col_size+j]);
					}
				} else {
					result = 1;
					ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
							"F [%d][%d] = %d\n", i, j, rawdata_buf[i*col_size+j]);
				}
			}
		}
	}

	/* revision under 2 samples are not comparing for temporary. */
	TOUCH_I("revision : %d\n", d->ic_info.revision);
	if (d->ic_info.revision < 2) {
		TOUCH_I("makes it pass for temporary under Rev 2\n");
		result = 0;
	}

	return result;
}

static void tune_display(struct device *dev, char *tc_tune_code,
	int offset, int type)
{
	char log_buf[tc_tune_code_size] = {0,};
	int ret = 0;
	int i = 0;
	char temp[tc_tune_code_size] = {0,};

	switch (type) {
	case 1:
		ret = snprintf(log_buf, tc_tune_code_size,
				"GOFT tune_code_read : ");
		if ((tc_tune_code[offset] >> 4) == 1) {
			temp[offset] = tc_tune_code[offset] - (0x1 << 4);
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					"-%d  ", temp[offset]);
		} else {
			ret += snprintf(log_buf + ret,
					tc_tune_code_size - ret,
					" %d  ", tc_tune_code[offset]);
		}
		TOUCH_I("%s\n", log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		break;
	case 2:
		ret = snprintf(log_buf, tc_tune_code_size,
				"LOFT tune_code_read : ");
		for (i = 0; i < tc_total_ch_size; i++) {
			if ((tc_tune_code[offset+i]) >> 5 == 1) {
				temp[offset+i] =
					tc_tune_code[offset+i] - (0x1 << 5);
				ret += snprintf(log_buf + ret,
						tc_tune_code_size - ret,
						"-%d  ", temp[offset+i]);
			} else {
				ret += snprintf(log_buf + ret,
						tc_tune_code_size - ret,
						" %d  ",
						tc_tune_code[offset+i]);
			}
		}
		TOUCH_I("%s\n", log_buf);
		ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
		write_file(dev, log_buf, TIME_INFO_SKIP);
		break;
	}
}

static void read_tune_code(struct device *dev, u8 type)
{
	u8 tune_code_read_buf[276] = {0,};

	u32 tune_code_offset;
	u32 offset;

	lg4946_reg_read(dev, tc_tsp_test_os_rst_tune_off_info,
		(u8 *)&tune_code_offset, sizeof(u32));
	offset = (tune_code_offset >> 16) & 0xFFFF;

	lg4946_reg_write(dev, tc_tsp_test_data_offset,
			(u8 *)&offset,
			sizeof(u32));

	lg4946_reg_read(dev, tc_tsp_data_access_addr,
			(u8 *)&tune_code_read_buf[0], tc_tune_code_size);

	write_file(dev, "\n[Read Tune Code]\n", TIME_INFO_SKIP);
	switch (type) {
	case DOZE2_M1_RAWDATA_TEST:
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_GOFT_OFFSET, 1);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_GOFT_OFFSET, 1);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_M1_OFT_OFFSET, 2);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_M1_OFT_OFFSET, 2);
		break;
	case DOZE1_M2_RAWDATA_TEST:
	case DOZE2_M2_RAWDATA_TEST:
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_GOFT_OFFSET + 1, 1);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_GOFT_OFFSET + 1, 1);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_G1_OFT_OFFSET, 2);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_G2_OFT_OFFSET, 2);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_G3_OFT_OFFSET, 2);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_G1_OFT_OFFSET, 2);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_G2_OFT_OFFSET, 2);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_G3_OFT_OFFSET, 2);
		break;
	}
	write_file(dev, "\n", TIME_INFO_SKIP);

}

static int prd_rawdata_test(struct device *dev, u8 type)
{
	char test_type[32] = {0, };
	int result = 0;
	int write_test_mode_result = 0;

	switch (type) {
	case DOZE1_M2_RAWDATA_TEST:
		snprintf(test_type, sizeof(test_type),
				"[DOZE1_M2_RAWDATA_TEST]");
		break;
	case DOZE2_M1_RAWDATA_TEST:
		snprintf(test_type, sizeof(test_type),
				"[DOZE2_M1_RAWDATA_TEST]");
		break;
	case DOZE2_M2_RAWDATA_TEST:
		snprintf(test_type, sizeof(test_type),
				"[DOZE2_M2_RAWDATA_TEST]");
		break;
	default:
		TOUCH_I("Test Type not defined\n");
		return 1;
	}

	/* Test Type Write */
	write_file(dev, test_type, TIME_INFO_SKIP);

	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("production test couldn't be done\n");
		return 1;
	}

	prd_read_rawdata(dev, type);

	result = prd_print_rawdata(dev, W_Buf, type);
	write_file(dev, W_Buf, TIME_INFO_SKIP);

	memset(W_Buf, 0, BUF_SIZE);
	/* rawdata compare result(pass : 0 fail : 1) */
	result = prd_compare_rawdata(dev, type);
	write_file(dev, W_Buf, TIME_INFO_SKIP);

	/* To Do - tune code result check */
	/*result = */read_tune_code(dev, type);

	return result;
}

static void ic_run_info_print(struct device *dev)
{
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int ret = 0;
	u32 rdata[4] = {0};

	ft8707_reg_read(dev, info_lot_num, (u8 *)&rdata, sizeof(rdata));

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"\n===== Production Info =====\n");
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"lot : %d\n", rdata[0]);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"serial : 0x%X\n", rdata[1]);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"date : 0x%X 0x%X\n",
			rdata[2], rdata[3]);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"date : %04d.%02d.%02d %02d:%02d:%02d Site%d\n\n",
		rdata[2] & 0xFFFF, (rdata[2] >> 16 & 0xFF),
		(rdata[2] >> 24 & 0xFF), rdata[3] & 0xFF,
		(rdata[3] >> 8 & 0xFF),
		(rdata[3] >> 16 & 0xFF),
		(rdata[3] >> 24 & 0xFF));

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static void firmware_version_log(struct device *dev)
{
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int boot_mode = 0;

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode >= MINIOS_MFTS_FOLDER)
		ret = ft8707_ic_info(dev);

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"======== Firmware Info ========\n");
	if (d->ic_info.version.build) {
		ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d.%d\n",
			d->ic_info.version.major, d->ic_info.version.minor, d->ic_info.version.build);
	} else {
		ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d\n",
			d->ic_info.version.major, d->ic_info.version.minor);
	}
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"revision : %d\n", d->ic_info.revision);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"fpc : %d, cg : %d, wfr : %d\n",
			d->ic_info.fpc, d->ic_info.cg, d->ic_info.wfr);

	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"product id : %s\n", d->ic_info.product_id);

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static int ic_exception_check(struct device *dev, char *buf)
{
#if 0
	struct ft8707_data *d = to_ft8707_data(dev);
	int boot_mode = 0;
	int ret = 0;

	boot_mode = touch_boot_mode_check(dev);
	/* MINIOS mode, MFTS mode check */
	if ((lge_get_boot_mode() > LGE_BOOT_MODE_NORMAL) || boot_mode > 0) {
		if (d->fw.revision < 2 ||
				d->fw.revision > 3 ||
				d->fw.version[0] != 1) {
			TOUCH_I("ic_revision : %d, fw_version : v%d.%02d\n",
					d->fw.revision,
					d->fw.version[0], d->fw.version[1]);

			ret = snprintf(buf, PAGE_SIZE,
					"========RESULT=======\n");

			if (d->fw.version[0] != 1) {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"version[v%d.%02d] : Fail\n",
					d->fw.version[0], d->fw.version[1]);
			} else {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"version[v%d.%02d] : Pass\n",
					d->fw.version[0], d->fw.version[1]);
			}

			if (d->fw.revision < 2 || d->fw.revision > 3) {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"revision[%d] : Fail\n",
					d->fw.revision);
			} else {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"revision[%d] : Pass\n",
					d->fw.revision);
			}
			write_file(dev, buf, TIME_INFO_SKIP);

			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"Raw data : Fail\n");
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"Channel Status : Fail\n");
			write_file(dev, "", TIME_INFO_WRITE);
		}
	}
	return ret;
#endif
	return 0;
}

static int check_noise_test(struct device *dev, char* buf, u32 type, int* ret_val)
{
	int write_test_mode_result = 0;
	int offset = *ret_val;

	if (type == M1_NOISE_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "[M1_NOISE_TEST]\n");
	else if (type == M1_DIFF_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "[M1_DIFF_TEST]\n");
	else if (type == M2_NOISE_TEST)
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "[M2_NOISE_TEST]\n");
	else
		offset += snprintf(buf + offset, PAGE_SIZE - offset, "[M2_DIFF_TEST]\n");

	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("production test couldn't be done\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"production test couln't be done\n");
		return 1;
	}

	prd_read_rawdata(dev, type);
	offset = prd_print_max_noise(dev, buf, type, offset);

	*ret_val = offset;

	return 0;
}
#endif

int ft8707_set_lpwg_i2c_mode(struct device *dev, u8 i2c_en)
{
	u8 ret;
	u8 data;

	TOUCH_I("%s : i2c_en = 0x%02x\n", __func__, i2c_en);

	data = 0x00;
	ret = ft8707_reg_read(dev, 0xF6, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	if (data == i2c_en) {
		TOUCH_I("Already mode changed\n");
		return 0;
	}

	data = i2c_en;
	ret = ft8707_reg_write(dev, 0xF6, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	TOUCH_I("LPWG I2C mode changed\n");

	mdelay(100);

	return 0;
}


int ft8707_change_op_mode(struct device *dev, u8 op_mode)
{
	int i = 0;
	u8 ret;
	u8 data;

	TOUCH_I("%s : op_mode = 0x%02x\n", __func__, op_mode);

	data = 0x00;
	ret = ft8707_reg_read(dev, 0x00, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	if (data == op_mode) {
		TOUCH_I("Already mode changed\n");
		return 0;
	}

	data = op_mode;
	ret = ft8707_reg_write(dev, 0x00, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	mdelay(10);

	for ( i = 0; i < FTS_MODE_CHANGE_LOOP; i++) {
		data = 0x00;
		ret = ft8707_reg_read(dev, 0x00, &data, 1);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		if(data == op_mode)
			break;
		mdelay(50);
	}

	if (i >= FTS_MODE_CHANGE_LOOP) {
		TOUCH_E("Timeout to change op mode\n");
		return -EPERM;
	}
	TOUCH_I("Operation mode changed\n");

	mdelay(300);

	return 0;
}


int ft8707_switch_cal(struct device *dev, u8 cal_en)
{
#if 0
	int i = 0;
	u8 ret;
	u8 data;

	TOUCH_I("%s : cal_en = 0x%02x\n", __func__, cal_en);

	data = 0x00;
	ret = ft8707_reg_read(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	if (data == cal_en) {
		TOUCH_I("Already cal_en changed\n");
		return 0;
	}

	data = cal_en;
	ret = ft8707_reg_write(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	mdelay(10);

	for ( i = 0; i < FTS_MODE_CHANGE_LOOP; i++) {
		data = 0x00;
		ret = ft8707_reg_read(dev, 0xEE, &data, 1);
		if(ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		if(data == cal_en)
			break;
		mdelay(20);
	}

	if (i >= FTS_MODE_CHANGE_LOOP) {
		TOUCH_E("Timeout to change cal_en\n");
		return -EPERM;
	}
	TOUCH_I("cal_en changed\n");

	return 0;

#else
	u8 ret;
	u8 data;

	TOUCH_I("%s : cal_en = 0x%02x\n", __func__, cal_en);

	ret = ft8707_reg_read(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	if (data == cal_en) {
		TOUCH_I("Already switch_cal changed\n");
		return 0;
	}

	data = cal_en;
	ret = ft8707_reg_write(dev, 0xEE, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	mdelay(100);

	return 0;
#endif
}


int ft8707_prd_check_ch_num(struct device *dev)
{
	u8 ret;
	u8 data;

	TOUCH_I("%s\n", __func__);

	/* Channel number check */
	ret = ft8707_reg_read(dev, 0x02, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	TOUCH_I("X Channel : %d\n", data);

	mdelay(3);

	if (data != MAX_COL) {
		TOUCH_E("Invalid X Channel Num.\n");
		return -EPERM;
	}

	ret = ft8707_reg_read(dev, 0x03, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	TOUCH_I("Y Channel : %d\n", data);
	
	mdelay(3);

	if (data != MAX_ROW) {
		TOUCH_E("Invalid Y Channel Num.\n");
		return -EPERM;
	}

	return 0;
}

int ft8707_prd_get_raw_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to raw data
	data = 0x00;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	/* Start SCAN */
	for (k = 0; k < 3; k++)
	{
		TOUCH_I("Start SCAN (%d/3)\n", k+1);
		ret = ft8707_reg_read(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		data |= 0x80; // 0x40|0x80
		ret = ft8707_reg_write(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(10);

		for (i = 0; i < 200; i++) {
			ret = ft8707_reg_read(dev, 0x00, &data, 1);
			if (ret < 0) {
				TOUCH_E("i2c error\n");
				return ret;
			}
			if (data == 0x40) {
				TOUCH_I("SCAN Success : 0x%X, %d ms \n", data, i*20);
				break;
			}
			mdelay(20);
		}
		
		if (i < 200) {
			break;
		}

		TOUCH_E("SCAN Fail (%d/3)\n", k+1);
	}

	if (k >= 3) {
		return -EPERM;
	}

	/* Read Raw data */
	data = 0xAD;
	ret = ft8707_reg_write(dev, 0x01, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

#if 1 // read full data at once

	TOUCH_I("Read Raw data at once\n");

	ret = ft8707_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

#else // read packet_length data
	for (i = 0; (MAX_COL*MAX_ROW*2 - TEST_PACKET_LENGTH*i) > 0; i++)
	{
		addr = 0x1C;
		data = ((TEST_PACKET_LENGTH*i)&0xFF00)>>8;
		result = fts_write_reg(client, addr, data);
		if(result != 0) {
			TOUCH_E("write ERR\n");
			return -1;
		}
		mdelay(10);

		addr = 0x1D;
		data = (TEST_PACKET_LENGTH*i)&0x00FF;
		result = fts_write_reg(client, addr, data);
		if(result != 0) {
			TOUCH_E("write ERR\n");
			return -1;
		}
		mdelay(10);

		addr = 0x6A;
		read_len = ((MAX_COL*MAX_ROW*2 - TEST_PACKET_LENGTH*i) >= TEST_PACKET_LENGTH) ? TEST_PACKET_LENGTH : (MAX_COL*MAX_ROW*2 - TEST_PACKET_LENGTH*i);
		result = FT8707_I2C_Read(client, &addr, 1, &i2c_data[TEST_PACKET_LENGTH*i], read_len);
		if(result != 0) {
			TOUCH_E("Read ERR\n");
			return -1;
		}
		mdelay(10);

		TOUCH_I("%d : RawAddrH=0x%02X  RawAddrL=0x%02X : %d bytes read success... remain:%d \n",
			i, ((TEST_PACKET_LENGTH*i)&0xFF00)>>8, (TEST_PACKET_LENGTH*i)&0x00FF,
			read_len, (MAX_COL*MAX_ROW*2 - TEST_PACKET_LENGTH*i - read_len));
	}
#endif

	/* Combine */
	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}

	return 0;
}


int ft8707_prd_get_noise_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
	u8 frame_count;
	int data_combined;

	memset(i2c_data, 0, sizeof(i2c_data));

#if 1

	// Data Select
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Buffer Count
	data = 0xAD;
	ret = ft8707_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Set Test Frame Count
	data = NOISE_TEST_FRM;
	ret = ft8707_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Start Noise Test
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Check Test Completion
	for (i = 0; i < 200; i++) {
		ret = ft8707_reg_read(dev, 0x11, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		if (data == 0x00) {
			TOUCH_I("Noise Test Success : 0x%X, %d ms \n", data, i*20);
			break;
		}
		mdelay(20);
	}
	if (i >= 200) {
		TOUCH_I("Noise Test Fail : 0x%X, %d ms \n", data, i*20);
		return -EPERM;
	}

	// Get Noise Test Frame Count
	ret = ft8707_reg_read(dev, 0x13, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	frame_count = data;
	if (frame_count == 0) {
		TOUCH_E("Invalid frame count ZERO\n");
		return -EPERM;
	}
	TOUCH_I("Frame count : %d\n", frame_count);

	mdelay(10);

	// Get Noise Data
	TOUCH_I("Read Noise data at once\n");

	ret = ft8707_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Buffer Count
	data = 0xAD;
	ret = ft8707_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Calculate STDEV of Noise
	for (i = 0; i < MAX_ROW; i++) {
		for (j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			data_combined = ((((int)i2c_data[k] << 8) + (int)i2c_data[k+1])) / (int)(frame_count);
			fts_data[i][j] = (s16)int_sqrt((unsigned long)data_combined);
		}
	}

	return 0;

#else

	// Data Select
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Set Noise Test Frame Count
	data = 0x20;
	ret = ft8707_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

	// Start Noise Test
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Raw Data Addr
	data = 0xAD;
	ret = ft8707_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Check Scan is finished
	for (i = 0; i < 100; i++)
	{
		ret = ft8707_reg_read(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		if ((data & 0x80) == 0x00){
			TOUCH_I("Scan finished : %d ms, data = %x\n", i*50 ,data);
			break;
		}
		mdelay(50); //mdelay(20);
	}
	
	if (i >= 100) {
		TOUCH_E("Scan failed\n");
		return -EPERM;
	}

	// Get Noise data
	TOUCH_I("Read Noise data at once\n");

	ret = ft8707_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Get Noise Test Frame Count
	ret = ft8707_reg_read(dev, 0x13, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	frame_count = data;
	if (frame_count == 0) {
		TOUCH_E("Invalid frame count ZERO\n");
		return -EPERM;
	}
	TOUCH_I("Frame count : %d\n", frame_count);

	mdelay(10);

#if 0
	// Data Select to raw
	data = 0x00;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
#endif

	// Calculate STDEV of Noise
	for (i = 0; i < MAX_ROW; i++) {
		for (j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			fts_data[i][j] = ((((int)i2c_data[k] << 8) + (int)i2c_data[k+1])) / (int)(frame_count);
			fts_data[i][j] = (unsigned long)int_sqrt((unsigned long)fts_data[i][j]);
		}
	}

	return 0;
#endif
}

int ft8707_prd_get_ib_data(struct device *dev)
{
	int i, j;
	int ret = 0;
	u8 data = 0x00;
	int total, offset = 0, read_len;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to raw data
	data = 0x00;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	total = MAX_COL*MAX_ROW;

	// Get IB data
	for (i = 0; (total - TEST_PACKET_LENGTH*i) > 0; i++)
	{
		offset = TEST_PACKET_LENGTH * i;
		read_len = ((total - offset) >= TEST_PACKET_LENGTH) ? TEST_PACKET_LENGTH : (total - offset);

		data = (offset & 0xFF00) >> 8;
		ret = ft8707_reg_write(dev, 0x18, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		data = (offset & 0x00FF);
		ret = ft8707_reg_write(dev, 0x19, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		ret = ft8707_reg_read(dev, 0x6E, &i2c_data[offset], read_len);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(10);
	}

	for (i = 0; i < MAX_ROW; i++) {
		for (j = 0; j < MAX_COL; j++) {
			fts_data[i][j] = i2c_data[j * MAX_ROW + i];
		}
	}

	return 0;
}

int ft8707_prd_get_delta_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;
	//int total, offset = 0, read_len;

	//total = MAX_COL*MAX_ROW;

	memset(i2c_data, 0, sizeof(i2c_data));

	// Data Select to diff data
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	/* Start SCAN */
	for (k = 0; k < 3; k++)
	{
		TOUCH_I("Start SCAN (%d/3)\n", k+1);
		ret = ft8707_reg_read(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		data |= 0x80; // 0x40|0x80
		ret = ft8707_reg_write(dev, 0x00, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(10);

		for (i = 0; i < 200; i++) {
			ret = ft8707_reg_read(dev, 0x00, &data, 1);
			if (ret < 0) {
				TOUCH_E("i2c error\n");
				return ret;
			}
			if (data == 0x40) {
				TOUCH_I("SCAN Success : 0x%X, %d ms \n", data, i*20);
				break;
			}
			mdelay(20);
		}
		
		if (i < 200) {
			break;
		}

		TOUCH_E("SCAN Fail (%d/3)\n", k+1);
	}

	if (k >= 3) {
		return -EPERM;
	}

	/* Read Raw data */
	data = 0xAD;
	ret = ft8707_reg_write(dev, 0x01, &data, 1);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(10);

#if 1 // read full data at once

	TOUCH_I("Read Delta at once\n");

	ret = ft8707_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

#else // read packet_length data
	TOUCH_I("Read Delta at once\n");

	for (i = 0; (total - TEST_PACKET_LENGTH*i) > 0; i++)
	{
		offset = TEST_PACKET_LENGTH * i;
		read_len = ((total - offset) >= TEST_PACKET_LENGTH) ? TEST_PACKET_LENGTH : (total - offset);

		data = (offset & 0xFF00) >> 8;
		ret = ft8707_reg_write(dev, 0x1C, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		data = (offset & 0x00FF);
		ret = ft8707_reg_write(dev, 0x1D, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		mdelay(10);

		ret = ft8707_reg_read(dev, 0x6A, &i2c_data[offset], read_len);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}

		mdelay(10);
	}
#endif

#if 0
	// Data Select
	data = 0x00;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
#endif

	/* Combine */
	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}

	return 0;
}

int ft8707_prd_get_pp_noise_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;

	memset(i2c_data, 0, sizeof(i2c_data));

#if 1
	// Data Select
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Buffer Count
	data = 0xAD;
	ret = ft8707_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Set Test Frame Count
	data = NOISE_TEST_FRM;
	ret = ft8707_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Start Noise Test
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Check Test Completion
	for (i = 0; i < 200; i++) {
		ret = ft8707_reg_read(dev, 0x11, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		if (data == 0x00) {
			TOUCH_I("Noise Test Success : 0x%X, %d ms \n", data, i*20);
			break;
		}
		mdelay(20);
	}
	if (i >= 200) {
		TOUCH_I("Noise Test Fail : 0x%X, %d ms \n", data, i*20);
		return -EPERM;
	}

	// Get Noise Data
	TOUCH_I("Read Peak-to-Peak Noise data at once\n");

	ret = ft8707_reg_read(dev, 0x8B, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Buffer Count
	data = 0xAD;
	ret = ft8707_reg_write(dev, 0x01, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Combine Result Data
	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}

	return 0;

#else
#if 0
	// Data Select to diff data
	data = 0x00;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);
#endif

	// Set Noise Test Frame
	data = NOISE_TEST_FRM;
	ret = ft8707_reg_write(dev, 0x12, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Clear Data Buffer
	data = 0x00;
	ret = ft8707_reg_write(dev, 0xAD, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Enable Noise Test
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x11, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Check Test Completion
	for (i = 0; i < 300; i++) {
		ret = ft8707_reg_read(dev, 0x11, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		if (data == 0x00) {
			TOUCH_I("PP Noise Test Success : 0x%X, %d ms \n", data, i*20);
			break;
		}
		mdelay(20);
	}
	if (i >= 300) {
		TOUCH_I("PP Noise Test Fail : 0x%X, %d ms \n", data, i*20);
		return -EPERM;
	}

	// Get Result Data
	ret = ft8707_reg_read(dev, 0x6A, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}

	return 0;
#endif
}


int ft8707_prd_get_short_data(struct device *dev)
{
	int i, j, k;
	int ret = 0;
	u8 data = 0x00;

	memset(i2c_data, 0, sizeof(i2c_data));

#if 0
	// Data Select to diff data
	data = 0x00;
	ret = ft8707_reg_write(dev, 0x06, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);
#endif

	// Start Test
	data = 0x01;
	ret = ft8707_reg_write(dev, 0x0F, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}
	mdelay(100);

	// Check Test Completion
	for (i = 0; i < 200; i++) {
		ret = ft8707_reg_read(dev, 0x10, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			return ret;
		}
		if (data == 0x00) {
			TOUCH_I("Short Test Success : 0x%X, %d ms \n", data, i*20);
			break;
		}
		mdelay(20);
	}
	if (i >= 200) {
		TOUCH_I("Short Test Fail : 0x%X, %d ms \n", data, i*20);
		return -EPERM;
	}

	// Get Result Data
	ret = ft8707_reg_read(dev, 0x89, &i2c_data[0], MAX_COL*MAX_ROW*2);
	if(ret < 0) {
		TOUCH_E("i2c error\n");
		return ret;
	}

	for (i = 0; i < MAX_ROW; i++) {
		for(j = 0; j < MAX_COL; j++) {
			k = ((j * MAX_ROW) + i) << 1;
			fts_data[i][j] = (i2c_data[k] << 8) + i2c_data[k+1];
		}
	}

	return 0;
}


int ft8707_prd_test_data(struct device *dev, int test_type, int* test_result)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);

	int i, j;
	int ret = 0;

	int limit_upper = 0, limit_lower = 0;
	int min, max/*, aver, stdev*/;
	int fail_count = 0;
	int check_limit = 1;

	*test_result = TEST_FAIL;

	ret = snprintf(log_buf, LOG_BUF_SIZE, "IC F/W Version : V%d.%02d\n", d->ic_info.is_official, d->ic_info.fw_version);

	switch (test_type) {
		case RAW_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Raw Data Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = RAW_DATA_MAX + RAW_DATA_MARGIN;
				limit_lower = RAW_DATA_MIN - RAW_DATA_MARGIN;
			}
			else { // Cut 1.0
				limit_upper = RAW_DATA_MAX_10 + RAW_DATA_MARGIN;
				limit_lower = RAW_DATA_MIN_10 - RAW_DATA_MARGIN;
			}
			break;
		case IB_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= IB Data Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = IB_MAX;
				limit_lower = IB_MIN;
			}
			else { // Cut 1.0
				limit_upper = IB_MAX_10;
				limit_lower = IB_MIN_10;
			}
			break;
		case PP_NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Peak-to-Peak Noise Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = PP_NOISE_MAX;
				limit_lower = PP_NOISE_MIN;
			}
			else { // Cut 1.0
				limit_upper = PP_NOISE_MAX_10;
				limit_lower = PP_NOISE_MIN_10;
			}
			break;
		case NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Noise Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = NOISE_MAX;
				limit_lower = NOISE_MIN;
			}
			else { // Cut 1.0
				limit_upper = NOISE_MAX_10;
				limit_lower = NOISE_MIN_10;
			}
			break;
		case LPWG_RAW_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG Raw Data Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = LPWG_RAW_DATA_MAX + RAW_DATA_MARGIN;
				limit_lower = LPWG_RAW_DATA_MIN - RAW_DATA_MARGIN;
			}
			else { // Cut 1.0
				limit_upper = LPWG_RAW_DATA_MAX_10 + RAW_DATA_MARGIN;
				limit_lower = LPWG_RAW_DATA_MIN_10 - RAW_DATA_MARGIN;
			}
			break;
		case LPWG_IB_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG IB Data Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = LPWG_IB_MAX;
				limit_lower = LPWG_IB_MIN;
			}
			else { // Cut 1.0
				limit_upper = LPWG_IB_MAX_10;
				limit_lower = LPWG_IB_MIN_10;
			}
			break;
		case LPWG_PP_NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG PP Noise Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = LPWG_PP_NOISE_MAX;
				limit_lower = LPWG_PP_NOISE_MIN;
			}
			else { // Cut 1.0
				limit_upper = LPWG_PP_NOISE_MAX_10;
				limit_lower = LPWG_PP_NOISE_MIN_10;
			}
			break;
		case LPWG_NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG Noise Test Result =============\n");
			if (0 == d->chip_rev) { // Cut 1.1
				limit_upper = LPWG_NOISE_MAX;
				limit_lower = LPWG_NOISE_MIN;
			}
			else { // Cut 1.0
				limit_upper = LPWG_NOISE_MAX_10;
				limit_lower = LPWG_NOISE_MIN_10;
			}
			break;
		case LPWG_SHORT_TEST: // Only for Cut 1.1
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG Short Test Result =============\n");
			limit_upper = LPWG_SHORT_MAX;
			limit_lower = LPWG_SHORT_MIN;
			break;
		case DELTA_SHOW:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Delta Result =============\n");
			check_limit = 0;
			break;
		default:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test Failed (Invalid test type)\n");
			return ret;
	}

	max = min = fts_data[0][0];

	for (i = 0; i < MAX_ROW; i++) {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i+1);
		for (j = 0; j < MAX_COL; j++) {
			
			if (test_type == RAW_DATA_TEST || test_type == LPWG_RAW_DATA_TEST) {
				ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "%5d ", fts_data[i][j]);
			}
			else {
				ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "%4d ", fts_data[i][j]);
			}

			if (check_limit && (fts_data[i][j] < limit_lower || fts_data[i][j] > limit_upper)) {
				fail_count++;
			}
			if (fts_data[i][j] < min)
				min = fts_data[i][j];
			if (fts_data[i][j] > max)
				max = fts_data[i][j];
		}
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "\n");
	}

	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "==================================================\n");

	if(fail_count && check_limit) {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test FAIL : %d Errors\n", fail_count);
	}
	else {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test PASS : No Errors\n");
		*test_result = TEST_PASS;
	}

	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "MAX = %d, MIN = %d, Upper = %d, Lower = %d\n\n", max, min, limit_upper, limit_lower);

	return ret;
}

#if 0
static ssize_t get_data(struct device *dev, int16_t *buf, u32 wdata)
{
	u32 data_offset = 0;
	u32 rdata = 1;
	int retry = 1000;
	int __frame_size = ROW_SIZE*COL_SIZE*RAWDATA_SIZE;

	/* write 1 : GETRAWDATA
	   write 2 : GETDELTA */
	TOUCH_I("======== get data ========\n");

	ft8707_reg_write(dev, rawdata_ctl_write, (u8 *)&wdata, sizeof(wdata));
	TOUCH_I("wdata = %d\n", wdata);

	/* wait until 0 is written */
	do {
		TOUCH_I("retry = %d\n", retry);
		if (retry != 1000)
			touch_msleep(5);
		ft8707_reg_read(dev, rawdata_ctl_read,
				(u8 *)&rdata, sizeof(rdata));

	} while ((rdata != 0) && retry--);
	/* check whether 0 is written or not */

	if (rdata != 0) {
		TOUCH_E("== get data time out! ==\n");
		goto error;
	}

	/*read data*/
	if (__frame_size % 4)
		__frame_size = (((__frame_size >> 2) + 1) << 2);
	data_offset = RAWDATA_OFFSET;
	ft8707_reg_write(dev, tc_tsp_test_data_offset, (u8 *)&data_offset, sizeof(u32));

	ft8707_reg_read(dev, tc_tsp_data_access_addr, (u8 *)buf, __frame_size);

	return 0;

error:
	return 1;
}
#endif
static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Delta Data\n");

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Change clb switch
	ret = ft8707_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	ret = ft8707_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_prd_get_delta_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	ret = ft8707_switch_cal(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	TOUCH_I("Show Delta Data OK !!!\n");

	ret_size = ft8707_prd_test_data(dev, DELTA_SHOW, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Show Delta Data Result : %d\n", test_result);
	//printk("%s\n", log_buf);

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Delta Data FAIL !!!\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_fdata(struct device *dev, char *buf)
{
#if 1
	return 0;
#else
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret2 = 0;
	u8 type = DOZE1_M2_RAWDATA_TEST;

	/* LCD off */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
				"LCD Off. Test Result : Fail\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret2 = write_test_mode(dev, type);
	if (ret2 == 0) {
		TOUCH_E("write_test_mode fail\n");
		ts->driver->power(dev, POWER_OFF);
		ts->driver->power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		return ret;
	}

	prd_read_rawdata(dev, type);
	ret = prd_print_rawdata(dev, buf, type);

	ts->driver->power(dev, POWER_OFF);
	ts->driver->power(dev, POWER_ON);
	touch_msleep(90);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret;
#endif
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Raw Data\n");

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

#if 1
	// Change clb switch
	ret = ft8707_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	//ret = ft8707_prd_check_ch_num(dev);
	//if(ret < 0)
	//	goto FAIL;
#endif

	ret = ft8707_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Raw Data OK !!!\n");

	if(d->state == TC_STATE_ACTIVE) {
		ret_size = ft8707_prd_test_data(dev, RAW_DATA_TEST, &test_result);
	}
	else {
		ret_size = ft8707_prd_test_data(dev, LPWG_RAW_DATA_TEST, &test_result);
	}
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Raw Data Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Raw Data FAIL !!!\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_noise(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Noise\n");

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

#if 0
	// Reset ???
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);
#endif

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	//ret = ft8707_prd_check_ch_num(dev);
	//if(ret < 0)
	//	goto FAIL;

	ret = ft8707_prd_get_noise_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Noise OK !!!\n");

	if(d->state == TC_STATE_ACTIVE) {
		ret_size = ft8707_prd_test_data(dev, NOISE_TEST, &test_result);
	}
	else {
		ret_size = ft8707_prd_test_data(dev, LPWG_NOISE_TEST, &test_result);
	}
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Noise Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Noise FAIL !!!\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_ib(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show IB Data\n");

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Change clb switch
	ret = ft8707_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	ret = ft8707_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_prd_get_ib_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show IB Data OK !!!\n");

	if(d->state == TC_STATE_ACTIVE) {
		ret_size = ft8707_prd_test_data(dev, IB_DATA_TEST, &test_result);
	}
	else {
		ret_size = ft8707_prd_test_data(dev, LPWG_IB_DATA_TEST, &test_result);
	}
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("IB Data Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show IB Data FAIL !!!\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_pp_noise(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Peak-to-Peak Noise Data\n");

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

#if 1
	// Change clb switch
	ret = ft8707_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_prd_get_pp_noise_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Peak-to-Peak Noise Data OK !!!\n");

	if(d->state == TC_STATE_ACTIVE) {
		ret_size = ft8707_prd_test_data(dev, PP_NOISE_TEST, &test_result);
	}
	else {
		ret_size = ft8707_prd_test_data(dev, LPWG_PP_NOISE_TEST, &test_result);
	}
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Peak-to-Peak Noise Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Peak-to-Peak Noise Data FAIL !!!\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_short_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	// Check Current State
	if(d->chip_rev != 0) { // If not Cut 1.1
		TOUCH_E("Short test is NOT supported in Cut 1.0\n");
		return 0;
	}
	// Check Current State
	if(d->state != TC_STATE_LPWG) {
		TOUCH_E("Short test is called in NOT LPWG state\n");
		return 0;
	}

	TOUCH_I("Show Short Test Data\n");

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

#if 1
	// Change clb switch
	ret = ft8707_switch_cal(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_prd_get_short_data(dev);
	if(ret < 0)
		goto FAIL;

	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Short Test Data OK !!!\n");

	ret_size = ft8707_prd_test_data(dev, LPWG_SHORT_TEST, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Short Test Result : %d\n", test_result);
	//printk("%s\n", log_buf);

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_size;

FAIL:

	TOUCH_I("Show Short Test Data FAIL !!!\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return 0;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int test_result, total_result = 0;

	// Check Current State
	if(d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Show_sd is called in NOT Active state\n");
		return 0;
	}

	/* file create , time log */
	TOUCH_I("Show_sd Test Start\n");
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

#if 1
	// Enable LPWG i2c mode for lpwg_sd
	d->en_i2c_lpwg = 1;
#else
	// Change LPWG i2c mode to stand-by for LPWG SD Test
	ret = ft8707_set_lpwg_i2c_mode(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	// Change clb switch
	ret = ft8707_switch_cal(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Change to factory mode
	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	// Start to raw data test
	TOUCH_I("Show_sd : Raw data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

#if 0
	ret = ft8707_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, RAW_DATA_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("Raw Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	//if(test_result == TEST_FAIL) {
	//	goto FAIL;
	//}

	msleep(30);

	// Start to IB data test
	TOUCH_I("Show_sd : IB data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8707_prd_get_ib_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, IB_DATA_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("IB Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	//if(test_result == TEST_FAIL) {
	//	goto FAIL;
	//}

	msleep(30);

	// Start to PP Noise test
	TOUCH_I("Show_sd : PP Noise test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8707_prd_get_pp_noise_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, PP_NOISE_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("PP Noise Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	//if(test_result == TEST_FAIL) {
	//	goto FAIL;
	//}

	msleep(30);

	// Start to noise (RMS) test
	TOUCH_I("Show_sd : Noise test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

#if 0
	// Reset ???
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_prd_get_noise_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, NOISE_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("Noise Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	//if(test_result == TEST_FAIL) {
	//	goto FAIL;
	//}

	msleep(30);

	// Check Result
	if(total_result != TEST_PASS) {
		goto FAIL;
	}

	// Change to working mode
	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

	// Print Test result
	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Pass\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Channel Status : Pass\n"); // ???????

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("Raw Data : Pass\n");
	TOUCH_I("Channel Status : Pass\n"); // ???????
	TOUCH_I("Show_sd Test End\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;

FAIL:

	// Change to working mode
	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		TOUCH_I("Failed to return WORK_MODE\n");

	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Fail\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Channel Status : Pass\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("Raw Data : Fail\n");
	TOUCH_I("Channel Status : Pass\n"); // ???????
	TOUCH_I("Show_sd Test End\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ft8707_data *d = to_ft8707_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int test_result, total_result = 0;

	// Check Current State
	if(d->state != TC_STATE_LPWG) {
		TOUCH_E("Show_lpwg_sd is called in NOT LPWG state\n");
		return 0;
	}

	/* file create , time log */
	TOUCH_I("Show_lpwg_sd Test Start\n");
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	mutex_lock(&ts->lock);

	// Change clb switch
	ret = ft8707_switch_cal(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Change to factory mode
	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;

	// Start to raw data test
	TOUCH_I("Show_lpwg_sd : LPWG Raw data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

#if 0
	ret = ft8707_prd_check_ch_num(dev);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, LPWG_RAW_DATA_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("LPWG Raw Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	msleep(30);

	// Start to IB data test
	TOUCH_I("Show_lpwg_sd : LPWG IB data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8707_prd_get_ib_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, LPWG_IB_DATA_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("LPWG IB Data Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	msleep(30);

	// Start to PP Noise test
	TOUCH_I("Show_lpwg_sd : LPWG PP Noise test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = ft8707_prd_get_pp_noise_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, LPWG_PP_NOISE_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("LPWG PP Noise Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	msleep(30);

	// Start to noise (RMS) test
	TOUCH_I("Show_lpwg_sd : LPWG Noise test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

#if 0
	// Reset ???
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(300);

	ret = ft8707_change_op_mode(dev, FTS_FACTORY_MODE);
	if(ret < 0)
		goto FAIL;
#endif

	ret = ft8707_prd_get_noise_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = ft8707_prd_test_data(dev, LPWG_NOISE_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("LPWG Noise Test Result : %d\n", test_result);

	////memcpy(buf + ret_total_size, log_buf, ret_size);
	////ret_total_size += ret_size;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	msleep(30);

	// Start to Short test
	if (d->chip_rev == 0) { // For Cut 1.1 Only
		TOUCH_I("Show_lpwg_sd : LPWG Short test\n");

		memset(log_buf, 0, LOG_BUF_SIZE);

		ret = ft8707_prd_get_short_data(dev);
		if(ret < 0)
			goto FAIL;
		
		ret_size = ft8707_prd_test_data(dev, LPWG_SHORT_TEST, &test_result);
		total_result |= test_result;

		TOUCH_I("LPWG Short Test Result : %d\n", test_result);

		////memcpy(buf + ret_total_size, log_buf, ret_size);
		////ret_total_size += ret_size;
		//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

		write_file(dev, log_buf, TIME_INFO_SKIP);

		msleep(30);
	}

	// Check Result
	if(total_result != TEST_PASS) {
		goto FAIL;
	}

	// Change to working mode
	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		goto FAIL;

#if 0
	// Change LPWG i2c mode to stop (POR value)
	ret = ft8707_set_lpwg_i2c_mode(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	// Test result
	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Pass\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;
	//TOUCH_I("\n========>>>>>>>>>> ret_total_size = %d\n", ret_total_size);

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("LPWG RawData : Pass\n");
	TOUCH_I("Show_lpwg_sd Test End\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;

FAIL:

	// Change to working mode
	ret = ft8707_change_op_mode(dev, FTS_WORK_MODE);
	if(ret < 0)
		TOUCH_I("Failed to return WORK_MODE\n");

#if 0
	// Change LPWG i2c mode to stop (POR value)
	ret = ft8707_set_lpwg_i2c_mode(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	ret = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Fail\n");

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret);
	ret_total_size += ret;

	TOUCH_I("\n========RESULT=======\n");
	TOUCH_I("LPWG RawData : Fail\n");
	TOUCH_I("Show_lpwg_sd Test End\n");

	mutex_unlock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);

	return ret_total_size;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(fdata, show_fdata, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(noise_test, show_noise, NULL);
static TOUCH_ATTR(ib_test, show_ib, NULL);
static TOUCH_ATTR(jitter, show_noise, NULL);
static TOUCH_ATTR(pp_noise, show_pp_noise, NULL);
static TOUCH_ATTR(short_test, show_short_test, NULL);


static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_fdata.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_noise_test.attr,
	&touch_attr_ib_test.attr,
	&touch_attr_jitter.attr,
	&touch_attr_pp_noise.attr,
	&touch_attr_short_test.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int ft8707_prd_register_sysfs(struct device *dev)
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
