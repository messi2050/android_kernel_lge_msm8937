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
#include "touch_td4100.h"
#include "touch_td4100_prd.h"

/*
 *  Include to Local Variables
 */
static const int retry_count = 30;
static s16 result_data[ROW_SIZE][COL_SIZE];
static s16 combined_data1[ROW_SIZE][COL_SIZE];
static s16 combined_data2[ROW_SIZE][COL_SIZE];
static u8 buffer[PAGE_SIZE];

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
	TOUCH_TRACE();

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
	TOUCH_TRACE();

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

#define ELEM_SWAP(a,b) { register s16 t=(a);(a)=(b);(b)=t; }
s16 get_median(s16 arr[], int n)
{
	int low, high;
	int median;
	int middle, ll, hh;

	low = 0; high = n-1; median = (low + high) / 2;
	for (;;) {
		if (high <= low) /* One element only */
			return arr[median];

		if (high == low + 1) {  /* Two elements only */
			if (arr[low] > arr[high])
				ELEM_SWAP(arr[low], arr[high]);
            return arr[median];
        }

		/* Find median of low, middle and high items; swap into position low */
		middle = (low + high) / 2;
		if (arr[middle] > arr[high])    ELEM_SWAP(arr[middle], arr[high]);
		if (arr[low] > arr[high])       ELEM_SWAP(arr[low], arr[high]);
		if (arr[middle] > arr[low])     ELEM_SWAP(arr[middle], arr[low]);

		/* Swap low item (now in position middle) into position (low+1) */
		ELEM_SWAP(arr[middle], arr[low+1]);

		/* Nibble from each end towards middle, swapping items when stuck */
		ll = low + 1;
		hh = high;
		for (;;) {
			do ll++; while (arr[low] > arr[ll]);
			do hh--; while (arr[hh]  > arr[low]);

			if (hh < ll)
				break;

			ELEM_SWAP(arr[ll], arr[hh]);
		}

		/* Swap middle item (in position low) back into correct position */
		ELEM_SWAP(arr[low], arr[hh]);

		/* Re-set active partition */
		if (hh < median)
			low = ll;
		if (hh >= median)
			high = hh - 1;
	}
}
#undef ELEM_SWAP

#if 0 // Not Use
static int sdcard_spec_file_read(struct device *dev)
{
	int ret = 0;
	int fd = 0;
	char *path[2] = { "/mnt/sdcard/sf3_limit.txt",
		"/mnt/sdcard/sf3_limit_mfts.txt"
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

//static int synaptics_get_limit(struct device *dev, char *breakpoint,
int synaptics_get_limit(struct device *dev, char *breakpoint,
			 u16 (*buf)[COL_SIZE])
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
#endif

static int switchPage(struct device *dev, u8 page)
{
	int ret = 0;
	int count = 0;
	u8 data = 0;

	do {
		synaptics_write(dev, PAGE_SELECT_REG, &page, sizeof(page));
		msleep(20);
		synaptics_read(dev, PAGE_SELECT_REG, &data, sizeof(page));
		count++;
	} while ((data != page) && (count < retry_count));

	if (count >= retry_count) {
		TOUCH_I("Timeout -- Page switch fail !\n");
		ret = -EAGAIN;
	}

	return ret;
}

static int RspSetCommandType(struct device *dev, u8 setValue, int report_type)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int retry = 0;

	TOUCH_TRACE();

	/*Get report*/
	ret = synaptics_write(dev, ANALOG_COMMAND_REG, &setValue, sizeof(setValue));
	if (ret < 0) {
		TOUCH_E("ANALOG_COMMAND_REG write error\n");
		ret = -EAGAIN;
	}

	/*Waiting for complete*/
	do {
		touch_msleep(5);
		ret = synaptics_read(dev, ANALOG_COMMAND_REG, &setValue, sizeof(setValue));
		if (ret < 0) {
			TOUCH_E("ANALOG_COMMAND_REG read error\n");
			ret = -EAGAIN;
		}
		if ((setValue & 0x01) == 0x00) /* Get Report bit cleared */
			break;
	} while ((retry++) < 200); /* changed delay from 60 to 200 for E2E short test */

	if (retry >= 200) {
		TOUCH_E("Get Report[RT=%d] Time Out!! (%d ms)\n", report_type, (retry+1)*5);
		ret = -EPERM;
	} else {
		TOUCH_I("Get Report[RT=%d] Complete : %d ms\n", report_type, (retry+1)*5);
	}

	return ret;
}

static int RspResetReportAddress(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 data = 0;
	u8 addr = 0;

	TOUCH_TRACE();

	addr = FIFO_INDEX_LSB_REG;
	ret = synaptics_write(dev, addr, &data, 1);
	if (ret < 0)
		return ret;
	addr = FIFO_INDEX_MSB_REG;
	ret = synaptics_write(dev, addr, &data, 1);
	if (ret < 0)
		return ret;

	return ret;
}

static int RspReadImageReport(struct device *dev, int16_t *buf, int buf_size)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret= synaptics_read(dev, REPORT_DATA_REG,
		(u8 *)buf, buf_size);
	if (ret < 0)
		return ret;

	return ret;
}

/* Start - kiwoo.han 2016-9-19 */
#if 0 // TBD (Not Use)
static int synaptics_dynamic_sensing_ctrl(struct device *dev, int value)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 data;

	TOUCH_I("synaptics_dynamic_sensing_ctrl : %d\n", value);

	ret = switchPage(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("Switch page error\n");
		return ret;
	}

	data = value << 1; /* Dynamic Sensing Select */
	ret = synaptics_write(dev, DYNAMIC_SENSING_CTRL_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("dynamic_sensing_ctrl set error\n");
		synaptics_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	/* Force update */
	ret = TD4100_ForceUpdate(dev);
	if (ret < 0) {
		TOUCH_E("Force update failed\n");
		synaptics_set_page(dev, DEFAULT_PAGE);
		return ret;
	}

	ret = switchPage(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("Switch page error\n");
		return ret;
	}

	return ret;
}
#endif
/* End - kiwoo.han 2016-9-19 */

static int TD4100_Get_Report_Data(struct device *dev, int report_type, int16_t* result_buf, int buf_size)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	u8 data = 0x00;

	TOUCH_TRACE();

	ret = switchPage(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("Switch page error\n");
		goto FAIL;
	}

	/* Set F54 Report Type and Command register (to get report) */
	data = report_type;
	ret = synaptics_write(dev, REPORT_TYPE_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("Not set Report Type Register\n");
		goto FAIL;
	}

	ret = RspSetCommandType(dev, 0x1, report_type);
	if (ret < 0) {
		TOUCH_E("Not set CommandType\n");
		goto FAIL;
	}

	/* Set Index cleard */
	ret = RspResetReportAddress(dev);
	if (ret < 0) {
		TOUCH_E("Not set ReportAddress\n");
		goto FAIL;
	}

	memset(result_buf , 0, buf_size);
	/* ReadImageReport Data*/
	ret = RspReadImageReport(dev, result_buf, buf_size);
	if (ret < 0) {
		TOUCH_E("Not set ReadImageReport\n");
		goto FAIL;
	}

	ret = switchPage(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("Switch page error\n");
		goto FAIL;
	}

	return ret;

FAIL:
	synaptics_set_page(dev, DEFAULT_PAGE);
	if (result_buf != NULL)
		kfree(result_buf);

	return ret;
}

static int TD4100_Get_Delta_Data(struct device *dev)
{
	int i, j, k, ret = 0;
	int16_t *delta = NULL;

	TOUCH_TRACE();

	delta = kzalloc(sizeof(int16_t) * (REPORT_DATA_LEN), GFP_KERNEL);

	ret = TD4100_Get_Report_Data(dev, REPORT_TYPE_DELTA, delta, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		return ret;
	}

	/* Combine report data to result data */
	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			k = ((COL_SIZE-1-j)*ROW_SIZE + i);
			result_data[i][j] = ((s16*)(delta))[k];
		}
	}

	if (delta != NULL)
		kfree(delta);

	return ret;
}

static int TD4100_Get_Rawdata(struct device *dev)
{
	int i, j, k, ret = 0;
	int16_t *rawdata = NULL;

	TOUCH_TRACE();

	rawdata = kzalloc(sizeof(int16_t) * (REPORT_DATA_LEN), GFP_KERNEL);

	ret = TD4100_Get_Report_Data(dev, REPORT_TYPE_RAW_DATA, rawdata, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		return ret;
	}

	/* Combine report data to result data */
	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			k = ((COL_SIZE-1-j)*ROW_SIZE + i);
			result_data[i][j] = ((s16*)(rawdata))[k];
		}
	}

	if (rawdata != NULL)
		kfree(rawdata);

	return ret;
}

static int TD4100_Get_P2P_Noise_Data(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int i, j, k, ret = 0;
	int frm;
	int16_t *noisedata = NULL;
	u8 data = 0x00;

	TOUCH_TRACE();

	noisedata = kzalloc(sizeof(int16_t) * (REPORT_DATA_LEN), GFP_KERNEL);

	memset(combined_data1, 0x00, sizeof(combined_data1));	/* min */
	memset(combined_data2, 0x00, sizeof(combined_data2));	/* max */

	ret = switchPage(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("Switch page error\n");
		goto FAIL;
	}
	for (frm = 0; frm < NOISE_TEST_FRM; frm++) {
		/* Set F54 Report Type and Command register (to get report) */
		data = REPORT_TYPE_P2P_NOISE;
		ret = synaptics_write(dev, REPORT_TYPE_REG, &data, 1);
		if (ret < 0) {
			TOUCH_E("Not set Report Type Register\n");
			goto FAIL;
		}

		ret = RspSetCommandType(dev, 0x1, REPORT_TYPE_P2P_NOISE);
		if (ret < 0) {
			TOUCH_E("Not set CommandType\n");
			goto FAIL;
		}

		/* Set Index clear */
		ret = RspResetReportAddress(dev);
		if (ret < 0) {
			TOUCH_E("Not set ReportAddress\n");
			goto FAIL;
		}

		memset(noisedata , 0, REPORT_DATA_LEN);
		/* ReadImageReport Data*/
		ret = RspReadImageReport(dev, noisedata, REPORT_DATA_LEN);
		if (ret < 0) {
			TOUCH_E("Not set RspReadImageReport\n");
			goto FAIL;
		}
		for (i=0; i<ROW_SIZE; i++) {
			for (j=0; j<COL_SIZE; j++) {
				k = ((COL_SIZE-1-j)*ROW_SIZE + i);
				result_data[i][j] = ((s16*)(noisedata))[k];
				if (result_data[i][j] < combined_data1[i][j])	/* min */
					combined_data1[i][j] = result_data[i][j];
				if (result_data[i][j] > combined_data2[i][j])	/* max */
					combined_data2[i][j] = result_data[i][j];
			}
		}
		msleep(20);
	}

	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			result_data[i][j] = combined_data2[i][j] - combined_data1[i][j];
			if (result_data[i][j] < 0)
				result_data[i][j] = 0 - result_data[i][j];
		}
	}

	ret = switchPage(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("Switch page error\n");
		goto FAIL;
	}

	if (noisedata != NULL)
		kfree(noisedata);

	return ret;

FAIL:
	synaptics_set_page(dev, DEFAULT_PAGE);
	if (noisedata != NULL)
		kfree(noisedata);
	return ret;
}

/* LGD considers test result with only image 1 data compared with upper limit */
static int TD4100_Get_E2E_Short_Data(struct device *dev)
{
	int i, j, k, ret = 0, index;
	u8 buf[64] = {0, };
	int16_t *shortdata = NULL;
	s16 median_left, median_right, median = 0;
	int result = 0;
	TOUCH_TRACE();

	shortdata = kzalloc(sizeof(int16_t) * (REPORT_DATA_LEN), GFP_KERNEL);
	memset(combined_data1, 0x00, sizeof(combined_data1));
	memset(combined_data2, 0x00, sizeof(combined_data2));

	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	ret = TD4100_ForceUpdate(dev);
	if (ret < 0) {
		TOUCH_E("Force update failed\n");
		return ret;
	}

	/* reading 2 images at the same time do size of read is (REPORT_DATA_LEN*2) */
	ret = TD4100_Get_Report_Data(dev, REPORT_TYPE_E2E_SHORT, shortdata, (REPORT_DATA_LEN*2));
	if (ret < 0) {
		TOUCH_E("Error to get report data type E2E Short\n");
		return ret;
	}
	index = 0; /* reading image 1 at index 0*/
	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			k = ((COL_SIZE-1-j)*ROW_SIZE + i);
			combined_data1[i][j] = ((s16*)(shortdata))[index + k];
		}
	}
	index = ROW_SIZE*COL_SIZE; /* reading image 2 at index (total read/2) */

	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			k = ((COL_SIZE-1-j)*ROW_SIZE + i);
			combined_data2[i][j] = ((s16*)(shortdata))[index + k];
		}
	}

/* Left Mux Size = 9, Right Mux Size = 9 */
	/* Ratio for Top/Bottom */
	for (i = 0; i < ROW_SIZE; i++) {
		for (j = 0; j < COL_SIZE; j++) {
			((s16*)buf)[j] = combined_data2[i][j];
		}
		median_left = get_median((s16*)buf, MUX_SIZE);
		median_right = get_median(((s16*)buf)+(MUX_SIZE), MUX_SIZE);

		for (j = 0; j < COL_SIZE; j++) {
			if (j < MUX_SIZE)
				median = median_left;
			else
				median = median_right;

			if (median != 0) {
				result = (int)(combined_data2[i][j]) * 100 / median;
			} else {
				result = S16_MAX;
				TOUCH_E("Division by zero detected!!!\n");
			}

			if (result <= S16_MAX) {
				combined_data2[i][j] = result;
			} else {
				combined_data2[i][j] = S16_MAX;
			}
		}
	}
	for (i = 0; i < ROW_SIZE; i++) {
		for (j = 0; j < COL_SIZE; j++) {
			result_data[i][j] = combined_data1[i][j];
		}
	}

	if (shortdata != NULL)
		kfree(shortdata);

	return ret;
}

/* LGD considers test result with only image 1 data */
static int TD4100_Get_AMP_Open_Data(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int i, j, k, ret = 0;
	int result = 0;
	u16 data = 0x00;
	u8 original_intdur[3] = {0, };
	u8 intdur_phase1[3] = {0, };
	u8 intdur_phase2[3] = {0, };
	u8 buf[64] = {0, };
	s16 median_left, median_right, median = 0;
	int16_t *opendata = NULL;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, ANALOG_PAGE);

	ret = synaptics_read(dev, WAVEFORM_DURATION_CTRL_REG, original_intdur, sizeof(original_intdur)); /* F54_ANALOG_CTRL99 (Integration Duration) */
	if (ret < 0) {
		TOUCH_E("WAVEFORM_DURATION_CTRL_REG (original_intdur) read error\n");
		goto FAIL;
	}
	data = (original_intdur[1] << 8 | original_intdur[0]);

	/* Get Integration Duration - Phase 1 */
	ret = synaptics_read(dev, WAVEFORM_DURATION_CTRL_REG, intdur_phase1, sizeof(intdur_phase1)); /* F54_ANALOG_CTRL99 (Integration Duration) */
	if (ret < 0) {
		TOUCH_E("WAVEFORM_DURATION_CTRL_REG (Phase1) read error\n");
		goto FAIL;
	}

	/* Set Integration Duration Only - Phase 1 (0x14 -> 0x5A) : Apply LGD Inspection Tool value */
	intdur_phase1[0] = 0x5A & 0xff;
	intdur_phase1[1] = (0x5A >> 8) & 0xff;
	ret = synaptics_write(dev, WAVEFORM_DURATION_CTRL_REG, intdur_phase1, sizeof(intdur_phase1));  /* F54_ANALOG_CTRL99 (Integration Duration) */
	if (ret < 0) {
		TOUCH_E("Set Integration Duration (Phase1) error\n");
		goto FAIL;
	}
	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	ret = TD4100_ForceUpdate(dev);
	if (ret < 0) {
		TOUCH_E("Force update failed\n");
		return ret;
	}

	/* read the first image with original integration duration and CBC on count config */
	opendata = kzalloc(sizeof(int16_t) * (REPORT_DATA_LEN), GFP_KERNEL);

	ret = TD4100_Get_Report_Data(dev, REPORT_TYPE_RAW_DATA, opendata, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		goto FAIL;
	}

	/* Combine report data to result data */
	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			k = ((COL_SIZE-1-j)*ROW_SIZE + i);
			result_data[i][j] = ((s16*)(opendata))[k];
		}
	}

	/* Get Integration Duration - Phase 2 (0x0A) */
	ret = synaptics_set_page(dev, ANALOG_PAGE);
	ret |= synaptics_read(dev, WAVEFORM_DURATION_CTRL_REG, intdur_phase2, sizeof(intdur_phase2)); /* F54_ANALOG_CTRL99 (Integration Duration) */
	if (ret < 0) {
		TOUCH_E("WAVEFORM_DURATION_CTRL_REG (Phase2) read error\n");
		goto FAIL;
	}
	/* Set Integration Duration Only - Phase 2 */
	intdur_phase2[0] = 0x0A & 0xff; // 0000 1010
	intdur_phase2[1] = (0x0A >> 8) & 0xff; // 0000 0000
	ret = synaptics_write(dev, WAVEFORM_DURATION_CTRL_REG, intdur_phase2, sizeof(intdur_phase2));  /* F54_ANALOG_CTRL99 (Integration Duration) */
	if (ret < 0) {
		TOUCH_E("Set Integration Duration (Phase2) error\n");
		goto FAIL;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	ret = TD4100_ForceUpdate(dev);
	if (ret < 0) {
		TOUCH_E("Force update failed\n");
		return ret;
	}

	memset(opendata, 0, (sizeof(int16_t) * REPORT_DATA_LEN));
	/* read the first image with original integration duration and CBC on count config */
	ret = TD4100_Get_Report_Data(dev, REPORT_TYPE_RAW_DATA, opendata, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		goto FAIL;
	}

	memset(original_intdur, 0, sizeof(original_intdur));
	/* restore original integration duration */
	ret = synaptics_set_page(dev, ANALOG_PAGE);
	ret |= synaptics_read(dev, WAVEFORM_DURATION_CTRL_REG, original_intdur, sizeof(original_intdur)); /* F54_ANALOG_CTRL99 (Integration Duration) */
	if (ret < 0) {
		TOUCH_E("WAVEFORM_DURATION_CTRL_REG (original_intdur) read error\n");
		goto FAIL;
	}

	/* Set Integration Duration Only - Original */
	original_intdur[0] = data & 0xff;
	original_intdur[1] = (data >> 8) & 0xff;
	ret = synaptics_write(dev, WAVEFORM_DURATION_CTRL_REG, original_intdur, sizeof(original_intdur));  /* F54_ANALOG_CTRL99 (Integration Duration) */
	if (ret < 0) {
		TOUCH_E("Set Integration Duration (original_intdur) error\n");
		goto FAIL;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	ret = TD4100_ForceUpdate(dev);
	if (ret < 0) {
		TOUCH_E("Force update failed\n");
		return ret;
	}

	/* Calculate the delta value between A and B */
	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			k = ((COL_SIZE-1-j)*ROW_SIZE + i);
			combined_data1[i][j] = result_data[i][j] - ((s16*)(opendata))[k];
		}
	}

	for (i=0; i<ROW_SIZE; i++) {
		for (j=0; j<COL_SIZE; j++) {
			result_data[i][j] = combined_data1[i][j];
			if (result_data[i][j] < 0)
				result_data[i][j] = 0 - result_data[i][j];
		}
	}

	/* Left Mux Size = 9, Right Mux Size = 9 */
	/* Ratio for Top/Bottom */
	for (i = 0; i < ROW_SIZE; i++) {
		for (j = 0; j < COL_SIZE; j++) {
			((s16*)buf)[j] = result_data[i][j];
		}
		median_left = get_median((s16*)buf, MUX_SIZE);
		median_right = get_median(((s16*)buf)+(MUX_SIZE), MUX_SIZE);

		for (j = 0; j < COL_SIZE; j++) {
			if (j < MUX_SIZE)
				median = median_left;
			else
				median = median_right;

			if (median != 0) {
				result = (int)(result_data[i][j]) * 100 / median;
			} else {
				result = S16_MAX;
				TOUCH_E("Division by zero detected!!!\n");
			}

			if (result <= S16_MAX) {
				result_data[i][j] = result;
			} else {
				result_data[i][j] = S16_MAX;
			}
		}
	}

	if (opendata != NULL)
		kfree(opendata);

	return ret;
FAIL:
	if (opendata != NULL)
		kfree(opendata);

	return ret;
}

static int TD4100_Get_LPWG_Doze_Data(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int i, max, ret, fail_count = 0;
	u8 data = 0x00;

	TOUCH_TRACE();

	for(i =0; i < 50; i++) {
		ret = synaptics_read(dev, DOZE_DATA_REG, &data, sizeof(data));

		if (ret < 0) {
			TOUCH_E("DOZE_DATA_REG (data) read error\n");
			return ret;
		}

		if(max<data) {
			max = data;
		}

		if(data > LPWG_DOZE_DATA_MAX) {
			TOUCH_E("[%2d] LPWG Doze Data : %d\n", i, data);
			fail_count++;
		}
	}

	ret += snprintf(buffer + ret, PAGE_SIZE - ret, "============= LPWG Doze Data Test Result =============\n");
	if(fail_count>0) {
		TOUCH_E("Test FAIL : %d Errors\n", fail_count);
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "Test FAIL : %d Errors\n", fail_count);
	} else {
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "Test PASS : No Errors\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "MAX = %d, Upper = %d\n", max, LPWG_DOZE_DATA_MAX);
	}
	return ret;
}

static int TD4100_Test_Result_Data(struct device *dev, int test_type, int* test_result)
{
	int i = 0;
	int j = 0;
	int ret = 0;
	int result = 0;
	int limit_upper = 0, limit_lower = 0;
	int min, max = 0; /*aver, stdev*/
	int fail_count = 0;
	int check_limit_upper = 1, check_limit_lower = 1;
	//int log_ret = 0;
	//char log_buf[LOG_BUF_SIZE] = {0,};

	TOUCH_TRACE();

	*test_result = TEST_FAIL;

	//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "======TD4100 Inspection Test Result (Type : %d)======\n", test_type);

	switch (test_type) {
	case RAW_DATA_TEST:
		result = TD4100_Get_Rawdata(dev);
		if(result < 0) {
			TOUCH_E("TD4100_Get_Rawdata error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get Raw Data Complete !!!=====\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "============= Raw Data Test Result =============\n");
		limit_upper = RAW_DATA_MAX + RAW_DATA_MARGIN;
		limit_lower = RAW_DATA_MIN - RAW_DATA_MARGIN;
		break;

	case P2P_NOISE_TEST:
		result = TD4100_Get_P2P_Noise_Data(dev);
		if(result < 0) {
			TOUCH_E("TD4100_Get_P2P_Noise_Data error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get Peak-to-Peak Noise Data Complete !!!=====\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "============= Peak-to-Peak Noise Test Result =============\n");
		limit_upper = P2P_NOISE_MAX;
		//limit_lower = P2P_NOISE_MIN;
		check_limit_lower = 0;
		break;
	case E2E_SHORT_TEST:
		result = TD4100_Get_E2E_Short_Data(dev);
		if(result < 0) {
			TOUCH_E("TD4100_Get_E2E_Short_Data error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get Electrode to Electrode Short Data Complete !!!=====\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "============= Electrode to Electrode Short Test Result =============\n");
		limit_upper = E2E_SHORT_MAX;
		limit_lower = E2E_SHORT_MIN;
		check_limit_lower=0;
		break;
	case AMP_OPEN_TEST:
		result = TD4100_Get_AMP_Open_Data(dev);
		if(result < 0) {
			TOUCH_E("TD4100_Get_AMP_Open_Data error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get AMP Open Data Complete !!!=====\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "============= AMP Open Test Result =============\n");
		limit_upper = AMP_OPEN_MAX;
		limit_lower = AMP_OPEN_MIN;
		break;
	case LPWG_RAW_DATA_TEST:
		result = TD4100_Get_Rawdata(dev);
		if(result < 0) {
			TOUCH_E("TD4100_Get_Rawdata error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get Raw Data Complete !!!=====\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "\n============= LPWG Raw Data Test Result =============\n");
		limit_upper = LPWG_RAW_DATA_MAX + RAW_DATA_MARGIN;
		limit_lower = LPWG_RAW_DATA_MIN - RAW_DATA_MARGIN;
		break;
	case LPWG_P2P_NOISE_TEST:
		result = TD4100_Get_P2P_Noise_Data(dev);
		if(result < 0) {
			TOUCH_E("TD4100_Get_P2P_Noise_Data error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get Peak-to-Peak Noise Data Complete !!!=====\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "============= LPWG P2P Noise Test Result =============\n");
		limit_upper = LPWG_P2P_NOISE_MAX;
		//limit_lower = LPWG_P2P_NOISE_MIN;
		check_limit_lower = 0;
		break;
	case LPWG_DOZE_DATA_TEST:
		result = TD4100_Get_LPWG_Doze_Data(dev);
		if(result < 0) {
			TOUCH_E("TD4100_Get_LPWG_Doze_Data error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get LPWG Doze Data Complete !!!=====\n");
		return result;
		break;
	case DELTA_SHOW:
		ret = TD4100_Get_Delta_Data(dev);
		if(ret < 0) {
			TOUCH_E("TD4100_Get_Delta_Data error\n");
			return TEST_FAIL;
		}
		TOUCH_I("===== Get Delta Data Complete !!!=====\n");
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "============= Delta Result =============\n");
		check_limit_upper = check_limit_lower = 0;
		break;
	default:
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "Test Failed (Invalid test type)\n");
		return ret;
	}

	max = min = result_data[0][0];

	/* Reverse for Up & Down */
	//for (i = 0; i < ROW_SIZE; i++) {
	for (i = ROW_SIZE-1; i >= 0; i--) {
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "[%2d] ", (ROW_SIZE-i));
		//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "[%2d] ", i+1);
		for (j = 0; j < COL_SIZE; j++) {

			if (check_limit_upper && result_data[i][j] > limit_upper) {
				ret += snprintf(buffer + ret, PAGE_SIZE - ret, "!%4d ", result_data[i][j]);
				//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "!%4d ", result_data[i][j]);
				fail_count++;
			} else if (check_limit_lower && result_data[i][j] < limit_lower) {
				ret += snprintf(buffer + ret, PAGE_SIZE - ret, "!%4d ", result_data[i][j]);
				//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "!%4d ", result_data[i][j]);
				fail_count++;
			} else {
				ret += snprintf(buffer + ret, PAGE_SIZE - ret, "%5d ", result_data[i][j]);
				//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "%5d ", result_data[i][j]);
			}

			if (result_data[i][j] < min)
				min = result_data[i][j];
			if (result_data[i][j] > max)
				max = result_data[i][j];
		}
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "\n");
		//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "\n");
	}

	ret += snprintf(buffer + ret, PAGE_SIZE - ret, "==================================================\n");

	if(fail_count && (check_limit_upper || check_limit_lower)) {
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "Test FAIL : %d Errors\n", fail_count);
		//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "Test FAIL : %d Errors\n", fail_count);
	} else {
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, "Test PASS : No Errors\n");
		//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "Test PASS : No Errors\n");
		*test_result = TEST_PASS;
	}

	ret += snprintf(buffer + ret, PAGE_SIZE - ret, "MAX = %d, MIN = %d", max, min);
	//log_ret += snprintf(log_buf + log_ret, LOG_BUF_SIZE - log_ret, "MAX = %d, MIN = %d, Upper = %d, Lower = %d\n\n", max, min, limit_upper, limit_lower);
	if(check_limit_upper)
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, ", Upper = %d", limit_upper);
	if(check_limit_lower)
		ret += snprintf(buffer + ret, PAGE_SIZE - ret, ", Lower = %d", limit_lower);
	//TOUCH_I("%s\n", log_buf);
	ret += snprintf(buffer + ret, PAGE_SIZE - ret, "\n\n");

	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret_size = 0;
	int test_result = 0;

	TOUCH_TRACE();

	TOUCH_I("===== Show Delta Data =====\n");
	mutex_lock(&ts->lock);

	ret_size = TD4100_Test_Result_Data(dev, DELTA_SHOW, &test_result);

	memcpy(buf, buffer, ret_size);
	TOUCH_I("Show Delta Data Result is : %d\n", test_result);

	mutex_unlock(&ts->lock);
	return ret_size; /* "ret_size" is size of data buffer */
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret_size = 0;
	int test_result = 0;

	TOUCH_TRACE();

	TOUCH_I("===== Show Raw Data =====\n");
	mutex_lock(&ts->lock);

	if (ts->lpwg.screen) {
		TOUCH_I("Inspection for Raw Data (Idle)\n");
		ret_size = TD4100_Test_Result_Data(dev, RAW_DATA_TEST, &test_result);
	} else {
		TOUCH_I("Inspection for Raw Data (LPWG)\n");
		ret_size = TD4100_Test_Result_Data(dev, LPWG_RAW_DATA_TEST, &test_result);
	}

	memcpy(buf, buffer, ret_size);
	TOUCH_I("Show Raw Data Result is : %d\n", test_result);

	mutex_unlock(&ts->lock);
	return ret_size; /* "ret_size" is size of data buffer */
}

static ssize_t show_p2p_noise(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret_size = 0;
	int test_result = 0;

	TOUCH_TRACE();

	TOUCH_I("===== Show Peak-to-Peak Noise Data=====\n");
	mutex_lock(&ts->lock);

#if 0  // TBD
	/* Set Dynamic Sensing Control */
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0) {
		TOUCH_E("synaptics_dynamic_sensing_ctrl error\n");
		goto FAIL;
	}
#endif

	if (ts->lpwg.screen) {
		TOUCH_I("Inspection for P2P Noise Data (Idle)\n");
		ret_size = TD4100_Test_Result_Data(dev, P2P_NOISE_TEST, &test_result);
	} else {
		TOUCH_I("Inspection for P2P Noise Data (LPWG)\n");
		ret_size = TD4100_Test_Result_Data(dev, LPWG_P2P_NOISE_TEST, &test_result);
	}

	memcpy(buf, buffer, ret_size);
	TOUCH_I("Show P2P Noise Data Result is : %d\n", test_result);

#if 0  // TBD
	/* Set Dynamic Sensing Control */
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0) {
		TOUCH_E("synaptics_dynamic_sensing_ctrl error\n");
		goto FAIL;
	}
#endif

	mutex_unlock(&ts->lock);
	return ret_size; /* "ret_size" is size of data buffer */
}

static ssize_t show_e2e_short(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret_size = 0;
	int test_result = 0;

	TOUCH_TRACE();

	TOUCH_I("===== Show Electrode to Electrode Short Data=====\n");
	mutex_lock(&ts->lock);

	ret_size = TD4100_Test_Result_Data(dev, E2E_SHORT_TEST, &test_result);
	memcpy(buf, buffer, ret_size);
	TOUCH_I("Show Electrode to Electrode Short Data Result is : %d\n", test_result);

	mutex_unlock(&ts->lock);
	return ret_size; /* "ret_size" is size of data buffer */
}

static ssize_t show_amp_open(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret_size = 0;
	int test_result = 0;

	TOUCH_TRACE();

	TOUCH_I("===== Show AMP Open Data=====\n");
	mutex_lock(&ts->lock);

	ret_size = TD4100_Test_Result_Data(dev, AMP_OPEN_TEST, &test_result);
	memcpy(buf, buffer, ret_size);
	TOUCH_I("Show AMP Open Data Result is : %d\n", test_result);

	mutex_unlock(&ts->lock);
	return ret_size; /* "ret_size" is size of data buffer */
}

static char *synaptics_panel_parse(struct device *dev, unsigned char *product)
{
	int panel_id;
	char *str_panel[]
		= { "0", "0", "Tovis", "0", "0", "LGD", };

	TOUCH_TRACE();

	panel_id = (product[0] & 0xF0) >> 4;

	return str_panel[panel_id];
}

static int synaptics_sd(struct device *dev, char *buf, int *raw, int *ch)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	char *panel;
	int raw_result = 0;
	int jitter_result = 0;
	int open_result = 0;
	int short_result = 0;

	TOUCH_TRACE();

	panel = synaptics_panel_parse(dev, d->ic_info.raws);

	ret += snprintf(buffer + ret, PAGE_SIZE, "IC F/W Version : V%d.%02d\n", d->ic_info.version.major, d->ic_info.version.minor);
	ret += snprintf(buffer + ret, PAGE_SIZE, "Product ID = %s\n", d->ic_info.product_id);
	write_file(dev, buffer, TIME_INFO_SKIP);

	if(!strcmp(panel, "Tovis")) {
		TOUCH_I("Tovis Production Test\n");
		ret += TD4100_Test_Result_Data(dev, RAW_DATA_TEST, &raw_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);

		ret += TD4100_Test_Result_Data(dev, P2P_NOISE_TEST, &jitter_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);
	} else {
		TOUCH_I("LGD Production Test\n");
		ret += TD4100_Test_Result_Data(dev, RAW_DATA_TEST, &raw_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);

		ret += TD4100_Test_Result_Data(dev, P2P_NOISE_TEST, &jitter_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);

		ret += TD4100_Test_Result_Data(dev, E2E_SHORT_TEST, &short_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);

		ret += TD4100_Test_Result_Data(dev, AMP_OPEN_TEST, &open_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);
	}

	TOUCH_I("raw_result : %d / jitter_result : %d / short_result : %d / open_result : %d\n",
		raw_result,
		jitter_result,
		short_result,
		open_result);

	if(!strcmp(panel, "Tovis")) {
		*raw = raw_result;
		*ch = jitter_result;
	} else {
		*raw = raw_result || jitter_result;
		*ch = open_result || short_result;
	}

	return ret;
}

static int synaptics_lpwg_sd(struct device *dev, char *buf, int *raw)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	char *panel;
	int raw_result = 0;
	int jitter_result = 0;

	TOUCH_TRACE();

	panel = synaptics_panel_parse(dev, d->ic_info.raws);

	if(!strcmp(panel, "Tovis")) {
		TOUCH_I("Tovis LPWG Production Test\n");
		if (synaptics_is_product(d, "PLG640", 6)) {
			touch_msleep(2000);
			ret += TD4100_Test_Result_Data(dev, LPWG_DOZE_DATA_TEST, &jitter_result);
			touch_msleep(30);
			write_file(dev, buffer, TIME_INFO_SKIP);
		}

		ret += TD4100_Test_Result_Data(dev, LPWG_RAW_DATA_TEST, &raw_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);

		ret += TD4100_Test_Result_Data(dev, LPWG_P2P_NOISE_TEST, &jitter_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);
	} else {
		TOUCH_I("LGD LPWG Production Test\n");
		ret += TD4100_Test_Result_Data(dev, LPWG_RAW_DATA_TEST, &raw_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);

		ret += TD4100_Test_Result_Data(dev, LPWG_P2P_NOISE_TEST, &jitter_result);
		touch_msleep(30);
		write_file(dev, buffer, TIME_INFO_SKIP);
	}

	*raw = raw_result || jitter_result;

	return ret;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret_size = 0, ret_total_size = 0;
	int rawstatus = 0, chstatus = 0;

	TOUCH_TRACE();

	mutex_lock(&ts->lock);
	//touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (ts->lpwg.screen == 0) {
		TOUCH_E("LCD OFF state. please turn on the display\n");
		ret_size = snprintf(buffer, PAGE_SIZE, "LCD Off state. please turn on the display\n");
		//touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		mutex_unlock(&ts->lock);
		memcpy(buf, buffer, ret_size);
		return ret_size;
	}

	TOUCH_I("========== Show [Self-Diagnostic Test] Start ==========\n");
	/* file create , time log */
	write_file(dev, "\n[Self-Diagnostic Test] Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	memset(buffer, 0, PAGE_SIZE);

	/* Get data and Result Right */
	ret_size = synaptics_sd(dev, buf, &rawstatus, &chstatus);

	/* Check Result and Print out */
	ret_size = snprintf(buffer, PAGE_SIZE, "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	/* Rawdata Check */
	if(rawstatus != TEST_PASS) {	/* FAIL */
		ret_size += snprintf(buffer + ret_size, PAGE_SIZE - ret_size, "Raw Data : Fail\n");
		TOUCH_I("Raw Data : Fail\n");
	} else {	/* PASS */
		ret_size += snprintf(buffer + ret_size, PAGE_SIZE - ret_size, "Raw Data : Pass\n");
		TOUCH_I("Raw Data : Pass\n");
	}

	/* Channel Status Check */
	if(chstatus != TEST_PASS) {	/* FAIL */
		ret_size += snprintf(buffer + ret_size, PAGE_SIZE - ret_size, "Channel Status : Fail\n");
		TOUCH_I("Channel Status : Fail\n");
	} else {	/* PASS */
		ret_size += snprintf(buffer + ret_size, PAGE_SIZE - ret_size, "Channel Status : Pass\n");
		TOUCH_I("Channel Status : Pass\n");
	}

	write_file(dev, buffer, TIME_INFO_SKIP);
	write_file(dev, "[Self-Diagnostic Test] End\n", TIME_INFO_WRITE);
	TOUCH_I("========== Show [Self-Diagnostic Test] End ==========\n");

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, buffer, ret_size);
	ret_total_size += ret_size;

	//touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret_total_size;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret_size = 0, ret_total_size = 0;
	int lpwg_rawstatus = 0;

	TOUCH_TRACE();

	mutex_lock(&ts->lock);
	//touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (!(ts->role.mfts_lpwg) && ts->lpwg.screen) {
		TOUCH_E("LCD ON state. please turn off the display\n");
		ret_size = snprintf(buffer, PAGE_SIZE, "LCD ON state. please turn off the display\n");
		//touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		mutex_unlock(&ts->lock);
		return ret_size;
	}

	TOUCH_I("========== Show [LPWG Self-Diagnostic Test] Start ==========\n");
	/* file create , time log */
	write_file(dev, "\n[LPWG Self-Diagnostic Test] Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	memset(buffer, 0, PAGE_SIZE);

	/* Get data and Result Right */
	ret_size = synaptics_lpwg_sd(dev, buf, &lpwg_rawstatus);

	/* Check Result and Print out */
	ret_size = snprintf(buffer, PAGE_SIZE, "\n========RESULT=======\n");
	TOUCH_I("========RESULT=======\n");

	/* Rawdata Check */
	if(lpwg_rawstatus != TEST_PASS) {	/* FAIL */
		ret_size += snprintf(buffer + ret_size, PAGE_SIZE - ret_size, "LPWG RawData : Fail\n");
		TOUCH_I("LPWG RawData : Fail\n");
	} else {	/* PASS */
		ret_size += snprintf(buffer + ret_size, PAGE_SIZE - ret_size, "LPWG RawData : Pass\n");
		TOUCH_I("LPWG RawData : Pass\n");
	}

	write_file(dev, buffer, TIME_INFO_SKIP);
	write_file(dev, "[LPWG Self-Diagnostic Test] End\n", TIME_INFO_WRITE);
	TOUCH_I("========== Show [LPWG Self-Diagnostic Test] End ==========\n");

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, buffer, ret_size);
	ret_total_size += ret_size;

	//touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	return ret_total_size;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(p2p_noise, show_p2p_noise, NULL);
static TOUCH_ATTR(e2e_short, show_e2e_short, NULL);
static TOUCH_ATTR(amp_open, show_amp_open, NULL);


static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_p2p_noise.attr,
	&touch_attr_e2e_short.attr,
	&touch_attr_amp_open.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int td4100_prd_register_sysfs(struct device *dev)
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
