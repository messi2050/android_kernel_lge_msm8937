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
#include "touch_td4300.h"
#include "touch_td4300_prd.h"

enum {
	RAW_DATA_TEST = 0,
	P2P_NOISE_TEST,
	E2E_SHORT_TEST,
	AMP_OPEN_TEST,
	SENSOR_SPEED_TEST,
	LPWG_RAW_DATA_TEST,
	LPWG_P2P_NOISE_TEST,
	DELTA_SHOW,
};

enum {
	TEST_PASS = 0,
	TEST_FAIL,
};

#define REPORT_TYPE_RAW_DATA			92 // raw capacitance (pF)
#define REPORT_TYPE_P2P_NOISE			94 // raw capacitance delta
#define REPORT_TYPE_TD43XX_E2E_SHORT	95 // raw capacitance delta
#define REPORT_TYPE_SENSOR_SPEED		96
#define REPORT_TYPE_DELTA				02 // 16-bit normalized image

// Spec. (V0.16/K7J/Set/4.18) -> P2P Noise 35
#define RAW_DATA_MAX 			2200
#define RAW_DATA_MIN 			480
#define RAW_DATA_MARGIN 		0
#define P2P_NOISE_MAX 			35 // 45
//#define P2P_NOISE_MIN 		0
#define NOISE_TEST_FRM 			50
#define E2E_SHORT_MAX 			150	// Upper limit for Image1
#define E2E_SHORT_MIN 			70	// Lower Limit for Image2
#define E2E_SHORT_RESULT		0
#define AMP_OPEN_MAX			120
#define AMP_OPEN_MIN			80
#define SENSOR_SPEED_MAX		120
#define SENSOR_SPEED_MIN		80
#define LPWG_RAW_DATA_MAX 		2200
#define LPWG_RAW_DATA_MIN 		480
#define LPWG_P2P_NOISE_MAX 		35 // 45
//#define LPWG_P2P_NOISE_MIN 0

#define LOG_BUF_SIZE			4096 // 4*1024
static u8 log_buf[LOG_BUF_SIZE]; /* !!!!!!!!!Should not exceed the log size !!!!!!!! */

#define LOG_ADD_BUF_SIZE		16384 // 16*1024
static u8 log_add_buf[LOG_ADD_BUF_SIZE]; /* !!!!!!!!!Should not exceed the log size !!!!!!!! */
static int log_add_used = 0;

/* Number of channel */
//
// TD4300 (FHD)
//
// [TX17][RX00] .... [TX00][RX00]
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// ................................
// [TX17][RX29] .... [TX00][RX29]

#define TX_NUM				18
#define RX_NUM				30
#define MUX_SIZE			9
#define RX_NUM_HALF			15
#define REPORT_DATA_LEN		(RX_NUM*TX_NUM*2)
static u8 synaptics_report_data[REPORT_DATA_LEN*2];
static s16 result_data[RX_NUM][TX_NUM];
static s16 combined_data1[RX_NUM][TX_NUM];
static s16 combined_data2[RX_NUM][TX_NUM];

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
		if (hh <= median)
			low = ll;
		if (hh >= median)
			high = hh - 1;
	}
}

#undef ELEM_SWAP

int td4300_prd_get_report_data(struct device *dev, int report_type, u8* result_buf, int buf_size)
{
//	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int retry = 0;
	int ret = 0;
	u8 data = 0x00;

	memset(result_buf, 0, buf_size);

	ret = synaptics_set_page(dev, ANALOG_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	// Set Report Type and Command to get report
	data = report_type;
	ret = synaptics_write(dev, REPORT_TYPE_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	data = 0x01; // Get Report
	ret = synaptics_write(dev, ANALOG_COMMAND_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	// Waiting for complete
	do {
		touch_msleep(5);
		ret = synaptics_read(dev, ANALOG_COMMAND_REG, &data, 1);
		if (ret < 0) {
			TOUCH_E("i2c error\n");
			goto FAIL;
		}
		if ((data & 0x01) == 0x00) // Get Report bit cleared
			break;
	} while ((retry++) < 60);

	if (retry >= 60) {
		TOUCH_E("Get Report[RT=%d] Time Out!!\n", report_type);
		ret = -EPERM;
		goto FAIL;
	}
	else {
		TOUCH_I("Get Report[RT=%d] Complete : %d ms\n", report_type, (retry+1)*5);
	}

	// Set index cleared
	data = 0x00;
	ret = synaptics_write(dev, FIFO_INDEX_LSB_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	data = 0x00;
	ret = synaptics_write(dev, FIFO_INDEX_MSB_REG, &data, 1);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	// Read Data
	ret = synaptics_read(dev, REPORT_DATA_REG, result_buf, buf_size);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
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

int td4300_prd_get_raw_data(struct device *dev)
{
	int i, j, k, ret = 0;

	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_RAW_DATA, synaptics_report_data, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		return ret;
	}

	// Combine report data to result data
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			result_data[i][j] = ((s16*)(synaptics_report_data))[k];
		}
	}

	return ret;
}

int td4300_prd_get_delta_data(struct device *dev)
{
	int i, j, k, ret = 0;

	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_DELTA, synaptics_report_data, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		return ret;
	}

	// Combine report data to result data
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			result_data[i][j] = ((s16*)(synaptics_report_data))[k];
		}
	}

	return ret;
}

int td4300_prd_get_p2p_noise_data(struct device *dev)
{
	int i, j, k, ret = 0;
	int frm;

	memset(combined_data1, 0x00, sizeof(combined_data1)); // min
	memset(combined_data2, 0x00, sizeof(combined_data2)); // max

	for (frm = 0; frm < NOISE_TEST_FRM; frm++) {
		ret = td4300_prd_get_report_data(dev, REPORT_TYPE_P2P_NOISE, synaptics_report_data, REPORT_DATA_LEN);
		if (ret < 0) {
			TOUCH_E("Error to get report data\n");
			return ret;
		}

		for (i=0; i<RX_NUM; i++) {
			for (j=0; j<TX_NUM; j++) {
				k = ((TX_NUM-1-j)*RX_NUM + i);
				result_data[i][j] = ((s16*)(synaptics_report_data))[k];
				if (result_data[i][j] < combined_data1[i][j]) // min
					combined_data1[i][j] = result_data[i][j];
				if (result_data[i][j] > combined_data2[i][j]) // max
					combined_data2[i][j] = result_data[i][j];
			}
		}
	}

	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			result_data[i][j] = combined_data2[i][j] - combined_data1[i][j];
			if (result_data[i][j] < 0)
				result_data[i][j] = 0 - result_data[i][j];
		}
	}

	return ret;
}

#define USE_TEST_SHORT_IMAGE2	0

#if !USE_TEST_SHORT_IMAGE2

int td4300_prd_get_short_test_data(struct device *dev)
{
	int i, j, k, ret = 0;

	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_TD43XX_E2E_SHORT, synaptics_report_data, REPORT_DATA_LEN*2);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		return ret;
	}

	// Combine report data to result data for ONLY Image1
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			result_data[i][j] = ((s16*)(synaptics_report_data))[k];
		}
	}

	return ret;
}

#else
int td4300_prd_get_short_test_data(struct device *dev)
{
	int i, j, k, ret = 0;
	s16 median_left, median_right, median;
	int result1, result2;
	u8 buf[64];

	log_add_buf[0] = 0;
	log_add_used = 0;

	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_TD43XX_E2E_SHORT, synaptics_report_data, REPORT_DATA_LEN*2);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		return ret;
	}

	// Combine report data to result data
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			combined_data1[i][j] = ((s16*)(synaptics_report_data))[k];
		}
	}
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i) + (RX_NUM*TX_NUM);
			combined_data2[i][j] = ((s16*)(synaptics_report_data))[k];
		}
	}

#if 1
	// Log out Image 1 & 2 into additional buffer for debugging
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "============= E-to-E Short Test (Image1) =============\n");
	for (i = 0; i < RX_NUM; i++) {
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "[%2d] ", i+1);
		for (j = 0; j < TX_NUM; j++) {
			log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "%4d ", combined_data1[i][j]);
		}
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
	}
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n============= E-to-E Short Test (Image2) =============\n");
	for (i = 0; i < RX_NUM; i++) {
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "[%2d] ", i+1);
		for (j = 0; j < TX_NUM; j++) {
			log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "%4d ", combined_data2[i][j]);
		}
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
	}
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
#endif

	// Configs for this model in F55_SENSOR_CTRL43
	// Swap Sensor Side = 0
	// Left Mux Size = 9, Right Mux Size = 9

#if 0
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n============= E-to-E Short Test (Image2-1) =============\n");
#endif
	for (i = 0; i < RX_NUM; i++) {
		for (j = 0; j < TX_NUM; j++) {
			((s16*)buf)[j] = combined_data2[i][j];
		}
		median_left = get_median((s16*)buf, MUX_SIZE);
		median_right = get_median(((s16*)buf)+(MUX_SIZE), MUX_SIZE);
#if 0
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "[%2d] ", i+1);
#endif
		for (j = 0; j < TX_NUM; j++) {
			if (j < MUX_SIZE)
				median = median_left;
			else
				median = median_right;

			if (median != 0) {
				result2 = (int)(combined_data2[i][j]) * 100 / median;
			}
			else {
				result2 = S16_MAX;
				TOUCH_E("Division by zero detected!!!\n");
			}
#if 0
			log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "%4d ", result2);
#endif
			if (combined_data1[i][j] > E2E_SHORT_MAX)
				result1 = 1; // Fail
			else
				result1 = 0;

			if (result2 < E2E_SHORT_MIN || median == 0)
				result_data[i][j] = result1 + 2; // Fail
			else
				result_data[i][j] = result1;
		}
#if 0
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
#endif
	}

#if 0
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
#endif

	return ret;
}
#endif

int td4300_prd_get_open_test_data(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	int i, j, k, ret = 0;
	s16 median_left, median_right, median;
	int result;
	u8 ref_transcap[5];
	u8 cbc_transcap[64];
	u8 buf[64];

	log_add_buf[0] = 0;
	log_add_used = 0;

	// ButtonCount = 0

	// Get Transcap for backup
	ret = synaptics_set_page(dev, ANALOG_PAGE);
	ret |= synaptics_read(dev, ANALOG_CONTOL_2_REG, ref_transcap, sizeof(ref_transcap)); // F54_ANALOG_CTRL91(Analog Control 2)
	ret |= synaptics_read(dev, CBC_TANSCAP_REG, cbc_transcap, sizeof(cbc_transcap)); // F54_ANALOG_CTRL96(CBC Transcap)
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}

	// Change RefLo/RefHi to 6.5pF
	memcpy(buf, ref_transcap, sizeof(ref_transcap));
	buf[0] = 12; // RefLo = 6.5pF
	buf[1] = 12; // RefHi = 6.5pF
	ret = synaptics_write(dev, ANALOG_CONTOL_2_REG, buf, sizeof(ref_transcap)); // F54_ANALOG_CTRL91(Analog Control 2)
	ret |= synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	ret = synaptics_force_update(dev);
	if (ret < 0) {
		TOUCH_E("Error in force update\n");
		goto FAIL;
	}

	// Get RT92 Image A
	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_RAW_DATA, synaptics_report_data, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		goto FAIL;
	}
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			result_data[i][j] = ((s16*)(synaptics_report_data))[k];
		}
	}

	// Increase Local CBC to 2pF for Rx0-15 and 48-63
	memcpy(buf, cbc_transcap, sizeof(cbc_transcap));
	memset(buf, 0x07, 16);
	memset(buf+48, 0x07, 16);
	ret = synaptics_set_page(dev, ANALOG_PAGE);
	ret |= synaptics_write(dev, CBC_TANSCAP_REG, buf, sizeof(cbc_transcap)); // F54_ANALOG_CTRL96(CBC Transcap)
	ret |= synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	ret = synaptics_force_update(dev);
	if (ret < 0) {
		TOUCH_E("Error in force update\n");
		goto FAIL;
	}

	// Get RT92 Image B
	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_RAW_DATA, synaptics_report_data, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		goto FAIL;
	}

	// Get Delta Image A = (Image B - Image A)
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			combined_data1[i][j] = ((s16*)(synaptics_report_data))[k] - result_data[i][j];
			if (combined_data1[i][j] < 0)
				combined_data1[i][j] = 0 - combined_data1[i][j];
		}
	}

	// Increase Local CBC to 2pF for Rx16-31 and 32-47
	memcpy(buf, cbc_transcap, sizeof(cbc_transcap));
	memset(buf+16, 0x07, 32);
	ret = synaptics_set_page(dev, ANALOG_PAGE);
	ret |= synaptics_write(dev, CBC_TANSCAP_REG, buf, sizeof(cbc_transcap)); // F54_ANALOG_CTRL96(CBC Transcap)
	ret |= synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	ret = synaptics_force_update(dev);
	if (ret < 0) {
		TOUCH_E("Error in force update\n");
		goto FAIL;
	}

	// Get RT92 Image C
	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_RAW_DATA, synaptics_report_data, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		goto FAIL;
	}

	// Get Delta Image B = (Image C - Image A)
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			combined_data2[i][j] = ((s16*)(synaptics_report_data))[k] - result_data[i][j];
			if (combined_data2[i][j] < 0)
				combined_data2[i][j] = 0 - combined_data2[i][j];
		}
	}

	// Restore Transcap
	ret = synaptics_set_page(dev, ANALOG_PAGE);
	ret |= synaptics_write(dev, ANALOG_CONTOL_2_REG, ref_transcap, sizeof(ref_transcap)); // F54_ANALOG_CTRL91(Analog Control 2)
	ret |= synaptics_write(dev, CBC_TANSCAP_REG, cbc_transcap, sizeof(cbc_transcap)); // F54_ANALOG_CTRL96(CBC Transcap)
	ret |= synaptics_set_page(dev, DEFAULT_PAGE);
	if (ret < 0) {
		TOUCH_E("i2c error\n");
		goto FAIL;
	}
	ret = synaptics_force_update(dev);
	if (ret < 0) {
		TOUCH_E("Error in force update\n");
		goto FAIL;
	}

#if 0
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "============= Amp Open Test (Delta A) =============\n");
	for (i = 0; i < RX_NUM; i++) {
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "[%2d] ", i+1);
		for (j = 0; j < TX_NUM; j++) {
			log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "%4d ", combined_data1[i][j]);
		}
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
	}
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n============= Amp Open Test (Delta B) =============\n");
	for (i = 0; i < RX_NUM; i++) {
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "[%2d] ", i+1);
		for (j = 0; j < TX_NUM; j++) {
			log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "%4d ", combined_data2[i][j]);
		}
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
	}
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
#endif

	// Configs for this model in F55_SENSOR_CTRL43
	// Swap Sensor Side = 0
	// Left Mux Size = 9, Right Mux Size = 9

	// Assemble Image with Delta A and Delta B
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			if (j < MUX_SIZE) {
				if (i < 16)
					result_data[i][j] = combined_data1[i][j];
				else
					result_data[i][j] = combined_data2[i][j];
			}
			else {
				if (i < 16)
					result_data[i][j] = combined_data2[i][j];
				else
					result_data[i][j] = combined_data1[i][j];
			}				
		}
	}

#if 1
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "============= Amp Open Test (Assembled) =============\n");
	for (i = 0; i < RX_NUM; i++) {
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "[%2d] ", i+1);
		for (j = 0; j < TX_NUM; j++) {
			log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "%4d ", result_data[i][j]);
		}
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
	}
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
#endif

	// Adjust Edges
	for (i = 0; i < RX_NUM; i++) {
		((s16*)buf)[i] = result_data[i][0] - result_data[i][1];
	}
	median_left = get_median((s16*)buf, RX_NUM);
	for (i = 0; i < RX_NUM; i++) {
		((s16*)buf)[i] = result_data[i][TX_NUM-1] - result_data[i][TX_NUM-2];
	}
	median_right = get_median((s16*)buf, RX_NUM);
	for (i = 0; i < RX_NUM; i++) {
		result_data[i][0] = result_data[i][0] - median_left;
		result_data[i][TX_NUM-1] = result_data[i][TX_NUM-1] - median_right;
	}

	// Ratio for Top/Bottom
#if 0
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "============= Amp Open Test (Assembled-1) =============\n");
#endif
	for (i = 0; i < RX_NUM; i++) {
		for (j = 0; j < TX_NUM; j++) {
			((s16*)buf)[j] = result_data[i][j];
		}
		median_left = get_median((s16*)buf, MUX_SIZE);
		median_right = get_median(((s16*)buf)+(MUX_SIZE), MUX_SIZE);
#if 0
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "[%2d] ", i+1);
#endif
		for (j = 0; j < TX_NUM; j++) {
			if (j < MUX_SIZE)
				median = median_left;
			else
				median = median_right;

			if (median != 0) {
				result = (int)(result_data[i][j]) * 1000 / median;
			}
			else {
				result = S16_MAX;
				TOUCH_E("Division by zero detected!!!\n");
			}

			if (result <= S16_MAX) {
				result_data[i][j] = result;
			}
			else {
				result_data[i][j] = S16_MAX;
			}
#if 0
			log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "%4d ", result_data[i][j]);
#endif
		}
#if 0
		log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
#endif
	}
#if 0
	log_add_used += snprintf(log_add_buf + log_add_used, LOG_ADD_BUF_SIZE - log_add_used, "\n");
#endif

	// Ratio for Left/Right
	for (i = 0; i < TX_NUM; i++) {
		for (j = 0; j < RX_NUM; j++) {
			((s16*)buf)[j] = result_data[j][i];
		}
		median_left = get_median((s16*)buf, RX_NUM_HALF);
		median_right = get_median(((s16*)buf)+(RX_NUM_HALF), RX_NUM_HALF);

		for (j = 0; j < RX_NUM; j++) {
			if (j < RX_NUM_HALF)
				median = median_left;
			else
				median = median_right;

			if (median != 0 && result_data[j][i] != S16_MAX) {
				result = (int)(result_data[j][i]) * 100 / median;
			}
			else {
				result = S16_MAX;
				TOUCH_E("Division by zero detected!!!\n");
			}

			if (result <= S16_MAX) {
				result_data[j][i] = result;
			}
			else {
				result_data[j][i] = S16_MAX;
			}
		}
	}

	return ret;

FAIL:

	synaptics_set_page(dev, DEFAULT_PAGE);
	
	return ret;

}

int td4300_prd_get_sensor_speed_data(struct device *dev)
{
	int i, j, k, ret = 0;

	ret = td4300_prd_get_report_data(dev, REPORT_TYPE_SENSOR_SPEED, synaptics_report_data, REPORT_DATA_LEN);
	if (ret < 0) {
		TOUCH_E("Error to get report data\n");
		return ret;
	}

	// Combine report data to result data
	for (i=0; i<RX_NUM; i++) {
		for (j=0; j<TX_NUM; j++) {
			k = ((TX_NUM-1-j)*RX_NUM + i);
			result_data[i][j] = ((s16*)(synaptics_report_data))[k];
		}
	}

	return ret;
}

int td4300_prd_test_data(struct device *dev, int test_type, int* test_result)
{
	//struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);

	int i, j;
	int ret = 0;

	int limit_upper = 0, limit_lower = 0;
	int min, max/*, aver, stdev*/;
	int fail_count = 0;
	int check_limit_upper = 1, check_limit_lower = 1;

	*test_result = TEST_FAIL;

	ret = snprintf(log_buf, LOG_BUF_SIZE, "IC F/W Version : V%d.%02d\n", d->fw.version[3] & 0x80 ? 1 : 0, d->fw.version[3] & 0x7F);

	switch (test_type) {
		case RAW_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Raw Data Test Result =============\n");
			limit_upper = RAW_DATA_MAX + RAW_DATA_MARGIN;
			limit_lower = RAW_DATA_MIN - RAW_DATA_MARGIN;
			break;
		case P2P_NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Peak-to-Peak Noise Test Result =============\n");
			limit_upper = P2P_NOISE_MAX;
			//limit_lower = P2P_NOISE_MIN;
			check_limit_lower = 0;
			break;
		case E2E_SHORT_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= E-to-E Short Test Result =============\n");
#if !USE_TEST_SHORT_IMAGE2
			limit_upper = E2E_SHORT_MAX;
			check_limit_lower = 0;
#else
			limit_upper = E2E_SHORT_RESULT;
			check_limit_lower = 0;
#endif
			break;
		case AMP_OPEN_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Amp Open Test Result =============\n");
			limit_upper = AMP_OPEN_MAX;
			limit_lower = AMP_OPEN_MIN;
			break;
		case SENSOR_SPEED_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Sensor Speed Test Result =============\n");
			limit_upper = SENSOR_SPEED_MAX;
			limit_lower = SENSOR_SPEED_MIN;
			break;
		case LPWG_RAW_DATA_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG Raw Data Test Result =============\n");
			limit_upper = LPWG_RAW_DATA_MAX + RAW_DATA_MARGIN;
			limit_lower = LPWG_RAW_DATA_MIN - RAW_DATA_MARGIN;
			break;
		case LPWG_P2P_NOISE_TEST:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= LPWG P2P Noise Test Result =============\n");
			limit_upper = LPWG_P2P_NOISE_MAX;
			//limit_lower = LPWG_P2P_NOISE_MIN;
			check_limit_lower = 0;
			break;
		case DELTA_SHOW:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "============= Delta Result =============\n");
			check_limit_upper = check_limit_lower = 0;
			break;
		default:
			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test Failed (Invalid test type)\n");
			return ret;
	}

	max = min = result_data[0][0];

	for (i = 0; i < RX_NUM; i++) {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i+1);
		for (j = 0; j < TX_NUM; j++) {

			ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "%4d ", result_data[i][j]);

			if (check_limit_upper && result_data[i][j] > limit_upper) {
				fail_count++;
			} else if (check_limit_lower && result_data[i][j] < limit_lower) {
				fail_count++;
			}

			if (result_data[i][j] < min)
				min = result_data[i][j];
			if (result_data[i][j] > max)
				max = result_data[i][j];
		}
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "\n");
	}

	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "==================================================\n");

	if(fail_count && (check_limit_upper || check_limit_lower)) {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test FAIL : %d Errors\n", fail_count);
	}
	else {
		ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "Test PASS : No Errors\n");
		*test_result = TEST_PASS;
	}

	ret += snprintf(log_buf + ret, LOG_BUF_SIZE - ret, "MAX = %d, MIN = %d, Upper = %d, Lower = %d\n\n", max, min, limit_upper, limit_lower);

	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Delta Data\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

#if 0 // Prevent rebase
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	// Get Report Data and Show/Test
	ret = td4300_prd_get_delta_data(dev);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Delta Data OK !!!\n");

	ret_size = td4300_prd_test_data(dev, DELTA_SHOW, &test_result);
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Show Delta Data Result : %d\n", test_result);

#if 0
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_size;

FAIL:

	TOUCH_E("Show Delta Data ERROR[%d] !!!\n", ret);

	// Reset Device
	synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return 0;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Raw Data\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	// Get Report Data and Show/Test
	ret = td4300_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Raw Data OK !!!\n");

	if(d->state == TC_STATE_ACTIVE) {
		ret_size = td4300_prd_test_data(dev, RAW_DATA_TEST, &test_result);
	}
	else {
		ret_size = td4300_prd_test_data(dev, LPWG_RAW_DATA_TEST, &test_result);
	}
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Raw Data Test Result : %d\n", test_result);

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_size;

FAIL:

	TOUCH_E("Show Raw Data ERROR[%d] !!!\n", ret);

	// Reset Device
	synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return 0;
}

static ssize_t show_p2p_noise(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Peak-to-Peak Noise Data\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	// Get Report Data and Show/Test
	ret = td4300_prd_get_p2p_noise_data(dev);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Peak-to-Peak Noise Data OK !!!\n");

	if(d->state == TC_STATE_ACTIVE) {
		ret_size = td4300_prd_test_data(dev, P2P_NOISE_TEST, &test_result);
	}
	else {
		ret_size = td4300_prd_test_data(dev, LPWG_P2P_NOISE_TEST, &test_result);
	}
	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Peak-to-Peak Noise Test Result : %d\n", test_result);

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_size;

FAIL:

	TOUCH_E("Show Peak-to-Peak Noise Data ERROR[%d] !!!\n", ret);

	// Reset Device
	synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return 0;
}

static ssize_t show_short_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Short Test\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Check Current State
	if(d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Show Short Test is called in NOT Active state\n");
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		synaptics_irq_clear(dev);
		mutex_unlock(&ts->lock);
		return 0;
	}

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	// Get Report Data and Show/Test
	ret = td4300_prd_get_short_test_data(dev);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Short Test OK !!!\n");

	ret_size = td4300_prd_test_data(dev, E2E_SHORT_TEST, &test_result);

	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Show Short Test Result : %d\n", test_result);

#if USE_TEST_SHORT_IMAGE2
	{
		int i = 0;
		u8 buf[129];

		while (i < log_add_used) {
			memcpy(buf, log_add_buf+i, 128);
			buf[128] = 0;
			printk("%s", buf);
			i += 128;
		}
	}
#endif

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_size;

FAIL:

	TOUCH_E("Show Short Test ERROR[%d] !!!\n", ret);

	// Reset Device
	synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return 0;
}

static ssize_t show_open_test(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Open Test\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Check Current State
	if(d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Show Open Test is called in NOT Active state\n");
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		synaptics_irq_clear(dev);
		mutex_unlock(&ts->lock);
		return 0;
	}

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	// Get Report Data and Show/Test
	ret = td4300_prd_get_open_test_data(dev);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Open Test OK !!!\n");

	ret_size = td4300_prd_test_data(dev, AMP_OPEN_TEST, &test_result);

	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Show Open Test Result : %d\n", test_result);

#if 1
	{
		int i = 0;
		u8 buf[129];

		while (i < log_add_used) {
			memcpy(buf, log_add_buf+i, 128);
			buf[128] = 0;
			printk("%s", buf);
			i += 128;
		}
	}
#endif

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_size;

FAIL:

	TOUCH_E("Show Open Test ERROR[%d] !!!\n", ret);

	// Reset Device
	synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return 0;
}

static ssize_t show_sensor_speed(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ret_size = 0;
	int test_result;

	TOUCH_I("Show Sensor Speed Test\n");

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Check Current State
	if(d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Show Sensor Speed Test is called in NOT Active state\n");
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		synaptics_irq_clear(dev);
		mutex_unlock(&ts->lock);
		return 0;
	}

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;
#endif

	// Get Report Data and Show/Test
	ret = td4300_prd_get_sensor_speed_data(dev);
	if(ret < 0)
		goto FAIL;

	TOUCH_I("Show Sensor Speed Test OK !!!\n");

	ret_size = td4300_prd_test_data(dev, SENSOR_SPEED_TEST, &test_result);

	memcpy(buf, log_buf, ret_size);
	TOUCH_I("Sensor Speed Test Result : %d\n", test_result);

#if 1
	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;
#endif

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_size;

FAIL:

	TOUCH_E("Sensor Speed Test ERROR[%d] !!!\n", ret);

	// Reset Device
	synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return 0;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int test_result, total_result = 0, total_result_ch = 0;

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Check Current State
	if(d->state != TC_STATE_ACTIVE) {
		TOUCH_E("Show_sd is called in NOT Active state\n");
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		synaptics_irq_clear(dev);
		mutex_unlock(&ts->lock);
		return 0;
	}

	/* file create , time log */
	TOUCH_I("Show_sd Test Start\n");
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Start to raw data test
	TOUCH_I("Show_sd : Raw data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = td4300_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = td4300_prd_test_data(dev, RAW_DATA_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("Raw Data Test Result : %d\n", test_result);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to P2P Noise test
	TOUCH_I("Show_sd : P2P Noise test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = td4300_prd_get_p2p_noise_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = td4300_prd_test_data(dev, P2P_NOISE_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("P2P Noise Test Result : %d\n", test_result);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to E2E Short test
	TOUCH_I("Show_sd : E2E Short test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = td4300_prd_get_short_test_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = td4300_prd_test_data(dev, E2E_SHORT_TEST, &test_result);
	total_result_ch |= test_result;

	TOUCH_I("E2E Short Test Result : %d\n", test_result);

	write_file(dev, log_buf, TIME_INFO_SKIP);

#if USE_TEST_SHORT_IMAGE2
	write_file(dev, log_add_buf, TIME_INFO_SKIP); // Additional informations for debugging
#endif

	touch_msleep(30);

#if 1	// AMP Open Test is replaced with Sensor Speed Test in K7/L5/LEMON
	TOUCH_I("Show_sd : Sensor Speed test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = td4300_prd_get_sensor_speed_data(dev);
	if(ret < 0)
		goto FAIL;

	ret_size = td4300_prd_test_data(dev, SENSOR_SPEED_TEST, &test_result);
	total_result_ch |= test_result;

	TOUCH_I("Sensor Speed Test Result : %d\n", test_result);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);
#else
	// Start to Amp Open test
	TOUCH_I("Show_sd : Amp Open test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = td4300_prd_get_open_test_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = td4300_prd_test_data(dev, AMP_OPEN_TEST, &test_result);
	total_result_ch |= test_result;

	TOUCH_I("Amp Open Test Result : %d\n", test_result);

	write_file(dev, log_buf, TIME_INFO_SKIP);

#if 1
	write_file(dev, log_add_buf, TIME_INFO_SKIP); // Additional informations for debugging
#endif

	touch_msleep(30);
#endif

	// Check Result and Print out
	ret_size = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	TOUCH_I("\n========RESULT=======\n");
	if(total_result != TEST_PASS) { // FAIL
		ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "Raw Data : Fail\n");
		TOUCH_I("Raw Data : Fail\n");
	}
	else { // PASS
		ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "Raw Data : Pass\n");
		TOUCH_I("Raw Data : Pass\n");
	}
	if(total_result_ch != TEST_PASS) { // FAIL
		ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "Channel Status : Fail\n");
		TOUCH_I("Channel Status : Fail\n");
	}
	else { // PASS
		ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "Channel Status : Pass\n");
		TOUCH_I("Channel Status : Pass\n");
	}

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test End\n");

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret_size);
	ret_total_size += ret_size;

	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_total_size;

FAIL:

	ret_size = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "Raw Data : Fail\n");
	ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "Channel Status : Fail\n");
	ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "ERROR[%d] !!!\n", ret);
	TOUCH_E("\n========RESULT=======\n");
	TOUCH_E("Raw Data : Fail\n");
	TOUCH_E("Channel Status : Fail\n");
	TOUCH_E("ERROR[%d] !!!\n", ret);

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test End\n");

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret_size);
	ret_total_size += ret_size;

	// Set Dynamic Sensing Control
	synaptics_dynamic_sensing_ctrl(dev, 0);

	// Set No Sleep
	synaptics_set_no_sleep(dev, 0);

	// Reset Device
	synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_total_size;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	int ret = 0;
	int ret_size = 0, ret_total_size = 0;
	int test_result, total_result = 0;

	mutex_lock(&d->fb_lock); // For the case called in progress of LCD off
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	// Check Current State
	if(d->state != TC_STATE_LPWG && d->state != TC_STATE_DEEP_SLEEP) {
		TOUCH_E("Show_lpwg_sd is called in NOT LPWG/SLEEP state\n");
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		synaptics_irq_clear(dev);
		mutex_unlock(&ts->lock);
		mutex_unlock(&d->fb_lock);
		return 0;
	}
	mutex_unlock(&d->fb_lock);

	/* file create , time log */
	TOUCH_I("Show_lpwg_sd Test Start\n");
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 1);
	if(ret < 0)
		goto FAIL;

	// Start to raw data test
	TOUCH_I("Show_lpwg_sd : LPWG Raw data test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = td4300_prd_get_raw_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = td4300_prd_test_data(dev, LPWG_RAW_DATA_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("LPWG Raw Data Test Result : %d\n", test_result);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Start to P2P Noise test
	TOUCH_I("Show_lpwg_sd : LPWG P2P Noise test\n");

	memset(log_buf, 0, LOG_BUF_SIZE);

	ret = td4300_prd_get_p2p_noise_data(dev);
	if(ret < 0)
		goto FAIL;
	
	ret_size = td4300_prd_test_data(dev, LPWG_P2P_NOISE_TEST, &test_result);
	total_result |= test_result;

	TOUCH_I("LPWG P2P Noise Test Result : %d\n", test_result);

	write_file(dev, log_buf, TIME_INFO_SKIP);

	touch_msleep(30);

	// Check Result and Print out
	ret_size = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	TOUCH_I("\n========RESULT=======\n");
	if(total_result != TEST_PASS) { //FAIL
		ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "LPWG RawData : Fail\n");
		TOUCH_I("LPWG RawData : Fail\n");
	}
	else { // PASS
		ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "LPWG RawData : Pass\n");
		TOUCH_I("LPWG RawData : Pass\n");
	}

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test End\n");

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret_size);
	ret_total_size += ret_size;

	// Set Dynamic Sensing Control
	ret = synaptics_dynamic_sensing_ctrl(dev, 0);
	if(ret < 0)
		goto FAIL;

	// Set No Sleep
	ret = synaptics_set_no_sleep(dev, 0);
	if(ret < 0)
		goto FAIL;

#if 0
	// Reset Device
	ret = synaptics_soft_reset(dev);
	if(ret < 0)
		goto FAIL;
#endif

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_total_size;

FAIL:

	ret_size = snprintf(log_buf, LOG_BUF_SIZE, "\n========RESULT=======\n");
	ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "LPWG RawData : Fail\n");
	ret_size += snprintf(log_buf + ret_size, LOG_BUF_SIZE - ret_size, "ERROR[%d] !!!\n", ret);
	TOUCH_E("\n========RESULT=======\n");
	TOUCH_E("LPWG RawData : Fail\n");
	TOUCH_E("ERROR[%d] !!!\n", ret);

	write_file(dev, log_buf, TIME_INFO_SKIP);
	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test End\n");

	log_file_size_check(dev);

	memcpy(buf + ret_total_size, log_buf, ret_size);
	ret_total_size += ret_size;

	// Set Dynamic Sensing Control
	synaptics_dynamic_sensing_ctrl(dev, 0);

	// Set No Sleep
	synaptics_set_no_sleep(dev, 0);

	// Reset Device
	//synaptics_soft_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	synaptics_irq_clear(dev);

	mutex_unlock(&ts->lock);

	return ret_total_size;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(p2p_noise, show_p2p_noise, NULL);
static TOUCH_ATTR(short_test, show_short_test, NULL);
static TOUCH_ATTR(open_test, show_open_test, NULL);
static TOUCH_ATTR(sensor_speed, show_sensor_speed, NULL);

static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_p2p_noise.attr,
	&touch_attr_short_test.attr,
	&touch_attr_open_test.attr,
	&touch_attr_sensor_speed.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int td4300_prd_register_sysfs(struct device *dev)
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
