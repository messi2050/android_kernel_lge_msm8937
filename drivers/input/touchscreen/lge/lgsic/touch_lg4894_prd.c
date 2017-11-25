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
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
//#include <soc/qcom/lge/board_lge.h>
#endif

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_lg4894.h"
#include "touch_lg4894_prd.h"

static char line[50000];
static char W_Buf[BUF_SIZE];
static u16 M2_Rawdata_buf[2][ROW_SIZE*(COL_SIZE+1)];
static u16 M1_Rawdata_buf[ROW_SIZE*M1_COL_SIZE];
static u16 open_buf[ROW_SIZE*(COL_SIZE+1)];
static u16 short_buf[SHORT_ROW_SIZE*SHORT_COL_SIZE];
static u16 LowerImage[ROW_SIZE][COL_SIZE];
static u16 UpperImage[ROW_SIZE][COL_SIZE];
static u16 AverageImage[ROW_SIZE][COL_SIZE];

//for siw app
enum {
	REPORT_END_RS_NG = 0x05,
	REPORT_END_RS_OK = 0xAA,
};

enum {
	PRD_CMD_TYPE_1 = 0,		//new type: base only
	PRD_CMD_TYPE_2 = 1,		//old type: base_even, base_odd
};


enum {
	APP_REPORT_OFF = 0,
	APP_REPORT_RAW,
	APP_REPORT_BASE,
	APP_REPORT_DELTA,
	APP_REPORT_LABEL,
	APP_REPORT_DEBUG_BUF,
	APP_REPORT_MAX,
};

enum {
	PRD_DATA_NAME_SZ	= 128,
	/* */
	//PRD_RAWDATA_SZ_POW	= 1,
	//PRD_RAWDATA_SIZE	= (1<<PRD_RAWDATA_SZ_POW),
	/* */
	PRD_ROW_SIZE		= ROW_SIZE,
	PRD_COL_SIZE		= COL_SIZE,
	PRD_COL_ADD			= 1,
	PRD_CH				= tc_total_ch_size,
	PRD_M1_COL			= M1_COL_SIZE,
	PRD_CMD_TYPE		= PRD_CMD_TYPE_1,
	SECOND_SCR_BOUND_I	= 0,
	SECOND_SCR_BOUND_J	= 0,
	//PRD_M1_COL_SIZE		= (1<<1),
	/* */
	//PRD_LOG_BUF_SIZE	= (1<<10),		//1K
	//PRD_BUF_SIZE		= (PAGE_SIZE<<1),
	/* */
	PRD_SHOW_FLAG_DISABLE_PRT_RAW	= (1<<0),
	PRD_APP_INFO_SIZE	= 32,
};

enum {
	PRD_M2_ROW_COL_SIZE		= (PRD_ROW_SIZE * PRD_COL_SIZE),
	PRD_M2_ROW_COL_BUF_SIZE	= (PRD_ROW_SIZE * (PRD_COL_SIZE + PRD_COL_ADD)),
	//PRD_M1_ROW_COL_SIZE		= (PRD_ROW_SIZE * PRD_M1_COL_SIZE),
	/* */
	//PRD_M2_FRAME_SIZE		= (PRD_M2_ROW_COL_BUF_SIZE<<PRD_RAWDATA_SZ_POW),
	//PRD_M1_FRAME_SIZE		= (PRD_M1_ROW_COL_SIZE<<PRD_RAWDATA_SZ_POW),
	/* */
	PRD_DELTA_SIZE			= ((PRD_ROW_SIZE+2)*(PRD_COL_SIZE+2)),
	/* */
	PRD_LABEL_TMP_SIZE		= ((PRD_ROW_SIZE+2)*(PRD_COL_SIZE+2)),
	PRD_DEBUG_BUF_SIZE		= (PRD_M2_ROW_COL_SIZE),
};

struct siw_hal_prd_data {
	struct device *dev;
	char name[PRD_DATA_NAME_SZ];
	/* */
	int prd_app_mode;
	/* */
	int16_t	m2_buf_rawdata[PRD_M2_ROW_COL_BUF_SIZE];
	//int16_t	m2_buf_even_rawdata[PRD_M2_ROW_COL_BUF_SIZE];
	/* */
	//int16_t	m1_buf_odd_rawdata[PRD_M1_ROW_COL_SIZE];
	//int16_t	m1_buf_even_rawdata[PRD_M1_ROW_COL_SIZE];
	//int16_t m1_buf_tmp[PRD_M1_ROW_COL_SIZE];
	/* */
	//int image_lower;
	//int image_upper;
	/* */
	int16_t	buf_delta[PRD_DELTA_SIZE];
	int16_t	buf_debug[PRD_DEBUG_BUF_SIZE];
	u8	buf_label_tmp[PRD_LABEL_TMP_SIZE];
	u8	buf_label[PRD_M2_ROW_COL_SIZE];
};

enum {
	PRD_SYS_EN_IDX_SD = 0,
	PRD_SYS_EN_IDX_DELTA,
	PRD_SYS_EN_IDX_LABEL,
	PRD_SYS_EN_IDX_RAWDATA_PRD,
	PRD_SYS_EN_IDX_RAWDATA_TCM,
	PRD_SYS_EN_IDX_RAWDATA_AIT,
	PRD_SYS_EN_IDX_BASE,
	PRD_SYS_EN_IDX_DEBUG_BUF,
	PRD_SYS_EN_IDX_LPWG_SD,
	PRD_SYS_EN_IDX_FILE_TEST,
	PRD_SYS_EN_IDX_APP_RAW,
	PRD_SYS_EN_IDX_APP_BASE,
	PRD_SYS_EN_IDX_APP_LABEL,
	PRD_SYS_EN_IDX_APP_DELTA,
	PRD_SYS_EN_IDX_APP_DEBUG_BUF,
	PRD_SYS_EN_IDX_APP_END,
	PRD_SYS_EN_IDX_APP_INFO,
	//
	PRD_SYS_ATTR_MAX,
};

enum {
	PRD_SYS_EN_SD					= (1<<PRD_SYS_EN_IDX_SD),
	PRD_SYS_EN_DELTA				= (1<<PRD_SYS_EN_IDX_DELTA),
	PRD_SYS_EN_LABEL				= (1<<PRD_SYS_EN_IDX_LABEL),
	PRD_SYS_EN_RAWDATA_PRD			= (1<<PRD_SYS_EN_IDX_RAWDATA_PRD),
	PRD_SYS_EN_RAWDATA_TCM			= (1<<PRD_SYS_EN_IDX_RAWDATA_TCM),
	PRD_SYS_EN_RAWDATA_AIT			= (1<<PRD_SYS_EN_IDX_RAWDATA_AIT),
	PRD_SYS_EN_BASE					= (1<<PRD_SYS_EN_IDX_BASE),
	PRD_SYS_EN_DEBUG_BUF			= (0<<PRD_SYS_EN_IDX_DEBUG_BUF),
	PRD_SYS_EN_LPWG_SD				= (1<<PRD_SYS_EN_IDX_LPWG_SD),
	PRD_SYS_EN_FILE_TEST			= (1<<PRD_SYS_EN_IDX_FILE_TEST),
	PRD_SYS_EN_APP_RAW				= (1<<PRD_SYS_EN_IDX_APP_RAW),
	PRD_SYS_EN_APP_BASE				= (1<<PRD_SYS_EN_IDX_APP_BASE),
	PRD_SYS_EN_APP_LABEL			= (1<<PRD_SYS_EN_IDX_APP_LABEL),
	PRD_SYS_EN_APP_DELTA			= (1<<PRD_SYS_EN_IDX_APP_DELTA),
	PRD_SYS_EN_APP_DEBUG_BUF		= (1<<PRD_SYS_EN_IDX_APP_DEBUG_BUF),
	PRD_SYS_EN_APP_END				= (1<<PRD_SYS_EN_IDX_APP_END),
	PRD_SYS_EN_APP_INFO				= (1<<PRD_SYS_EN_IDX_APP_INFO),
};

#define PRD_SYS_ATTR_EN_FLAG 		(0 |	\
									PRD_SYS_EN_SD |	\
									PRD_SYS_EN_DELTA |	\
									PRD_SYS_EN_LABEL |	\
									PRD_SYS_EN_RAWDATA_PRD |	\
									PRD_SYS_EN_RAWDATA_TCM |	\
									PRD_SYS_EN_RAWDATA_AIT |	\
									PRD_SYS_EN_BASE |	\
									PRD_SYS_EN_DEBUG_BUF |	\
									PRD_SYS_EN_LPWG_SD |	\
									PRD_SYS_EN_FILE_TEST |	\
									PRD_SYS_EN_APP_RAW |	\
									PRD_SYS_EN_APP_BASE |	\
									PRD_SYS_EN_APP_LABEL |	\
									PRD_SYS_EN_APP_DELTA | 	\
									PRD_SYS_EN_APP_DEBUG_BUF |	\
									PRD_SYS_EN_APP_END |	\
									PRD_SYS_EN_APP_INFO |	\
									0)
//for siw app end

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

static int write_test_control(struct device *dev, u32 mode)
{

	u32 test_mode_enter_cmt = mode;
	u32 test_mode_enter_check = -1;
	unsigned int delay_ms = 30;
	int ret = 0;

	ret = lg4894_reg_write(dev, tc_test_mode_ctl,
				(u32 *)&test_mode_enter_cmt, sizeof(test_mode_enter_cmt));
	TOUCH_I("write tc_test_mode_ctl = %x\n", test_mode_enter_cmt);
	touch_msleep(delay_ms);

	lg4894_reg_read(dev, tc_test_mode_ctl,
			(u32 *)&test_mode_enter_check, sizeof(u32));
	TOUCH_I("read tc_test_mode_ctl= %x\n", test_mode_enter_check);

	return ret;
}

static int write_test_mode(struct device *dev, u8 type)
{
	u32 testmode = 0;
	u8 disp_mode = 0x3;
	int retry = 40;
	u32 rdata = 0x01;
	int waiting_time = 200;
	u8 type_temp = 0;
	u32 backlightness = 0;

	switch (type) {
	case OPEN_NODE_TEST:
		type_temp = 0x2;
		testmode = (disp_mode << 8) + type_temp;
		waiting_time = 10;
		break;
	case SHORT_NODE_TEST:
		type_temp = 0x3;
		testmode = (disp_mode << 8) + type_temp;
		waiting_time = 1000;
		break;
	case U3_M2_RAWDATA_TEST:
		type_temp = 0x5;
		testmode = (disp_mode << 8) + type_temp;
		break;
	case U0_M1_RAWDATA_TEST:
		type_temp = 0x6;
		testmode = type_temp;
		break;
	case U0_M2_RAWDATA_TEST:
		type_temp = 0x5;
		testmode = type_temp;
		break;
	case U3_BLU_JITTER_TEST:
		type_temp = 0xC;
		waiting_time = 10;
		retry = 60;
		testmode = ((disp_mode << 8) + type_temp) | LINE_FILTER_OPTION;
		break;
	case U3_JITTER_TEST:
		type_temp = 0xC;
		testmode = ((disp_mode << 8) + type_temp) | LINE_FILTER_OPTION;
		break;
	case U0_JITTER_TEST:
		type_temp = 0xC;
		testmode = (type_temp) | LINE_FILTER_OPTION;
		break;
	}

	/* TestType Set */
	lg4894_reg_write(dev, tc_tsp_test_ctl,
			(u8 *)&testmode,
			sizeof(testmode));
	TOUCH_I("write testmode = %x\n", testmode);
	touch_msleep(waiting_time);

	if (type == U3_BLU_JITTER_TEST) {
#if defined(CONFIG_TOUCHSCREEN_MTK)
		backlightness = mt_get_bl_brightness();
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
		backlightness = mdss_fb_get_bl_brightness_extern();
#endif
		touch_msleep(742);
#if defined(CONFIG_TOUCHSCREEN_MTK)
		mt65xx_leds_brightness_set(6, 0);
		TOUCH_I("BackLight Off at BLU JITTER (CONFIG_TOUCHSCREEN_MTK)\n");
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
		mdss_fb_set_bl_brightness_extern(0);
		TOUCH_I("BackLight Off at BLU JITTER TEST (CONFIG_LGE_TOUCH_CORE_QCT)\n");
#endif
		touch_msleep(278);
#if defined(CONFIG_TOUCHSCREEN_MTK)
		mt65xx_leds_brightness_set(6, backlightness);
		TOUCH_I("BackLight On at BLU JITTER (CONFIG_TOUCHSCREEN_MTK)\n");
#elif defined(CONFIG_LGE_TOUCH_CORE_QCT)
		mdss_fb_set_bl_brightness_extern(backlightness);
		TOUCH_I("BackLight On at BLU JITTER (CONFIG_LGE_TOUCH_CORE_QCT)\n");
#endif
		}

	/* Check Test Result - wait until 0 is written */
	do {
		touch_msleep(50);
		lg4894_reg_read(dev, tc_tsp_test_sts,
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

static int prd_os_result_get(struct device *dev, int type)
{
	u32 os_result_offset;
	u16 open_test_offset[3] = {0,0,0};
	u16 short_test_offset = 0;
/*
	lg4894_reg_read(dev, prod_open1_open2_offset,
		(u8 *)&os_result_offset, sizeof(u32));
	open_test_offset[0] = (os_result_offset & 0xFFFF);
	open_test_offset[1] = (os_result_offset >> 16);
*/
	lg4894_reg_read(dev, prod_open3_short_offset,
		(u8 *)&os_result_offset, sizeof(u32));
	open_test_offset[2] = (os_result_offset & 0xFFFF);
	short_test_offset = (os_result_offset >> 16);

	switch(type) {
	case OPEN_NODE_TEST:
		TOUCH_I("open_test_offset = %x\n", open_test_offset[2]);
		lg4894_reg_write(dev, tc_tsp_test_data_offset,
							(u8 *)&open_test_offset[2], sizeof(u16));
		lg4894_reg_read(dev, tc_tsp_data_access_addr,
							(u8 *)open_buf, ROW_SIZE*(COL_SIZE+1)*sizeof(u16));
		break;
	case SHORT_NODE_TEST:
		TOUCH_I("short_test_offset = %x\n", short_test_offset);
		lg4894_reg_write(dev, tc_tsp_test_data_offset,
							(u8 *)&short_test_offset, sizeof(u16));
		lg4894_reg_read(dev, tc_tsp_data_access_addr,
							(u16 *)short_buf, SHORT_ROW_SIZE*SHORT_COL_SIZE*sizeof(u16));
		break;
	}

	return 0;
}

static int prd_os_xline_result_read(struct device *dev, int type)
{
	int ret = 0;

	ret = prd_os_result_get(dev, type);

	return ret;
}

static int prd_print_os_data(struct device *dev, char *buf, u8 type)
{
	int i = 0, j = 0;
	int ret = 0;
	u16 *os_buf = NULL;
	int col_size = 0;

	/* print a frame data */
	ret = snprintf(buf, PAGE_SIZE, "\n   : ");

	switch (type) {
	case OPEN_NODE_TEST:
		col_size = COL_SIZE + 1;
		os_buf = open_buf;
		break;
	case SHORT_NODE_TEST:
		col_size = M1_COL_SIZE;
		os_buf = short_buf;
		break;
	}

	for (i = 0; i < col_size - 1; i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, " [%2d] ", i);

	for (i = 0; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0, };
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret,  "\n[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,  "[%2d]  ", i);
		for (j = 0; j < col_size; j++) {
			if (j == COL_SIZE)
				continue;
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", os_buf[i*col_size+j]);
			log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret,
					"%5d ", os_buf[i*col_size+j]);
		}
		TOUCH_I("%s\n", log_buf);
	}

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	write_file(dev, W_Buf, TIME_INFO_SKIP);
	memset(W_Buf, 0, BUF_SIZE);

	return ret;
}

static int sdcard_spec_file_read(struct device *dev)
{
	int ret = 0;
	int fd = 0;
	char *path[2] = { "/mnt/sdcard/lv5_limit.txt",
		"/mnt/sdcard/lv5_limit_mfts.txt"
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

static int prd_compare_os_data(struct device *dev, u8 type)
{
	/* spec reading */
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	u16 short_rerange_buf;

	int col_size = 0;
	int i, j;
	int short_buf_index = 0;
	int short_buf_index_change = 0;
	int ret = 0;
	int result = 0;

	switch (type) {
	case 	OPEN_NODE_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"OPEN_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"OPEN_Upper");
		col_size = COL_SIZE+1;
		break;
	case	SHORT_NODE_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"SHORT_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"SHORT_Upper");
		col_size = M1_COL_SIZE;
		break;
	}

	sic_get_limit(dev, lower_str, LowerImage);
	sic_get_limit(dev, upper_str, UpperImage);

	if (type == SHORT_NODE_TEST) {
		for (i = 0; i < ROW_SIZE; i++) { //ROW_SIZE = 26
			for (j = 0; j < col_size; j++) { //col_size = 2 //M1_COL_SIZE(2)+1; --> j < 2

				short_buf_index = i*col_size + j;

				if (short_buf_index > (ROW_SIZE-1)){ //Right
					short_buf_index_change = short_buf_index + 38; //SHORT_ROW_SIZE =	32
				} else {
					short_buf_index_change = short_buf_index;
				}

				short_rerange_buf = short_buf[short_buf_index_change];
				short_buf[i*col_size + j] = short_rerange_buf; //Test

				if ((short_buf[i*col_size + j] < LowerImage[i][j]) ||
					(short_buf[i*col_size + j] > UpperImage[i][j])) {
					result = 1;
					ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
						"F [%d][%d] = %d\n", i, j, short_buf[i*col_size + j]);
				}
			}
		}
	}
	else if (type == OPEN_NODE_TEST) {
		for (i = 0; i < ROW_SIZE; i++) {
			for (j = 0; j < col_size-1; j++) {
				if ((open_buf[i*col_size+j] < LowerImage[i][j]) ||
					(open_buf[i*col_size+j] > UpperImage[i][j])) {
					result = 1;
					ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
						"F [%d][%d] = %d\n", i, j, open_buf[i*col_size+j]);
				}
			}
		}
	}
	else {
		TOUCH_E("Check your configuration\n");
	}
	return result;
}

static int prd_open_short_test(struct device *dev)
{
	int type = 0;
	int ret = 0;
	int write_test_mode_result = 0;
#if 0 //old solution
	u32 open_result = 0;
	u32 short_result = 0;
#endif
	u32 openshort_all_result = 0;

	/* Test Type Write */
	write_file(dev, "[OPEN_SHORT_ALL_TEST]\n", TIME_INFO_SKIP);

#if 0 //old solution
	/* 1. open_test */
	type = OPEN_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	lg4894_reg_read(dev, tc_tsp_test_pf_result,
			(u8 *)&open_result, sizeof(open_result));
	TOUCH_I("open_result = %d\n", open_result);

	if (open_result) {
		// open test logging: M2_DIFF_data_offset
		ret = prd_os_xline_result_read(dev, type);
		prd_print_os_data(dev, W_Buf, type);
		openshort_all_result |= 0x1;
	}

	/* 2. short_test */
	type = SHORT_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	lg4894_reg_read(dev, tc_tsp_test_pf_result,
		(u8 *)&short_result, sizeof(short_result));
	TOUCH_I("short_result = %d\n", short_result);

	if (short_result) {
		// short test logging
		ret = prd_os_xline_result_read(dev, type);
		prd_print_os_data(dev, W_Buf, type);
		openshort_all_result |= 0x2;
	}

	/* fail case */
	if (openshort_all_result != 0) {
		ret = snprintf(W_Buf + ret, BUF_SIZE - ret,
				"OPEN_SHORT_ALL_TEST : Fail\n");
	} else
		ret = snprintf(W_Buf + ret, BUF_SIZE - ret,
				"OPEN_SHORT_ALL_TEST : Pass\n");
#else //new solution
	/* 1. open_test */
	type = OPEN_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}
	prd_os_xline_result_read(dev, type);
	openshort_all_result = prd_compare_os_data(dev, type);
	prd_print_os_data(dev, W_Buf, type);


	/* 2. short_test */
	type = SHORT_NODE_TEST;
	write_test_mode_result = write_test_mode(dev, type);
	if (write_test_mode_result == 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}
	prd_os_xline_result_read(dev, type);
	openshort_all_result = prd_compare_os_data(dev, type);
	prd_print_os_data(dev, W_Buf, type);


	/* fail case */
	if (openshort_all_result != 0) {
		ret = snprintf(W_Buf + ret, BUF_SIZE - ret,
				"OPEN_SHORT_ALL_TEST : Fail\n");
	} else
		ret = snprintf(W_Buf + ret, BUF_SIZE - ret,
				"OPEN_SHORT_ALL_TEST : Pass\n");
#endif

	write_file(dev, W_Buf, TIME_INFO_SKIP);

	return openshort_all_result;
}

static void prd_read_rawdata(struct device *dev, u8 type)
{
	u32 raw_offset_info = 0;
	u16 raw_offset = 0;
	u16 m2_raw_offset[M1_M2_RAWDATA_TEST_CNT_MAX] = {0, 0};

	int __m1_frame_size = ROW_SIZE*M1_COL_SIZE*RAWDATA_SIZE;
	int __m2_frame_size = ROW_SIZE*(COL_SIZE+1)*RAWDATA_SIZE;
	int i = 0;

	if (__m1_frame_size % 4)
		__m1_frame_size = (((__m1_frame_size >> 2) + 1) << 2);
	if (__m2_frame_size % 4)
		__m2_frame_size = (((__m2_frame_size >> 2) + 1) << 2);

	/* get offset for m1/m2. same address are being used */
	lg4894_reg_read(dev, prod_open1_open2_offset,
		(u8 *)&raw_offset_info, sizeof(u32));
	m2_raw_offset[0] = (raw_offset_info >> 16);		// even
	m2_raw_offset[1] = (raw_offset_info & 0xFFFF); // odd

	switch (type) {
	case U0_M1_RAWDATA_TEST:
		memset(M1_Rawdata_buf, 0, sizeof(M1_Rawdata_buf));

		for (i = 0; i < M1_M2_RAWDATA_TEST_CNT; i++){
			raw_offset = m2_raw_offset[i];
			TOUCH_I("raw_offset[%d] = %0x\n", i, raw_offset);
			lg4894_reg_write(dev, tc_tsp_test_data_offset,
					(u8 *)&raw_offset, sizeof(u16));
			lg4894_reg_read(dev, tc_tsp_data_access_addr,
					(u8 *)&M1_Rawdata_buf[i*ROW_SIZE], __m1_frame_size);
		}

		break;
	case U3_M2_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
	case U3_BLU_JITTER_TEST:
	case U3_JITTER_TEST:
	case U0_JITTER_TEST:
		memset(M2_Rawdata_buf, 0, sizeof(M2_Rawdata_buf));

		for(i = 0; i < M1_M2_RAWDATA_TEST_CNT; i++){
			raw_offset = m2_raw_offset[i];
			TOUCH_I("raw_offset[%d] = %0x\n", i, raw_offset);
			lg4894_reg_write(dev, tc_tsp_test_data_offset,
					(u8 *)&raw_offset, sizeof(u16));
			lg4894_reg_read(dev, tc_tsp_data_access_addr,
					(u8 *)&M2_Rawdata_buf[i], __m2_frame_size);
		}
		break;
	}
}

static int prd_print_rawdata(struct device *dev, char *buf, u8 type)
{
	int i = 0, j = 0, k = 0;
	int ret = 0;
	int min = 9999;
	int max = 0;
	u16 *m1_raw_buf = NULL;
	u16 (*m2_raw_buf)[ROW_SIZE*(COL_SIZE+1)] = NULL;
	int col_size = 0;
	int cnt = 0;

	/* print a frame data */
	ret = snprintf(buf, PAGE_SIZE, "\n   : ");

	switch (type) {
	case U0_M1_RAWDATA_TEST:
		col_size = M1_COL_SIZE;
		m1_raw_buf = M1_Rawdata_buf;

		ret = snprintf(buf, PAGE_SIZE, "\n   : ");

		for (i = 0; i < col_size; i++)
			ret += snprintf(buf + ret, PAGE_SIZE - ret, " [%2d] ", i);

		for (i = 0; i < ROW_SIZE; i++) {
			char log_buf[LOG_BUF_SIZE] = {0, };
			int log_ret = 0;

			ret += snprintf(buf + ret, PAGE_SIZE - ret,  "\n[%2d] ", i);
			log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret,  "[%2d]  ", i);
			for (j = 0; j < col_size; j++) {
				TOUCH_I("%d ", m1_raw_buf[cnt]);
				cnt++;
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
						"%5d ", m1_raw_buf[i+ROW_SIZE*j]);
				log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret,
						"%5d ", m1_raw_buf[i+ROW_SIZE*j]);
				if (m1_raw_buf[i+ROW_SIZE*j] != 0 &&
						m1_raw_buf[i+ROW_SIZE*j] < min)
					min = m1_raw_buf[i+ROW_SIZE*j];
				if (m1_raw_buf[i+ROW_SIZE*j] > max)
					max = m1_raw_buf[i+ROW_SIZE*j];
			}
			TOUCH_I("%s\n", log_buf);
		}

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");

		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"\nRawdata min : %d , max : %d\n", min, max);

		write_file(dev, buf, TIME_INFO_SKIP);
		memset(buf, 0, BUF_SIZE);

		break;
	case U0_M2_RAWDATA_TEST:
	case U3_M2_RAWDATA_TEST:
	case U3_BLU_JITTER_TEST:
	case U3_JITTER_TEST:
	case U0_JITTER_TEST:
		col_size = COL_SIZE + 1;
		m2_raw_buf = M2_Rawdata_buf;

		for (k = 0; k < M1_M2_RAWDATA_TEST_CNT; k++) {
			ret = snprintf(buf, PAGE_SIZE, "\n   : ");

			for (i = 0; i < col_size - 1; i++)
				ret += snprintf(buf + ret, PAGE_SIZE - ret, " [%2d] ", i);

			for (i = 0; i < ROW_SIZE; i++) {
				char log_buf[LOG_BUF_SIZE] = {0, };
				int log_ret = 0;

				ret += snprintf(buf + ret, PAGE_SIZE - ret,  "\n[%2d] ", i);
				log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret,  "[%2d]  ", i);
				for (j = 0; j < col_size; j++) {
					if (j == COL_SIZE)
						continue;
					ret += snprintf(buf + ret, PAGE_SIZE - ret,
							"%5d ", m2_raw_buf[k][i*col_size+j]);
					log_ret += snprintf(log_buf + log_ret,
							LOG_BUF_SIZE - log_ret,
							"%5d ", m2_raw_buf[k][i*col_size+j]);
					if (m2_raw_buf[k][i*col_size+j] != 0 &&
							m2_raw_buf[k][i*col_size+j] < min)
						min = m2_raw_buf[k][i*col_size+j];
					if (m2_raw_buf[k][i*col_size+j] > max)
						max = m2_raw_buf[k][i*col_size+j];
				}
				TOUCH_I("%s\n", log_buf);
			}

			ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");

			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"\nRawdata min : %d , max : %d\n", min, max);

			write_file(dev, buf, TIME_INFO_SKIP);
			memset(buf, 0, BUF_SIZE);
		}
		break;
	}

	return ret;
}


/* Rawdata compare result
	Pass : reurn 0
	Fail : return 1
*/
static int prd_compare_rawdata(struct device *dev, u8 type)
{
	/* spec reading */
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	char average_str[64] = {0, };
	u16 *m1_raw_buf = NULL;
	u16 (*m2_raw_buf)[ROW_SIZE*(COL_SIZE+1)] = NULL;

	//Average Jiiter Buf
	u16 m2_raw_average_buf[2][ROW_SIZE] = {{0}};
	int col_size = 0;
	int i, j, k;
	int ret = 0;
	int result = 0;

	switch (type) {
	case U0_M1_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U0_M1_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U0_M1_Upper");
		col_size = M1_COL_SIZE+1;
		m1_raw_buf = M1_Rawdata_buf;
		break;
	case U0_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U0_M2_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U0_M2_Upper");
		col_size = COL_SIZE+1;
		m2_raw_buf = M2_Rawdata_buf;
		break;
	case U3_M2_RAWDATA_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U3_M2_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U3_M2_Upper");
		col_size = COL_SIZE+1;
		m2_raw_buf = M2_Rawdata_buf;
		break;
	case U3_BLU_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U3_BLU_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U3_BLU_JITTER_Upper");
		col_size = COL_SIZE+1;
		m2_raw_buf = M2_Rawdata_buf;
		break;
	case U3_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U3_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U3_JITTER_Upper");
		col_size = COL_SIZE+1;
		m2_raw_buf = M2_Rawdata_buf;
		break;
	case U0_JITTER_TEST:
		snprintf(lower_str, sizeof(lower_str),
				"U0_JITTER_Lower");
		snprintf(upper_str, sizeof(upper_str),
				"U0_JITTER_Upper");
		col_size = COL_SIZE+1;
		m2_raw_buf = M2_Rawdata_buf;
		break;
	}
	snprintf(average_str, sizeof(average_str),
		"AVERAGE_JITTER");

	sic_get_limit(dev, lower_str, LowerImage);
	sic_get_limit(dev, upper_str, UpperImage);
	sic_get_limit(dev, average_str, AverageImage);

	if (type == U0_M1_RAWDATA_TEST) {
		for (i = 0; i < ROW_SIZE; i++) {
			for (j = 0; j < col_size-1; j++) {
				if ((m1_raw_buf[i+col_size*j] < LowerImage[i][j]) ||
						(m1_raw_buf[i+col_size*j] > UpperImage[i][j])) {
#if 0  // if non Sub-LCD(AOD), need not this code
					if ((type != U0_M1_RAWDATA_TEST) && (i <= 1 && j <= 4)) {
						if (m1_raw_buf[i+col_size*j] != 0) {
						result = 1;
						ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
						"F [%d][%d] = %d\n", i, j, m1_raw_buf[i+col_size*j]);
						}
					} else {
						result = 1;
						ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
							"F [%d][%d] = %d\n", i, j, m1_raw_buf[i+col_size*j]);
					}
#else
					result = 1;
					ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
						"F [%d][%d] = %d\n", i, j, m1_raw_buf[i+col_size*j]);

#endif
				}
			}
		TOUCH_I("\n ");
		}
	}
	else if (type == U3_M2_RAWDATA_TEST ||
				type == U0_M2_RAWDATA_TEST ||
				type == U3_BLU_JITTER_TEST ||
				type == U3_JITTER_TEST ||
				type == U0_JITTER_TEST) {
		for (k =0; k < M1_M2_RAWDATA_TEST_CNT; k++) {
			for (i = 0; i < ROW_SIZE; i++) {
				for (j = 0; j < col_size-1; j++) {
					if ((m2_raw_buf[k][i*col_size+j] < LowerImage[i][j]) ||
						(m2_raw_buf[k][i*col_size+j] > UpperImage[i][j])) {
#if 0  // if non Sub-LCD(AOD), need not this code
						if ((type != U0_M1_RAWDATA_TEST) &&
								(i <= 1 && j <= 4)) {
							if (m2_raw_buf[k][i*col_size+j] != 0) {
								result = 1;
								ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
										"F [%d][%d] = %d\n", i, j, m2_raw_buf[k][i*col_size+j]);
							}
						} else {
							result = 1;
							ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
									"F [%d][%d] = %d\n", i, j, m2_raw_buf[k][i*col_size+j]);
						}
#else
						result = 1;
						ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
							"F [%d][%d] = %d\n", i, j, m2_raw_buf[k][i*col_size+j]);
#endif
					}
				}
			}

			if ((type == U3_JITTER_TEST) || (type == U3_BLU_JITTER_TEST) || (type == U0_JITTER_TEST))
			{
				//Average jitter
				for (i = 0; i < ROW_SIZE; i++) {
					for (j = 0; j < col_size-1; j++) {

						//Average Jitter Buf store
						//m2_raw_average_buf[j] += m2_raw_buf[k][i*col_size+j];
						if(j < 8)
						{
							m2_raw_average_buf[0][i] += m2_raw_buf[k][i*col_size+j];
						}
						else
						{
							m2_raw_average_buf[1][i] += m2_raw_buf[k][i*col_size+j];
						}
					}
					m2_raw_average_buf[0][i] /= 8; //Left
					m2_raw_average_buf[1][i] /= 7; //Right

					if (m2_raw_average_buf[0][i] > AverageImage[i][0])
					{
						result = 1;
						ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
							"F [%d] Row Left Average = %d\n", i, m2_raw_average_buf[0][i]);
					}
					if (m2_raw_average_buf[1][i] > AverageImage[i][1])
					{
						result = 1;
						ret += snprintf(W_Buf + ret, BUF_SIZE - ret,
							"F [%d] Row Right Average = %d\n", i, m2_raw_average_buf[1][i]);
					}
				}
			}
		}
	}
	else {
		TOUCH_E("Check your configuration\n");
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

	lg4894_reg_read(dev, 0x289,
		(u8 *)&tune_code_offset, sizeof(u32));
	offset = (tune_code_offset >> 16) & 0xFFFF;

	lg4894_reg_write(dev, tc_tsp_test_data_offset,
			(u8 *)&offset,
			sizeof(u32));

	lg4894_reg_read(dev, tc_tsp_data_access_addr,
			(u8 *)&tune_code_read_buf[0], tc_tune_code_size);


	switch (type) {
	case U0_M1_RAWDATA_TEST:
		write_file(dev, "\n[Read Tune Code]\n", TIME_INFO_SKIP);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_GOFT_OFFSET, 1);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_GOFT_OFFSET, 1);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_L_M1_OFT_OFFSET, 2);
		tune_display(dev, tune_code_read_buf,
			TSP_TUNE_CODE_R_M1_OFT_OFFSET, 2);
		break;
	case U3_M2_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
		write_file(dev, "\n[Read Tune Code]\n", TIME_INFO_SKIP);
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
	case U3_M2_RAWDATA_TEST:
		snprintf(test_type, sizeof(test_type),
				"[U3_M2_RAWDATA_TEST]");
		break;
	case U0_M1_RAWDATA_TEST:
		snprintf(test_type, sizeof(test_type),
				"[U0_M1_RAWDATA_TEST]");
		break;
	case U0_M2_RAWDATA_TEST:
		snprintf(test_type, sizeof(test_type),
				"[U0_M2_RAWDATA_TEST]");
		break;
	case U3_BLU_JITTER_TEST:
		snprintf(test_type, sizeof(test_type),
				"[U3_BLU_JITTER_TEST]");
		break;
	case U3_JITTER_TEST:
		snprintf(test_type, sizeof(test_type),
				"[U3_JITTER_TEST]");
		break;
	case U0_JITTER_TEST:
		snprintf(test_type, sizeof(test_type),
				"[U0_JITTER_TEST]");
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
	//write_file(dev, W_Buf, TIME_INFO_SKIP);
	//memset(W_Buf, 0, BUF_SIZE);

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

	lg4894_reg_read(dev, info_lot_num, (u8 *)&rdata, sizeof(rdata));

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
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int boot_mode = 0;

	boot_mode = touch_boot_mode_check(dev);
	if (boot_mode >= MINIOS_MFTS_FOLDER)
		ret = lg4894_ic_info(dev);

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"======== Firmware Info ========\n");
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d\n",
			d->fw.version[0], d->fw.version[1]);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"revision : %d\n", d->fw.revision);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"product id : %s\n", d->fw.product_id);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"lcd fpcb revision : %d\n", d->fw.lcd_fpcb_revision);

	write_file(dev, buffer, TIME_INFO_SKIP);
}

static int ic_exception_check(struct device *dev, char *buf)
{
#if 0
	struct lg4894_data *d = to_lg4894_data(dev);
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

static ssize_t show_sd(struct device *dev, char *buf)
{

	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int openshort_ret = 1;
	int rawdata_ret = 1;
	int blu_jitter_ret = 1;
	int jitter_ret = 1;
	int ret = 0;

	/* file create , time log */
	write_file(dev, "\nShow_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_sd Test Start\n");


	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"LCD mode is not U3. Test Result : Fail\n");
		return ret;
	}

	/* ic rev check - MINIOS mode, MFTS mode check */
	ret = ic_exception_check(dev, buf);
	if (ret > 0)
		return ret;

	firmware_version_log(dev);
	ic_run_info_print(dev);

	mutex_lock(&ts->lock);

	write_test_control(ts->dev, cmd_test_enter);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	lg4894_tc_driving(dev, LCD_MODE_STOP);

	/*
		U3_M2_RAWDATA_TEST
		rawdata - pass : 0, fail : 1
		rawdata tunecode - pass : 0, fail : 2
	*/
	rawdata_ret = prd_rawdata_test(dev, U3_M2_RAWDATA_TEST);
	/*
		U3_JITTER_TEST
		BLU Jitter - pass : 0, fail : 1
		This will be enabled later.
	*/
	jitter_ret = prd_rawdata_test(dev, U3_JITTER_TEST);
		/*
		U3_BLU_JITTER_TEST
		BLU Jitter - pass : 0, fail : 1
		This will be enabled later.
	*/
	blu_jitter_ret = prd_rawdata_test(dev, U3_BLU_JITTER_TEST);
	/*
		OPEN_SHORT_ALL_TEST
		open - pass : 0, fail : 1
		short - pass : 0, fail : 2
	*/
	openshort_ret = prd_open_short_test(dev);

	ret = snprintf(buf, PAGE_SIZE,
			"\n========RESULT=======\n");
	if (rawdata_ret == 0 && jitter_ret == 0 && blu_jitter_ret == 0)
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Pass\n");
	else
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Fail (raw:%d/jitter:%d/blu:%d)\n",
				rawdata_ret, jitter_ret, blu_jitter_ret);

	if (openshort_ret == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Channel Status : Fail (%d/%d)\n",
			((openshort_ret & 0x1) == 0x1) ? 0 : 1,
			((openshort_ret & 0x2) == 0x2) ? 0 : 1);
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	write_test_control(ts->dev, cmd_test_exit);

	/*
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(ts->caps.hw_reset_delay);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	*/
	lg4894_reset_ctrl(dev, HW_RESET_SYNC);
	mutex_unlock(&ts->lock);

	write_file(dev, "Show_sd Test End\n", TIME_INFO_WRITE);
	log_file_size_check(dev);
	TOUCH_I("Show_sd Test End\n");
	return ret;
}

static int start_firmware(struct device *dev)
{
	u32 const cmd = IT_NONE;
	u32 check_data = 0;
	int ret = 0;

	/* Release F/W to operate */
	ret = lg4894_reg_write(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, (void *)&cmd,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	ret = lg4894_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	ret = lg4894_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	if (ret < 0) {
		goto error;
	}
	TOUCH_I("check_data : %x\n", check_data);

error:
	return ret;
}

static int stop_firmware(struct device *dev, u32 wdata)
{
	u32 read_val;
	u32 check_data=0;
	int try_cnt=0;
	int ret = 0;

	/* STOP F/W to check */
	lg4894_reg_write(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &wdata, sizeof(u32));
	lg4894_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));
	lg4894_reg_read(dev, ADDR_CMD_REG_SIC_IMAGECTRL_TYPE, &check_data,
			sizeof(u32));

	try_cnt = 1000;
	do
	{
		--try_cnt;
		if (try_cnt == 0) {
			TOUCH_E("[ERR]get_data->try_cnt == 0\n");
			ret = 1;
			goto error;
		}
		lg4894_reg_read(dev, ADDR_CMD_REG_SIC_GETTER_READYSTATUS,
				&read_val, sizeof(u32));
		TOUCH_I("read_val = [%x] , RS_IMAGE = [%x]\n",read_val,
				(u32)RS_IMAGE);
		touch_msleep(10);
	} while(read_val != (u32)RS_IMAGE);

error:
	return ret;
}

//for siw app
static ssize_t prd_show_app_op_end(struct device *dev, char *buf, int prev_mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	buf[0] = REPORT_END_RS_OK;
	if (prev_mode != APP_REPORT_OFF) {
		prd->prd_app_mode = APP_REPORT_OFF;
		ret = start_firmware(dev);
		if (ret < 0) {
			TOUCH_E("Invalid get_data request!\n");
			buf[0] = REPORT_END_RS_NG;
		}
	}

	return 1;
}

static int prd_show_prd_get_data_raw_core(struct device *dev,
					u8
					*buf, int size,
					u32 cmd, u32 offset, int flag)
{
	int ret = 0;

	if (cmd != DONT_USE_CMD) {
		ret = stop_firmware(dev, cmd);
		if (ret < 0) {
			goto out;
		}
	}

	ret = lg4894_reg_write(dev, tc_tsp_test_data_offset, (u8 *)&offset, sizeof(offset));
	if (ret < 0) {
		goto out;
	}

	ret = lg4894_reg_read(dev, tc_tsp_data_access_addr, (void *)buf, size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_raw_ait(struct device *dev, u8
*buf, int size, int flag)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_rawdata;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				CMD_RAWDATA, AIT_RAW_DATA_OFFSET, flag);
}

static int prd_show_prd_get_data_do_ait_basedata(struct device *dev,
					u8
*buf, int size, int step, int flag)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_rawdata;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				CMD_BASE_DATA, AIT_BASE_DATA_OFFSET, flag);
}


static int prd_show_prd_get_data_do_deltadata(struct device *dev, u8 *buf, int size, int flag)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int16_t *pbuf = (buf) ? (int16_t *)buf : prd->m2_buf_rawdata;
	int size_rd = (PRD_DELTA_SIZE<<1);
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_delta, size_rd,
				CMD_DELTADATA, DELTA_DATA_OFFSET, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < PRD_M2_ROW_COL_SIZE; i++){
		row = i / PRD_COL_SIZE;
		col = i % PRD_COL_SIZE;
		pbuf[i] = prd->buf_delta[(row + 1)*(PRD_COL_SIZE + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_labeldata(struct device *dev, u8 *buf, int size, int flag)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : prd->buf_label;
	int size_rd = PRD_LABEL_TMP_SIZE;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_label_tmp, size_rd,
				CMD_LABELDATA, LABLE_DATA_OFFSET, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < PRD_M2_ROW_COL_SIZE; i++){
		row = i / PRD_COL_SIZE;
		col = i % PRD_COL_SIZE;
		pbuf[i] = prd->buf_label_tmp[(row + 1)*(PRD_COL_SIZE + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_debug_buf(struct device *dev, u8 *buf, int size, int flag)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->buf_debug;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				CMD_DEBUGDATA, AIT_DEBUG_BUF_DATA_OFFSET, flag);
}

/*
static const char *prd_app_mode_str[] = {
	[APP_REPORT_OFF]		= "OFF",
	[APP_REPORT_RAW]		= "RAW",
	[APP_REPORT_BASE]		= "BASE",
	[APP_REPORT_LABEL]		= "LABEL",
	[APP_REPORT_DELTA]		= "DELTA",
	[APP_REPORT_DEBUG_BUF]	= "DEBUG_BUF",
};
*/
static ssize_t prd_show_app_operator(struct device *dev, char *buf, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (u8 *)prd->m2_buf_rawdata;
	int size = (PRD_M2_ROW_COL_SIZE<<1);
	int flag = PRD_SHOW_FLAG_DISABLE_PRT_RAW;
	int prev_mode = prd->prd_app_mode;

	//TOUCH_I("show app mode : %s(%d), 0x%X\n",
	//			prd_app_mode_str[mode], mode, flag);

	if (mode == APP_REPORT_OFF) {
		size = prd_show_app_op_end(dev, buf, prev_mode);
		goto out;
	}

	if (mode < APP_REPORT_MAX) {
		prd->prd_app_mode = mode;
	}

	switch (mode) {
	case APP_REPORT_RAW:
		prd_show_prd_get_data_do_raw_ait(dev, pbuf, size, flag);
		break;
	case APP_REPORT_BASE:
		prd_show_prd_get_data_do_ait_basedata(dev, pbuf, size, 0, flag);
		break;
	case APP_REPORT_DELTA:
		prd_show_prd_get_data_do_deltadata(dev, pbuf, size, flag);
		break;
	case APP_REPORT_LABEL:
		size = PRD_M2_ROW_COL_SIZE;
		pbuf = (u8 *)prd->buf_label,
		prd_show_prd_get_data_do_labeldata(dev, pbuf, size, flag);
		break;
	case APP_REPORT_DEBUG_BUF:
		size = PRD_DEBUG_BUF_SIZE;
		pbuf = (u8 *)prd->buf_debug,
		prd_show_prd_get_data_do_debug_buf(dev, pbuf, size, flag);
		//prd_show_prd_get_data_raw_core(dev, pbuf, PRD_DEBUG_BUF_SIZE,
		//		APP_IT_DONT_USE_CMD, AIT_DEBUG_BUF_DATA_OFFSET, flag);
		break;
	default:
		if (prev_mode != APP_REPORT_OFF) {
			prd_show_app_op_end(dev, buf, prev_mode);
		}
		size = 0;
		break;
	}

	if (size) {
		memcpy(buf, pbuf, size);
	}

out:
	return (ssize_t)size;
}

static ssize_t prd_show_app_raw(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_RAW);
}

static ssize_t prd_show_app_base(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_BASE);
}

static ssize_t prd_show_app_label(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_LABEL);
}

static ssize_t prd_show_app_delta(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_DELTA);
}

static ssize_t prd_show_app_debug_buf(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_DEBUG_BUF);
}

static ssize_t prd_show_app_end(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, APP_REPORT_OFF);
}

static ssize_t prd_show_app_info(struct device *dev, char *buf)
{
	u32 temp = PRD_SYS_ATTR_EN_FLAG;

	memset(buf, 0, PRD_APP_INFO_SIZE);

	buf[0] = (temp & 0xff);
	buf[1] = ((temp >> 8) & 0xff);
	buf[2] = ((temp >> 16) & 0xff);
	buf[3] = ((temp >> 24) & 0xff);

	buf[8] = PRD_ROW_SIZE;
	buf[9] = PRD_COL_SIZE;
	buf[10] = PRD_COL_ADD;
	buf[11] = PRD_CH;
	buf[12] = PRD_M1_COL;
	buf[13] = PRD_CMD_TYPE;
	buf[14] = SECOND_SCR_BOUND_I;
	buf[15] = SECOND_SCR_BOUND_J;

	TOUCH_I("<prd info> F:%08Xh \n",temp);
	TOUCH_I("R:%d C:%d C_A:%d CH:%d M1_C:%d \n	\
			CMD_T:%d S_SCR_I:%d S_SCR_J:%d \n",	\
		PRD_ROW_SIZE, PRD_COL_SIZE, PRD_COL_ADD, PRD_CH, PRD_M1_COL,	\
		PRD_CMD_TYPE, SECOND_SCR_BOUND_I, SECOND_SCR_BOUND_J);

	return PRD_APP_INFO_SIZE;
}

static struct siw_hal_prd_data *siw_hal_prd_alloc(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd;

	prd = devm_kzalloc(dev, sizeof(*prd), GFP_KERNEL);
	if (!prd) {
		TOUCH_E("failed to allocate memory for prd\n");
		goto out;
	}

	snprintf(prd->name, sizeof(prd->name)-1, "%s-prd", dev_name(dev));

	prd->dev = ts->dev;

	ts->prd = prd;

out:
	return prd;
}
//for siw app end

static ssize_t get_data(struct device *dev, int16_t *buf, u32 wdata)
{
	s16 *delta_buf = NULL;
	s8 *label_buf = NULL;
	u32 m2_data_offset;
	int i, row, col;
	int ret = 0;

	TOUCH_I("======== get_data(%d) ========\n", wdata);

	/***********************************************
	 * Match enum value between CMD_XXX and IT_XXX *
	 * when you call stop_firmware.                *
	 ***********************************************/
	if (stop_firmware(dev, wdata)) {
		TOUCH_E("fail to stop FW\n");
		ret = 1;
		goto getdata_error;
	}

	switch(wdata){
	case CMD_RAWDATA:
		m2_data_offset = AIT_RAW_DATA_OFFSET;
		lg4894_reg_write(dev, Serial_Data_Offset,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		lg4894_reg_read(dev, DATA_BASE_ADDR, (int16_t *)buf, sizeof(int16_t)*ROW_SIZE*COL_SIZE);
		break;
	case CMD_BASE_DATA:
		m2_data_offset = AIT_BASE_DATA_OFFSET;
		lg4894_reg_write(dev, Serial_Data_Offset,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		lg4894_reg_read(dev, DATA_BASE_ADDR, (int16_t *)buf, sizeof(int16_t)*ROW_SIZE*COL_SIZE);
		break;
	case CMD_DELTADATA:
		delta_buf = kzalloc(sizeof(u16) *
			((COL_SIZE+2) * (ROW_SIZE+2)), GFP_KERNEL);
		if (delta_buf == NULL) {
			TOUCH_E("delta_buf mem_error\n");
			ret = 1;
			goto getdata_error;
		}
		m2_data_offset = DELTA_DATA_OFFSET;
		lg4894_reg_write(dev, Serial_Data_Offset,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		lg4894_reg_read(dev, DATA_BASE_ADDR, (s16 *)delta_buf,
								sizeof(int16_t)*(ROW_SIZE+2)*(COL_SIZE+2));

		for(i = 0; i < ROW_SIZE*COL_SIZE; i++)
		{
			row = i / COL_SIZE;
			col = i % COL_SIZE;
			buf[i] = delta_buf[(row + 1)*(COL_SIZE + 2*1) + (col + 1)];
		}

		if(delta_buf)
			kfree(delta_buf);
		break;
	case CMD_LABELDATA:
		label_buf = kzalloc(sizeof(s8) * ((COL_SIZE+2) * (ROW_SIZE+2)), GFP_KERNEL);
		if (label_buf == NULL) {
			TOUCH_E("label_buf mem_error\n");
			ret = 1;
			goto getdata_error;
		}
		m2_data_offset = LABLE_DATA_OFFSET;
		lg4894_reg_write(dev, Serial_Data_Offset,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		lg4894_reg_read(dev, DATA_BASE_ADDR, (s8 *)label_buf,
								sizeof(s8)*(ROW_SIZE+2)*(COL_SIZE+2));

		for(i = 0; i < ROW_SIZE*COL_SIZE; i++)
		{
			row = i / COL_SIZE;
			col = i % COL_SIZE;
			buf[i] = (int16_t)label_buf[(row + 1)*(COL_SIZE + 2*1) + (col + 1)];
		}

		if(label_buf)
			kfree(label_buf);
		break;
	case CMD_DEBUGDATA:
		m2_data_offset = AIT_DEBUG_BUF_DATA_OFFSET;
		lg4894_reg_write(dev, Serial_Data_Offset,
							(u32 *)&m2_data_offset, sizeof(m2_data_offset));
		lg4894_reg_read(dev, DATA_BASE_ADDR, (int16_t *)buf,
							sizeof(int16_t)*DEBUG_ROW_SIZE*DEBUG_COL_SIZE);
		break;
	default:
		TOUCH_E("Invalid get_data request!\n");
	}

getdata_error:
	start_firmware(dev);

	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	int ret = 0;
	int ret2 = 0;
	int16_t *delta = NULL;
	int i = 0;
	int j = 0;

	delta = kzalloc(sizeof(int16_t) * (COL_SIZE) * (ROW_SIZE), GFP_KERNEL);

	if (delta == NULL) {
		TOUCH_E("delta mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== Deltadata ========\n");

	ret2 = get_data(dev, delta, CMD_DELTADATA);  /* 2 == deltadata */
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret, "[%2d]  ", i);

		for (j = 0 ; j < COL_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", delta[i * COL_SIZE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ", delta[i * COL_SIZE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (delta != NULL)
		kfree(delta);

	return ret;
}

static ssize_t show_fdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int ret = 0;
	int ret2 = 0;
	u8 type = U3_M2_RAWDATA_TEST;

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
		/*
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_gpio_direction_output(ts->reset_pin, 1);
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
		*/
		lg4894_reset_ctrl(dev, HW_RESET_SYNC);
		return ret;
	}

	prd_read_rawdata(dev, type);
	ret = prd_print_rawdata(dev, buf, type);

	/*
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(ts->caps.hw_reset_delay);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	*/
	lg4894_reset_ctrl(dev, HW_RESET_SYNC);
	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int i = 0;
	int j = 0;

	rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (rawdata == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== rawdata ========\n");

	ret2 = get_data(dev, rawdata, CMD_RAWDATA);  /* 2 == deltadata */
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < ROW_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"[%2d]  ", i);

		for (j = 0 ; j < COL_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%5d ",
				rawdata[i * COL_SIZE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ",
				rawdata[i * COL_SIZE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct lg4894_data *d = to_lg4894_data(dev);
	int m1_rawdata_ret = 1;
	int m2_rawdata_ret = 1;
	int jitter_ret = 1;
	int ret = 0;

	/* file create , time log */
	write_file(dev, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	write_file(dev, "\n", TIME_INFO_WRITE);
	TOUCH_I("Show_lpwg_sd Test Start\n");

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U0) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"LCD mode is not U0. Test Result : Fail\n");
		return ret;
	}

	firmware_version_log(dev);
	ic_run_info_print(dev);

	mutex_lock(&ts->lock);
	write_test_control(ts->dev, cmd_test_enter);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	lg4894_tc_driving(dev, LCD_MODE_STOP);

	m1_rawdata_ret = prd_rawdata_test(dev, U0_M1_RAWDATA_TEST);
	m2_rawdata_ret = prd_rawdata_test(dev, U0_M2_RAWDATA_TEST);

	/*
		U0_BLU_JITTER_TEST
		BLU Jitter - pass : 0, fail : 1
		This will be enabled later.
	*/
	jitter_ret = prd_rawdata_test(dev, U0_JITTER_TEST);

	ret = snprintf(buf + ret, PAGE_SIZE, "========RESULT=======\n");

	if (!m1_rawdata_ret && !m2_rawdata_ret && !jitter_ret) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : %s\n", "Pass");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"LPWG RawData : %s (raw_M2:%d/raw_M1:%d/jitter:%d)\n", "Fail",
			m2_rawdata_ret ? 0 : 1, m1_rawdata_ret ? 0 : 1, jitter_ret ? 0 : 1);
	}

	write_file(dev, buf, TIME_INFO_SKIP);
	write_test_control(ts->dev, cmd_test_exit);
/*
	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_gpio_direction_output(ts->reset_pin, 1);
	touch_msleep(ts->caps.hw_reset_delay);
	ts->driver->init(dev);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	*/
	lg4894_reset_ctrl(dev, HW_RESET_SYNC);
	mutex_unlock(&ts->lock);

	write_file(dev, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	log_file_size_check(dev);
	TOUCH_I("Show_lpwg_sd Test End\n");

	return ret;
}

static ssize_t show_labeldata(struct device *dev, char *buf)
{
	int ret = 0;
	int ret2 = 0;
	s8 *label = NULL;
	int i = 0;
	int j = 0;

	label = kzalloc(sizeof(s16) * (COL_SIZE+2) * (ROW_SIZE+2), GFP_KERNEL);

	if (label == NULL) {
		TOUCH_E("label mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== LabelData ========\n");

	ret2 = get_data(dev, (int16_t*)label, CMD_LABELDATA);  /* 2 == labeldata */
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < ROW_SIZE; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret, "[%2d]  ", i);

		for (j = 0 ; j < COL_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", label[i*COL_SIZE*2 + j*2]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ", label[i*COL_SIZE*2 + j*2]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (label != NULL)
		kfree(label);

	return ret;
}

static ssize_t show_debug(struct device *dev, char *buf)
{
	int ret = 0;
	int ret2 = 0;
	int16_t *debugdata = NULL;
	int i = 0;
	int j = 0;

	debugdata = kzalloc(sizeof(int16_t) * (DEBUG_ROW_SIZE*DEBUG_COL_SIZE), GFP_KERNEL);

	if (debugdata == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== DebugData ========\n");

	ret2 = get_data(dev, debugdata, CMD_DEBUGDATA);
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < DEBUG_ROW_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"[%2d]	", i);

		for (j = 0 ; j < DEBUG_COL_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%5d ",
				debugdata[i * DEBUG_COL_SIZE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ",
				debugdata[i * DEBUG_COL_SIZE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (debugdata != NULL)
		kfree(debugdata);

	return ret;
}

static ssize_t show_base(struct device *dev, char *buf)
{
	int ret = 0;
	int ret2 = 0;
	int16_t *baseline = NULL;
	int i = 0;
	int j = 0;

	baseline = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (baseline == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== rawdata ========\n");

	ret2 = get_data(dev, baseline, CMD_BASE_DATA);
	if (ret2 == 1) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < ROW_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"[%2d]  ", i);

		for (j = 0 ; j < COL_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%5d ",
				baseline[i * COL_SIZE + j]);
			log_ret += snprintf(log_buf + log_ret,
				LOG_BUF_SIZE - log_ret,
				"%5d ",
				baseline[i * COL_SIZE + j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (baseline != NULL)
		kfree(baseline);

	return ret;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(fdata, show_fdata, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(label, show_labeldata, NULL);
static TOUCH_ATTR(debug, show_debug, NULL);
static TOUCH_ATTR(base, show_base, NULL);

//for siw app
static TOUCH_ATTR(prd_app_raw, prd_show_app_raw, NULL);
static TOUCH_ATTR(prd_app_base, prd_show_app_base, NULL);
static TOUCH_ATTR(prd_app_label, prd_show_app_label, NULL);
static TOUCH_ATTR(prd_app_delta, prd_show_app_delta, NULL);
static TOUCH_ATTR(prd_app_debug_buf, prd_show_app_debug_buf, NULL);
static TOUCH_ATTR(prd_app_end, prd_show_app_end, NULL);
static TOUCH_ATTR(prd_app_info, prd_show_app_info, NULL);
//for siw app end

static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_fdata.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_label.attr,
	&touch_attr_debug.attr,
	&touch_attr_base.attr,

	//for siw app
	&touch_attr_prd_app_raw.attr,
	&touch_attr_prd_app_base.attr,
	&touch_attr_prd_app_label.attr,
	&touch_attr_prd_app_delta.attr,
	&touch_attr_prd_app_debug_buf.attr,
	&touch_attr_prd_app_end.attr,
	&touch_attr_prd_app_info.attr,
	//for siw app end
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int lg4894_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	TOUCH_TRACE();

	//for siw app
	prd = siw_hal_prd_alloc(dev);
	if (!prd) {
		ret = -ENOMEM;
		return ret;
	}
	//for siw app end

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
