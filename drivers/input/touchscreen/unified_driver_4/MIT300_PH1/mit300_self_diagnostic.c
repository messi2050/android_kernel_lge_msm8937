/***************************************************************************
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
 *    File  	: lgtp_mit300_self_diagnostic.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[MIT_SELFD]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>
#include <linux/input/unified_driver_4/lgtp_common_driver.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_i2c.h>
#include <linux/input/unified_driver_4/lgtp_device_mit300_ph1.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
/*Use Test Register or Image Register*/
#define TESTREG 0
#define IMGREG 1

/* Self-Diagnosis Limitation Value */
/* CUT 7*/
#define C7_RAW_DATA_MAX 3000
#define C7_RAW_DATA_MIN 1000
#define C7_RAW_DATA_MARGIN 0
#define C7_CM_DELTA_MIN 55
#define C7_JITTER_MAX 34
#define C7_JITTER_MIN 0
#define C7_OPEN_SHORT_MAX 3600
#define C7_OPEN_SHORT_MIN 800
#define C7_MUX_SHORT_MAX 3600
#define C7_MUX_SHORT_MIN 800
#define C7_LPWG_RAW_DATA_MAX 3000
#define C7_LPWG_RAW_DATA_MIN 1000
#define C7_LPWG_JITTER_MAX 23
#define C7_LPWG_JITTER_MIN 0
#define C7_AVG_JITTER_MAX 23

/* CUT 6*/
#define C6_RAW_DATA_MAX 3000
#define C6_RAW_DATA_MIN 1400
#define C6_RAW_DATA_MARGIN 0
#define C6_CM_DELTA_MIN 55
#define C6_JITTER_MAX 34
#define C6_JITTER_MIN 0
#define C6_OPEN_SHORT_MAX 3500
#define C6_OPEN_SHORT_MIN 850
#define C6_MUX_SHORT_MAX 3500
#define C6_MUX_SHORT_MIN 850
#define C6_LPWG_RAW_DATA_MAX 2950
#define C6_LPWG_RAW_DATA_MIN 1050
#define C6_LPWG_JITTER_MAX 23
#define C6_LPWG_JITTER_MIN 0
#define C6_AVG_JITTER_MAX 26

/* CUT 5*/
#define C5_RAW_DATA_MAX 3000
#define C5_RAW_DATA_MIN 750
#define C5_RAW_DATA_MARGIN 0
#define C5_CM_DELTA_MIN 55
#define C5_JITTER_MAX 32
#define C5_JITTER_MIN 0
#define C5_OPEN_SHORT_MAX 3500
#define C5_OPEN_SHORT_MIN 1500
#define C5_MUX_SHORT_MAX 3500
#define C5_MUX_SHORT_MIN 1500
#define C5_LPWG_RAW_DATA_MAX 3000
#define C5_LPWG_RAW_DATA_MIN 1000
#define C5_LPWG_JITTER_MAX 23
#define C5_LPWG_JITTER_MIN 0
#define C5_AVG_JITTER_MAX 26

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/
struct mit300_data_format{
	u8 row_num;
	u8 col_num;
	u8 buffer_col_num;
	u8 rotate;
	u8 key_num;
	u8 data_type;
	u8 data_type_size;
	u8 data_type_sign;
};
struct mit300_buf_addr{
	u8 buf_addr_h;
	u8 buf_addr_l;
};

/****************************************************************************
* Variables
****************************************************************************/
bool test_busy = false;
int bootmode;
uint16_t mit_data[MAX_ROW][MAX_COL];
s16 intensity_data[MAX_ROW][MAX_COL];

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
static int MIT300_GetReadyStatus(struct i2c_client *client)
{
	u8 wbuf[16];
	u8 rbuf[16] = {0};
	int ret = 0;

	//TOUCH_FUNC();

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if(Mit300_I2C_Read(client, wbuf, 2, rbuf, 1) < 0) {
		TOUCH_ERR("[ERROR] mip_i2c_read\n");
		goto ERROR;
	}
	ret = rbuf[0];

	//check status
	if((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) || (ret == MIP_CTRL_STATUS_READY)){
		if (ret == 0xA0)
			TOUCH_LOG("status [0x%02X]\n", ret);
	}
	else{
		TOUCH_ERR("[ERROR] Unknown status [0x%02X]\n", ret);
		goto ERROR;
	}

	if(ret == MIP_CTRL_STATUS_LOG){
		//skip log event
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if( Mit300_I2C_Write(client, wbuf, 3) < 0 ){
			TOUCH_ERR("[ERROR] mip_i2c_write\n");
		}
	}

	//TOUCH_LOG("%s [DONE]\n", __func__);
	return ret;

ERROR:
	TOUCH_ERR("%s [ERROR]\n", __func__);
	return -1;
}

static int MIT300_GetDataFormat(struct i2c_client *client, struct mit300_data_format *data, int reg_type)
{
	u8 wbuf[2];
	u8 rbuf[10] = {0,};
	TOUCH_FUNC();

	if (reg_type == TESTREG){
		wbuf[0] = MIP_R0_TEST;
		wbuf[1] = MIP_R1_TEST_DATA_FORMAT;
		if(Mit300_I2C_Read(client, wbuf, 2, rbuf, 6)<0){
			TOUCH_ERR("%s [ERROR] Read data format\n", __func__);
			return TOUCH_FAIL;
		}
	} else if (reg_type == IMGREG) {
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_DATA_FORMAT;
		if(Mit300_I2C_Read(client, wbuf, 2, rbuf, 6)<0){
			TOUCH_ERR("%s [ERROR] Read data format\n", __func__);
			return TOUCH_FAIL;
		}
	}

	data->row_num = rbuf[0];
	data->col_num = rbuf[1];
	data->buffer_col_num = rbuf[2];
	data->rotate = rbuf[3];
	data->key_num = rbuf[4];
	data->data_type = rbuf[5];

	data->data_type_sign = (data->data_type & 0x80) >> 7;
	data->data_type_size = data->data_type & 0x7F;

	TOUCH_LOG("row_num[%d] col_num[%d] buffer_col_num[%d] rotate[%d] key_num[%d]\n", data->row_num, data->col_num, data->buffer_col_num, data->rotate, data->key_num);
	TOUCH_LOG("data_type[0x%02X] data_sign[%d] data_size[%d]\n", data->data_type, data->data_type_sign, data->data_type_size);

	return TOUCH_SUCCESS;
}

static int MIT300_GetBufAddr(struct i2c_client *client, struct mit300_buf_addr *buf_addr, int reg_type)
{
	u8 wbuf[2];
	u8 rbuf[10] = {0};
	TOUCH_FUNC();

	if (reg_type == TESTREG){
		wbuf[0] = MIP_R0_TEST;
		wbuf[1] = MIP_R1_TEST_BUF_ADDR;
		if(Mit300_I2C_Read(client, wbuf, 2,rbuf, 2)<0){
			TOUCH_ERR("%s [ERROR] Read buf addr\n", __func__);
			return TOUCH_FAIL;
		}
	} else if (reg_type == IMGREG) {
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_BUF_ADDR;
		if(Mit300_I2C_Read(client, wbuf, 2,rbuf, 2)<0){
			TOUCH_ERR("%s [ERROR] Read buf addr\n", __func__);
			return TOUCH_FAIL;
		}
	}

	buf_addr->buf_addr_l = rbuf[0];
	buf_addr->buf_addr_h = rbuf[1];
	TOUCH_LOG("buf_addr[0x%02X 0x%02X]\n", buf_addr->buf_addr_l, buf_addr->buf_addr_h);

	return TOUCH_SUCCESS;
}


static int MIT300_ReadRMIBuffer(struct i2c_client *client, struct mit300_data_format *data, struct mit300_buf_addr *buf_addr)
{

	u8 size = 0;
	int i_col, i_row;
	int lim_col, lim_row;
	int max_x = 0;
	int max_y = 0;
	bool flip_x = false;
	int sValue = 0;
	unsigned int uValue = 0;
	int value = 0;
	u8 wbuf[8] = {0};
	u8 rbuf[512] = {0};
	unsigned int addr;
	int offset;
	int has_key = 0;
	TOUCH_FUNC();

	//set axis
	if(data->rotate == 0){
		max_x = data->col_num;
		max_y = data->row_num;
		if(data->key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = false;
	}
	else if(data->rotate == 1){
		max_x = data->row_num;
		max_y = data->col_num;
		if(data->key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = true;
	}
	else{
		TOUCH_ERR("[ERROR] rotate [%d]\n", data->rotate);
		goto ERROR;
	}
	//get table data
	lim_row = data->row_num + has_key;
	for(i_row = 0; i_row < lim_row; i_row++){
		//get line data
		offset = data->buffer_col_num * data->data_type_size;
		size = data->col_num * data->data_type_size;

		addr = (buf_addr->buf_addr_h << 8) | buf_addr->buf_addr_l | (offset * i_row);
		wbuf[0] = (addr >> 8) & 0xFF;
		wbuf[1] = addr & 0xFF;
		if(Mit300_I2C_Read(client, wbuf, 2, rbuf, size)){
			TOUCH_ERR("[ERROR] Read data buffer\n");
			goto ERROR;
		}

		//save data
		if((data->key_num > 0) && (i_row == (lim_row - 1))){
			lim_col = data->key_num;
		}
		else{
			lim_col = data->col_num;
		}
		for(i_col = 0; i_col < lim_col; i_col++){
			if(data->data_type_sign == 0){
				//unsigned
				if(data->data_type_size == 1){
					uValue = (u8)rbuf[i_col];
				}
				else if(data->data_type_size == 2){
					uValue = (u16)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8));
				}
				else if(data->data_type_size == 4){
					uValue = (u32)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8)
					| (rbuf[data->data_type_size * i_col + 2] << 16)
					| (rbuf[data->data_type_size * i_col + 3] << 24));
				}
				else{
					TOUCH_ERR("[ERROR] data_size [%d]\n", data->data_type_size);
					goto ERROR;
				}
				value = (int)uValue;
			}
			else{
				//signed
				if(data->data_type_size == 1){
					sValue = (s8)rbuf[i_col];
				}
				else if(data->data_type_size == 2){
					sValue = (s16)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8));
				}
				else if(data->data_type_size == 4){
					sValue = (s32)(rbuf[data->data_type_size * i_col]
					| (rbuf[data->data_type_size * i_col + 1] << 8)
					| (rbuf[data->data_type_size * i_col + 2] << 16)
					| (rbuf[data->data_type_size * i_col + 3] << 24));
				}
				else{
					TOUCH_ERR("[ERROR] data_size [%d]\n", data->data_type_size);
					goto ERROR;
				}
				value = (int)sValue;
			}

			switch(data->rotate){
				case 0:
					mit_data[i_row][i_col] = value;
					intensity_data[i_row][i_col] = value;
					break;
				case 1:
					if((data->key_num > 0) && (i_row == (lim_row - 1))){
						mit_data[i_row][i_col] = value;
						intensity_data[i_row][i_col] = value;
					}
					else{
						mit_data[i_col][i_row] = value;
						intensity_data[i_col][i_row] = value;
					}
					break;
				default:
					TOUCH_ERR("[ERROR] rotate [%d]\n", data->rotate);
					goto ERROR;
					break;
			}
		}
	}

	TOUCH_LOG("%s [DONE]\n", __func__);
	return TOUCH_SUCCESS;

ERROR:

	TOUCH_ERR("%s [ERROR]\n", __func__);
	return TOUCH_FAIL;
}

static int  MIT300_PrintData(struct i2c_client *client, char *buf, int* result, int type)
{
	int col = 0;
	int row = 0;
	int ret = 0;
	int min_data = 0;
	int max_data = 0;
	int limit_upper = 0;
	int limit_lower = 0;
	int jitter_avg_1 = 0;
	int jitter_avg_2 = 0;
	int avg_max_1 = 0;
	int avg_max_2 = 0;
	int avg_jitter_max = 0;
	int error_count = 0;

	TOUCH_FUNC();

#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		TOUCH_LOG("DDIC Revision Status : Cut.%d\n", lge_get_db7400_cut());
#endif

	switch (type) {
	case RAW_DATA_SHOW:
		TOUCH_LOG("[Rawdata Result]\n");
		ret += sprintf(buf + ret,"[DDIC Revision Status : Cut.%d]\n\n", lge_get_db7400_cut());
		ret += sprintf(buf + ret,"[Rawdata Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_RAW_DATA_MAX + C7_RAW_DATA_MARGIN;
			limit_lower = C7_RAW_DATA_MIN - C7_RAW_DATA_MARGIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_RAW_DATA_MAX + C6_RAW_DATA_MARGIN;
			limit_lower = C6_RAW_DATA_MIN - C6_RAW_DATA_MARGIN;
		} else {
			limit_upper = C5_RAW_DATA_MAX + C5_RAW_DATA_MARGIN;
			limit_lower = C5_RAW_DATA_MIN - C5_RAW_DATA_MARGIN;
		}
#endif
		break;
	case RAW_DATA_STORE:
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_RAW_DATA_MAX + C7_RAW_DATA_MARGIN;
			limit_lower = C7_RAW_DATA_MIN - C7_RAW_DATA_MARGIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_RAW_DATA_MAX + C6_RAW_DATA_MARGIN;
			limit_lower = C6_RAW_DATA_MIN - C6_RAW_DATA_MARGIN;
		} else {
			limit_upper = C5_RAW_DATA_MAX + C5_RAW_DATA_MARGIN;
			limit_lower = C5_RAW_DATA_MIN - C5_RAW_DATA_MARGIN;
		}
#endif
		break;
	case OPENSHORT_SHOW:
		TOUCH_LOG("[Openshort Result]\n");
		ret += sprintf(buf + ret,"[Openshort Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_OPEN_SHORT_MAX;
			limit_lower = C7_OPEN_SHORT_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_OPEN_SHORT_MAX;
			limit_lower = C6_OPEN_SHORT_MIN;
		} else {
			limit_upper = C5_OPEN_SHORT_MAX;
			limit_lower = C5_OPEN_SHORT_MIN;
		}
#endif
		break;
	case OPENSHORT_STORE:
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_OPEN_SHORT_MAX;
			limit_lower = C7_OPEN_SHORT_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_OPEN_SHORT_MAX;
			limit_lower = C6_OPEN_SHORT_MIN;
		} else {
			limit_upper = C5_OPEN_SHORT_MAX;
			limit_lower = C5_OPEN_SHORT_MIN;
		}
#endif
		break;
	case MUXSHORT_SHOW:
		TOUCH_LOG("[MUXSHORT Result]\n");
		ret += sprintf(buf + ret,"[MUXSHORT Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_MUX_SHORT_MAX;
			limit_lower = C7_MUX_SHORT_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_MUX_SHORT_MAX;
			limit_lower = C6_MUX_SHORT_MIN;
		} else {
			limit_upper = C5_MUX_SHORT_MAX;
			limit_lower = C5_MUX_SHORT_MIN;
		}
#endif
		break;
	case CM_DELTA_SHOW:
		TOUCH_LOG("[Delta Result]\n");
		ret += sprintf(buf + ret,"[Delta Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_lower = C7_CM_DELTA_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_lower = C6_CM_DELTA_MIN;
		} else {
			limit_lower = C5_CM_DELTA_MIN;
		}
		break;
#endif
	case CM_DELTA_STORE:
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_lower = C7_CM_DELTA_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_lower = C6_CM_DELTA_MIN;
		} else {
			limit_lower = C5_CM_DELTA_MIN;
		}
#endif
		break;
	case CM_JITTER_SHOW:
		TOUCH_LOG("[Jitter Result]\n");
		ret += sprintf(buf + ret,"[Jitter Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_JITTER_MAX;
			limit_lower = C7_JITTER_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_JITTER_MAX;
			limit_lower = C6_JITTER_MIN;
		} else {
			limit_upper = C5_JITTER_MAX;
			limit_lower = C5_JITTER_MIN;
		}
#endif
		break;
	case CM_JITTER_STORE:
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_JITTER_MAX;
			limit_lower = C7_JITTER_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_JITTER_MAX;
			limit_lower = C6_JITTER_MIN;
		} else {
			limit_upper = C5_JITTER_MAX;
			limit_lower = C5_JITTER_MIN;
		}
#endif
		break;
    case LPWG_JITTER_SHOW:
		TOUCH_LOG("[LPWG Jitter Result]\n");
		ret += sprintf(buf + ret,"[LPWG Jitter Result]\n");
		min_data = mit_data[0][0];
		max_data = mit_data[0][0];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_LPWG_JITTER_MAX;
			limit_lower = C7_LPWG_JITTER_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_LPWG_JITTER_MAX;
			limit_lower = C6_LPWG_JITTER_MIN;
		} else {
			limit_upper = C5_LPWG_JITTER_MAX;
			limit_lower = C5_LPWG_JITTER_MIN;
		}
#endif
		break;
    case LPWG_ABS_SHOW:
		TOUCH_LOG("[LPWG ABS Result]\n");
		ret += sprintf(buf + ret,"[LPWG ABS Result]\n");
		min_data = mit_data[1][1];
		max_data = mit_data[1][1];
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
		if (lge_get_db7400_cut() >= CUT7) {
			limit_upper = C7_LPWG_RAW_DATA_MAX;
			limit_lower = C7_LPWG_RAW_DATA_MIN;
		} else if(lge_get_db7400_cut() == CUT6) {
			limit_upper = C6_LPWG_RAW_DATA_MAX;
			limit_lower = C6_LPWG_RAW_DATA_MIN;
		} else {
			limit_upper = C5_LPWG_RAW_DATA_MAX;
			limit_lower = C5_LPWG_RAW_DATA_MIN;
		}
#endif
		break;
	case INTENSITY_SHOW:
		TOUCH_LOG("[Intensity Result]\n");
		ret += sprintf(buf + ret,"[Intensity Result]\n");
		break;
	default:
		return TOUCH_FAIL;
	}

	/* LPWG ABS Inspection of except for both ends of column(0,18) */
	if (type == LPWG_ABS_SHOW) {
		for (row = 0 ; row < MAX_ROW ; row++) {
				ret += sprintf(buf + ret,"[%2d]  ", row);

			for (col = 0 ; col < MAX_COL ; col++) {
				if ((col == 0) || (col == MAX_COL-1)) {
					ret += sprintf(buf + ret," XXX ");
				} else {
					if ((mit_data[row][col] >= limit_lower) && (mit_data[row][col] <= limit_upper)) {
						ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
					} else {
						ret += sprintf(buf + ret,"!%4d ", mit_data[row][col]);
						error_count++;
						*result = TOUCH_FAIL;
					}
				}
				if (col > 0 && col < MAX_COL-1) {
					min_data = (min_data > mit_data[row][col]) ? mit_data[row][col] : min_data;
					max_data = (max_data < mit_data[row][col]) ? mit_data[row][col] : max_data;
				}
			}
		ret += sprintf(buf + ret,"\n");
		}
	} else if (type == LPWG_JITTER_SHOW) {  /* Inspection for 10 column areas(0~9) in LPWG Jitter Test */
		for (row = 0 ; row < MAX_ROW ; row++) {
				ret += sprintf(buf + ret,"[%2d]  ", row);

			for (col = 0 ; col < MAX_COL ; col++) {
				if (col >= MAX_COL-8) {
					ret += sprintf(buf + ret," XXX ");
				} else {
					if ((mit_data[row][col] >= limit_lower) && (mit_data[row][col] <= limit_upper)) {
						ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
					} else {
						ret += sprintf(buf + ret,"!%4d ", mit_data[row][col]);
						error_count++;
						*result = TOUCH_FAIL;
					}
				}
				if (col < MAX_COL-8) {
					min_data = (min_data > mit_data[row][col]) ? mit_data[row][col] : min_data;
					max_data = (max_data < mit_data[row][col]) ? mit_data[row][col] : max_data;
				}
			}
		ret += sprintf(buf + ret,"\n");
		}
	} else { /* Another Inspection */
		for (row = 0 ; row < MAX_ROW ; row++) {
			if (type == RAW_DATA_SHOW ||
				type == OPENSHORT_SHOW ||
				type == MUXSHORT_SHOW ||
				type == CM_DELTA_SHOW ||
				type == CM_JITTER_SHOW ||
				type == INTENSITY_SHOW) {

				ret += sprintf(buf + ret,"[%2d]  ", row);
			}

			for (col = 0 ; col < MAX_COL ; col++) {
				if (type != INTENSITY_SHOW) {

					/* Do not check limit_upper value in CM Delta Test */
					if (type == CM_DELTA_SHOW || type == CM_DELTA_STORE) {
						if (mit_data[row][col] >= limit_lower) {
							//TOUCH_LOG("%5d ", mit_data[row][col]);
							ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
						} else {
							ret += sprintf(buf + ret,"!%4d ", mit_data[row][col]);
							error_count++;
							*result = TOUCH_FAIL;
						}
					} else {
						if (mit_data[row][col] >= limit_lower && mit_data[row][col] <= limit_upper) {
							//TOUCH_LOG("%5d ", mit_data[row][col]);
							ret += sprintf(buf + ret,"%5d ", mit_data[row][col]);
						} else {
							ret += sprintf(buf + ret,"!%4d ", mit_data[row][col]);
							error_count++;
							*result = TOUCH_FAIL;
						}
					}

					if (type == RAW_DATA_STORE ||
						type == OPENSHORT_STORE ||
						type == CM_DELTA_STORE ||
						type == CM_JITTER_STORE) {
							ret += sprintf(buf + ret,",");
					}

					/* Do not check limit_upper value in CM Delta Test */
					if (type == CM_DELTA_SHOW || type == CM_DELTA_STORE) {
						min_data = (min_data > mit_data[row][col]) ? mit_data[row][col] : min_data;
					} else {
						min_data = (min_data > mit_data[row][col]) ? mit_data[row][col] : min_data;
						max_data = (max_data < mit_data[row][col]) ? mit_data[row][col] : max_data;
					}

					if (type == CM_JITTER_SHOW || type == CM_JITTER_STORE) {
						if (col < MAX_COL / 2)
							jitter_avg_1 += mit_data[row][col];
						else
							jitter_avg_2 += mit_data[row][col];
					}
				} else if (type == INTENSITY_SHOW){
					//TOUCH_LOG("%4d ", intensity_data[row][col]);
					/* 17 : MAX_COL - 1 (Because of invert left and right side) */
					ret += sprintf(buf + ret,"%4d ", intensity_data[row][17-col]);
				}
			}

			//TOUCH_LOG("\n");
			if (type == CM_JITTER_SHOW) {
#if defined(CONFIG_LGD_PH1DONGBU_INCELL_VIDEO_HD_PANEL)
				if (lge_get_db7400_cut() >= CUT7) {
					avg_jitter_max = C7_AVG_JITTER_MAX;
				} else if(lge_get_db7400_cut() == CUT6) {
					avg_jitter_max = C6_AVG_JITTER_MAX;
				} else {
					avg_jitter_max = C5_AVG_JITTER_MAX;
				}
#endif
				jitter_avg_1 = jitter_avg_1 / (MAX_COL / 2);
				jitter_avg_2 = jitter_avg_2 / (MAX_COL / 2);

				if (jitter_avg_1 > avg_jitter_max) {
					//TOUCH_LOG(" L: %d ", jitter_avg_1);
					error_count++;
					*result = TOUCH_FAIL;
				}
				if (jitter_avg_2 > avg_jitter_max) {
					//TOUCH_LOG(" R: %d ", jitter_avg_1);
					error_count++;
					*result = TOUCH_FAIL;
				}

				avg_max_1 = (avg_max_1 < jitter_avg_1) ? jitter_avg_1 : avg_max_1;
				avg_max_2 = (avg_max_2 < jitter_avg_2) ? jitter_avg_2 : avg_max_2;

				ret += sprintf(buf + ret,"\n");
			} else if (type == RAW_DATA_SHOW ||
				type == OPENSHORT_SHOW ||
				type == MUXSHORT_SHOW ||
				type == CM_DELTA_SHOW ||
				type == INTENSITY_SHOW) {

				ret += sprintf(buf + ret,"\n");
			}
		}
	}

	if (*result == TOUCH_FAIL){
		TOUCH_LOG("=======Type(%d) Result is FAIL=======\n", type);
		ret += sprintf(buf + ret, "Type(%d) Result is FAIL\n", type);
		ret += sprintf(buf + ret, "This is %d error in test with '!' character\n", error_count);
	}

	if (type == RAW_DATA_SHOW ||
		type == OPENSHORT_SHOW ||
		type == MUXSHORT_SHOW ||
		type == CM_DELTA_SHOW ||
		type == CM_JITTER_SHOW ||
		type == LPWG_JITTER_SHOW ||
		type == LPWG_ABS_SHOW ||
		type == INTENSITY_SHOW) {

		if (*result != TOUCH_FAIL) {
			TOUCH_LOG("=======Type(%d) Result is SUCCESS=======\n", type);
			ret += sprintf(buf + ret, "Type(%d) Result is SUCCESS\n", type);
		}

		/* Do not check limit_upper value in CM Delta Test */
		if (type == CM_DELTA_SHOW) {
			//TOUCH_LOG("MIN : %d, Spec lower : %d\n\n", min_data, limit_lower);
			ret += sprintf(buf + ret,"MIN = %d, Spec lower : %d\n\n", min_data, limit_lower);
		} else {
			//TOUCH_LOG("MAX : %d,  MIN : %d  (MAX - MIN = %d), Spec lower : %d, upper : %d\n\n", max_data , min_data, max_data - min_data, limit_lower, limit_upper);
			ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d), Spec lower : %d, upper : %d\n\n",max_data, min_data, max_data - min_data, limit_lower, limit_upper);
			if (type == CM_JITTER_SHOW) {
				avg_max_1 = (avg_max_1 < avg_max_2) ? avg_max_2 : avg_max_1;
				//TOUCH_LOG("AVERAGE JITTER MAX = %d\n\n",avg_max_1);
				ret += sprintf(buf + ret,"AVERAGE JITTER MAX = %d\n\n", avg_max_1);
			}
		}
	}
	return ret;
}

int MIT300_DoTest(struct i2c_client *client, int type){
	int busy_cnt = 50;
	int wait_cnt = 50;
	u8 wbuf[8];
	u8 test_type = 0;
	int reg_type = 0;
	struct mit300_data_format *data_form = NULL;
	struct mit300_buf_addr *buf_addr = NULL;

	/* Allocate memory */
    data_form = kzalloc(sizeof(struct mit300_data_format), GFP_KERNEL);
	if (data_form == NULL) {
		TOUCH_ERR("read_buf mem_error\n");
		goto ERROR;
	}
	buf_addr = kzalloc(sizeof(struct mit300_buf_addr), GFP_KERNEL);
	if (buf_addr == NULL) {
		TOUCH_ERR("read_buf mem_error\n");
		goto ERROR;
        }

	TOUCH_FUNC();

	/* Check test type */

	switch(type){
		case RAW_DATA_SHOW:
			TOUCH_LOG("=== Rawdata image ===\n");
			test_type = MIP_IMG_TYPE_RAWDATA;
			reg_type = IMGREG;
			break;
		case RAW_DATA_STORE:
			TOUCH_LOG("=== Rawdata image ===\n");
			test_type = MIP_IMG_TYPE_RAWDATA;
			reg_type = IMGREG;
			break;
		case INTENSITY_SHOW:
			TOUCH_LOG("=== Intensity image ===\n");
			test_type = MIP_IMG_TYPE_INTENSITY;
			reg_type = IMGREG;
			break;
		case ABS_SHOW:
		case LPWG_ABS_SHOW:
			TOUCH_LOG("=== ABS test ===\n");
			test_type = MIP_TEST_TYPE_CM_ABS;
			reg_type = TESTREG;
			break;
		case CM_DELTA_SHOW:
			TOUCH_LOG("=== Delta test ===\n");
			test_type = MIP_TEST_TYPE_CM_DELTA;
			reg_type = TESTREG;
			break;
		case CM_DELTA_STORE:
			TOUCH_LOG("=== Delta test ===\n");
			test_type = MIP_TEST_TYPE_CM_DELTA;
			reg_type = TESTREG;
			break;
		case CM_JITTER_SHOW:
		case LPWG_JITTER_SHOW:
			TOUCH_LOG("=== Jitter test ===\n");
			test_type = MIP_TEST_TYPE_CM_JITTER;
			reg_type = TESTREG;
			break;
		case CM_JITTER_STORE:
			TOUCH_LOG("=== Jitter test ===\n");
			test_type = MIP_TEST_TYPE_CM_JITTER;
			reg_type = TESTREG;
			break;
		case OPENSHORT_SHOW:
			TOUCH_LOG("=== OPEN Short test ===\n");
			test_type = MIP_TEST_TYPE_SHORT;
			reg_type = TESTREG;
			break;
		case OPENSHORT_STORE:
			TOUCH_LOG("=== OPEN Short test ===\n");
			test_type = MIP_TEST_TYPE_SHORT;
			reg_type = TESTREG;
			break;
		case MUXSHORT_SHOW:
			TOUCH_LOG("=== MUX Short test ===\n");
			test_type = MIP_TEST_TYPE_SHORT2;
			reg_type = TESTREG;
			break;
		default:
			TOUCH_LOG("type select error, type : %d", type);
			goto ERROR;
	}

	TOUCH_LOG("reg_type = %s, test type = %d\n", (reg_type==TESTREG)?"Test Register":"Image Register", test_type);

	/* Check busy status */
	while(busy_cnt--){
		if(test_busy == false){
			break;
		}
		TOUCH_LOG("busy_cnt = %d\n", busy_cnt);
		msleep(10);
	}

	test_busy = true;

	/* Set Disable Interrupt but except MuxShort show Test */
	if (type != MUXSHORT_SHOW) {
		wbuf[0] = MIP_R0_CTRL;
		wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
		wbuf[2] = MIP_TRIGGER_TYPE_NONE;
		if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("[ERROR] Write - Disable interrupt\n");
			goto ERROR;
		}
	}

	if (reg_type == TESTREG) { //Read Test Register (Change test mode)
		/* Entry Test mode but except MuxShort show Test */
		if (type != MUXSHORT_SHOW) {
			/* Set test mode */
			wbuf[0] = MIP_R0_CTRL;
			wbuf[1] = MIP_R1_CTRL_MODE;
			wbuf[2] = MIP_CTRL_MODE_TEST_CM;
			if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
				TOUCH_ERR("[ERROR] Write - test mode\n");
				goto ERROR;
			}
			/* Wait ready status */
			wait_cnt = 20;
			while(wait_cnt--){
				if(MIT300_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
					TOUCH_LOG("wait_cnt : %d\n", wait_cnt);
					break;
				}
				msleep(50);
			}
			if (wait_cnt <= 0){
				TOUCH_ERR("[ERROR] Wait timeout\n");
				goto ERROR;
			}
		}
		/* Set test type */
		wbuf[0] = MIP_R0_TEST;
		wbuf[1] = MIP_R1_TEST_TYPE;
		wbuf[2] = test_type;
		if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("%s [ERROR] Write - test type\n", __func__);
			goto ERROR;
		}
		if (type == LPWG_JITTER_SHOW)
			mdelay(1000);
		/* Wait ready status */
		wait_cnt = 40;
		while(wait_cnt--){
			if(MIT300_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
				TOUCH_LOG("wait_cnt : %d\n", wait_cnt);
				break;
			}
			msleep(50);
		}
	} else if (reg_type == IMGREG) { //Read Image Register
		/* Set image type */
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_TYPE;
		wbuf[2] = test_type;
		if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("%s [ERROR] Write - image type\n", __func__);
			goto ERROR;
		}
		/* Wait ready status */
		wait_cnt = 50;
		while(wait_cnt--){
			if(MIT300_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
				TOUCH_LOG("wait_cnt : %d\n", wait_cnt);
				break;
			}
			msleep(50);
		}
	}

	if (wait_cnt <= 0){
		TOUCH_ERR("[ERROR] Wait timeout\n");
		goto ERROR;
	}

	/* Read self-diagnosis information */
	if (MIT300_GetDataFormat(client, data_form, reg_type)) {
		TOUCH_ERR("[ERROR] MIT300_GetDataFormat\n");
		goto ERROR;
	}

	if (MIT300_GetBufAddr(client, buf_addr, reg_type)) {
		TOUCH_ERR("[ERROR] MIT300_GetBufAddr\n");
		goto ERROR;
	}

	if(MIT300_ReadRMIBuffer(client, data_form, buf_addr)) {
		TOUCH_ERR("[ERROR] MIT300_ReadRMIBuffer\n");
		goto ERROR;
	}

	if (reg_type == TESTREG) {
		/* Entry Normal Mode but except Openshort Show Test */
		if (type != OPENSHORT_SHOW) {
			/* Set normal mode */
			wbuf[0] = MIP_R0_CTRL;
			wbuf[1] = MIP_R1_CTRL_MODE;
			wbuf[2] = MIP_CTRL_MODE_NORMAL;
			if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
				TOUCH_ERR("[ERROR] Write image type\n");
				goto ERROR;
			}

			/* Wait ready status */
			wait_cnt = 50;
			while(wait_cnt--){
				if(MIT300_GetReadyStatus(client) == MIP_CTRL_STATUS_READY){
					TOUCH_LOG("wait_cnt : %d\n", wait_cnt);
					break;
				}
				msleep(50);
			}
			if (wait_cnt <= 0){
				TOUCH_ERR("[ERROR] Wait timeout\n");
				goto ERROR;
			}
		}
	} else if (reg_type == IMGREG) {
		/* Clear image type */
		wbuf[0] = MIP_R0_IMAGE;
		wbuf[1] = MIP_R1_IMAGE_TYPE;
		wbuf[2] = MIP_IMG_TYPE_NONE;
		if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("[ERROR] Write image type\n");
			goto ERROR;
		}
	}
	/* Set Enable Interrupt but except Openshort Show Test */
	if (type != OPENSHORT_SHOW) {
		/* Enable Interrupt */
		wbuf[0] = MIP_R0_CTRL;
		wbuf[1] = MIP_R1_CTRL_EVENT_TRIGGER_TYPE;
		wbuf[2] = MIP_TRIGGER_TYPE_INTR;
		if (Mit300_I2C_Write(client, wbuf, 3) < 0) {
			TOUCH_ERR("[ERROR] Write - Enable interrupt\n");
			goto ERROR;
		}
	}
	test_busy = false;

	if (data_form != NULL) {
		kfree(data_form);
	}
	if (buf_addr != NULL) {
		kfree(buf_addr);
	}

	TOUCH_LOG("%s [DONE]\n", __func__);
	return TOUCH_SUCCESS;

ERROR:
	if (data_form != NULL) {
		kfree(data_form);
        }
	if (buf_addr != NULL) {
		kfree(buf_addr);
        }
	test_busy = false;
	TOUCH_ERR("%s [ERROR]\n", __func__);
	return TOUCH_FAIL;
}

ssize_t MIT300_GetTestResult(struct i2c_client *client, char *pBuf, int *result, int type)
{
	int i = 0;
	int ret = 0;
	int retry_max = 3;
	int retry_count = 0;
	int fd = 0;
	char *read_buf = NULL;
	char temp_buf[255] = {0,};
	char data_path[64] = {0,};

	mm_segment_t old_fs = get_fs();

	TOUCH_FUNC();

	for ( i = 0 ; i < MAX_ROW ; i++) {
		memset(mit_data[i], 0, sizeof(uint16_t) * MAX_COL);
		memset(intensity_data[i], 0x00, sizeof(s16) * MAX_COL);
	}

	read_buf = kzalloc(sizeof(u8) * 4096, GFP_KERNEL);
	if (read_buf == NULL) {
		TOUCH_ERR("read_buf mem_error\n");
		goto MEM_ERROR;
	}

	retry_count = 0;
	while (retry_count++ < retry_max) {
		if (MIT300_DoTest(client, type) == TOUCH_FAIL) {
			TOUCH_ERR("Getting data failed, retry (%d/%d)\n", retry_count, retry_max);
			MIT300_Reset(0,10);
			MIT300_Reset(1,100);
			mdelay(100);
		} else { break; }

		if (retry_count >= retry_max)
		{
			TOUCH_ERR("%s all retry failed \n", __func__);
			goto ERROR;
		}
	}

	switch(type) {
	case RAW_DATA_SHOW:
		ret += MIT300_PrintData(client, pBuf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		break;
	case RAW_DATA_STORE:
		snprintf(temp_buf, strlen(pBuf), "%s", pBuf);
		if (bootmode == BOOT_MINIOS)
			sprintf(data_path, "/data/logger/%s.csv", temp_buf);
		else if (bootmode == BOOT_NORMAL)
			sprintf(data_path, "/sdcard/%s.csv", temp_buf);
		ret += MIT300_PrintData(client, read_buf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		set_fs(KERNEL_DS);
		fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
		if (fd >= 0) {
			sys_write(fd, read_buf, 4096);
			sys_close(fd);
			TOUCH_LOG("%s saved \n", data_path);
		} else {
			TOUCH_LOG("%s open failed \n", data_path);
		}
		set_fs(old_fs);
		break;
	case OPENSHORT_SHOW:
		if (channelstatus_check) {
			TOUCH_LOG("Do not print data [only channel test]\n");
		} else {
			ret += MIT300_PrintData(client, pBuf, result, type);
			if (ret < 0) {
				TOUCH_ERR("fail to print type data\n");
				goto ERROR;
			}
		}
		break;
	case OPENSHORT_STORE:
		snprintf(temp_buf, strlen(pBuf), "%s", pBuf);
		if (bootmode == BOOT_MINIOS)
			sprintf(data_path, "/data/logger/%s.csv", temp_buf);
		else if (bootmode == BOOT_NORMAL)
			sprintf(data_path, "/sdcard/%s.csv", temp_buf);
		ret += MIT300_PrintData(client, read_buf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		set_fs(KERNEL_DS);
		fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
		if (fd >= 0) {
			sys_write(fd, read_buf, 4096);
			sys_close(fd);
			TOUCH_LOG("%s saved \n", data_path);
		} else {
			TOUCH_LOG("%s open failed \n", data_path);
		}
		set_fs(old_fs);
		break;
	case MUXSHORT_SHOW:
		if (channelstatus_check) {
			TOUCH_LOG("Do not print data [only channel test]\n");
		} else {
			ret += MIT300_PrintData(client, pBuf, result, type);
			if (ret < 0) {
				TOUCH_ERR("fail to print type data\n");
				goto ERROR;
			}
		}
		break;
	case CM_DELTA_SHOW:
		ret += MIT300_PrintData(client, pBuf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		break;
	case CM_DELTA_STORE:
		snprintf(temp_buf, strlen(pBuf), "%s", pBuf);
		if (bootmode == BOOT_MINIOS)
			sprintf(data_path, "/data/logger/%s.csv", temp_buf);
		else if (bootmode == BOOT_NORMAL)
			sprintf(data_path, "/sdcard/%s.csv", temp_buf);
		ret += MIT300_PrintData(client, read_buf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		set_fs(KERNEL_DS);
		fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
		if (fd >= 0) {
			sys_write(fd, read_buf, 4096);
			sys_close(fd);
			TOUCH_LOG("%s saved \n", data_path);
		} else {
			TOUCH_LOG("%s open failed \n", data_path);
		}
		set_fs(old_fs);
		break;
	case CM_JITTER_SHOW:
		ret += MIT300_PrintData(client, pBuf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		break;
	case CM_JITTER_STORE:
		snprintf(temp_buf, strlen(pBuf), "%s", pBuf);
		if (bootmode == BOOT_MINIOS)
			sprintf(data_path, "/data/logger/%s.csv", temp_buf);
		else if (bootmode == BOOT_NORMAL)
			sprintf(data_path, "/sdcard/%s.csv", temp_buf);
		ret += MIT300_PrintData(client, read_buf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		set_fs(KERNEL_DS);
		fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
		if (fd >= 0) {
			sys_write(fd, read_buf, 4096);
			sys_close(fd);
			TOUCH_LOG("%s saved \n", data_path);
		} else {
			TOUCH_LOG("%s open failed \n", data_path);
		}
		set_fs(old_fs);
		break;
	case LPWG_JITTER_SHOW:
		ret += MIT300_PrintData(client, pBuf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		break;
	case LPWG_ABS_SHOW:
		ret += MIT300_PrintData(client, pBuf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		break;
	case INTENSITY_SHOW:
		ret += MIT300_PrintData(client, pBuf, result, type);
		if (ret < 0) {
			TOUCH_ERR("fail to print type data\n");
			goto ERROR;
		}
		break;
	default :
		TOUCH_LOG("type = default[%d]\n", type);
		break;
	}

	if (read_buf != NULL)
		kfree(read_buf);

	return ret; // ret is size of data buffer .

ERROR:
	if (read_buf != NULL)
		kfree(read_buf);
	*result = TOUCH_FAIL;
	return TOUCH_FAIL;

MEM_ERROR :
	if (read_buf != NULL)
		kfree(read_buf);
	return TOUCH_FAIL;

}
