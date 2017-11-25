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
 *    File  	: lgtp_device_dummy.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[MIT300]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>

#include <linux/input/unified_driver_4/lgtp_common_driver.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_i2c.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>
#include <linux/input/unified_driver_4/lgtp_device_mit300.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
#if defined( TOUCH_MODEL_M2)
static const char defaultFirmware[] = "melfas/mit300/m2/melfas_mip4.bin";
#endif

#if defined(TOUCH_MODEL_PH1)
static const char defaultFirmware[] = "melfas/mit300/ph1/melfas_mip4.bin";
#endif

#if defined(TOUCH_MODEL_PH2)
static const char defaultFirmware[] = "melfas/mit300/ph2/melfas_mip4.bin";
#endif

static struct melfas_ts_data *ts = NULL;

#if defined(ENABLE_SWIPE_MODE)
static int get_swipe_mode = 1;
/*
static int wakeup_by_swipe = 0;
*/
extern int lockscreen_stat;
#endif


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
int event_format = 0;
u8 event_size = 0;

/****************************************************************************
* Local Functions
****************************************************************************/
static void MIT300_WriteFile(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	/*Need to use adb pull option in MiniOs Boot*/
	sys_chmod(filename, 0666);
	if (fd >= 0) {
		if (time > 0) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, 64, "\n%02d-%02d %02d:%02d:%02d.%03lu \n\n\n", my_date.tm_mon + 1,my_date.tm_mday, my_date.tm_hour, my_date.tm_min, my_date.tm_sec, (unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}


/****************************************************************************
* Device Specific Functions
****************************************************************************/
int mip_i2c_dummy(struct i2c_client* client,  char *write_buf, unsigned int write_len)
{
	int retry = 3;
	int res = 0;

	while (retry--) {
		TOUCH_FUNC();
		res = Mit300_I2C_Write(client, write_buf, write_len);
		if(res < 0) {
			TOUCH_ERR("i2c_transfer - errno[%d]\n", res);
			return TOUCH_FAIL;
		}
	}

	return TOUCH_SUCCESS;
}


int mip_lpwg_config(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
	wbuf[2] = 20;							// LPWG_IDLE_REPORTRATE
	wbuf[3] = 40;							// LPWG_ACTIVE_REPORTRATE
	wbuf[4] = 30;							// LPWG_SENSITIVITY
	wbuf[5] = 0 & 0xFF;					// LPWG_ACTIVE_AREA (horizontal start low byte)
	wbuf[6] = 0 >> 8 & 0xFF; 				// LPWG_ACTIVE_AREA (horizontal start high byte)
	wbuf[7] = 0 & 0xFF;					// LPWG_ACTIVE_AREA (vertical start low byte)
	wbuf[8] = 0 >> 8 & 0xFF;				// LPWG_ACTIVE_AREA (vertical start high byte)
	wbuf[9] = 720 & 0xFF;					// LPWG_ACTIVE_AREA (horizontal end low byte)
	wbuf[10] = 720 >> 8 & 0xFF;			// LPWG_ACTIVE_AREA (horizontal end high byte)
	wbuf[11] = 1280 & 0xFF; 				// LPWG_ACTIVE_AREA (vertical end low byte)
	wbuf[12] = 1280 >> 8 & 0xFF;			// LPWG_ACTIVE_AREA (vertical end high byte)
	wbuf[13] = ts->lpwg_fail_reason;		// LPWG_FAIL_REASON

	if( Mit300_I2C_Write(client, wbuf, 14) )
	{
		TOUCH_ERR("mip_lpwg_config failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_config_knock_on(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE;
	wbuf[2] = 1;							// LPWG_ENABLE
	wbuf[3] = 2;							// LPWG_TAP_COUNT
	wbuf[4] = 10 & 0xFF;					// LPWG_TOUCH_SLOP (low byte)
	wbuf[5] = 10 >> 8 & 0xFF;				// LPWG_TOUCH_SLOP (high byte)
	wbuf[6] = 0 & 0xFF;					// LPWG_MIN_DISTANCE (low byte)
	wbuf[7] = 0 >> 8 & 0xFF;				// LPWG_MIN_DISTANCE (high byte)
	wbuf[8] = 10 & 0xFF;					// LPWG_MAX_DISTANCE (low byte)
	wbuf[9] = 10 >> 8 & 0xFF;				// LPWG_MAX_DISTANCE (high byte)
	wbuf[10] = 0 & 0xFF;					// LPWG_MIN_INTERTAP_TIME (low byte)
	wbuf[11] = 0 >> 8 & 0xFF;				// LPWG_MIN_INTERTAP_TIME (high byte)
	wbuf[12] = 700 & 0xFF;					// LPWG_MAX_INTERTAP_TIME (low byte)
	wbuf[13] = 700 >> 8 & 0xFF;			// LPWG_MAX_INTERTAP_TIME (high byte)
	wbuf[14] = (DELAY_ENABLE ? KNOCKON_DELAY : 0) & 0xFF;			// LPWG_INTERTAP_DELAY (low byte)
	wbuf[15] = ((DELAY_ENABLE ? KNOCKON_DELAY : 0) >> 8) & 0xFF;	// LPWG_INTERTAP_DELAY (high byte)

	if( Mit300_I2C_Write(client, wbuf, 16) )
	{
		TOUCH_ERR("Knock on Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_config_knock_code(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE2;
	wbuf[2] = 1;							// LPWG_ENABLE2
	wbuf[3] = ts->lpwgSetting.tapCount;		// LPWG_TAP_COUNT2
	wbuf[4] = 10 & 0xFF;					// LPWG_TOUCH_SLOP2 (low byte)
	wbuf[5] = 10 >> 8 & 0xFF;				// LPWG_TOUCH_SLOP2 (high byte)
	wbuf[6] = 0 & 0xFF;						// LPWG_MIN_DISTANCE2 (low byte)
	wbuf[7] = 0 >> 8 & 0xFF;				// LPWG_MIN_DISTANCE2 (high byte)
	wbuf[8] = 65535 & 0xFF;					// LPWG_MAX_DISTANCE2 (low byte)
	wbuf[9] = 65535 >>8 & 0xFF;				// LPWG_MAX_DISTANCE2 (high byte)
	wbuf[10] = 0 & 0xFF;					// LPWG_MIN_INTERTAP_TIME2 (low byte)
	wbuf[11] = 0 >> 8 & 0xFF;				// LPWG_MIN_INTERTAP_TIME2 (high byte)
	wbuf[12] = 700 & 0xFF;					// LPWG_MAX_INTERTAP_TIME2 (low byte)
	wbuf[13] = 700 >> 8 & 0xFF;				// LPWG_MAX_INTERTAP_TIME2 (high byte)
	wbuf[14] = 250 & 0xFF;					// LPWG_INTERTAP_DELAY2 (low byte)
	wbuf[15] = 250 >> 8 & 0xFF;				// LPWG_INTERTAP_DELAY2 (high byte)

	if( Mit300_I2C_Write(client, wbuf, 16) )
	{
		TOUCH_ERR("Knock code Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_debug_enable(struct i2c_client* client, int enable)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_LPWG_DEBUG_ENABLE;
	wbuf[2] = enable;

	if( Mit300_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("LPWG debug Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_enable_sensing(struct i2c_client* client, bool enable)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE_SENSING;
	wbuf[2] = enable;

	if( Mit300_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("mip_lpwg_enable_sensing failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_start(struct i2c_client* client)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_START;
	wbuf[2] = 1;

	if( Mit300_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("mip_lpwg_start failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	TOUCH_FUNC();

	switch( newState )
	{
		case STATE_NORMAL:
			break;

		case STATE_KNOCK_ON_ONLY:
			mip_lpwg_config(client);
			mip_lpwg_config_knock_on(client);
			if( ts->lpwg_debug_enable )
				mip_lpwg_debug_enable(client, 1);
			mip_lpwg_start(client);
			if( ts->currState == STATE_OFF )
				mip_lpwg_enable_sensing(client, 1);
			break;

		case STATE_KNOCK_ON_CODE:
			mip_lpwg_config(client);
			mip_lpwg_config_knock_on(client);
			mip_lpwg_config_knock_code(client);
			if( ts->lpwg_debug_enable )
				mip_lpwg_debug_enable(client, 1);
			mip_lpwg_start(client);
			if( ts->currState == STATE_OFF )
				mip_lpwg_enable_sensing(client, 1);
			break;

		case STATE_OFF:
			mip_lpwg_start(client);
			mip_lpwg_enable_sensing(client, 0);
			break;

		default:
			TOUCH_ERR("invalid touch state ( %d )\n", newState);
			break;
	}

	return TOUCH_SUCCESS;
}

static ssize_t show_fw_dump(TouchDriverData *pDriverData, char *buf)
{
	int len = 0;
	int readsize = 0;
	int addr = 0;
	int retrycnt = 0;
	int fd = 0;
	u8 *pDump = NULL;
	char *dump_path = "/sdcard/touch_dump.fw";
	mm_segment_t old_fs = get_fs();

	TOUCH_LOG("F/W Dumping... \n");

	TouchDisableIrq();

	pDump = kzalloc(FW_MAX_SIZE, GFP_KERNEL);

RETRY :
	readsize = 0;
	retrycnt++;
	MIT300_Reset(0, 2);
	MIT300_Reset(1, 50);
	msleep(50);

	for (addr = 0; addr < FW_MAX_SIZE; addr += FW_BLOCK_SIZE ) {
		if ( mip_isc_read_page(ts, addr, &pDump[addr]) ) {
			TOUCH_ERR("F/W Read failed \n");
			if (retrycnt > 10) {
				len += snprintf(buf + len, PAGE_SIZE - len, "dump failed \n");
				goto EXIT;
			}
			else
				goto RETRY;
		}

		readsize += FW_BLOCK_SIZE;
		if (readsize % (FW_BLOCK_SIZE * 20) == 0) {
			TOUCH_LOG("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);
		}
	}

	TOUCH_LOG("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);

	set_fs(KERNEL_DS);
	fd = sys_open(dump_path, O_WRONLY|O_CREAT, 0666);
	if (fd >= 0) {
		sys_write(fd, pDump, FW_MAX_SIZE);
		sys_close(fd);
		len += snprintf(buf + len, PAGE_SIZE - len, "%s saved \n", dump_path);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "%s open failed \n", dump_path);
	}
	set_fs(old_fs);

EXIT :
	kfree(pDump);

	mip_isc_exit(ts);

	TouchSetGpioReset(0);
	msleep(10);
	TouchSetGpioReset(1);
	msleep(100);
	TOUCH_LOG("Device was reset\n");

	TouchEnableIrq();

	return len;
}

static ssize_t show_lpwg_debug_enable(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "LPWG Debug option enable : 1\n");
	ret += sprintf(buf+ret, "LPWG Debug option disable : 0\n");
	ret += sprintf(buf+ret, "ts->lpwg_debug_enable : [%d]\n", ts->lpwg_debug_enable);

	return ret;
}

static ssize_t store_lpwg_debug_enable(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int cmd = 0;

	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On \n");
		return count;
	}
	if (sscanf(buf, "%d", &cmd) !=1)
		return -EINVAL;

	ts->lpwg_debug_enable = cmd;

	return count;
}

static ssize_t show_lpwg_fail_reason(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "Fail Reason option enable : 1\n");
	ret += sprintf(buf+ret, "Fail Reason option disable : 0\n");
	ret += sprintf(buf+ret, "ts->lpwg_fail_reason : [%d]\n", ts->lpwg_fail_reason);

	return ret;
}

static ssize_t store_lpwg_fail_reason(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int cmd = 0;

	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On \n");
		return count;
	}
	if (sscanf(buf, "%d", &cmd) !=1)
		return -EINVAL;

	ts->lpwg_fail_reason = cmd;

	return count;
}

static ssize_t show_openshort(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int rawdataStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		ret = sprintf(buf, "LCD is off. Try after LCD On\n");
		return ret;
	}

	/* openshort check */
	ret = MIT300_GetTestResult(client, buf, &rawdataStatus, OPENSHORT_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(buf, "failed to get raw data\n");
	}

	return ret;
}

#define NAME_BUFFER_SIZE 	128
static ssize_t store_openshort(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int ret = 0;
	int rawdataStatus = 0;
	char temp_buf[255] = {0};
	char cmd[NAME_BUFFER_SIZE] = {0};
	struct i2c_client *client = Touch_Get_I2C_Handle();

	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for rawdata\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		return count;
	}

	if (count > NAME_BUFFER_SIZE)
		return count;

	if (sscanf(buf, "%127s", cmd) != 1)
		return -EINVAL;

	if (strlen(buf) < 254) {
		strlcpy(temp_buf, buf, strlen(buf) + 1);
	} else {
		TOUCH_ERR("buffer size is more than 255\n");
	}

	/* openshort check */
	ret = MIT300_GetTestResult(client, temp_buf, &rawdataStatus, OPENSHORT_STORE);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(temp_buf, "failed to get raw data\n");
	}

	return count;
}

static ssize_t show_cmjitter(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int rawdataStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		ret = sprintf(buf, "LCD is off. Try after LCD On\n");
		return ret;
	}

	/* cmjitter check */
	ret = MIT300_GetTestResult(client, buf, &rawdataStatus, CM_JITTER_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(buf, "failed to get raw data\n");
	}

	return ret;
}

#define NAME_BUFFER_SIZE 	128
static ssize_t store_cmjitter(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int ret = 0;
	int rawdataStatus = 0;
	char temp_buf[255] = {0};
	char cmd[NAME_BUFFER_SIZE] = {0};
	struct i2c_client *client = Touch_Get_I2C_Handle();

	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for rawdata\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		return count;
	}

	if (count > NAME_BUFFER_SIZE)
		return count;

	if (sscanf(buf, "%127s", cmd) != 1)
		return -EINVAL;

	if (strlen(buf) < 254) {
		strlcpy(temp_buf, buf, strlen(buf) + 1);
	} else {
		TOUCH_ERR("buffer size is more than 255\n");
	}

	/* cmjitter check */
	ret = MIT300_GetTestResult(client, temp_buf, &rawdataStatus, CM_JITTER_STORE);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(temp_buf, "failed to get raw data\n");
	}

	return count;
}

static ssize_t show_cmdelta(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int rawdataStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		ret = sprintf(buf, "LCD is off. Try after LCD On\n");
		return ret;
	}

	/* cmdelta check */
	ret = MIT300_GetTestResult(client, buf, &rawdataStatus, CM_DELTA_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(buf, "failed to get raw data\n");
	}

	return ret;
}

#define NAME_BUFFER_SIZE 	128
static ssize_t store_cmdelta(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int ret = 0;
	int rawdataStatus = 0;
	char temp_buf[255] = {0};
	char cmd[NAME_BUFFER_SIZE] = {0};
	struct i2c_client *client = Touch_Get_I2C_Handle();

	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for rawdata\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		return count;
	}

	if (count > NAME_BUFFER_SIZE)
		return count;

	if (sscanf(buf, "%127s", cmd) != 1)
		return -EINVAL;

	if (strlen(buf) < 254) {
		strlcpy(temp_buf, buf, strlen(buf) + 1);
	} else {
		TOUCH_ERR("buffer size is more than 255\n");
	}

	/* cmdelta check */
	ret = MIT300_GetTestResult(client, temp_buf, &rawdataStatus, CM_DELTA_STORE);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(temp_buf, "failed to get raw data\n");
	}

	return count;
}

static ssize_t show_rawdata(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int rawdataStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		ret = sprintf(buf, "LCD is off. Try after LCD On\n");
		return ret;
	}

	/* rawdata check */
	ret = MIT300_GetTestResult(client, buf, &rawdataStatus, RAW_DATA_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(buf, "failed to get raw data\n");
	}

	return ret;
}

#define NAME_BUFFER_SIZE 	128
static ssize_t store_rawdata(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int ret = 0;
	int rawdataStatus = 0;
	char temp_buf[255] = {0};
	char cmd[NAME_BUFFER_SIZE] = {0};
	struct i2c_client *client = Touch_Get_I2C_Handle();

	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for rawdata\n");
	}

	TOUCH_FUNC();
	if (pDriverData->lpwgSetting.lcdState == 0) {
		TOUCH_LOG("LCD is off. Try after LCD On\n");
		return count;
	}

	if (count > NAME_BUFFER_SIZE)
		return count;

	if (sscanf(buf, "%127s", cmd) != 1)
		return -EINVAL;

	if (strlen(buf) < 254) {
		strlcpy(temp_buf, buf, strlen(buf) + 1);
	} else {
		TOUCH_ERR("buffer size is more than 255\n");
	}

	/* rawdata check */
	ret = MIT300_GetTestResult(client, temp_buf, &rawdataStatus, RAW_DATA_STORE);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(temp_buf, "failed to get raw data\n");
	}

	return count;
}

static ssize_t show_intensity(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int intensityStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();

	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();

	/* intensity check */
	ret = MIT300_GetTestResult(client, buf, &intensityStatus, INTENSITY_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get intensity data\n");
		ret = sprintf(buf, "failed to get intensity data\n");
	}

	return ret;
}

static ssize_t store_reg_control(TouchDriverData *pDriverData, const char *buf, size_t count)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();
    int reg_addr[2] = {0};
    int cmd = 0;
    int value = 0;
    uint8_t write_buf[50] = {0};
    uint8_t read_buf[50] = {0};
    int i = 0;
    int len = 2;
    if ( sscanf(buf, "%d %x %x %d", &cmd, &reg_addr[0], &reg_addr[1], &value) != 4) {
        TOUCH_LOG("data parsing fail.\n");
        return -EINVAL;
    }
    TOUCH_LOG("%d, 0x%x, 0x%x, %d\n", cmd, reg_addr[0], reg_addr[1], value);

    switch (cmd) {
        case 1:
            write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];
            if( Mit300_I2C_Read(client, write_buf, len, read_buf, value) )
            {
                TOUCH_LOG("store_reg_control failed\n");
            }

            for (i = 0; i < value; i ++) {
                TOUCH_LOG("read_buf=[%d]\n",read_buf[i]);
            }
            break;
        case 2:
            write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];
            if (value >= 256) {
				write_buf[2] = (value >> 8);
				write_buf[3] = (value & 0xFF);
				len = len + 2;
			} else {
				write_buf[2] = value;
				len++;
			}
            if( Mit300_I2C_Write(client, write_buf, len) )
            {
                 TOUCH_ERR("store_reg_control failed\n");
            }
            break;
        default:
            TOUCH_LOG("usage: echo [1(read)|2(write)], [reg address0], [reg address1], [length(read)|value(write)] > reg_control\n");
            break;
    }
    return count;
}

static ssize_t store_quick_cover_status(TouchDriverData *pDriverData, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	if ((value == QUICKCOVER_CLOSE) && (ts->quick_cover_status == QUICKCOVER_OPEN)) {
		ts->quick_cover_status = QUICKCOVER_CLOSE;
	} else if ((value == QUICKCOVER_OPEN) && (ts->quick_cover_status == QUICKCOVER_CLOSE)) {
		ts->quick_cover_status = QUICKCOVER_OPEN;
	} else {
		return count;
	}

	TOUCH_LOG("quick cover status = %s\n",
			(ts->quick_cover_status == QUICKCOVER_CLOSE) ? "QUICK_COVER_CLOSE" : "QUICK_COVER_OPEN");

	return count;
}

static ssize_t show_pen_support(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int pen_support = 0;   /* 1: Support , 0: Not support */

	pen_support = 1;

	TOUCH_LOG("Read Pen Support : %d\n", pen_support);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", pen_support);

	return ret;
}

static LGE_TOUCH_ATTR(fw_dump, S_IRUSR | S_IWUSR, show_fw_dump, NULL);
static LGE_TOUCH_ATTR(lpwg_debug_enable, S_IRUGO | S_IWUSR, show_lpwg_debug_enable, store_lpwg_debug_enable);
static LGE_TOUCH_ATTR(lpwg_fail_reason, S_IRUGO | S_IWUSR, show_lpwg_fail_reason, store_lpwg_fail_reason);
static LGE_TOUCH_ATTR(openshort, S_IRUGO | S_IWUSR, show_openshort, store_openshort);
static LGE_TOUCH_ATTR(cmjitter, S_IRUGO | S_IWUSR, show_cmjitter, store_cmjitter);
static LGE_TOUCH_ATTR(cmdelta, S_IRUGO | S_IWUSR, show_cmdelta, store_cmdelta);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR, show_rawdata, store_rawdata);
static LGE_TOUCH_ATTR(intensity, S_IRUGO | S_IWUSR, show_intensity, NULL);
static LGE_TOUCH_ATTR(reg_control,  S_IRUGO | S_IWUSR, NULL, store_reg_control);
static LGE_TOUCH_ATTR(quick_cover_status, S_IRUGO | S_IWUSR, NULL, store_quick_cover_status);
static LGE_TOUCH_ATTR(pen_support, S_IRUGO | S_IWUSR, show_pen_support, NULL);

static struct attribute *MIT300_attribute_list[] = {
	&lge_touch_attr_fw_dump.attr,
	&lge_touch_attr_lpwg_debug_enable.attr,
	&lge_touch_attr_lpwg_fail_reason.attr,
	&lge_touch_attr_openshort.attr,
	&lge_touch_attr_cmjitter.attr,
	&lge_touch_attr_cmdelta.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_intensity.attr,
	&lge_touch_attr_reg_control.attr,
	&lge_touch_attr_quick_cover_status.attr,
	&lge_touch_attr_pen_support.attr,
	NULL,
};

static int MIT300_Initialize(TouchDriverData *pDriverData)
{
	struct i2c_client *client = Touch_Get_I2C_Handle();
	TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;
	bootmode = pDriverData->bootMode;
	TOUCH_LOG("factory boot check : %d\n", bootmode);
	return TOUCH_SUCCESS;
}

void MIT300_Reset(int status, int delay)
{
    if(!status)TouchDisableIrq();
	TouchSetGpioReset(status);
	 if( delay <= 0 || delay > 1000 )
        {
            TOUCH_LOG("%s exeeds limit %d\n", __func__, delay);
            return;
        }

	if(delay<=20)
		mdelay(delay);
    else
		msleep(delay);
    if(status)TouchEnableIrq();
}

static void MIT300_Reset_Dummy(void)
{

}

static int MIT300_InitRegister(void)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();
    u8 wbuf[4];

	/* IMPLEMENT : Register initialization after reset */
    if(lge_get_boot_mode() == LGE_BOOT_MODE_QEM_130K) {
        TOUCH_ERR("LGE_BOOT_MODE_QEM_130K\n");
        mip_lpwg_start(client);
        wbuf[0] = 0x06;
		wbuf[1] = 0x18;
		wbuf[2] = 1;
		if( Mit300_I2C_Write(client, wbuf, 3) )
		{
			TOUCH_ERR("mip_lpwg_start failed\n");
			return TOUCH_FAIL;
		}
	}
	return TOUCH_SUCCESS;
}

static int MIT300_InterruptHandler(TouchReadData *pData)
{
	TouchFingerData *pFingerData = NULL;
	u8 i = 0;
	u8 wbuf[8] = {0};
	u8 rbuf[256] = {0};
	u32 packet_size = 0;
	u8 packet_type = 0;
	u8 alert_type = 0;
	u8 index = 0;
	u8 state = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();

	pData->type = DATA_UNKNOWN;
	pData->count = 0;

	if (ts->lpwg_debug_enable == 0 && (ts->currState == STATE_KNOCK_ON_ONLY || ts->currState == STATE_KNOCK_ON_CODE)) {
		if(mip_i2c_dummy(client, wbuf, 2) == TOUCH_FAIL){
			TOUCH_ERR("Fail to send dummy packet\n");
			return TOUCH_FAIL;
		}
	}

	//Read packet info
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if( Mit300_I2C_Read(client, wbuf, 2, rbuf, 1) )
	{
		TOUCH_ERR("Read packet info\n");
		return TOUCH_FAIL;
	}

	packet_size = (rbuf[0] & 0x7F);
	packet_type = ((rbuf[0] >> 7) & 0x1);

	//Check size
	if( packet_size == 0 )
		return TOUCH_SUCCESS;

	//Read packet data
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
	if( Mit300_I2C_Read(client, wbuf, 2, rbuf, packet_size) )
	{
		TOUCH_ERR("Read packet data\n");
		return TOUCH_FAIL;
	}

	//Event handler
	if( packet_type == 0 )	/* Touch event */
	{
		for( i = 0 ; i < packet_size ; i += event_size )
		{
			u8 *tmp = &rbuf[i];

			if( (tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0 )
			{
				TOUCH_LOG("use sofrware key\n");
				continue;
			}

			index = (tmp[0] & 0xf) - 1;
			state = (tmp[0] & 0x80) ? 1 : 0;

			if( (index < 0) || (index > MAX_NUM_OF_FINGERS - 1) )
			{
				TOUCH_ERR("invalid touch index (%d)\n", index);
				return TOUCH_FAIL;
			}

			pData->type = DATA_FINGER;

			if( (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4 )
			{
				if( state )
					TOUCH_LOG("Palm detected : %d \n", tmp[4]);
				else
					TOUCH_LOG("Palm released : %d \n", tmp[4]);

				return TOUCH_SUCCESS;
			}

			if( state )
			{
				pFingerData = &pData->fingerData[index];
				pFingerData->id = index ;
				pFingerData->x = tmp[2] | ((tmp[1] & 0x0f) << 8);
				pFingerData->y = tmp[3] | ((tmp[1] & 0xf0) << 4);
				pFingerData->width_major = tmp[5];
				pFingerData->width_minor = 0;
				pFingerData->orientation = 0;
				pFingerData->pressure = tmp[4];
				if( tmp[4] < 1 )
					pFingerData->pressure = 1;
				else if( tmp[4] > 255 - 1 )
					pFingerData->pressure = 255 - 1;

				pData->count++;
				pFingerData->status = FINGER_PRESSED;
			}else{
				pFingerData = &pData->fingerData[index];
				pFingerData->id = index ;
				pFingerData->status = FINGER_RELEASED;
			}
		}
	}
	else	/* Alert event */
	{
		alert_type = rbuf[0];

		if( alert_type == MIP_ALERT_ESD )	//ESD detection
		{
			TOUCH_LOG("ESD Detected!\n");
		}
		else if( alert_type == MIP_ALERT_WAKEUP )	//Wake-up gesture
		{
			if( rbuf[1] == MIP_EVENT_GESTURE_DOUBLE_TAP )
			{
				TOUCH_LOG("Knock-on Detected\n");
				pData->type = DATA_KNOCK_ON;
			}
			else if( rbuf[1] == MIP_EVENT_GESTURE_MULTI_TAP )
			{
				TOUCH_LOG("Knock-code Detected\n");
				pData->type = DATA_KNOCK_CODE;

				for( i = 2 ; i < packet_size ; i += 3 )
				{
					u8 *tmp = &rbuf[i];
					pData->knockData[((i + 1) / 3) - 1].x = tmp[1] | ((tmp[0] & 0xf) << 8);
					pData->knockData[((i + 1) / 3) - 1].y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
					pData->count++;
				}
			}
			else
			{
				//Re-enter tap mode
				wbuf[0] = MIP_R0_CTRL;
				wbuf[1] = MIP_R1_CTRL_POWER_STATE;
				wbuf[2] = MIP_CTRL_POWER_LOW;
				if( Mit300_I2C_Write(client, wbuf, 3) )
				{
					TOUCH_ERR("mip_i2c_write failed\n");
					return TOUCH_FAIL;
				}
			}
		}
		else if( alert_type == MIP_ALERT_F1 )	//Gesture Fail Reason
		{
			if( rbuf[1] == MIP_LPWG_EVENT_TYPE_FAIL )
			{
				switch( rbuf[2] )
				{
					case OUT_OF_AREA:
						TOUCH_LOG("LPWG FAIL REASON = Out of Area\n");
						break;
					case PALM_DETECTED:
						TOUCH_LOG("LPWG FAIL REASON = Palm\n");
						break;
					case DELAY_TIME:
						TOUCH_LOG("LPWG FAIL REASON = Delay Time\n");
						break;
					case TAP_TIME:
						TOUCH_LOG("LPWG FAIL REASON = Tap Time\n");
						break;
					case TAP_DISTACE:
						TOUCH_LOG("LPWG FAIL REASON = Tap Distance\n");
						break;
					case TOUCH_SLOPE:
						TOUCH_LOG("LPWG FAIL REASON = Touch Slope\n");
						break;
					case MULTI_TOUCH:
						TOUCH_LOG("LPWG FAIL REASON = Multi Touch\n");
						break;
					case LONG_PRESS:
						TOUCH_LOG("LPWG FAIL REASON = Long Press\n");
						break;
					default:
						TOUCH_LOG("LPWG FAIL REASON = Unknown Reason\n");
						break;
				}
			}
			else
			{
				//Re-enter tap mode
				wbuf[0] = MIP_R0_CTRL;
				wbuf[1] = MIP_R1_CTRL_POWER_STATE;
				wbuf[2] = MIP_CTRL_POWER_LOW;
				if( Mit300_I2C_Write(client, wbuf, 3) )
				{
					TOUCH_ERR("mip_i2c_write failed\n");
					return TOUCH_FAIL;
				}
			}
		}
		else
		{
			TOUCH_LOG("Unknown alert type [%d]\n", alert_type);
		}
	}

	return TOUCH_SUCCESS;

}

static int MIT300_ReadIcFirmwareInfo( TouchFirmwareInfo *pFwInfo)
{
	u8 wbuf[2] = {0, };
	u8 rbuf[64] = {0, };
	//u8 version[2] = {0, };
	int ret = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();

	TOUCH_FUNC();
	TOUCH_LOG("===========================================\n");
	/* IMPLEMENT : read IC firmware information function */
	//Product name
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
	ret = Mit300_I2C_Read(client, wbuf, 2, rbuf, 16);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("[ERROR] read product name\n");
		return TOUCH_FAIL;
	}
	memcpy((u8 *) &ts->product_code, rbuf, 16);
	TOUCH_LOG("F/W Product : %s \n", ts->product_code);

	//Ic name
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_IC_NAME;
	ret = Mit300_I2C_Read(client, wbuf, 2, rbuf, 4);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("[ERROR] read ic name\n");
		return TOUCH_FAIL;
	}
	memcpy((u8 *) &ts->ic_name, rbuf, 4);
	TOUCH_LOG("IC Name : %s \n", ts->ic_name);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_CUSTOM;
	ret = Mit300_I2C_Read(client, wbuf, 2, rbuf, 2);
	if( ret == TOUCH_FAIL )
		return TOUCH_FAIL;

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = rbuf[1];
	pFwInfo->version = rbuf[0];
	TOUCH_LOG("IC F/W Version = v%X.%02X ( %s )\n", rbuf[1], rbuf[0], pFwInfo->isOfficial ? "Official Release" : "Test Release");

	//Resolution
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	if(Mit300_I2C_Read(client, wbuf, 2, rbuf, 7)) {
		TOUCH_ERR("[ERROR] get resulution\n");
		ts->x_resolution = 720;
		ts->y_resolution = 1280;
		return -EIO;
	} else {
		ts->x_resolution = (rbuf[0]) | (rbuf[1] << 8);
		ts->y_resolution = (rbuf[2]) | (rbuf[3] << 8);
	}

	//Node info
	ts->col_num = rbuf[4];
	ts->row_num = rbuf[5];
	ts->key_num = rbuf[6];

	TOUCH_LOG("max_x[%d] max_y[%d]\n", ts->x_resolution, ts->y_resolution);
	TOUCH_LOG("col_num[%d] row_num[%d] key_num[%d]\n", ts->col_num, ts->row_num, ts->key_num);

	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_SUPPORTED_FUNC;
	ret = Mit300_I2C_Read(client, wbuf, 2, rbuf, 7);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("[ERROR] read node info\n");
		return TOUCH_FAIL;
	}
	event_format = (rbuf[4]) | (rbuf[5] << 8);
	event_size = rbuf[6];
	TOUCH_LOG("event_format[%d] event_size[%d]\n", event_format, event_size);

	TOUCH_LOG("===========================================\n");

	return TOUCH_SUCCESS;
}

static int MIT300_GetBinFirmwareInfo( char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 version[2] = {0, };
	u8 *pFwFilename = NULL;
    struct i2c_client *client = Touch_Get_I2C_Handle();

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	mip_bin_fw_version(ts, fw->data, fw->size, version);

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = version[0] ;
	pFwInfo->version = version[1];

	/* Free firmware image buffer */
	release_firmware(fw);

	TOUCH_LOG("BIN F/W Version = v%X.%02X ( %s )\n", version[0], version[1], pFwInfo->isOfficial ? "Official Release" : "Test Release");

	return TOUCH_SUCCESS;
}

static int MIT300_UpdateFirmware( char *pFilename)
{
	int ret = 0;
	char *pFwFilename = NULL;
	const struct firmware *fw = NULL;
    struct i2c_client *client = Touch_Get_I2C_Handle();

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ret = mip_flash_fw(ts, fw->data, fw->size, false, true);
	if( ret < fw_err_none ) {
		return TOUCH_FAIL;
	}

	release_firmware(fw);

	return TOUCH_SUCCESS;
}

static int MIT300_SetLpwgMode( TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;
    struct i2c_client *client = Touch_Get_I2C_Handle();

	TOUCH_FUNC();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

	if( ts->currState == newState ) {
		TOUCH_LOG("device state is same as driver requested\n");
		return TOUCH_SUCCESS;
	}

	if( ( newState < STATE_NORMAL ) && ( newState > STATE_KNOCK_ON_CODE ) ) {
		TOUCH_LOG("invalid request state ( state = %d )\n", newState);
		return TOUCH_FAIL;
	}

	ret = lpwg_control(client, newState);
	if( ret == TOUCH_FAIL ) {
		TOUCH_ERR("failed to set lpwg mode in device\n");
		return TOUCH_FAIL;
	}

	if( ret == TOUCH_SUCCESS ) {
		ts->currState = newState;
	}

	switch( newState )
	{
		case STATE_NORMAL:
			TOUCH_LOG("device was set to NORMAL\n");
			break;
		case STATE_OFF:
			TOUCH_LOG("device was set to OFF\n");
			break;
		case STATE_KNOCK_ON_ONLY:
			TOUCH_LOG("device was set to KNOCK_ON_ONLY\n");
			break;
		case STATE_KNOCK_ON_CODE:
			TOUCH_LOG("device was set to KNOCK_ON_CODE\n");
			break;
		default:
			TOUCH_LOG("impossilbe state ( state = %d )\n", newState);
			ret = TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;
}

static int MIT300_DoSelfDiagnosis(int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	/* CAUTION : be careful not to exceed buffer size */
	char *sd_path = NULL;
	int ret = 0;
	int OpenShortStatus = 0;
	int MuxShortStatus = 0;
	int deltaStatus = 0;
	int jitterStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	memset(pBuf, 0, bufSize);
	*pDataLen = 0;
	TOUCH_FUNC();
	/* CAUTION : be careful not to exceed buffer size */

	/* IMPLEMENT : self-diagnosis function */

	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;
	OpenShortStatus = TOUCH_SUCCESS;
	MuxShortStatus = TOUCH_SUCCESS;
	deltaStatus = TOUCH_SUCCESS;
	jitterStatus = TOUCH_SUCCESS;
	if (bootmode == BOOT_MINIOS)
		sd_path = "/data/logger/touch_self_test.txt";
	else
		sd_path = "/mnt/sdcard/touch_self_test.txt";
	MIT300_WriteFile(sd_path, pBuf, 1);
	msleep(30);

	// open short check
	ret = MIT300_GetTestResult(client, pBuf, &OpenShortStatus, OPENSHORT_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get open short data\n");
		memset(pBuf, 0, bufSize);
		*pChannelStatus = TOUCH_FAIL;
		goto error;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// open short 2 check (MUX)
	ret = MIT300_GetTestResult(client, pBuf, &MuxShortStatus, MUXSHORT_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get open short (mux) data\n");
		memset(pBuf, 0, bufSize);
		*pChannelStatus = TOUCH_FAIL;
		goto error;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// cm_delta check
	ret = MIT300_GetTestResult(client, pBuf, &deltaStatus, CM_DELTA_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get delta data\n");
		memset(pBuf, 0, bufSize);
		deltaStatus = TOUCH_FAIL;
		goto error;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// cm_jitter check
	ret = MIT300_GetTestResult(client, pBuf, &jitterStatus, CM_JITTER_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get jitter data\n");
		memset(pBuf, 0, bufSize);
		jitterStatus = TOUCH_FAIL;
		goto error;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// raw data check
	ret = MIT300_GetTestResult(client, pBuf, pRawStatus, RAW_DATA_SHOW);
	if( ret < 0 ) {
		TOUCH_ERR("failed to get raw data\n");
		memset(pBuf, 0, bufSize);
		*pRawStatus = TOUCH_FAIL;
		goto error;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	ret = sprintf(pBuf, "%s", "======ADDITIONAL======\n");
	ret += sprintf(pBuf+ret, "Delta Test: %s", (deltaStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	ret += sprintf(pBuf+ret, "Jitter Test: %s", (jitterStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	*pDataLen = ret;

	if ((OpenShortStatus == TOUCH_FAIL) || (MuxShortStatus == TOUCH_FAIL)) {
		TOUCH_LOG("OpenShortStatus : %s\n", OpenShortStatus == TOUCH_SUCCESS ? "PASS" : "FAIL");
		TOUCH_LOG("MuxShortStatus : %s\n", MuxShortStatus == TOUCH_SUCCESS ? "PASS" : "FAIL");
		*pChannelStatus = TOUCH_FAIL;
	} else {
		*pChannelStatus = TOUCH_SUCCESS;
	}

	return TOUCH_SUCCESS;
error :
	return TOUCH_FAIL;
}

static void MIT300_PowerOn(int isOn)
{

}

static void MIT300_ClearInterrupt(void)
{

}

static void MIT300_NotifyHandler(TouchNotify notify, int data)
{

}

TouchDeviceControlFunction MIT300_Func = {
    .Power                  = MIT300_PowerOn,
	.Initialize 			= MIT300_Initialize,
	.Reset 					= MIT300_Reset_Dummy,
	.InitRegister 			= MIT300_InitRegister,
	.ClearInterrupt         = MIT300_ClearInterrupt,
	.InterruptHandler 		= MIT300_InterruptHandler,
	.ReadIcFirmwareInfo 	= MIT300_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo 	= MIT300_GetBinFirmwareInfo,
	.UpdateFirmware 		= MIT300_UpdateFirmware,
	.SetLpwgMode 			= MIT300_SetLpwgMode,
	.DoSelfDiagnosis 		= MIT300_DoSelfDiagnosis,
	.device_attribute_list 	= MIT300_attribute_list,
	.NotifyHandler          = MIT300_NotifyHandler,
};


/* End Of File */


