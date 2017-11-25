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
 *    File  	: lgtp_device_lr388k6.c
 *    Author(s)   : BSP Touch Team
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[LR388K6]"

/****************************************************************************
 * Include Files
 ****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>

#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_device_lr388k6.h>

#include <linux/fs.h>
#include "shtsc_ioctl.h"
#include "lgtp_lr388k6_panel_spec.h"
#include "lgtp_lr388k6_panel_spec2.h"

/****************************************************************************
 * Manifest Constants / Defines
 ****************************************************************************/
/* INT STATUS */
#define SHTSC_STATUS_TOUCH_READY    (1<<0)
#define SHTSC_STATUS_POWER_UP       (1<<1)
#define SHTSC_STATUS_RESUME_PROX    (1<<2)
#define SHTSC_STATUS_WDT            (1<<3)
#define SHTSC_STATUS_DCMAP_READY    (1<<4)
#define SHTSC_STATUS_COMMAND_RESULT (1<<5)
#define SHTSC_STATUS_LG_LPWG_TCI1   (1<<6)
#define SHTSC_STATUS_LG_LPWG_TCI2   (1<<7)
#define SHTSC_STATUS_FLASH_LOAD_ERROR    (1<<8)
#define SHTSC_STATUS_PLL_UNLOCK     (1<<9)

/* DONE IND */
#define SHTSC_IND_CMD   0x20
#define SHTSC_IND_TOUCH 0x01
#define SHTSC_IND_DCMAPDONE 0x10

/* BANK Address */
#define SHTSC_BANK_TOUCH_REPORT   0x00
#define SHTSC_BANK_LPWG_DATA      0x00
#define SHTSC_BANK_LPWG_PARAM     0x01
#define SHTSC_BANK_COMMAND        0x02
#define SHTSC_BANK_COMMAND_RESULT 0x03
#define SHTSC_BANK_DCMAP          0x05
#define SHTSC_BANK_SYSTEM_HA      0x1C

/* Common Register Address */
#define SHTSC_ADDR_INT0  0x00
#define SHTSC_ADDR_INTMASK0 0x01
#define SHTSC_ADDR_BANK 0x02
#define SHTSC_ADDR_IND  0x03
#define SHTSC_ADDR_INT1  0x04
#define SHTSC_ADDR_INTMASK1 0x05

/* Touch Report Register Address */
#define SHTSC_ADDR_TOUCH_NUM 0x08
#define SHTSC_ADDR_RESUME_PROX 0x09
#define SHTSC_ADDR_TOUCH_REPORT 0x10
#define SHTSC_ADDR_LPWG_REPORT 0x10

/* Touch Parmeters */
#define SHTSC_MAX_FINGERS 10
#define SHTSC_MAX_TOUCH_1PAGE 10
#define SHTSC_LENGTH_OF_TOUCH 8
#define SHTSC_LENGTH_OF_LPWG 4

/* Touch Status */
#define SHTSC_F_TOUCH ((u8)0x01)
#define SHTSC_F_TOUCH_OUT ((u8)0x03)
//#define SHTSC_P_TOUCH ((u8)0x02)
//#define SHTSC_P_TOUCH_OUT ((u8)0x04)

#define SHTSC_TOUCHOUT_STATUS ((u8)0x80)

#define SHTSC_ADDR_COMMAND 0x08

#define CMD_GETPROPERTY "\xE0\x00\x00\x11"
#define CMD_GETPROPERTY_LEN 4

#define CMD_SETSYSTEMSTATE_SLEEP "\x02\x00\x01\x00\x00"
#define CMD_SETSYSTEMSTATE_SLEEP_LEN 5
#define CMD_SETSYSTEMSTATE_DEEPIDLE "\x02\x00\x01\x00\x04"
#define CMD_SETSYSTEMSTATE_DEEPIDLE_LEN 5
#define CMD_SETSYSTEMSTATE_IDLE "\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_IDLE_LEN 5
#define CMD_SETSYSTEMSTATE_COVER "\x02\x00\x01\x00\x0D"
#define CMD_SETSYSTEMSTATE_COVER_LEN 5
#define CMD_GETSYSTEMSTATE "\x03\x00\x00\x01"
#define CMD_GETSYSTEMSTATE_LEN 4

#define CMD_SETSYSTEMSTATE_PEN_MODE "\x02\x00\x01\x00\x08"
#define CMD_SETSYSTEMSTATE_PEN_MODE_LEN 5

#define CMD_SETSYSTEMSTATE_NORMAL_MODE "\x02\x00\x01\x00\x03"
#define CMD_SETSYSTEMSTATE_NORMAL_MODE_LEN 5

#define CMD_GETPANELPARAM "\xD8\x00\x00\x06"
#define CMD_GETPANELPARAM_LEN 4

#define CMD_EXECCALIBRATION_FOR_WRITE "\x0F\x00\x03\x00\x00\x64\0x00"
#define CMD_EXECCALIBRATION_FOR_WRITE2 "\x0F\x00\x03\x00\x07\xE8\0x03"
#define CMD_EXECCALIBRATION_FOR_WRITE_LEN 7


#define CMD_DCMAP_ON  "\xD7\x00\x01\x00\x01"
#define CMD_DCMAP_OFF "\xD7\x00\x01\x00\x00"
#define CMD_DCMAP_ON_LEN 5
#define CMD_DCMAP_OFF_LEN 5

#define CMD_TOUCH_OFF "\x10\x00\x09\x00\x01\xD0\x07\xD0\x07\xD1\x07\xD1\x07" // (2000,2000)-(2001,2001) non exist coordinate
#define CMD_TOUCH_ON  "\x10\x00\x09\x00\x01\x00\x00\x00\x00\x37\x04\x7F\x07" // (0,0)-(1079, 1919)
#define CMD_TOUCH_ENABLE_CONTROL_LENGTH (13)

#define CMD_SETREGTBL_CALIB_ON  "\x06\x00\x04\x00\x00\x12\x6E\x01"
#define CMD_SETREGTBL_CALIB_OFF  "\x06\x00\x04\x00\x00\x12\x6E\x21"
#define CMD_SETREGTBL_CALIB_LEN 8
#define DC_CALIBED_DATA 0
#define DC_RAW_DATA 1

//2014.11.20 added
//

#define CMD_DELAY             16

#define WAIT_NONE   (0)
#define WAIT_CMD    (1)
#define WAIT_RESET  (2)
#define WAIT_DCMAP  (3)

#define MAX_16BIT			0xffff
#define MAX_12BIT			0xfff

#define NUM_DRIVE (31)
#define NUM_SENSE (18)
#define MAX_FRAME_SIZE (NUM_DRIVE * NUM_SENSE)

#define MAX_DCMAP_SIZE ( NUM_DRIVE * (NUM_SENSE + 2 ) * 2 ) // 1240 = 31 * 20 * 2

#define LOW_LEVEL 0
#define HIGH_LEVEL 1

#define MAX_COMMAND_RESULT_LEN (64-8)

#define SHTSC_DEVBUF_SIZE 1500

#define ADDR_ID_JIG_4REN 0x1F300
#define LEN_ID_JIG_4REN 10
#define ADDR_ID_JIG_LPC 0x1F100
#define LEN_ID_JIG_LPC 9
#define TYPE_ID_JIG_4REN 1
#define TYPE_ID_JIG_LPC 2

//#define FLASH_WAIT 500
//#define FIRMWARE_SIZE (44*1024)

#define KNOCKDATA_SIZE (4*12) // 20151105 for maximum 12 taps for knock-code

/* KNOCK ON/CODE FAILURE STATUS */
#define SHTSC_KNOCKON_FAILURE_DISTANCE_INTER_TAP    (1<<0)
#define SHTSC_KNOCKON_FAILURE_DISTANCE_TOUCHSLOP    (1<<1)
#define SHTSC_KNOCKON_FAILURE_TIMEOUT_INTERTAP      (1<<2)
#define SHTSC_KNOCKON_FAILURE_MULTI_FINGER          (1<<3)
#define SHTSC_KNOCKON_FAILURE_DELAY_TIME            (1<<4)
#define SHTSC_KNOCKON_FAILURE_PALM_STATE            (1<<5)

#define KNOCK_CODE_FAILURE_REASONS (\
		SHTSC_KNOCKON_FAILURE_DISTANCE_INTER_TAP | \
		SHTSC_KNOCKON_FAILURE_DISTANCE_TOUCHSLOP | \
		SHTSC_KNOCKON_FAILURE_TIMEOUT_INTERTAP   | \
		SHTSC_KNOCKON_FAILURE_MULTI_FINGER       | \
		SHTSC_KNOCKON_FAILURE_DELAY_TIME         | \
		SHTSC_KNOCKON_FAILURE_PALM_STATE         )


/* Software reset delay */
#define SHTSC_RESET_TIME	250	/* msec */
#define SHTSC_SLEEP_TIME	100	/* msec */

/* ======== EEPROM command ======== */
#define	FLASH_CMD_WRITE_EN		(0x06)		/* Write enable command */
#define	FLASH_CMD_PAGE_WR			(0x02)		/* Page write command */
#define	FLASH_CMD_READ_ST			(0x05)		/* Read status command */
#define	FLASH_CMD_SECTOR_ERASE	(0xD7)		/* Sector erase command */
#define	FLASH_CMD_READ			(0x03)		/* Read command */

/* ======== EEPROM status ======== */
#define	FLASH_ST_BUSY		(1 << 0)	/* Busy with a write operation */

#define CMD_SETSYSTEMSTATE_LEN 5
#define CMD_SETSYSTEMSTATE "\x02\x00\x01\x00\xff"

/* ======== Version information  ======== */

#define DEVICE_CODE_LR388K6 2

#define VERSION_YEAR 15
#define VERSION_MONTH 6
#define VERSION_DAY 12
#define VERSION_SERIAL_NUMBER 20
#define VERSION_MODEL_CODE DEVICE_CODE_LR388K6
#define DRIVER_VERSION_LEN 5

#define DRIVER_NAME "shtsc"
/* ======= SelfDiagnosis ====== */
#define RESULT_DATA_MAX_LEN (48 * 1024)
#define TIME_DATA_MAX_LEN 64
#define FILE_PATH_RESULT "/data/logger/touch_self_test.txt"


/* FLASH TIME IMPROVEMENT FEATURE */
#define FLASH_TIME_ZERO_WAIT
#define FLASH_MULTI_WRITE
#define FLASH_NO_VERIFY

#ifdef FLASH_TIME_ZERO_WAIT
#define FLASH_WAIT 0
#else
#define FLASH_WAIT 500
#endif

#define MAX_LOG_FILE_SIZE		(10 * 1024 * 1024)      /* 10 MBytes */
#define MAX_LOG_FILE_COUNT		(4)

#define CALIBRATION_DATA_SRAM_ADDRESS	0x1B400
#define CALIBRATION_DATA_FLASH_ADDRESS	0x1D000
#define CALIBRATION_DATA_FLASH_SIZE	0x2000

#define CALIB_CHECKSUM_SIZE	3
#define CALIB_PAD_SIZE		0 // 1
#define CALIB_FOOTER_SIZE	25
#define CALIB_HEADER_SIZE	8

#define CALIB_TOOL_TYPE_LPC		1
#define CALIB_TOOL_TYPE_B6		2
#define CALIB_TOOL_TYPE_UPDCAL		3
#define CALIB_TOOL_TYPE_DRIVER		6
#define CALIB_TOOL_TYPE_NONE		0xFF

#define CALIB_RANGE			(4096)

#define SPEC_FACTORY_CAL_DIFF_CENTER_ROOM_TEMP				250 // Th :  500
#define SPEC_FACTORY_CAL_DIFF_EDGE_ROOM_TEMP				450 // Th : 1300
#define SPEC_FACTORY_CAL_DIFF_CENTER_HIGH_LOW_TEMP			450
#define SPEC_FACTORY_CAL_DIFF_EDGE_HIGH_LOW_TEMP			1000

#define BATT_TEMP_N_10									(-100)
#define BATT_TEMP_10									100
#define BATT_TEMP_40									400
#define BATT_TEMP_55									550

#define REPEAT_DIFF_SPEC			25
#define SPEC_DELTA_DIFF				200 // Pen Threshold 250
#define SPEC_DELTA_DIFF_MFTS		67
#define SPEC_LPWG_DELTA_MAX			250
#define DELTA_READ_REPEAT		    1

#define SPEC_LPWG_RAWDATA_MAX		350
#define SPEC_LPWG_JITTER_MAX		250

#define SPEC_DC_JITTER_MAX			260
#define JITTER_READ_REPEAT			6

#define MAX_CH_LINE_NUM		38
#define MAX_TOTAL_LINE_NUM	58

#define D_REG_HOST_AP_FULL 	    (0x20) //D_REG_BANK_18+
#define D_REG_IDLE_BREAK_THRESH_DC0 (0x72) //D_REG_BANK_12+
#define D_REG_IDLE_BREAK_THRESH_DC1 (0x73) //D_REG_BANK_12+
#define D_SENSE_DATA_NUM (16)

#define CMD_EXECCALIBRATION_LPWG_WRITE "\x0F\x00\x03\x00\x00\x2c\x01" //300ms

#define CMD_SETREGTBL_IDLE_BRK_TRSH_DC0  "\x06\x00\x04\x00\x09\x12\x72\xFF"
#define CMD_SETREGTBL_IDLE_BRK_TRSH_DC1  "\x06\x00\x04\x00\x09\x12\x73\x7F"
#define CMD_SETREGTBL_IDLE_BRK_TRSH_LEN 8

#define CMD_GETREGTBL_IDLE_BRK_TRSH_DC  "\x07\x00\x03\x02\x09\x12\x72"
#define CMD_GETREGTBL_IDLE_BRK_TRSH_LEN 7

#if 0 //call before LPWG setting

int lgtp_set_deepidle_test_flag = 0;
unsigned char lgtp_old_idle_brk_thsh_d0 = 0;
unsigned char lgtp_old_idle_brk_thsh_d1 = 0;

void lgtp_save_deepidle_val(void)
{
	TOUCH_FUNC();

	issue_command_wo_IRQ(&g_ts, CMD_GETREGTBL_IDLE_BRK_TRSH_DC, CMD_GETREGTBL_IDLE_BRK_TRSH_LEN);

	lgtp_old_idle_brk_thsh_d0 = CommandResultBuf[2];
	lgtp_old_idle_brk_thsh_d1 = CommandResultBuf[3];
	TOUCH_LOG("lgtp_old_idle_brk_thsh_d 0x%02x, 0x%02x\n", lgtp_old_idle_brk_thsh_d0, lgtp_old_idle_brk_thsh_d1);

	return;
}

void lgtp_restore_deepidle_val(void)
{
	unsigned char cmd[9] = {0x06, 0x00, 0x05, 0x00, 0x09, 0x12, 0x72, 0x80, 0x01};

	TOUCH_FUNC();

	cmd[7] = lgtp_old_idle_brk_thsh_d0;
	cmd[8] = lgtp_old_idle_brk_thsh_d1;

	TOUCH_LOG("lgtp_old_idle_brk_thsh_d 0x%02x, 0x%02x\n", lgtp_old_idle_brk_thsh_d0, lgtp_old_idle_brk_thsh_d1);

	issue_command_wo_IRQ(&g_ts, cmd, CMD_SETREGTBL_IDLE_BRK_TRSH_LEN);

	return;
}
#endif

/****************************************************************************
 * Macros
 ****************************************************************************/
#define DBGLOG TOUCH_LOG

#define TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG 1

//#define DEBUG_SHTSC 0

#define WRITE_RESULT(_desc, _size, fmt, args...) do {					\
	_size += snprintf(_desc + _size, RESULT_DATA_MAX_LEN  - _size, fmt, ##args);	\
} while (0)


/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef enum _cmd_state_e {
	CMD_STATE_SLEEP         = 0x00,
	CMD_STATE_IDLE          = 0x03,
	CMD_STATE_DEEP_IDLE     = 0x04,
	CMD_STATE_HOVER         = 0x06,  // if applicable
	CMD_STATE_PROX          = 0x07,  // if applicable
	CMD_STATE_GLOVE         = 0x08,  // if applicable
	CMD_STATE_COVER         = 0x0D,  // if applicable
	CMD_STATE_MAX
} dCmdState_e ;

/*
 * The touch driver structure.
 */
struct shtsc_touch {
	u8 status;
	u8 id;
	u8 major;
	u8 minor;
	u8 type;
	u16 x;
	u16 y;
	u8 z;
};


struct shtsc_i2c {
	u8 cmd;//2014.11.19 added
	u8 wait_state;//2014.11.19 added
	bool wait_result;//2014.11.19 added
	u8 disabled;		/* interrupt status */ //2014.11.6 added
	struct input_dev *input;
	struct shtsc_i2c_pdata *pdata;//2014.10.16 added
	char phys[32];
	struct i2c_client *client;
	int reset_pin;
	int irq_pin;
	struct shtsc_touch touch[SHTSC_MAX_FINGERS];
	struct mutex mutex_irq;
	//#ifdef FORCE_FIRM_UPDATE
	struct work_struct workstr;
	//struct workqueue_struct *wq_reset_check;
	//struct delay_work *work_reset_check;

	//#endif /* FORCE_FIRM_UPDATE */
	LpwgSetting lpwgSetting;

	unsigned int            max_num_touch;
	int                     min_x;
	int                     min_y;
	int                     max_x;
	int                     max_y;
	int                     pressure_max;
	int                     touch_num_max;
	bool                    flip_x;
	bool                    flip_y;
	bool                    swap_xy;
};

/****************************************************************************
 * Variables
 ****************************************************************************/
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2216_p1222E_151221_ext_h.bin";
// 1222 MSM8937 and PP1st 1.47
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_fF3012216_p1222F_151222_ext_h.bin";
// 1224 MSM8937 and PP1st 1.48
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2218_p12230_151223_ext_h.bin";
// 0111 Apply Palm & Pen Tuning
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2221_p12234_160110_ext_h.bin";
// 0113 Change Raw Data Spec.
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2223_p12236_160113_ext_h.bin";
// 0115 Change Raw Data Spec. and TA simulator noise improvement
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2224_p12237_160114_ext_h.bin";
// 0115 Change FW check list, Knock-on, Knock-code bug fixed
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2227_p1223A_160117_ext_h.bin";
// 0119 Remove 1 second wait after LPWG fail 1.59
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2228_p1223B_160118_ext_h.bin";
// 0122 Fix Initial calibration issue(Green mat / Metal board), Proxi Sensor LPWG issue 1.63
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f222C_p1223F_160121_ext_h.bin";
// 0123 remove debugging scheme V1.64
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f222D_p12240_160123_ext_h.bin";
// 0210 bug fix of force calibration data read, delta data read and LPWG2 state transition V1.66
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f222F_p12242_160209_ext_h.bin";
// 0211 TA noise function improved V1.68
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2231_p12244_160210_ext_h.bin";
// 0220 Add stage 4(dc level less than 75) V1.71
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2235_p12247_160220_ext_h.bin";
// 0223 Drawing Test improved V1.72 for DV1
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2236_p12248_160223_ext_h.bin";
// 0305 Calibration process changed, Line bending of manual drawing issue fixed, Real TA noise ghost V1.74
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2228_p1224A_160304_ext_h.bin";
// 0311 improve green mat noise, add new auto calibration stage for edge calibration recovery V1.83
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2241_p12253_160311_ext_h.bin";
// 0314 Change LPWG ignore sense line to 1 sense line, improve 5mm slug separation V1.85 for DV2
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2243_p12255_160314_ext_h.bin";
// 0315 FW 1.87 Calibration stage & Rawdata spec update
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2246_p12257_160316_ext_h.bin";
// 0318 FW 1.90 For VP4
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f2249_p1225A_160318_ext_h.bin";
// 0325 FW 1.92 For DV3
//static const char defaultFirmware[] = "sharp/ph2/LR388K6_f224D_p1225C_160325_ext_h.bin";
// 0403 FW 1.93 For QP1
static const char defaultFirmware[] = "sharp/ph2/LR388K6_f224E_p1225D_160401_ext_h.bin";

int	G_reset_done = false;
u16 G_Irq_Mask = 0xffff;
unsigned G_touch=0;

unsigned char resumeStatus; // bank 0, address 9

unsigned char CommandResultBuf[MAX_COMMAND_RESULT_LEN];

// LPWG Delta testing spec for ABSMAX(Delta)  (values are temporary for now)
unsigned Lpwg_spec_upper[NUM_SENSE-1] = {
	1000, // Sense 0
	1000, // Sense 1
	1000, // Sense 2
	1000, // Sense 3
	1000, // Sense 4
	1000, // Sense 5
	1000, // Sense 6
	1000, // Sense 7
	1000, // Sense 8
	1000, // Sense 9
	1000, // Sense 10
	1000, // Sense 11
	1000, // Sense 12
	1000, // Sense 13
	1000, // Sense 14
	1000, // Sense 15
	1000, // Sense 16
};

u8 dcmapBuf[MAX_DCMAP_SIZE+128];
u8 dcmap[MAX_DCMAP_SIZE+128]; // greater than 17*32*2


volatile static int buf_pos;
static u8 devbuf[SHTSC_DEVBUF_SIZE];

struct shtsc_i2c g_ts;

#define REP_SIZE (4+4+4+120)
#define MAX_REPORTS 14

unsigned char reportBuf[4+ MAX_REPORTS*REP_SIZE];
volatile unsigned char Mutex_GetReport = false;

int current_page = 0;

#define IO_BUF_SIZE 4*1024
char iobuf[IO_BUF_SIZE];

#define SHTSC_BUF_SIZE 16*1024
u8 s_shtsc_addr;
u8 s_shtsc_buf[SHTSC_BUF_SIZE];
unsigned long s_shtsc_len;

pid_t pid = 0;
#define TOTAL_CHANNAL (NUM_DRIVE * NUM_SENSE)
#define UPPER_DATA 0
#define LOWER_DATA 1

int lpwg_fail_reason = 0;
unsigned int lpwg_fail_reason_mask = KNOCK_CODE_FAILURE_REASONS ;
#define SIZE_KNOCKDATA_BUF (KNOCKDATA_SIZE + 16)
u8 knockData_buf[SIZE_KNOCKDATA_BUF];

// register control
#define SIZE_REG_COMMON_BUF 16
u8 regcommonbuf[SIZE_REG_COMMON_BUF];

// irq thread temp buf
#define SIZE_IRQ_THREAD_TEMP_BUF 16
u8 irq_thread_tmpbuf[SIZE_IRQ_THREAD_TEMP_BUF];

// TOUCH Report buffer
#define SIZE_TOUCH_REPORT_BUF 128
u8 touchReportBuf[SIZE_TOUCH_REPORT_BUF];

TouchFingerData release_fingerData[MAX_FINGER+1];

#define UNIQUE_ID_BUFFER_MAX 32
unsigned char LCM_unique_id_Buf[UNIQUE_ID_BUFFER_MAX];
unsigned int unique_id_saved = 0;

// firmware version
unsigned K6_firmver = 0;
unsigned K6_paramver = 0;

int ta_connection_status = 0;
int q_cover_status = 0;

int debug_app_enable = 0;

int s_last_lpwg_data[18];
int s_last_lpwg_max_diff[18];

int s_last_dc_data[31][18];
int s_last_dc_max_diff[31][18];

int s_open_short_map[31][18];
int s_open_short_result[18];
int s_open_short_diff_tx_map[31][18];
int s_open_short_diff_tx_result[18];

int lgtp_set_deepidle_test_flag = 0;
unsigned char lgtp_old_idle_brk_thsh_d0 = 0;
unsigned char lgtp_old_idle_brk_thsh_d1 = 0;

//====================================================================
// NORMAL : general touch(finger,key) is working
// OFF : touch is not working even knock-on ( lowest power saving )
// KNOCK_ON_ONLY : knock-on is only enabled
// KNOCK_ON_CODE : knock-on and knock-code are enabled
// NORMAL_HOVER : hover detection is enabled and general touch is working
// HOVER : only hover detection is enabled
//====================================================================
static TouchState dummyDeviceState = STATE_UNKNOWN;

struct workqueue_struct *wq_reset_check = NULL;
struct delayed_work work_reset_check;
int reset_again_count = 0;

struct workqueue_struct *wq_folio_cover_check = NULL;
struct delayed_work work_folio_cover_check;
/****************************************************************************
 * Extern Function Prototypes
 ****************************************************************************/

extern int touch_ref_batt_temp;

/****************************************************************************
 * Local Function Prototypes
 ****************************************************************************/
int flash_access_start_shtsc(void *_ts);
int flash_access_end_shtsc(void *_ts);
int flash_erase_page_shtsc(void *_ts, int page);
int flash_write_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data);
int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data);
static int WriteMultiBytes(void *ts, u8 u8Addr, u8 *data, int len);
static int WriteOneByte(void *ts, u8 u8Addr, u8 u8Val);
static u8 ReadOneByte(void *ts, u8 u8Addr);
static void ReadMultiBytes(void *ts, u8 u8Addr, u16 u16Len, u8 *u8Buf);
static int tci_control(struct i2c_client *client, int type, unsigned int value);
int issue_command(void *ts, unsigned char *cmd, unsigned int len);
int issue_command_wo_IRQ(void *ts, unsigned char *cmd, unsigned int len);
int issue_command2(void *ts, unsigned char *cmd, unsigned int len);
int issue_command2_wo_IRQ(void *ts, unsigned char *cmd, unsigned int len);
int lpwg_param_command(void *ts, u8 u8Addr, unsigned char *cmd, unsigned int len);

static void LR388K6_Reset(struct i2c_client *client);
static int lr388k6_misc_open(struct inode *inode, struct file *file);
static int lr388k6_misc_release(struct inode *inode, struct file *file);
static ssize_t lr388k6_misc_read(struct file *file, char *buf, size_t count, loff_t *pos);
static long lr388k6_misc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int WaitAsync(void *_ts);
//char * LR388K6_property(void);

static int LR388K6_UpdateFirmware(struct i2c_client *client, char *pFilename);
int shtsc_CMD_getDCMap_Jitter(struct i2c_client *client, void *ts, int rawflag, int frame_num, char* buf, int* pJitterMax);
static int checkDCJitter(struct i2c_client *client, char *buf, int *pDataLen, int count, int frame_num, int*pJitterMax);


/****************************************************************************
 * Local Functions
 ****************************************************************************/


/****************************************************************************
 * Device Specific Functions
 ****************************************************************************/
static int get_spec_factory_cal_diff(struct i2c_client *client, int d, int s){
	TouchDriverData *pDriverData = NULL;
	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}

	if ((s == 0) || (s == 16) || (d == 0) || (d == 30)) {
		// Edge area
		if( touch_ref_batt_temp < BATT_TEMP_10 || touch_ref_batt_temp > BATT_TEMP_40){
			return SPEC_FACTORY_CAL_DIFF_EDGE_HIGH_LOW_TEMP;
		} else {
			return SPEC_FACTORY_CAL_DIFF_EDGE_ROOM_TEMP;
		}

	} else {
		// Center area
		if( touch_ref_batt_temp < BATT_TEMP_10 || touch_ref_batt_temp > BATT_TEMP_40){
			return SPEC_FACTORY_CAL_DIFF_CENTER_HIGH_LOW_TEMP;
		} else {
			return SPEC_FACTORY_CAL_DIFF_CENTER_ROOM_TEMP;
		}
	}
}

static int get_spec_delta_diff(struct i2c_client *client){
	TouchDriverData *pDriverData = NULL;
	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}
	if(pDriverData != NULL && pDriverData->bootMode == BOOT_MFTS){
		return SPEC_DELTA_DIFF_MFTS;
	}
	return SPEC_DELTA_DIFF;
}

static int get_current_time(char * buf)
{
	struct timespec my_time;
	struct tm my_date;

	if(buf == NULL)
	{
		TOUCH_ERR("buf is NULL.\n");
		return TOUCH_FAIL;
	}

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(buf, 64, "%02d-%02d %02d:%02d:%02d.%03lu ",
			my_date.tm_mon + 1, my_date.tm_mday,
			my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);

	TOUCH_LOG("CURRENT TIME : %s\n", buf);

	return TOUCH_SUCCESS;
}

static void log_file_size_check(char* filename)
{
	//char *filename = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;

	set_fs(KERNEL_DS);

	if (filename) {
		file = filp_open(filename, O_RDONLY, 0666);
	} else {
		TOUCH_ERR("%s : filename is NULL, can not open FILE\n",
				__func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_LOG("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), filename);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_LOG("%s : [%s] file_size = %lld\n",
			__func__, filename, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_LOG("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, filename, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", filename);
			else
				sprintf(buf1, "%s.%d", filename, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_LOG("%s : file [%s] exist\n", __func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_ERR(
								"%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto error;
					}

					TOUCH_LOG(
							"%s : remove file [%s]\n",
							__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d", filename,
							(i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_ERR(
								"%s : failed to rename file [%s] -> [%s]\n",
								__func__, buf1, buf2);
						goto error;
					}

					TOUCH_LOG(
							"%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_LOG("%s : file [%s] does not exist (ret = %d)\n", __func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
	return;
}

static int write_result_data_to_file(char* filename, char *data, int datalen){
	int fd = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(filename == NULL) {
		fd = sys_open(FILE_PATH_RESULT, O_WRONLY | O_CREAT | O_APPEND, 0666);
		TOUCH_LOG(" SelfD Test default result file has been opened fd %d size %d \n", fd, datalen);
	} else {
		//fd = sys_open(filename, O_WRONLY | O_CREAT , 0666);
		fd = sys_open(filename, O_WRONLY | O_CREAT | O_APPEND, 0666);
		if(fd == 0 ){
			TOUCH_ERR(" SelfD Test result file %s has not been created datalen %d\n", filename, datalen);
			set_fs(old_fs);
			return 0;
		} else {
			TOUCH_LOG(" SelfD Test result file %s has been opened fd %d dataLen %d\n", filename, fd, datalen);
		}
	}
	if( fd > 0 ) {
		sys_write(fd, data, datalen);
		sys_close(fd);
	}
	set_fs(old_fs);

	return datalen;
}


static int shtsc_system_init(void *ts)
{
	//struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;

	TOUCH_FUNC();

	// NOT USED any more
	return 0;

#if 0
	{
		unsigned int cnt;
		for(cnt=0;cnt<SHTSC_MAX_FINGERS;cnt++){
			input_mt_slot(input_dev, cnt);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("shtsc_system_init() clear mt_slot(%d)\n",cnt);
#endif
		}
	}
	input_report_key(input_dev, BTN_TOUCH, false );

	input_sync(input_dev);
	return 0;
#endif

}

#if 0
static void shtsc_reset_delay(void)
{
	msleep(SHTSC_RESET_TIME);
}

static void shtsc_reset(void *_ts, bool reset)
{
	struct shtsc_i2c *ts = _ts;

	TOUCH_FUNC();

	if (ts->reset_pin) {
		G_reset_done = reset;
		if (reset) {
			shtsc_reset_delay();

		}
		//TOUCH_LOG("shtsc: shtsc_reset: %d\n", reset);
		if(reset = false){
			TouchResetCtrl(0);
			G_Irq_Mask = 0xffff;
		} else {
			TouchResetCtrl(1);
		}

		gpio_direction_output(ts->reset_pin, reset);
		if (! reset) {
			G_Irq_Mask = 0xffff;
		}
	}
}

#endif

static int WriteMultiBytes(void *_ts, u8 u8Addr, u8 *data, int len)
{
	struct shtsc_i2c *ts = _ts;

	return Touch_I2C_Write(ts->client, u8Addr, data, len);
}

static int WriteOneByte(void *_ts, u8 u8Addr, u8 u8Val)
{
	struct shtsc_i2c *ts = _ts;
	u8 wData[1];
	wData[0] = u8Val;

	if( (ts == NULL) ){
		TOUCH_LOG("ts is NULL error\n");
		return 0;
	}

	return Touch_I2C_Write_Byte(ts->client, u8Addr, u8Val);
}

#if defined(DEBUG_SHTSC)
int testingStart = 0;
int bankAddr = 0;
int readOncebankAddr = 0;
#endif

static u8 ReadOneByte(void *_ts, u8 u8Addr)
{
	struct shtsc_i2c *ts = _ts;
	u8 rData[1+1]={0,}; //requires one more byte to hold

#if defined(DEBUG_SHTSC)
	if( testingStart == 1){
		if(bankAddr != readOncebankAddr){
			TOUCH_LOG("BankAddr %d \n", bankAddr);
			readOncebankAddr = bankAddr;
		}
	}
#endif

	Touch_I2C_Read_Byte(ts->client, u8Addr, rData);
	return rData[0];
}

static void ReadMultiBytes(void *_ts, u8 u8Addr, u16 u16Len, u8 *u8Buf)
{
	struct shtsc_i2c *ts = _ts;

	if( (ts == NULL) || (ts->client == NULL) || (u8Buf == NULL) ){
		TOUCH_LOG("ts is NULL or client is NULL or u8Buf is NULL error\n");
		return ;
	}

#if defined(DEBUG_SHTSC)
	if( testingStart == 1){
		if(bankAddr != readOncebankAddr){
			TOUCH_LOG("BankAddr %d \n", bankAddr);
			readOncebankAddr = bankAddr;
		}
	}
#endif

	Touch_I2C_Read(ts->client, u8Addr, u8Buf, u16Len);
}

static int SetBankAddr(void *_ts, u8 u8Bank)
{
	int ret = TOUCH_FAIL;
	struct shtsc_i2c *ts = _ts;

	if( (ts == NULL) ){
		TOUCH_LOG("ts is NULL error\n");
		return ret;
	}

#if defined(DEBUG_SHTSC)
	if( testingStart == 1){
		TOUCH_LOG("SetBankAddr %d \n", u8Bank);
	}
#endif

	ret = WriteOneByte(ts, SHTSC_ADDR_BANK, u8Bank);

#if defined(DEBUG_SHTSC)
	bankAddr = u8Bank;
#endif

	return ret;
}

static void Sharp_ClearInterrupt(void *_ts, u16 u16Val)
{
	struct shtsc_i2c *ts = _ts;

	//TOUCH_FUNC();
#if defined(DEBUG_SHTSC)
	TOUCH_LOG(" Sharp_ClearInterrupt %04X \n", u16Val);
#endif

	if(u16Val & 0x00FF)
		WriteOneByte(ts, SHTSC_ADDR_INT0, (u16Val & 0x00FF));
	if((u16Val & 0xFF00) >> 8)
		WriteOneByte(ts, SHTSC_ADDR_INT1, ((u16Val & 0xFF00) >> 8));
}

static void SetIndicator(void *_ts, u8 u8Val)
{
	struct shtsc_i2c *ts = _ts;

	WriteOneByte(ts, SHTSC_ADDR_IND, u8Val);
}


int flash_access_start_shtsc(void *_ts)
{
	struct shtsc_i2c *ts = _ts;

	// mask everything
	WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	msleep(100);

	/* TRIM_OSC = 0 */
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x18); // bank change reg
	WriteOneByte(ts, (unsigned char)0x11, (unsigned char)0x19);
	return 0;
}

int flash_access_end_shtsc(void *_ts)
{
	struct shtsc_i2c *ts = _ts;

	TOUCH_FUNC();

	msleep(100);

	TouchResetCtrl(0);
	G_Irq_Mask = 0xffff;

	//shtsc_reset(ts, false);

	msleep(100);

	//shtsc_reset(ts, true);
	TouchResetCtrl(1);

	msleep(10);
	shtsc_system_init(ts);

	msleep(100);

	return 0;
}

#define RETRY_COUNT (2000*10) //experimental
int flash_erase_page_shtsc(void *_ts, int page)
{
	struct shtsc_i2c *ts = _ts;

	//  int chan= 0;
	volatile unsigned char readData;
	int retry=0;

	/* BankChange */
	//SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL CS_HIGH,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

	/* FLC_CTL CS_LOW,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x16);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
	/* FLC_TxDATA WRITE_ENABLE */
	//SPI_ArrieRegWrite(0x3D,SPI_CMD_WRITE_EN);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)FLASH_CMD_WRITE_EN);
	/* FLC_CTL CS_HIGH,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

	/* FLC_CTL CS_LOW,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x16);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);


	/* FLC_TxDATA CHIP_ERASE_COMMAND */
	//	SPI_ArrieRegWrite(0x3D,SPI_CMD_CHIP_ERASE);
	// not a chip erase, but a sector erase for the backward compatibility!!
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)FLASH_CMD_SECTOR_ERASE);
	// 24bit address. 4kByte=001000H. 00x000H:x=0-f -> 4kB*16=64kB
	// for K6 or more than 64kB, 0(0x page)000 : page:00H-1FH for 128kB
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((page>>4)&0xFF));//20150912
	// K5 code below
	//WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((page<<4)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	/* FLC_CTL CS_HIGH,WP_DISABLE */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);


	/* wait until 'BUSY = LOW' */
	do {
		retry++;
		////		msleep(10);
		if (retry > RETRY_COUNT)
			goto RETRY_ERROR;

		/* FLC_CTL CS_LOW,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x16);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
		/* FLC_TxDATA READ_STATUS_COMMAND*/
		//		SPI_ArrieRegWrite(0x3D,SPI_CMD_READ_ST);
		WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ_ST);
		/* Dummy data */
		//		SPI_ArrieRegWrite(0x3D,0);
		WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
		/* FLC_RxDATA */
		//		readData = SPI_ArrieRegRead(0x3F);
		readData = ReadOneByte(ts, 0x3F);
		/* FLC_CTL CS_HIGH,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x14);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);
	} while (readData & FLASH_ST_BUSY); 		/* check busy bit */

	return 0;

RETRY_ERROR:
	TOUCH_LOG("FATAL: flash_erase_page_shtsc retry %d times for page %d - FAILED!\n", retry, page);
	return 1;
}
#define FLASH_PAGE_SIZE (4<<10) // 4k block for each page
#define FLASH_PHYSICAL_PAGE_SIZE (256) // can write 256bytes at a time.

int flash_write_page_shtsc(void *_ts, int page, unsigned char *data)
{
	struct shtsc_i2c *ts = _ts;

	//  int chan = 0;
	int retry = 0;
	//  unsigned addr; // in-page address (from 0 to FLASH_PAGE_SIZE-1)
	unsigned paddr; // address (32 or 64kB area)
	volatile unsigned char readData;
	int cnt, idx;

	/* BankChange */
	//	SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL */
	//	SPI_ArrieRegWrite(0x3C,0x14);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

	/* 256 bytes / Flash write page, 4kByte / logical(virtual) flash page */
	for (cnt = 0; cnt < (FLASH_PAGE_SIZE / FLASH_PHYSICAL_PAGE_SIZE); cnt++) {
		paddr = (page * FLASH_PAGE_SIZE) + (cnt * FLASH_PHYSICAL_PAGE_SIZE);
		// 4k page offset + in-page offset. 4k*n+256*m

		/* FLC_CTL CS_LOW,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x16);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
		/* FLC_TxDATA WRITE_ENABLE */
		//		SPI_ArrieRegWrite(0x3D,SPI_CMD_WRITE_EN);
		WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_WRITE_EN);
		/* FLC_CTL CS_HIGH,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x14);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);

		/* FLC_CTL CS_LOW,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x16);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);

		/* FLC_TxDATA PAGE_PROGRAM_COMMAND */
		//		SPI_ArrieRegWrite(0x3D,SPI_CMD_PAGE_WR);
		WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_PAGE_WR);
		/* FLC_TxDATA Address(bit16~23) */
		//		SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
		WriteOneByte(ts, (unsigned char)0x3D, ((paddr>>16)&0xFF));
		/* FLC_TxDATA Address(bit8~15) */
		//		SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
		WriteOneByte(ts, (unsigned char)0x3D, ((paddr>>8)&0xFF));
		/* FLC_TxDATA Address(bit0~7) */
		//		SPI_ArrieRegWrite(0x3D,(address&0xFF));
		WriteOneByte(ts, (unsigned char)0x3D, (paddr&0xFF));

#ifdef FLASH_MULTI_WRITE
		{
			// random access mode to speed up

			char tbuf[512];
			for(idx=0;idx<256;idx++){
				tbuf[idx*2] = (0x80 | 0x3D);
				tbuf[idx*2+1] = data[(cnt*FLASH_PHYSICAL_PAGE_SIZE) +idx];
			}
			WriteMultiBytes(ts,tbuf[0],&(tbuf[1]),511);
		}
#else
		/* Data write 1page = 256byte */
		for(idx=0;idx<256;idx++){
			//			SPI_ArrieRegWrite(0x3D,*pData++);
			// addr=in-page(virtual, in 4k block) 256xN
			WriteOneByte(ts, (unsigned char)0x3D, data[(cnt*FLASH_PHYSICAL_PAGE_SIZE) +idx]);
		}
#endif

		/* FLC_CTL CS_HIGH,WP_DISABLE */
		//		SPI_ArrieRegWrite(0x3C,0x14);
		WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);


		/* wait until 'BUSY = LOW' */
		do {
			retry++;
			////		  msleep(10);
			if (retry > RETRY_COUNT)
				goto RETRY_ERROR;

			/* FLC_CTL CS_LOW,WP_DISABLE */
			//		SPI_ArrieRegWrite(0x3C,0x16);
			WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x16);
			/* FLC_TxDATA READ_STATUS_COMMAND*/
			//		SPI_ArrieRegWrite(0x3D,SPI_CMD_READ_ST);
			WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ_ST);
			/* Dummy data */
			//		SPI_ArrieRegWrite(0x3D,0);
			WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
			/* FLC_RxDATA */
			//		readData = SPI_ArrieRegRead(0x3F);
			readData = ReadOneByte(ts, 0x3F);
			/* FLC_CTL CS_HIGH,WP_DISABLE */
			//		SPI_ArrieRegWrite(0x3C,0x14);
			WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x14);
		} while (readData & FLASH_ST_BUSY); 		/* check busy bit */
	}

	return 0;

RETRY_ERROR:
	TOUCH_LOG("FATAL: flash_write_page_shtsc retry %d times for page %d, addr %04X - FAILED!\n", retry, page, paddr);
	return 1;
}

#define FLASH_VERIFY_SIZE 512
unsigned char readBuf[FLASH_VERIFY_SIZE];

int flash_verify_page_shtsc(void *_ts, int page, unsigned char *data)
{
	struct shtsc_i2c *ts = _ts;
	unsigned addr; // in-page address (from 0 to FLASH_PAGE_SIZE-1)
	int cnt;

	/* BankChange */
	//	SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL CS_HIGH,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x12);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x12);

	/* FLC_TxDATA READ_COMMAND*/
	//	SPI_ArrieRegWrite(0x3D,SPI_CMD_READ);
	WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ);
	/* FLC_TxDATA Address(bit16~23) */
	//	SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));

	WriteOneByte(ts, (unsigned char)0x3D, ((page>>4)&0xFF)); //20150912
	//	K5 code below
	//WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	/* FLC_TxDATA Address(bit8~15) */
	//	SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, ((page<<4)&0xFF));
	/* FLC_TxDATA Address(bit0~7) */
	//	SPI_ArrieRegWrite(0x3D,(address&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);
	/* FLC_TxDATA Dummy data */
	//	SPI_ArrieRegWrite(0x3D,0);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	for (addr = 0; addr < FLASH_PAGE_SIZE; addr += FLASH_VERIFY_SIZE) {
		for(cnt=0; cnt<FLASH_VERIFY_SIZE; cnt++){
			/* FLC_RxDATA */
			//		*pData++ = SPI_ArrieRegRead(0x3F);
			readBuf[cnt] = ReadOneByte(ts, 0x3F);
		}
		if (memcmp((unsigned char *)&(data[addr]), (unsigned char *)readBuf, FLASH_VERIFY_SIZE)) {
			goto VERIFY_ERROR;
		}
	}

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	return 0;

VERIFY_ERROR:
	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);
	// verify error
	TOUCH_LOG("FATAL: flash_verify_page_shtsc for page %d - FAILED!\n", page);

	return 1;
}

int flash_read(void *_ts, unsigned address, unsigned length, unsigned char *data)
{
	struct shtsc_i2c *ts = _ts;
	int cnt;

	/* BankChange */
	//	SPI_ArrieRegWrite(0x02,0x1C);
	WriteOneByte(ts, (unsigned char)0x02, (unsigned char)0x1C); // bank change reg

	/* CLKON_CTL0 */
	//	SPI_ArrieRegWrite(0x14,0x22);	/* CLKON_FLC,CLKON_REGBUS */
	WriteOneByte(ts, (unsigned char)0x14, (unsigned char)0x22);

	/* FLC_CTL CS_HIGH,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x12);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x12);

	/* FLC_TxDATA READ_COMMAND*/
	//	SPI_ArrieRegWrite(0x3D,SPI_CMD_READ);
	WriteOneByte(ts, (unsigned char)0x3D, FLASH_CMD_READ);
	/* FLC_TxDATA Address(bit16~23) */
	//	SPI_ArrieRegWrite(0x3D,((address>>16)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((address>>16)&0xFF));
	/* FLC_TxDATA Address(bit8~15) */
	//	SPI_ArrieRegWrite(0x3D,((address>>8)&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)((address>>8)&0xFF));
	/* FLC_TxDATA Address(bit0~7) */
	//	SPI_ArrieRegWrite(0x3D,(address&0xFF));
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)(address&0xFF));
	/* FLC_TxDATA Dummy data */
	//	SPI_ArrieRegWrite(0x3D,0);
	WriteOneByte(ts, (unsigned char)0x3D, (unsigned char)0x00);

	for(cnt=0; cnt<length; cnt++){
		/* FLC_RxDATA */
		//		*pData++ = SPI_ArrieRegRead(0x3F);
		data[cnt] = ReadOneByte(ts, 0x3F);
	}

	/* FLC_CTL CS_LOW,WP_ENABLE */
	//	SPI_ArrieRegWrite(0x3C,0x10);
	WriteOneByte(ts, (unsigned char)0x3C, (unsigned char)0x10);

	return 0;
}

/*
 * force re-programming the firm image held in the driver
 */
int update_flash(void *_ts, unsigned char *data, unsigned int len)
{
	int page;

	TOUCH_LOG("shtsc: force updating K5 firmware....\n");
	TOUCH_LOG("shtsc: flash_access start\n");
	flash_access_start_shtsc(_ts);

	for (page = 0; page < (len/FLASH_PAGE_SIZE); page++) {
		msleep(FLASH_WAIT);
		flash_erase_page_shtsc(_ts, page);
		TOUCH_LOG("shtsc: flash_erase_page_shtsc done: page %d\n",  page);
		msleep(FLASH_WAIT);
		flash_write_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
		TOUCH_LOG("shtsc: flash_write_page_shtsc done: page %d\n",  page);
#ifndef FLASH_NO_VERIFY
		msleep(FLASH_WAIT);
		flash_verify_page_shtsc(_ts, page, &(data[page*FLASH_PAGE_SIZE]));
		TOUCH_LOG("shtsc: flash_verify_page_shtsc done: page %d\n",  page);
#endif
	}

	TOUCH_LOG("shtsc: flash_access end\n");
	//flash_access_end_shtsc(_ts);

	TOUCH_LOG("shtsc: force updating K5 firmware....done\n");

	return 0;
}

static void GetTouchReport(void *ts, u8 u8Num, u8 *u8Buf, TouchReadData *pData  )
{
	struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch;
	//struct input_dev *input_dev = ((struct shtsc_i2c *)ts)->input;

	int i;
	u8 ID,Status;
	int touchNum = 0;//2014.11.12 added

	//TOUCH_FUNC();

	if( u8Num > SHTSC_MAX_TOUCH_1PAGE ){
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("shtsc touch number erro (num=%d)\n",u8Num);
#endif
		return;
	}

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("shtsc touch number num=%d\n",u8Num);
#endif

	for(i = 0;i < u8Num;i++){
		Status = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0xF0);
		touch[i].id = ID = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH] & 0x0F );
		touch[i].minor = u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0x3F;
		touch[i].major = u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 6];
		touch[i].type = (u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 1] & 0xC0) >> 6;
		touch[i].x =
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 2] << 0) |
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 3] << 8);
		touch[i].y =
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 4] << 0) |
			(u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 5] << 8);
		touch[i].z = u8Buf[i * SHTSC_LENGTH_OF_TOUCH + 7]; // pressure

#if defined(DEBUG_SHTSC)
		TOUCH_LOG("shtsc [%d] ID=%2d, Status=%02x, Major=%3d, Minor=%3d, X=%5d, Y=%5d, Z=%3d\n"
				,u8Num
				,ID
				,Status
				,touch[i].major
				,touch[i].minor
				,touch[i].x
				,touch[i].y
				,touch[i].z);
#endif



		if(Status & SHTSC_TOUCHOUT_STATUS){
#if defined(DEBUG_SHTSC)
			TOUCH_LOG(" release event \n");
#endif
			touch[i].status = SHTSC_F_TOUCH_OUT;
			release_fingerData[ID].id =  touch[i].id;
			release_fingerData[ID].x =  touch[i].x;
			release_fingerData[ID].y =  touch[i].y;
			release_fingerData[ID].pressure = touch[i].z;
			release_fingerData[ID].width_major =  touch[i].major;
			release_fingerData[ID].width_minor =  touch[i].minor;
			continue;
		}

		pData->fingerData[touchNum].id =  touch[i].id;
		pData->fingerData[touchNum].x =  touch[i].x;
		pData->fingerData[touchNum].y =  touch[i].y;
		pData->fingerData[touchNum].pressure = touch[i].z;  // report the value from K6 with no change
		pData->fingerData[touchNum].width_major =  touch[i].major;
		pData->fingerData[touchNum].width_minor =  touch[i].minor;

		touchNum++;

		//		touch[i].status = SHTSC_F_TOUCH;
	}

	pData->type = DATA_FINGER;
	pData->count = touchNum;
}
static int check_buffer_overflow(u8* buffer, int read_length, int buffer_size, char* name)
{
	int ret = TOUCH_SUCCESS;
	int index = 0;

	for( index = read_length; index < buffer_size; index++) {
		if( buffer[index] != 0 ){
			TOUCH_LOG(" Something wrong this buffer %s index %d\n", name, index);
			ret = TOUCH_FAIL;
		}
	}

	//if(ret == TOUCH_SUCCESS)
	//	TOUCH_LOG("Buffer overflow check OK %s\n", name);
	return ret;
}

static irqreturn_t shtsc_irq_thread(struct i2c_client *client, void *_ts, TouchReadData *pData)
{
	struct shtsc_i2c *ts = _ts;

	u16 u16status;
	u8 u8Num = 0;
	//u8 tmpbuf[8];
	u8 numDriveLine2=0;
	u8 numSenseLine2=0;
	u8 num_adc_dmy[3] ={0,};
	//u8 regcommonbuf[12];

	TouchDriverData *pDriverData = NULL;


	//TOUCH_FUNC();

	//return IRQ_HANDLED;

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		TOUCH_LOG("[IRQ] shtsc SELF-D state  \n");
		return IRQ_HANDLED;
	}
#if defined(DEBUG_SHTSC)
	if (G_touch) {
		TOUCH_LOG("[IRQ] shtsc touch-ready %d\n", G_touch);
	}
#endif

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("[ENTER] shtsc_irq\n");
#endif

	mutex_lock(&g_ts.mutex_irq);

	G_reset_done = true;

	/* Get Interrupt State */
	memset(regcommonbuf, 0x0, SIZE_REG_COMMON_BUF);
	ReadMultiBytes(ts,SHTSC_ADDR_INT0,11,regcommonbuf);// less than 12 bytes
	if( TOUCH_FAIL == check_buffer_overflow(regcommonbuf, 11, SIZE_REG_COMMON_BUF , "regcommonbuf")){
		TOUCH_ERR("Check this overflow issue\n");
		dump_stack();
	}

	u16status = ((regcommonbuf[SHTSC_ADDR_INT1] << 8) | regcommonbuf[SHTSC_ADDR_INT0]);

	//TOUCH_ERR("[IRQ] shtsc_irq STATUS [ 0x%04X ] IRQ Mask (0x%04x) \n", u16status, G_Irq_Mask);

	u16status &= G_Irq_Mask;
#if defined(DEBUG_SHTSC)
	if ((u16status != 0x0001) && (u16status != 0x0000)) {
		TOUCH_ERR("[IRQ] shtsc_irq: %04X\n", u16status);
	}
#endif

	//TOUCH_ERR("[IRQ] shtsc_irq STATUS [ 0x%04X ] \n", u16status);

	while(u16status != 0){

		// PLL_UNLOCK
		// 1<<9
		if (u16status & SHTSC_STATUS_PLL_UNLOCK) {
			//
			// PLL unlock
			//

#if  1 // defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc pll-unlock\n");
#endif
			// mask it
			WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
			regcommonbuf[SHTSC_ADDR_INTMASK1] = 0x03;
			// from now on, no int for this
			G_Irq_Mask &= ~SHTSC_STATUS_PLL_UNLOCK;

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_PLL_UNLOCK);
			u16status &= ~SHTSC_STATUS_PLL_UNLOCK;

#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc_irq PLL_UNLOCK: %04X\n", u16status);
#endif
			if(wq_reset_check != NULL ){
				TOUCH_LOG("RESET OK! canceled work\n");
				cancel_delayed_work_sync(&work_reset_check);
			}
		}

		// POWER_UP
		// 1 << 1
		if (u16status & SHTSC_STATUS_POWER_UP) {
			//
			// Power-up
			//

#if  1 //defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc_irq  POWER_UP\n");
#endif

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_POWER_UP);
			u16status &= ~SHTSC_STATUS_POWER_UP;

			if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
				ts->wait_state = WAIT_NONE;
				ts->wait_result = true;
			}
			TOUCH_LOG("folio cover setting q_cover_status=%d\n", q_cover_status);
			if(wq_folio_cover_check != NULL && q_cover_status ==1){
				//TOUCH_LOG("folio cover setting q_cover_status=%d\n", q_cover_status);
				queue_delayed_work(wq_folio_cover_check, &work_folio_cover_check, msecs_to_jiffies(500));
			}
		}
		// WDT
		//
		if (u16status & SHTSC_STATUS_WDT) {
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_WDT);
			u16status &= ~SHTSC_STATUS_WDT;
			TOUCH_ERR("[IRQ] shtsc_irq WDT");

			// WDT make LR388K6 power recyle so that it needs PLL_UNLOCK handling
			G_Irq_Mask = 0xffff;
		}
		// RESUME_PROX
		//
		if (u16status & SHTSC_STATUS_RESUME_PROX) {
			//
			// Resume from DeepIdle
			// or
			// PROXIMITY
			//

#if defined(DEBUG_SHTSC)
			TOUCH_ERR("[IRQ] shtsc resume from DeepIdle or prox\n");
#endif

			SetBankAddr(ts, SHTSC_BANK_TOUCH_REPORT);
			resumeStatus = ReadOneByte(ts, SHTSC_ADDR_RESUME_PROX);

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_RESUME_PROX);
			u16status &= ~SHTSC_STATUS_RESUME_PROX;

		}
		// FLASH_LOAD_ERROR
		//
		if (u16status & SHTSC_STATUS_FLASH_LOAD_ERROR) {
			//
			// FLASH_LOAD_ERROR
			// occurs when flash is erased
			// nothing can be done
			//
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc flash load error\n");
#endif

			TOUCH_LOG("[IRQ] shtsc flash load error\n");

			if (ts->wait_state == WAIT_RESET) {//2014.11.20 added
				ts->wait_state = WAIT_NONE;
				ts->wait_result = true;
			}
			// from now on, no int for this
			G_Irq_Mask &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;

			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_FLASH_LOAD_ERROR);
			u16status &= ~SHTSC_STATUS_FLASH_LOAD_ERROR;
		}
		// TOUCH_READY
		//
		if (u16status & SHTSC_STATUS_TOUCH_READY) {
			//
			// Touch report
			//

#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc touch-ready cleared\n");
#endif


			/* Get number of touches */
			{
				if( regcommonbuf[SHTSC_ADDR_BANK] != SHTSC_BANK_TOUCH_REPORT ){
					WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_TOUCH_REPORT);
					ReadMultiBytes(ts,SHTSC_ADDR_TOUCH_NUM,3,regcommonbuf+SHTSC_ADDR_TOUCH_NUM);
					if( TOUCH_FAIL == check_buffer_overflow(regcommonbuf+SHTSC_ADDR_TOUCH_NUM, 3, SIZE_REG_COMMON_BUF-SHTSC_ADDR_TOUCH_NUM , "regcommonbuf3")){
						TOUCH_ERR("Check this overflow issue\n");
						dump_stack();
					}
				}
				u8Num = regcommonbuf[SHTSC_ADDR_TOUCH_NUM];

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("shtsc touch num=%d\n", u8Num);
#endif

				/* Retrieve touch report */
				if (u8Num > 0){
					memset(touchReportBuf, 0x0, SIZE_TOUCH_REPORT_BUF);
					ReadMultiBytes((struct shtsc_i2c *)ts, SHTSC_ADDR_TOUCH_REPORT, SHTSC_LENGTH_OF_TOUCH * u8Num, touchReportBuf);
					if( TOUCH_FAIL == check_buffer_overflow(touchReportBuf, (SHTSC_LENGTH_OF_TOUCH * u8Num), SIZE_TOUCH_REPORT_BUF , "touchReportBuf")){
						TOUCH_ERR("Check this overflow issue\n");
						dump_stack();
					}
				}
				/* Clear Interrupt */
				u16status &= ~SHTSC_STATUS_TOUCH_READY;
				regcommonbuf[SHTSC_ADDR_INT0] = SHTSC_STATUS_TOUCH_READY;
				regcommonbuf[SHTSC_ADDR_BANK] = SHTSC_BANK_TOUCH_REPORT;
				regcommonbuf[SHTSC_ADDR_IND] = SHTSC_IND_TOUCH;
				WriteMultiBytes(ts,SHTSC_ADDR_INT0,regcommonbuf,4);
				if (u8Num > 0){
					GetTouchReport(ts, u8Num, touchReportBuf, pData);
				}

				//input_sync(ts->input);
			}
		}

		// COMMAND_RESULT
		//
		if (u16status & SHTSC_STATUS_COMMAND_RESULT) {
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc command result PID %d, BankAddr %d\n", current->pid, bankAddr);
#endif
			SetBankAddr(ts,SHTSC_BANK_COMMAND_RESULT);
			ReadMultiBytes(ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.

#if defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc command result, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2]);
#endif


			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_COMMAND_RESULT);
			u16status &= ~SHTSC_STATUS_COMMAND_RESULT;

			if (ts->wait_state == WAIT_CMD) {
				if ((CommandResultBuf[0] != ts->cmd) || (CommandResultBuf[1] != 0)) {
					ts->wait_state = WAIT_NONE;
					ts->wait_result = false;
				} else {
					ts->wait_state = WAIT_NONE;
					ts->wait_result = true;
				}
			}
		}
		// DCAMP_READY
		//
		if (u16status & SHTSC_STATUS_DCMAP_READY) {
#if 1 //defined(DEBUG_SHTSC)
			TOUCH_LOG("[IRQ] shtsc DCMAP READY\n");
#endif
			/* Clear Interrupt */
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_DCMAP_READY);
			u16status &= ~SHTSC_STATUS_DCMAP_READY;

			{ // L2
				unsigned char dsFlag, readingSenseNum, ram_addr[3];
				unsigned int vramAddr;
				unsigned int readingSize;

				// get SD/DS and size
				ram_addr[0] = 0x58;
				ram_addr[1] = 0x7F;
				ram_addr[2] = 0x01;
				WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);
				WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
				WriteOneByte(ts, 0x43, ram_addr[2]);

				SetBankAddr(ts,SHTSC_BANK_DCMAP);

				memset(irq_thread_tmpbuf, 0x0, SIZE_IRQ_THREAD_TEMP_BUF);
				ReadMultiBytes(ts, 0x08, 7, irq_thread_tmpbuf); // less than 8 bytes
				if( TOUCH_FAIL == check_buffer_overflow(irq_thread_tmpbuf, 7, SIZE_IRQ_THREAD_TEMP_BUF, "irq_thread_tmpbuf")){
					TOUCH_ERR("Check this overflow issue\n");
					dump_stack();
				}


				numSenseLine2 = irq_thread_tmpbuf[0];
				numDriveLine2 = irq_thread_tmpbuf[1];

				dsFlag = irq_thread_tmpbuf[6]; // 1 for DS, 0 for SD
				vramAddr = ((irq_thread_tmpbuf[4]<<16) | (irq_thread_tmpbuf[3]<<8) | irq_thread_tmpbuf[2]);
				// readingSenseNum is greater or equal to itself, but a multiply of 4
				readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
				readingSenseNum *= 4;

				readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

				num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
				num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

				// read DCmap values from register
				// store it to read buffer memory for read action
				{ // L1
					/* read 120 bytes from Bank5, address 8 */
					/* read loop required */
					int bytes = readingSize;
					int size;
					int index = 0;
					//SetBankAddr(ts,SHTSC_BANK_DCMAP);

					memset(dcmapBuf, 0x0, sizeof(dcmapBuf));

					//	      TOUCH_LOG("%s(%d):readingSize:%d\n", __FILE__, __LINE__, readingSize);
					while (bytes > 0) {
						ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
						ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Middle)
						ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // address to read (Higher)
						WriteMultiBytes(ts,(unsigned char)0x06,ram_addr,2);
						WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
						WriteOneByte(ts, 0x43, ram_addr[2]);
						WriteOneByte(ts, SHTSC_ADDR_BANK, SHTSC_BANK_DCMAP);

						size = ((bytes >= 120) ? 120 : bytes);
						//TOUCH_LOG("bytes:%d, size:%d, index:%d, vramAddr:%x\n", bytes, size, index, vramAddr);

						ReadMultiBytes(ts, 0x08, size, &(dcmapBuf[index]));
						index += size;
						bytes -= size;
						vramAddr += size;
					} // while
				} // L1

#if 0
				TOUCH_LOG("DCmap Drive x Sense: %d x %d, dsFlag:%02X, num_adc_dmy:%02X %02X %02X\n", numDriveLine2, numSenseLine2, dsFlag, num_adc_dmy[0], num_adc_dmy[1], num_adc_dmy[2]);
#endif /* 0 */

				{ //L3
					int sindex = 0, dindex = 0;
					int l, x, y;
#if 0 // debug
					static unsigned char frm_count = 0;
#endif /* 1 */

					// dcmap header
					// [0]: horizontal data num (in short, not byte)
					// [1]: vertical data num

					TOUCH_LOG(" Sense : %d Drive : %d\n", numSenseLine2, numDriveLine2);

					x = dcmap[dindex++] = numSenseLine2;
					y = dcmap[dindex++] = numDriveLine2;
					dcmap[dindex++] = dsFlag;
#if 0 // debug
					dcmap[dindex++] = frm_count++; // just for debug
#else /* 1 */
					dcmap[dindex++] = 0x00; // reserved
#endif /* 1 */

					//top
					sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

					// contents line
					for (l = 0; l < y; l++) {
						// left
						sindex += (num_adc_dmy[0] * 2);

						// contents
						memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(dcmapBuf[sindex]), (x*2));
						dindex += (x*2);
						sindex += (x*2);

						// right
						sindex += (num_adc_dmy[1] * 2);
					}

					// for read()
					//	      TOUCH_LOG("check buf_pos: %d\n", buf_pos);
					if (buf_pos == 0) {
						memcpy((u8 *)devbuf, (u8 *)dcmap, (4+x*y*2));
						//		TOUCH_LOG("setting buf_pos: %d\n", buf_pos);
						buf_pos = (4+x*y*2);
						//		TOUCH_LOG("set buf_pos: %d\n", buf_pos);
					}

#if 0 // DCmap debug
					TOUCH_LOG("DC map size HxV: %d x %d = %d\n", dcmap[0], dcmap[1], dcmap[0]*dcmap[1]);
					TOUCH_LOG("[0-3, %d-%d]: %02X %02X %02X %02X, %02X %02X %02X %02X\n", (x*y-4), (x*y-1), dcmap[4], dcmap[5], dcmap[6], dcmap[7], dcmap[x*y*2+4-4], dcmap[x*y*2+4-3], dcmap[x*y*2+4-2], dcmap[x*y*2+4-1]);
#endif /* 0 */
#if 0 // DCmap debug
					{
						int i = 0;
						int j = 0;
						for (j = 0; j < y; j++) {
							for (i = 0; i < x; i++) {
								TOUCH_LOG("%d: %02X ", (y*j+i), dcmap[y*j + i + 4]);
							}
							TOUCH_LOG("\n");
						}
					}
#endif
				} //L3
			} // L2 DEVICE_LR388K6 block
			//*************************

			if (ts->wait_state == WAIT_DCMAP) {//2015.11.12 added
				ts->wait_state = WAIT_NONE;
				ts->wait_result = true;
			}
		}

		// SHTSC_STATUS_TCI_1(KNOCK_ON)
		if (u16status & SHTSC_STATUS_LG_LPWG_TCI1) {
			u8 buffer[3] = {0};
			u8 failureReason = 0;

#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
			struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch; // for testing Dec.2
			//u8 knockData_buf[KNOCKDATA_SIZE];
			int i = 0, n = 0;
#endif

			Sharp_ClearInterrupt(ts, SHTSC_STATUS_LG_LPWG_TCI1);
			u16status &= ~(SHTSC_STATUS_LG_LPWG_TCI1);
			if( pDriverData != NULL && ( pDriverData->currState != STATE_NORMAL)) {
				TOUCH_LOG("[IRQ] shtsc_irq KNOCK_ON\n");
			}

			SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);
			ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
			failureReason = buffer[0] & 0x0F; // now it's ready
#if 0
			TOUCH_LOG("TCI1 Failure Reason [%d]\n", failureReason);
#endif

			switch (failureReason) {
				case 0:
					TOUCH_LOG("TCI1 SUCCESS\n");
					pData->type = DATA_KNOCK_ON;
					break;
				case 1:
					if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
						n = 1;
					} else {
						TOUCH_LOG("TCI1 FAIL - DISTANCE_INTER_TAP\n");
#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
						n = 3;
#endif
					}
					break;
				case 2:
					if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
						TOUCH_LOG("[Calibration] - Enter Stage 1.5 \n");
					} else {
						TOUCH_LOG("TCI1 FAIL - DISTANCE_TOUCHSLOP\n");
#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
						n = 4;
#endif
					}
					break;
				case 3:
					if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
						TOUCH_LOG("[Calibration] - Exit Stage 1 (Touch removed) \n");
					} else {
						TOUCH_LOG("TCI1 FAIL - TIMEOUT_INTERTAP\n");
					}
					break;
				case 4:
					if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
						n = 1;
					} else {
						TOUCH_LOG("TCI1 FAIL - MULTI_FINGER\n");
#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG
						n = 2;
#endif
					}
					break;
				case 5:
					if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
						TOUCH_LOG("[Calibration] - Enter Stage 2 \n");
					} else {
						TOUCH_LOG("TCI1 FAIL - DELAY_TIME\n");
					}
					break;
				case 6:
					TOUCH_LOG("TCI1 FAIL - PALM_STATE\n");
					break;
				default:
					TOUCH_LOG("TCI1 FAIL - RESERVED\n");
					break;
			}

#if TOUCH_LPWG_KNOCK_ON_FAIL_DEBUG

			SetBankAddr(ts,SHTSC_BANK_LPWG_DATA);

			memset(knockData_buf, 0x0, SIZE_KNOCKDATA_BUF);
			ReadMultiBytes(ts,SHTSC_ADDR_LPWG_REPORT,KNOCKDATA_SIZE,knockData_buf);
			if( TOUCH_FAIL == check_buffer_overflow(knockData_buf, KNOCKDATA_SIZE, SIZE_KNOCKDATA_BUF, "knockData_buf")){
				TOUCH_ERR("Check this overflow issue\n");
				dump_stack();
			}

			for ( i = 0; i < n; i++) { // just two points to print out
				touch[i].x =
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG    ] << 0) |
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 1] << 8);
				touch[i].y =
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 2] << 0) |
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 3] << 8);

				if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
					if(touch[i].x == 0 && touch[i].y == 0){
						TOUCH_LOG("[Calibration] - Enter Stage 3\n");
					} else if ( touch[i].x == 1 && touch[i].y == 1) {
						TOUCH_LOG("[Calibration] - Enter Stage 4\n");
					} else if ( touch[i].x == 3 && touch[i].y == 3) {
						TOUCH_LOG("[Calibration] - Exit Stage 5\n");
					} else if ( touch[i].x == 4 && touch[i].y == 4) {
						TOUCH_LOG("[Calibration] - Enter Stage 5\n");
					} else if ( touch[i].x == 65535 && touch[i].y == 65535) {
						TOUCH_LOG("[Calibration] - Enter Stage 1\n");
					} else {
						TOUCH_LOG("[Calibration] - Enter Stage ? (%d , %d)\n", touch[i].x, touch[i].y);
					}
				} else {

					if(i == 0  && touch[i].x == 0 && touch[i].y == 0){
						TOUCH_LOG("[Calibration / LPWG] - Enter Stage 3 \n");
					} else if ( i == 0 && touch[i].x == 1 && touch[i].y == 1) {
						TOUCH_LOG("[Calibration / LPWG] - Enter Stage 4 \n");
					} else if (  i== 0 && touch[i].x == 65535 && touch[i].y == 65535) {
						TOUCH_LOG("[Calibration / LPWG] - Enter Stage 1 \n");
					} else {
						TOUCH_LOG("[IRQ][DEBUG] KNOCK_ON coodinate(%d) (x, y) = (%d, %d)\n", i, touch[i].x, touch[i].y);
					}
				}
			}
#endif

		}
		// SHTSC_STATUS_TCI_2(KNOCK_CODE)
		if (u16status & SHTSC_STATUS_LG_LPWG_TCI2) {
			u8 buffer[3] = {0};
			u8 failureReason = 0;
			int i;
			struct shtsc_touch *touch = ((struct shtsc_i2c *)ts)->touch;
			//u8 knockData_buf[KNOCKDATA_SIZE];
			Sharp_ClearInterrupt(ts, SHTSC_STATUS_LG_LPWG_TCI2);
			u16status &= ~(SHTSC_STATUS_LG_LPWG_TCI2);

			if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
				TOUCH_LOG("[IRQ] shtsc_irq CALIBRATION\n");
			} else {
				TOUCH_LOG("[IRQ] shtsc_irq KNOCK_CODE\n");
			}
#if 0
			TOUCH_LOG("[IRQ] shtsc_irq KNOCK_CODE count %d\n", g_ts.lpwgSetting.tapCount);
#endif

			SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);
			ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
			failureReason = (buffer[0] & 0xF0) >> 4; // now it's ready
#if 0
			TOUCH_LOG("TCI2 Failure Reason [%d]\n", failureReason);
#endif

			switch (failureReason) {
				case 0:
					TOUCH_LOG("TCI2 SUCCESS\n");
					break;
				case 1:
					TOUCH_LOG("TCI2 FAIL - DISTANCE_INTER_TAP\n");
					break;
				case 2:
					TOUCH_LOG("TCI2 FAIL - DISTANCE_TOUCHSLOP\n");
					break;
				case 3:
					TOUCH_LOG("TCI2 FAIL - TIMEOUT_INTERTAP\n");
					break;
				case 4:
					if( pDriverData != NULL && ( pDriverData->currState == STATE_NORMAL)) {
						TOUCH_LOG("[Calibration] - Exit Stage 2 \n");
					} else {
						TOUCH_LOG("TCI2 FAIL - MULTI_FINGER\n");
					}
					break;
				case 5:
					TOUCH_LOG("TCI2 FAIL - DELAY_TIME\n");
					break;
				case 6:
					TOUCH_LOG("TCI2 FAIL - PALM_STATE\n");
					break;
				default:
					TOUCH_LOG("TCI2 FAIL - RESERVED\n");
					break;
			}

			if (failureReason == 0) { // SUCCESS
				pData->type = DATA_KNOCK_CODE;
				SetBankAddr(ts,SHTSC_BANK_LPWG_DATA);

				memset(knockData_buf, 0x0, SIZE_KNOCKDATA_BUF);
				ReadMultiBytes(ts,SHTSC_ADDR_LPWG_REPORT,KNOCKDATA_SIZE,knockData_buf);
				if( TOUCH_FAIL == check_buffer_overflow(knockData_buf, KNOCKDATA_SIZE, SIZE_KNOCKDATA_BUF, "knockData_buf")){
					TOUCH_ERR("Check this overflow issue\n");
					dump_stack();
				}

				pData->count = g_ts.lpwgSetting.tapCount;

				for (i = 0; i < g_ts.lpwgSetting.tapCount; i++) {
					touch[i].x =
						(knockData_buf[i * SHTSC_LENGTH_OF_LPWG    ] << 0) |
						(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 1] << 8);
					touch[i].y =
						(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 2] << 0) |
						(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 3] << 8);

					pData->knockData[i].x =  touch[i].x;
					pData->knockData[i].y =  touch[i].y;
				}
			}
		}
		if (u16status != 0) {
			TOUCH_LOG("[IRQ] shtsc unknown interrupt status %04X\n", u16status);
			/* Clear all interrupts and mask.  */
			WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xFF);
			WriteOneByte(ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
			u16status = 0;
			Sharp_ClearInterrupt(ts, u16status);
		}
	}

	if (u8Num != 0 ) {
#if defined(DEBUG_SHTSC)
		//TOUCH_LOG( "shtsc flush touch input(%d)\n", u8Num);
#endif
		/*
		   input_report_key(ts->input, BTN_TOUCH, touchNum?true:false );

		   input_sync(ts->input);
		   */
	}

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("[IRQ] shtsc_irq done \n");
#endif

	mutex_unlock(&g_ts.mutex_irq);

	return IRQ_HANDLED;
}

static ssize_t store_tci(struct i2c_client *client,
		const char *buf, size_t count)
{
	u32 type = 0, tci_num = 0, value = 0;

	sscanf(buf, "%d %d %d", &type, &tci_num, &value);

	tci_control(client, type, (unsigned int)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	int ret = 0;
	u8 buffer[3] = {0};

	SetBankAddr(&g_ts, SHTSC_BANK_LPWG_PARAM);

	ReadMultiBytes(&g_ts, REPORT_RATE_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Hz [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	ReadMultiBytes(&g_ts, SENSITIVITY_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Sensitivity [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));


	ReadMultiBytes(&g_ts, ACTIVE_AREA_X1_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea X1 [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, ACTIVE_AREA_Y1_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea Y1 [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, ACTIVE_AREA_X2_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea X2 [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, ACTIVE_AREA_Y2_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "ActiveArea Y2 [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));


	WRITE_SYSBUF(buf, ret, "TCI - 1\n");
	ReadMultiBytes(&g_ts, SHTSC_ADDR_INTMASK0, 1, buffer);
	WRITE_SYSBUF(buf, ret, "TCI [%s]\n",
			((buffer[0] & SHTSC_STATUS_LG_LPWG_TCI1) == SHTSC_STATUS_LG_LPWG_TCI1 ) ? "disabled" : "enabled");
	ReadMultiBytes(&g_ts, TOUCH_SLOP_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Touch Slop [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_DISTANCE_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Distance [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MIN_INTERTAP_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Min InterTap [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MAX_INTERTAP_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Max InterTap [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_COUNT_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Count [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, INTERRUPT_DELAY_CTRL_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Interrupt Delay [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
	WRITE_SYSBUF(buf, ret, "Failure Reason [%d]\n",
			buffer[0] & 0x0F);
	ReadMultiBytes(&g_ts, FAILURE_INT_ENABLE_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Enable [0x%2X]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, FAILURE_INT_STATUS_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Status [0x%2X]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	WRITE_SYSBUF(buf, ret, "TCI - 2\n");
	ReadMultiBytes(&g_ts, SHTSC_ADDR_INTMASK0, 1, buffer);
	WRITE_SYSBUF(buf, ret, "TCI [%s]\n",
			((buffer[0] & SHTSC_STATUS_LG_LPWG_TCI2) == SHTSC_STATUS_LG_LPWG_TCI2 ) ? "disabled" : "enabled");
	ReadMultiBytes(&g_ts, TOUCH_SLOP_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Touch Slop [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_DISTANCE_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Distance [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MIN_INTERTAP_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Min InterTap [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, MAX_INTERTAP_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Max InterTap [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, TAP_COUNT_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Tap Count [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, INTERRUPT_DELAY_CTRL2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Interrupt Delay [%d]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	ReadMultiBytes(&g_ts, FAILURE_REASON_REG, 1, buffer);
	WRITE_SYSBUF(buf, ret, "Failure Reason [%d]\n",
			(buffer[0] & 0xF0) >> 4);
	ReadMultiBytes(&g_ts, FAILURE_INT_ENABLE2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Enable [0x%2X]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));
	ReadMultiBytes(&g_ts, FAILURE_INT_STATUS2_REG, 2, buffer);
	WRITE_SYSBUF(buf, ret, "Failure INT Status [0x%2X]\n",
			(unsigned int)((buffer[1] << 8) & 0xFF00) | (unsigned int)(buffer[0] & 0xFF));

	return ret;
}

static ssize_t show_lpwg_touch_data(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int i = 0;
	u8 total_touch_points = 0;
	//u8 knockData_buf[KNOCKDATA_SIZE];

	if((dummyDeviceState == STATE_KNOCK_ON_ONLY)||(dummyDeviceState == STATE_KNOCK_ON_CODE)){

		SetBankAddr(&g_ts,SHTSC_BANK_LPWG_DATA);
		ReadMultiBytes(&g_ts , SHTSC_ADDR_TOUCH_NUM , 1 , &total_touch_points);

		memset(knockData_buf, 0x0, SIZE_KNOCKDATA_BUF);
		ReadMultiBytes(&g_ts , SHTSC_ADDR_LPWG_REPORT , KNOCKDATA_SIZE , knockData_buf);
		if( TOUCH_FAIL == check_buffer_overflow(knockData_buf, KNOCKDATA_SIZE, SIZE_KNOCKDATA_BUF, "knockData_buf")){
			TOUCH_ERR("Check this overflow issue\n");
			dump_stack();
		}

		total_touch_points = total_touch_points & 0xF;

		WRITE_SYSBUF(buf, ret, "total_touch_points [%d]\n",total_touch_points);

		for (i = 0; i < total_touch_points; i++) {
			WRITE_SYSBUF(buf, ret, "lpwg_touch_data[%d] x = [%d] y = [%d] \n",i,
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG] << 0) |(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 1] << 8),
					(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 2] << 0) |(knockData_buf[i * SHTSC_LENGTH_OF_LPWG + 3] << 8));
		}
	}else{
		WRITE_BUFFER(buf, ret, "current state is not LPWG mode\n");
	}

	return ret;
}

static int LR388K6_MftsControl(struct i2c_client *client)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

static int tci_control(struct i2c_client *client, int type, unsigned int value)
{
	int ret = TOUCH_SUCCESS;
	volatile unsigned char readData;
	u8 buffer[3] = {0};

	//TOUCH_FUNC();


	buffer[0] = (u8)(value & 0xFF);
	buffer[1] = (u8)(value >> 8) & 0xFF;

	switch (type) {
		case REPORT_RATE_CTRL:
			lpwg_param_command(&g_ts, REPORT_RATE_CTRL_REG, buffer, 2);
			break;
		case SENSITIVITY_CTRL:
			lpwg_param_command(&g_ts, SENSITIVITY_CTRL_REG, buffer, 2);
			break;

			// Active area
		case ACTIVE_AREA_X1_CTRL:
			lpwg_param_command(&g_ts, ACTIVE_AREA_X1_CTRL_REG, buffer, 2);
			break;
		case ACTIVE_AREA_Y1_CTRL:
			lpwg_param_command(&g_ts, ACTIVE_AREA_Y1_CTRL_REG, buffer, 2);
			break;
		case ACTIVE_AREA_X2_CTRL:
			lpwg_param_command(&g_ts, ACTIVE_AREA_X2_CTRL_REG, buffer, 2);
			break;
		case ACTIVE_AREA_Y2_CTRL:
			lpwg_param_command(&g_ts, ACTIVE_AREA_Y2_CTRL_REG, buffer, 2);
			break;

			// TCI1
		case TCI_ENABLE_CTRL:
			readData = ReadOneByte(&g_ts, SHTSC_ADDR_INTMASK0);
			if (value) {
				readData &= ~SHTSC_STATUS_LG_LPWG_TCI1;
			} else {
				readData |= SHTSC_STATUS_LG_LPWG_TCI1;
			}
			WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)readData);
			break;
		case TOUCH_SLOP_CTRL:
			lpwg_param_command(&g_ts, TOUCH_SLOP_CTRL_REG, buffer, 2);
			break;
		case TAP_DISTANCE_CTRL:
			lpwg_param_command(&g_ts, TAP_DISTANCE_CTRL_REG, buffer, 2);
			break;
		case MIN_INTERTAP_CTRL:
			lpwg_param_command(&g_ts, MIN_INTERTAP_CTRL_REG, buffer, 2);
			break;
		case MAX_INTERTAP_CTRL:
			lpwg_param_command(&g_ts, MAX_INTERTAP_CTRL_REG, buffer, 2);
			break;
		case TAP_COUNT_CTRL:
			lpwg_param_command(&g_ts, TAP_COUNT_CTRL_REG, buffer, 2);
			break;
		case INTERRUPT_DELAY_CTRL:
			lpwg_param_command(&g_ts, INTERRUPT_DELAY_CTRL_REG, buffer, 2);
			break;
		case FAILURE_INT_ENABLE:
			lpwg_param_command(&g_ts, FAILURE_INT_ENABLE_REG, buffer, 2);
			break;

			// TCI2
		case TCI_ENABLE_CTRL2:
			readData = ReadOneByte(&g_ts, SHTSC_ADDR_INTMASK0);
			if (value) {
				readData &= ~SHTSC_STATUS_LG_LPWG_TCI2;
			} else {
				readData |= SHTSC_STATUS_LG_LPWG_TCI2;
			}
			WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)readData);
			break;
		case TOUCH_SLOP_CTRL2:
			lpwg_param_command(&g_ts, TOUCH_SLOP_CTRL2_REG, buffer, 2);
			break;
		case TAP_DISTANCE_CTRL2:
			lpwg_param_command(&g_ts, TAP_DISTANCE_CTRL2_REG, buffer, 2);
			break;
		case MIN_INTERTAP_CTRL2:
			lpwg_param_command(&g_ts, MIN_INTERTAP_CTRL2_REG, buffer, 2);
			break;
		case MAX_INTERTAP_CTRL2:
			lpwg_param_command(&g_ts, MAX_INTERTAP_CTRL2_REG, buffer, 2);
			break;
		case TAP_COUNT_CTRL2:
			lpwg_param_command(&g_ts, TAP_COUNT_CTRL2_REG, buffer, 2);
			break;
		case INTERRUPT_DELAY_CTRL2:
			lpwg_param_command(&g_ts, INTERRUPT_DELAY_CTRL2_REG, buffer, 2);
			break;
		case FAILURE_INT_ENABLE2:
			lpwg_param_command(&g_ts, FAILURE_INT_ENABLE2_REG, buffer, 2);
			break;

		default:
			TOUCH_ERR("invalid tci param type ( %d )\n", type);
			ret = TOUCH_FAIL;
			break;
	}

	return ret;
}

#define CLOCK_OFF 0
#define CLOCK_ON  1

static int controlDeepSleepClock(int flag)
{
	if(flag == CLOCK_OFF) { // DEEP_SLEEP_IN
		TOUCH_LOG("CLOCK_OFF\n");
		msleep(10);

		SetBankAddr(&g_ts, 0x1C);
		WriteOneByte(&g_ts, (unsigned char)0x24, (unsigned char)0x00);
		msleep(1);

		WriteOneByte(&g_ts, (unsigned char)0x28, (unsigned char)0x11);

		SetBankAddr(&g_ts, 0x1E);
		WriteOneByte(&g_ts, (unsigned char)0x11, (unsigned char)0x00);

		SetBankAddr(&g_ts, 0x1C);
		WriteOneByte(&g_ts, (unsigned char)0x5D, (unsigned char)0x01);

		WriteOneByte(&g_ts, (unsigned char)0x5C, (unsigned char)0x00);
		msleep(1); // for stability

	} else { // CLOCK_ON = DEEP_SLEEP_OUT
		TOUCH_LOG("CLOCK_ON\n");
		SetBankAddr(&g_ts, 0x1C);
		WriteOneByte(&g_ts, (unsigned char)0x5C, (unsigned char)0x01);
		msleep(1);

		WriteOneByte(&g_ts, (unsigned char)0x5D, (unsigned char)0x00);

		WriteOneByte(&g_ts, (unsigned char)0x28, (unsigned char)0x12);
		msleep(1);

		WriteOneByte(&g_ts, (unsigned char)0x24, (unsigned char)0x03);
		msleep(1);

		SetBankAddr(&g_ts, 0x1E);
		WriteOneByte(&g_ts, (unsigned char)0x11, (unsigned char)0x01);

		msleep(10);
	}
	return 0;
}


static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	int ret = TOUCH_SUCCESS;
	TouchDriverData *pDriverData = NULL;
	pDriverData = i2c_get_clientdata(client);

	//TOUCH_FUNC();

	switch (newState) {
		case STATE_NORMAL:
			//tci_control(client, TCI_ENABLE_CTRL, 0);
			//tci_control(client, TCI_ENABLE_CTRL2, 0);

			// To Idle
			//issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);
			break;

		case STATE_KNOCK_ON_ONLY:
			controlDeepSleepClock(CLOCK_ON);

			tci_control(client, TAP_COUNT_CTRL, 2);
			tci_control(client, TAP_COUNT_CTRL2, 4);
			tci_control(client, MIN_INTERTAP_CTRL, 0);
			tci_control(client, MAX_INTERTAP_CTRL, 500);
			tci_control(client, TOUCH_SLOP_CTRL, 100);
			tci_control(client, TAP_DISTANCE_CTRL, 100);
			tci_control(client, INTERRUPT_DELAY_CTRL, 0);

			tci_control(client, ACTIVE_AREA_X1_CTRL, g_ts.lpwgSetting.activeTouchAreaX1);
			tci_control(client, ACTIVE_AREA_Y1_CTRL, g_ts.lpwgSetting.activeTouchAreaY1);
			tci_control(client, ACTIVE_AREA_X2_CTRL, g_ts.lpwgSetting.activeTouchAreaX2);
			tci_control(client, ACTIVE_AREA_Y2_CTRL, g_ts.lpwgSetting.activeTouchAreaY2);

			tci_control(client, FAILURE_INT_ENABLE, lpwg_fail_reason_mask); // KNOCK_CODE_FAILURE_REASONS);
			tci_control(client, TCI_ENABLE_CTRL, 1);
			tci_control(client, TCI_ENABLE_CTRL2, 0);

#if 1
			// only for Self-D LPWG testing
			if(pDriverData->bootMode == BOOT_MINIOS || pDriverData->bootMode == BOOT_MFTS){
				if( pDriverData->mfts_lpwg == 1 ){
					TOUCH_LOG("LPWG Testing save Threshold test_flag : %d [0x%02x],[0x%02x]\n", lgtp_set_deepidle_test_flag, CommandResultBuf[2], CommandResultBuf[3] );
					lgtp_set_deepidle_test_flag = 1;
					//lgtp_save_deepidle_val();
					issue_command_wo_IRQ(&g_ts, CMD_GETREGTBL_IDLE_BRK_TRSH_DC, CMD_GETREGTBL_IDLE_BRK_TRSH_LEN);
					lgtp_old_idle_brk_thsh_d0 = CommandResultBuf[2];
					lgtp_old_idle_brk_thsh_d1 = CommandResultBuf[3];
					TOUCH_LOG("LPWG Testing Old threshold [0x%02x], [0x%02x]\n", lgtp_old_idle_brk_thsh_d0, lgtp_old_idle_brk_thsh_d1);

					// Set Threshold 7FFF
					issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_IDLE_BRK_TRSH_DC0, CMD_SETREGTBL_IDLE_BRK_TRSH_LEN);
					issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_IDLE_BRK_TRSH_DC1, CMD_SETREGTBL_IDLE_BRK_TRSH_LEN);
				}
			}
#endif

			// To Deep Idle
			issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
			break;

		case STATE_KNOCK_ON_CODE:
			controlDeepSleepClock(CLOCK_ON);

			tci_control(client, TAP_COUNT_CTRL, 2);
			tci_control(client, MIN_INTERTAP_CTRL, 0);
			tci_control(client, MAX_INTERTAP_CTRL, 500);
			tci_control(client, TOUCH_SLOP_CTRL, 100);
			tci_control(client, TAP_DISTANCE_CTRL, 100);
			tci_control(client, INTERRUPT_DELAY_CTRL, 500);

			tci_control(client, ACTIVE_AREA_X1_CTRL, g_ts.lpwgSetting.activeTouchAreaX1);
			tci_control(client, ACTIVE_AREA_Y1_CTRL, g_ts.lpwgSetting.activeTouchAreaY1);
			tci_control(client, ACTIVE_AREA_X2_CTRL, g_ts.lpwgSetting.activeTouchAreaX2);
			tci_control(client, ACTIVE_AREA_Y2_CTRL, g_ts.lpwgSetting.activeTouchAreaY2);

			tci_control(client, TAP_COUNT_CTRL2, g_ts.lpwgSetting.tapCount);
			tci_control(client, MIN_INTERTAP_CTRL2, 0);
			tci_control(client, MAX_INTERTAP_CTRL2, 500);
			tci_control(client, TOUCH_SLOP_CTRL2, 100);
			tci_control(client, TAP_DISTANCE_CTRL2, 1700);
			tci_control(client, INTERRUPT_DELAY_CTRL2, 0);

			tci_control(client, FAILURE_INT_ENABLE, lpwg_fail_reason_mask); //KNOCK_CODE_FAILURE_REASONS);
			tci_control(client, FAILURE_INT_ENABLE2, lpwg_fail_reason_mask); //KNOCK_CODE_FAILURE_REASONS);
			tci_control(client, TCI_ENABLE_CTRL, 1);
			tci_control(client, TCI_ENABLE_CTRL2, 1);

			// To Deep Idle
			issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
			break;

		case STATE_OFF:
			tci_control(client, TCI_ENABLE_CTRL, 0);
			tci_control(client, TCI_ENABLE_CTRL2, 0);

			// To Sleep
			issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);

			// To DeepSleep
			controlDeepSleepClock(CLOCK_OFF);
			break;

		default:
			TOUCH_ERR("invalid touch state ( %d )\n", newState);
			ret = TOUCH_FAIL;
			break;

	}

	return ret;
}

int shtsc_CMD_SetSystemStateSleep_wo_IRQ(void *ts)
{
	//int ret;
	int count = 0;
	u8 value = 0;

	//TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10){
			TOUCH_LOG("waiting the result count %d. Please check it\n", count);
			break;
		}
	}

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}


int shtsc_CMD_GetProperty( // wo_IRQ
		void *ts,		/**<[in] pointor to struct shtsc_i2c or struct shtsc_spi */
		unsigned char *buf	/**<[in] 17 byte buffer for result */
		)
{
	int count = 0;

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	//value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	//TOUCH_LOG("Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10){
			TOUCH_LOG("waiting the result count %d. Please check it\n", count);
			break;
		}
	}

#ifdef DEBUG_SHTSC
	TOUCH_LOG("%s : Before BANK_COMMMAND_RESULT BankAddr %d \n", __func__, bankAddr);
#endif
	SetBankAddr(ts,SHTSC_BANK_COMMAND_RESULT);
	ReadMultiBytes(ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("[IRQ] shtsc command result, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2]);
#endif

	memcpy(buf,CommandResultBuf+2,17-2);

	Sharp_ClearInterrupt(ts, SHTSC_STATUS_COMMAND_RESULT);  // must be cleared for the next command

	return 0;
}

int shtsc_CMD_SetSystemState(void *ts, dCmdState_e eState)
{
	char cmdArray[CMD_SETSYSTEMSTATE_LEN];
	int ret;

	memcpy(cmdArray, CMD_SETSYSTEMSTATE, CMD_SETSYSTEMSTATE_LEN);
	cmdArray[4] = eState;
	ret =  issue_command(ts, cmdArray, CMD_SETSYSTEMSTATE_LEN);
	return ret;
}


/****************************************************************************
 * Device Specific Functions
 ****************************************************************************/

// this function is only for testing for prox sensor and lpwg time interval
// LPWG STATE_OFF -> wait -> LPWG_KNOCK_ON_ONLY -> wait loop
// we don't use issue_command_wo_IRQ() because this test is in the IRQ enabled context (replaced)
int lpwg_prox_test_delay_time = 50;
int lpwg_prox_test_repeat_count = 5;
static ssize_t store_lpwg_prox_test(struct i2c_client *client, const char *buf, size_t count)
{
	int delay_time = 0;
	int repeat_count = 0;

	//TOUCH_FUNC();

	sscanf(buf, "%d %d", &delay_time, &repeat_count);

	//WRITE_SYSBUF(buf, ret, "LPWG Proximity Testing :  %d ms delay %d repeat count\n", delay_time, repeat_count);

	TOUCH_LOG("LPWG Proximity Testing :  %d ms delay %d repeat count\n", delay_time, repeat_count);

	lpwg_prox_test_delay_time = delay_time;
	lpwg_prox_test_repeat_count = repeat_count;

	return count;
}
static ssize_t show_lpwg_prox_test(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int i = 0;
	TouchDriverData *pDriverData = NULL;

	TOUCH_FUNC();

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}

	if( pDriverData != NULL && ( pDriverData->currState == STATE_KNOCK_ON_ONLY || pDriverData->currState == STATE_KNOCK_ON_CODE)) {
		TOUCH_LOG("prox_lpwg_test time %d (ms), repeat value %d\n", lpwg_prox_test_delay_time, lpwg_prox_test_repeat_count);
	} else {

		TOUCH_LOG("STATE is mismatched\n");
		WRITE_SYSBUF(buf, ret, "LPWG Proximity Testing :  Failure (mismatch state)\n");
		return ret;
	}

	WRITE_SYSBUF(buf, ret, "LPWG Proximity Testing :  Delay time %d Repeat count %d\n", lpwg_prox_test_delay_time, lpwg_prox_test_repeat_count);

	for (i = 0; i < lpwg_prox_test_repeat_count; i++) {

		TOUCH_LOG("LPWG: STATE_OFF\n");
		tci_control(client, TCI_ENABLE_CTRL, 0);
		tci_control(client, TCI_ENABLE_CTRL2, 0);

		// To Sleep
		//    issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
		issue_command(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);

		// To DeepSleep
		controlDeepSleepClock(CLOCK_OFF);

		// wait for a while
		msleep(lpwg_prox_test_delay_time);


		TOUCH_LOG("LPWG: STATE_KNOCK_ON_ONLY\n");
		controlDeepSleepClock(CLOCK_ON);

		tci_control(client, TAP_COUNT_CTRL, 2);
		tci_control(client, TAP_COUNT_CTRL2, 4);
		tci_control(client, MIN_INTERTAP_CTRL, 0);
		tci_control(client, MAX_INTERTAP_CTRL, 500);
		tci_control(client, TOUCH_SLOP_CTRL, 100);
		tci_control(client, TAP_DISTANCE_CTRL, 100);
		tci_control(client, INTERRUPT_DELAY_CTRL, 0);

		tci_control(client, ACTIVE_AREA_X1_CTRL, g_ts.lpwgSetting.activeTouchAreaX1);
		tci_control(client, ACTIVE_AREA_Y1_CTRL, g_ts.lpwgSetting.activeTouchAreaY1);
		tci_control(client, ACTIVE_AREA_X2_CTRL, g_ts.lpwgSetting.activeTouchAreaX2);
		tci_control(client, ACTIVE_AREA_Y2_CTRL, g_ts.lpwgSetting.activeTouchAreaY2);

		tci_control(client, FAILURE_INT_ENABLE, lpwg_fail_reason_mask); // KNOCK_CODE_FAILURE_REASONS);
		tci_control(client, TCI_ENABLE_CTRL, 1);
		tci_control(client, TCI_ENABLE_CTRL2, 0);

		// To Deep Idle
		issue_command(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);
		//issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_DEEPIDLE, CMD_SETSYSTEMSTATE_DEEPIDLE_LEN);

		// wait for a while
		msleep(lpwg_prox_test_delay_time);

	}

	TOUCH_LOG("LPWG Proximity Testing Done\n");
	WRITE_SYSBUF(buf, ret, "LPWG Proximity Testing : Sucess\n");

	return ret;
}
static ssize_t get_unique_id(char *idBuf)
{
	int ret = 0;
	unsigned char irqmask[2];

	// read LCM ID
	issue_command(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
	// save reg values
	irqmask[0] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK0);
	irqmask[1] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK1);

	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);

	// use 4Ren-JIG testing ID for LCM unique ID
	flash_read(&g_ts, ADDR_ID_JIG_4REN, LEN_ID_JIG_4REN, idBuf);
	idBuf[11] = TYPE_ID_JIG_4REN;

	if (idBuf[0] == 0xFF) {
		// if no data, use another for LCM unique ID
		flash_read(&g_ts, ADDR_ID_JIG_LPC, LEN_ID_JIG_LPC, idBuf);
		idBuf[11] = TYPE_ID_JIG_LPC;
	}

	// restore irqmask
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, irqmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, irqmask[1]);

	issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);

	return ret;
}

static ssize_t get_unique_id_wo_IRQ(char *idBuf)
{
	int ret = 0;
	unsigned char irqmask[2];

	// read LCM ID
	issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN);
	// save reg values
	irqmask[0] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK0);
	irqmask[1] = ReadOneByte(&g_ts,(unsigned char)SHTSC_ADDR_INTMASK1);

	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);

	// use 4Ren-JIG testing ID for LCM unique ID
	flash_read(&g_ts, ADDR_ID_JIG_4REN, LEN_ID_JIG_4REN, idBuf);
	idBuf[11] = TYPE_ID_JIG_4REN;

	if (idBuf[0] == 0xFF) {
		// if no data, use another for LCM unique ID
		flash_read(&g_ts, ADDR_ID_JIG_LPC, LEN_ID_JIG_LPC, idBuf);
		idBuf[11] = TYPE_ID_JIG_LPC;
	}

	// restore irqmask
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, irqmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, irqmask[1]);

	issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN);

	return ret;
}

static int get_unique_id_from_cache(struct i2c_client *client, char *buffer, int bufferLength)
{
	TouchDriverData *pDriverData = NULL;
	unsigned char read_buffer[16];

	TOUCH_FUNC();

	if(buffer == NULL){
		TOUCH_LOG("buffer is null \n");
		return TOUCH_FAIL;
	}

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	} else {
		TOUCH_LOG("client is null \n");
		return TOUCH_FAIL;
	}
	if(bufferLength < UNIQUE_ID_BUFFER_MAX){
		TOUCH_LOG("buffer length is smalll %d\n", bufferLength);
		return TOUCH_FAIL;
	}

	memset(read_buffer, 0x0, sizeof(read_buffer));

	if(pDriverData != NULL && pDriverData->bootMode == BOOT_MFTS){
		if( get_unique_id_wo_IRQ(read_buffer) == TOUCH_SUCCESS){

			if (read_buffer[11] == TYPE_ID_JIG_4REN) {
				snprintf(buffer, bufferLength, "%02X%02X%02X-%04X-%04X-%02d-%04X\n",
						read_buffer[0],
						read_buffer[1],
						read_buffer[2],
						((read_buffer[4] << 8) | read_buffer[3]), //little endian
						((read_buffer[7] << 8) | read_buffer[6]), //little endian
						read_buffer[5],
						((read_buffer[9] << 8) | read_buffer[8])); //little endian
			} else {
				snprintf(buffer, bufferLength, "%02d%02d%02d-%04X-%04X\n",
						read_buffer[0],
						read_buffer[1],
						read_buffer[2],
						((read_buffer[4] << 8) | read_buffer[3]),
						((read_buffer[6] << 8) | read_buffer[5])); //little endian
			}

			return TOUCH_SUCCESS;
		} else {
			return TOUCH_FAIL;
		}
	}

	if( unique_id_saved == 1){
		if(strlen(LCM_unique_id_Buf)<sizeof(LCM_unique_id_Buf)){
			memcpy(buffer, LCM_unique_id_Buf, strlen(LCM_unique_id_Buf));
		}
		return TOUCH_SUCCESS;

	} else {

		int result = TOUCH_FAIL;

		memset(LCM_unique_id_Buf, 0x0, sizeof(LCM_unique_id_Buf));

		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			result = get_unique_id_wo_IRQ(read_buffer);
		} else {
			result = get_unique_id(read_buffer);
		}

		if( result  == TOUCH_SUCCESS){
			if (read_buffer[11] == TYPE_ID_JIG_4REN) {
				snprintf(buffer, bufferLength, "%02X%02X%02X-%04X-%04X-%02d-%04X\n",
						read_buffer[0],
						read_buffer[1],
						read_buffer[2],
						((read_buffer[4] << 8) | read_buffer[3]), //little endian
						((read_buffer[7] << 8) | read_buffer[6]), //little endian
						read_buffer[5],
						((read_buffer[9] << 8) | read_buffer[8])); //little endian
			} else {
				snprintf(buffer, bufferLength, "%02d%02d%02d-%04X-%04X\n",
						read_buffer[0],
						read_buffer[1],
						read_buffer[2],
						((read_buffer[4] << 8) | read_buffer[3]),
						((read_buffer[6] << 8) | read_buffer[5])); //little endian
			}
			if(strlen(buffer)<sizeof(LCM_unique_id_Buf)){
				memcpy(LCM_unique_id_Buf, buffer, strlen(buffer));
			}
			TOUCH_LOG("Unique ID (Cached) : %s\n", LCM_unique_id_Buf);

			unique_id_saved = 1;
		} else {
			TOUCH_LOG("unique id read failed\n");
			return TOUCH_FAIL;
		}
	}

	return TOUCH_SUCCESS;
}

static ssize_t shtsc_proc_updatecalib_func(struct i2c_client *client, char *buf, int len, int* result)
{
	int ret = 0;
	int drvNum = 0;
	int snsNum = 0;
	int snsNum2 = 0;
	unsigned char *calibbuf = NULL;
	unsigned char intmask[2] = {0,};
	int remain = 0;
	int bufidx = CALIB_HEADER_SIZE;
	int address = CALIBRATION_DATA_SRAM_ADDRESS;
	int datasize = 0;
	TouchDriverData *pDriverData = NULL;

	TOUCH_FUNC();

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}

	/* get panel param */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_GETPANELPARAM, CMD_GETPANELPARAM_LEN)) {
			TOUCH_LOG("CMD_GETPANELPARAM failed\n");
			return ret;
		}
	} else {
		if(issue_command(&g_ts, CMD_GETPANELPARAM, CMD_GETPANELPARAM_LEN)) {
			TOUCH_LOG("CMD_GETPANELPARAM failed\n");
			return ret;
		}
	}

	snsNum = CommandResultBuf[3];
	drvNum = CommandResultBuf[4];

	if( snsNum * drvNum > MAX_FRAME_SIZE ){
		TOUCH_LOG("GET_PANELPARAM value too big %d %d\n", snsNum, drvNum);
		return ret;
	}
	if( snsNum&1 ) snsNum2 = snsNum+1;
	else snsNum2 = snsNum;

	/* allocate memory for calibration data */
	calibbuf = kzalloc(CALIBRATION_DATA_FLASH_SIZE, GFP_KERNEL);
	if( !calibbuf ){
		TOUCH_LOG("GET_PANELPARAM malloc error\n");
		return ret;
	}
	memset(calibbuf, 0xff, CALIBRATION_DATA_FLASH_SIZE);
	intmask[0] = ReadOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0);
	intmask[1] = ReadOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1);

	/* calibraion(calfactor=0,time=100ms) */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE failed\n");
			goto updatecalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE failed\n");
			goto updatecalib_exit;
		}
	}

	/* calibraion(calfactor=7,time=1000ms) */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE2, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE2 failed\n");
			goto updatecalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE2, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE2 failed\n");
			goto updatecalib_exit;
		}
	}
	/* set system state to sleep */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
			goto updatecalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
			goto updatecalib_exit;
		}
	}


	/* ReadSRAM */
	/* disable interrupt */
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	/* Change bank to 5 */
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_BANK, (unsigned char)0x05);
	remain = snsNum2 * drvNum * 4;
	while(remain){
		int remainLocal = 0;
		/* Set SRAM Address */
		WriteOneByte(&g_ts, 0x06, (unsigned char)(address&0xFF));
		WriteOneByte(&g_ts, 0x07, (unsigned char)((address>>8)&0xFF));
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, (unsigned char)((address>>16)&0xFF));
		remainLocal = remain;
		if( remainLocal > 120 ){
			remainLocal = 120;
		}

		//TOUCH_LOG("shtsc-updatecalib-test:info sram read address=%08x size=%d\n",address,remainLocal);

		/* Change bank to 5 */
		WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_BANK, (unsigned char)0x05);
		ReadMultiBytes(&g_ts,0x08,remainLocal,calibbuf+bufidx);
		remain -= remainLocal;
		address += remainLocal;
		bufidx += remainLocal;
	}

	/* build calib data */
	{
		unsigned long checksum = 0;
		int cnt = 0;
		/* Data number */
		calibbuf[0] = (bufidx&0xFF);
		calibbuf[1] = ((bufidx>>8)&0xFF);
		/* Type */
		calibbuf[2] = 0;
		/* Sense data number */
		calibbuf[3] = snsNum;
		/* Drive data number */
		calibbuf[4] = drvNum;
		/* Calculate check sum */
		for( cnt=0 ; cnt<bufidx ; cnt++ ){
			checksum += calibbuf[cnt];
		}
		calibbuf[bufidx+0] = ((checksum>>0)&0xFF);
		calibbuf[bufidx+1] = ((checksum>>8)&0xFF);
		calibbuf[bufidx+2] = ((checksum>>16)&0xFF);
	}
	datasize = (bufidx+CALIB_CHECKSUM_SIZE);
	/* build calib footer */
	if( (datasize + CALIB_PAD_SIZE+CALIB_FOOTER_SIZE)  < CALIBRATION_DATA_FLASH_SIZE ){
		struct timeval tmval;
		struct tm timelocal;
		unsigned char *calibfoot=calibbuf+bufidx+CALIB_CHECKSUM_SIZE+CALIB_PAD_SIZE;

		do_gettimeofday(&tmval);
		time_to_tm(tmval.tv_sec,0,&timelocal);

		calibfoot[0]  = CALIB_TOOL_TYPE_DRIVER;	/* written by driver */
		calibfoot[1]  = VERSION_SERIAL_NUMBER/100+'0';
		calibfoot[2]  = VERSION_SERIAL_NUMBER%100/10+'0';
		calibfoot[3]  = VERSION_SERIAL_NUMBER%10+'0';
		calibfoot[4]  = '\0';
		calibfoot[5]  = 0xFF;	/* firmware version (no data) */
		calibfoot[6]  = 0xFF;
		calibfoot[7]  = 0xFF;
		calibfoot[8]  = 0xFF;
		calibfoot[9]  = 0xFF;	/* parameter version (no data) */
		calibfoot[10] = 0xFF;
		calibfoot[11] = 0xFF;
		calibfoot[12] = 0xFF;
		calibfoot[13] = (timelocal.tm_year)%100;	/* Year */
		calibfoot[14] = (timelocal.tm_mon)+1;	/* Month */
		calibfoot[15] = (timelocal.tm_mday);	/* Day */
		calibfoot[16] = (timelocal.tm_hour);	/* Hour */
		calibfoot[17] = (timelocal.tm_min);	/* Minute */
		calibfoot[18] = (timelocal.tm_sec);	/* Second */
		calibfoot[19] = 0xFF;			/* Reserved */
		calibfoot[20] = 0xFF;
		calibfoot[21] = 0xFF;
		calibfoot[22] = 0xFF;
		calibfoot[23] = 0xFF;
		calibfoot[24] = 0xFF;

		datasize += CALIB_PAD_SIZE+CALIB_FOOTER_SIZE;
	}

	/* Erase calibration data */
	if( flash_erase_page_shtsc(&g_ts, CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE) ) {
		TOUCH_LOG("shtsc: flash_erase_page_shtsc done: page %d\n",  CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE);
		goto updatecalib_exit;
	}
	if( datasize > FLASH_PAGE_SIZE ){
		if( flash_erase_page_shtsc(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE+1) ){
			TOUCH_LOG("shtsc: flash_erase_page_shtsc done: page %d\n",  CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE+1);
			goto updatecalib_exit;
		}
	}

	/* Write calibration data to flash */
	if( flash_write_page_shtsc(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE,calibbuf) ){
		TOUCH_LOG("Error:flash write error\n");
		goto updatecalib_exit;
	}
	if( datasize > FLASH_PAGE_SIZE ){
		if( flash_write_page_shtsc(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE+1,calibbuf+FLASH_PAGE_SIZE) ){
			TOUCH_LOG("Error:flash write error\n");
			goto updatecalib_exit;
		}
	}

	/* Verify */
	if( flash_verify_page_shtsc(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE,calibbuf) ){
		TOUCH_LOG("Error:flash verify error\n");
		goto updatecalib_exit;
	}
	if( datasize > FLASH_PAGE_SIZE ){
		if( flash_verify_page_shtsc(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS/FLASH_PAGE_SIZE+1,calibbuf+FLASH_PAGE_SIZE) ){
			TOUCH_LOG("Error:flash verify error\n");
			goto updatecalib_exit;
		}
	}

	*result = TOUCH_SUCCESS;
	WRITE_SYSBUF(buf, ret, "Calibration update : OK\n");

updatecalib_exit:
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, intmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, intmask[1]);

	/* set system state to idle */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if( issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN) ) {
			TOUCH_LOG("Warning:command issue(set system state(idle))\n");
		}
	} else {
		if( issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN) ) {
			TOUCH_LOG("Warning:command issue(set system state(idle))\n");
		}
	}

	if(calibbuf != NULL)
		kfree(calibbuf);
	return ret;
}

static ssize_t shtsc_proc_readcalib_func(struct i2c_client *client, char* buf, int len, int* result)
{
	int ret = 0;
	int drvNum = 0;
	int snsNum = 0;
	int snsNum2 = 0;
	unsigned char *calibbuf = NULL;
	unsigned char intmask[2] = {0, };
	int caldatasize = 0;
	int footer = 0;
	int cntx = 0;
	int *pcalib = NULL;
	TouchDriverData *pDriverData = NULL;

	TOUCH_FUNC();

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}

	/* allocate memory for calibration data */
	calibbuf = kzalloc(CALIBRATION_DATA_FLASH_SIZE, GFP_KERNEL);
	if( !calibbuf ){
		TOUCH_LOG("malloc failed(1)\n");
		return ret;
	}

	intmask[0] = ReadOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0);
	intmask[1] = ReadOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1);

	/* set system state to sleep */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
			goto readcalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
			goto readcalib_exit;
		}
	}

	/* disable interrupt */
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	/* Check flash area for calib */
	flash_read(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS,2,calibbuf);
	caldatasize = (calibbuf[0]|(calibbuf[1]<<8));
	TOUCH_LOG("HMOON read  caldatasize %d -- [0] %d [1] %d\n", caldatasize, calibbuf[0], calibbuf[1]);
	if( caldatasize == 0xFFFF ){
		TOUCH_LOG("Error:no calibration data\n");
		goto readcalib_exit;
	}
	if( (caldatasize + CALIB_CHECKSUM_SIZE) > CALIBRATION_DATA_FLASH_SIZE ){
		TOUCH_LOG("Error:calibration data size error : %d\n", caldatasize);
		goto readcalib_exit;
	}
	if( (caldatasize + CALIB_CHECKSUM_SIZE + CALIB_PAD_SIZE + CALIB_FOOTER_SIZE) <= CALIBRATION_DATA_FLASH_SIZE ){
		flash_read(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS,caldatasize + CALIB_CHECKSUM_SIZE + CALIB_PAD_SIZE + CALIB_FOOTER_SIZE,calibbuf);
		footer = 1;
	}
	else{
		flash_read(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS,caldatasize+CALIB_CHECKSUM_SIZE,calibbuf);
	}

	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, intmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, intmask[1]);

	if( footer && calibbuf[caldatasize + CALIB_CHECKSUM_SIZE + CALIB_PAD_SIZE] == 0xFF ){
		footer = 0;
	}

	snsNum = calibbuf[3];
	drvNum = calibbuf[4];

	if( snsNum & 1 ) snsNum2 = snsNum+1;
	else snsNum2 = snsNum;

	if( ( snsNum2 * drvNum * 4)  > (caldatasize - CALIB_HEADER_SIZE) ){
		TOUCH_LOG("Error:calibration data size error2\n");
		goto readcalib_exit;
	}
	if( snsNum+1 > MAX_CH_LINE_NUM || drvNum > MAX_CH_LINE_NUM || snsNum + 1 + drvNum > MAX_TOTAL_LINE_NUM ){
		TOUCH_LOG("Error:calibration data size error3\n");
		goto readcalib_exit;
	}

	TOUCH_LOG("Info:calibration data\n");
	pcalib = (int *)(calibbuf + CALIB_HEADER_SIZE);
	for( cntx = 0 ; cntx < snsNum ; cntx++ ){
		WRITE_SYSBUF(buf, ret, "[%02d] %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\n",\
				cntx + 1,\
				pcalib[cntx +  0 * snsNum2] / 4096,\
				pcalib[cntx +  1 * snsNum2] / 4096,\
				pcalib[cntx +  2 * snsNum2] / 4096,\
				pcalib[cntx +  3 * snsNum2] / 4096,\
				pcalib[cntx +  4 * snsNum2] / 4096,\
				pcalib[cntx +  5 * snsNum2] / 4096,\
				pcalib[cntx +  6 * snsNum2] / 4096,\
				pcalib[cntx +  7 * snsNum2] / 4096,\
				pcalib[cntx +  8 * snsNum2] / 4096,\
				pcalib[cntx +  9 * snsNum2] / 4096,\
				pcalib[cntx + 10 * snsNum2] / 4096,\
				pcalib[cntx + 11 * snsNum2] / 4096,\
				pcalib[cntx + 12 * snsNum2] / 4096,\
				pcalib[cntx + 13 * snsNum2] / 4096,\
				pcalib[cntx + 14 * snsNum2] / 4096,\
				pcalib[cntx + 15 * snsNum2] / 4096,\
				pcalib[cntx + 16 * snsNum2] / 4096,\
				pcalib[cntx + 17 * snsNum2] / 4096,\
				pcalib[cntx + 18 * snsNum2] / 4096,\
				pcalib[cntx + 19 * snsNum2] / 4096,\
				pcalib[cntx + 20 * snsNum2] / 4096,\
				pcalib[cntx + 21 * snsNum2] / 4096,\
				pcalib[cntx + 22 * snsNum2] / 4096,\
				pcalib[cntx + 23 * snsNum2] / 4096,\
				pcalib[cntx + 24 * snsNum2] / 4096,\
				pcalib[cntx + 25 * snsNum2] / 4096,\
				pcalib[cntx + 26 * snsNum2] / 4096,\
				pcalib[cntx + 27 * snsNum2] / 4096,\
				pcalib[cntx + 28 * snsNum2] / 4096,\
				pcalib[cntx + 29 * snsNum2] / 4096,\
				pcalib[cntx + 30 * snsNum2] / 4096);

	}
	if( footer ){
		unsigned char *pfooter= calibbuf + caldatasize + CALIB_CHECKSUM_SIZE + CALIB_PAD_SIZE;
		//TOUCH_LOG("Tool ID    = %02x\n",pfooter[0]);
		//TOUCH_LOG("Tool  ver. = %02x %02x %02x %02x \n",pfooter[1],pfooter[2],pfooter[3],pfooter[4]);
		//TOUCH_LOG("Firm  ver. = %02x%02x%02x%02x\n",pfooter[8],pfooter[7],pfooter[6],pfooter[5]);
		//TOUCH_LOG("Param ver. = %02x%02x%02x%02x\n",pfooter[12],pfooter[11],pfooter[10],pfooter[9]);
		//TOUCH_LOG("Day        = %2d/%2d/%2d\n",pfooter[13],pfooter[14],pfooter[15]);
		//TOUCH_LOG("Time       = %2d:%02d:%02d\n",pfooter[16],pfooter[17],pfooter[18]);
		//TOUCH_LOG("Reserved   = %02x %02x %02x %02x %02x %02x\n",pfooter[19],pfooter[20],pfooter[21],pfooter[22],pfooter[23],pfooter[24]);

		WRITE_SYSBUF(buf, ret, "Tool ID    = %02x\n",pfooter[0]);
		WRITE_SYSBUF(buf, ret, "Tool  ver. = %02x %02x %02x %02x \n",pfooter[1],pfooter[2],pfooter[3],pfooter[4]);
		WRITE_SYSBUF(buf, ret, "Firm  ver. = %02x%02x%02x%02x\n",pfooter[8],pfooter[7],pfooter[6],pfooter[5]);
		WRITE_SYSBUF(buf, ret, "Param ver. = %02x%02x%02x%02x\n",pfooter[12],pfooter[11],pfooter[10],pfooter[9]);
		WRITE_SYSBUF(buf, ret, "Day        = %2d/%2d/%2d\n",pfooter[13],pfooter[14],pfooter[15]);
		WRITE_SYSBUF(buf, ret, "Time       = %2d:%02d:%02d\n",pfooter[16],pfooter[17],pfooter[18]);
		WRITE_SYSBUF(buf, ret, "Reserved   = %02x %02x %02x %02x %02x %02x\n",pfooter[19],pfooter[20],pfooter[21],pfooter[22],pfooter[23],pfooter[24]);
	}
	else{
		//TOUCH_LOG("Info(footer):no data\n");
		WRITE_SYSBUF(buf, ret, "No footer data\n");
	}

	*result = TOUCH_SUCCESS;
	TOUCH_LOG("OK:read calibration data\n");

readcalib_exit:
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, intmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, intmask[1]);
	/* set system state to idle */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if( issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN) ) {
			TOUCH_LOG("Warning:command issue(set system state(idle))\n");
		}
	} else {
		if( issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN) ) {
			TOUCH_LOG("Warning:command issue(set system state(idle))\n");
		}
	}
	if(calibbuf != NULL)
		kfree(calibbuf);
	return ret;
}

int readForcedCalib(TouchDriverData *pDriverData, unsigned char *rambuf, int snsNum2, int drvNum, int count)
{
	int ret = TOUCH_FAIL;
	int remain = 0;
	int bufidx = 0;
	int address = CALIBRATION_DATA_SRAM_ADDRESS;
	int cntx = 0;
	int *pram = NULL;

	if( count == 0 ) {
		/* set system state to sleep */
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			if(issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
				TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
				ret = TOUCH_FAIL;
				goto readForcedCalib_exit;
			}
		} else {
			if(issue_command(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
				TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
				ret = TOUCH_FAIL;
				goto readForcedCalib_exit;
			}
		}
	}

	/* set system state to sleep */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_IDLE failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_IDLE failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	}

	/* calibraion(calfactor=0,time=100ms) */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	}

	/* calibraion(calfactor=7,time=1000ms) */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE2, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE2 failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_EXECCALIBRATION_FOR_WRITE2, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE2 failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	}

	/* set system state to sleep */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	} else {
		if(issue_command(&g_ts, CMD_SETSYSTEMSTATE_SLEEP, CMD_SETSYSTEMSTATE_SLEEP_LEN)) {
			TOUCH_LOG("CMD_SETSYSTEMSTATE_SLEEP failed\n");
			ret = TOUCH_FAIL;
			goto readForcedCalib_exit;
		}
	}

	/* ReadSRAM */
	/* disable interrupt */
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, (unsigned char)0xff);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, (unsigned char)0x03);
	/* Change bank to 5 */
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_BANK, (unsigned char)0x05);
	remain = snsNum2 * drvNum * 4;
	while(remain){
		int remainLocal = 0;
		/* Set SRAM Address */
		WriteOneByte(&g_ts, 0x06, (unsigned char)(address&0xFF));
		WriteOneByte(&g_ts, 0x07, (unsigned char)((address>>8)&0xFF));
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, (unsigned char)((address>>16)&0xFF));
		remainLocal = remain;
		if( remainLocal > 120 ) {
			remainLocal = 120;
		}

		//TOUCH_LOG("shtsc-checkcalib-test:info sram read address=%08x size=%d\n",address,remainLocal);

		/* Change bank to 5 */
		WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_BANK, (unsigned char)0x05);
		ReadMultiBytes(&g_ts,0x08,remainLocal,rambuf+bufidx);
		remain -= remainLocal;
		address += remainLocal;
		bufidx += remainLocal;
	}


	if( pDriverData != NULL && pDriverData->bootMode == BOOT_NORMAL ) {

		pram = (int *)(rambuf);

		//WRITE_RESULT(buf, ret, "Forced calibration data (RAM)\n");
		TOUCH_LOG("Forced calibration data (RAM) - %d - \n", count);

		for( cntx = 0 ; cntx < (snsNum2 - 1) ; cntx++ ){
			TOUCH_LOG("[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
					cntx + 1,\
					pram[cntx +  0 * snsNum2] / 4096,\
					pram[cntx +  1 * snsNum2] / 4096,\
					pram[cntx +  2 * snsNum2] / 4096,\
					pram[cntx +  3 * snsNum2] / 4096,\
					pram[cntx +  4 * snsNum2] / 4096,\
					pram[cntx +  5 * snsNum2] / 4096,\
					pram[cntx +  6 * snsNum2] / 4096,\
					pram[cntx +  7 * snsNum2] / 4096,\
					pram[cntx +  8 * snsNum2] / 4096,\
					pram[cntx +  9 * snsNum2] / 4096,\
					pram[cntx + 10 * snsNum2] / 4096,\
					pram[cntx + 11 * snsNum2] / 4096,\
					pram[cntx + 12 * snsNum2] / 4096,\
					pram[cntx + 13 * snsNum2] / 4096,\
					pram[cntx + 14 * snsNum2] / 4096,\
					pram[cntx + 15 * snsNum2] / 4096,\
					pram[cntx + 16 * snsNum2] / 4096,\
					pram[cntx + 17 * snsNum2] / 4096,\
					pram[cntx + 18 * snsNum2] / 4096,\
					pram[cntx + 19 * snsNum2] / 4096,\
					pram[cntx + 20 * snsNum2] / 4096,\
					pram[cntx + 21 * snsNum2] / 4096,\
					pram[cntx + 22 * snsNum2] / 4096,\
					pram[cntx + 23 * snsNum2] / 4096,\
					pram[cntx + 24 * snsNum2] / 4096,\
					pram[cntx + 25 * snsNum2] / 4096,\
					pram[cntx + 26 * snsNum2] / 4096,\
					pram[cntx + 27 * snsNum2] / 4096,\
					pram[cntx + 28 * snsNum2] / 4096,\
					pram[cntx + 29 * snsNum2] / 4096,\
					pram[cntx + 30 * snsNum2] / 4096);
		}
	}

	ret = TOUCH_SUCCESS;

readForcedCalib_exit:

	return ret;
}

static ssize_t shtsc_proc_checkcalib_func(struct i2c_client *client, char *buf, int len, int* result, int* diffMaxValue)
{
	int ret = len;
	int drvNum = 0;
	int snsNum = 0;
	int snsNum2 = 0;
	unsigned char *calibbuf = NULL;
	unsigned char *rambuf = NULL;
	int *diffbuf = NULL;
	unsigned char *rambuf1 = NULL;
	unsigned char *rambuf2 = NULL;
	unsigned char *rambuf3 = NULL;
	unsigned char *rambuf4 = NULL;
	unsigned char *rambuf5 = NULL;
	unsigned char intmask[2] = {0,};
	int caldatasize = 0;
	int footer = 0;
	int cntx = 0;
	int cnty = 0;
	int *pcalib = NULL;
	int *pram = NULL;
	int *pramNew = NULL;
	int *pramOld = NULL;
	int errcnt = 0;
	int max_diff = 0;
	int max_diff_edge = 0;
	int max_diff_repeat = 0;
	TouchDriverData *pDriverData = NULL;

	TOUCH_FUNC();

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("debug: Command-done bit cleared before GetPanelParam? : %0XH\n", (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)));
#endif

	/* get panel param */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_GETPANELPARAM, CMD_GETPANELPARAM_LEN)) {
			TOUCH_LOG("CMD_GETPANELPARAM failed\n");
			return ret;
		}
	} else {
		if(issue_command(&g_ts, CMD_GETPANELPARAM, CMD_GETPANELPARAM_LEN)) {
			TOUCH_LOG("CMD_GETPANELPARAM failed\n");
			return ret;
		}
	}

	snsNum = CommandResultBuf[3];
	drvNum = CommandResultBuf[4];

	if( ((snsNum + 1) > MAX_CH_LINE_NUM) || (drvNum > MAX_CH_LINE_NUM) || ((snsNum + 1 + drvNum) > MAX_TOTAL_LINE_NUM) ){
		TOUCH_LOG("GET_PANELPARAM value too big %d %d\n", snsNum, drvNum);
		return ret;
	}

	if( snsNum&1 ){
		snsNum2 = snsNum+1;
	} else {
		snsNum2 = snsNum;
	}

	/* allocate memory for calibration data */
	calibbuf = kzalloc(CALIBRATION_DATA_FLASH_SIZE, GFP_KERNEL);
	if( !calibbuf ){
		TOUCH_LOG("malloc error(1)\n");
		return ret;
	}

	/* allocate memory for calibration data */
	rambuf = kzalloc(snsNum2*drvNum*sizeof(unsigned int), GFP_KERNEL);
	if( !rambuf ){
		if(calibbuf != NULL)
			kfree(calibbuf);
		TOUCH_LOG("malloc error(2)\n");
		return ret;
	}

	diffbuf = kzalloc(snsNum2 * drvNum*sizeof(int), GFP_KERNEL);
	if( !diffbuf ){
		if(calibbuf != NULL)
			kfree(calibbuf);
		if(rambuf != NULL)
			kfree(rambuf);
		TOUCH_LOG("malloc error(3)\n");
		return ret;
	}

	//if( pDriverData != NULL && pDriverData->bootMode == BOOT_MFTS ) {
	if( 0 ) {

		rambuf1 = kzalloc(snsNum2*drvNum*sizeof(unsigned int), GFP_KERNEL);
		if( !rambuf1 ){
			if(calibbuf != NULL)
				kfree(calibbuf);
			if(rambuf != NULL)
				kfree(rambuf);
			if(diffbuf != NULL )
				kfree(diffbuf);
			TOUCH_LOG("malloc error(4)\n");
			return ret;
		}


		rambuf2 = kzalloc(snsNum2*drvNum*sizeof(unsigned int), GFP_KERNEL);
		if( !rambuf2 ){
			if(calibbuf != NULL)
				kfree(calibbuf);
			if(rambuf != NULL)
				kfree(rambuf);
			if(diffbuf != NULL)
				kfree(diffbuf);
			if(rambuf1 != NULL)
				kfree(rambuf1);
			TOUCH_LOG("malloc error(5)\n");
			return ret;
		}

		rambuf3 = kzalloc(snsNum2*drvNum*sizeof(unsigned int), GFP_KERNEL);
		if( !rambuf3 ){
			if(calibbuf != NULL)
				kfree(calibbuf);
			if(rambuf != NULL)
				kfree(rambuf);
			if(diffbuf != NULL)
				kfree(diffbuf);
			if(rambuf1 != NULL)
				kfree(rambuf1);
			if(rambuf2 != NULL)
				kfree(rambuf2);
			TOUCH_LOG("malloc error(6)\n");
			return ret;
		}

		rambuf4 = kzalloc(snsNum2*drvNum*sizeof(unsigned int), GFP_KERNEL);
		if( !rambuf4 ){
			if(calibbuf != NULL)
				kfree(calibbuf);
			if(rambuf != NULL)
				kfree(rambuf);
			if(diffbuf != NULL)
				kfree(diffbuf);
			if(rambuf1 != NULL)
				kfree(rambuf1);
			if(rambuf2 != NULL)
				kfree(rambuf2);
			if(rambuf3 != NULL)
				kfree(rambuf3);
			TOUCH_LOG("malloc error(7)\n");
			return ret;
		}

		rambuf5 = kzalloc(snsNum2*drvNum*sizeof(unsigned int), GFP_KERNEL);
		if( !rambuf5 ){
			if(calibbuf != NULL)
				kfree(calibbuf);
			if(rambuf != NULL)
				kfree(rambuf);
			if(diffbuf != NULL)
				kfree(diffbuf);
			if(rambuf1 != NULL)
				kfree(rambuf1);
			if(rambuf2 != NULL)
				kfree(rambuf2);
			if(rambuf3 != NULL)
				kfree(rambuf3);
			if(rambuf4 != NULL)
				kfree(rambuf4);
			TOUCH_LOG("malloc error(8)\n");
			return ret;
		}
	}

	// Clear Interrupt if bit masked
	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	intmask[0] = ReadOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0);
	intmask[1] = ReadOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1);

	if( (rambuf != NULL) && (TOUCH_FAIL == readForcedCalib(pDriverData, rambuf, snsNum2, drvNum, 0))){
		TOUCH_LOG("Error: rambuf\n");
		goto checkcalib_exit;
	}
	if( (rambuf1 != NULL ) && (TOUCH_FAIL == readForcedCalib(pDriverData, rambuf1, snsNum2, drvNum, 1))){
		TOUCH_LOG("Error: rambuf1\n");
		goto checkcalib_exit;
	}
	if( (rambuf2 != NULL ) && (TOUCH_FAIL == readForcedCalib(pDriverData, rambuf2, snsNum2, drvNum, 2))){
		TOUCH_LOG("Error: rambuf2\n");
		goto checkcalib_exit;
	}
	if( (rambuf3 != NULL) && (TOUCH_FAIL == readForcedCalib(pDriverData, rambuf3, snsNum2, drvNum, 3))){
		TOUCH_LOG("Error: rambuf3\n");
		goto checkcalib_exit;
	}
	if( (rambuf4 != NULL) && (TOUCH_FAIL == readForcedCalib(pDriverData, rambuf4, snsNum2, drvNum, 4))){
		TOUCH_LOG("Error: rambuf4\n");
		goto checkcalib_exit;
	}
	if( (rambuf5 != NULL) && (TOUCH_FAIL == readForcedCalib(pDriverData, rambuf5, snsNum2, drvNum, 5))){
		TOUCH_LOG("Error: rambuf5\n");
		goto checkcalib_exit;
	}

	/* Read calib data from flash */
	/* Get length of calib data */
	flash_read(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS,2,calibbuf);
	caldatasize = (calibbuf[0]|(calibbuf[1]<<8));
	if( caldatasize == 0xFFFF ){
		TOUCH_LOG("Error:no calibration data\n");
		goto checkcalib_exit;
	}
	if( (caldatasize + CALIB_CHECKSUM_SIZE)  > CALIBRATION_DATA_FLASH_SIZE ){
		TOUCH_LOG("Error:calibration data size error : %d\n", caldatasize);
		goto checkcalib_exit;
	}
	/* read calibration data */
	if( (caldatasize + CALIB_CHECKSUM_SIZE + CALIB_PAD_SIZE + CALIB_FOOTER_SIZE) <= CALIBRATION_DATA_FLASH_SIZE ){
		flash_read(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS,caldatasize + CALIB_CHECKSUM_SIZE + CALIB_PAD_SIZE + CALIB_FOOTER_SIZE,calibbuf);
		footer = 1;
	}
	else{
		flash_read(&g_ts,CALIBRATION_DATA_FLASH_ADDRESS,caldatasize + CALIB_CHECKSUM_SIZE,calibbuf);
	}

	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, intmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, intmask[1]);

	if( footer && calibbuf[caldatasize + CALIB_CHECKSUM_SIZE+CALIB_PAD_SIZE] == 0xFF ){
		footer = 0;
	}

	if( snsNum != calibbuf[3] ||  drvNum != calibbuf[4] ){
		TOUCH_LOG("Error:calibration data size error snsNum %d, drvNum %d\n", snsNum, drvNum);
		goto checkcalib_exit;
	}
	if( (snsNum2 * drvNum * 4) > (caldatasize - CALIB_HEADER_SIZE) ){
		TOUCH_LOG("Error:calibration data size error2\n");
		goto checkcalib_exit;
	}

	pcalib = (int *)(calibbuf + CALIB_HEADER_SIZE);
	pram = (int *)(rambuf);

	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		WRITE_RESULT(buf, ret, "Factory calibration data (Flash)\n");
	}

	for( cntx = 0 ; cntx < snsNum ; cntx++ ){ // 17
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
				cntx + 1,\
				pcalib[cntx +  0 * snsNum2] / 4096,\
				pcalib[cntx +  1 * snsNum2] / 4096,\
				pcalib[cntx +  2 * snsNum2] / 4096,\
				pcalib[cntx +  3 * snsNum2] / 4096,\
				pcalib[cntx +  4 * snsNum2] / 4096,\
				pcalib[cntx +  5 * snsNum2] / 4096,\
				pcalib[cntx +  6 * snsNum2] / 4096,\
				pcalib[cntx +  7 * snsNum2] / 4096,\
				pcalib[cntx +  8 * snsNum2] / 4096,\
				pcalib[cntx +  9 * snsNum2] / 4096,\
				pcalib[cntx + 10 * snsNum2] / 4096,\
				pcalib[cntx + 11 * snsNum2] / 4096,\
				pcalib[cntx + 12 * snsNum2] / 4096,\
				pcalib[cntx + 13 * snsNum2] / 4096,\
				pcalib[cntx + 14 * snsNum2] / 4096,\
				pcalib[cntx + 15 * snsNum2] / 4096,\
				pcalib[cntx + 16 * snsNum2] / 4096,\
				pcalib[cntx + 17 * snsNum2] / 4096,\
				pcalib[cntx + 18 * snsNum2] / 4096,\
				pcalib[cntx + 19 * snsNum2] / 4096,\
				pcalib[cntx + 20 * snsNum2] / 4096,\
				pcalib[cntx + 21 * snsNum2] / 4096,\
				pcalib[cntx + 22 * snsNum2] / 4096,\
				pcalib[cntx + 23 * snsNum2] / 4096,\
				pcalib[cntx + 24 * snsNum2] / 4096,\
				pcalib[cntx + 25 * snsNum2] / 4096,\
				pcalib[cntx + 26 * snsNum2] / 4096,\
				pcalib[cntx + 27 * snsNum2] / 4096,\
				pcalib[cntx + 28 * snsNum2] / 4096,\
				pcalib[cntx + 29 * snsNum2] / 4096,\
				pcalib[cntx + 30 * snsNum2] / 4096);
#endif
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
					cntx + 1,\
					pcalib[cntx +  0 * snsNum2] / 4096,\
					pcalib[cntx +  1 * snsNum2] / 4096,\
					pcalib[cntx +  2 * snsNum2] / 4096,\
					pcalib[cntx +  3 * snsNum2] / 4096,\
					pcalib[cntx +  4 * snsNum2] / 4096,\
					pcalib[cntx +  5 * snsNum2] / 4096,\
					pcalib[cntx +  6 * snsNum2] / 4096,\
					pcalib[cntx +  7 * snsNum2] / 4096,\
					pcalib[cntx +  8 * snsNum2] / 4096,\
					pcalib[cntx +  9 * snsNum2] / 4096,\
					pcalib[cntx + 10 * snsNum2] / 4096,\
					pcalib[cntx + 11 * snsNum2] / 4096,\
					pcalib[cntx + 12 * snsNum2] / 4096,\
					pcalib[cntx + 13 * snsNum2] / 4096,\
					pcalib[cntx + 14 * snsNum2] / 4096,\
					pcalib[cntx + 15 * snsNum2] / 4096,\
					pcalib[cntx + 16 * snsNum2] / 4096,\
					pcalib[cntx + 17 * snsNum2] / 4096,\
					pcalib[cntx + 18 * snsNum2] / 4096,\
					pcalib[cntx + 19 * snsNum2] / 4096,\
					pcalib[cntx + 20 * snsNum2] / 4096,\
					pcalib[cntx + 21 * snsNum2] / 4096,\
					pcalib[cntx + 22 * snsNum2] / 4096,\
					pcalib[cntx + 23 * snsNum2] / 4096,\
					pcalib[cntx + 24 * snsNum2] / 4096,\
					pcalib[cntx + 25 * snsNum2] / 4096,\
					pcalib[cntx + 26 * snsNum2] / 4096,\
					pcalib[cntx + 27 * snsNum2] / 4096,\
					pcalib[cntx + 28 * snsNum2] / 4096,\
					pcalib[cntx + 29 * snsNum2] / 4096,\
					pcalib[cntx + 30 * snsNum2] / 4096);
		}
	}

	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		WRITE_RESULT(buf, ret, "Forced calibration data (RAM)\n");
	}

	for( cntx = 0 ; cntx < snsNum ; cntx++ ){
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
				cntx + 1,\
				pram[cntx +  0 * snsNum2] / 4096,\
				pram[cntx +  1 * snsNum2] / 4096,\
				pram[cntx +  2 * snsNum2] / 4096,\
				pram[cntx +  3 * snsNum2] / 4096,\
				pram[cntx +  4 * snsNum2] / 4096,\
				pram[cntx +  5 * snsNum2] / 4096,\
				pram[cntx +  6 * snsNum2] / 4096,\
				pram[cntx +  7 * snsNum2] / 4096,\
				pram[cntx +  8 * snsNum2] / 4096,\
				pram[cntx +  9 * snsNum2] / 4096,\
				pram[cntx + 10 * snsNum2] / 4096,\
				pram[cntx + 11 * snsNum2] / 4096,\
				pram[cntx + 12 * snsNum2] / 4096,\
				pram[cntx + 13 * snsNum2] / 4096,\
				pram[cntx + 14 * snsNum2] / 4096,\
				pram[cntx + 15 * snsNum2] / 4096,\
				pram[cntx + 16 * snsNum2] / 4096,\
				pram[cntx + 17 * snsNum2] / 4096,\
				pram[cntx + 18 * snsNum2] / 4096,\
				pram[cntx + 19 * snsNum2] / 4096,\
				pram[cntx + 20 * snsNum2] / 4096,\
				pram[cntx + 21 * snsNum2] / 4096,\
				pram[cntx + 22 * snsNum2] / 4096,\
				pram[cntx + 23 * snsNum2] / 4096,\
				pram[cntx + 24 * snsNum2] / 4096,\
				pram[cntx + 25 * snsNum2] / 4096,\
				pram[cntx + 26 * snsNum2] / 4096,\
				pram[cntx + 27 * snsNum2] / 4096,\
				pram[cntx + 28 * snsNum2] / 4096,\
				pram[cntx + 29 * snsNum2] / 4096,\
				pram[cntx + 30 * snsNum2] / 4096);
#endif

		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
					cntx + 1,\
					pram[cntx +  0 * snsNum2] / 4096,\
					pram[cntx +  1 * snsNum2] / 4096,\
					pram[cntx +  2 * snsNum2] / 4096,\
					pram[cntx +  3 * snsNum2] / 4096,\
					pram[cntx +  4 * snsNum2] / 4096,\
					pram[cntx +  5 * snsNum2] / 4096,\
					pram[cntx +  6 * snsNum2] / 4096,\
					pram[cntx +  7 * snsNum2] / 4096,\
					pram[cntx +  8 * snsNum2] / 4096,\
					pram[cntx +  9 * snsNum2] / 4096,\
					pram[cntx + 10 * snsNum2] / 4096,\
					pram[cntx + 11 * snsNum2] / 4096,\
					pram[cntx + 12 * snsNum2] / 4096,\
					pram[cntx + 13 * snsNum2] / 4096,\
					pram[cntx + 14 * snsNum2] / 4096,\
					pram[cntx + 15 * snsNum2] / 4096,\
					pram[cntx + 16 * snsNum2] / 4096,\
					pram[cntx + 17 * snsNum2] / 4096,\
					pram[cntx + 18 * snsNum2] / 4096,\
					pram[cntx + 19 * snsNum2] / 4096,\
					pram[cntx + 20 * snsNum2] / 4096,\
					pram[cntx + 21 * snsNum2] / 4096,\
					pram[cntx + 22 * snsNum2] / 4096,\
					pram[cntx + 23 * snsNum2] / 4096,\
					pram[cntx + 24 * snsNum2] / 4096,\
					pram[cntx + 25 * snsNum2] / 4096,\
					pram[cntx + 26 * snsNum2] / 4096,\
					pram[cntx + 27 * snsNum2] / 4096,\
					pram[cntx + 28 * snsNum2] / 4096,\
					pram[cntx + 29 * snsNum2] / 4096,\
					pram[cntx + 30 * snsNum2] / 4096);
		}
	}

	TOUCH_LOG("Factory calibration Diff Check\n");

	max_diff = 0;
	max_diff_edge = 0;
	for( cntx = 0 ; cntx < snsNum ; cntx++ ){
		for( cnty = 0 ; cnty < drvNum ; cnty++ ){
			int diff_spec = 0;

			int diff = (pcalib[cntx + cnty * snsNum2] / 4096) - (pram[cntx + cnty * snsNum2] / 4096);

			diffbuf[cntx + cnty * snsNum2] = diff;

			diff = abs(diff);


			diff_spec = get_spec_factory_cal_diff(client, cnty, cntx);

			//TOUCH_LOG(" d [ %d ] s [ %d ]  : diff % d spec %d \n", cnty, cntx, diff, diff_spec);


#if 0
			if( cnty == 0  || cnty == 1) {
				TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d : spec %d\n",\
						cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096), diff_spec);
			}
			if( cntx == 15  || cntx == 16) {
				TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d : spec %d\n",\
						cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096), diff_spec);
			}
#endif
			if( diff >= diff_spec){
				errcnt++;
				if(errcnt < 5){
					TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d\n",\
							cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096));
				}
			}
			if ((cntx == 0) || (cntx == 16) || (cnty == 0) || (cnty == 30)) {
				if(max_diff_edge < diff){
					max_diff_edge = diff;
				}
			} else {
				if(max_diff < diff){
					max_diff = diff;
				}
			}
		}
	}

	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		WRITE_RESULT(buf, ret, "[F-0] Factory Cal - Forced Diff Max Center %d, Edge %d (Spec Center: %d, Edge: %d)\n", max_diff, max_diff_edge, get_spec_factory_cal_diff(client, 1, 1), get_spec_factory_cal_diff(client, 0, 0));
	} else {
		WRITE_SYSBUF(buf, ret, "[F-0] Factory Cal - Forced Diff Max Center %d, Edge %d (Spec Center: %d, Edge: %d)\n", max_diff, max_diff_edge, get_spec_factory_cal_diff(client, 1, 1), get_spec_factory_cal_diff(client, 0, 0));
	}

	TOUCH_LOG("Diff Factory Cal - Foreced Cal Max Center %d, Edge %d (Spec Center: %d, Edge: %d) \n", max_diff, max_diff_edge, get_spec_factory_cal_diff(client, 1, 1), get_spec_factory_cal_diff(client, 0, 0));

	for( cntx = 0 ; cntx < snsNum ; cntx++ ){
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
				cntx + 1,\
				diffbuf[cntx +  0 * snsNum2] ,\
				diffbuf[cntx +  1 * snsNum2] ,\
				diffbuf[cntx +  2 * snsNum2] ,\
				diffbuf[cntx +  3 * snsNum2] ,\
				diffbuf[cntx +  4 * snsNum2] ,\
				diffbuf[cntx +  5 * snsNum2] ,\
				diffbuf[cntx +  6 * snsNum2] ,\
				diffbuf[cntx +  7 * snsNum2] ,\
				diffbuf[cntx +  8 * snsNum2] ,\
				diffbuf[cntx +  9 * snsNum2] ,\
				diffbuf[cntx + 10 * snsNum2] ,\
				diffbuf[cntx + 11 * snsNum2] ,\
				diffbuf[cntx + 12 * snsNum2] ,\
				diffbuf[cntx + 13 * snsNum2] ,\
				diffbuf[cntx + 14 * snsNum2] ,\
				diffbuf[cntx + 15 * snsNum2] ,\
				diffbuf[cntx + 16 * snsNum2] ,\
				diffbuf[cntx + 17 * snsNum2] ,\
				diffbuf[cntx + 18 * snsNum2] ,\
				diffbuf[cntx + 19 * snsNum2] ,\
				diffbuf[cntx + 20 * snsNum2] ,\
				diffbuf[cntx + 21 * snsNum2] ,\
				diffbuf[cntx + 22 * snsNum2] ,\
				diffbuf[cntx + 23 * snsNum2] ,\
				diffbuf[cntx + 24 * snsNum2] ,\
				diffbuf[cntx + 25 * snsNum2] ,\
				diffbuf[cntx + 26 * snsNum2] ,\
				diffbuf[cntx + 27 * snsNum2] ,\
				diffbuf[cntx + 28 * snsNum2] ,\
				diffbuf[cntx + 29 * snsNum2] ,\
				diffbuf[cntx + 30 * snsNum2] );
#endif
		WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
				cntx + 1,\
				diffbuf[cntx +  0 * snsNum2] ,\
				diffbuf[cntx +  1 * snsNum2] ,\
				diffbuf[cntx +  2 * snsNum2] ,\
				diffbuf[cntx +  3 * snsNum2] ,\
				diffbuf[cntx +  4 * snsNum2] ,\
				diffbuf[cntx +  5 * snsNum2] ,\
				diffbuf[cntx +  6 * snsNum2] ,\
				diffbuf[cntx +  7 * snsNum2] ,\
				diffbuf[cntx +  8 * snsNum2] ,\
				diffbuf[cntx +  9 * snsNum2] ,\
				diffbuf[cntx + 10 * snsNum2] ,\
				diffbuf[cntx + 11 * snsNum2] ,\
				diffbuf[cntx + 12 * snsNum2] ,\
				diffbuf[cntx + 13 * snsNum2] ,\
				diffbuf[cntx + 14 * snsNum2] ,\
				diffbuf[cntx + 15 * snsNum2] ,\
				diffbuf[cntx + 16 * snsNum2] ,\
				diffbuf[cntx + 17 * snsNum2] ,\
				diffbuf[cntx + 18 * snsNum2] ,\
				diffbuf[cntx + 19 * snsNum2] ,\
				diffbuf[cntx + 20 * snsNum2] ,\
				diffbuf[cntx + 21 * snsNum2] ,\
				diffbuf[cntx + 22 * snsNum2] ,\
				diffbuf[cntx + 23 * snsNum2] ,\
				diffbuf[cntx + 24 * snsNum2] ,\
				diffbuf[cntx + 25 * snsNum2] ,\
				diffbuf[cntx + 26 * snsNum2] ,\
				diffbuf[cntx + 27 * snsNum2] ,\
				diffbuf[cntx + 28 * snsNum2] ,\
				diffbuf[cntx + 29 * snsNum2] ,\
				diffbuf[cntx + 30 * snsNum2] );
	}

	if(pcalib != NULL && rambuf1 != NULL) {
		pramOld = pcalib;
		//pramOld = (int *)rambuf;
		//pramOld = (unsigned int *)(calibbuf + CALIB_HEADER_SIZE);
		pramNew = (int *)rambuf1;
		max_diff_repeat = 0;
		max_diff = 0;
		errcnt = 0;
		for( cntx = 0 ; cntx < snsNum ; cntx++ ){
			for( cnty = 0 ; cnty < drvNum ; cnty++ ){
				int diff = (pramOld[cntx + cnty * snsNum2] / 4096) - (pramNew[cntx + cnty * snsNum2] / 4096);
				//if( abs(diff) > REPEAT_DIFF_SPEC){
				//	TOUCH_LOG("Diff (%d,%d) diff=%d Old %d  New1 %d\n",	cntx, cnty, diff,(pramOld[cntx+cnty*snsNum2]/4096),(pramNew[cntx+cnty*snsNum2]/4096));
				//}
				if( diff >= get_spec_factory_cal_diff(client, cnty, cntx)){
					errcnt++;
					if(errcnt < 5){
						TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d\n",\
								cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096));
					}
				}
				if(max_diff < diff){
					max_diff = diff;
				}
				//if(max_diff_repeat < diff){
				//	max_diff_repeat = diff;
				//}
			}
		}

		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS && pDriverData->bootMode == BOOT_MFTS) {
			for( cntx = 0 ; cntx < snsNum ; cntx++ ){
				WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
						cntx + 1,\
						pramNew[cntx +  0 * snsNum2] / 4096,\
						pramNew[cntx +  1 * snsNum2] / 4096,\
						pramNew[cntx +  2 * snsNum2] / 4096,\
						pramNew[cntx +  3 * snsNum2] / 4096,\
						pramNew[cntx +  4 * snsNum2] / 4096,\
						pramNew[cntx +  5 * snsNum2] / 4096,\
						pramNew[cntx +  6 * snsNum2] / 4096,\
						pramNew[cntx +  7 * snsNum2] / 4096,\
						pramNew[cntx +  8 * snsNum2] / 4096,\
						pramNew[cntx +  9 * snsNum2] / 4096,\
						pramNew[cntx + 10 * snsNum2] / 4096,\
						pramNew[cntx + 11 * snsNum2] / 4096,\
						pramNew[cntx + 12 * snsNum2] / 4096,\
						pramNew[cntx + 13 * snsNum2] / 4096,\
						pramNew[cntx + 14 * snsNum2] / 4096,\
						pramNew[cntx + 15 * snsNum2] / 4096,\
						pramNew[cntx + 16 * snsNum2] / 4096,\
						pramNew[cntx + 17 * snsNum2] / 4096,\
						pramNew[cntx + 18 * snsNum2] / 4096,\
						pramNew[cntx + 19 * snsNum2] / 4096,\
						pramNew[cntx + 20 * snsNum2] / 4096,\
						pramNew[cntx + 21 * snsNum2] / 4096,\
						pramNew[cntx + 22 * snsNum2] / 4096,\
						pramNew[cntx + 23 * snsNum2] / 4096,\
						pramNew[cntx + 24 * snsNum2] / 4096,\
						pramNew[cntx + 25 * snsNum2] / 4096,\
						pramNew[cntx + 26 * snsNum2] / 4096,\
						pramNew[cntx + 27 * snsNum2] / 4096,\
						pramNew[cntx + 28 * snsNum2] / 4096,\
						pramNew[cntx + 29 * snsNum2] / 4096,\
						pramNew[cntx + 30 * snsNum2] / 4096);
			}
		}

		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "[0-1] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		} else {
			WRITE_SYSBUF(buf, ret, "[0-1] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		}
	}

	if(rambuf1 != NULL && rambuf2 != NULL) {
		pramOld = pcalib;
		pramOld = (int *)rambuf1;
		//pramOld = (unsigned int *)(calibbuf + CALIB_HEADER_SIZE);
		pramNew = (int *)rambuf2;
		max_diff_repeat = 0;
		max_diff = 0;
		errcnt = 0;
		for( cntx = 0 ; cntx < snsNum ; cntx++ ){
			for( cnty = 0 ; cnty < drvNum ; cnty++ ){
				int diff = (pramOld[cntx + cnty * snsNum2] / 4096) - (pramNew[cntx + cnty * snsNum2] / 4096);
				//	if( abs(diff) > REPEAT_DIFF_SPEC){
				//		TOUCH_LOG("Diff (%d,%d) diff=%d Old %d  New2 %d\n",cntx, cnty, diff,(pramOld[cntx+cnty*snsNum2]/4096),(pramNew[cntx+cnty*snsNum2]/4096));
				//	}
				if( diff >= get_spec_factory_cal_diff(client, cnty, cntx)){
					errcnt++;
					if(errcnt < 5){
						TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d\n",\
								cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096));
					}
				}
				if(max_diff < diff){
					max_diff = diff;
				}
				//if(max_diff_repeat < diff){
				//	max_diff_repeat = diff;
				//}
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS && pDriverData->bootMode == BOOT_MFTS) {
			for( cntx = 0 ; cntx < snsNum ; cntx++ ){
				WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
						cntx + 1,\
						pramNew[cntx +  0 * snsNum2] / 4096,\
						pramNew[cntx +  1 * snsNum2] / 4096,\
						pramNew[cntx +  2 * snsNum2] / 4096,\
						pramNew[cntx +  3 * snsNum2] / 4096,\
						pramNew[cntx +  4 * snsNum2] / 4096,\
						pramNew[cntx +  5 * snsNum2] / 4096,\
						pramNew[cntx +  6 * snsNum2] / 4096,\
						pramNew[cntx +  7 * snsNum2] / 4096,\
						pramNew[cntx +  8 * snsNum2] / 4096,\
						pramNew[cntx +  9 * snsNum2] / 4096,\
						pramNew[cntx + 10 * snsNum2] / 4096,\
						pramNew[cntx + 11 * snsNum2] / 4096,\
						pramNew[cntx + 12 * snsNum2] / 4096,\
						pramNew[cntx + 13 * snsNum2] / 4096,\
						pramNew[cntx + 14 * snsNum2] / 4096,\
						pramNew[cntx + 15 * snsNum2] / 4096,\
						pramNew[cntx + 16 * snsNum2] / 4096,\
						pramNew[cntx + 17 * snsNum2] / 4096,\
						pramNew[cntx + 18 * snsNum2] / 4096,\
						pramNew[cntx + 19 * snsNum2] / 4096,\
						pramNew[cntx + 20 * snsNum2] / 4096,\
						pramNew[cntx + 21 * snsNum2] / 4096,\
						pramNew[cntx + 22 * snsNum2] / 4096,\
						pramNew[cntx + 23 * snsNum2] / 4096,\
						pramNew[cntx + 24 * snsNum2] / 4096,\
						pramNew[cntx + 25 * snsNum2] / 4096,\
						pramNew[cntx + 26 * snsNum2] / 4096,\
						pramNew[cntx + 27 * snsNum2] / 4096,\
						pramNew[cntx + 28 * snsNum2] / 4096,\
						pramNew[cntx + 29 * snsNum2] / 4096,\
						pramNew[cntx + 30 * snsNum2] / 4096);
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "[1-2] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		} else {
			WRITE_SYSBUF(buf, ret, "[1-2] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		}
	}

	if(rambuf2 != NULL && rambuf3 != NULL) {
		pramOld = pcalib;
		pramOld = (int *)rambuf2;
		//pramOld = (unsigned int *)(calibbuf + CALIB_HEADER_SIZE);
		pramNew = (int *)rambuf3;
		max_diff_repeat = 0;
		max_diff = 0;
		errcnt = 0;
		for( cntx = 0 ; cntx < snsNum ; cntx++ ){
			for( cnty = 0 ; cnty < drvNum ; cnty++ ){
				int diff = (pramOld[cntx + cnty * snsNum2] / 4096) - (pramNew[cntx + cnty * snsNum2] / 4096);
				//if( abs(diff) > REPEAT_DIFF_SPEC){
				//	TOUCH_LOG("Diff (%d,%d) diff=%d Old %d  New3 %d\n",cntx, cnty, diff,(pramOld[cntx+cnty*snsNum2]/4096),(pramNew[cntx+cnty*snsNum2]/4096));
				//}
				if( diff >= get_spec_factory_cal_diff(client,cnty, cntx)){
					errcnt++;
					if(errcnt < 5){
						TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d\n",\
								cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096));
					}
				}
				if(max_diff < diff){
					max_diff = diff;
				}
				//if(max_diff_repeat < diff){
				//	max_diff_repeat = diff;
				//}
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS && pDriverData->bootMode == BOOT_MFTS) {
			for( cntx = 0 ; cntx < snsNum ; cntx++ ){
				WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
						cntx + 1,\
						pramNew[cntx +  0 * snsNum2] / 4096,\
						pramNew[cntx +  1 * snsNum2] / 4096,\
						pramNew[cntx +  2 * snsNum2] / 4096,\
						pramNew[cntx +  3 * snsNum2] / 4096,\
						pramNew[cntx +  4 * snsNum2] / 4096,\
						pramNew[cntx +  5 * snsNum2] / 4096,\
						pramNew[cntx +  6 * snsNum2] / 4096,\
						pramNew[cntx +  7 * snsNum2] / 4096,\
						pramNew[cntx +  8 * snsNum2] / 4096,\
						pramNew[cntx +  9 * snsNum2] / 4096,\
						pramNew[cntx + 10 * snsNum2] / 4096,\
						pramNew[cntx + 11 * snsNum2] / 4096,\
						pramNew[cntx + 12 * snsNum2] / 4096,\
						pramNew[cntx + 13 * snsNum2] / 4096,\
						pramNew[cntx + 14 * snsNum2] / 4096,\
						pramNew[cntx + 15 * snsNum2] / 4096,\
						pramNew[cntx + 16 * snsNum2] / 4096,\
						pramNew[cntx + 17 * snsNum2] / 4096,\
						pramNew[cntx + 18 * snsNum2] / 4096,\
						pramNew[cntx + 19 * snsNum2] / 4096,\
						pramNew[cntx + 20 * snsNum2] / 4096,\
						pramNew[cntx + 21 * snsNum2] / 4096,\
						pramNew[cntx + 22 * snsNum2] / 4096,\
						pramNew[cntx + 23 * snsNum2] / 4096,\
						pramNew[cntx + 24 * snsNum2] / 4096,\
						pramNew[cntx + 25 * snsNum2] / 4096,\
						pramNew[cntx + 26 * snsNum2] / 4096,\
						pramNew[cntx + 27 * snsNum2] / 4096,\
						pramNew[cntx + 28 * snsNum2] / 4096,\
						pramNew[cntx + 29 * snsNum2] / 4096,\
						pramNew[cntx + 30 * snsNum2] / 4096);
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "[2-3] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		} else {
			WRITE_SYSBUF(buf, ret, "[2-3] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		}
	}

	if(rambuf3 != NULL && rambuf4 != NULL) {
		pramOld = pcalib;
		pramOld = (int *)rambuf3;
		//pramOld = (unsigned int *)(calibbuf + CALIB_HEADER_SIZE);
		pramNew = (int *)rambuf4;
		max_diff_repeat = 0;
		max_diff = 0;
		errcnt = 0;
		for( cntx = 0 ; cntx < snsNum ; cntx++ ){
			for( cnty = 0 ; cnty < drvNum ; cnty++ ){
				int diff = (pramOld[cntx + cnty * snsNum2] / 4096) - (pramNew[cntx + cnty * snsNum2] / 4096);
				//	if( abs(diff) > REPEAT_DIFF_SPEC){
				//		TOUCH_LOG("Diff (%d,%d) diff=%d Old %d  New4 %d\n",cntx, cnty, diff,(pramOld[cntx+cnty*snsNum2]/4096),(pramNew[cntx+cnty*snsNum2]/4096));
				//	}
				if( diff >= get_spec_factory_cal_diff(client, cnty, cntx)){
					errcnt++;
					if(errcnt < 5){
						TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d\n",\
								cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096));
					}
				}
				if(max_diff < diff){
					max_diff = diff;
				}
				//if(max_diff_repeat < diff){
				//	max_diff_repeat = diff;
				//}
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS && pDriverData->bootMode == BOOT_MFTS) {
			for( cntx = 0 ; cntx < snsNum ; cntx++ ){
				WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
						cntx + 1,\
						pramNew[cntx +  0 * snsNum2] / 4096,\
						pramNew[cntx +  1 * snsNum2] / 4096,\
						pramNew[cntx +  2 * snsNum2] / 4096,\
						pramNew[cntx +  3 * snsNum2] / 4096,\
						pramNew[cntx +  4 * snsNum2] / 4096,\
						pramNew[cntx +  5 * snsNum2] / 4096,\
						pramNew[cntx +  6 * snsNum2] / 4096,\
						pramNew[cntx +  7 * snsNum2] / 4096,\
						pramNew[cntx +  8 * snsNum2] / 4096,\
						pramNew[cntx +  9 * snsNum2] / 4096,\
						pramNew[cntx + 10 * snsNum2] / 4096,\
						pramNew[cntx + 11 * snsNum2] / 4096,\
						pramNew[cntx + 12 * snsNum2] / 4096,\
						pramNew[cntx + 13 * snsNum2] / 4096,\
						pramNew[cntx + 14 * snsNum2] / 4096,\
						pramNew[cntx + 15 * snsNum2] / 4096,\
						pramNew[cntx + 16 * snsNum2] / 4096,\
						pramNew[cntx + 17 * snsNum2] / 4096,\
						pramNew[cntx + 18 * snsNum2] / 4096,\
						pramNew[cntx + 19 * snsNum2] / 4096,\
						pramNew[cntx + 20 * snsNum2] / 4096,\
						pramNew[cntx + 21 * snsNum2] / 4096,\
						pramNew[cntx + 22 * snsNum2] / 4096,\
						pramNew[cntx + 23 * snsNum2] / 4096,\
						pramNew[cntx + 24 * snsNum2] / 4096,\
						pramNew[cntx + 25 * snsNum2] / 4096,\
						pramNew[cntx + 26 * snsNum2] / 4096,\
						pramNew[cntx + 27 * snsNum2] / 4096,\
						pramNew[cntx + 28 * snsNum2] / 4096,\
						pramNew[cntx + 29 * snsNum2] / 4096,\
						pramNew[cntx + 30 * snsNum2] / 4096);
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "[3-4] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		} else {
			WRITE_SYSBUF(buf, ret, "[3-4] Cal Max Diff Step %d  (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		}
	}

	if(rambuf4 != NULL && rambuf5 != NULL) {
		pramOld = pcalib;
		pramOld = (int *)rambuf4;
		//pramOld = (unsigned int *)(calibbuf + CALIB_HEADER_SIZE);
		pramNew = (int *)rambuf5;
		max_diff_repeat = 0;
		max_diff = 0;
		errcnt = 0;
		for( cntx = 0 ; cntx < snsNum ; cntx++ ){
			for( cnty = 0 ; cnty < drvNum ; cnty++ ){
				int diff = (pramOld[cntx + cnty * snsNum2] / 4096) - (pramNew[cntx + cnty * snsNum2] / 4096);
				//if( abs(diff) > REPEAT_DIFF_SPEC){
				//		TOUCH_LOG("Diff (%d,%d) diff=%d Old %d  New5 %d\n",cntx, cnty, diff,(pramOld[cntx+cnty*snsNum2]/4096),(pramNew[cntx+cnty*snsNum2]/4096));
				//	}
				if( diff >= get_spec_factory_cal_diff(client, cnty, cntx)){
					errcnt++;
					if(errcnt < 5){
						TOUCH_LOG("Diff (%d,%d) diff=%d Factory Cal %d Forced Cal 0th  %d\n",\
								cntx, cnty, diff,(pcalib[cntx+cnty*snsNum2]/4096),(pram[cntx+cnty*snsNum2]/4096));
					}
				}
				if(max_diff < diff){
					max_diff = diff;
				}
				//if(max_diff_repeat < diff){
				//	max_diff_repeat = diff;
				//}
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS && pDriverData->bootMode == BOOT_MFTS) {
			for( cntx = 0 ; cntx < snsNum ; cntx++ ){
				WRITE_RESULT(buf, ret, "[%02d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
						cntx + 1,\
						pramNew[cntx +  0 * snsNum2] / 4096,\
						pramNew[cntx +  1 * snsNum2] / 4096,\
						pramNew[cntx +  2 * snsNum2] / 4096,\
						pramNew[cntx +  3 * snsNum2] / 4096,\
						pramNew[cntx +  4 * snsNum2] / 4096,\
						pramNew[cntx +  5 * snsNum2] / 4096,\
						pramNew[cntx +  6 * snsNum2] / 4096,\
						pramNew[cntx +  7 * snsNum2] / 4096,\
						pramNew[cntx +  8 * snsNum2] / 4096,\
						pramNew[cntx +  9 * snsNum2] / 4096,\
						pramNew[cntx + 10 * snsNum2] / 4096,\
						pramNew[cntx + 11 * snsNum2] / 4096,\
						pramNew[cntx + 12 * snsNum2] / 4096,\
						pramNew[cntx + 13 * snsNum2] / 4096,\
						pramNew[cntx + 14 * snsNum2] / 4096,\
						pramNew[cntx + 15 * snsNum2] / 4096,\
						pramNew[cntx + 16 * snsNum2] / 4096,\
						pramNew[cntx + 17 * snsNum2] / 4096,\
						pramNew[cntx + 18 * snsNum2] / 4096,\
						pramNew[cntx + 19 * snsNum2] / 4096,\
						pramNew[cntx + 20 * snsNum2] / 4096,\
						pramNew[cntx + 21 * snsNum2] / 4096,\
						pramNew[cntx + 22 * snsNum2] / 4096,\
						pramNew[cntx + 23 * snsNum2] / 4096,\
						pramNew[cntx + 24 * snsNum2] / 4096,\
						pramNew[cntx + 25 * snsNum2] / 4096,\
						pramNew[cntx + 26 * snsNum2] / 4096,\
						pramNew[cntx + 27 * snsNum2] / 4096,\
						pramNew[cntx + 28 * snsNum2] / 4096,\
						pramNew[cntx + 29 * snsNum2] / 4096,\
						pramNew[cntx + 30 * snsNum2] / 4096);
			}
		}
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "[4-5] Cal Max Diff Step %d (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		} else {
			WRITE_SYSBUF(buf, ret, "[4-5] Cal Max Diff Step %d (Spec : %d)\n", max_diff_repeat, REPEAT_DIFF_SPEC);
		}
	}

	*diffMaxValue = max_diff;
	if( errcnt ){
		*result = TOUCH_FAIL;
		TOUCH_LOG("Factory calibration check : NG (Max diff Center: %d, Edge: %d )\n", max_diff, max_diff_edge);
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "Calibration : NG (Spec Center: %d Edge: %d Max diff Center: %d, Edge: %d)\n", get_spec_factory_cal_diff(client,1,1), get_spec_factory_cal_diff(client,0,0), max_diff, max_diff_edge);
		} else {
			WRITE_SYSBUF(buf, ret, "Calibration : NG (Spec Center: %d Edge: %d Max diff Center: %d, Edge: %d)\n", get_spec_factory_cal_diff(client,1,1), get_spec_factory_cal_diff(client,0,0), max_diff,max_diff_edge);

		}
	}
	else{
		*result = TOUCH_SUCCESS;
		TOUCH_LOG("Factory calibration check : OK (Max diff Center: %d, Edge: %d )\n", max_diff, max_diff_edge);
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			WRITE_RESULT(buf, ret, "Calibration : OK (Spec Center: %d Edge: %d Max diff Center: %d, Edge: %d)\n", get_spec_factory_cal_diff(client,1,1), get_spec_factory_cal_diff(client,0,0), max_diff, max_diff_edge);
		} else {
			WRITE_SYSBUF(buf, ret, "Calibration : OK (Spec Center: %d Edge: %d Max diff Center: %d, Edge: %d)\n", get_spec_factory_cal_diff(client,1,1), get_spec_factory_cal_diff(client,0,0), max_diff, max_diff_edge);
		}
	}

checkcalib_exit:
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK0, intmask[0]);
	WriteOneByte(&g_ts, (unsigned char)SHTSC_ADDR_INTMASK1, intmask[1]);

	/* set system state to idle */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if( issue_command_wo_IRQ(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN) ) {
			TOUCH_LOG("Warning:command issue(set system state(idle))\n");
		}
	} else {
		if( issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN) ) {
			TOUCH_LOG("Warning:command issue(set system state(idle))\n");
		}
	}

	if( calibbuf != NULL)
		kfree(calibbuf);
	if( rambuf != NULL)
		kfree(rambuf);
	if( diffbuf != NULL)
		kfree(diffbuf);
	if(rambuf1 != NULL)
		kfree(rambuf1);
	if(rambuf2 != NULL)
		kfree(rambuf2);
	if(rambuf3 != NULL)
		kfree(rambuf3);
	if(rambuf4 != NULL)
		kfree(rambuf4);
	if(rambuf5 != NULL)
		kfree(rambuf5);
	return ret;
}

int read_dcmap_wo_irq(struct i2c_client *client)
{
	int ret = 0;
	u8 numDriveLine2 = 0;
	u8 numSenseLine2 = 0;
	u8 num_adc_dmy[3] = {0,};

	unsigned char dsFlag, readingSenseNum, ram_addr[3];

	//TOUCH_FUNC();

	{ // L2
		unsigned int vramAddr = 0;
		unsigned int readingSize = 0;
		u8 tmpbuf[16];

		// get SD/DS and size
		ram_addr[0] = 0x58;
		ram_addr[1] = 0x7F;
		ram_addr[2] = 0x01;
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);

		mutex_lock(&g_ts.mutex_irq);

		SetBankAddr(&g_ts,SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, ram_addr[2]);

		memset(tmpbuf, 0x0, sizeof(tmpbuf));
		SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
		ReadMultiBytes(&g_ts, 0x08, 7, tmpbuf); // less than 8 bytes

		mutex_unlock(&g_ts.mutex_irq);

		numSenseLine2 = tmpbuf[0];
		numDriveLine2 = tmpbuf[1];

		dsFlag = tmpbuf[6]; // 1 for DS, 0 for SD
		vramAddr = ((tmpbuf[4]<<16) | (tmpbuf[3]<<8) | tmpbuf[2]);
		// readingSenseNum is greater or equal to itself, but a multiply of 4
		readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
		readingSenseNum *= 4;

		readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

		num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
		num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

		// read DCmap values from register
		// store it to read buffer memory for read action
		{ // L1
			/* read 120 bytes from Bank5, address 8 */
			/* read loop required */
			int bytes = readingSize;
			int size;
			int index = 0;

			memset(dcmapBuf, 0x0, sizeof(dcmapBuf));

			while (bytes > 0) {
				ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
				ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Middle)
				ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // address to read (Higher)
				WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);

				SetBankAddr(&g_ts,SHTSC_BANK_SYSTEM_HA);
				WriteOneByte(&g_ts, 0x43, ram_addr[2]);

				SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);

				size = ((bytes >= 120) ? 120 : bytes);
#if defined(DEBUG_SHTSC)
				TOUCH_LOG("bytes:%d, size:%d, index:%d, vramAddr:%x\n", bytes, size, index, vramAddr);
#endif

				ReadMultiBytes(&g_ts, 0x08, size, &(dcmapBuf[index]));
				index += size;
				bytes -= size;
				vramAddr += size;
			} // while
		} // L1
	}
#if defined(DEBUG_SHTSC)
	TOUCH_LOG("DCmap Drive x Sense: %d x %d, dsFlag:%02X, num_adc_dmy:%02X %02X %02X\n", numDriveLine2, numSenseLine2, dsFlag, num_adc_dmy[0], num_adc_dmy[1], num_adc_dmy[2]);
#endif /* 0 */

	{ //L3
		int sindex = 0;
		int dindex = 0;
		int l = 0;
		int x = 0;
		int y = 0;

		x = dcmap[dindex++] = numSenseLine2;
		y = dcmap[dindex++] = numDriveLine2;
		dcmap[dindex++] = dsFlag;
		dcmap[dindex++] = 0x00; // reserved

		//top
		sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

		// contents line
		for (l = 0; l < y; l++) {
			// left
			sindex += (num_adc_dmy[0] * 2);

			// contents
			memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(dcmapBuf[sindex]), (x*2));
			dindex += (x*2);
			sindex += (x*2);

			// right
			sindex += (num_adc_dmy[1] * 2);
		}
	}

	return ret;
}

int readDCRawdataValue(struct i2c_client *client, char *buf, int *pDataLen, int repeat_count, int *max_value)
{
	int ret = *pDataLen;
	int max = 0;
	int currentDC = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	u8 numDriveLine2 = 0;
	u8 numSenseLine2 = 0;
	u8 num_adc_dmy[3] = {0,};

	//TOUCH_FUNC();

	{ // L2
		unsigned char dsFlag, readingSenseNum, ram_addr[3];
		unsigned int vramAddr = 0;
		unsigned int readingSize = 0;
		u8 tmpbuf[16];

		// get SD/DS and size
		ram_addr[0] = 0x58;
		ram_addr[1] = 0x7F;
		ram_addr[2] = 0x01;
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, ram_addr[2]);

		SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);

		memset(tmpbuf, 0x0, sizeof(tmpbuf));
		ReadMultiBytes(&g_ts, 0x08, 7, tmpbuf); // less than 8 bytes

		numSenseLine2 = tmpbuf[0];
		numDriveLine2 = tmpbuf[1];

		dsFlag = tmpbuf[6]; // 1 for DS, 0 for SD
		vramAddr = ((tmpbuf[4]<<16) | (tmpbuf[3]<<8) | tmpbuf[2]);
		// readingSenseNum is greater or equal to itself, but a multiply of 4
		readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
		readingSenseNum *= 4;

		readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

		num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
		num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

		// read DCmap values from register
		// store it to read buffer memory for read action
		{ // L1
			/* read 120 bytes from Bank5, address 8 */
			/* read loop required */
			int bytes = readingSize;
			int size;
			int index = 0;

			memset(dcmapBuf, 0x0, sizeof(dcmapBuf));

			while (bytes > 0) {
				ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
				ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Middle)
				ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // address to read (Higher)
				WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
				WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
				WriteOneByte(&g_ts, 0x43, ram_addr[2]);
				WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_DCMAP);

				size = ((bytes >= 120) ? 120 : bytes);
#if defined(DEBUG_SHTSC)
				TOUCH_LOG("bytes:%d, size:%d, index:%d, vramAddr:%x\n", bytes, size, index, vramAddr);
#endif

				ReadMultiBytes(&g_ts, 0x08, size, &(dcmapBuf[index]));
				index += size;
				bytes -= size;
				vramAddr += size;
			} // while
		} // L1

#if defined(DEBUG_SHTSC)
		TOUCH_LOG("DCmap Drive x Sense: %d x %d, dsFlag:%02X, num_adc_dmy:%02X %02X %02X\n", numDriveLine2, numSenseLine2, dsFlag, num_adc_dmy[0], num_adc_dmy[1], num_adc_dmy[2]);
#endif /* 0 */

		{ //L3
			int sindex = 0, dindex = 0;
			int l, x, y, i, j;

			// dcmap header
			// [0]: horizontal data num (in short, not byte)
			// [1]: vertical data num

			//TOUCH_LOG(" Sense : %d Drive : %d\n", numSenseLine2, numDriveLine2);

			x = dcmap[dindex++] = numSenseLine2;
			y = dcmap[dindex++] = numDriveLine2;
			dcmap[dindex++] = dsFlag;
#if 0 // debug
			dcmap[dindex++] = frm_count++; // just for debug
#else /* 1 */
			dcmap[dindex++] = 0x00; // reserved
#endif /* 1 */

			//top
			sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

			// contents line
			for (l = 0; l < y; l++) {
				// left
				sindex += (num_adc_dmy[0] * 2);

				// contents
				memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(dcmapBuf[sindex]), (x*2));
				dindex += (x*2);
				sindex += (x*2);

				// right
				sindex += (num_adc_dmy[1] * 2);
			}

			for (j = 0; j < numDriveLine2; j++) { // TX line looping 31
				int tempBuf[20] = {0,};

				for (i = 0; i < numSenseLine2; i++) { // RX line looping 17
					currentDC = abs((short)((dcmap[4+ (numSenseLine2*j+i)*2 +1]<<8) | dcmap[4+ (numSenseLine2*j+i)*2]));
					tempBuf[i] = currentDC;
					if (max < currentDC) {
						//TOUCH_LOG("currentDC %d:,  max %d\n", currentDC, max);
						max = currentDC;
					}
				}


			}

			TOUCH_LOG("[%d] DC Rawdata Max %d\n", repeat_count, max);
			WRITE_RESULT(buf, ret, "[%d] DC Rawdata Max %d\n", repeat_count, max);

#if 1
			TOUCH_LOG("DC Rawdata reading \n");
			if(pDriverData != NULL && pDriverData->bootMode == BOOT_MFTS){
				WRITE_RESULT(buf, ret, "DC Rawdata reading \n");
			}

			for (j = 0; j < numDriveLine2; j++) { // TX line looping 31

				if(pDriverData != NULL && pDriverData->bootMode != BOOT_MFTS){

					TOUCH_LOG("[%2d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n", \
							j+1,\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  0) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  0) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  1) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  1) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  2) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  2) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  3) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  3) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  4) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  4) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  5) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  5) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  6) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  6) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  7) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  7) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  8) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  8) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  9) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  9) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 10) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 10) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 11) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 11) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 12) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 12) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 13) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 13) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 14) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 14) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 15) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 15) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 16) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 16) * 2)])));
				}
#if 0
				WRITE_RESULT(buf, ret, "[%2d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n", \
						j+1,\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  0) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  0) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  1) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  1) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  2) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  2) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  3) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  3) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  4) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  4) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  5) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  5) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  6) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  6) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  7) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  7) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  8) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  8) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  9) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  9) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 10) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 10) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 11) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 11) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 12) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 12) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 13) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 13) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 14) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 14) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 15) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 15) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 16) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 16) * 2)])));
#endif

			}

#endif

		} //L3
	} // L2 DEVICE_LR388K6 block

	*max_value = max;

	return ret;

}

static ssize_t check_jitter_spec_wo_irq(struct i2c_client *client, char *buf, int* pDataLen, int repeat, int *pJitterResult)
{
	int ret = *pDataLen;
	int count = 0;
	int max = 0;
	int repeat_count = 0;
	int jitterTotalMax = 0;
	int jitterMax = 0;
	u8 value = 0;

	TOUCH_FUNC();

	// Clear TOUCH_READY bit
	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);

	msleep(50); // need for changing mode

	issue_command_wo_IRQ(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);

	memset(s_last_dc_data, 0x0, sizeof(s_last_dc_data));
	memset(s_last_dc_max_diff, 0x0, sizeof(s_last_dc_max_diff));

	for( repeat_count = 0; repeat_count < repeat; repeat_count++){

		count = 0;

		// Clear TOUCH_READY bit
		if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
			TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
			Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
			SetIndicator(&g_ts, SHTSC_IND_TOUCH);
		}

		value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
		if( testingStart == 1){
			TOUCH_LOG("%s : before while-loop value 0x%X\n", __func__, value);
		}
#endif

		while (! (SHTSC_STATUS_DCMAP_READY & value )) {
			; // waiting the result
			value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
			if( testingStart == 1){
				TOUCH_LOG("DCMAP_READY waiting the result count %d value %d\n", count, value);
			}
#endif
			msleep(5);
			count++;

			if(count > 50){
				TOUCH_ERR("waiting the result count %d. Please check it\n", count);
				break;
			}
		}

		TOUCH_LOG("DCMAP_READY check waiting count %d value %d\n", count, value);

		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_DCMAP_READY);

		// Read DC Rawdata
		ret = readDCRawdataValue(client, buf, &ret, repeat_count, &max);

		// Check Jitter here
		ret = checkDCJitter(client, buf, &ret, repeat_count, repeat, &jitterMax);

		if( jitterTotalMax < jitterMax){
			jitterTotalMax = jitterMax;
		}

		if( repeat_count > 0){
			WRITE_RESULT(buf, ret, "[%d] Jitter Max : %d , TotalMax : %d\n", repeat_count, jitterMax, jitterTotalMax);
			TOUCH_LOG("[%d] Jitter Max : %d , TotalMax: %d\n", repeat_count, jitterMax, jitterTotalMax);
		}

		//read_dcmap_wo_irq(client);

		//ret = shtsc_CMD_getDCMap_Jitter(client, &g_ts, DC_RAW_DATA, JITTER_READ_REPEAT, buf, &dc_jitter_max);

		SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);
	}

	// Clear TOUCH_READY bit
	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	// DC MAP off
	issue_command2_wo_IRQ(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);

	count = 0;
	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(&g_ts, SHTSC_ADDR_INT0))) {
		; // waiting the result
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("DCMAP_OFF COMMAND_RESULT waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 50){
			TOUCH_ERR("waiting the result count %d. Please check it\n", count);
			break;
		}
	}
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	if (jitterTotalMax <  SPEC_DC_JITTER_MAX) {
		WRITE_RESULT(buf, ret, "Jitter test : OK ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		TOUCH_LOG("Jitter  : OK ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		*pJitterResult = TOUCH_SUCCESS;
	} else {
		WRITE_RESULT(buf, ret, "Jitter test : NG ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		TOUCH_LOG("Jitter test : NG ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		*pJitterResult = TOUCH_FAIL;
	}

	*pDataLen = ret;

	return ret;
}

static ssize_t check_jitter_spec(struct i2c_client *client, char *buf, int* pDataLen, int repeat, int *pJitterResult)
{
	int ret = *pDataLen;
	int count = 0;
	int repeat_count = 0;
	int jitterTotalMax = 0;
	int jitterMax = 0;
	int err = 0;

	TOUCH_FUNC();

	// Clear TOUCH_READY bit
	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	issue_command(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);

	msleep(50); // need for changing mode

	issue_command(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);

	memset(s_last_dc_data, 0x0, sizeof(s_last_dc_data));
	memset(s_last_dc_max_diff, 0x0, sizeof(s_last_dc_max_diff));

	for( repeat_count = 0; repeat_count < repeat; repeat_count++){

		count = 0;

		// Clear TOUCH_READY bit
		if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
			TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
			Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
			SetIndicator(&g_ts, SHTSC_IND_TOUCH);
		}

		// waiting for DCMAP ready
		(&g_ts)->wait_state = WAIT_DCMAP;
		(&g_ts)->wait_result = false;

		// wait for DCMAP ready
		err = WaitAsync(&g_ts);
		if( err != 0){
			//dump_stack();
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("WaitAsync in getDCMap return %d bankAddr %d\n",err, bankAddr);
#else
			TOUCH_LOG("WaitAsync in getDCMap return %d\n",err);
#endif
		}

		// Check Jitter here
		ret = checkDCJitter(client, buf, &ret, repeat_count, repeat, &jitterMax);

		if( jitterTotalMax < jitterMax){
			jitterTotalMax = jitterMax;
		}

		if( repeat_count > 0){
			WRITE_SYSBUF(buf, ret, "[%d] Jitter check : Max : %d , TotalMax : %d\n", repeat_count, jitterMax, jitterTotalMax);
			TOUCH_LOG("[%d] Jitter check :  Max : %d , TotalMax: %d\n", repeat_count, jitterMax, jitterTotalMax);
		}

		SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);
	}

	// Clear TOUCH_READY bit
	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	// DC MAP off
	issue_command2(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);

	count = 0;
	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(&g_ts, SHTSC_ADDR_INT0))) {
		; // waiting the result
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("DCMAP_OFF COMMAND_RESULT waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 50){
			TOUCH_ERR("waiting the result count %d. Please check it\n", count);
			break;
		}
	}
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	if (jitterTotalMax <  SPEC_DC_JITTER_MAX) {
		WRITE_SYSBUF(buf, ret, "Jitter test : OK ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		TOUCH_LOG("Jitter  : OK ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		*pJitterResult = TOUCH_SUCCESS;
	} else {
		WRITE_SYSBUF(buf, ret, "Jitter test : NG ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		TOUCH_LOG("Jitter test : NG ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, jitterTotalMax);
		*pJitterResult = TOUCH_FAIL;
	}

	*pDataLen = ret;

	return ret;
}

/* read and find maximum value. */
/* called by deltacheck in self-d  */
int checkDeltaValue(struct i2c_client *client, char *buf, int *pDataLen, int *max_value)
{
	int ret = *pDataLen;
	int max = 0;
	int currentDC = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	u8 numDriveLine2 = 0;
	u8 numSenseLine2 = 0;
	u8 num_adc_dmy[3] = {0,};

	//TOUCH_FUNC();

	{ // L2
		unsigned char dsFlag, readingSenseNum, ram_addr[3];
		unsigned int vramAddr = 0;
		unsigned int readingSize = 0;
		u8 tmpbuf[16];

		// get SD/DS and size
		ram_addr[0] = 0x58;
		ram_addr[1] = 0x7F;
		ram_addr[2] = 0x01;
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, ram_addr[2]);

		SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);

		memset(tmpbuf, 0x0, sizeof(tmpbuf));
		ReadMultiBytes(&g_ts, 0x08, 7, tmpbuf); // less than 8 bytes

		numSenseLine2 = tmpbuf[0];
		numDriveLine2 = tmpbuf[1];

		dsFlag = tmpbuf[6]; // 1 for DS, 0 for SD
		vramAddr = ((tmpbuf[4]<<16) | (tmpbuf[3]<<8) | tmpbuf[2]);
		// readingSenseNum is greater or equal to itself, but a multiply of 4
		readingSenseNum = (unsigned char)((numSenseLine2+3)/4); // not a double but int
		readingSenseNum *= 4;

		readingSize = readingSenseNum * numDriveLine2 * 2; /* 2:16bit */

		num_adc_dmy[0] = num_adc_dmy[2] = 0; //top and left have no 0-filled lines
		num_adc_dmy[1] = readingSenseNum - numSenseLine2; //right side

		// read DCmap values from register
		// store it to read buffer memory for read action
		{ // L1
			/* read 120 bytes from Bank5, address 8 */
			/* read loop required */
			int bytes = readingSize;
			int size;
			int index = 0;

			memset(dcmapBuf, 0x0, sizeof(dcmapBuf));

			while (bytes > 0) {
				ram_addr[0] = (unsigned char)(vramAddr&0xff); // address to read (Lower)
				ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // address to read (Middle)
				ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // address to read (Higher)
				WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
				WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
				WriteOneByte(&g_ts, 0x43, ram_addr[2]);
				WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_DCMAP);

				size = ((bytes >= 120) ? 120 : bytes);
#if defined(DEBUG_SHTSC)
				TOUCH_LOG("bytes:%d, size:%d, index:%d, vramAddr:%x\n", bytes, size, index, vramAddr);
#endif

				ReadMultiBytes(&g_ts, 0x08, size, &(dcmapBuf[index]));
				index += size;
				bytes -= size;
				vramAddr += size;
			} // while
		} // L1

#if defined(DEBUG_SHTSC)
		TOUCH_LOG("DCmap Drive x Sense: %d x %d, dsFlag:%02X, num_adc_dmy:%02X %02X %02X\n", numDriveLine2, numSenseLine2, dsFlag, num_adc_dmy[0], num_adc_dmy[1], num_adc_dmy[2]);
#endif /* 0 */

		{ //L3
			int sindex = 0, dindex = 0;
			int l, x, y, i, j;

			// dcmap header
			// [0]: horizontal data num (in short, not byte)
			// [1]: vertical data num

			//TOUCH_LOG(" Sense : %d Drive : %d\n", numSenseLine2, numDriveLine2);

			x = dcmap[dindex++] = numSenseLine2;
			y = dcmap[dindex++] = numDriveLine2;
			dcmap[dindex++] = dsFlag;
#if 0 // debug
			dcmap[dindex++] = frm_count++; // just for debug
#else /* 1 */
			dcmap[dindex++] = 0x00; // reserved
#endif /* 1 */

			//top
			sindex = (num_adc_dmy[0] + x + num_adc_dmy[1]) * num_adc_dmy[2] * 2;

			// contents line
			for (l = 0; l < y; l++) {
				// left
				sindex += (num_adc_dmy[0] * 2);

				// contents
				memcpy((u8 *)&(dcmap[dindex]), (u8 *)&(dcmapBuf[sindex]), (x*2));
				dindex += (x*2);
				sindex += (x*2);

				// right
				sindex += (num_adc_dmy[1] * 2);
			}

			for (j = 0; j < numDriveLine2; j++) { // TX line looping 31
				int tempBuf[20] = {0,};

				for (i = 0; i < numSenseLine2; i++) { // RX line looping 17
					currentDC = abs((short)((dcmap[4+ (numSenseLine2*j+i)*2 +1]<<8) | dcmap[4+ (numSenseLine2*j+i)*2]));
					tempBuf[i] = currentDC;
					if (max < currentDC) {
						//TOUCH_LOG("currentDC %d:,  max %d\n", currentDC, max);
						max = currentDC;
					}
				}


			}

			TOUCH_LOG("Delta reading \n");

			for (j = 0; j < numDriveLine2; j++) { // TX line looping 31

				if(pDriverData != NULL && pDriverData->bootMode != BOOT_MFTS){

					TOUCH_LOG("[%2d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n", \
							j+1,\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  0) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  0) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  1) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  1) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  2) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  2) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  3) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  3) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  4) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  4) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  5) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  5) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  6) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  6) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  7) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  7) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  8) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  8) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) +  9) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  9) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 10) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 10) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 11) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 11) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 12) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 12) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 13) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 13) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 14) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 14) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 15) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 15) * 2)])),\
							((short)((dcmap[4 + (((numSenseLine2 * j) + 16) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 16) * 2)])));
				}
				WRITE_RESULT(buf, ret, "[%2d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n", \
						j+1,\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  0) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  0) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  1) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  1) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  2) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  2) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  3) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  3) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  4) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  4) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  5) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  5) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  6) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  6) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  7) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  7) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  8) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  8) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) +  9) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) +  9) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 10) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 10) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 11) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 11) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 12) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 12) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 13) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 13) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 14) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 14) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 15) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 15) * 2)])),\
						((short)((dcmap[4 + (((numSenseLine2 * j) + 16) * 2) + 1] << 8) | dcmap[4+ (((numSenseLine2 * j) + 16) * 2)])));

			}

			TOUCH_LOG("Delta Max %d\n", max);

		} //L3
	} // L2 DEVICE_LR388K6 block

	*max_value = max;

	return ret;

}


static ssize_t check_delta_spec(struct i2c_client *client, char *buf, int* pDataLen, int spec, int repeat, int *result, int* maxValue)
{

	int ret = *pDataLen;
	int count = 0;
	int repeat_count = 0;
	int max = 0;
	int totalMax = 0;
	u8 value = 0;

	TOUCH_FUNC();

#if defined(DEBUG_SHTSC)
	testingStart = 1;
#endif
	*result = TOUCH_SUCCESS;


	// Clear TOUCH_READY bit
	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	// DC MAP on
	issue_command_wo_IRQ(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);

	// delta check here
	for( repeat_count = 0; repeat_count < repeat; repeat_count++){

		count = 0;

		// Clear TOUCH_READY bit
		if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
			TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
			Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
			SetIndicator(&g_ts, SHTSC_IND_TOUCH);
		}

		value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
		if( testingStart == 1){
			TOUCH_LOG("%s : before while-loop value 0x%X\n", __func__, value);
		}
#endif

		while (! (SHTSC_STATUS_DCMAP_READY & value )) {
			; // waiting the result
			value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
			if( testingStart == 1){
				TOUCH_LOG("DCMAP_READY waiting the result count %d value %d\n", count, value);
			}
#endif
			msleep(5);
			count++;

			if(count > 50){
				TOUCH_ERR("waiting the result count %d. Please check it\n", count);
				break;
			}
		}
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_DCMAP_READY);


		// check count
		ret = checkDeltaValue(client, buf, &ret, &max);

		WRITE_RESULT(buf, ret,"[%d] : Delta Max %d \n",repeat_count+1, max);

		if( max > totalMax ){
			totalMax = max;
		}
		if( max > spec ){
			*result = TOUCH_FAIL;
			//break; HMOON TEMP
		}
		SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);
	}

	// Clear TOUCH_READY bit
	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	// DC MAP off
	issue_command2_wo_IRQ(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);
	//SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);
	count = 0;
	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(&g_ts, SHTSC_ADDR_INT0))) {
		; // waiting the result
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("DCMAP_OFF COMMAND_RESULT waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 50){
			TOUCH_ERR("waiting the result count %d. Please check it\n", count);
			break;
		}
	}
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);


	if( totalMax < spec ) {
		WRITE_RESULT(buf, ret,"Delta result : OK (Spec : %d  MAX : %d)\n", spec, totalMax);
	} else {
		WRITE_RESULT(buf, ret,"Delta result : NG (Spec : %d  MAX : %d)\n", spec, totalMax);
	}

#if defined(DEBUG_SHTSC)
	testingStart = 0;
#endif

	*maxValue = totalMax;
	*pDataLen = ret;
	return ret;
}

static ssize_t show_device_name(struct i2c_client *client, char *buf)
{
	int ret = 0;
	unsigned char unique_id_Buf[UNIQUE_ID_BUFFER_MAX];

	TOUCH_FUNC();

	issue_command(&g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	K6_firmver = 0;
	K6_paramver = 0;

	K6_firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	K6_paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

	WRITE_SYSBUF(buf, ret,	"Sharp LR388K6 Firmware=%08X, Parameter=%08X\n", K6_firmver, K6_paramver);

	memset(unique_id_Buf, 0x0, sizeof(unique_id_Buf));

	if(get_unique_id_from_cache(client, unique_id_Buf, sizeof(unique_id_Buf)) == TOUCH_SUCCESS){
		TOUCH_LOG("Unique ID : %s\n", unique_id_Buf);
		WRITE_SYSBUF(buf, ret, "Unique ID : %s\n", unique_id_Buf);
	} else {
		TOUCH_LOG("Unique ID : Unknown\n");
		WRITE_SYSBUF(buf, ret, "Unique ID : Unknown\n");
	}

	return ret;
}

static ssize_t show_factory_cal_check(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int testResult = TOUCH_FAIL;
	int diffMax = 0;
	TOUCH_FUNC();
	ret = shtsc_proc_checkcalib_func(client, buf, ret, &testResult, &diffMax);
	if( testResult == TOUCH_SUCCESS ){
		WRITE_SYSBUF(buf, ret, "Factory calibration check : SUCCESS (Max : %d)\n", diffMax);
	} else {
		WRITE_SYSBUF(buf, ret, "Factory calibration check : FAIL (Max : %d) \n", diffMax);
	}
	return ret;
}

static ssize_t show_factory_cal_read(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int testResult = TOUCH_FAIL;
	TOUCH_FUNC();
	ret = shtsc_proc_readcalib_func(client, buf, ret, &testResult);
	if( testResult == TOUCH_SUCCESS ){
		WRITE_SYSBUF(buf, ret, "Factory calibration read : SUCCESS\n");
	} else {
		WRITE_SYSBUF(buf, ret, "Factory calibration read : FAIL\n");
	}
	return ret;
}

static ssize_t show_factory_cal_update(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int testResult = TOUCH_FAIL;
	TOUCH_FUNC();
	ret = shtsc_proc_updatecalib_func(client, buf, ret, &testResult);
	if( testResult == TOUCH_SUCCESS ){
		WRITE_SYSBUF(buf, ret, "Factory calibration update : SUCCESS\n");
	} else {
		WRITE_SYSBUF(buf, ret, "Factory calibration update : FAIL\n");
	}
	return ret;
}

static ssize_t show_property(struct i2c_client *client, char *buf)
{
	int ret = 0;
	unsigned char unique_id_Buf[UNIQUE_ID_BUFFER_MAX];

	TOUCH_FUNC();

	issue_command(&g_ts, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	K6_firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	K6_paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

	WRITE_SYSBUF(buf, ret,	"Firmware %08X, Parameter %08X\n", K6_firmver, K6_paramver);

	TOUCH_LOG("Firmware %08X, Parameter %08X\n", K6_firmver, K6_paramver);

	memset( unique_id_Buf, 0x0, sizeof(unique_id_Buf));

	if( get_unique_id_from_cache(client, unique_id_Buf, sizeof(unique_id_Buf)) == TOUCH_SUCCESS ){
		TOUCH_LOG("Unique ID : %s\n", unique_id_Buf);
		WRITE_SYSBUF(buf, ret, "Unique ID : %s\n", unique_id_Buf);
	} else {
		TOUCH_LOG("Unique ID : Unknown\n");
		WRITE_SYSBUF(buf, ret, "Unique ID : Unknown\n");
		return TOUCH_FAIL;
	}

	return ret;
}

// ONE SHORT CALIBRATION
//#define CMD_EXECCALIBRATION "\x0F\x00\x03\x00\x00\x50\0x00"
// 800 ms ALIBRATION
#define CMD_EXECCALIBRATION "\x0F\x00\x03\x00\x05\x20\0x03"
#define CMD_EXECCALIBRATION_LEN 7

int shtsc_CMD_ExecCalibration_wo_IRQ(void *ts)
{
	//int ret;
	int count = 0;
	u8 value = 0;

	//TOUCH_FUNC();

	// should be in idle or active mode

	SetBankAddr(ts, SHTSC_BANK_COMMAND);
#ifdef DEBUG_SHTSC
	TOUCH_LOG("exec calibration command\n");
#endif

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_EXECCALIBRATION, CMD_EXECCALIBRATION_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	//msleep(400);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 100){
			TOUCH_LOG("waiting the result count %d. Please check it\n", count);
			break;
		}
	}

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	TOUCH_ERR("CALIBRATION done\n");

	return 0;
}

static ssize_t show_calibration(struct i2c_client *client, char *buf)
{
	int ret = 0;

	struct timeval start;
	struct timeval finish;

	long elapsed_time = 0;

	TOUCH_FUNC();

	do_gettimeofday(&start);

	shtsc_CMD_ExecCalibration_wo_IRQ(&g_ts);

	do_gettimeofday(&finish);

	if(finish.tv_sec == start.tv_sec) {
		elapsed_time = finish.tv_usec - start.tv_usec;
	} else if ( finish.tv_sec > start.tv_sec) {
		elapsed_time = (finish.tv_usec + 1000000 ) - start.tv_usec;
	} else {
		TOUCH_ERR("Time wrong. check!\n");
	}

	WRITE_SYSBUF(buf, ret,	"Calbiration done duration  %ld\n", elapsed_time);
	TOUCH_LOG("Calbiration done elapsed time  %ld\n", elapsed_time);

	return ret;
}


/* get DC map */
//#define DC_CALIBED_DATA 0
//#define DC_RAW_DATA 1

static int readDcMapResult(char *buf, int *pDataLen)
{
	/*
	 *  (0,0)L,H (1,0)L,H ....(16,0)L,H = 34byte
	 *  (0,1)L,H (1,1)L,H ....(16,1)L,H = 34byte
	 *  (0,2)L,H (1,2)L,H ....(16,2)L,H = 34byte
	 *  :
	 *  (0,30)L,H (1,30)L,H ....(16,30)L,H = 34byte
	 *  singed Little endian
	 */

	// total S(18-1) * D 31 * 2byte = 1054 byte

	u8* pDcDataBuffer = &(dcmap[4]);
	int drive =0;
	int ret = *pDataLen;

	TOUCH_FUNC();

	//msleep(50);

	// for DCmap, sense(18)-1=17
	if(debug_app_enable == 0){

		TOUCH_LOG("    [%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);

	}

	WRITE_SYSBUF(buf, ret, "    [%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);

	for (drive = 0; drive < NUM_DRIVE; drive++) {

		if(debug_app_enable == 0){

			TOUCH_LOG("[%02d]%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
					(drive + 1),
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
					(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));
		}

		WRITE_SYSBUF(buf, ret, "[%02d]%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
				(drive + 1),
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));
	}

	*pDataLen = ret;

	TOUCH_LOG("readDcMapResult done ret = %d\n", ret );

	return ret;
}

static int checkLpwgRawdataJitter(struct i2c_client *client, char *buf, int *pDataLen, int*pRawdataMax, int*pJitterMax)
{

	u8* pDcDataBuffer = &(dcmap[4]);
	int drive =0;
	int sense = 0;
	int ret = *pDataLen;
	int rawdataMax = 0;
	int jitterMax = 0;


	TOUCH_FUNC();

	// for DCmap, sense(18)-1=17

	if(debug_app_enable == 0){

		TOUCH_LOG("[%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);
		TOUCH_LOG("%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));
	}


	WRITE_RESULT(buf, ret, "[%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);
	WRITE_RESULT(buf, ret, "%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
			(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));


	for( sense = 1 ; sense < 16 ; sense++){

		int lpwg_rawdata = (signed short)((pDcDataBuffer[sense * 2 + 1]<<8) | pDcDataBuffer[sense * 2]);

		int tmp_diff = 0;
		unsigned int abs_diff = 0;;


		// Compare with the previous frame
		if( s_last_lpwg_data[sense] != 0) {

			tmp_diff = s_last_lpwg_data[sense] - lpwg_rawdata;
			abs_diff = abs(tmp_diff);

#ifdef DEBUG_SHTSC
			TOUCH_LOG("[%d] Previous Max %d , Current Diff %d \n", sense, s_last_lpwg_max_diff[sense], abs_diff);
#endif

			if (s_last_lpwg_max_diff[sense] < abs_diff) {
				s_last_lpwg_max_diff[sense] = abs_diff;
			}
		}

		s_last_lpwg_data[sense] = lpwg_rawdata;


		lpwg_rawdata = abs(lpwg_rawdata);

		if( rawdataMax < lpwg_rawdata){
			rawdataMax = lpwg_rawdata;
		}
	}

	for(sense = 0; sense < 17 ;sense++){

		if( jitterMax <  s_last_lpwg_max_diff[sense]){
			jitterMax = s_last_lpwg_max_diff[sense];
		}
	}

#ifdef DEBUG_SHTSC
	TOUCH_LOG("LPWG Rawdata Max = %d, Jitter Max = %d\n", num, rawdataMax, jitterMax);
#endif

	*pJitterMax = jitterMax;
	*pRawdataMax = rawdataMax;
	*pDataLen = ret;

	return ret;
}

static int checkDCJitter(struct i2c_client *client, char *buf, int *pDataLen, int count, int repeat, int*pJitterMax)
{
	TouchDriverData *pDriverData = NULL;
	u8* pDcDataBuffer = &(dcmap[4]);
	int drive =0;
	int sense = 0;
	int ret = *pDataLen;
	int jitterMax = 0;
	int isSameRawdata = 1;

	pDriverData = i2c_get_clientdata(client);

	//TOUCH_FUNC();

#if 0
	TOUCH_LOG("Rawdata DC Data (By-pass) data \n");

	for (drive = 0; drive < NUM_DRIVE; drive++) {

		TOUCH_LOG("[%02d]%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
				(drive + 1),
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));
	}
#endif

	//readDcMapResult(buf, pDataLen);

	for( drive = 0 ; drive < 31 ; drive++){
		for( sense = 0 ; sense < 17 ; sense++){

			int tmp_diff = 0;
			unsigned int abs_diff = 0;
			int dc_rawdata = (signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + sense*2+1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + sense*2]);

			// Compare with the previous frame
			if( count > 0 ) {
				tmp_diff = s_last_dc_data[drive][sense] - dc_rawdata;
				abs_diff = abs(tmp_diff);

				if(tmp_diff != 0){
					isSameRawdata = 0;
				}
#ifdef DEBUG_SHTSC
				TOUCH_LOG("[%d][%d] Previous Max %d , Current Diff %d \n", drive, sense, s_last_dc_max_diff[drive][sense], abs_diff);
#endif

				if (s_last_dc_max_diff[drive][sense] < abs_diff) {
					s_last_dc_max_diff[drive][sense] = abs_diff;
				}
			} else {
				isSameRawdata = 0;
			}
			s_last_dc_data[drive][sense] = dc_rawdata;

			dc_rawdata = abs(dc_rawdata);
		}
	}
	if( isSameRawdata == 1){
		TOUCH_LOG("Check DC Rawdata is same!\n");
	}

	//WRITE_SYSBUF( buf, ret, "DC Jitter Test\n");
	if(  count > 0 ) {
		for( drive = 0 ; drive < 31 ; drive++){
			for(sense = 0; sense < 17 ;sense++){
				if( jitterMax <  s_last_dc_max_diff[drive][sense]){
					jitterMax = s_last_dc_max_diff[drive][sense];
				}
			}
		}
		TOUCH_LOG("[%d] DC Jitter Max = %d\n", count, jitterMax);
	}

	if( ((count + 1) == repeat) ) {
		for( drive = 0 ; drive < 31 ; drive++){
			if( pDriverData != NULL && pDriverData->bootMode != BOOT_MFTS ) {
				TOUCH_LOG("[%02d]%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d\n",\
						(drive + 1),
						(signed short)( s_last_dc_max_diff[drive][0]),\
						(signed short)( s_last_dc_max_diff[drive][1]),\
						(signed short)( s_last_dc_max_diff[drive][2]),\
						(signed short)( s_last_dc_max_diff[drive][3]),\
						(signed short)( s_last_dc_max_diff[drive][4]),\
						(signed short)( s_last_dc_max_diff[drive][5]),\
						(signed short)( s_last_dc_max_diff[drive][6]),\
						(signed short)( s_last_dc_max_diff[drive][7]),\
						(signed short)( s_last_dc_max_diff[drive][8]),\
						(signed short)( s_last_dc_max_diff[drive][9]),\
						(signed short)( s_last_dc_max_diff[drive][10]),\
						(signed short)( s_last_dc_max_diff[drive][11]),\
						(signed short)( s_last_dc_max_diff[drive][12]),\
						(signed short)( s_last_dc_max_diff[drive][13]),\
						(signed short)( s_last_dc_max_diff[drive][14]),\
						(signed short)( s_last_dc_max_diff[drive][15]),\
						(signed short)( s_last_dc_max_diff[drive][16]));
			}
			WRITE_RESULT(buf, ret, "[%02d]%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d\n",\
					(drive + 1),
					(signed short)( s_last_dc_max_diff[drive][0]),\
					(signed short)( s_last_dc_max_diff[drive][1]),\
					(signed short)( s_last_dc_max_diff[drive][2]),\
					(signed short)( s_last_dc_max_diff[drive][3]),\
					(signed short)( s_last_dc_max_diff[drive][4]),\
					(signed short)( s_last_dc_max_diff[drive][5]),\
					(signed short)( s_last_dc_max_diff[drive][6]),\
					(signed short)( s_last_dc_max_diff[drive][7]),\
					(signed short)( s_last_dc_max_diff[drive][8]),\
					(signed short)( s_last_dc_max_diff[drive][9]),\
					(signed short)( s_last_dc_max_diff[drive][10]),\
					(signed short)( s_last_dc_max_diff[drive][11]),\
					(signed short)( s_last_dc_max_diff[drive][12]),\
					(signed short)( s_last_dc_max_diff[drive][13]),\
					(signed short)( s_last_dc_max_diff[drive][14]),\
					(signed short)( s_last_dc_max_diff[drive][15]),\
					(signed short)( s_last_dc_max_diff[drive][16]));
		}
	}

	*pJitterMax = jitterMax;

	return ret;
}

int shtsc_CMD_getDCMap_wo_IRQ(void *ts, int rawflag, char* buf)
{
	int resultDataLen = 0;
	int err = 0;

	//TOUCH_FUNC();

	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	//TEMP 1116 SetBankAddr(ts, 0x12);
	if (rawflag == DC_CALIBED_DATA) {
		// normal DC data (delta)
		// bank=0x12, addr=0x6e, data=0x01
		issue_command(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
		TOUCH_LOG("DC Delta (calibrated) \n");
		WRITE_SYSBUF(buf, resultDataLen, "=== DC Delta ===\n");
	} else {
		// calib by-pass data (rawdata)
		// bank=0x12, addr=0x6e, data=0x21
		issue_command(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);
		TOUCH_LOG("DC Rawdata (by-pass) \n");
		WRITE_SYSBUF(buf, resultDataLen, "=== DC Rawdata ====\n");
	}
	msleep(50); // need for changing mode


#ifdef DEBUG_SHTSC
	TOUCH_LOG("exec DCMAP ON command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command
	issue_command(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);

	// waiting for DCMAP ready
	(&g_ts)->wait_state = WAIT_DCMAP;
	(&g_ts)->wait_result = false;

	// wait for DCMAP ready
	err = WaitAsync(&g_ts);
	if( err != 0){
		//dump_stack();
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("WaitAsync in getDCMap return %d bankAddr %d\n",err, bankAddr);
#else
		TOUCH_LOG("WaitAsync in getDCMap return %d\n",err);
#endif
	}
#if 0
	TOUCH_LOG("WaitAsync for DCMAP-ready is done");
#endif

	/* read DC map */
	readDcMapResult(buf, &resultDataLen);


#ifdef DEBUG_SHTSC
	TOUCH_LOG("exec DCMAP OFF command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command2. waitAsync is in later
	issue_command2(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);

	/* dc map read done */
	/* this must  be done first before COMMAND_RESULT interrupt*/
	SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);

	err = 0;
	err = WaitAsync(&g_ts); // for CMD_DCMAP_OFF in issue command2
	if( err != 0){
		//dump_stack();
		TOUCH_LOG("WaitAsync in getDCMap  return %d\n",err);
	}

	issue_command(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);

	//TOUCH_LOG("back to normal DC (CALIB-ed = delta) mode \n");
	msleep(50); // need for changing mode

	return resultDataLen;
}

int shtsc_CMD_getDCMap_LPWG_Rawdata_Jitter(struct i2c_client *client, void *ts, int rawflag, int frame_num, char* pBuf, int dataLen, int* pRawdataMax, int* pJitterMax)
{
	int resultDataLen = dataLen;
	int err = 0;
	int i = 0;
	int rawdataMax = 0;
	int jitterMax = 0;
	int rawdataTotalMax = 0;
	int jitterTotalMax = 0;
	u8 tmp0 = 0;
	u8 tmp1 = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	for (i = 0; i < 50; i++) {
		issue_command_wo_IRQ(&g_ts, CMD_GETSYSTEMSTATE, CMD_GETSYSTEMSTATE_LEN);

#if 1//for only debugging
		mutex_lock(&g_ts.mutex_irq);

		SetBankAddr(&g_ts, 0x18);
		WriteOneByte(&g_ts, D_REG_HOST_AP_FULL, 0x01);
		SetBankAddr(&g_ts, 0x12);
		tmp0 = ReadOneByte(&g_ts, D_REG_IDLE_BREAK_THRESH_DC0);
		tmp1 = ReadOneByte(&g_ts, D_REG_IDLE_BREAK_THRESH_DC1);
		SetBankAddr(&g_ts, 0);

		mutex_unlock(&g_ts.mutex_irq);
		TOUCH_LOG("D_REG_IDLE_BREAK_THRESH_DC 0x%x 0x%x\n", tmp0, tmp1);
#endif

#if 1
		TOUCH_LOG("Command result (GetSystemState), Op:%02X, Err:%02X State:%02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2]);
#endif
		msleep(20);
		if (CommandResultBuf[2] == 0x4) {
			break;
		}
	}
	if (i) {
		TOUCH_LOG("Commad resut timeout  error(%d)\n", i);
	}

	/* calibraion(calfactor=0,time=300ms) */
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		if(issue_command_wo_IRQ(&g_ts, CMD_EXECCALIBRATION_LPWG_WRITE, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE failed\n");
			err = TOUCH_FAIL;
		}
	} else {
		if(issue_command(&g_ts, CMD_EXECCALIBRATION_LPWG_WRITE, CMD_EXECCALIBRATION_FOR_WRITE_LEN)) {
			TOUCH_LOG("CMD_EXECCALIBRAION_FOR_WRITE failed\n");
			err = TOUCH_FAIL;
		}
	}
	msleep(300); // for waiting to finish calibration.

#ifdef DEBUG_SHTSC
	TOUCH_LOG("exec LPWG DCMAP ON command for LPWG (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif
	memset(s_last_lpwg_max_diff, 0x0, sizeof(s_last_lpwg_max_diff));;
	memset(s_last_lpwg_data, 0x0, sizeof(s_last_lpwg_data));

	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		issue_command_wo_IRQ(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);
	} else {
		issue_command(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);
	}

	for (i = 0; i < frame_num; i++) {

		// update dcmap manually
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			// delta check here
			int count = 0;
			u8 value = 0;

			// Clear TOUCH_READY bit
			if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
				TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
				Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
				SetIndicator(&g_ts, SHTSC_IND_TOUCH);
			}

			value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
			if( testingStart == 1){
				TOUCH_LOG("%s : before while-loop value 0x%X\n", __func__, value);
			}
#endif
			while (! (SHTSC_STATUS_DCMAP_READY & value )) {
				// waiting the result
				value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
				if( testingStart == 1){
					TOUCH_LOG("LPWG DCMAP_READY waiting the result count %d value %d\n", count, value);
				}
#endif
				msleep(5);
				count++;

				if(count > 50){
					TOUCH_ERR("waiting the result count %d. Please check it\n", count);
					break;
				}
			}
			Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_DCMAP_READY);
			/* read dcmap polling */
			read_dcmap_wo_irq(client);

		} else {
			(&g_ts)->wait_state = WAIT_DCMAP;
			(&g_ts)->wait_result = false;

			// wait for DCMAP ready
			err = WaitAsync(&g_ts);
			if( err != 0){
				//dump_stack();
#if defined(DEBUG_SHTSC)
				TOUCH_LOG("WaitAsync in getDCMap return %d bankAddr %d\n",err, bankAddr);
#else
				TOUCH_ERR("WaitAsync in getDCMap return %d\n",err);
#endif
			}
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("WaitAsync for LPWG DCMAP-ready is done");
#endif
		}

		checkLpwgRawdataJitter(client, pBuf, &resultDataLen, &rawdataMax, &jitterMax);

		WRITE_RESULT(pBuf, resultDataLen, "[%d] LPWG Rawdata Max = %d, Jitter Max = %d\n", i, rawdataMax, jitterMax);
		TOUCH_LOG("[%d] LPWG Rawdata Max = %d, Jitter Max = %d\n", i, rawdataMax, jitterMax);

		if(rawdataMax > rawdataTotalMax){
			rawdataTotalMax = rawdataMax;
		}
		if(jitterMax > jitterTotalMax){
			jitterTotalMax = jitterMax;
		}

		/* dc map read done */
		/* this must  be done first before COMMAND_RESULT interrupt*/
		SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);
	}

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("exec LPWG DCMAP OFF command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif
	// set command2. waitAsync is in later
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		int count = 0;

		issue_command2_wo_IRQ(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);


		count = 0;
		while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(&g_ts, SHTSC_ADDR_INT0))) {
			; // waiting the result
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("LPWG DCMAP_OFF COMMAND_RESULT waiting the result count %d\n", count);
#endif
			msleep(5);
			count++;

			if(count > 50){
				TOUCH_ERR("waiting the result count %d. Please check it\n", count);
				break;
			}
		}
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);
	} else {
		issue_command2(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);
		/* dc map read done */
		/* this must  be done first before COMMAND_RESULT interrupt*/
		SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);

		err = 0;
		err = WaitAsync(&g_ts); // for CMD_DCMAP_OFF in issue command2
		if( err != 0){
			//dump_stack();
			TOUCH_LOG("WaitAsync in getDCMap  return %d\n",err);
		}
	}

	//TOUCH_LOG("back to normal DC (CALIB-ed = delta) mode \n");
	//msleep(50); // need for changing mode


	//TOUCH_LOG("LPWG Rawdata Max = %d, Jitter Max = %d\n", rawdataTotalMax, jitterTotalMax);

	*pJitterMax = jitterTotalMax;
	*pRawdataMax = rawdataTotalMax;

	if( pDriverData->mfts_lpwg == 1 ){
		TOUCH_LOG("LPWG Testing restore Threshold test_flag : %d\n", lgtp_set_deepidle_test_flag );
		if (lgtp_set_deepidle_test_flag == 1) {
			//lgtp_restore_deepidle_val();
			unsigned char cmd[9] = {0x06, 0x00, 0x05, 0x00, 0x09, 0x12, 0x72, 0x80, 0x01};

			cmd[7] = lgtp_old_idle_brk_thsh_d0;
			cmd[8] = lgtp_old_idle_brk_thsh_d1;

			TOUCH_LOG("lgtp_old_idle_brk_thsh_d 0x%02x, 0x%02x\n", lgtp_old_idle_brk_thsh_d0, lgtp_old_idle_brk_thsh_d1);

			issue_command_wo_IRQ(&g_ts, cmd, CMD_SETREGTBL_IDLE_BRK_TRSH_LEN);

			lgtp_set_deepidle_test_flag = 0;
		}
	}

	TOUCH_LOG("shtsc_CMD_getDCMap_LPWG_Rawdata_Jitter done ret = %d\n", resultDataLen);

	return resultDataLen;
}

int shtsc_CMD_getDCMap_Jitter(struct i2c_client *client, void *ts, int rawflag, int frame_num, char* buf, int* pJitterMax)
{
	int resultDataLen = 0;
	int err = 0;
	int repeat_count = 0;
	int jitterMax = 0;
	int jitterTotalMax = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	//TOUCH_FUNC();
#ifdef DEBUG_SHTSC
	TOUCH_LOG("exec DCMAP ON command(%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	if (rawflag == DC_CALIBED_DATA) {
		// normal DC data (delta)
		// bank=0x12, addr=0x6e, data=0x01
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
		} else {
			issue_command(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
		}
		TOUCH_LOG("DC Delta (calibrated) \n");
		WRITE_SYSBUF(buf, resultDataLen, "=== DC Delta ===\n");
	} else {
		// calib by-pass data (rawdata)
		// bank=0x12, addr=0x6e, data=0x21
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);
		} else {
			issue_command(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);
		}
		TOUCH_LOG("DC Rawdata (by-pass) \n");
		WRITE_SYSBUF(buf, resultDataLen, "=== DC Rawdata ====\n");
	}

	msleep(50); // need for changing mode

	memset(s_last_dc_data, 0x0, sizeof(s_last_dc_data));
	memset(s_last_dc_max_diff, 0x0, sizeof(s_last_dc_max_diff));

	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		issue_command_wo_IRQ(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);
	} else {
		issue_command(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);
	}

	for (repeat_count  = 0; repeat_count < frame_num; repeat_count++) {
		// update dcmap manually
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			// delta check here
			int count = 0;
			u8 value = 0;

			// Clear TOUCH_READY bit
			if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
				TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
				Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
				SetIndicator(&g_ts, SHTSC_IND_TOUCH);
			}

			value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
			if( testingStart == 1){
				TOUCH_LOG("%s : before while-loop value 0x%X\n", __func__, value);
			}
#endif
			while (! (SHTSC_STATUS_DCMAP_READY & value )) {
				// waiting the result
				value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
				if( testingStart == 1){
					TOUCH_LOG("DCMAP_READY waiting the result count %d value %d\n", count, value);
				}
#endif
				msleep(5);
				count++;

				if(count > 50){
					TOUCH_ERR("waiting the result count %d. Please check it\n", count);
					break;
				}
			}
			Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_DCMAP_READY);
			/* read dcmap polling */
			read_dcmap_wo_irq(client);
		} else {
			// update dcmap manually
			(&g_ts)->wait_state = WAIT_DCMAP;
			(&g_ts)->wait_result = false;

			// wait for DCMAP ready
			err = WaitAsync(&g_ts);
			if( err != 0){
				//dump_stack();
#if defined(DEBUG_SHTSC)
				TOUCH_LOG("WaitAsync in getDCMap return %d bankAddr %d\n",err, bankAddr);
#else
				TOUCH_ERR("WaitAsync in getDCMap return %d\n",err);
#endif
			}
#if 1 //defined(DEBUG_SHTSC)
			TOUCH_LOG("WaitAsync for DCMAP-ready is done");
#endif
		}

		resultDataLen = checkDCJitter(client, buf, &resultDataLen, repeat_count, frame_num, &jitterMax);

#if 1 //def DEBUG_SHTSC
		TOUCH_LOG("[%d] DC Jitter Max = %d\n", frame_num, jitterMax);
#endif
		if(jitterMax > jitterTotalMax){
			jitterTotalMax = jitterMax;
		}

		/* dc map read done */
		/* this must  be done first before COMMAND_RESULT interrupt*/
		SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);
	}
#if defined(DEBUG_SHTSC)
	TOUCH_LOG("exec DCMAP OFF command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif
	// set command2. waitAsync is in later
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		issue_command2_wo_IRQ(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);
	} else {
		issue_command2(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);
	}
	/* dc map read done */
	/* this must  be done first before COMMAND_RESULT interrupt*/
	SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);

	err = 0;
	err = WaitAsync(&g_ts); // for CMD_DCMAP_OFF in issue command2
	if( err != 0){
		//dump_stack();
		TOUCH_LOG("WaitAsync in getDCMap  return %d\n",err);
	}

	//TOUCH_LOG("back to normal DC (CALIB-ed = delta) mode \n");
	//msleep(50); // need for changing mode

	TOUCH_LOG("Dc Jitter Max = %d\n", jitterTotalMax);

	*pJitterMax = jitterTotalMax;

	TOUCH_LOG("shtsc_CMD_getDCMap_Jitter done ret = %d\n", resultDataLen);

	return resultDataLen;
}


static int readDcMapResultLPWG(struct i2c_client *client, char *buf, int *pDataLen, int* pDeltaMax)
{

	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	u8* pDcDataBuffer = &(dcmap[4]);
	int drive =0;
	int sense = 0;
	int max = 0;
	int ret = *pDataLen;

	TOUCH_FUNC();

	// for DCmap, sense(18)-1=17

	if(debug_app_enable == 0){

		TOUCH_LOG("[%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);
		TOUCH_LOG("%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));
	}

	if( pDriverData != NULL && pDriverData->bootMode == BOOT_NORMAL){

		WRITE_SYSBUF(buf, ret, "[%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d][%5d]\n", 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);
		WRITE_SYSBUF(buf, ret, "%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d%7d\n",\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 33]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 32]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 31]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 30]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 29]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 28]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 27]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 26]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 25]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 24]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 23]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 22]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 21]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 20]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 19]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 18]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 17]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 16]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 15]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 14]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 13]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 12]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 11]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 10]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 9]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 8]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 7]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 6]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 5]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 4]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 3]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 2]),\
				(signed short)((pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 1]<<8) | pDcDataBuffer[(NUM_SENSE-1)*drive*2 + 0]));
	}

	for( sense = (16 - 1) ; sense >= (0 + 1) ; sense--){
		int lpwg_delta = (signed short)((pDcDataBuffer[sense * 2 + 1]<<8) | pDcDataBuffer[sense * 2]);
		lpwg_delta = abs(lpwg_delta);

#if 0
		if (lpwg_delta > Lpwg_spec_upper[sense]) { // check each sense line
			TOUCH_LOG("LPWG Delta spec out! Sense : %d, Max : %d \n", sense, max);
		}
#endif

		// one line blocking
		if( lpwg_delta > max ){
			max = lpwg_delta;
		}
	}

	TOUCH_LOG("LPWG Delta Max : %d \n", max);

	*pDataLen = ret;
	*pDeltaMax = max;

	TOUCH_LOG("readDcMapResultLPWG done ret = %d\n", ret );

	return ret;
}

int shtsc_CMD_getDCMap_LPWG_wo_IRQ(struct i2c_client *client, void *ts, int rawflag, char* buf, int dataLen, int* pDeltaMax)
{
	int resultDataLen = dataLen;
	int err = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	//TOUCH_FUNC();

	if (SHTSC_STATUS_TOUCH_READY & ReadOneByte(&g_ts, SHTSC_ADDR_INT0)) {
		TOUCH_LOG("Bit clear SHTSC_STATUS_TOUCH_READY\n");
		Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_TOUCH_READY);
		SetIndicator(&g_ts, SHTSC_IND_TOUCH);
	}

	//TEMP 1116 SetBankAddr(ts, 0x12);
	if (rawflag == DC_CALIBED_DATA) {
		// normal DC data (delta)
		// bank=0x12, addr=0x6e, data=0x01
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
		} else {
			issue_command(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
		}
		TOUCH_LOG("DC Delta (calibrated) \n");
		WRITE_SYSBUF(buf, resultDataLen, "=== DC Delta ===\n");
	} else {
		// calib by-pass data (rawdata)
		// bank=0x12, addr=0x6e, data=0x21
		if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
			issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);
		} else {
			issue_command(&g_ts, CMD_SETREGTBL_CALIB_OFF, CMD_SETREGTBL_CALIB_LEN);
		}
		TOUCH_LOG("DC Rawdata (by-pass) \n");
		WRITE_SYSBUF(buf, resultDataLen, "=== DC Rawdata ====\n");
	}
	msleep(50); // need for changing mode


#ifdef DEBUG_SHTSC
	TOUCH_LOG("exec DCMAP ON command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		issue_command_wo_IRQ(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);
	} else {
		issue_command(&g_ts, CMD_DCMAP_ON, CMD_DCMAP_ON_LEN);
	}

	// waiting for DCMAP ready
	(&g_ts)->wait_state = WAIT_DCMAP;
	(&g_ts)->wait_result = false;

	// wait for DCMAP ready
	err = WaitAsync(&g_ts);
	if( err != 0){
		//dump_stack();
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("WaitAsync in getDCMap return %d bankAddr %d\n",err, bankAddr);
#else
		TOUCH_ERR("WaitAsync in getDCMap return %d\n",err);
#endif
	}

#if defined(DEBUG_SHTSC)
	TOUCH_LOG("WaitAsync for DCMAP-ready is done");
#endif

	/* read DC map */
	readDcMapResultLPWG(client, buf, &resultDataLen, pDeltaMax);


#if defined(DEBUG_SHTSC)
	TOUCH_LOG("exec DCMAP OFF command (%s)\n", (rawflag ? "calib by-pass": "normal"));
#endif

	// set command2. waitAsync is in later
	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		issue_command2_wo_IRQ(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);
	} else {
		issue_command2(&g_ts, CMD_DCMAP_OFF, CMD_DCMAP_OFF_LEN);
	}

	/* dc map read done */
	/* this must  be done first before COMMAND_RESULT interrupt*/
	SetIndicator(&g_ts, SHTSC_IND_DCMAPDONE);

	err = 0;
	err = WaitAsync(&g_ts); // for CMD_DCMAP_OFF in issue command2
	if( err != 0){
		//dump_stack();
		TOUCH_LOG("WaitAsync in getDCMap  return %d\n",err);
	}

	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS ) {
		issue_command_wo_IRQ(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
	} else {
		issue_command(&g_ts, CMD_SETREGTBL_CALIB_ON, CMD_SETREGTBL_CALIB_LEN);
	}

	//TOUCH_LOG("back to normal DC (CALIB-ed = delta) mode \n");
	msleep(50); // need for changing mode

	return resultDataLen;
}


static ssize_t show_delta(struct i2c_client *client, char *buf)
{
	int ret = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	if (pDriverData->lpwgSetting.lcdState == 0){
		TOUCH_ERR("LCD Off state\n");
		WRITE_SYSBUF(buf, ret, "LCD OFF! Plase turn LCD On\n");
		return ret;
	}

#if defined(DEBUG_SHTSC)
	testingStart = 1;
#endif
	ret = shtsc_CMD_getDCMap_wo_IRQ(&g_ts, DC_CALIBED_DATA, buf );
#if defined(DEBUG_SHTSC)
	testingStart = 0;
#endif

	if(debug_app_enable == 1){
		debug_app_enable = 0;
	}

	TOUCH_LOG("Show delta done\n");

	return ret;
}

static ssize_t show_lpwg_rawdata_jitter(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int lpwg_rawdata_max = 0;
	int lpwg_jitter_max = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	if (pDriverData->lpwgSetting.lcdState == 1){
		TOUCH_ERR("LCD On state\n");
		WRITE_SYSBUF(buf, ret, "Please turn LCD off\n");
		return ret;
	}

	ret = shtsc_CMD_getDCMap_LPWG_Rawdata_Jitter(client, &g_ts, DC_CALIBED_DATA, JITTER_READ_REPEAT, buf, ret, &lpwg_rawdata_max, &lpwg_jitter_max);


	if (lpwg_rawdata_max <  SPEC_LPWG_RAWDATA_MAX) {
		WRITE_SYSBUF(buf, ret, "LPWG Rawdata : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
		TOUCH_LOG("LPWG Rawdata : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
	} else {
		WRITE_SYSBUF(buf, ret, "LPWG Rawdata : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
		TOUCH_LOG("LPWG Rawdata : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
	}

	if (lpwg_jitter_max <  SPEC_LPWG_JITTER_MAX) {
		WRITE_SYSBUF(buf, ret, "LPWG Jitter  : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
		TOUCH_LOG("LPWG Jitter  : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
	} else {
		WRITE_SYSBUF(buf, ret, "LPWG Jitter : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
		TOUCH_LOG("LPWG Jitter : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
	}

	TOUCH_LOG("LPWG rawdata and jitter done\n");

	return ret;
}

static ssize_t show_lpwg_delta(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int lpwg_deltaMax = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	if (pDriverData->lpwgSetting.lcdState == 1){
		TOUCH_ERR("LCD On state\n");
		WRITE_SYSBUF(buf, ret, "Please turn LCD off\n");
		return ret;
	}

#if defined(DEBUG_SHTSC)
	testingStart = 1;
#endif

	// LPWG Delta implementaiton
	ret = shtsc_CMD_getDCMap_LPWG_wo_IRQ(client, &g_ts, DC_CALIBED_DATA, buf, ret, &lpwg_deltaMax );

	if (lpwg_deltaMax > SPEC_LPWG_DELTA_MAX) {
		WRITE_SYSBUF(buf, ret, "LPWG Delta : OK ( Spec : %d , Max : %d\n", SPEC_LPWG_DELTA_MAX, lpwg_deltaMax);
	} else {
		WRITE_SYSBUF(buf, ret, "LPWG Delta : NG ( Spec : %d , Max : %d\n", SPEC_LPWG_DELTA_MAX, lpwg_deltaMax);
	}

#if defined(DEBUG_SHTSC)
	testingStart = 0;
#endif

	if(debug_app_enable == 1){
		debug_app_enable = 0;
	}

	TOUCH_LOG("LPWG delta done\n");

	return ret;
}

static ssize_t show_open_short_test(struct i2c_client *client, char *buf)
{
	int ret = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	if (pDriverData->lpwgSetting.lcdState == 0){
		TOUCH_ERR("LCD Off state\n");
		WRITE_SYSBUF(buf, ret, "Please turn LCD on\n");
		return ret;
	}

	WRITE_SYSBUF(buf, ret, "Open Short test : OK \n");
	TOUCH_LOG("Open Short test  : OK \n");

	TOUCH_LOG("Open Short test done\n");

	return ret;
}

static ssize_t show_jitter_test(struct i2c_client *client, char *buf)
{
	int ret = 0;
	//int dc_jitter_max = 0;
	int jitterCheckResult = TOUCH_FAIL;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	if (pDriverData->lpwgSetting.lcdState == 0){
		TOUCH_ERR("LCD Off state\n");
		WRITE_SYSBUF(buf, ret, "Please turn LCD on\n");
		return ret;
	}

	check_jitter_spec(client, buf, &ret, JITTER_READ_REPEAT, &jitterCheckResult);
	if(jitterCheckResult == TOUCH_SUCCESS){
		TOUCH_LOG("Jitter check : OK\n");
	} else {
		TOUCH_LOG("Jitter check : NG\n");
	}

#if 0
	if (dc_jitter_max <  SPEC_DC_JITTER_MAX) {
		WRITE_SYSBUF(buf, ret, "Jitter test : OK ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, dc_jitter_max);
		TOUCH_LOG("Jitter  : OK ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, dc_jitter_max);
	} else {
		WRITE_SYSBUF(buf, ret, "Jitter test : NG ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, dc_jitter_max);
		TOUCH_LOG("Jitter test : NG ( Spec : %d , Max : %d )\n", SPEC_DC_JITTER_MAX, dc_jitter_max);
	}
#endif

	TOUCH_LOG("Jitter test done\n");

	return ret;
}

static ssize_t show_rawdata(struct i2c_client *client, char *buf)
{
	int ret = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	if (pDriverData->lpwgSetting.lcdState == 0){
		TOUCH_ERR("LCD Off state\n");
		WRITE_SYSBUF(buf, ret, "Please turn LCD on\n");
		return ret;
	}

	ret = shtsc_CMD_getDCMap_wo_IRQ(&g_ts, DC_RAW_DATA, buf);

	if(debug_app_enable == 1){
		debug_app_enable = 0;
	}

	TOUCH_LOG("Rawdata (DC data) done\n");

	return ret;
}

static ssize_t show_pen_support(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	WRITE_BUFFER(buf, ret,	"1\n");

	return ret;
}

static ssize_t show_lpwg_fail_reason(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	WRITE_SYSBUF(buf, ret,	"%d\n", lpwg_fail_reason);

	return ret;
}

static ssize_t store_lpwg_fail_reason(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	TOUCH_FUNC();

	sscanf(buf, "%d", &value);

	TOUCH_LOG(" lpwg_fail_reason %d\n", value);

	if( value == 1 ) {
		lpwg_fail_reason_mask = KNOCK_CODE_FAILURE_REASONS ;
	} else {
		lpwg_fail_reason_mask = 0x00;
	}

	lpwg_fail_reason = value;

	return count;
}

static ssize_t show_unique_id(struct i2c_client *client, char *buf)
{
	int ret = 0;
	unsigned char unique_id_Buf[UNIQUE_ID_BUFFER_MAX];

	memset(unique_id_Buf, 0x0, sizeof(unique_id_Buf));

	if(get_unique_id_from_cache(client, unique_id_Buf, sizeof(unique_id_Buf)) == TOUCH_SUCCESS){
		TOUCH_LOG("Unique ID : %s\n", unique_id_Buf);
		WRITE_SYSBUF(buf, ret, "Unique ID : %s\n", unique_id_Buf);
	} else {
		TOUCH_LOG("Unique ID : Unknown\n");
		WRITE_SYSBUF(buf, ret, "Unique ID : Unknown\n");
		return TOUCH_FAIL;
	}

	return ret;
}

static ssize_t show_debug_app_enable(struct i2c_client *client, char *buf)
{
	int ret = 0;
	TOUCH_FUNC();

	debug_app_enable = 1;

	return ret;
}

static LGE_TOUCH_ATTR(factory_cal_update, S_IRUGO, show_factory_cal_update, NULL);
static LGE_TOUCH_ATTR(factory_cal_read, S_IRUGO, show_factory_cal_read, NULL);
static LGE_TOUCH_ATTR(factory_cal_check, S_IRUGO, show_factory_cal_check, NULL);
static LGE_TOUCH_ATTR(device_name, S_IRUGO, show_device_name, NULL);
static LGE_TOUCH_ATTR(lpwg_prox_test, (S_IWUSR | S_IRUGO), show_lpwg_prox_test, store_lpwg_prox_test);
static LGE_TOUCH_ATTR(property, S_IRUGO, show_property, NULL);
static LGE_TOUCH_ATTR(open_short_test, S_IRUGO, show_open_short_test, NULL);
static LGE_TOUCH_ATTR(jitter_test, S_IRUGO, show_jitter_test, NULL);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO, show_rawdata, NULL);
static LGE_TOUCH_ATTR(delta, S_IRUGO, show_delta, NULL);
static LGE_TOUCH_ATTR(lpwg_delta, S_IRUGO, show_lpwg_delta, NULL);
static LGE_TOUCH_ATTR(lpwg_rawdata_jitter, S_IRUGO, show_lpwg_rawdata_jitter, NULL);
static LGE_TOUCH_ATTR(calibration, S_IRUGO, show_calibration, NULL);
static LGE_TOUCH_ATTR(tci, (S_IWUSR | S_IRUGO), show_tci, store_tci);
static LGE_TOUCH_ATTR(lpwg_touch, S_IRUGO, show_lpwg_touch_data, NULL);
static LGE_TOUCH_ATTR(pen_support, S_IRUGO, show_pen_support, NULL);
static LGE_TOUCH_ATTR(lpwg_fail_reason, (S_IWUSR | S_IRUGO) , show_lpwg_fail_reason, store_lpwg_fail_reason);
static LGE_TOUCH_ATTR(unique_id, S_IRUGO, show_unique_id, NULL);
static LGE_TOUCH_ATTR(debug_app_enable, S_IRUGO, show_debug_app_enable, NULL);

static struct attribute *LR388K6_attribute_list[] = {
	&lge_touch_attr_factory_cal_update.attr,
	&lge_touch_attr_factory_cal_read.attr,
	&lge_touch_attr_factory_cal_check.attr,
	&lge_touch_attr_device_name.attr,
	&lge_touch_attr_lpwg_prox_test.attr,
	&lge_touch_attr_property.attr,
	&lge_touch_attr_open_short_test.attr,
	&lge_touch_attr_jitter_test.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_lpwg_delta.attr,
	&lge_touch_attr_lpwg_rawdata_jitter.attr,
	&lge_touch_attr_calibration.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_lpwg_touch.attr,
	&lge_touch_attr_pen_support.attr,
	&lge_touch_attr_lpwg_fail_reason.attr,
	&lge_touch_attr_unique_id.attr,
	&lge_touch_attr_debug_app_enable.attr,
	NULL,
};


static const struct file_operations lr388k6_fops = {
	.owner = THIS_MODULE,
	.open	= lr388k6_misc_open,
	.release = lr388k6_misc_release,
	.read	= lr388k6_misc_read,
	.llseek = no_llseek,
	.unlocked_ioctl = lr388k6_misc_unlocked_ioctl,
	.compat_ioctl = lr388k6_misc_unlocked_ioctl,
};

static int lr388k6_misc_open(struct inode *inode, struct file *file)
{
	TOUCH_FUNC();

	return 0;
}

static int lr388k6_misc_release(struct inode *inode, struct file *file)
{
	TOUCH_FUNC();

	return 0;
}

static ssize_t lr388k6_misc_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	int copy_len;
	int i;

	TOUCH_FUNC();

	if( count > buf_pos){
		copy_len = buf_pos;
	} else {
		copy_len = count;
	}


	if( copy_to_user( buf, devbuf, copy_len) ) {
		TOUCH_ERR(" copy_to_user failed \n");
		return -EFAULT;
	}

	*pos += copy_len;

	for ( i = copy_len; i < buf_pos ; i++){
		devbuf[ i - copy_len ] = devbuf[i];
	}
	buf_pos -= copy_len;

	return copy_len;
}

static long lr388k6_misc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	u8 addr = 0;
	u8 val = 0;
	int r = 0;

#if defined(DEBUG_SHTSC)
	TOUCH_LOG(" cmd : %x, arg : %lx \n", cmd, arg);
#endif

	switch(cmd) {

		case SHTSC_IOCTL_SET_PAGE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case SET_PAGE; cmd %x, arg %lx\n", cmd, arg);
#endif
			current_page = arg;
			break;

		case SHTSC_IOCTL_FLASH_ACCESS_START:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case FLASH_ACCESS_START; cmd %x\n", cmd);
#endif
			flash_access_start_shtsc(&g_ts);
			break;

		case SHTSC_IOCTL_FLASH_ACCESS_END:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case FLASH_ACCESS_END; cmd %x\n", cmd);
#endif
			flash_access_end_shtsc(&g_ts);
			break;

		case SHTSC_IOCTL_ERASE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case ERASE; cmd %x, current_page %d\n", cmd, current_page);
#endif
#ifdef FLASH_CHECK
			if (flash_erase_page_shtsc(&g_ts, current_page))
				return -1;
#else /* FLASH_CHECK */
			flash_erase_page_shtsc(&g_ts, current_page);
#endif /* FLASH_CHECK */
			break;

		case SHTSC_IOCTL_WRITE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case WRITE; cmd %x, current_page %d\n", cmd, current_page);
#endif
			memset(iobuf, 0x0, IO_BUF_SIZE);
			if (copy_from_user(iobuf, (char *)arg, IO_BUF_SIZE)) {
				TOUCH_ERR("ERROR by copy_from_user\n");
				return -1;
			}
#ifdef FLASH_CHECK
			if (flash_write_page_shtsc(&g_ts, current_page, iobuf))
				return -1;
#else /* FLASH_CHECK */
			flash_write_page_shtsc(&g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
			break;

		case SHTSC_IOCTL_VERIFY:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case VERIFY; cmd %x, current_page %d\n", cmd, current_page);
#endif
			memset(iobuf, 0x0, IO_BUF_SIZE);
			if (copy_from_user(iobuf, (char *)arg, IO_BUF_SIZE)) {
				TOUCH_ERR("ERROR by copy_from_user\n");
				return -1;
			}
#ifdef FLASH_CHECK
			if (flash_verify_page_shtsc(&g_ts, current_page, iobuf))
				return -1;
#else /* FLASH_CHECK */
			flash_verify_page_shtsc(&g_ts, current_page, iobuf);
#endif /* FLASH_CHECK */
			break;

		case SHTSC_IOCTL_ERASE_ALL:
			{
#define LAST_PAGE 16
				int page;
				TOUCH_LOG("case ERASE_ALL flash_access start\n");
				flash_access_start_shtsc(&g_ts);
				for (page = 0; page < LAST_PAGE; page++) {
					flash_erase_page_shtsc(&g_ts, page);
					TOUCH_LOG("flash_erase_page_shtsc done: page %d\n",  page);
				}
				TOUCH_LOG("flash_access end\n");
				flash_access_end_shtsc(&g_ts);
				TOUCH_LOG("flash erased.\n");
			}
			break;
		case SHTSC_IOCTL_REG_1WRITE:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_1WRITE: cmd %x, arg %lx \n", cmd, arg);
#endif
			addr = (arg >> 8) & 0xFF;
			val = arg & 0xFF;
			TOUCH_LOG("cmd %x, arg %lx a:0x%02X value :0x%02X(%d)\n", cmd, arg, addr, val, val);

			WriteOneByte(&g_ts, addr, val);
			break;

		case SHTSC_IOCTL_REG_1READ:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_1READ: cmd %x, arg %lx\n", cmd, arg);
#endif
			val = ReadOneByte(&g_ts, ((struct reg *)arg)->addr);
			((struct reg *)arg)->data = val;

			TOUCH_LOG("cmd %x, arg %lx a:0x%02X value :0x%02X(%d)\n", cmd, arg, ((struct reg *)arg)->addr, val, val);
			break;


		case SHTSC_IOCTL_REG_N_RW_SET_ADDR:
			s_shtsc_addr = 0xFF & (arg >> 16);
			s_shtsc_len = 0xFFFF & arg;
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_N_RW_SET_ADDR: cmd %x, arg %lx a:0x%x len:%ld\n", cmd, arg, s_shtsc_addr, s_shtsc_len);
#endif
			break;

		case SHTSC_IOCTL_REG_N_WRITE_1ADDR_GO:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_N_WRITE_1ADDR_GO : cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#endif
			/* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
			r = copy_from_user(s_shtsc_buf, (char *)arg, s_shtsc_len);
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_from_user(%d)\n", r);
				return -1;
			}
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("Driver Multibyte write. addr %02X, len: %lx, data[0-3]: %02X %02X %02X %02X ....\n", s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
#if defined(DEBUG_SHTSC)
			TOUCH_LOG(" cmd %x, arg %lx a:0x%x d:0x%lx(%ld)\n", cmd, arg, s_shtsc_addr, s_shtsc_len, s_shtsc_len);
#endif
			WriteMultiBytes(&g_ts, s_shtsc_addr, s_shtsc_buf, s_shtsc_len);

			break;

		case SHTSC_IOCTL_REG_N_READ_1ADDR_GO:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case REG_N_READ_1ADDR_GO : cmd %x, arg %lx a:0x%x d:0x%x(%d)\n", cmd, arg, addr, val, val);
#endif
			/* s_shtsc_len is initialized __IF__ SHTSC_IOCTL_REG_N_RW_SET_ADDR is issued before */
			ReadMultiBytes(&g_ts, s_shtsc_addr, s_shtsc_len, s_shtsc_buf);
			//msleep(10); // not checked yet
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("Driver Multibyte read done. addr %02X, len: %lx, data[0-3]: %02X %02X %02X %02X ....\n",
					s_shtsc_addr, s_shtsc_len, s_shtsc_buf[0], s_shtsc_buf[1], s_shtsc_buf[2], s_shtsc_buf[3]);
#endif
			r = copy_to_user((char *)arg, s_shtsc_buf, s_shtsc_len);
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
				return -1;
			}

			break;

		case SHTSC_IOCTL_SETIRQMASK:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case SETIRQMASK; cmd %x, arg %lx\n", cmd, arg);
#endif
			if (arg) {
				TouchEnableIrq();
			} else {
				TouchDisableIrq();
			}
			break;

		case SHTSC_IOCTL_RESET:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case RESET; cmd %x, arg %lx\n", cmd, arg);
#endif
			if (arg) {
				int err = 0;
				g_ts.wait_state = WAIT_RESET;
				g_ts.wait_result = false;
				//shtsc_reset(&g_ts, true);
				TouchResetCtrl(1);
				// wait
				err = WaitAsync(&g_ts);
				if( err != 0){
					//dump_stack();
					TOUCH_LOG("WaitAsync in SHTSC_IOCTL_RESET return %d\n",err);
				}
				shtsc_system_init(&g_ts);
				msleep(100);
			} else {
				TouchResetCtrl(0);
				G_Irq_Mask = 0xffff;
				//shtsc_reset(&g_ts, false);
			}
			break;

		case SHTSC_IOCTL_DEBUG:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case DEBUG; cmd %x, arg %lx\n", cmd, arg);
#endif
			break;

		case SHTSC_IOCTL_CMD_ISSUE_RESULT:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case CMD_ISSUE_RESULT; cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				unsigned int magicnumber; /* 'int' width is 32bit for LP64 and LLP64 mode */
				if ( copy_from_user(&magicnumber, (char *) arg, 4) ){
					magicnumber = 0;
				}
				if( magicnumber != 0xA5A5FF00 ){
					msleep(100);
				}
				r = copy_to_user((char *) arg, CommandResultBuf,MAX_COMMAND_RESULT_LEN);
				if( magicnumber == 0xA5A5FF00 ){
					CommandResultBuf[0] = 0;
				}
			}
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
				return -1;
			}
			break;

		case SHTSC_IOCTL_DRIVER_VERSION_READ:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case DRIVER_VERSION_READ; cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				char versionBuf[16];
				versionBuf[0] = VERSION_YEAR;
				versionBuf[1] = VERSION_MONTH;
				versionBuf[2] = VERSION_DAY;
				versionBuf[3] = VERSION_SERIAL_NUMBER;
				versionBuf[4] = VERSION_MODEL_CODE;

				r = copy_to_user((char *)arg, versionBuf, DRIVER_VERSION_LEN);
				if (r != 0) {
					TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
					return -1;
				}
			}
			break;

		case SHTSC_IOCTL_DCMAP:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case DCMAP; cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				int x = dcmap[0];
				int y = dcmap[1];
				int len = (x * y * 2) + 4;

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("cmd %x, arg %lx, %d*%d+4=%d\n", cmd, arg, x, y, len);
#endif

				if (buf_pos) {
					/* DC map ready to send out */
					r = copy_to_user((char *)arg, dcmap, len);
					if (r != 0) {
						TOUCH_LOG("ERROR by copy_to_user failed\n" );
						return -EFAULT;
					}
					buf_pos = 0;
				}
				break;
			}

#ifdef GET_REPORT
		case SHTSC_IOCTL_GET_REPORT:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case GET_REPORT;  cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				volatile int count;
				int len;


				while (Mutex_GetReport)
					;
				Mutex_GetReport = true;

				count = reportBuf[0];
				len = 4+ count*REP_SIZE;

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("cmd %x, arg %lx, count=%d, len=%d\n", cmd, arg, count, len);
#endif

				r = copy_to_user((char *)arg, reportBuf, len);
				if (r != 0) {
					TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
					return -1;
				}

				reportBuf[0] = (unsigned char)0;

				Mutex_GetReport = false;
			}
			break;
#endif /* GET_REPORT */

		case SHTSC_IOCTL_FLASH_READ:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case FLASH_READ;  cmd %x, arg %lx\n", cmd, arg);
#endif
			{
				/* arg: pointer to the content buffer */
				/* arg[3:0]: address to read (little endian) */
				/* arg[5:4]: length to read (little endian) */

				unsigned address;
				unsigned length;

				memset(iobuf, 0x0, IO_BUF_SIZE);

				if (copy_from_user(iobuf, (char *)arg, (4+2))) {
					TOUCH_LOG("ERROR by copy_from_user\n");
					return -1;
				}
				address = (iobuf[3] << 24) | (iobuf[2] << 16) | (iobuf[1] << 8) | (iobuf[0] << 0);
				length = (iobuf[5] << 8) | (iobuf[4] << 0);

#if defined(DEBUG_SHTSC)
				TOUCH_LOG("addr %x, arg %x\n", address, length);
#endif

				flash_read(&g_ts, address, length, iobuf);
				if ( copy_to_user( (char *)arg, iobuf, length ) ) {
					printk( KERN_INFO "shtsc : copy_to_user failed\n" );
					return -EFAULT;
				}
				break;
			}

		case SHTSC_IOCTL_NOTIFY_PID:
			pid = arg; // save pid for later kill();
			TOUCH_LOG("case NOTIFY_PID: pid: %d\n", pid);
			break;

		case SHTSC_IOCTL_GET_INTERRUPT_STATUS:
#if defined(DEBUG_SHTSC)
			TOUCH_LOG("case GET_INTERRUPT_STATUS, %d\n", resumeStatus);
#endif
			r = copy_to_user((char *)arg, &resumeStatus, 1); // copy one-byte status
			if (r != 0) {
				TOUCH_LOG("ERROR by copy_to_user(%d)\n", r);
				return -1;
			}
			break;

		default:
			ret = 0;
			break;
	}
	return ret;
}

static int check_flash_load_error(struct i2c_client *client)
{
	u16 u16status = 0;
	//u8 regcommonbuf[12] = {0, }; // less than 12

	g_ts.client = client;

	memset(regcommonbuf, 0x0, SIZE_REG_COMMON_BUF);
	ReadMultiBytes(&g_ts,SHTSC_ADDR_INT0,11,regcommonbuf);
	if( TOUCH_FAIL == check_buffer_overflow(regcommonbuf, 11, SIZE_REG_COMMON_BUF , "regcommonbuf")){
		TOUCH_ERR("Check this overflow issue\n");
		dump_stack();
	}
	u16status = ((regcommonbuf[SHTSC_ADDR_INT1] << 8) | regcommonbuf[SHTSC_ADDR_INT0]);

	if (u16status & SHTSC_STATUS_FLASH_LOAD_ERROR){
		TOUCH_LOG("LR388K6_Initialize [IRQ] shtsc flash load error u16status=0x%04x\n",u16status);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static void WqResetAgain(struct work_struct *work)
{
	TOUCH_FUNC();

	reset_again_count++;

	TOUCH_LOG("RESET_AGAIN! count %d \n", reset_again_count);

	LR388K6_Reset(NULL);

}
static void WqFolioCoverSetting(struct work_struct *work)
{
	int xs = 920; // 895;
	int xe = 1060; // 1079; //1049;
	int ys = 0;
	int ye = 1919; //1866;
	// xs, ys, xe, ye should be given from the upper layer or fixed values
	char areaSetBuf[CMD_TOUCH_ENABLE_CONTROL_LENGTH];

	TOUCH_FUNC();

	TOUCH_LOG("Folio cover closed. active area changed to limited area (%d, %d)-(%d, %d)\n", xs, ys, xe, ye);

	memcpy(areaSetBuf, CMD_TOUCH_OFF, (CMD_TOUCH_ENABLE_CONTROL_LENGTH-8));
	//
	// (xs,ys): left-upper touch enabled rectangle corner
	// (xe,ye): right-lower touch enabled rectangle corner
	// xs(2bytes), ys(2bytes), xe(2bytes), ye(2bytes) (little endian)
	//
	areaSetBuf[5] = (xs&0xff); // left-upper x lower byte
	areaSetBuf[6] = ((xs&0xff00)>>8); // left-upper x higher byte
	areaSetBuf[7] = (ys&0xff); // left-upper y lower byte
	areaSetBuf[8] = ((ys&0xff00)>>8); // left-upper y higher byte
	areaSetBuf[9] = (xe&0xff); // right-lower x lower byte
	areaSetBuf[10] = ((xe&0xff00)>>8); // right-lower x higher byte
	areaSetBuf[11] = (ye&0xff); // right-lower y lower byte
	areaSetBuf[12] = ((ye&0xff00)>>8); // right-lower y higher byte
	issue_command(&g_ts, areaSetBuf, CMD_TOUCH_ENABLE_CONTROL_LENGTH); // use _wo_IRQ if IRQ-disable context

	// now LCD is ON (based on the information of upper "if"
	TOUCH_LOG("Going to COVER mode\n");
	issue_command(&g_ts, CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN); // use _wo_IRQ if IRQ-disable context

}
//====================================================================
// Function : LR388K6_Initialize
// Description
//   -
//   -
//====================================================================

static int LR388K6_Initialize(struct i2c_client *client)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int retry_cnt = 0;
	int result = TOUCH_SUCCESS;

	TOUCH_FUNC();

	// init g_ts;
	g_ts.client = client;
	g_ts.wait_state = WAIT_NONE;
	g_ts.wait_result = false;


	pDriverData->isMiscDevice = 1;
	pDriverData->touch_misc_device.name = DRIVER_NAME;
	pDriverData->touch_misc_device.fops = &lr388k6_fops;

	if(check_flash_load_error(client) == TOUCH_FAIL){
		for(retry_cnt = 0; retry_cnt < 3; retry_cnt++){
			result = LR388K6_UpdateFirmware(client, NULL);
			if(result == TOUCH_SUCCESS){
				TOUCH_LOG("Firmware Recovery was Succeeded\n");
				break;
			}
			TOUCH_WARN("Retry firmware Recovery\n");
		}
		LR388K6_Reset(client);
	}

	mutex_init(&g_ts.mutex_irq);

	if(wq_reset_check == NULL) {
		wq_reset_check = create_singlethread_workqueue("reset_check");
		TOUCH_LOG("reset_check wq created\n");
		INIT_DELAYED_WORK(&work_reset_check, WqResetAgain);
	}

	if(wq_folio_cover_check == NULL) {
		wq_folio_cover_check = create_singlethread_workqueue("folio_cover_check");
		TOUCH_LOG("folio_cover_check wq created\n");
		INIT_DELAYED_WORK(&work_folio_cover_check, WqFolioCoverSetting);
	}

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K6_Reset
// Description
//   -
//   -
//====================================================================
static void LR388K6_Reset(struct i2c_client *client)
{
	//	int count = 0;

	//TOUCH_FUNC();

	TouchResetCtrl(0);
	msleep(50);
	// frequent irq interrupted
	//
	G_Irq_Mask = 0xffff;
	TouchResetCtrl(1);

	msleep(200);

	dummyDeviceState = STATE_NORMAL;

	if(wq_reset_check != NULL){
		TOUCH_LOG("RESET start! target 500ms \n");
		queue_delayed_work(wq_reset_check, &work_reset_check, msecs_to_jiffies(500));
	}

}


//====================================================================
// Function : LR388K6_Connect
// Description
//   -
//   -
//====================================================================
static int LR388K6_Connect(void)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K6_InitRegister
// Description
//   - Initialize touch IC register
//   - will be called after IC reset ( by reset pin )
//====================================================================
static int LR388K6_InitRegister(struct i2c_client *client)
{
	//	struct device_node *np;
	//	TouchDriverData *pDriverData;

	TOUCH_FUNC();

	//msleep(100);

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : LR388K6_ClearInterrupt
// Description
//   - Clear interrupt
//   - will be called before interrupt enable to clear interrupt happened during interrupt disabled time
//====================================================================
static void LR388K6_ClearInterrupt(struct i2c_client *client)
{
	//TOUCH_FUNC();

	return;
}

//====================================================================
// Function : LR388K6_InterruptHandler
// Description
//   - process interrupt
//   - will be called if interrupt detected by AP
//====================================================================
static int LR388K6_InterruptHandler(struct i2c_client *client, TouchReadData *pData)
{
	//TOUCH_FUNC();

	g_ts.client = client;

	shtsc_irq_thread(client, &g_ts, pData);

	return TOUCH_SUCCESS;
}

static int WaitAsync(void *_ts)
{
	struct shtsc_i2c *ts = _ts;

	int i;
	for(i=0;i<50;i++) {
		mdelay(CMD_DELAY);//16ms
		//DBGLOG("shtsc: wait 16ms for state change\n");
		switch(ts->wait_state) {
			case WAIT_RESET:
				break;
			case WAIT_CMD:
				break;
			case WAIT_DCMAP:
				break;
			case WAIT_NONE:
				if (ts->wait_result == true) {
					//TOUCH_LOG("shtsc: wait state change: success\n");
					return 0;
				}
				else
					return -EIO;
			default:
				break;
		}
	}
	//DBGLOG("wait state change: failure\n");
	return -EIO;
}

// this function should be called while IRQ is disabled or multex is locked
// i.e. WaitAsync() cannot be called because it will be timeout without IRQ handler to move to WAIT_NONE
int issue_command_wo_IRQ(void *ts, unsigned char *cmd, unsigned int len)
{
	int count = 0;
	u8 value = 0;

	//TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

#if defined (DEBUG_SHTSC)
	TOUCH_LOG("%s : cmd (%x) (%x) (%x) (%x)\n",__func__,cmd[0], cmd[1], cmd[2], cmd[3]);
#endif

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);

	SetIndicator(ts, SHTSC_IND_CMD);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
	TOUCH_ERR("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));
#endif

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result
#if defined(DEBUG_SHTSC)
		if(testingStart == 1){
			TOUCH_LOG("waiting the result count %d\n", count);
		}
#endif
		msleep(5);
		count++;

		if(count > 50){
			TOUCH_ERR("waiting the result count %d. Please check it\n", count);
			break;
		}
	}
#if defined(DEBUG_SHTSC)
	TOUCH_LOG("%s : Before BANK_COMMMAND_RESULT BankAddr %d \n", __func__, bankAddr);
#endif

	SetBankAddr(&g_ts,SHTSC_BANK_COMMAND_RESULT);
	ReadMultiBytes(&g_ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result.

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}

int issue_command(void *ts, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	mutex_lock(&g_ts.mutex_irq);

	SetBankAddr(ts, SHTSC_BANK_COMMAND);
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);

	mutex_unlock(&g_ts.mutex_irq);

#if defined (DEBUG_SHTSC)
	TOUCH_LOG("%s : cmd (%x) (%x) (%x) (%x)\n",__func__,cmd[0], cmd[1], cmd[2], cmd[3]);
#endif

	// prepare waiting
	((struct shtsc_i2c *)ts)->cmd = cmd[0];
	((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
	((struct shtsc_i2c *)ts)->wait_result = true;

	// do it
	SetIndicator(ts, SHTSC_IND_CMD);
	//DBGLOG("do it\n");


	// wait
	err = WaitAsync(ts);
	if( err != 0){
#if defined(DEBUG_SHTSC)
		TOUCH_LOG("WaitAsync in issue_command return %d bankAddr %d\n",err, bankAddr);
#else
		TOUCH_LOG("WaitAsync in issue_command return %d\n",err);
#endif
	}

	return err;
}

/* only for DCmap off for one shot */
int issue_command2(void *ts, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	mutex_lock(&g_ts.mutex_irq);

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);

	mutex_unlock(&g_ts.mutex_irq);

#if defined (DEBUG_SHTSC)
	TOUCH_LOG("%s : cmd (%x) (%x) (%x) (%x)\n",__func__,cmd[0], cmd[1], cmd[2], cmd[3]);
#endif

	// prepare waiting
	((struct shtsc_i2c *)ts)->cmd = cmd[0];
	((struct shtsc_i2c *)ts)->wait_state = WAIT_CMD;
	((struct shtsc_i2c *)ts)->wait_result = true;

	// do it
	SetIndicator(ts, SHTSC_IND_CMD);
	//DBGLOG("do it\n");


	// wait
	//	WaitAsync(ts);
	TOUCH_LOG("issue_command2 executed.\n");

	return err;
}

/* only for DCmap off for one shot */
int issue_command2_wo_IRQ(void *ts, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	mutex_lock(&g_ts.mutex_irq);

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, cmd, len);

	mutex_unlock(&g_ts.mutex_irq);

#if defined (DEBUG_SHTSC)
	TOUCH_LOG("%s : cmd (%x) (%x) (%x) (%x)\n",__func__,cmd[0], cmd[1], cmd[2], cmd[3]);
#endif

	SetIndicator(ts, SHTSC_IND_CMD);

	//TOUCH_LOG("issue_command2_wo_IRQ executed.\n");

	return err;
}

int lpwg_param_command(void *ts, u8 u8Addr, unsigned char *cmd, unsigned int len)
{
	int err = 0;

	SetBankAddr(ts, SHTSC_BANK_LPWG_PARAM);
	//TOUCH_LOG("tci bank command LPWG_PARM \n");
	// set command
	WriteMultiBytes(ts, u8Addr, cmd, len);
	//TOUCH_LOG("tci set LPWG_PARM command (0x%x) (0x%x) \n",cmd[0], cmd[1]);

	return err;
}

//====================================================================
// Function : LR388K6_ReadIcFirmwareInfo
// Description
//   - Read firmware information from touch IC
//   - will be called at boot time or after writing new firmware
//====================================================================
static int LR388K6_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{

	// unsigned firmver = 0;
	// unsigned paramver = 0;

	int value =0;
	int count =0;

	TOUCH_FUNC();

	// Bank Address
	if( TOUCH_FAIL == SetBankAddr(&g_ts, SHTSC_BANK_COMMAND)){
		TOUCH_LOG("I2C failure \n");
		return TOUCH_FAIL;
	}

	// set command
	WriteMultiBytes(&g_ts, SHTSC_ADDR_COMMAND, CMD_GETPROPERTY, CMD_GETPROPERTY_LEN);

	// Indicator
	SetIndicator(&g_ts, SHTSC_IND_CMD);

	msleep(100);
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(&g_ts, SHTSC_ADDR_INT0))) {
		; // waiting the result of getproperty
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 10){
			TOUCH_LOG("waiting the result count %d. Please check it\n", count);
			break;
		}
	}

	value = ReadOneByte(&g_ts, SHTSC_ADDR_INT0);
#ifdef DEBUG_SHTSC
	TOUCH_LOG("1st Value check value %d SHTSC_STATUS_COMMAND_RESULT & value %d \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));
#endif

#ifdef DEBUG_SHTSC
	TOUCH_LOG("%s : Before BANK_COMMMAND_RESULT BankAddr %d \n", __func__, bankAddr);
#endif
	// Bank address
	SetBankAddr(&g_ts,SHTSC_BANK_COMMAND_RESULT);

	// READ result
	ReadMultiBytes(&g_ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf); // always read entire result. TEGAPP will handle it.
#ifdef DEBUG_SHTSC
	TOUCH_LOG("Command result (GetProperty[E0]), Operation code: %02X, Error code:%02X\n", CommandResultBuf[0], CommandResultBuf[1]);
#endif

#if 0
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[0], CommandResultBuf[1], CommandResultBuf[2],CommandResultBuf[3]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[4], CommandResultBuf[5], CommandResultBuf[6],CommandResultBuf[7]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[8], CommandResultBuf[9], CommandResultBuf[10],CommandResultBuf[11]);
	TOUCH_LOG(" command result, %02X, %02X, %02X, %02X\n", CommandResultBuf[12], CommandResultBuf[13], CommandResultBuf[14],CommandResultBuf[15]);
#endif

	K6_firmver = 0;
	K6_paramver = 0;

	K6_firmver = ((CommandResultBuf[0x0a-0x08 +3] << 24) |
			(CommandResultBuf[0x0a-0x08 +2] << 16) |
			(CommandResultBuf[0x0a-0x08 +1] << 8) |
			(CommandResultBuf[0x0a-0x08 +0] << 0));
	K6_paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
			(CommandResultBuf[0x12-0x08 +2] << 16) |
			(CommandResultBuf[0x12-0x08 +1] << 8) |
			(CommandResultBuf[0x12-0x08 +0] << 0));

	TOUCH_LOG("Firmware %08X, Parameter %08X\n", K6_firmver, K6_paramver);

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	pFwInfo->moduleMakerID = 2;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = (CommandResultBuf[0x12-0x08+2] & 0x01);
	pFwInfo->version = CommandResultBuf[0x12-0x08];

	TOUCH_LOG("FW official %d version %d \n", pFwInfo->isOfficial, pFwInfo->version);

	if(pFwInfo->version == 0) {
		unsigned paramver = 0;
		msleep(50);
		paramver = ((CommandResultBuf[0x12-0x08 +3] << 24) |
				(CommandResultBuf[0x12-0x08 +2] << 16) |
				(CommandResultBuf[0x12-0x08 +1] << 8) |
				(CommandResultBuf[0x12-0x08 +0] << 0));
		TOUCH_LOG(" paramver  %08X \n", paramver);
	}


	return TOUCH_SUCCESS;
}

//====================================================================
// Function : LR388K6_GetBinFirmwareInfo
// Description
//   - parse and return firmware information from firmware image
//   - if filename is NULL, return information of default firmware image
//   - will be called at boot time or needed
//====================================================================
static int LR388K6_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;
	unsigned long image_size = 0;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

#if 1
	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	image_size = fw->size;
	pBin = kzalloc(sizeof(char) * (image_size+1), GFP_KERNEL);
	if (pBin == NULL) {
		TOUCH_ERR("Can not allocate memory\n");
		goto  error;
	}

	memcpy(pBin, fw->data, image_size);

	TOUCH_ERR("success from image to buffer size %lu \n", image_size );

#if 1//HMOON
	TOUCH_LOG("file fw ver=%02x%02x%02x%02x,param ver=%02x%02x%02x%02x\n",
			pBin[0x23+7],pBin[0x22+7],pBin[0x21+7],pBin[0x20+7],
			pBin[image_size-6],pBin[image_size-7],pBin[image_size-8],pBin[image_size-9] );
#endif

	pFwInfo->version = pBin[image_size-9]; // HMOON ((firmver >> 32) | paramver); // 32bit value

#endif

	pFwInfo->moduleMakerID = 2;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = (pBin[image_size-7] & 0x01);
	pFwInfo->version = pBin[image_size-9];

	TOUCH_LOG("BIN official %d version %d \n", pFwInfo->isOfficial, pFwInfo->version);

	if(pBin != NULL){
		kfree(pBin);
		TOUCH_ERR("free buffer \n");
		pBin=NULL;
	}
	/* Free firmware image buffer */
	release_firmware(fw);
	return TOUCH_SUCCESS;

error:
	//HMOON TEMP
	pFwInfo->moduleMakerID = 2;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	//	pFwInfo->isOfficial = 1;
	pFwInfo->isOfficial = 0;
	pFwInfo->version = 0; // HMOON ((firmver >> 32) | paramver); // 32bit value


	if(pBin != NULL){
		kfree(pBin);
		TOUCH_ERR("free buffer \n");
		pBin = NULL;
	}

	/* Free firmware image buffer */
	release_firmware(fw);
	return TOUCH_FAIL;
}


//====================================================================
// Function : LR388K6_UpdateFirmware
// Description
//   - Write firmware to touch IC
//   - if filename is NULL, use default firmware image
//   - common driver will call Reset(), InitRegister() and ReadIcFirmwareInfo() one by one after writing
//====================================================================
static int LR388K6_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;
	unsigned long image_size;
	TouchDriverData *pDriverData = NULL;

	TOUCH_FUNC();

	pDriverData = i2c_get_clientdata(client);

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	if(pDriverData != NULL && pDriverData->bootMode == BOOT_MFTS){
		pFwFilename = (char *)defaultFirmware;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	image_size = fw->size;
	pBin = kzalloc(sizeof(char) * (image_size+1), GFP_KERNEL);
	if (pBin == NULL) {
		TOUCH_ERR("Can not allocate memory\n");
		goto  error;
	}

	memcpy(pBin, fw->data, image_size);

	TOUCH_ERR("success from image to buffer size %lu \n", image_size);

	/* IMPLEMENT : firmware update function */
	//shtsc_proc_firmup_func(pBin, image_size);

	shtsc_CMD_SetSystemStateSleep_wo_IRQ(&g_ts);

	update_flash(&g_ts,pBin,image_size);

	unique_id_saved = 0;

error:

	if(pBin != NULL){
		kfree(pBin);
	}

	/* Free firmware image buffer */
	release_firmware(fw);

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : LR388K6_SetLpwgMode
// Description
//   - Set device to requested state
//====================================================================
static int LR388K6_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;

#if 0
	TOUCH_LOG("activeTouchAreaX1 [%d]\n",	pLpwgSetting->activeTouchAreaX1);
	TOUCH_LOG("activeTouchAreaX2 [%d]\n",	pLpwgSetting->activeTouchAreaX2);
	TOUCH_LOG("activeTouchAreaY1 [%d]\n",	pLpwgSetting->activeTouchAreaY1);
	TOUCH_LOG("activeTouchAreaY2 [%d]\n",	pLpwgSetting->activeTouchAreaY2);
#endif

	/* FOLIOCOVER_CLOSED : 0 FOLIOCOVER_OPEN : 1 */
	if(pLpwgSetting->coverState == FOLIOCOVER_CLOSED) {

		TOUCH_LOG("%s FOLIOCOVER_CLOSE!\n", __func__);
	}
	else {

		TOUCH_LOG("%s FOLIOCOVER_OPEN!\n", __func__);
		// 80 pixel 5.25 mm
		// 106 pixel 7mm
		// 120 pixel about 8 mm
		// 152 pixel 10mm

		// Top,Bottom 106 pixel 7mm, Left,Right 137 pixel for 9 mm

		pLpwgSetting->activeTouchAreaX1 = 80; // 5mm
		pLpwgSetting->activeTouchAreaX2 = (1080 - 80); // 5mm
		pLpwgSetting->activeTouchAreaY1 = 80;// 5mm
		pLpwgSetting->activeTouchAreaY2 = (1920 - 80);// 5mm

	}
#if 0
	TOUCH_FUNC();
	TOUCH_LOG("mode [%d]\n",				pLpwgSetting->mode);
	TOUCH_LOG("lcdPixelSizeX [%d]\n",		pLpwgSetting->lcdPixelSizeX);
	TOUCH_LOG("lcdPixelSizeY [%d]\n",		pLpwgSetting->lcdPixelSizeY);
	TOUCH_LOG("activeTouchAreaX1 [%d]\n",	pLpwgSetting->activeTouchAreaX1);
	TOUCH_LOG("activeTouchAreaX2 [%d]\n",	pLpwgSetting->activeTouchAreaX2);
	TOUCH_LOG("activeTouchAreaY1 [%d]\n",	pLpwgSetting->activeTouchAreaY1);
	TOUCH_LOG("activeTouchAreaY2 [%d]\n",	pLpwgSetting->activeTouchAreaY2);
	TOUCH_LOG("tapCount [%d]\n",			pLpwgSetting->tapCount);
	TOUCH_LOG("isFirstTwoTapSame [%d]\n",	pLpwgSetting->isFirstTwoTapSame);
	TOUCH_LOG("lcdState [%d]\n",			pLpwgSetting->lcdState);
	TOUCH_LOG("proximityState [%d]\n",		pLpwgSetting->proximityState);
	TOUCH_LOG("coverState [%d]\n",			pLpwgSetting->coverState);
	TOUCH_LOG("callState [%d]\n",			pLpwgSetting->callState);

#endif
	memcpy(&(g_ts.lpwgSetting), pLpwgSetting, sizeof(LpwgSetting));

	if( dummyDeviceState == newState ) {
		TOUCH_LOG("device state is same as driver requested\n");
		return TOUCH_SUCCESS;
	}

	if ((newState < STATE_NORMAL) && (newState > STATE_KNOCK_ON_CODE)) {
		TOUCH_LOG("invalid request state ( state = %d )\n", newState);
		return TOUCH_FAIL;
	}

	ret = lpwg_control(client, newState);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("failed to set lpwg mode in device\n");
		return TOUCH_FAIL;
	}

	switch( newState )
	{
		case STATE_NORMAL:
			//TOUCH_LOG("device was set to NORMAL\n");
			break;
		case STATE_OFF:
			//TOUCH_LOG("device was set to OFF\n");
			break;
		case STATE_KNOCK_ON_ONLY:
			//TOUCH_LOG("device was set to KNOCK_ON_ONLY\n");
			break;
		case STATE_KNOCK_ON_CODE:
			//TOUCH_LOG("device was set to KNOCK_ON_CODE\n");
			break;
		case STATE_NORMAL_HOVER:
			//TOUCH_LOG("device was set to NORMAL_HOVER\n");
			break;
		case STATE_HOVER:
			//TOUCH_LOG("device was set to HOVER\n");
			break;
		default:
			TOUCH_LOG("invalid state ( state = %d )\n", newState);
			ret = TOUCH_FAIL;
			break;

	}

	if( ret == TOUCH_SUCCESS ) {
		dummyDeviceState = newState;
	}

	return TOUCH_SUCCESS;

}

static int checkLimitData(u8* buffer, int limitType, char* buf, int dataLen, int *pRawDataStatus, int* pTotalMin, int* pTotalMax, int batt_temp)
{
	int driving = 0;
	int sensing = 0;
	int brokenPoint = 0;
	unsigned short rawData = 0;
	int printTitle = 0;
	int min = 4096;
	int max = 0;
	int node_upper_limit  = 0;
	int node_lower_limit  = 0;

	if(debug_app_enable == 0){
		TOUCH_LOG("%s limit Test\n", (limitType == UPPER_DATA)?"Upper":"Lower");
	}

	for(sensing = 0; sensing < NUM_SENSE; sensing++)
	{
		for(driving = 0; driving < NUM_DRIVE; driving++)
		{
			rawData = (signed short)((buffer[(NUM_SENSE * driving + sensing) * 2 +  1]<<8) | buffer[(NUM_SENSE * driving + sensing) * 2 ]);

			if( rawData == 0 ) {
				brokenPoint++;

			} else if((limitType == UPPER_DATA)){

				if( rawData >  max){
					max = rawData;
				}
				if( rawData < min){
					min = rawData;
				}

				if(batt_temp > BATT_TEMP_55 ) {
					node_upper_limit = Upper_limit[(NUM_DRIVE * sensing) + driving] + Very_High_temp_margin[(NUM_DRIVE * sensing) + driving];
				} else if(batt_temp > BATT_TEMP_40 ) {
					node_upper_limit = Upper_limit[(NUM_DRIVE * sensing) + driving] + High_temp_margin[(NUM_DRIVE * sensing) + driving];
				} else {
					node_upper_limit = Upper_limit[(NUM_DRIVE * sensing) + driving] + Room_upper_margin[(NUM_DRIVE * sensing) + driving];
				}

				//if(sensing == 0 && driving < 10)
				//	TOUCH_LOG("[%d] Upper limit : %d ( %d )\n", batt_temp, node_upper_limit, Upper_limit[(NUM_DRIVE * sensing) + driving]);

				if((rawData > node_upper_limit)){
					if(brokenPoint < 5){
						if(printTitle == 0){
							printTitle = 1;
							WRITE_RESULT(buf, dataLen, "[RX][TX] : Value\n");
							if(debug_app_enable == 0){
								TOUCH_LOG("[RX]    [TX]    Value\n");
							}
						}
						WRITE_RESULT(buf, dataLen, "[%2d][%2d] : %d\n", sensing + 1, driving + 1, rawData);
						if(debug_app_enable == 0){
							TOUCH_LOG("[%2d][%2d] : %d\n", sensing + 1, driving + 1, rawData);
						}
					}
					brokenPoint++;
				}
			} else {

				if( rawData >  max){
					max = rawData;
				}
				if( rawData < min){
					min = rawData;
				}

				if(batt_temp < BATT_TEMP_N_10 ) {
					node_lower_limit = Lower_limit[(NUM_DRIVE * sensing) + driving] + Very_Low_temp_margin[(NUM_DRIVE * sensing) + driving];
				} else if(batt_temp < BATT_TEMP_10 ) {
					node_lower_limit = Lower_limit[(NUM_DRIVE * sensing) + driving] + Low_temp_margin[(NUM_DRIVE * sensing) + driving];
				} else {
					node_lower_limit = Lower_limit[(NUM_DRIVE * sensing) + driving] + Room_lower_margin[(NUM_DRIVE * sensing) + driving];
				}

				// Debugging
				//if(sensing == 0 && driving < 10)
				//	TOUCH_LOG("[%d] Lower limit : %d ( %d )\n", batt_temp, node_lower_limit, Lower_limit[(NUM_DRIVE * sensing) + driving]);

				if((rawData < node_lower_limit)){
					if(brokenPoint < 5){
						if(printTitle == 0){
							printTitle = 1;
							WRITE_RESULT(buf, dataLen, "[RX][TX] : Value\n");
							if(debug_app_enable == 0){
								TOUCH_LOG("[RX]    [TX]    Value\n");
							}
						}
						WRITE_RESULT(buf, dataLen, "[%2d][%2d] : %d\n", sensing + 1, driving + 1, rawData);
						if(debug_app_enable == 0){
							TOUCH_LOG("[%2d]    [%2d]    %d\n", sensing + 1, driving + 1, rawData);
						}
					}
					brokenPoint++;
				}
			}
		}
	}

	if( brokenPoint == 0 ){
		WRITE_RESULT(buf, dataLen, "%s limit check :  OK \n", (limitType == UPPER_DATA)?"Upper":"Lower");
		TOUCH_LOG("%s limit check :  OK \n", (limitType == UPPER_DATA)?"Upper":"Lower");
	} else {
		*pRawDataStatus = TOUCH_FAIL;
		WRITE_RESULT(buf, dataLen, "%s limit check :  NG (Node : %d)\n", (limitType == UPPER_DATA)?"Upper":"Lower", brokenPoint);
		TOUCH_LOG("%s limit check :  NG (Node : %d)\n", (limitType == UPPER_DATA)?"Upper":"Lower", brokenPoint);
	}

	if(debug_app_enable == 0){
		TOUCH_LOG("    Fail    %4d\n", brokenPoint);
	}

	*pTotalMin = min;
	*pTotalMax = max;

	return dataLen;
}

int checkOpenShortTest(u8* buffer, signed short limitData, signed short limitSideData, char* buf, int dataLen, int* pNgCount, int *pMaxDiff, int *pCheckResult, int batt_temp)
{
	int driving = 0;
	int sensing = 0;
	int openShortRxCount = 0;
	int openShortMax = 0;
	int rawDataRx0 = 0;
	int rawDataRx1 = 0;
	int diffRxNode = 0;

	memset(s_open_short_result, 0, sizeof(s_open_short_result));
	memset(s_open_short_map, 0, sizeof(s_open_short_map));

	// Open Short Map (Diff beteween RX nodes)
	for(sensing = 0; sensing < 17/* 18 - 1 */; sensing++) {
		for(driving = 0; driving < 31; driving++) {
			rawDataRx0 = (signed short)(((buffer[(NUM_SENSE * driving + sensing) * 2 +  1]<<8) | buffer[(NUM_SENSE * driving + sensing) * 2 ])
					-((Lower_limit[(NUM_DRIVE * sensing) + driving] + Upper_limit[(NUM_DRIVE * sensing) + driving])/2));
			rawDataRx1 = (signed short)(((buffer[(NUM_SENSE * driving + sensing+1) * 2 +  1]<<8) | buffer[(NUM_SENSE * driving + sensing+1) * 2 ])
					-((Lower_limit[(NUM_DRIVE * (sensing+1)) + driving] + Upper_limit[(NUM_DRIVE * (sensing+1)) + driving])/2));
			diffRxNode = abs(rawDataRx0 - rawDataRx1);
#if 0
			TOUCH_LOG("00:%d(0x%x)\n", (signed short)(((buffer[(NUM_SENSE * driving + sensing) * 2 +  1]<<8) | buffer[(NUM_SENSE * driving + sensing) * 2 ])));
			TOUCH_LOG("01:%d(0x%x)\n", (signed short)(((buffer[(NUM_SENSE * driving + sensing+1) * 2 +  1]<<8) | buffer[(NUM_SENSE * driving + sensing+1) * 2 ])));
			TOUCH_LOG("LowAve00:ave %d(upper %d lower %d)\n", ((Lower_limit[(NUM_DRIVE * sensing) + driving] + Upper_limit[(NUM_DRIVE * sensing) + driving])/2), Lower_limit[(NUM_DRIVE * sensing) + driving], Upper_limit[(NUM_DRIVE * sensing) + driving]);
			TOUCH_LOG("LowAve01:ave %d(upper %d lower %d)\n", ((Lower_limit[(NUM_DRIVE * (sensing+1)) + driving] + Upper_limit[(NUM_DRIVE * (sensing+1)) + driving])/2), Lower_limit[(NUM_DRIVE * (sensing+1)) + driving], Upper_limit[(NUM_DRIVE * (sensing+1)) + driving]);
			TOUCH_LOG("0: drive: %d, sense:%d = %d\n", driving, sensing, rawDataRx0);
			TOUCH_LOG("1: drive: %d, sense:%d = %d\n", driving, sensing, rawDataRx1);
			TOUCH_LOG("diff: d:%d, s:%d = %d\n", driving, sensing, s_open_short_map[driving][sensing]);
#endif
			s_open_short_map[driving][sensing] = diffRxNode;
		}
	}

	for(sensing = 0; sensing < 17; sensing++){
		for(driving = 0; driving < 30; driving++) {
			if( batt_temp > BATT_TEMP_40 || batt_temp < BATT_TEMP_10) {
				if (s_open_short_map[driving][sensing] > s_OpenShortRxLimit_HighLowTemp[sensing][driving]) {
					openShortRxCount++;
					TOUCH_LOG("OpenShort Big Rx Diff Node %d [d:%d][s:%d] :  %d > %d\n", openShortRxCount, driving+1, sensing+1, s_open_short_map[driving][sensing], s_OpenShortRxLimit[sensing][driving]);
				}
			} else {
				if (s_open_short_map[driving][sensing] > s_OpenShortRxLimit[sensing][driving]) {
					openShortRxCount++;
					TOUCH_LOG("OpenShort Big Rx Diff Node %d [d:%d][s:%d] :  %d > %d\n", openShortRxCount, driving+1, sensing+1, s_open_short_map[driving][sensing], s_OpenShortRxLimit[sensing][driving]);
				}
			}
			if (openShortMax < s_open_short_map[driving][sensing]) {
				openShortMax = s_open_short_map[driving][sensing];
			}
		}
	}

	*pNgCount = openShortRxCount;
	*pMaxDiff = openShortMax;

	if(openShortRxCount > 0){
		*pCheckResult = TOUCH_FAIL;
	} else {
		*pCheckResult = TOUCH_SUCCESS;
	}
	TOUCH_LOG("OpenShort Big Diff Rx Count :  %d Max : %d \n", openShortRxCount, openShortMax);

	return dataLen;
}


int checkOpenShortDiffTxTest(u8* buffer, signed short limitData, signed short limitSideData, char* buf, int dataLen, int* pNgCount, int *pMaxDiff, int *pCheckResult, int batt_temp)
{
	int driving = 0;
	int sensing = 0;
	int rawDataTx0 = 0;
	int rawDataTx1 = 0;
	int diffTxNode = 0;
	int openShortTxCount = 0;
	int openShortMax = 0;

	memset(s_open_short_diff_tx_map, 0, sizeof(s_open_short_diff_tx_map));
	memset(s_open_short_diff_tx_result, 0, sizeof(s_open_short_diff_tx_result));

	// Open Short Map (Diff beteween RX nodes)
	for(sensing = 0; sensing < 18; sensing++) {
		for(driving = 0; driving < 30; driving++) {
			rawDataTx0 = (signed short)(((buffer[(NUM_SENSE * driving + sensing) * 2 +  1]<<8) | buffer[(NUM_SENSE * driving + sensing) * 2 ])
					-((Lower_limit[(NUM_DRIVE * sensing) + driving] + Upper_limit[(NUM_DRIVE * sensing) + driving])/2));
			rawDataTx1 = (signed short)(((buffer[(NUM_SENSE * (driving + 1) + sensing) * 2 +  1]<<8) | buffer[(NUM_SENSE * (driving + 1) + sensing) * 2 ])
					-((Lower_limit[(NUM_DRIVE * sensing) + driving + 1] + Upper_limit[(NUM_DRIVE * sensing) + driving + 1])/2));
			diffTxNode = abs(rawDataTx0 - rawDataTx1);
			s_open_short_diff_tx_map[driving][sensing] = diffTxNode;
#if 0
			//TOUCH_LOG("[%d][%d] %d : (DiffTx %d) \n", driving, sensing, s_open_short_diff_tx_map[driving][sensing], diffTxNode);
			TOUCH_LOG("0: drive: %d, sense:%d = %d\n", driving, sensing, rawDataTx0);
			TOUCH_LOG("1: drive: %d, sense:%d = %d\n", driving, sensing, rawDataTx1);
			TOUCH_LOG("diff: d:%d, s:%d = %d\n", driving, sensing, s_open_short_diff_tx_map[driving][sensing]);
#endif
		}
	}

	for(sensing = 0; sensing < 18; sensing++){
		for(driving = 0; driving < 30; driving++) {
			if(batt_temp > BATT_TEMP_40 || batt_temp < BATT_TEMP_10){
				if (s_open_short_diff_tx_map[driving][sensing] > s_OpenShortTxLimit_HighLowTemp[sensing][driving]) {
					openShortTxCount++;
					TOUCH_LOG("OpenShort Tx High / Low Big Diff Node %d [d:%d][s:%d] :  %d > %d\n", openShortTxCount, driving+1, sensing+1, s_open_short_diff_tx_map[driving][sensing], s_OpenShortTxLimit[sensing][driving]);
				}
			} else {
				if (s_open_short_diff_tx_map[driving][sensing] > s_OpenShortTxLimit[sensing][driving]) {
					openShortTxCount++;
					TOUCH_LOG("OpenShort Tx Big Diff Node %d [d:%d][s:%d] :  %d > %d\n", openShortTxCount, driving+1, sensing+1, s_open_short_diff_tx_map[driving][sensing], s_OpenShortTxLimit[sensing][driving]);
				}
			}
			if (openShortMax < s_open_short_diff_tx_map[driving][sensing]) {
				openShortMax = s_open_short_diff_tx_map[driving][sensing];
			}
		}
	}

	*pNgCount = openShortTxCount;
	*pMaxDiff = openShortMax;

	if(openShortTxCount > 0){
		*pCheckResult = TOUCH_FAIL;
	} else {
		*pCheckResult = TOUCH_SUCCESS;
	}
	TOUCH_LOG("OpenShort Big Diff Tx Count :  %d Max : %d \n", openShortTxCount, openShortMax);

	return dataLen;
}

static int readDiagResult(struct i2c_client *client, char *buf, int *pDataLen, int *pRawDataStatus, int *pRawdataAvg, int * pRawdataMin, int* pRawdataMax,  int* pOpenShortRxMax, int* pOpenShortTxMax, int batt_temp)
{
	/*
	 *  (0,0)L,H (1,0)L,H ....(17,0)L,H = 36byte
	 *  (0,1)L,H (1,1)L,H ....(17,1)L,H = 36byte
	 *  (0,2)L,H (1,2)L,H ....(17,2)L,H = 36byte
	 *  :
	 *  (0,30)L,H (1,30)L,H ....(17,30)L,H = 36byte
	 *  singed Little endian
	 */

	unsigned char ram_addr[3];
	unsigned int vramAddr = 0x0170E8; // the first address of the map
	int index = 0;
	int sense = 0; // for debug
	//int drive = 0;
	int ret = *pDataLen;
	unsigned int senseAverage[NUM_SENSE] = {0,};
	int totalAverage = 0;
	int totalMin = 4000;
	int totalMax = 0;
	int openShortRxCheckResult = TOUCH_FAIL;
	int openShortRxCount = 0;
	int openShortRxMaxValue = 0;
	int openShortTxCheckResult = TOUCH_FAIL;
	int openShortTxCount = 0;
	int openShortTxMaxValue;
	u8* pRawDataBuffer = NULL;
	unsigned int totalValue = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	//TOUCH_FUNC();
	//TOUCH_LOG("SelfDiag readingSize(%d) pDataLen %d\n", readingSize, *pDataLen);

	pRawDataBuffer =  kzalloc(sizeof(u8) * 1536 , GFP_KERNEL);
	if(pRawDataBuffer == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  1536 ) );
		return TOUCH_FAIL;
	}

	if(pRawDataBuffer == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(signed short) *  TOTAL_CHANNAL ) );
		return TOUCH_FAIL;
	}

	memset(pRawDataBuffer, 0x0, (sizeof(u8) * 1536));

	SetBankAddr(&g_ts,SHTSC_BANK_DCMAP);
	/* read 120 bytes from Bank5, address 8 */
	/* read loop required */

	for(index=0;index<NUM_DRIVE;index++){
		ram_addr[0] = (unsigned char)(vramAddr&0xff); // set address to read (Lower)
		ram_addr[1] = (unsigned char)((vramAddr&0xff00)>>8); // set address to read (Middle)
		ram_addr[2] = (unsigned char)((vramAddr&0xff0000)>>16); // set address to read (Higher)
		WriteMultiBytes(&g_ts,(unsigned char)0x06,ram_addr,2);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_SYSTEM_HA);
		WriteOneByte(&g_ts, 0x43, ram_addr[2]);
		WriteOneByte(&g_ts, SHTSC_ADDR_BANK, SHTSC_BANK_DCMAP);

		/* 36 bytes = 36/2=18 elements (signed 16bit little endian) */
		ReadMultiBytes(&g_ts, 0x08, NUM_SENSE*sizeof(s16), &(pRawDataBuffer[index*NUM_SENSE*sizeof(s16)]));
		vramAddr += (NUM_SENSE+3)/4*8;  //40 bytes including 4 bytes padding
		//TOUCH_LOG("vramAddr 0x%X\n", vramAddr);

	}
	/* Panal limit data filtering */
	ret = checkLimitData(pRawDataBuffer, UPPER_DATA, buf, ret, pRawDataStatus, &totalMin, &totalMax, batt_temp);

	ret = checkLimitData(pRawDataBuffer, LOWER_DATA, buf, ret, pRawDataStatus, &totalMin, &totalMax, batt_temp);

	ret = checkOpenShortTest(pRawDataBuffer, 0, 0, buf, ret, &openShortRxCount, &openShortRxMaxValue, &openShortRxCheckResult, batt_temp);
	*pOpenShortRxMax = openShortRxMaxValue;

	ret = checkOpenShortDiffTxTest(pRawDataBuffer, 0, 0, buf, ret, &openShortTxCount, &openShortTxMaxValue, &openShortTxCheckResult, batt_temp);
	*pOpenShortTxMax = openShortTxMaxValue;

	TOUCH_LOG("Raw data Min : %d Max : %d\n", totalMin, totalMax);
	WRITE_RESULT(buf, ret, "Raw Data Min : %d Max : %d\n", totalMin, totalMax);

	WRITE_RESULT(buf, ret, "     [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);

#if 0
	if(debug_app_enable == 0){

		TOUCH_LOG("     [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d] [%2d]\n", \
				1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);
	}

#endif

	for (sense = 0; sense < NUM_SENSE; sense++) {

		WRITE_RESULT(buf, ret, "[%02d]%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d\n",\
				sense+1,
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  0 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  0 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  1 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  1 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  2 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  2 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  3 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  3 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  4 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  4 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  5 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  5 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  6 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  6 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  7 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  7 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  8 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  8 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE *  9 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  9 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 10 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 10 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 11 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 11 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 12 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 12 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 13 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 13 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 14 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 14 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 15 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 15 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 16 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 16 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 17 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 17 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 18 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 18 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 19 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 19 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 20 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 20 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 21 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 21 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 22 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 22 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 23 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 23 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 24 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 24 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 25 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 25 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 26 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 26 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 27 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 27 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 28 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 28 + sense ) * 2 ]),\
				(signed short)((pRawDataBuffer[ (NUM_SENSE * 29 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 29 + sense ) * 2 ]),\

				(signed short)((pRawDataBuffer[ (NUM_SENSE * 30 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 30 + sense ) * 2 ]));

#if 0
		if(debug_app_enable == 0){

			TOUCH_LOG("[%02d]%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d%5d\n",\
					sense+1,
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  0 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  0 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  1 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  1 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  2 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  2 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  3 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  3 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  4 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  4 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  5 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  5 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  6 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  6 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  7 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  7 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  8 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  8 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE *  9 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  9 + sense ) * 2 ]),\

					(signed short)((pRawDataBuffer[ (NUM_SENSE * 10 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 10 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 11 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 11 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 12 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 12 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 13 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 13 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 14 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 14 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 15 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 15 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 16 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 16 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 17 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 17 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 18 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 18 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 19 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 19 + sense ) * 2 ]),\

					(signed short)((pRawDataBuffer[ (NUM_SENSE * 20 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 20 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 21 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 21 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 22 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 22 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 23 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 23 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 24 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 24 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 25 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 25 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 26 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 26 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 27 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 27 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 28 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 28 + sense ) * 2 ]),\
					(signed short)((pRawDataBuffer[ (NUM_SENSE * 29 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 29 + sense ) * 2 ]),\

					(signed short)((pRawDataBuffer[ (NUM_SENSE * 30 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 30 + sense ) * 2 ]));
		}
#endif

	}

	if( pDriverData != NULL && pDriverData->currState == STATE_SELF_DIAGNOSIS) {

		int senseTotal = 0;
		for (sense = 0; sense < NUM_SENSE; sense++) {

			senseTotal = 0;
			senseTotal = \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  0 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  0 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  1 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  1 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  2 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  2 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  3 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  3 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  4 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  4 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  5 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  5 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  6 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  6 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  7 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  7 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  8 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  8 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE *  9 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE *  9 + sense ) * 2 ]) + \

						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 10 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 10 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 11 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 11 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 12 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 12 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 13 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 13 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 14 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 14 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 15 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 15 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 16 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 16 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 17 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 17 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 18 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 18 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 19 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 19 + sense ) * 2 ]) + \

						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 20 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 20 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 21 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 21 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 22 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 22 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 23 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 23 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 24 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 24 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 25 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 25 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 26 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 26 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 27 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 27 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 28 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 28 + sense ) * 2 ]) + \
						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 29 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 29 + sense ) * 2 ]) + \

						 (signed short)((pRawDataBuffer[ (NUM_SENSE * 30 + sense) * 2 +  1]<<8) | pRawDataBuffer[ (NUM_SENSE * 30 + sense ) * 2 ]);
			senseAverage[sense] = senseTotal / NUM_DRIVE;
		}
#if 0
		WRITE_RESULT(buf, ret, "[Sense Average] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
				senseAverage[0], \
				senseAverage[1], \
				senseAverage[2], \
				senseAverage[3], \
				senseAverage[4], \
				senseAverage[5], \
				senseAverage[6], \
				senseAverage[7], \
				senseAverage[8], \
				senseAverage[9], \
				senseAverage[10], \
				senseAverage[11], \
				senseAverage[12], \
				senseAverage[13], \
				senseAverage[14], \
				senseAverage[15], \
				senseAverage[16], \
				senseAverage[17]);
#endif
		totalValue = (senseAverage[0] - 1000) + (senseAverage[1] - 1000) + (senseAverage[2] - 1000) + (senseAverage[3] - 1000) + (senseAverage[4] - 1000);
		totalValue = totalValue + (senseAverage[5] - 1000) + (senseAverage[6] - 1000) + (senseAverage[7] - 1000) + (senseAverage[8] - 1000) + (senseAverage[9] - 1000);
		totalValue = totalValue + (senseAverage[10] - 1000) + (senseAverage[11] - 1000) + (senseAverage[12] - 1000) + (senseAverage[13] - 1000) + (senseAverage[14] - 1000);
		totalValue = totalValue + (senseAverage[15] - 1000) + (senseAverage[16] - 1000) + (senseAverage[17] - 1000);
		totalAverage = totalValue / NUM_SENSE;
		totalAverage = totalAverage + 1000;

		TOUCH_LOG("Total Average %d \n", totalAverage);
		WRITE_RESULT(buf, ret, "Total [ %4d] %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",\
				totalAverage,
				(totalAverage - senseAverage[0]), \
				(totalAverage - senseAverage[1]), \
				(totalAverage - senseAverage[2]), \
				(totalAverage - senseAverage[3]), \
				(totalAverage - senseAverage[4]), \
				(totalAverage - senseAverage[5]), \
				(totalAverage - senseAverage[6]), \
				(totalAverage - senseAverage[7]), \
				(totalAverage - senseAverage[8]), \
				(totalAverage - senseAverage[9]), \
				(totalAverage - senseAverage[10]), \
				(totalAverage - senseAverage[11]), \
				(totalAverage - senseAverage[12]), \
				(totalAverage - senseAverage[13]), \
				(totalAverage - senseAverage[14]), \
				(totalAverage - senseAverage[15]), \
				(totalAverage - senseAverage[16]), \
				(totalAverage - senseAverage[17]));
	}

	*pRawdataAvg = totalAverage;
	*pRawdataMin = totalMin;
	*pRawdataMax = totalMax;

	TOUCH_LOG("OPEN_SHORT Rx(N) - Rx(N-1)\n");
	WRITE_RESULT(buf, ret, "OPEN_SHORT Rx Diff\n");
	if( openShortRxCheckResult == TOUCH_SUCCESS){
		TOUCH_LOG("OpenShort RX : OK  (Max : %d)\n", openShortRxMaxValue);
		WRITE_RESULT(buf, ret, "OpenShort RX : OK (Max : %d)\n", openShortRxMaxValue);
	} else {
		*pRawDataStatus = TOUCH_FAIL;
		TOUCH_LOG("OpenShort RX : NG (Count : %d Max : %d)\n", openShortRxCount, openShortRxMaxValue);
		WRITE_RESULT(buf, ret, "OpenShort RX : NG (Count  : %d Max : %d)\n", openShortRxCount, openShortRxMaxValue);
	}

	TOUCH_LOG("     [%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);
#if 0
	WRITE_RESULT(buf, ret, "     [%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d]\n",\
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);
#endif
	for (sense = 0; sense < NUM_SENSE-1; sense++) {
		TOUCH_LOG("[%02d]%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d\n",\
				sense+1,\
				(signed short)s_open_short_map[0][sense],\
				(signed short)s_open_short_map[1][sense],\
				(signed short)s_open_short_map[2][sense],\
				(signed short)s_open_short_map[3][sense],\
				(signed short)s_open_short_map[4][sense],\
				(signed short)s_open_short_map[5][sense],\
				(signed short)s_open_short_map[6][sense],\
				(signed short)s_open_short_map[7][sense],\
				(signed short)s_open_short_map[8][sense],\
				(signed short)s_open_short_map[9][sense],\
				(signed short)s_open_short_map[10][sense],\
				(signed short)s_open_short_map[11][sense],\
				(signed short)s_open_short_map[12][sense],\
				(signed short)s_open_short_map[13][sense],\
				(signed short)s_open_short_map[14][sense],\
				(signed short)s_open_short_map[15][sense],\
				(signed short)s_open_short_map[16][sense],\
				(signed short)s_open_short_map[17][sense],\
				(signed short)s_open_short_map[18][sense],\
				(signed short)s_open_short_map[19][sense],\
				(signed short)s_open_short_map[20][sense],\
				(signed short)s_open_short_map[21][sense],\
				(signed short)s_open_short_map[22][sense],\
				(signed short)s_open_short_map[23][sense],\
				(signed short)s_open_short_map[24][sense],\
				(signed short)s_open_short_map[25][sense],\
				(signed short)s_open_short_map[26][sense],\
				(signed short)s_open_short_map[27][sense],\
				(signed short)s_open_short_map[28][sense],\
				(signed short)s_open_short_map[29][sense],\
				(signed short)s_open_short_map[30][sense]);
		WRITE_RESULT(buf, ret, "[%02d]%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d\n",\
				sense+1,\
				(signed short)s_open_short_map[0][sense],\
				(signed short)s_open_short_map[1][sense],\
				(signed short)s_open_short_map[2][sense],\
				(signed short)s_open_short_map[3][sense],\
				(signed short)s_open_short_map[4][sense],\
				(signed short)s_open_short_map[5][sense],\
				(signed short)s_open_short_map[6][sense],\
				(signed short)s_open_short_map[7][sense],\
				(signed short)s_open_short_map[8][sense],\
				(signed short)s_open_short_map[9][sense],\
				(signed short)s_open_short_map[10][sense],\
				(signed short)s_open_short_map[11][sense],\
				(signed short)s_open_short_map[12][sense],\
				(signed short)s_open_short_map[13][sense],\
				(signed short)s_open_short_map[14][sense],\
				(signed short)s_open_short_map[15][sense],\
				(signed short)s_open_short_map[16][sense],\
				(signed short)s_open_short_map[17][sense],\
				(signed short)s_open_short_map[18][sense],\
				(signed short)s_open_short_map[19][sense],\
				(signed short)s_open_short_map[20][sense],\
				(signed short)s_open_short_map[21][sense],\
				(signed short)s_open_short_map[22][sense],\
				(signed short)s_open_short_map[23][sense],\
				(signed short)s_open_short_map[24][sense],\
				(signed short)s_open_short_map[25][sense],\
				(signed short)s_open_short_map[26][sense],\
				(signed short)s_open_short_map[27][sense],\
				(signed short)s_open_short_map[28][sense],\
				(signed short)s_open_short_map[29][sense],\
				(signed short)s_open_short_map[30][sense]);
	}

	TOUCH_LOG("OPEN_SHORT Tx(N) - Tx(N-1)\n");
	WRITE_RESULT(buf, ret, "OPEN_SHORT Tx Diff\n");
	if( openShortTxCheckResult == TOUCH_SUCCESS) {
		TOUCH_LOG("OpenShort TX : OK (Max : %d)\n", openShortTxMaxValue);
		WRITE_RESULT(buf, ret, "OpenShort TX : OK (Max : %d)\n", openShortTxMaxValue);
	} else {
		*pRawDataStatus = TOUCH_FAIL;
		TOUCH_LOG("OpenShort TX : NG (Count : %d Max : %d)\n", openShortTxCount, openShortTxMaxValue);
		WRITE_RESULT(buf, ret, "OpenShort TX : NG(Count : %d Max : %d)\n", openShortTxCount, openShortTxMaxValue);
	}

	TOUCH_LOG("     [%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30);
#if 0
	WRITE_RESULT(buf, ret, "     [%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d][%2d]\n", \
			1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30);
#endif

	for (sense = 0; sense < NUM_SENSE; sense++) {
		TOUCH_LOG("[%02d]%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d\n",\
				sense+1,\
				(signed short)s_open_short_diff_tx_map[0][sense],\
				(signed short)s_open_short_diff_tx_map[1][sense],\
				(signed short)s_open_short_diff_tx_map[2][sense],\
				(signed short)s_open_short_diff_tx_map[3][sense],\
				(signed short)s_open_short_diff_tx_map[4][sense],\
				(signed short)s_open_short_diff_tx_map[5][sense],\
				(signed short)s_open_short_diff_tx_map[6][sense],\
				(signed short)s_open_short_diff_tx_map[7][sense],\
				(signed short)s_open_short_diff_tx_map[8][sense],\
				(signed short)s_open_short_diff_tx_map[9][sense],\
				(signed short)s_open_short_diff_tx_map[10][sense],\
				(signed short)s_open_short_diff_tx_map[11][sense],\
				(signed short)s_open_short_diff_tx_map[12][sense],\
				(signed short)s_open_short_diff_tx_map[13][sense],\
				(signed short)s_open_short_diff_tx_map[14][sense],\
				(signed short)s_open_short_diff_tx_map[15][sense],\
				(signed short)s_open_short_diff_tx_map[16][sense],\
				(signed short)s_open_short_diff_tx_map[17][sense],\
				(signed short)s_open_short_diff_tx_map[18][sense],\
				(signed short)s_open_short_diff_tx_map[19][sense],\
				(signed short)s_open_short_diff_tx_map[20][sense],\
				(signed short)s_open_short_diff_tx_map[21][sense],\
				(signed short)s_open_short_diff_tx_map[22][sense],\
				(signed short)s_open_short_diff_tx_map[23][sense],\
				(signed short)s_open_short_diff_tx_map[24][sense],\
				(signed short)s_open_short_diff_tx_map[25][sense],\
				(signed short)s_open_short_diff_tx_map[26][sense],\
				(signed short)s_open_short_diff_tx_map[27][sense],\
				(signed short)s_open_short_diff_tx_map[28][sense],\
				(signed short)s_open_short_diff_tx_map[29][sense]);
		WRITE_RESULT(buf, ret, "[%02d]%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d%4d\n",\
				sense+1,\
				(signed short)s_open_short_diff_tx_map[0][sense],\
				(signed short)s_open_short_diff_tx_map[1][sense],\
				(signed short)s_open_short_diff_tx_map[2][sense],\
				(signed short)s_open_short_diff_tx_map[3][sense],\
				(signed short)s_open_short_diff_tx_map[4][sense],\
				(signed short)s_open_short_diff_tx_map[5][sense],\
				(signed short)s_open_short_diff_tx_map[6][sense],\
				(signed short)s_open_short_diff_tx_map[7][sense],\
				(signed short)s_open_short_diff_tx_map[8][sense],\
				(signed short)s_open_short_diff_tx_map[9][sense],\
				(signed short)s_open_short_diff_tx_map[10][sense],\
				(signed short)s_open_short_diff_tx_map[11][sense],\
				(signed short)s_open_short_diff_tx_map[12][sense],\
				(signed short)s_open_short_diff_tx_map[13][sense],\
				(signed short)s_open_short_diff_tx_map[14][sense],\
				(signed short)s_open_short_diff_tx_map[15][sense],\
				(signed short)s_open_short_diff_tx_map[16][sense],\
				(signed short)s_open_short_diff_tx_map[17][sense],\
				(signed short)s_open_short_diff_tx_map[18][sense],\
				(signed short)s_open_short_diff_tx_map[19][sense],\
				(signed short)s_open_short_diff_tx_map[20][sense],\
				(signed short)s_open_short_diff_tx_map[21][sense],\
				(signed short)s_open_short_diff_tx_map[22][sense],\
				(signed short)s_open_short_diff_tx_map[23][sense],\
				(signed short)s_open_short_diff_tx_map[24][sense],\
				(signed short)s_open_short_diff_tx_map[25][sense],\
				(signed short)s_open_short_diff_tx_map[26][sense],\
				(signed short)s_open_short_diff_tx_map[27][sense],\
				(signed short)s_open_short_diff_tx_map[28][sense],\
				(signed short)s_open_short_diff_tx_map[29][sense]);
	}

	*pDataLen = ret;

	TOUCH_LOG("readDiagResult done ret = %d pDatalen = %d totalAverage %d, Min %d Max %d\n", ret, *pDataLen, totalAverage, totalMin, totalMax);
	TOUCH_LOG("readDiagResult done openShort Rx Count %d Max %d , Tx Count %d Max %d\n", openShortRxCount, openShortRxMaxValue, openShortTxCount, openShortTxMaxValue);

	if(pRawDataBuffer != NULL)
		kfree(pRawDataBuffer);

	return TOUCH_SUCCESS;
}

#define NO_INTERRUPT_FOR_SELFDIAG 1
#if defined ( NO_INTERRUPT_FOR_SELFDIAG)

#define CMD_SELFDIAG "\xDA\x00\x00\x01"
#define CMD_SELFDIAG_LEN 4

int shtsc_CMD_SelfDiag_wo_IRQ(void *ts)
{
	//int ret;
	int count = 0;
	u8 value = 0;

	//TOUCH_FUNC();

	SetBankAddr(ts, SHTSC_BANK_COMMAND);

	// set command
	WriteMultiBytes(ts, SHTSC_ADDR_COMMAND, CMD_SELFDIAG, CMD_SELFDIAG_LEN);

	SetIndicator(ts, SHTSC_IND_CMD);

	msleep(500);

	value = ReadOneByte(ts, SHTSC_ADDR_INT0);
#if defined(DEBUG_SHTSC)
	TOUCH_LOG("Value check value %02XH SHTSC_STATUS_COMMAND_RESULT & value %02XH \n", value, (SHTSC_STATUS_COMMAND_RESULT & value));
#endif

	while (! (SHTSC_STATUS_COMMAND_RESULT & ReadOneByte(ts, SHTSC_ADDR_INT0))) {
		; // waiting the result
#ifdef DEBUG_SHTSC
		TOUCH_LOG("waiting the result count %d\n", count);
#endif
		msleep(5);
		count++;

		if(count > 200){
			TOUCH_LOG("waiting the result count %d. Please check it\n", count);
			break;
		}
	}

#ifdef DEBUG_SHTSC
	TOUCH_LOG("%s : Before BANK_COMMMAND_RESULT BankAddr %d \n", __func__, bankAddr);
#endif

	// Bank address
	SetBankAddr(&g_ts,SHTSC_BANK_COMMAND_RESULT);

	// READ result
	ReadMultiBytes(&g_ts, 0x08, MAX_COMMAND_RESULT_LEN, CommandResultBuf);

	/* Clear Interrupt */
	Sharp_ClearInterrupt(&g_ts, SHTSC_STATUS_COMMAND_RESULT);

	return 0;
}

#endif /* NO_INTERRUPT_FOR_SELFDIAG */



//====================================================================
// Function : LR388K6_DoSelfDiagnosis
// Description
//   - diagnose touch pannel and return result
//   - can use pBuf to give more information ( be careful not to exceed buffer size )
//   - should create a file of result ( TBD : consider file saving on common driver side )
//====================================================================

static int LR388K6_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int* pDataLen)
{
	u8 * pResultData = NULL;
	int resultDataLen = 0;
	int dataLen = 0;
	int ret = 0;
	int calibrationCheckResult = TOUCH_FAIL;
	int calibrationDiffMaxValue = 0;
	int deltaCheckResult = TOUCH_FAIL;
	int jitterCheckResult = TOUCH_FAIL;
	int deltaMaxValue = 0;
	char cmdbin_selfdiag[]={0xDA,0x00,0x00,0x01}; // selfdiag command
	unsigned char unique_id_Buf[UNIQUE_ID_BUFFER_MAX];
	int rawDataMin = 0;
	int rawDataMax = 0;
	int rawDataAvg = 0;
	int openShortRxMax = 0;
	int openShortTxMax = 0;

	TouchDriverData *pDriverData = NULL;

	u8 *time_string = NULL;
	char *filename = NULL;
	int currentBattTemp = 0;

	TOUCH_FUNC();

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
		if(pDriverData == NULL)
			return TOUCH_FAIL;
	}else{
		return TOUCH_FAIL;
	}

	*pDataLen = (int) 0;

	*pRawStatus = (int) TOUCH_FAIL;

	*pChannelStatus = (int) TOUCH_FAIL;


	pResultData =  kzalloc(sizeof(u8) * RESULT_DATA_MAX_LEN , GFP_KERNEL);
	if(pResultData == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  RESULT_DATA_MAX_LEN ) );
		return TOUCH_FAIL;
	}

	WRITE_RESULT(pResultData, resultDataLen, "\n\n");
	// [1] Information
	WRITE_RESULT(pResultData, resultDataLen, "=== Information ===\n");
	WRITE_RESULT(pResultData, resultDataLen, "FW Info       = %d.%02d\n", pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version);
	WRITE_RESULT(pResultData, resultDataLen, "Module Info   = %d\n", pDriverData->icFwInfo.moduleVersion);
	WRITE_RESULT(pResultData, resultDataLen, "Model ID      = %d\n", pDriverData->icFwInfo.modelID);

	//WRITE_RESULT(pResultData, resultDataLen, "========= Additional Information =========\n");
	WRITE_RESULT(pResultData, resultDataLen, "Device name   = LR388K6 , Provider = Sharp \n" );
	WRITE_RESULT(pResultData, resultDataLen, "Product ID    = %d\n", pDriverData->icFwInfo.moduleMakerID);
	WRITE_RESULT(pResultData, resultDataLen, "Firmware Ver  = %08X\n", K6_firmver);
	WRITE_RESULT(pResultData, resultDataLen, "Parameter Ver = %08X\n", K6_paramver);

	// [2] Unique ID
	memset(unique_id_Buf, 0x0, sizeof(unique_id_Buf));
	if(get_unique_id_from_cache(client, unique_id_Buf, sizeof(unique_id_Buf)) == TOUCH_SUCCESS){
		TOUCH_LOG("Unique ID : %s\n", unique_id_Buf);
		WRITE_RESULT(pResultData, resultDataLen, "Unique ID : %s\n", unique_id_Buf);
	} else {
		TOUCH_LOG("Unique ID : Unknown\n");
		WRITE_SYSBUF(pResultData, resultDataLen, "Unique ID : Unknown\n");
	}

	time_string =  kzalloc(sizeof(u8) * TIME_DATA_MAX_LEN, GFP_KERNEL);
	if(time_string == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  TIME_DATA_MAX_LEN ) );
		if(pResultData != NULL)
		{
			kfree(pResultData);
			pResultData = NULL;
		}
		return TOUCH_FAIL;
	}

	memset(time_string, 0x0, (sizeof(u8) * TIME_DATA_MAX_LEN));

	WRITE_RESULT(pResultData, resultDataLen, "=== Environment ===\n");
	// [3] Current Time
	if(TOUCH_FAIL == get_current_time(time_string))
	{
		if(pResultData != NULL)
		{
			kfree(pResultData);
			pResultData = NULL;
		}
		if(time_string != NULL)
		{
			kfree(time_string);
			time_string = NULL;
		}
		TOUCH_ERR("get current time failed!\n");
		return TOUCH_FAIL;
	}

	WRITE_RESULT(pResultData, resultDataLen, "Time : %s\n", time_string);

	// [4] BOOT MODE
	if(pDriverData != NULL ){
		switch(pDriverData->bootMode){
			case BOOT_OFF_CHARGING:
				TOUCH_LOG("BOOT : OFF_CHARGING\n");
				WRITE_RESULT(pResultData, resultDataLen, "BOOT : BOOT OFF CHARGING\n");
				break;
			case BOOT_MINIOS:
				TOUCH_LOG("BOOT : MINIOS\n");
				WRITE_RESULT(pResultData, resultDataLen, "BOOT : MINI OS\n");
				break;
			case BOOT_MFTS:
				TOUCH_LOG("BOOT : MFTS\n");
				WRITE_RESULT(pResultData, resultDataLen, "BOOT : MFTS\n");
				break;
			default:
				TOUCH_LOG("BOOT : NORMAL\n");
				WRITE_RESULT(pResultData, resultDataLen, "BOOT : NORMAL \n");
				break;
		}
		//TOUCH_LOG("BOOT : %d\n", pDriverData->bootMode);
	}

	// [5] TA CONNECTION
	WRITE_RESULT(pResultData, resultDataLen, "TA : %s\n", (ta_connection_status == 1 ? "Connected" : "Disconnected" ));

	TOUCH_LOG("TA : %s\n", (ta_connection_status == 1 ? "Connected" : "Disconnected" ));

	// [6] Battery Temperature
	currentBattTemp = touch_ref_batt_temp;
	WRITE_RESULT(pResultData, resultDataLen, "Temp. : %3d\n", currentBattTemp );
	TOUCH_LOG("Temp : %3d\n", currentBattTemp );


	//  Calbiration
	//shtsc_CMD_ExecCalibration_wo_IRQ(&g_ts);
	//WRITE_RESULT(pResultData, resultDataLen, "\n========== Calibration Done ==========\n");

	// [7] FACTORY CALIBRATION CHECK
	resultDataLen = shtsc_proc_checkcalib_func(client, pResultData, resultDataLen, &calibrationCheckResult, &calibrationDiffMaxValue);
	if(calibrationCheckResult == TOUCH_SUCCESS){
		TOUCH_LOG("Fac. Cal. check : OK\n");
	} else {
		TOUCH_LOG("Fac. Cal.  check : NG\n");
	}

	// [8] DELTA CHECK - For debugging purpose ONLY
	check_delta_spec(client, pResultData, &resultDataLen, get_spec_delta_diff(client), DELTA_READ_REPEAT, &deltaCheckResult, &deltaMaxValue);
	if(deltaCheckResult == TOUCH_SUCCESS){
		TOUCH_LOG("Delta check : OK (Only Debuggging)\n");
	} else {
		TOUCH_LOG("Delta check : NG (Only Debugging)\n");
	}

	// [9] JITTER CHECK
	check_jitter_spec_wo_irq(client, pResultData, &resultDataLen, JITTER_READ_REPEAT, &jitterCheckResult);
	if(jitterCheckResult == TOUCH_SUCCESS){
		TOUCH_LOG("Jitter check : OK\n");
	} else {
		TOUCH_LOG("Jitter check : NG\n");
	}

	// [10] TOUCH IC SELF TESTING
	shtsc_CMD_SelfDiag_wo_IRQ(&g_ts); // reset required after this command, so this should be after delta check

	if ((CommandResultBuf[0x08-0x08] == cmdbin_selfdiag[0]) && // test command
			(CommandResultBuf[0x09-0x08] == 0x00)) { // error code  // now test is done. check ok or not

		if (CommandResultBuf[0x0a-0x08] == 0x00) { // Result
			// test is passed

			*pRawStatus = (int) TOUCH_SUCCESS;

			*pChannelStatus = (int) TOUCH_SUCCESS;

			TOUCH_LOG("SelfDiag command IC Testing PASS\n");

			ret = TOUCH_SUCCESS;

		} else if (CommandResultBuf[0x0a-0x08] == 0x01) {

			// test is not passed
			TOUCH_LOG("SelfDiag command test not passed\n");

			ret =  TOUCH_SUCCESS; // or TOUCH_FAIL? which should I return?

		} else if (CommandResultBuf[0x0a-0x08] == 0xff) {
			// abnormal termination
			TOUCH_LOG("SelfDiag command test abnormal termination\n");
			ret =  TOUCH_FAIL;
		} else {

			TOUCH_LOG("SelfDiag command test something wrong!!!\n");

		}


	} else if (CommandResultBuf[0x09-0x08] != 0x00) {
		// what happend?
		TOUCH_LOG("SelfDiag ErrorCode(%02X)\n", CommandResultBuf[0x09-0x08]);
		ret = TOUCH_FAIL;
	} else {
		TOUCH_LOG("SelfDiag command test result empty!!! Please check here\n");
	}



	if(calibrationCheckResult == TOUCH_FAIL){
		*pRawStatus = TOUCH_FAIL;
	}

	//if(deltaCheckResult == TOUCH_FAIL){
	//	*pRawStatus = TOUCH_FAIL;
	//}

	if(jitterCheckResult == TOUCH_FAIL){
		*pRawStatus = TOUCH_FAIL;
	}

	// [11] RAW DATA CHECK (Open /Short)
	readDiagResult(client, pResultData, &resultDataLen, pRawStatus, &rawDataAvg, &rawDataMin, &rawDataMax, &openShortRxMax, &openShortTxMax, currentBattTemp);

	WRITE_RESULT(pResultData, resultDataLen, "=== Result ===\n" );
	WRITE_RESULT(pResultData, resultDataLen, "Channel Status : %s\n", (*pChannelStatus == TOUCH_SUCCESS) ? "Pass" : "Fail" );
	WRITE_RESULT(pResultData, resultDataLen, "Raw Data : %s\n", (*pRawStatus == TOUCH_SUCCESS) ? "Pass" : "Fail" );
	WRITE_RESULT(pResultData, resultDataLen, "==============\n" );

	// [12] RESULT FILE SAVE
	if(pDriverData->bootMode == BOOT_NORMAL)
		filename = "/sdcard/touch_self_test.txt";
	else if(pDriverData->bootMode == BOOT_MINIOS)
		filename = "/data/touch/touch_self_test.txt";
	else if(pDriverData->bootMode == BOOT_MFTS)
		filename = "/data/touch/touch_self_mfts.txt";
	else{
		filename = "/sdcard/touch_self_test.txt";
	}

	write_result_data_to_file(filename, pResultData, resultDataLen);

	log_file_size_check(filename);

	WRITE_SYSBUF(pBuf, dataLen, "=== Information ===\n");
	WRITE_SYSBUF(pBuf, dataLen, "FW Info         = %d.%02d\n", pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version);
	WRITE_SYSBUF(pBuf, dataLen, "Module Info     = %d\n", pDriverData->icFwInfo.moduleVersion);
	WRITE_SYSBUF(pBuf, dataLen, "Model ID        = %d\n", pDriverData->icFwInfo.modelID);
	WRITE_SYSBUF(pBuf, dataLen, "Device name     = LR388K6 , Provider = Sharp \n" );
	WRITE_SYSBUF(pBuf, dataLen, "Product ID      = %d\n", pDriverData->icFwInfo.moduleMakerID);
	WRITE_SYSBUF(pBuf, dataLen, "Firmware Ver    = %08X\n", K6_firmver);
	WRITE_SYSBUF(pBuf, dataLen, "Parameter Ver   = %08X\n", K6_paramver);
	WRITE_SYSBUF(pBuf, dataLen, "Delta Max       = %d\n", deltaMaxValue);
	WRITE_SYSBUF(pBuf, dataLen, "Factory Cal     = %d\n", calibrationDiffMaxValue);
	WRITE_SYSBUF(pBuf, dataLen, "Min/Avg/Max     = %d / %d / %d\n", rawDataMin, rawDataAvg, rawDataMax);
	WRITE_SYSBUF(pBuf, dataLen, "OpenShort Rx/Tx = %d / %d\n", openShortRxMax, openShortTxMax);
	WRITE_SYSBUF(pBuf, dataLen, "Unique ID      = %s\n", unique_id_Buf);

	TOUCH_LOG(" PAGE_SIZE  %ld resultDataLen %d\n", PAGE_SIZE, resultDataLen);

	*pDataLen = (int) dataLen;

	if(pResultData != NULL){
		kfree(pResultData);
		pResultData = NULL;
	}

	if(time_string != NULL)
	{
		kfree(time_string);
		time_string = NULL;
	}

	if(debug_app_enable == 1){
		debug_app_enable = 0;
	}

	return ret;
}

static int LR388K6_DoSelfDiagnosis_Lpwg(struct i2c_client *client, int *lpwgStatus, char *pBuf, int *pDataLen)
{
	u8 * pResultData = NULL;
	int resultDataLen = 0;
	int dataLen = *pDataLen;
	int lpwg_rawdata_max = 0;
	int lpwg_jitter_max = 0;
	TouchDriverData *pDriverData = NULL;
	char *filename = NULL;

	TOUCH_FUNC();

	pResultData =  kzalloc(sizeof(u8) * RESULT_DATA_MAX_LEN , GFP_KERNEL);
	if(pResultData == NULL){
		TOUCH_ERR("kzalloc failed size %d \n", ( sizeof(u8) *  RESULT_DATA_MAX_LEN ) );
		return TOUCH_FAIL;
	}

	WRITE_RESULT(pResultData, resultDataLen, "\n\n");
	WRITE_RESULT(pResultData, resultDataLen, "======== LPWG Test ========\n");

	if(client != NULL){
		pDriverData = i2c_get_clientdata(client);
	}

	// LPWG Delta implementaiton
	resultDataLen = shtsc_CMD_getDCMap_LPWG_Rawdata_Jitter(client, &g_ts, DC_CALIBED_DATA, JITTER_READ_REPEAT, pResultData, resultDataLen, &lpwg_rawdata_max, &lpwg_jitter_max);


	*lpwgStatus = TOUCH_SUCCESS;

	if (lpwg_rawdata_max <  SPEC_LPWG_RAWDATA_MAX) {
		WRITE_RESULT(pResultData, resultDataLen, "LPWG Rawdata : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
		TOUCH_LOG("LPWG Rawdata : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
	} else {
		WRITE_RESULT(pResultData, resultDataLen, "LPWG Rawdata : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
		TOUCH_LOG("LPWG Rawdata : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_RAWDATA_MAX, lpwg_rawdata_max);
		*lpwgStatus = TOUCH_FAIL;
	}

	if (lpwg_jitter_max <  SPEC_LPWG_JITTER_MAX) {
		WRITE_RESULT(pResultData, resultDataLen, "LPWG Jitter  : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
		TOUCH_LOG("LPWG Jitter  : OK ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
	} else {
		WRITE_RESULT(pResultData, resultDataLen, "LPWG Jitter : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
		TOUCH_LOG("LPWG Jitter : NG ( Spec : %d , Max : %d )\n", SPEC_LPWG_JITTER_MAX, lpwg_jitter_max);
		*lpwgStatus = TOUCH_FAIL;
	}

	// [] RESULT FILE SAVE
	if(pDriverData->bootMode == BOOT_NORMAL)
		filename = "/sdcard/touch_self_test.txt";
	else if(pDriverData->bootMode == BOOT_MINIOS)
		filename = "/data/touch/touch_self_test.txt";
	else if(pDriverData->bootMode == BOOT_MFTS)
		filename = "/data/touch/touch_self_mfts.txt";
	else{
		filename = "/sdcard/touch_self_test.txt";
	}

	write_result_data_to_file(filename, pResultData, resultDataLen);

	log_file_size_check(filename);

	*pDataLen = dataLen;

	if(pResultData != NULL){
		kfree(pResultData);
		pResultData = NULL;
	}

	return TOUCH_SUCCESS;
}

// Function : LR388K6_AccessRegister
// Description
//   - read from or write to touch IC
//====================================================================
static int LR388K6_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;

	TOUCH_FUNC();

	switch( cmd )
	{
		case READ_IC_REG:
			ret = Touch_I2C_Read_Byte(client, (u8)reg, (u8 *)pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Touch_I2C_Write_Byte(client, (u8)reg, (u8)*pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;

}

static void LR388K6_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
{
	int updateStatus = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	switch (notify) {
		case NOTIFY_CALL:
			TOUCH_LOG("Call was notified ( data = %d )\n", data);
			break;

		case NOTIFY_Q_COVER:
			TOUCH_LOG("Quick Cover was notified ( data = %d )\n", data);
			if(q_cover_status != data){
				updateStatus = 1;
			}
			q_cover_status = data; // covered: 1, not covered: 0
#if  1 // Not ready
			// LCD ON
			if (pDriverData != NULL && pDriverData->currState == STATE_NORMAL && updateStatus == 1){

				// Folio cover closed
				if ( q_cover_status == 1 ) {

					int xs = 920; // 895;
					int xe = 1060; // 1079; //1049;
					int ys = 0;
					int ye = 1919; //1866;
					// xs, ys, xe, ye should be given from the upper layer or fixed values
					char areaSetBuf[CMD_TOUCH_ENABLE_CONTROL_LENGTH];

					TOUCH_LOG("Folio cover closed. active area changed to limited area (%d, %d)-(%d, %d)\n", xs, ys, xe, ye);

					memcpy(areaSetBuf, CMD_TOUCH_OFF, (CMD_TOUCH_ENABLE_CONTROL_LENGTH-8));
					//
					// (xs,ys): left-upper touch enabled rectangle corner
					// (xe,ye): right-lower touch enabled rectangle corner
					// xs(2bytes), ys(2bytes), xe(2bytes), ye(2bytes) (little endian)
					//
					areaSetBuf[5] = (xs&0xff); // left-upper x lower byte
					areaSetBuf[6] = ((xs&0xff00)>>8); // left-upper x higher byte
					areaSetBuf[7] = (ys&0xff); // left-upper y lower byte
					areaSetBuf[8] = ((ys&0xff00)>>8); // left-upper y higher byte
					areaSetBuf[9] = (xe&0xff); // right-lower x lower byte
					areaSetBuf[10] = ((xe&0xff00)>>8); // right-lower x higher byte
					areaSetBuf[11] = (ye&0xff); // right-lower y lower byte
					areaSetBuf[12] = ((ye&0xff00)>>8); // right-lower y higher byte
					issue_command(&g_ts, areaSetBuf, CMD_TOUCH_ENABLE_CONTROL_LENGTH); // use _wo_IRQ if IRQ-disable context

					// now LCD is ON (based on the information of upper "if"
					TOUCH_LOG("Going to COVER mode\n");
					issue_command(&g_ts, CMD_SETSYSTEMSTATE_COVER, CMD_SETSYSTEMSTATE_COVER_LEN); // use _wo_IRQ if IRQ-disable context
				} else {
					if(wq_folio_cover_check != NULL ){
						TOUCH_LOG("canceled folio_cover work\n");
						cancel_delayed_work_sync(&work_folio_cover_check);
					}
					TOUCH_LOG("Going to IDLE mode\n");
					issue_command(&g_ts, CMD_SETSYSTEMSTATE_IDLE, CMD_SETSYSTEMSTATE_IDLE_LEN); // use _wo_IRQ if IRQ-disable context
					TOUCH_LOG("Folio cover open. active area changed to full screen\n");
					issue_command(&g_ts, CMD_TOUCH_ON,  CMD_TOUCH_ENABLE_CONTROL_LENGTH); // use _wo_IRQ if IRQ-disable context
				}
			}
#endif
			break;

		case NOTIFY_FPS_CHANGED:
			TOUCH_LOG("FPS change  was notified ( data = %d )\n", data);
			break;

		case NOTIFY_TA_STATUS:
			TOUCH_LOG("TA status  was notified ( data = %s )\n", ( data == 1 ? "Connected" : "Disconnected"));
			ta_connection_status = data;
			break;

		default:
			TOUCH_ERR("Invalid notification ( notify = %d )\n", notify);
			break;
	}
	return;
}


TouchDeviceSpecificFunction LR388K6_Func = {

	.Initialize = LR388K6_Initialize,
	.Reset = LR388K6_Reset,
	.Connect = LR388K6_Connect,
	.InitRegister = LR388K6_InitRegister,
	.ClearInterrupt = LR388K6_ClearInterrupt,
	.InterruptHandler = LR388K6_InterruptHandler,
	.ReadIcFirmwareInfo = LR388K6_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = LR388K6_GetBinFirmwareInfo,
	.UpdateFirmware = LR388K6_UpdateFirmware,
	.SetLpwgMode = LR388K6_SetLpwgMode,
	.DoSelfDiagnosis = LR388K6_DoSelfDiagnosis,
	.DoSelfDiagnosis_Lpwg = LR388K6_DoSelfDiagnosis_Lpwg,
	.AccessRegister = LR388K6_AccessRegister,
	.NotifyHandler = LR388K6_NotifyHandler,
	.MftsControl = LR388K6_MftsControl,
	.device_attribute_list = LR388K6_attribute_list,

};

/* End Of File */
