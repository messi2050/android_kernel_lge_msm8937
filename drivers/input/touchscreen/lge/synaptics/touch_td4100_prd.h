/* production_test.h
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

#include <touch_core.h>

#include "touch_td4100.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

#define ROW_SIZE			32
#define COL_SIZE			18
#define MUX_SIZE			9
#define LOG_BUF_SIZE		4096	/* 4x1024 */
#define BUF_SIZE			(PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024)	/* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define REPORT_DATA_LEN		(ROW_SIZE*COL_SIZE*2)

/* Production Report Type - TBD(TD4100)*/
#ifdef CONFIG_LGE_TOUCH_TOVIS_SYNAPTICS_TD4100
#define REPORT_TYPE_DELTA			02	/* 16-bit normalized image */
#define REPORT_TYPE_RAW_DATA			03	/* raw capacitance (pF) */
#define REPORT_TYPE_E2E_SHORT			95	/* raw capacitance delta */
#define REPORT_TYPE_P2P_NOISE			02	/* raw capacitance delta */
#else
#define REPORT_TYPE_DELTA			94	/* 16-bit normalized image */
#define REPORT_TYPE_RAW_DATA			92	/* raw capacitance (pF) */
#define REPORT_TYPE_E2E_SHORT			95	/* raw capacitance delta */
#define REPORT_TYPE_P2P_NOISE			94	/* raw capacitance delta */
#endif

/* Normal Mode SET Spec - (TD4100) */
#ifdef CONFIG_LGE_TOUCH_TOVIS_SYNAPTICS_TD4100
#define RAW_DATA_MAX 			3850
#define RAW_DATA_MIN 			550
#define RAW_DATA_MARGIN 			0
#define P2P_NOISE_MAX 			50
#define P2P_NOISE_MIN 			0
#else /*LGD+TD4100(SF3)*/
#define RAW_DATA_MAX 			2950
#define RAW_DATA_MIN 			1100
#define RAW_DATA_MARGIN 			0
#define P2P_NOISE_MAX 			50
#define P2P_NOISE_MIN 			0
#endif
#define NOISE_TEST_FRM			50
#define E2E_SHORT_MAX 			55	/* Upper limit for Image1 */
#define E2E_SHORT_MIN 			90	/* Lower Limit for Image2 */
#define AMP_SHORT_RESULT			0
#define AMP_OPEN_MAX			175
#define AMP_OPEN_MIN			45


/* LPWG Mode SET Spec - (TD4100) */
#ifdef CONFIG_LGE_TOUCH_TOVIS_SYNAPTICS_TD4100
#define LPWG_RAW_DATA_MAX		3950
#define LPWG_RAW_DATA_MIN		500
#define LPWG_P2P_NOISE_MAX		50
#define LPWG_P2P_NOISE_MIN		0
#define LPWG_DOZE_DATA_MAX		35
#else
#define LPWG_RAW_DATA_MAX		2950
#define LPWG_RAW_DATA_MIN		1100
#define LPWG_P2P_NOISE_MAX		40
#define LPWG_P2P_NOISE_MIN		0
#define LPWG_DOZE_DATA_MAX		35
#endif

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

enum {
	RAW_DATA_TEST = 0,
	P2P_NOISE_TEST,
	E2E_SHORT_TEST,
	AMP_OPEN_TEST,
	LPWG_RAW_DATA_TEST,
	LPWG_P2P_NOISE_TEST,
	LPWG_DOZE_DATA_TEST,
	DELTA_SHOW,
};

enum {
	TEST_PASS = 0,
	TEST_FAIL,
};

extern void touch_msleep(unsigned int msecs);
int td4100_prd_register_sysfs(struct device *dev);

#endif


