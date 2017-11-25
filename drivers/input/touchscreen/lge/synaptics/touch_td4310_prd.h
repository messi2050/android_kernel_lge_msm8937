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

#include "touch_td4310.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

#define ROW_SIZE			30
#define COL_SIZE			18
#define MUX_SIZE			9
#define LOG_BUF_SIZE		4096	/* 4x1024 */
#define BUF_SIZE			(PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024)	/* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define REPORT_DATA_LEN		(ROW_SIZE*COL_SIZE*2)

#define REPORT_TYPE_DELTA			94	/* 16-bit normalized image */
#define REPORT_TYPE_RAW_DATA			92	/* raw capacitance (pF) */
#define REPORT_TYPE_E2E_SHORT			95	/* raw capacitance delta */
#define REPORT_TYPE_P2P_NOISE			94	/* raw capacitance delta */

#define SET_LIMIT(COND, A, B)	(COND ? A##B : A)

// Parameters added as per LGD Electrode Open Test
#define ELECTRODE_OPEN_INTDUR1	4 	//Phase 1 image integration duration
#define ELECTRODE_OPEN_INTDUR2	15	//Phase 2 image integration duration
#define ELECTRODE_OPEN_TXON		2
#define ELECTRODE_OPEN_RXON		2
#define REFERRENCE_LOW_CAP		0x0f
#define REFERRENCE_HIGH_CAP		0x0f
#define RECEIVER_FEEDBACK_CAP	0x0f
#define RESET_DURATION			0x5c

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

enum {
	RAW_DATA_TEST = 0,
	P2P_NOISE_TEST,
	E2E_SHORT_TEST,
	ELECTRODE_OPEN_TEST,
	LPWG_RAW_DATA_TEST,
	LPWG_P2P_NOISE_TEST,
	DELTA_SHOW,
};

enum {
	TEST_PASS = 0,
	TEST_FAIL,
};

/* Normal Mode SET Spec as per LGD - (td4310) */
enum normal_spec {
	RAW_DATA_MAX 		= 2900,
	RAW_DATA_MIN 		= 1900,
	RAW_DATA_MARGIN 	= 0,
	P2P_NOISE_MAX 		= 50,
	P2P_NOISE_MIN 		= 0,

	NOISE_TEST_FRM		= 50,
	E2E_SHORT_MAX 		= 110,
	E2E_SHORT_MIN 		= 0,
	ELECTRODE_OPEN_MAX	= 500,
	ELECTRODE_OPEN_MIN	= 50,

	LPWG_RAW_DATA_MAX	= 2900,
	LPWG_RAW_DATA_MIN	= 1900,
	LPWG_P2P_NOISE_MAX	= 50,
	LPWG_P2P_NOISE_MIN	= 0,
};
/* MFTS Mode SET Spec as per LGD - (td4310) */
enum mfts_spec {
	RAW_DATA_MAX_MFTS 	= 2900,
	RAW_DATA_MIN_MFTS 	= 1900,
	RAW_DATA_MARGIN_MFTS 	= 0,
	P2P_NOISE_MAX_MFTS 	= 50,
	P2P_NOISE_MIN_MFTS 	= 0,

	NOISE_TEST_FRM_MFTS	= 50,
	E2E_SHORT_MAX_MFTS 	= 110,
	E2E_SHORT_MIN_MFTS 	= 0,
	ELECTRODE_OPEN_MAX_MFTS	= 500,
	ELECTRODE_OPEN_MIN_MFTS	= 50,

	LPWG_RAW_DATA_MAX_MFTS	= 2900,
	LPWG_RAW_DATA_MIN_MFTS	= 1900,
	LPWG_P2P_NOISE_MAX_MFTS	= 50,
	LPWG_P2P_NOISE_MIN_MFTS	= 0,
};

extern void touch_msleep(unsigned int msecs);
int td4310_prd_register_sysfs(struct device *dev);

#endif


