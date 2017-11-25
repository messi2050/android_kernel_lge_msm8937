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
 *    File  	: lgtp_device_lr388k6.h
 *    Author(s)   : 
 *    Description : 
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_LR388K6_H_ )
#define _LGTP_DEVICE_LR388K6_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/

#include <linux/input/unified_driver_3/lgtp_common.h>

/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/

enum TCI_CTRL {
	REPORT_MODE_CTRL	 = 0,
	REPORT_RATE_CTRL,
	SENSITIVITY_CTRL,

	ACTIVE_AREA_X1_CTRL,
	ACTIVE_AREA_Y1_CTRL,
	ACTIVE_AREA_X2_CTRL,
	ACTIVE_AREA_Y2_CTRL,

	TCI_ENABLE_CTRL,
	TOUCH_SLOP_CTRL,
	TAP_DISTANCE_CTRL,
	MIN_INTERTAP_CTRL,
	MAX_INTERTAP_CTRL,
	TAP_COUNT_CTRL,
	INTERRUPT_DELAY_CTRL,
	FAILURE_INT_ENABLE,

	TCI_ENABLE_CTRL2,
	TAP_COUNT_CTRL2,
	MIN_INTERTAP_CTRL2,
	MAX_INTERTAP_CTRL2,
	TOUCH_SLOP_CTRL2,
	TAP_DISTANCE_CTRL2,
	INTERRUPT_DELAY_CTRL2,
	FAILURE_INT_ENABLE2,
};

enum {
    FOLIOCOVER_OPEN = 0,
    FOLIOCOVER_CLOSED,
};
#define REPORT_RATE_CTRL_REG		0x08
#define SENSITIVITY_CTRL_REG		0x0A

#define ACTIVE_AREA_X1_CTRL_REG		0x0C
#define ACTIVE_AREA_Y1_CTRL_REG		0x0E
#define ACTIVE_AREA_X2_CTRL_REG		0x10
#define ACTIVE_AREA_Y2_CTRL_REG		0x12

#define TOUCH_SLOP_CTRL_REG			0x14
#define TAP_DISTANCE_CTRL_REG		0x16
#define MIN_INTERTAP_CTRL_REG		0x18
#define MAX_INTERTAP_CTRL_REG		0x1A
#define TAP_COUNT_CTRL_REG			0x1C
#define INTERRUPT_DELAY_CTRL_REG	0x1E

#define TOUCH_SLOP_CTRL2_REG		0x20
#define TAP_DISTANCE_CTRL2_REG		0x22
#define MIN_INTERTAP_CTRL2_REG		0x24
#define MAX_INTERTAP_CTRL2_REG		0x26
#define TAP_COUNT_CTRL2_REG			0x28
#define INTERRUPT_DELAY_CTRL2_REG	0x2A

#define FAILURE_REASON_REG			0x30
#define FAILURE_INT_ENABLE_REG		0x31
#define FAILURE_INT_ENABLE2_REG		0x33
#define FAILURE_INT_STATUS_REG		0x35
#define FAILURE_INT_STATUS2_REG		0x37


/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/



#endif /* _LGTP_DEVICE_LR388K6_H_ */

/* End Of File */

