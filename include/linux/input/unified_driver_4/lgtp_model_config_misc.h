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
 *    File  : lgtp_model_config_misc.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_MODEL_CONFIG_MISC_H_)
#define _LGTP_MODEL_CONFIG_MISC_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/

/* Hardware(Board) Configuration */
#if defined(TOUCH_MODEL_Y30)

#define TOUCH_GPIO_RESET 			0
#define TOUCH_GPIO_INTERRUPT 		1
#define TOUCH_GPIO_MAKER_ID 		76

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#elif defined(TOUCH_MODEL_C30)

#define TOUCH_GPIO_RESET 			(12+902)
#define TOUCH_GPIO_INTERRUPT 		(13+902)
#define TOUCH_GPIO_MAKER_ID 		(32+902)
#define TOUCH_GPIO_POWER 			(9+902)

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#elif defined(TOUCH_MODEL_C70)

#define TOUCH_GPIO_RESET 			(12+902)
#define TOUCH_GPIO_INTERRUPT 		(13+902)

#define TOUCH_IRQ_FLAGS (IRQF_ONESHOT)

#elif defined(TOUCH_MODEL_YG)

#define TOUCH_GPIO_RESET			(12+902)
#define TOUCH_GPIO_INTERRUPT		(13+902)

#define TOUCH_IRQ_FLAGS (IRQF_ONESHOT)

#elif defined(TOUCH_MODEL_C90NAS)

#define TOUCH_GPIO_RESET            (12+902)
#define TOUCH_GPIO_INTERRUPT        (13+902)

#define TOUCH_IRQ_FLAGS (IRQF_ONESHOT)

#elif defined(TOUCH_MODEL_M2) || defined(TOUCH_MODEL_PH1)

#define TOUCH_GPIO_RESET            (12+902)
#define TOUCH_GPIO_INTERRUPT        (13+902)

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#define USE_EARLY_FB_EVENT_BLANK
#define USE_FB_EVENT_UNBLANK

#elif defined(TOUCH_MODEL_PH2)

#define TOUCH_GPIO_RESET            (64) //+890)
#define TOUCH_GPIO_INTERRUPT        (65) //+890)

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#define USE_EARLY_FB_EVENT_BLANK
#define USE_FB_EVENT_UNBLANK

#elif defined(TOUCH_MODEL_Y90)

#elif defined(TOUCH_MODEL_Y70)

#define TOUCH_I2C_SLAVE_ADDR 0x20

#elif defined(TOUCH_MODEL_Y50)

#elif defined(TOUCH_MODEL_C90)

#elif defined(TOUCH_MODEL_C50)

#define TOUCH_GPIO_RESET 			(12+902)
#define TOUCH_GPIO_INTERRUPT 		(13+902)
#define TOUCH_GPIO_POWER 			(9+902)

#define TOUCH_IRQ_FLAGS (IRQF_ONESHOT)

#elif defined(TOUCH_MODEL_LION_3G)

#elif defined(TOUCH_MODEL_P1B)

#define TOUCH_GPIO_RESET			(12+902)
#define TOUCH_GPIO_INTERRUPT		(13+902)

#define TOUCH_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND)

#else
#error "Model should be defined"
#endif

/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/
void TouchVddPowerModel(int isOn);
void TouchVioPowerModel(int isOn);
void TouchAssertResetModel(void);
TouchDeviceControlFunction * TouchGetDeviceControlFunction(int index);
void TouchGetModelConfig(TouchDriverData *pDriverData);


#endif /* _LGTP_MODEL_CONFIG_MISC_H_ */

/* End Of File */

