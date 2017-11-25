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
 *    File  : lgtp_project_setting.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined(_LGTP_PROJECT_SETTING_H_)
#define _LGTP_PROJECT_SETTING_H_


/****************************************************************************
* Project Setting ( Model )
****************************************************************************/
#if defined(CONFIG_TOUCHSCREEN_LU201X)
#define TOUCH_MODEL_Y30
#endif

#if defined(CONFIG_TOUCHSCREEN_LU202X)
#define TOUCH_MODEL_C30
#endif

#if defined(CONFIG_TOUCHSCREEN_MSM8916_C70)
#define TOUCH_MODEL_C70
#endif

#if defined(CONFIG_MSM8916_C90)
#define TOUCH_MODEL_C90NAS
#endif

#if defined(CONFIG_MSM8916_P1C)
#define TOUCH_MODEL_P1C
#endif

#if defined(TARGET_MT6582_Y90)
#define TOUCH_MODEL_Y90
#endif

#if defined(TARGET_MT6582_Y70)
#define TOUCH_MODEL_Y70
#endif

#if defined(TARGET_MT6582_Y50)
#define TOUCH_MODEL_Y50
#endif

#if defined(TARGET_MT6732_C90)
#define TOUCH_MODEL_C90
#endif

#if defined(CONFIG_TOUCHSCREEN_LGE_MELFAS)
#define TOUCH_MODEL_C50
#endif

#if defined(TARGET_MT6582_LION_3G)
#define TOUCH_MODEL_LION_3G
#endif

#if defined(CONFIG_TOUCHSCREEN_MSM8939_P1B)
#define TOUCH_MODEL_P1B
#endif

#if defined(CONFIG_TOUCHSCREEN_MSM8916_YG)
#define TOUCH_MODEL_YG
#endif

#if defined(CONFIG_TOUCHSCREEN_MSM8916_C100N)
#define TOUCH_MODEL_C100N
#endif

#if defined(CONFIG_TOUCHSCREEN_MSM8952_P1V)
#define TOUCH_MODEL_P1V
#endif

#if defined(CONFIG_TOUCHSCREEN_MSM8937_PH2)
#define TOUCH_MODEL_PH2
#endif

/****************************************************************************
* Available Feature supported by Unified Driver
* If you want to use it, define it inside of model feature
****************************************************************************/
/* #define ENABLE_HOVER_DETECTION */
/* #define ENABLE_TOUCH_AT_OFF_CHARGING */

/****************************************************************************
* Project Setting ( AP Solution / AP Chipset / Touch Device )
****************************************************************************/
#if defined(TOUCH_MODEL_Y30)

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8210

/* Touch Device */
#define TOUCH_DEVICE_LU201X
#define TOUCH_DEVICE_LU202X
#define TOUCH_DEVICE_FT6X36
#define TOUCH_DEVICE_DUMMY

/* Driver Feature */
#define ENABLE_HOVER_DETECTION

/* IC Type */
#define TOUCH_TYPE_ONCELL
#elif defined(TOUCH_MODEL_C30)

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_LU202X
/* #define TOUCH_DEVICE_DUMMY */

#elif defined(TOUCH_MODEL_C70)

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_S3320

/* Swipe mode */
#define ENABLE_SWIPE_MODE

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

#elif defined(TOUCH_MODEL_YG) || defined (TOUCH_MODEL_C100N)

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_S3320

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

/* Swipe mode */
#define ENABLE_SWIPE_MODE

#elif defined(TOUCH_MODEL_C90NAS)

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_S3320

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

/* Swipe mode */
#define ENABLE_SWIPE_MODE

#elif defined(TOUCH_MODEL_P1C)

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_S3320

/* Driver Feature */
#define ENABLE_TOUCH_AT_OFF_CHARGING

/* Swipe mode */
#define ENABLE_SWIPE_MODE

#elif defined(TOUCH_MODEL_Y90)

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6582

/* Touch Device */
#define TOUCH_DEVICE_S3320

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

#elif defined(TOUCH_MODEL_Y70)

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6582

/* Touch Device */
#define TOUCH_DEVICE_S3320

/* Swipe mode */
#define ENABLE_SWIPE_MODE

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

#elif defined(TOUCH_MODEL_Y50)

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6582

/* Touch Device */
#define TOUCH_DEVICE_MIT200

/*LGD In-cell*/
#define TOUCH_TYPE_INCELL

#elif defined(TOUCH_MODEL_C90)

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6732

/* Touch Device */
#define TOUCH_DEVICE_S3320

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

#elif defined(TOUCH_MODEL_C50)

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_MIT200

/*LGD In-cell*/
#define TOUCH_LGD_PHASE2

/* Swipe mode */
#define ENABLE_SWIPE_MODE

#elif defined(TOUCH_MODEL_LION_3G)

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6582

/* Touch Device */
#define TOUCH_DEVICE_TD4191

#elif defined(TOUCH_MODEL_P1B)
/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8939

/* Touch Device */
#define TOUCH_DEVICE_S3320

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

/* Swipe mode */
#define ENABLE_SWIPE_MODE

/* Notify for call sate */
#define ENABLE_NOTIFY_CALL_STATE

#elif defined(TOUCH_MODEL_P1V)
/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8952

/* Touch Device */
#define TOUCH_DEVICE_LR388K5

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

/* Swipe mode */
#define ENABLE_SWIPE_MODE

/* Notify for call sate */
#define ENABLE_NOTIFY_CALL_STATE

#elif defined(TOUCH_MODEL_PH2)
/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8937

/* Touch Device */
#define TOUCH_DEVICE_LR388K6

/*JDI In-cell*/
#define TOUCH_TYPE_INCELL

/* Swipe mode */
#define ENABLE_SWIPE_MODE

/* Notify for call sate */
#define ENABLE_NOTIFY_CALL_STATE

#else
#error "Model should be defined"
#endif

#endif /* _LGTP_PROJECT_SETTING_H_ */

/* End Of File */

