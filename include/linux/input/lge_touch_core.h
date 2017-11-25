/* include/linux/input/lge_touch_core.h
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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

#ifndef LGE_TOUCH_CORE_H
#define LGE_TOUCH_CORE_H

#include <linux/wakelock.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

/*
 *#define LGE_TOUCH_TIME_DEBUG
 */

#define MAX_FINGER		10
#define MAX_BUTTON		4
#define MAX_POINT_SIZE_FOR_LPWG	256

struct bouncing_filter_role {
	u32	enable;
};

struct grip_filter_role {
	u32	enable;
	u32	edge_region;
	u32	max_delta;
	u32	width_ratio;
};

struct accuracy_filter_role {
	u32	enable;
	u32	min_delta;
	u32	curr_ratio;
	u32	min_pressure;
};

struct jitter_filter_role {
	u32	enable;
	u32	curr_ratio;
	u32	f_ratio;
};

struct anticipation_filter_role {
	u32	enable;
	u32	v_ratio;
	u32	v_scale;
};

struct touch_device_caps {
	u32	button_support;
	u32	number_of_button;
	u32	button_name[MAX_BUTTON];
	u32	max_x;
	u32	max_y;
	u32	max_pressure;
	u32	max_width;
	u32	max_orientation;
	u32	max_id;
};

struct touch_operation_role {
	u32	protocol_type;
	u32	report_mode;
	u32	delta_pos_threshold;
	u32	booting_delay;
	u32	reset_delay;
	u32	wake_up_by_touch;
	u32	use_sleep_mode;
	unsigned long	irqflags;
	struct bouncing_filter_role	*bouncing_filter;
	struct grip_filter_role		*grip_filter;
	struct accuracy_filter_role	*accuracy_filter;
	struct jitter_filter_role	*jitter_filter;
	struct anticipation_filter_role	*anticipation_filter;
};

struct touch_power_module {
	u32	use_regulator;
	char	vdd[30];
	u32	vdd_voltage;
	char	vio[30];
	u32	vio_voltage;
	int	(*power)	(int on);
};

struct touch_firmware_module {
	char	fw_image[256];
	u8	need_upgrade;
};

struct touch_platform_data {
	u32	int_pin;
	u32	reset_pin;
	struct touch_device_caps	*caps;
	struct touch_operation_role	*role;
	struct touch_power_module	*pwr;
	struct touch_firmware_module	*fw;
};

struct t_data {
	u16	id;
	u16	x;
	u16	y;
	u16	raw_x;	/* Do not change it. (in filter function) */
	u16	raw_y;	/* Do not change it. (in filter function) */
	u16	width_major;
	u16	width_minor;
	u16	orientation;
	u16	pressure; /* 0=Hover / 1~MAX-1=Finger / MAX=Palm */
	u16	type; /* finger, palm, pen, glove, hover */
};

struct b_data {
	u16	key_code;
	u16	state;
};

struct touch_data {
	u8		total_num;
	u32		id_mask;
	u32		report_id_mask;
	struct t_data	abs_data[MAX_FINGER];
	struct b_data	button_data;
};

struct touch_fw_info {
	char	fw_ic[32];
	char	fw_bin[32];
	char	fw_path[256];
	u8		fw_force_upgrade;
	u8		fw_type;
};

struct bouncing_filter_data {
	u16 	count;
};

struct grip_filter_data {
	u8	grip_mask;
};

struct accuracy_filter_data {
	u8 	do_filter;
	u16	down_x;
	u16	down_y;
	u16	down_z;
	u32	delta_pos;
	u32	delta_z;
	int	mod_x;
	int	mod_y;
};

struct jitter_filter_data {
	int	delta_x;
	int	delta_y;
};

struct anticipation_filter_data {
	int	vel_x;
	int	vel_y;
	int	force;
};

struct bouncing_filter_info {
	struct bouncing_filter_role	*role;
	struct bouncing_filter_data	data;
};

struct grip_filter_info {
	struct grip_filter_role		*role;
	struct grip_filter_data		data;
};

struct accuracy_filter_info {
	struct accuracy_filter_role	*role;
	struct accuracy_filter_data	data[MAX_FINGER];
};

struct jitter_filter_info {
	struct jitter_filter_role	*role;
	struct jitter_filter_data	data[MAX_FINGER];
};

struct anticipation_filter_info {
	struct anticipation_filter_role	*role;
	struct anticipation_filter_data	data[MAX_FINGER];
};

struct state_info {
	atomic_t power_state;
	atomic_t interrupt_state;
	atomic_t upgrade_state;
	atomic_t ta_state;
	atomic_t temperature_state;
	atomic_t proximity_state;
	atomic_t hallic_state;
	atomic_t uevent_state;
	atomic_t keyguard_state;
	atomic_t ime_state;
	atomic_t pm_state;
	u8 code;
	u32 value;
};

struct point {
	int x;
	int y;
};

struct lge_touch_data {
	void				*h_touch;
	struct state_info		state;
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct touch_platform_data	*pdata;
	struct touch_data		ts_curr_data;
	struct touch_data		ts_prev_data;
	struct touch_data		ts_report_data;
	struct touch_fw_info		fw_info;
	struct kobject 			lge_touch_kobj;
	struct delayed_work		work_init;
	struct delayed_work		work_upgrade;
	struct delayed_work		work_notify;
	struct delayed_work 	work_irq;
	struct mutex			thread_lock;
	struct wake_lock		lpwg_wake_lock;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_suspend;
#elif defined(CONFIG_FB)
	struct notifier_block	fb_notif;
#endif
#if defined(CONFIG_ARCH_MSM8916)
	struct pinctrl			*ts_pinctrl;
	struct pinctrl_state	*ts_pinset_state_active;
	struct pinctrl_state	*ts_pinset_state_suspend;
#endif
};

/* filter_func
 *
 * report_id_mask
 * - If an event is not included 'report_id_mask', it will not be reported.
 *
 * return value (frt : filter_return_type)
 * - return FRT_IGNORE if the event should be ignored. (FRT_REPORT if not)
 *
 * Each fliter can change only the 'curr_data' and 'filter_info'.
 * It should NOT change any other values such as 'pdata', 'prev_data', etc.
 */
enum filter_return_type {
	FRT_IGNORE = 0,
	FRT_REPORT,
};

struct filter_func {
	char *name;
	enum filter_return_type  (*filter)(struct i2c_client *client,
		u32 *report_id_mask);
	struct list_head list;
};

struct filter_data {
	u16 id_checklist[MAX_FINGER];
	u16 index;
};


/* touch_device_driver
 *
 * return values
 * : All functions in 'touch_device_driver' should use 'error_type'
 *   for return value.
 *
 * - NO_ERROR : NO Problem.
 * - ERROR : Error occurs, so the device will be reset.
 * - IGNORE_EVENT : Event will be ignored.
 * - IGNORE_EVENT_BUT_SAVE_IT : Event will not be reported,
 *   but saved in 'prev' data.
 *
 */

enum error_type {
	NO_ERROR = 0,
	ERROR,
	NO_FILTER,
	IGNORE_EVENT,
	IGNORE_EVENT_BUT_SAVE_IT,
};

enum gpio_type{
	GPIO_RST_PIN = 0,
	GPIO_INT_PIN,
};

struct touch_device_driver {
	enum error_type (*probe) (struct i2c_client *client,
		const struct touch_platform_data *lge_ts_data,
		const struct state_info *state,
		struct attribute ***attribute_list);
	enum error_type (*remove) (struct i2c_client *client);
	enum error_type (*suspend) (struct i2c_client *client);
	enum error_type (*resume) (struct i2c_client *client);
	enum error_type (*init) (struct i2c_client *client);
	enum error_type (*data) (struct i2c_client *client,
		struct touch_data *curr_data,
		const struct touch_data *prev_data);
	enum error_type (*filter) (struct i2c_client *client,
		struct touch_data *curr_data,
		const struct touch_data *prev_data);
	enum error_type (*power) (struct i2c_client *client, int power_ctrl);
	enum error_type (*ic_ctrl) (struct i2c_client *client,
		u8 code, u32 value, u32 *ret);
	enum error_type (*fw_upgrade) (struct i2c_client *client,
		const char* fw_path);
	enum error_type (*fw_ic_info) (struct i2c_client *client,
		struct touch_fw_info *info);
	enum error_type (*notify) (struct i2c_client *client,
		u8 code, u32 value);
	enum error_type (*lpwg) (struct i2c_client *client,
		u32 code, u32 value, struct point *data);
	enum error_type (*sd) (struct i2c_client *client,
		char* buf, int* raw_status, int* ch_status);
};

enum {
	MT_PROTOCOL_A = 0,
	MT_PROTOCOL_B,
};

enum {
	POWER_OFF = 0,
	POWER_ON,
	POWER_SLEEP,
	POWER_WAKE,
};

enum {
	INTERRUPT_DISABLE = 0,
	INTERRUPT_ENABLE,
};

enum {
	TA_DISCONNECTED = 0,
	TA_CONNECTED,
};

enum {
	PROXIMITY_FAR = 0,
	PROXIMITY_NEAR,
};

enum {
	HALL_NONE = 0,
	HALL_COVERED,
};

enum {
	UEVENT_IDLE = 0,
	UEVENT_BUSY,
};

enum {
	UPGRADE_FINISH = 0,
	UPGRADE_START,
};

enum {
	NON_FILTER = 0,
	BOUNCING_FILTER,
	GRIP_FILTER,
	ACCURACY_FILTER,
	JITTER_FILTER,
};

enum {
	FINGER = 0,
	PALM,
	PEN,
	GLOVE,
	HOVER,
};

enum {
	KEY_NONE = 0,
	TOUCH_HARD_KEY,
	TOUCH_SOFT_KEY,
	VIRTUAL_KEY,
};

enum {
	CONTINUOUS_REPORT_MODE = 0,
	REDUCED_REPORT_MODE,
};

enum {
	RESET_NONE = 0,
	SOFT_RESET,
	PIN_RESET,
	VDD_RESET,
};

enum {
	PM_RESUME = 0,
	PM_SUSPEND,
	PM_SUSPEND_IRQ,
};

enum {
	BUTTON_RELEASED	= 0,
	BUTTON_PRESSED	= 1,
	BUTTON_CANCLED	= 0xff,
};

enum {
	IC_CTRL_READ = 1,
	IC_CTRL_WRITE,
};

enum {
	NOTIFY_TA_CONNECTION = 1,
	NOTIFY_TEMPERATURE_CHANGE,
	NOTIFY_PROXIMITY,
	NOTIFY_HALL_IC,
	NOTIFY_KEYGUARD,
	NOTIFY_IME,
};

enum {
	LPWG_NONE = 0,
	LPWG_DOUBLE_TAP,
	LPWG_PASSWORD,
	LPWG_SIGNATURE,
};

enum {
	LPWG_DISABLE = 0,
	LPWG_ENABLE,
	LPWG_SET_LCD_AREA,
	LPWG_SET_ACTIVE_AREA,
	LPWG_TAP_COUNT,
	LPWG_LENGTH_BETWEEN_TAP,
	LPWG_EARLY_SUSPEND,
	LPWG_SENSOR_STATUS,
	LPWG_DOUBLE_TAP_CHECK,
	LPWG_UPDATE_ALL,
	LPWG_READ,
	LPWG_REPLY,
};

enum {
	DEBUG_NONE			= 0,
	DEBUG_BASE_INFO			= (1U << 0),
	DEBUG_TRACE			= (1U << 1),
	DEBUG_GET_DATA			= (1U << 2),
	DEBUG_ABS			= (1U << 3),
	DEBUG_BUTTON			= (1U << 4),
	DEBUG_FW_UPGRADE		= (1U << 5),
	DEBUG_GHOST			= (1U << 6),
	DEBUG_IRQ_HANDLE		= (1U << 7),
	DEBUG_POWER			= (1U << 8),
	DEBUG_JITTER			= (1U << 9),
	DEBUG_ACCURACY			= (1U << 10),
	DEBUG_BOUNCING			= (1U << 11),
	DEBUG_GRIP			= (1U << 12),
	DEBUG_ANTICIPATION		= (1U << 13),
	DEBUG_FILTER_RESULT		= (1U << 14),
	DEBUG_LPWG			= (1U << 15),
};

#ifdef LGE_TOUCH_TIME_DEBUG
enum {
	TIME_INTERRUPT = 0,
	TIME_WORKQUEUE_START,
	TIME_WORKQUEUE_END,
	TIME_FW_UPGRADE_START,
	TIME_FW_UPGRADE_END,
	TIME_PROFILE_MAX,
};

enum {
	DEBUG_TIME_PROFILE_NONE			= 0,
	DEBUG_TIME_INTERRUPT			= (1U << 0),
	DEBUG_TIME_WORKQUEUE			= (1U << 1),
	DEBUG_TIME_FW_UPGRADE			= (1U << 2),
	DEBUG_TIME_PROFILE_ALL			= (1U << 3),
};
#endif

enum {
	TEST_VERSION = 0,
	OFFICIAL_VERSION,
};

#define LGE_TOUCH_NAME		"lge_touch"

/* Basic Logging Macro
  *
  */
#define TOUCH_INFO_MSG(fmt, args...) 			\
	printk(KERN_INFO "[Touch] " fmt, ##args);

#define TOUCH_ERR_MSG(fmt, args...) 			\
	printk(KERN_ERR "[Touch E] [%s %d] " fmt,	\
		__func__, __LINE__, ##args);

#define TOUCH_DEBUG_MSG(fmt, args...) 			\
	printk(KERN_INFO "[Touch D] [%s %d] " fmt,	\
		__func__, __LINE__, ##args);

/* For Error Handling
  *
  * DO_IF : execute 'do_work',
  * and if the result is true, print 'error_log' and goto 'goto_error'.
  *
  * DO_SAFE : execute 'do_work',
  * and if the result is '< 0', print 'error_log' and goto 'goto_error'
  *
  * ASSIGN : excute 'do_assign',
  * and if the result is 'NULL', print 'error_log' and goto 'goto_error'
  *
  * ERROR_IF : if the condition is true(ERROR),
  * print 'string' and goto 'goto_error'.
  */

#define DO_IF(do_work, goto_error)				\
do {								\
	if (do_work) { 						\
		printk(KERN_INFO "[Touch E] Action Failed [%s %d] \n",	\
			__func__, __LINE__); \
		goto goto_error; 				\
	}							\
} while (0)

#define DO_SAFE(do_work, goto_error) 				\
	DO_IF(unlikely((do_work) < 0), goto_error)

#define ASSIGN(do_assign, goto_error) 				\
do {								\
	if ((do_assign) == NULL) { 				\
		printk(KERN_INFO "[Touch E] Assign Failed [%s %d] \n",	\
			__func__, __LINE__); 		\
		goto goto_error; 				\
	}							\
} while (0)

#define ERROR_IF(cond, string, goto_error)	\
do {						\
	if (cond) {				\
		TOUCH_ERR_MSG(string);		\
		goto goto_error;		\
	}					\
} while (0)


/* For using debug_mask more easily
  *
  */
extern u32 touch_debug_mask;
#define TOUCH_DEBUG(condition, fmt, args...)			\
do {								\
	if (unlikely(touch_debug_mask & (condition)))		\
		printk(KERN_INFO "[Touch] " fmt, ##args);	\
} while (0)

#ifdef LGE_TOUCH_TIME_DEBUG
extern u32 touch_time_debug_mask;
#define TOUCH_TIME_DEBUG(condition, fmt, args...)		\
do {								\
	if (unlikely(touch_time_debug_mask & (condition))) 	\
		printk(KERN_INFO "[Touch] " fmt, ##args);	\
} while (0)
#endif

#define TOUCH_TRACE()						\
	TOUCH_DEBUG(DEBUG_TRACE, " - %s %d\n", __func__, __LINE__)


/* sysfs
 *
 */
struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct i2c_client *client, char *buf);
	ssize_t (*store)(struct i2c_client *client,
		const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name 	\
	= __ATTR(_name, _mode, _show, _store)


int  touch_driver_register(struct touch_device_driver *driver,
	struct of_device_id *match_table);
void touch_driver_unregister(void);

void set_touch_handle(struct i2c_client *client, void *h_touch);
void *get_touch_handle(struct i2c_client *client);
void send_uevent(char *string[2]);
void send_uevent_lpwg(struct i2c_client *client, int type);
#endif
