/* Copyright (c) 2017, LG Eletronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "TIME-TO-FULL: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <soc/qcom/lge/lge_time_to_full.h>

#define MODULE_NAME "time-to-full"

#define DISCHARGE			-1
#define FULL				-1
#define EMPTY				-1
#define NOTYET				-1
#define ERROR				-1

#define DEFAULT_BATTERY_CAPACITY	3000
#define DEFAULT_DCP_CURRENT			1800
#define DEFAULT_HVDCP_CURRENT		3000
#define DEFAULT_WLC_CURRENT			1000
#define DEFAULT_COMP_VALUE			0

struct cc_step {
	int cur;
	int soc;
};

struct cv_slope {
	int cur;
	int soc;
	int time;
};

struct lge_time_to_full_data {
	struct device *dev;
	struct cc_step *cc_data;
	struct cc_step *dynamic_cc_data;
	struct cv_slope *cv_data;
	int *cc_step_time;
	int cc_data_length;
	int cv_data_length;
	int cv_soc_index;
	int capacity;
	int ttf_now;
	bool chg_complete;
	int dcp_current;
	int hvdcp_current;
	int hvdcp3_current;
	int wlc_current;
	bool hvdcp_supported;
	bool hvdcp3_supported;
	bool wlc_supported;
	bool parallel_supported;
#ifdef CONFIG_LGE_USB_TYPE_C
	bool typec_supported;
#endif
	/* compensation variables */
	int sdp_comp;
	int cdp_comp;
	int dcp_comp;
	int avg_sdp_comp;
	int avg_cdp_comp;
	int avg_dcp_comp;
	int avg_comp;
	int report_ttf_comp;
	int min_comp;
#ifdef CONFIG_LGE_PM_CHARGERLOGO_WAIT_FOR_FG_INIT
	bool first_soc_est_done;
#endif
	/* power supply */
	struct power_supply	ttf_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*batt_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*parallel_psy;
	struct power_supply	*typec_psy;
	/* delayed work */
	struct delayed_work time_to_full_report_work;
	struct delayed_work average_current_work;
	bool avg_calc_work;
	int avg_current;
	struct delayed_work battery_profile_read_work;
	const char *profile_name;
	/* for evaluation */
	int runtime_consumed[100];
	int	really_remained[100+1];
	int	ttf_remained[100];
	int evaluate_native[100];
	int avg_cur[100];
	long starttime_of_charging;
	long starttime_of_soc;
	int	soc_begin;
	int	soc_now;
};

static char *ttf_supplies[] = {
	"bms",
	"usb",
	"battery",
};

static enum power_supply_property ttf_power_props[] = {
	POWER_SUPPLY_PROP_TIME_TO_FULL,
};

enum print_reason {
	PR_ERROR = BIT(0),
	PR_INFO = BIT(1),
	PR_STATUS = BIT(2),
	PR_EVALUATE = BIT(3),

	PR_DEBUG = BIT(7),
};

static int ttf_debug_mask = PR_ERROR | PR_INFO | PR_EVALUATE | PR_STATUS;

#define pr_ttf(reason, fmt, ...)			\
do {							\
	if (ttf_debug_mask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

/* for debugging */
static struct lge_time_to_full_data *the_ttf;
/* for debugging */

/* for evaluation */
static void lge_time_to_full_evaluate_var_clear(struct lge_time_to_full_data *ttf)
{
	int i;

	pr_ttf(PR_DEBUG, "compare variable clear\n");
	for (i = 0; i < 100 ; i++) {
		ttf->runtime_consumed[i] = EMPTY;
		ttf->really_remained[i] = EMPTY;
		ttf->ttf_remained[i] = EMPTY;
		ttf->evaluate_native[i] = EMPTY;
		ttf->avg_cur[i] = EMPTY;
	}
	ttf->really_remained[100] = 0;
	ttf->starttime_of_charging = EMPTY;
	ttf->starttime_of_soc = EMPTY;
	ttf->soc_begin = EMPTY;
	ttf->soc_now = EMPTY;
}

#define NATIVE_DELTA_SOC 4
static int remains_by_native(int soc, struct lge_time_to_full_data *ttf) {
	int begin_soc = ttf->soc_begin;
	int delta_time = 0;
	int i;

	// How about the decreasing SoC during charging for native?
	if (ttf->evaluate_native[soc] == EMPTY) {
		// Finding the begin of SoC
		for (i=0; i<100; ++i) {
			if (ttf->runtime_consumed[i] != EMPTY) {
				begin_soc = i;
				break;
			}
		}

		// Accumulating the delta times
		if (begin_soc != EMPTY && begin_soc + NATIVE_DELTA_SOC <= soc) {
			for (i=0; i<NATIVE_DELTA_SOC; ++i){
				delta_time += ttf->runtime_consumed[soc-1-i];
			}
			ttf->evaluate_native[soc] = (100-soc) * delta_time / NATIVE_DELTA_SOC;
		}

		pr_ttf(PR_DEBUG, "begin SoC %2d : Delta time : %5d\n", begin_soc, delta_time);
	}

	return ttf->evaluate_native[soc];
}

static int remains_by_ttf(int soc, struct lge_time_to_full_data *ttf) {
	int begin_soc = ttf->soc_begin;

	if (ttf->ttf_remained[soc] == EMPTY) {
		if (begin_soc != EMPTY && begin_soc <= soc)
			ttf->ttf_remained[soc] = ttf->ttf_now;
	}
	pr_ttf(PR_DEBUG, "begin SoC %2d : \n", begin_soc);
	return ttf->ttf_remained[soc];
}

static void lge_time_to_full_evaluate_report(long eoc, struct lge_time_to_full_data *ttf) {
	// Evaluation has meaning only on charging termination (== soc 100%)
	union power_supply_propval prop = {0,};
	enum power_supply_type usb_supply_type;
	int rc = 0;
	int i, begin_soc = ttf->soc_begin;
	int really_remained[100+1];

	if (begin_soc == 100) {
		/* If charging is started on 100%,
		 * Skip to evaluate
		 */
		return;
	}

	rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_TYPE, &prop);
	if (rc)
		pr_ttf(PR_ERROR, "usb_psy usb type read fail rc = %d\n", rc);
	else
		usb_supply_type = prop.intval;

	really_remained[100] = 0;
	for (i=99; begin_soc<=i; --i)
		really_remained[i] = ttf->runtime_consumed[i] + really_remained[i+1];

	pr_ttf(PR_EVALUATE, "Evaluating... Charger Type (%d), charging from %2d(%ld) to 100(%ld), (duration %ld)\n",
		 usb_supply_type, begin_soc, ttf->starttime_of_charging, eoc, eoc-ttf->starttime_of_charging);
	pr_ttf(PR_EVALUATE, "soc, really_consumed"		/* really measured */
		", really_remained, native_remained, ttf_remained"		/* for comparison */
		", ttf_really_diff, avg_current"				/* ttf really diff to min */
		"\n");
	for (i=begin_soc; i<100; ++i) {
		pr_ttf(PR_EVALUATE, "%d, %d, %d, %d, %d, %d, %d\n",
			i, ttf->runtime_consumed[i], really_remained[i],
			ttf->evaluate_native[i], ttf->ttf_remained[i],
			(ttf->ttf_remained[i] - really_remained[i]) / 60, ttf->avg_cur[i]);
	}
}

static void lge_time_to_full_evaluate(struct lge_time_to_full_data *ttf)
{
	union power_supply_propval prop = {0,};
	int remains_ttf = EMPTY;
	int remains_native = EMPTY;
	int soc;
	int chg_present;
	int rc = 0;
	long now;
	struct timespec	tspec;

	rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "usb_psy usb present read fail rc = %d\n", rc);
		return;
	} else
		chg_present = prop.intval;

	rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "bms_psy capacity read fail rc = %d\n", rc);
		return;
	} else
		soc = prop.intval;

		// Simple check
	if ( !(chg_present && (0 <= soc && soc <= 100)) ) {
		/* Invalid invokation */
		return;
	}

	get_monotonic_boottime(&tspec);
	now = tspec.tv_sec;

	// This calling may NOT be bound with SoC changing
	if (ttf->soc_now != soc) {
		if (ttf->starttime_of_charging == EMPTY) {
				// New insertion
			ttf->soc_begin = soc;
			ttf->starttime_of_charging = now;
		}
		else {	// Soc rasing up
			ttf->runtime_consumed[soc-1] = now - ttf->starttime_of_soc;
		}

		/* Update time_me */
		ttf->soc_now = soc;
		ttf->starttime_of_soc = now;

		if (0 <= soc && soc < 100) {
			remains_native = remains_by_native(soc, ttf);
			remains_ttf = remains_by_ttf(soc, ttf);
			ttf->avg_cur[soc] = ttf->avg_current;
		}

		if (soc == 100) {
			/* Evaluate NOW! (at the 100% soc) :
			 * Evaluation has meaning only on full(100%) charged status
			 */
			lge_time_to_full_evaluate_report(now, ttf);
			lge_time_to_full_evaluate_var_clear(ttf);
		} else {
			pr_ttf(PR_STATUS, "soc %d, elapsed %ds... => ttf %d, native %d\n",
				soc, ttf->runtime_consumed[soc > 0 ? soc-1 : 0],
				remains_ttf, remains_native);
		}
	}
}
/* for evaluate */

/* Restore Dynamic cc step table to Original cc step table */
static void lge_time_to_full_cc_table_recovery(struct lge_time_to_full_data *ttf)
{
	struct cc_step *cc_data = ttf->cc_data;
	struct cc_step *dynamic_cc_data = ttf->dynamic_cc_data;
	int i;

	pr_ttf(PR_INFO, "TTF CC Table Recovery\n");
	memcpy(dynamic_cc_data, cc_data, sizeof(struct cc_step) * ttf->cc_data_length);
	for (i = 0; i < ttf->cc_data_length; i++)
		pr_ttf(PR_DEBUG, "Recovery CC Table [%d] : [%d]\n", i, dynamic_cc_data[i].cur);
}

/* cc step time update */
static void lge_time_to_full_update_cc_step_time(struct lge_time_to_full_data *ttf)
{
	struct cc_step *dynamic_cc_data = ttf->dynamic_cc_data;
	struct cv_slope *cv_data = ttf->cv_data;
	int *cc_step_time = ttf->cc_step_time;
	int i;

	for (i = 1; i < ttf->cc_data_length; i++)
		cc_step_time[i-1] = ttf->capacity * (dynamic_cc_data[i].soc - dynamic_cc_data[i-1].soc)\
						/ dynamic_cc_data[i-1].cur * 3600 / 1000;
	cc_step_time[ttf->cc_data_length-1] = ttf->capacity * (cv_data[ttf->cv_soc_index].soc - dynamic_cc_data[ttf->cc_data_length-1].soc)\
						/ dynamic_cc_data[ttf->cc_data_length-1].cur * 3600 / 1000;

	for (i = 0; i < ttf->cc_data_length; i++)
		pr_ttf(PR_DEBUG, "CC Step Time [%d] : [%d]\n", i, cc_step_time[i]);
}

/* Determine the charging current according to charger type */
static int lge_time_to_full_get_charging_current(struct lge_time_to_full_data *ttf)
{
	union power_supply_propval prop = {0,};
	enum power_supply_type usb_supply_type;
	int charge_current;
	bool aicl_done;
	int rc = 0;

	rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_TYPE, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "usb_psy usb type read fail rc = %d\n", rc);
		return ERROR;
	} else
		usb_supply_type = prop.intval;

	pr_ttf(PR_INFO, "usb_supply_type : %d\n", usb_supply_type);

	switch (usb_supply_type) {
		case POWER_SUPPLY_TYPE_USB_HVDCP:
			charge_current = ttf->hvdcp_current;
			break;
		case POWER_SUPPLY_TYPE_USB_HVDCP_3:
			charge_current = ttf->hvdcp3_current;
			break;
		case POWER_SUPPLY_TYPE_WIRELESS:
			charge_current = ttf->wlc_current;
			break;
		case POWER_SUPPLY_TYPE_USB:
			rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
			if (rc) {
				pr_ttf(PR_ERROR, "usb_psy current max read fail rc = %d\n", rc);
				return ERROR;
			} else
				charge_current = prop.intval/1000;

			if (ttf->sdp_comp)
				charge_current += (charge_current * ttf->sdp_comp / 100);
#ifdef CONFIG_LGE_USB_TYPE_C
			if(ttf->typec_supported) {
				if(!ttf->typec_psy)
					ttf->typec_psy = power_supply_get_by_name("usb_pd");

				if (!ttf->typec_psy) {
					pr_ttf(PR_ERROR, "no typec_psy found\n");
					return ERROR;
				}
				rc = ttf->typec_psy->get_property(ttf->typec_psy, POWER_SUPPLY_PROP_TYPE, &prop);
				if (rc) {
					pr_ttf(PR_ERROR, "typec_psy typec read fail rc = %d\n", rc);
					return ERROR;
				} else {
					if (prop.intval == POWER_SUPPLY_TYPE_CTYPE_PD)
						charge_current = ttf->hvdcp_current;
						break;
				}
			}
#endif
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
			if (rc) {
				pr_ttf(PR_ERROR, "usb_psy current max read fail rc = %d\n", rc);
				return ERROR;
			} else
				charge_current = prop.intval/1000;

			if (ttf->cdp_comp)
				charge_current += (charge_current * ttf->cdp_comp / 100);
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			rc = ttf->batt_psy->get_property(ttf->batt_psy, POWER_SUPPLY_PROP_AICL_DONE, &prop);
			if (rc) {
				pr_ttf(PR_ERROR, "batt_psy input current max read fail rc = %d\n", rc);
				return ERROR;
			} else
				aicl_done = prop.intval;

			if (aicl_done) {
				pr_ttf(PR_INFO, "Applied AICL Current\n");
				rc = ttf->batt_psy->get_property(ttf->batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &prop);
				if (rc) {
					pr_ttf(PR_ERROR, "batt_psy input current max read fail rc = %d\n", rc);
					return ERROR;
				} else
					charge_current = prop.intval/1000;

				if (ttf->parallel_supported) {
					if(!ttf->parallel_psy)
						ttf->parallel_psy = power_supply_get_by_name("usb-parallel");

					if (!ttf->parallel_psy) {
						pr_ttf(PR_ERROR, "no parallel_psy found\n");
						return ERROR;
					}
					rc = ttf->parallel_psy->get_property(ttf->parallel_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
					if (rc) {
						pr_ttf(PR_ERROR, "parallel_psy current max read fail rc = %d\n", rc);
						return ERROR;
					} else
						charge_current += prop.intval/1000;
				}
			} else
				charge_current = ttf->dcp_current;

			if (ttf->dcp_comp)
				charge_current += (charge_current * ttf->dcp_comp / 100);
			break;
		default:
			if (ttf->hvdcp_supported) {
				charge_current = ttf->hvdcp_current;
			} else {
				rc = ttf->batt_psy->get_property(ttf->batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &prop);
				if (rc) {
					pr_ttf(PR_ERROR, "batt_psy input current max read fail rc = %d\n", rc);
					return ERROR;
				} else
					charge_current = prop.intval;
			}
			break;
	}

	pr_ttf(PR_INFO, "Determine Setting Current : [%d]\n", charge_current);
	return charge_current;
}

/* Average current compensate according to charger type */
static int lge_time_to_full_average_current_comp(int avg_current, struct lge_time_to_full_data *ttf)
{
	union power_supply_propval prop = {0,};
	enum power_supply_type usb_supply_type;
	int rc = 0;

	rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_TYPE, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "usb_psy usb type read fail rc = %d\n", rc);
		return avg_current;
	} else
		usb_supply_type = prop.intval;

	pr_ttf(PR_INFO, "usb_supply_type : %d\n", usb_supply_type);

	switch (usb_supply_type) {
		case POWER_SUPPLY_TYPE_USB:
			avg_current += (avg_current * ttf->avg_sdp_comp / 100);
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			avg_current += (avg_current * ttf->avg_cdp_comp / 100);
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			avg_current += (avg_current * ttf->avg_dcp_comp / 100);
			break;
		default:
			break;
	}

	return avg_current;
}

/* Average current calculate work */
/* Find the average current by storing the charge current in the queue for 10 minutes every 30 seconds for 5 minutes */
#define AVERAGE_CHG_CURRENT_CALC_PERIOD (30*HZ)
#define CURRENT_LOW_THRESHOLD 300
#define QUEUE_SIZE 12
static void lge_time_to_full_average_current_work(struct work_struct *work)
{
	struct lge_time_to_full_data *ttf = container_of(work, struct lge_time_to_full_data, average_current_work.work);
	union power_supply_propval prop = {0,};
	static int avg_current[QUEUE_SIZE] = {0, };
	static int rear = 0;
	int front = 0;
	int current_now = 0;
	int chg_present = 0;
	int chg_status = 0;
	int i = 0;
	int min = 0;
	int max = 0;
	int rc = 0;

	rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if(rc) {
		pr_ttf(PR_ERROR, "current now read fail rc = %d\n", rc);
		schedule_delayed_work(&ttf->average_current_work, AVERAGE_CHG_CURRENT_CALC_PERIOD);
		return;
	} else
		current_now = prop.intval/1000;

	rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "usb_psy usb present read fail rc = %d\n", rc);
		schedule_delayed_work(&ttf->average_current_work, AVERAGE_CHG_CURRENT_CALC_PERIOD);
		return;
	} else
		chg_present = prop.intval;

	rc = ttf->batt_psy->get_property(ttf->batt_psy, POWER_SUPPLY_PROP_STATUS, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "batt_psy battery status read fail rc = %d\n", rc);
		schedule_delayed_work(&ttf->average_current_work, AVERAGE_CHG_CURRENT_CALC_PERIOD);
		return;
	} else
		chg_status = prop.intval;

	if (!chg_present || chg_status == POWER_SUPPLY_STATUS_FULL) {
		pr_ttf(PR_INFO, "Cancel AVG Current Work\n");
		if(avg_current[front] != 0 && ttf->avg_calc_work) {
			pr_ttf(PR_INFO, "AVG Current Queue set to Zero\n");
			for(i = 0; i < QUEUE_SIZE; i++)
				avg_current[i] = 0;
			ttf->avg_current = 0;
			rear = 0;
			ttf->avg_calc_work = false;
		}
		pr_ttf(PR_INFO, "Cancel AVG Current Work\n");
		cancel_delayed_work(&ttf->average_current_work);
	} else {
		/* insert queue */
		if (rear < QUEUE_SIZE) { //when queue is not full
			for(i = rear; i > front; i--)
				avg_current[i] = avg_current[i-1];
			rear++;
		} else {	//when queue is full
			for(i = QUEUE_SIZE-1; i > front; i--)
				avg_current[i] = avg_current[i-1];
		}
		avg_current[front] = current_now;

		/* calculation average when queue is full*/
		if (rear >= QUEUE_SIZE) {
			ttf->avg_current = 0;
			/* to exclude min, max current */
			for(i = 1; i < QUEUE_SIZE; i++)
			{
				if(avg_current[min] >= avg_current[i])
					min = i;

				if(avg_current[max] <= avg_current[i])
					max = i;
			}
			pr_ttf(PR_DEBUG, "MIN Index : %d, MAX Index : %d\n", min, max);

			if (min == max) {
				ttf->avg_current = avg_current[front];
				schedule_delayed_work(&ttf->average_current_work, AVERAGE_CHG_CURRENT_CALC_PERIOD);
				return;
			}

			for(i = 0; i < QUEUE_SIZE; i++)
			{
				if (i == min || i == max) {
					pr_ttf(PR_DEBUG, "AVG Current Queue[%d] = [%d] <- SKIP min max\n",
							i, avg_current[i]);
					continue;
				}
				pr_ttf(PR_DEBUG, "AVG Current Queue[%d] = [%d]\n", i, avg_current[i]);
				ttf->avg_current += avg_current[i];
			}

			if (ttf->avg_current >= (CURRENT_LOW_THRESHOLD * (QUEUE_SIZE - 2)) * (-1))
				ttf->avg_current = CURRENT_LOW_THRESHOLD * (QUEUE_SIZE - 2); // when device is discharged
			else
				ttf->avg_current = abs(ttf->avg_current);

			ttf->avg_current /= (QUEUE_SIZE - 2);

			if (ttf->avg_comp)
				ttf->avg_current += (ttf->avg_current * ttf->avg_comp / 100);

			pr_ttf(PR_DEBUG, "AVG Current : [%d]\n", ttf->avg_current);
			ttf->avg_current = lge_time_to_full_average_current_comp(ttf->avg_current, ttf);
			pr_ttf(PR_STATUS, "Queue is Full... New AVG Current : [%d]\n", ttf->avg_current);
		} else {
			pr_ttf(PR_STATUS, "Queue is not Full... Calculating AVG Current... Queue Index: [%d]\n", rear);
			ttf->avg_current = 0;
		}

		schedule_delayed_work(&ttf->average_current_work, AVERAGE_CHG_CURRENT_CALC_PERIOD);
	}
}

/* Starting average current calculation after 2 minutes of charger connection */
static void average_current_work_queue(struct lge_time_to_full_data *ttf)
{
	cancel_delayed_work_sync(&ttf->average_current_work);
	schedule_delayed_work(&ttf->average_current_work, AVERAGE_CHG_CURRENT_CALC_PERIOD * 4);
}

/* Calculate time to full now */
#define NOT_SUPPORT_STEP_CHARGING	1
static int lge_time_to_full_calc(int charge_current, struct lge_time_to_full_data *ttf)
{
	union power_supply_propval prop = {0,};
	struct cc_step *cc_data = ttf->cc_data;
	struct cc_step *dynamic_cc_data = ttf->dynamic_cc_data;
	struct cv_slope *cv_data = ttf->cv_data;
	int cc_time = 0, cv_time = 0;
	int i, j = 0;
	int soc = 0;
	int ttf_now = 0;
	int rc = 0;
/* for debugging */
	int current_now = 0;
	int current_set = 0;

	rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_TIME_TO_FULL_CAPACITY, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "bms_psy time to full capacity read fail rc = %d\n", rc);
		return ERROR;
	} else
		soc = prop.intval;

	if(soc >= 1000)
		return FULL;

	for (i = 0; i < ttf->cv_data_length ;i++) {
		if (abs(charge_current) >= cv_data[i].cur)
			break;
	}

	i = i >= ttf->cv_data_length ? ttf->cv_data_length - 1 : i;

	pr_ttf(PR_INFO, "Determine cv soc : %d\n", cv_data[i].soc);
	pr_ttf(PR_INFO, "soc : %d, chg_current : %d, cv_data[%d].cur : %d, cv_data[%d].soc : %d\n",
				soc, charge_current, i, cv_data[i].cur, i, cv_data[i].soc);

	if (cv_data[i].soc < soc) {
		pr_ttf(PR_INFO, "CV Charging\n");
		for (i = 0; i < ttf->cv_data_length; i++) {
			if (soc <= cv_data[i].soc)
				break;
		}
		pr_ttf(PR_INFO, "cv_data[%d].soc : %d, soc : %d\n", i, cv_data[i].soc, soc);
		cv_time = ((cv_data[i-1].time - cv_data[i].time) * (cv_data[i].soc - soc)\
				/ (cv_data[i].soc - cv_data[i-1].soc)) + cv_data[i].time;
		pr_ttf(PR_INFO, "cv_time = %d\n", cv_time);
	} else {
		pr_ttf(PR_INFO, "CC Charging\n");
		cv_time = cv_data[i].time;
		/* save the cv_soc_index to calculate new_cv_soc */
		ttf->cv_soc_index = i;
		pr_ttf(PR_INFO, "cv_soc_index = %d, cc_data_length = %d\n",
				ttf->cv_soc_index, ttf->cc_data_length);

		if (ttf->cc_data_length == NOT_SUPPORT_STEP_CHARGING) {
			pr_ttf(PR_STATUS, "Not Support Step Charging... \n");
			dynamic_cc_data[0].cur = abs(charge_current);

			cc_time = ttf->capacity * (cv_data[ttf->cv_soc_index].soc - soc)\
						/ dynamic_cc_data[ttf->cc_data_length-1].cur * 3600 / 1000;
		} else {
			pr_ttf(PR_STATUS, "Supported Step Charging... \n");
			for (j = 0; j < ttf->cc_data_length ; j++) {
				if (abs(charge_current) < cc_data[j].cur)
					dynamic_cc_data[j].cur = abs(charge_current);
				else
					dynamic_cc_data[j].cur = cc_data[j].cur;
				pr_ttf(PR_INFO, "dynamic_cc_data[%d].cur = [%d]\n", j, dynamic_cc_data[j].cur);
			}

			/* cc time update to new cc step table */
			lge_time_to_full_update_cc_step_time(ttf);

			/* cc step section check for current soc */
			for (j = 1; j < ttf->cc_data_length ;j++) {
				if (soc < (dynamic_cc_data[j].soc))
					break;
			}
			pr_ttf(PR_INFO, "cc_soc_index = %d\n", j);

			/* Calculate cc_time_soc */
			if (j < ttf->cc_data_length)
				cc_time = ttf->capacity * (dynamic_cc_data[j].soc - soc)\
						/ dynamic_cc_data[j-1].cur * 3600 / 1000;
			else
				cc_time = ttf->capacity * (cv_data[ttf->cv_soc_index].soc - soc)\
						/ dynamic_cc_data[ttf->cc_data_length-1].cur * 3600 / 1000;

			pr_ttf(PR_INFO, "cc_time_soc = %d\n", cc_time);

			/* Calculate total cc time */
			for ( ; j < ttf->cc_data_length; j++)
				cc_time = cc_time + ttf->cc_step_time[j];
		}

		if (cc_time < 0)
			cc_time = 0;

		pr_ttf(PR_INFO, "cc_time: %d, cv_time: %d\n", cc_time, cv_time);
	}


	if (cv_time + cc_time >= 0)
		ttf_now = cv_time + cc_time + 60;
	else
		ttf_now = 60;

	/* compensate ttf result */
	if (ttf->report_ttf_comp)
		ttf_now += (ttf_now * ttf->report_ttf_comp /100);

	if (ttf->min_comp)
		ttf_now += (ttf->min_comp * 60);

	pr_ttf(PR_INFO, "capacity: %d, soc: %d, ttf: %d, comp_ttf: %d, CV_SOC: %d, CV_SOC_INDEX: %d, CUR: %d\n",
		ttf->capacity, soc, cv_time + cc_time, ttf_now, cv_data[i].soc, i, charge_current);

	/* for debugging */
	if(ttf->parallel_supported) {
		if(!ttf->parallel_psy)
			ttf->parallel_psy = power_supply_get_by_name("usb-parallel");

		if (!ttf->parallel_psy) {
			pr_ttf(PR_ERROR, "no parallel_psy found\n");
		} else {
			rc = ttf->parallel_psy->get_property(ttf->parallel_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &prop);
			if (rc)
				pr_ttf(PR_ERROR, "parallel_psy constant charge current max read fail = %d\n", rc);
			else
				current_set = prop.intval/1000;
		}
	}

	rc = ttf->batt_psy->get_property(ttf->batt_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &prop);
	if (rc)
		pr_ttf(PR_ERROR, "batt_psy constant charge current max read fail = %d\n", rc);
	else
		current_set += prop.intval/1000;

	rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (rc)
		pr_ttf(PR_ERROR, "bms psy current now read fail = %d\n", rc);
	else
		current_now = prop.intval/1000;

	pr_ttf(PR_INFO, "cur_set : %d, cur_now : %d, charge_cur : %d, soc : %d, ttf : %d\n",
			current_set, current_now, charge_current, soc, ttf_now);
/* for debugging */
	return ttf_now;
}

/* Report time to full */
static int lge_time_to_full_report(struct lge_time_to_full_data *ttf)
{
	int ttf_time;
	int charge_current;

	if (ttf->avg_current > 0) {
		charge_current = ttf->avg_current;
		pr_ttf(PR_INFO, "use avg current : [%d]\n", charge_current);
	} else {
		charge_current = lge_time_to_full_get_charging_current(ttf);
		pr_ttf(PR_INFO, "use setting current : [%d]\n", charge_current);
		if (charge_current <= 0)
			return NOTYET;
	}

	ttf_time = lge_time_to_full_calc(charge_current, ttf);

	pr_ttf(PR_INFO, "Time To Full Result : [%d]\n", ttf_time);

	return ttf_time;
}

/* debugging */
int get_charging_time(void)
{
	return the_ttf->ttf_now;
}
/* debugging */

/* Read the battery profile & apply cc-cv data according to profile after 10 seconds of booting */
#define BATT_PROFILE_READ_TIME (10*HZ)
static void lge_battery_profile_read_work(struct work_struct *work)
{
	struct lge_time_to_full_data *ttf = container_of(work, struct lge_time_to_full_data, battery_profile_read_work.work);
	struct device_node *node = ttf->dev->of_node;
	struct device_node *child;
	union power_supply_propval prop = {0,};
	const char *profile_name = NULL;
	int len = 0;
	int i;
	int rc = 0;

	rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
	if (rc) {
		pr_ttf(PR_ERROR, "bms_psy battery type read fail rc = %d\n", rc);
		schedule_delayed_work(&ttf->battery_profile_read_work, BATT_PROFILE_READ_TIME);
		return;
	} else
		ttf->profile_name = prop.strval;

	pr_ttf(PR_INFO, "Battery Profile Name : %s", ttf->profile_name);

	for_each_child_of_node(node, child) {
		rc = of_property_read_string(child,
					"lge,profile_name", &profile_name);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read battery profile name : %d", rc);
			schedule_delayed_work(&ttf->battery_profile_read_work, BATT_PROFILE_READ_TIME);
		}

		if (!strcmp(ttf->profile_name, profile_name)) {
			pr_ttf(PR_STATUS, "Find Battery Profile\n");
			pr_ttf(PR_STATUS, "Battery Profile : %s\n", ttf->profile_name);

			kzfree(ttf->cc_data);
			ttf->cc_data = NULL;
			if (of_get_property(child, "lge,cc_data", &len)) {
				ttf->cc_data = kzalloc(len, GFP_KERNEL);
				ttf->cc_data_length = len / sizeof(struct cc_step);
				rc = of_property_read_u32_array(child, "lge,cc_data",
							(u32 *)ttf->cc_data, len/sizeof(u32));
				if (rc) {
					pr_ttf(PR_ERROR, "failed to read cc_data : %d\n", rc);
					kzfree(ttf->cc_data);
					ttf->cc_data = NULL;
				}
			} else {
				 pr_ttf(PR_ERROR, "there is not cc_data\n");
			}

			kzfree(ttf->cv_data);
			ttf->cv_data = NULL;
			if (of_get_property(child, "lge,cv_data", &len)) {
				ttf->cv_data = kzalloc(len, GFP_KERNEL);
				ttf->cv_data_length = len / sizeof(struct cv_slope);
				rc = of_property_read_u32_array(child, "lge,cv_data",
							(u32 *)ttf->cv_data, len/sizeof(u32));
				if (rc) {
					pr_ttf(PR_ERROR, "failed to read cv_data : %d\n", rc);
					kzfree(ttf->cv_data);
					ttf->cv_data = NULL;
				}
			} else {
				 pr_ttf(PR_ERROR," there is not cv_data\n");
			}
			pr_ttf(PR_STATUS, "Applied %s Profile CC CV DATA\n", ttf->profile_name);

			rc = of_property_read_u32(child, "lge,sdp_comp",
									&ttf->sdp_comp);
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read sdp_comp : %d\n", rc);
				ttf->sdp_comp = DEFAULT_COMP_VALUE;
			}

			rc = of_property_read_u32(child, "lge,cdp_comp",
									&ttf->cdp_comp);
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read cdp_comp : %d\n", rc);
				ttf->cdp_comp = DEFAULT_COMP_VALUE;
			}

			rc = of_property_read_u32(child, "lge,dcp_comp",
									&ttf->dcp_comp);
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read dcp_comp : %d\n", rc);
				ttf->dcp_comp = DEFAULT_COMP_VALUE;
			}

			rc = of_property_read_u32(child, "lge,avg_sdp_comp",
									&ttf->avg_sdp_comp);
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read avg_sdp_comp : %d\n", rc);
				ttf->avg_sdp_comp = DEFAULT_COMP_VALUE;
			}

			rc = of_property_read_u32(child, "lge,avg_cdp_comp",
									&ttf->avg_cdp_comp);
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read avg_cdp_comp : %d\n", rc);
				ttf->avg_cdp_comp = DEFAULT_COMP_VALUE;
			}

			rc = of_property_read_u32(child, "lge,avg_dcp_comp",
									&ttf->avg_dcp_comp);
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read avg_dcp_comp : %d\n", rc);
				ttf->avg_dcp_comp = DEFAULT_COMP_VALUE;
			}

			rc = of_property_read_u32(child, "lge,min_comp",
									&ttf->min_comp);
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read min_comp : %d\n", rc);
				ttf->min_comp = DEFAULT_COMP_VALUE;
			}
			break;
		}
	}

	if(!profile_name || strcmp(ttf->profile_name, profile_name)) {
		pr_ttf(PR_INFO,"Battery Profile couldn't Read... USE Default Battery Profile (%s)\n",
							ttf->profile_name);
	} else {
		lge_time_to_full_cc_table_recovery(ttf);
		pr_ttf(PR_INFO,"USE %s Battery Profile\n", ttf->profile_name);
	}

	if (ttf->cc_data) {
		pr_ttf(PR_INFO, "CC Table\n");
		pr_ttf(PR_INFO, "current, soc\n");
		for(i = 0; i < ttf->cc_data_length; i++)
			pr_ttf(PR_INFO, "%d, %d\n", ttf->cc_data[i].cur, ttf->cc_data[i].soc);
	} else {
		pr_ttf(PR_INFO, "CC Table Read Fail\n");
	}

	if (ttf->cv_data) {
		pr_ttf(PR_INFO, "CV Table\n");
		pr_ttf(PR_INFO, "current, soc, time\n");
		for(i = 0; i < ttf->cv_data_length; i++)
			pr_ttf(PR_INFO, "%d, %d, %d\n", ttf->cv_data[i].cur, ttf->cv_data[i].soc, ttf->cv_data[i].time);
	} else {
		pr_ttf(PR_INFO, "CV Table Read Fail\n");
	}

	cancel_delayed_work(&ttf->battery_profile_read_work);
}

/* Submit time to full now once every minute */
#define TIME_TO_FULL_REPORT_PERIOD (60*HZ)
static void lge_time_to_full_report_work(struct work_struct *work)
{
	struct lge_time_to_full_data *ttf = container_of(work, struct lge_time_to_full_data, time_to_full_report_work.work);
	union power_supply_propval prop = {0,};
	int chg_present;
	int soc;
	int rc = 0;

	pr_ttf(PR_INFO, "TTF Report Work... \n");
#ifdef CONFIG_LGE_PM_CHARGERLOGO_WAIT_FOR_FG_INIT
	if(!ttf->first_soc_est_done) {
		rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_FIRST_SOC_EST_DONE, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "bms_psy first soc est done read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
		} else
			ttf->first_soc_est_done = prop.intval;
	} else {
#endif
		rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "usb_psy usb present read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
			schedule_delayed_work(&ttf->time_to_full_report_work, TIME_TO_FULL_REPORT_PERIOD);
			return;
		} else
			chg_present = prop.intval;

		rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_TIME_TO_FULL_CAPACITY, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "bms_psy time to full soc read fail rc = %d\n", rc);
			schedule_delayed_work(&ttf->time_to_full_report_work, TIME_TO_FULL_REPORT_PERIOD);
			ttf->ttf_now = ERROR;
			return;
		} else
			soc = prop.intval;

		if ( !(chg_present && (0 <= soc && soc <= 1000)) || ttf->chg_complete)
			pr_ttf(PR_DEBUG, "Not Charging... \n");
		else
			ttf->ttf_now = lge_time_to_full_report(ttf);
#ifdef CONFIG_LGE_PM_CHARGERLOGO_WAIT_FOR_FG_INIT
	}
#endif
	schedule_delayed_work(&ttf->time_to_full_report_work, TIME_TO_FULL_REPORT_PERIOD);
}

static int ttf_power_get_property(struct power_supply *psy,
				 enum power_supply_property prop,
				 union power_supply_propval *val)
{
	struct lge_time_to_full_data *ttf = container_of(psy, struct lge_time_to_full_data, ttf_psy);
	int rc = 0;

	switch (prop) {
		case POWER_SUPPLY_PROP_TIME_TO_FULL:
			val->intval = ttf->ttf_now;
			break;
		default :
			pr_ttf(PR_ERROR, "Unsupported Property %d\n", prop);
			rc = -EINVAL;
			break;
	}

	if (rc < 0)
		return -ENODATA;

	return 0;
}

static void ttf_external_power_changed(struct power_supply *psy)
{
	struct lge_time_to_full_data *ttf = container_of(psy, struct lge_time_to_full_data, ttf_psy);
	union power_supply_propval prop = {0,};
	static int prev_soc_ttf;
	static int prev_soc;
	static bool prev_chg_present;
	static enum power_supply_type prev_usb_supply_type;
	int soc_ttf;
	int soc;
	int chg_status;
	bool chg_present;
	enum power_supply_type usb_supply_type;
	int rc = 0;

#ifdef CONFIG_LGE_PM_CHARGERLOGO_WAIT_FOR_FG_INIT
	if(!ttf->first_soc_est_done) {
		rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_FIRST_SOC_EST_DONE, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "bms_psy first soc est done read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
			return;
		}
		ttf->ttf_now = DISCHARGE;
		ttf->first_soc_est_done = prop.intval;
	}

	if (ttf->first_soc_est_done) {
#endif
		rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "usb_psy usb present read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
			return;
		}
		chg_present = prop.intval;

		rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_TIME_TO_FULL_CAPACITY, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "bms_psy time to full soc read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
			return;
		}
		soc_ttf = prop.intval;

		rc = ttf->bms_psy->get_property(ttf->bms_psy, POWER_SUPPLY_PROP_CAPACITY, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "bms_psy soc read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
			return;
		}
		soc = prop.intval;

		rc = ttf->batt_psy->get_property(ttf->batt_psy, POWER_SUPPLY_PROP_STATUS, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "batt_psy battery status read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
			return;
		}
		chg_status = prop.intval;

		rc = ttf->usb_psy->get_property(ttf->usb_psy, POWER_SUPPLY_PROP_TYPE, &prop);
		if (rc) {
			pr_ttf(PR_ERROR, "usb_psy usb type read fail rc = %d\n", rc);
			ttf->ttf_now = ERROR;
			return;
		}
		usb_supply_type = prop.intval;

		if (soc_ttf >= 1000 && chg_status == POWER_SUPPLY_STATUS_FULL)
			ttf->chg_complete = true;
		else
			ttf->chg_complete = false;

		if (chg_present && ttf->chg_complete) {
			if (chg_present != prev_chg_present)
				pr_ttf(PR_STATUS, "Charger Connected chg_present = %d, prev_chg_present = %d\n",
								chg_present, prev_chg_present);

			/* Evaluate TTF & avg current work cancel */
			if (soc != prev_soc) {
				pr_ttf(PR_STATUS, "Fully Charged...\n");
				lge_time_to_full_evaluate(ttf);
				lge_time_to_full_cc_table_recovery(ttf);
				cancel_delayed_work(&ttf->average_current_work);
				schedule_delayed_work(&ttf->average_current_work, 0);
				ttf->cv_soc_index = 0;
				ttf->ttf_now = FULL;
			}
			prev_chg_present = chg_present;
			prev_usb_supply_type = usb_supply_type;
			prev_soc_ttf = soc_ttf;
			prev_soc = soc;
			return;
		}

		if (chg_present) {
			if (chg_present != prev_chg_present || prev_usb_supply_type != usb_supply_type) {
				pr_ttf(PR_STATUS, "Charger Connected...\n");
				pr_ttf(PR_STATUS, "chg_present = %d, prev_chg_present = %d\n"
									"usb_type = %d, prev_usb_type = %d\n",
								chg_present, prev_chg_present,
								usb_supply_type, prev_usb_supply_type);

				cancel_delayed_work(&ttf->time_to_full_report_work);
				/* Delay to read charging current when charger is connected */
				msleep(500);
				ttf->ttf_now = lge_time_to_full_report(ttf);
				schedule_delayed_work(&ttf->time_to_full_report_work, TIME_TO_FULL_REPORT_PERIOD);
				average_current_work_queue(ttf);
				ttf->avg_calc_work = true;

				/* for evaluate */
				if (soc != prev_soc)
					lge_time_to_full_evaluate(ttf);
			} else {
				pr_ttf(PR_DEBUG, "Keep Charging...\n");
				if (soc_ttf != prev_soc_ttf) {
					pr_ttf(PR_INFO, "Keep Charging.... Update Time To Full\n");
					pr_ttf(PR_DEBUG, "soc = %d, prev_soc = %d\n", soc_ttf, prev_soc_ttf);
					cancel_delayed_work(&ttf->time_to_full_report_work);
					ttf->ttf_now = lge_time_to_full_report(ttf);
					schedule_delayed_work(&ttf->time_to_full_report_work, TIME_TO_FULL_REPORT_PERIOD);
				}

				/* for evaluate */
				if (soc != prev_soc)
					lge_time_to_full_evaluate(ttf);
			}
		} else {
			if (chg_present != prev_chg_present) {
				pr_ttf(PR_STATUS, "Charger Removed chg_present = %d, prev_chg_present = %d\n",
								chg_present, prev_chg_present);
				lge_time_to_full_cc_table_recovery(ttf);
				lge_time_to_full_evaluate_var_clear(ttf);
				cancel_delayed_work(&ttf->average_current_work);
				schedule_delayed_work(&ttf->average_current_work, 0);
				ttf->cv_soc_index = 0;
				ttf->chg_complete = false;
				ttf->ttf_now = DISCHARGE;
			} else {
				pr_ttf(PR_DEBUG, "Keep Discharging...\n");
				ttf->ttf_now = DISCHARGE;
			}
		}
#ifdef CONFIG_LGE_PM_CHARGERLOGO_WAIT_FOR_FG_INIT
	}
#endif

	prev_chg_present = chg_present;
	prev_usb_supply_type = usb_supply_type;
	prev_soc_ttf = soc_ttf;
	prev_soc = soc;
}

static int lge_time_to_full_member_init(struct lge_time_to_full_data *pdata)
{
	int i;

	/* dynamic cc step data initial */
	pdata->dynamic_cc_data = kzalloc(sizeof(struct cc_step) * pdata->cc_data_length, GFP_KERNEL);
	if (!pdata->dynamic_cc_data) {
		pr_ttf(PR_ERROR, "kzalloc failed for dynamic_cc_data\n");
		return -ENOMEM;
	}
	memcpy(pdata->dynamic_cc_data, pdata->cc_data, sizeof(struct cc_step) * pdata->cc_data_length);

	/* cc step time calculation */
	pdata->cc_step_time = kzalloc(sizeof(int) * pdata->cc_data_length, GFP_KERNEL);
	if (!pdata->cc_step_time) {
		pr_ttf(PR_ERROR, "kzalloc failed for cc_step_time\n");
		return -ENOMEM;
	}

	for (i = 1; i < pdata->cc_data_length; i++) {
		pdata->cc_step_time[i-1] = pdata->capacity * (pdata->cc_data[i].soc - pdata->cc_data[i-1].soc)\
						/ pdata->cc_data[i-1].cur * 3600 / 1000;
	}
	pdata->cc_step_time[pdata->cc_data_length-1] = pdata->capacity * (pdata->cv_data[0].soc - pdata->cc_data[pdata->cc_data_length-1].soc)\
						/ pdata->cc_data[pdata->cc_data_length-1].cur * 3600 / 1000;

	/* status check flag */
	pdata->chg_complete = false;
	pdata->avg_calc_work = false;
#ifdef CONFIG_LGE_PM_CHARGERLOGO_WAIT_FOR_FG_INIT
	pdata->first_soc_est_done = false;
#endif
	pdata->cv_soc_index = 0;
	pdata->ttf_now = EMPTY;

	/* for evaluation */
	for (i = 0; i < 100 ; i++) {
		pdata->runtime_consumed[i] = EMPTY;
		pdata->really_remained[i] = EMPTY;
		pdata->ttf_remained[i] = EMPTY;
		pdata->evaluate_native[i] = EMPTY;
		pdata->avg_cur[i] = EMPTY;
	}
	pdata->really_remained[100] = 0;
	pdata->starttime_of_charging = EMPTY;
	pdata->starttime_of_soc = EMPTY;
	pdata->soc_begin = EMPTY;
	pdata->soc_now = EMPTY;

	return 0;
}

static int lge_time_to_full_dt_to_pdata(struct platform_device *pdev,
					struct lge_time_to_full_data *pdata)
{
	struct device_node *node = (pdev) ? pdev->dev.of_node : NULL;
	struct device *dev_ttf = (pdev) ? &pdev->dev : NULL;
	struct device_node *child;
	const char *profile_name = NULL;
	int len = 0;
	int rc = 0;

	if (dev_ttf == NULL) {
		dev_err(dev_ttf, "failed to get platform_device data\n");
		return -EIO;
	}

	rc = of_property_read_u32(node, "lge,battery_full_capacity",
									&pdata->capacity);
	if (rc) {
		pr_ttf(PR_ERROR, "failed to read battery_full_capacity : %d\n", rc);
		pdata->capacity = DEFAULT_BATTERY_CAPACITY;
	}

	rc = of_property_read_u32(node, "lge,dcp_current",
								&pdata->dcp_current);
	if (rc) {
		pr_ttf(PR_ERROR, "failed to read dcp_current : %d\n", rc);
		pdata->dcp_current = DEFAULT_DCP_CURRENT;
	}

	pdata->hvdcp_supported =  of_property_read_bool(node, "lge,hvdcp-supported");
	if (pdata->hvdcp_supported) {
		rc = of_property_read_u32(node, "lge,hvdcp_current",
								&pdata->hvdcp_current);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read hvdcp_current : %d\n", rc);
			pdata->hvdcp_current = DEFAULT_HVDCP_CURRENT;
		}
	}

	pdata->hvdcp3_supported =  of_property_read_bool(node, "lge,hvdcp3-supported");
	if (pdata->hvdcp3_supported) {
		rc = of_property_read_u32(node, "lge,hvdcp3_current",
								&pdata->hvdcp3_current);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read hvdcp3_current : %d\n", rc);
			pdata->hvdcp3_current = DEFAULT_HVDCP_CURRENT;
		}
	}

#ifdef CONFIG_LGE_USB_TYPE_C
	pdata->typec_supported =  of_property_read_bool(node, "lge,typec-supported");
#endif

	pdata->wlc_supported =  of_property_read_bool(node, "lge,wlc-supported");
	if (pdata->wlc_supported) {
		rc = of_property_read_u32(node, "lge,wlc_current",
								&pdata->wlc_current);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read wlc_current : %d\n", rc);
			pdata->wlc_current = DEFAULT_WLC_CURRENT;
		}
	}

	pdata->parallel_supported =  of_property_read_bool(node, "lge,parallel-supported");

	rc = of_property_read_u32(node, "lge,avg_comp",
									&pdata->avg_comp);
	if (rc) {
			pr_ttf(PR_ERROR, "failed to read avg_comp : %d\n", rc);
			pdata->avg_comp = DEFAULT_COMP_VALUE;
	}

	rc = of_property_read_u32(node, "lge,report_ttf_comp",
									&pdata->report_ttf_comp);
	if (rc) {
			pr_ttf(PR_ERROR, "failed to read report_ttf_comp : %d\n", rc);
			pdata->report_ttf_comp = DEFAULT_COMP_VALUE;
	}

	for_each_child_of_node(node, child) {
		rc = of_property_read_string(child,
					"lge,profile_name", &profile_name);
		if (rc)
			pr_ttf(PR_ERROR, "failed to read battery profile name : %d", rc);
		else {
			pdata->profile_name = profile_name;
			pr_ttf(PR_INFO, "Default Profile Name : %s", pdata->profile_name);
		}

		if (of_get_property(child, "lge,cc_data", &len)) {
			pdata->cc_data = kzalloc(len, GFP_KERNEL);
			pdata->cc_data_length = len / sizeof(struct cc_step);
			rc = of_property_read_u32_array(child, "lge,cc_data",
						(u32 *)pdata->cc_data, len/sizeof(u32));
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read cc_data : %d\n", rc);
				kzfree(pdata->cc_data);
				pdata->cc_data = NULL;
			}
		} else {
			 pr_ttf(PR_ERROR, "there is not cc_data\n");
		}

		if (of_get_property(child, "lge,cv_data", &len)) {
			pdata->cv_data = kzalloc(len, GFP_KERNEL);
			pdata->cv_data_length = len / sizeof(struct cv_slope);
			rc = of_property_read_u32_array(child, "lge,cv_data",
						(u32 *)pdata->cv_data, len/sizeof(u32));
			if (rc) {
				pr_ttf(PR_ERROR, "failed to read cv_data : %d\n", rc);
				kzfree(pdata->cv_data);
				pdata->cv_data = NULL;
			}
		} else {
			 pr_ttf(PR_ERROR," there is not cv_data\n");
		}

		rc = of_property_read_u32(child, "lge,sdp_comp",
									&pdata->sdp_comp);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read sdp_comp : %d\n", rc);
			pdata->sdp_comp = DEFAULT_COMP_VALUE;
		}

		rc = of_property_read_u32(child, "lge,cdp_comp",
									&pdata->cdp_comp);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read cdp_comp : %d\n", rc);
			pdata->cdp_comp = DEFAULT_COMP_VALUE;
		}

		rc = of_property_read_u32(child, "lge,dcp_comp",
									&pdata->dcp_comp);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read dcp_comp : %d\n", rc);
			pdata->dcp_comp = DEFAULT_COMP_VALUE;
		}

		rc = of_property_read_u32(child, "lge,avg_sdp_comp",
									&pdata->avg_sdp_comp);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read avg_sdp_comp : %d\n", rc);
			pdata->avg_sdp_comp = DEFAULT_COMP_VALUE;
		}

		rc = of_property_read_u32(child, "lge,avg_cdp_comp",
									&pdata->avg_cdp_comp);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read avg_cdp_comp : %d\n", rc);
			pdata->avg_cdp_comp = DEFAULT_COMP_VALUE;
		}

		rc = of_property_read_u32(child, "lge,avg_dcp_comp",
									&pdata->avg_dcp_comp);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read avg_dcp_comp : %d\n", rc);
			pdata->avg_dcp_comp = DEFAULT_COMP_VALUE;
		}

		rc = of_property_read_u32(child, "lge,min_comp",
									&pdata->min_comp);
		if (rc) {
			pr_ttf(PR_ERROR, "failed to read min_comp : %d\n", rc);
			pdata->min_comp = DEFAULT_COMP_VALUE;
		}
		break;
	}
	return 0;
}

static struct of_device_id lge_time_to_full_match_table[] = {
	{ .compatible = "lge,time-to-full" },
	{}
};

static int lge_time_to_full_probe(struct platform_device *pdev)
{
	struct lge_time_to_full_data *ttf;
	struct power_supply *usb_psy, *batt_psy, * bms_psy = NULL;
	int rc;

	dev_info(&pdev->dev, "lge_time_to_full_probe : Start\n");

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_ttf(PR_ERROR, "batt_psy not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bms");
	if(!bms_psy) {
		pr_ttf(PR_ERROR, "bms_psy not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_ttf(PR_ERROR, "usb_psy not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	ttf = kzalloc(sizeof(struct lge_time_to_full_data), GFP_KERNEL);
	if(!ttf) {
			pr_ttf(PR_ERROR, "Fail to get *ttf.\n");
			return -ENOMEM;
	}

	rc = lge_time_to_full_dt_to_pdata(pdev, ttf);
	if (rc)
		goto err;

	platform_set_drvdata(pdev, ttf);

	rc = lge_time_to_full_member_init(ttf);
	if (rc)
		goto err;

	ttf->dev = &pdev->dev;

	ttf->usb_psy = usb_psy;
	ttf->batt_psy = batt_psy;
	ttf->bms_psy = bms_psy;

	ttf->ttf_psy.name = "ttf";
	ttf->ttf_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	ttf->ttf_psy.properties = ttf_power_props;
	ttf->ttf_psy.num_properties = ARRAY_SIZE(ttf_power_props);
	ttf->ttf_psy.get_property = ttf_power_get_property;
	ttf->ttf_psy.external_power_changed = ttf_external_power_changed;
	ttf->ttf_psy.supplied_from = ttf_supplies;
	ttf->ttf_psy.num_supplies = ARRAY_SIZE(ttf_supplies);

	rc = power_supply_register(&pdev->dev, &ttf->ttf_psy);
	if (rc < 0) {
			pr_ttf(PR_ERROR, "power supply register fail rc = %d\n", rc);
			goto err;
	}

/* debugging */
	the_ttf = ttf;
/* debugging */
	INIT_DELAYED_WORK(&ttf->time_to_full_report_work, lge_time_to_full_report_work);
	INIT_DELAYED_WORK(&ttf->average_current_work, lge_time_to_full_average_current_work);
	INIT_DELAYED_WORK(&ttf->battery_profile_read_work, lge_battery_profile_read_work);

	schedule_delayed_work(&ttf->time_to_full_report_work, TIME_TO_FULL_REPORT_PERIOD);
	schedule_delayed_work(&ttf->battery_profile_read_work, BATT_PROFILE_READ_TIME);

	pr_ttf(PR_STATUS,"Done\n");
	return 0;

err:
	power_supply_unregister(&ttf->ttf_psy);
	kzfree(ttf->cc_data);
	kzfree(ttf->cc_step_time);
	kzfree(ttf->dynamic_cc_data);
	kzfree(ttf->cv_data);
	kzfree(ttf);
	return rc;
}

static int lge_time_to_full_remove(struct platform_device *pdev)
{
	struct lge_time_to_full_data *ttf =
		(struct lge_time_to_full_data *)platform_get_drvdata(pdev);

	power_supply_unregister(&ttf->ttf_psy);
	cancel_delayed_work(&ttf->time_to_full_report_work);
	cancel_delayed_work(&ttf->average_current_work);
	cancel_delayed_work(&ttf->battery_profile_read_work);
	kzfree(ttf->cc_data);
	kzfree(ttf->cc_step_time);
	kzfree(ttf->dynamic_cc_data);
	kzfree(ttf->cv_data);
	kzfree(ttf);
	return 0;
}

#if defined(CONFIG_PM)
static int lge_time_to_full_suspend(struct device *dev)
{
	return 0;
}

static int lge_time_to_full_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops lge_time_to_full_dev_pm_ops = {
	.suspend_noirq = lge_time_to_full_suspend,
	.resume_noirq = lge_time_to_full_resume,
};
#endif

static struct platform_driver lge_time_to_full_driver = {
	.probe = lge_time_to_full_probe,
	.remove = lge_time_to_full_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm = &lge_time_to_full_dev_pm_ops,
#endif
		.of_match_table = lge_time_to_full_match_table,
	},
};

static int __init lge_time_to_full_init(void)
{
	return platform_driver_register(&lge_time_to_full_driver);
}

static void __exit lge_time_to_full_exit(void)
{
	platform_driver_unregister(&lge_time_to_full_driver);
}

late_initcall(lge_time_to_full_init);
module_exit(lge_time_to_full_exit);
MODULE_DESCRIPTION("LGE time to full driver");
MODULE_LICENSE("GPL v2");
