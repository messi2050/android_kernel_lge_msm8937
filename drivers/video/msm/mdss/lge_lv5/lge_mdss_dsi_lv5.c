#include <linux/delay.h>
#include "mdss_dsi.h"
#include "../lge/mfts_mode.h"
#include <linux/mfd/dw8768.h>

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
#include <soc/qcom/lge/board_lge.h>
#endif

#include <linux/input/lge_touch_notify.h>
enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

/* For Sending MIPI Commands */
struct mdss_panel_data *lgd_lg4894_pdata_base;
EXPORT_SYMBOL(lgd_lg4894_pdata_base);

/* Touch LPWG Status */
static unsigned int pre_panel_mode = LCD_MODE_STOP;
static unsigned int cur_panel_mode = LCD_MODE_STOP;

bool set_touch_osc_flag = false;
static struct mutex set_touch_osc_mutex;
static bool set_touch_osc_mutex_init = false;

static bool mdss_shutdown_state = false;

static void init_set_touch_osc_mutex(void)
{
	if (set_touch_osc_mutex_init)
		return;

	mutex_init(&set_touch_osc_mutex);
	set_touch_osc_mutex_init = true;
}

void lock_set_touch_osc(void)
{
	init_set_touch_osc_mutex();
	mutex_lock(&set_touch_osc_mutex);
}

void unlock_set_touch_osc(void)
{
	mutex_unlock(&set_touch_osc_mutex);
}

int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	int panel_type;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	panel_type = lge_get_panel_type();

	switch (event) {
	case MDSS_EVENT_RESET:
		if (panel_type == LV5_LGD) {
			set_touch_osc_flag = false;
			lock_set_touch_osc();
			unlock_set_touch_osc();
		}
		break;
	case MDSS_EVENT_POST_PANEL_ON:
		if (panel_type == LV5_LGD) {
			lgd_lg4894_pdata_base = pdata;
			cur_panel_mode = LCD_MODE_U3;
			pr_info("%s: event=MDSS_EVENT_POST_PANEL_ON panel_mode=%d,%d\n",
				__func__, pre_panel_mode, cur_panel_mode);
		}
		break;
	case MDSS_EVENT_PANEL_OFF:
		if (lge_get_panel_type() == LV5_LGD) {
			set_touch_osc_flag = true;
			cur_panel_mode = LCD_MODE_U0;
			pr_info("%s: event=MDSS_EVENT_PANEL_OFF panel_mode=%d,%d\n",
				__func__, pre_panel_mode, cur_panel_mode);
		}
		break;
	default:
		pr_info("%s: nothing to do about this event=%d\n", __func__, event);
	}

	if (lge_get_panel_type() == LV5_LGD) {
		if (pre_panel_mode != cur_panel_mode) {
			rc = touch_notifier_call_chain(LCD_EVENT_LCD_MODE, (void *)&cur_panel_mode);
			pre_panel_mode = cur_panel_mode;
		}
	}
	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON) || IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (lge_get_display_power_ctrl()) {
		pr_info("%s: turn panel power on\n", __func__);

		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 1);
		usleep_range(50000, 50000);

		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);
		if (ret) {
			pr_err("%s: failed to enable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
			return ret;
		}
		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 1);

		if (dw8768_set_output_voltage(0x0E))
			pr_err("%s: setting DSV output voltage failed\n", __func__);
	} else if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		if (lge_get_panel_type() == LV5_LGD) {
			lge_extra_gpio_set_value(ctrl_pdata, "vdda", 1);
			LGE_MDELAY(10);
			lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 1);

			dw8768_register_set(0x05, 0x0F);
			LGE_MDELAY(1);
			dw8768_register_set(0x00, 0x0D);
			dw8768_register_set(0x01, 0x14);

			pr_info("%s: set dsv setting in charger mode\n", __func__);
		}
	} else {
		pr_info("%s: skip panel power control\n", __func__);
	}

	/*
	 * If continuous splash screen feature is enabled, then we need to
	 * request all the GPIOs that have already been configured in the
	 * bootloader. This needs to be done irresepective of whether
	 * the lp11_init flag is set or not.
	 */
	if (pdata->panel_info.cont_splash_enabled ||
		!pdata->panel_info.mipi.lp11_init) {
		if (mdss_dsi_pinctrl_set_state(ctrl_pdata, true))
			pr_debug("reset enable: pinctrl not enabled\n");

		ret = mdss_dsi_panel_reset(pdata, 1);
		if (ret)
			pr_err("%s: Panel reset failed. rc=%d\n",
					__func__, ret);
	}

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto end;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	if (lge_get_display_power_ctrl()) {
		pr_info("%s: turn panel power off\n", __func__);

		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);

		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);
		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
	}  else if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		if (lge_get_panel_type() == LV5_LGD) {
			dw8768_register_set(0x03, 0x80);
			dw8768_register_set(0x05, 0x07);
			lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);
			LGE_MDELAY(1);
			lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);
			pr_info("%s: disable dsv in charger mode\n", __func__);
		}
	} else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
	}
end:
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
        int rc = 0;

        rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
        if (rc) {
                pr_err("request reset gpio failed, rc=%d\n",
                        rc);
        }

	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);
	if ((mdss_dsi_is_right_ctrl(ctrl_pdata) &&
		mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) ||
			pinfo->is_dba_panel) {
		pr_debug("%s:%d, right ctrl gpio configuration not needed\n",
			__func__, __LINE__);
		return rc;
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}

		if (!pinfo->cont_splash_enabled) {
			if (lge_get_panel_type() == LV5_LGD) {
				touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
			}
			pr_info("%s: panel reset high\n", __func__);

			if (pdata->panel_info.rst_seq_len) {
				rc = gpio_direction_output(ctrl_pdata->rst_gpio,
					pdata->panel_info.rst_seq[0]);
				if (rc) {
					pr_err("%s: unable to set dir for rst gpio\n",
						__func__);
					goto exit;
				}
			}

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio), pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}

			if (lge_get_panel_type() == LV5_LGD) {
				touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
				LGE_MDELAY(50);
			}
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (lge_get_display_power_ctrl()) {
			pr_info("%s: set panel reset low\n", __func__);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
		} else if (mdss_shutdown_state) {
			pr_info("%s: device is shutting down, set panel reset low\n", __func__);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			LGE_MDELAY(5);
			touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);
		} else {
			pr_info("%s: skip to set panel reset low\n", __func__);

		}
		gpio_free(ctrl_pdata->rst_gpio);
	}

exit:
	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
    int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	pr_info("%s: + (override: lv5)\n", __func__);
	lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);
	LGE_MDELAY(5);

	mdss_shutdown_state = true;
	ret = mdss_dsi_panel_reset(lgd_lg4894_pdata_base, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	LGE_MDELAY(1);

	lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);

	LGE_MDELAY(1);

	pr_info("%s: -\n", __func__);
	return;
}
#endif
