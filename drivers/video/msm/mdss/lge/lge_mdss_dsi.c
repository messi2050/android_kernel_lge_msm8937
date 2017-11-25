#include <linux/of_gpio.h>
#include "mdss_dsi.h"
#include "lge_mdss_dsi.h"
#if IS_ENABLED(CONFIG_LGE_DISPLAY_MFTS)
#include "mfts_mode.h"
#endif

static int lge_mdss_dsi_parse_gpio_params(struct platform_device *ctrl_pdev,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc, i;
	const char *name;
	char buf[256];

	rc = of_property_count_strings(ctrl_pdev->dev.of_node, "lge,extra-gpio-names");
	if (rc > 0) {
		ctrl_pdata->lge_extra.num_gpios = rc;
		pr_info("%s: num_gpios=%d\n", __func__, ctrl_pdata->lge_extra.num_gpios);
		ctrl_pdata->lge_extra.gpio_array = kmalloc(sizeof(struct lge_gpio_entry)*ctrl_pdata->lge_extra.num_gpios, GFP_KERNEL);
		if (NULL == ctrl_pdata->lge_extra.gpio_array) {
			pr_err("%s: no memory\n", __func__);
			ctrl_pdata->lge_extra.num_gpios = 0;
			return -ENOMEM;
		}
		for (i = 0; i < ctrl_pdata->lge_extra.num_gpios; ++i) {
			of_property_read_string_index(ctrl_pdev->dev.of_node, "lge,extra-gpio-names", i, &name);
			strlcpy(ctrl_pdata->lge_extra.gpio_array[i].name, name, sizeof(ctrl_pdata->lge_extra.gpio_array[i].name));
			snprintf(buf, sizeof(buf), "lge,gpio-%s", name);
			ctrl_pdata->lge_extra.gpio_array[i].gpio = of_get_named_gpio(ctrl_pdev->dev.of_node, buf, 0);
			if (!gpio_is_valid(ctrl_pdata->lge_extra.gpio_array[i].gpio))
				pr_err("%s: %s not specified\n", __func__, buf);
		}
	} else {
		ctrl_pdata->lge_extra.num_gpios = 0;
		pr_info("%s: no lge specified gpio\n", __func__);
	}
	return 0;
}

int lge_mdss_dsi_parse_extra_params(struct platform_device *ctrl_pdev,
        struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	lge_mdss_dsi_parse_gpio_params(ctrl_pdev, ctrl_pdata);

	return 0;
}

extern int mdss_dsi_get_dt_vreg_data(struct device *dev,
        struct device_node *of_node, struct dss_module_power *mp,
        enum dsi_pm_type module);

int lge_mdss_dsi_init_extra_pm(struct platform_device *ctrl_pdev,
	struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc;

        rc = mdss_dsi_get_dt_vreg_data(&ctrl_pdev->dev, pan_node,
		&ctrl_pdata->lge_extra.extra_power_data, DSI_EXTRA_PM);
        if (rc) {
		DEV_ERR("%s: '%s' get_dt_vreg_data failed.rc=%d\n",
			__func__, __mdss_dsi_pm_name(DSI_EXTRA_PM), rc);
                return 0;
        }

        rc = msm_dss_config_vreg(&ctrl_pdev->dev,
		ctrl_pdata->lge_extra.extra_power_data.vreg_config,
		ctrl_pdata->lge_extra.extra_power_data.num_vreg, 1);
        if (rc) {
                pr_err("%s: failed to init regulator, rc=%d\n", __func__, rc);
        }

	return rc;
}

extern void mdss_dsi_put_dt_vreg_data(struct device *dev,
        struct dss_module_power *module_power);

void lge_mdss_dsi_deinit_extra_pm(struct platform_device *pdev,
        struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (msm_dss_config_vreg(&pdev->dev,
                        ctrl_pdata->lge_extra.extra_power_data.vreg_config,
                        ctrl_pdata->lge_extra.extra_power_data.num_vreg, 1) < 0)
                pr_err("%s: failed to de-init vregs for %s\n",
                                __func__, __mdss_dsi_pm_name(DSI_EXTRA_PM));
        mdss_dsi_put_dt_vreg_data(&pdev->dev, &ctrl_pdata->lge_extra.extra_power_data);
}

static int gpio_name_to_index(struct mdss_dsi_ctrl_pdata *ctrl_pdata, const char *name)
{
	int i, index = -1;

	for (i = 0; i < ctrl_pdata->lge_extra.num_gpios; ++i) {
		if (!strcmp(ctrl_pdata->lge_extra.gpio_array[i].name, name)) {
			index = i;
			break;
		}
	}

	return index;
}

void lge_extra_gpio_set_value(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	const char *name, int value)
{
	int index = -1;

	index = gpio_name_to_index(ctrl_pdata, name);

	if (index != -1) {
		gpio_set_value(ctrl_pdata->lge_extra.gpio_array[index].gpio, value);
	} else {
		pr_err("%s: couldn't get gpio by name %s\n", __func__, name);
	}
}

int lge_extra_gpio_request(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	const char *name)
{
	int index = -1;

	index = gpio_name_to_index(ctrl_pdata, name);

	if (index != -1) {
		return gpio_request(ctrl_pdata->lge_extra.gpio_array[index].gpio, name);
	} else {
		pr_err("%s: couldn't get gpio by name %s\n", __func__, name);
		return -EINVAL;
	}
}

void lge_extra_gpio_free(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	const char *name)
{
	int index = -1;

	index = gpio_name_to_index(ctrl_pdata, name);

	if (index != -1) {
		gpio_free(ctrl_pdata->lge_extra.gpio_array[index].gpio);
	} else {
		pr_err("%s: couldn't get gpio by name %s\n", __func__, name);
	}
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_TOUCH_NOTIFIER_CALL_CHAIN)
/*
 * lge_get_lpwg_on_event(), lge_get_lpwg_off_event() returns
 * one of below values:
 *  for LCD_EVENT_TOUCH_LPWG_OFF
 *    MDSS_EVENT_PANEL_ON
 *    MDSS_EVENT_LINK_READY
 *  for LCD_EVENT_TOUCH_LPWG_ON
 *    MDSS_EVENT_PANEL_OFF
 *    MDSS_EVENT_BLANK
 */
__weak int lge_get_lpwg_on_event(void)
{
	return MDSS_EVENT_PANEL_OFF;
}

__weak int lge_get_lpwg_off_event(void)
{
	return MDSS_EVENT_PANEL_ON;
}
#endif
__weak int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	return 0;
}
#if IS_ENABLED(CONFIG_LGE_DISPLAY_PRE_ACTIVE_AREA_DELAY)
void lge_mdss_dsi_calc_pre_active_area_delay(struct mdss_panel_info *pinfo)
{
	u64 clk_rate;
	u32 clk_period, time_of_line;

	clk_rate = pinfo->mipi.dsi_pclk_rate;
	clk_rate = DIV_ROUND_UP_ULL(clk_rate, 1000); /* in kHz */
	clk_period = DIV_ROUND_UP_ULL(1000000000, clk_rate);
	time_of_line = (pinfo->lcdc.h_back_porch + pinfo->lcdc.h_front_porch + pinfo->lcdc.h_pulse_width + pinfo->xres) * clk_period;
	pinfo->pre_active_area_delay_us = (time_of_line * (pinfo->lcdc.v_pulse_width + pinfo->lcdc.v_back_porch)) / 1000000;
	pinfo->pre_active_area_delay_us += 1000; /* add 1ms for margin */
	pr_info("%s: %d us\n", __func__, pinfo->pre_active_area_delay_us);
}
#endif

__weak int lge_get_panel_recovery_flag()
{
	return 0;
}

__weak void lge_set_panel_recovery_flag(int flag)
{
}

int lge_mdss_dsi_panel_power_seq_all() {
	int ret = 0;
#if IS_ENABLED(CONFIG_LGE_DISPLAY_MFTS)
	if (lge_get_display_power_ctrl())
		ret = 1;
#endif

	if (lge_get_panel_recovery_flag())
		ret = 1;

	return ret;
}