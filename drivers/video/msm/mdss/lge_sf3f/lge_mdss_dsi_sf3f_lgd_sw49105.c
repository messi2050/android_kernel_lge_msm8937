#include <linux/delay.h>
#include "mdss_dsi.h"
#include <linux/mfd/sm5107.h>
#include "lge/mfts_mode.h"

int sf3f_lgd_sw49105_gpio_power_ctrl(struct mdss_panel_data *pdata, int enable);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON) || IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int sf3f_lgd_sw49105_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_info("%s ++\n", __func__);

	if (lge_get_display_power_ctrl()) {
		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);
		if (ret) {
			pr_err("%s: failed to enable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
			return ret;
		}

		ret = sf3f_lgd_sw49105_gpio_power_ctrl(pdata, 1);
		if (ret)
			pr_err("%s: Panel power on failed. rc=%d\n",
					__func__, ret);
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

	pr_info("%s --\n", __func__);

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int  sf3f_lgd_sw49105_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
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

	pr_info("%s ++\n", __func__);

	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	if (lge_get_display_power_ctrl()) {
		ret = msm_dss_enable_vreg(
				ctrl_pdata->panel_power_data.vreg_config,
				ctrl_pdata->panel_power_data.num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs for %s\n",
						__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));

		ret = sf3f_lgd_sw49105_gpio_power_ctrl(pdata, 0);
		if (ret)
			pr_err("%s: Panel power off failed. rc=%d\n",
					__func__, ret);
	} else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
	}

	pr_info("%s --\n", __func__);

end:
	return ret;
}
#endif

static int sf3f_lgd_sw49105_mdss_dsi_request_power_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	rc = lge_extra_gpio_request(ctrl_pdata, "iovcc");
	if (rc) {
		pr_err("request iovcc gpio failed, rc=%d\n",
			rc);
		goto iovcc_gpio_err;
	}

	rc = lge_extra_gpio_request(ctrl_pdata, "dsv_ena");
	if (rc) {
		pr_err("request dsv_ena gpio failed, rc=%d\n",
			rc);
		goto ena_gpio_err;
	}

	return rc;
ena_gpio_err:
	lge_extra_gpio_free(ctrl_pdata, "iovcc");
iovcc_gpio_err:
	return rc;
}


static int sf3f_lgd_sw49105_mdss_dsi_request_reset_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}

rst_gpio_err:
	return rc;
}


int sf3f_lgd_sw49105_gpio_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	pr_info("%s: enable = %d\n", __func__, enable);

	if (enable) {
		rc = sf3f_lgd_sw49105_mdss_dsi_request_power_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {

			pr_info("%s: turn panel power on\n", __func__);

			lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 1);
			pr_info("%s: turn on vddi\n", __func__);
			mdelay(10);

			lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 1);
			mdelay(1);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
			ext_dsv_mode_change(MODE_NORMAL); // DSV_ENREG enable
			pr_info("%s: turn on dsv\n", __func__);
			mdelay(4);
#endif
		}
	} else {
		pr_info("%s: turn panel power off\n", __func__);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
		//ext_dsv_mode_change(POWER_OFF);
		ext_dsv_mode_change(MODE_KNOCKON);  //DSV HIZ KNOCK_ON mode
#endif
		//lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 0); //DSV low
		lge_extra_gpio_free(ctrl_pdata, "dsv_ena");
		pr_info("%s: turn off dsv\n", __func__);
		mdelay(5);

		lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);   //iovcc low
		lge_extra_gpio_free(ctrl_pdata, "iovcc");
		pr_info("%s: turn off vddi\n", __func__);
		mdelay(1);
	}

	pr_info("%s: -\n", __func__);

	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
int sf3f_lgd_sw49105_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
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

	pr_info("%s: enable = %d\n", __func__, enable);

	if (enable) {
		rc = sf3f_lgd_sw49105_mdss_dsi_request_reset_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
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

		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		pr_info("%s: set panel reset low\n", __func__);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		mdelay(5);

		gpio_free(ctrl_pdata->rst_gpio);
	}

	pr_info("%s: -\n", __func__);
exit:
	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void sf3f_lgd_sw49105_mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	if(gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_info("%s: reset to low\n", __func__);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);		//Reset low
	}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
	ext_dsv_mode_change(POWER_OFF);  					//Disable DSV_EN
#endif

	lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 0);	//DSV low
	usleep_range(10000, 10000);

	lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);	//iovcc low

	pr_info("%s: turn panel shutdown\n", __func__);

	return;

}
#endif
