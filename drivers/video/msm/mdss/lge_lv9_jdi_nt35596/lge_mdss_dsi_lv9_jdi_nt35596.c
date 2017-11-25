#include <linux/delay.h>
#include "mdss_dsi.h"
#include <linux/mfd/sm5107.h>
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
#include "mdss_mdp.h"
#include "lge/reader_mode.h"
#endif
#include "lge/mfts_mode.h"

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
extern int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

#define IE_MONO_BIT      0
#define IE_DAYLIGHT_BIT  1

#define IE_MONO_ENABLE_IDX     1
#define IE_DAYLIGHT_ENABLE_IDX 2
#define VALUE_POS 1

static u32 ie_status = 0;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
static struct dsi_panel_cmds normal_mode_initial_gamma_cmds;
static struct dsi_panel_cmds reader_mode_initial_step1_cmds;
static struct dsi_panel_cmds reader_mode_initial_step2_cmds;
static struct dsi_panel_cmds reader_mode_initial_step3_cmds;
static struct dsi_panel_cmds reader_mode_initial_ie_enable_cmds;
static struct dsi_panel_cmds reader_mode_step1_cmds;
static struct dsi_panel_cmds reader_mode_step2_cmds;
static struct dsi_panel_cmds reader_mode_step3_cmds;
static struct dsi_panel_cmds reader_mode_off_cmds;
static struct dsi_panel_cmds reader_mode_ie_enable_cmds;
static struct dsi_panel_cmds reader_mode_ie_disable_cmds;
#endif

#if defined(CONFIG_LGE_DISPLAY_DAYLIGHT_MODE)
enum DAYLIGHT_MODE {
	DAYLIGHT_MODE_OFF = 0,
	DAYLIGHT_MODE_LOW,
	DAYLIGHT_MODE_MID,
	DAYLIGHT_MODE_HIGH,
	DAYLIGHT_MODE_MAX,
};
#endif

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

#if defined(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
int esd_detected = 0;  // to avoid build error, no necessary to control this variable
					   // because ESD is not detected by touch firmware.
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}

	rc = lge_extra_gpio_request(ctrl_pdata, "vddio");
	if (rc) {
		pr_err("request vddio gpio failed, rc=%d\n",
			rc);
		goto vddio_gpio_err;
	}

	rc = lge_extra_gpio_request(ctrl_pdata, "avdd");
	if (rc) {
		pr_err("request avdd gpio failed, rc=%d\n",
			rc);
		goto avdd_gpio_err;
	}

	rc = lge_extra_gpio_request(ctrl_pdata, "dsv_ena");
	if (rc) {
		pr_err("request dsv_ena gpio failed, rc=%d\n",
			rc);
		goto dsv_ena_gpio_err;
	}

	return rc;

dsv_ena_gpio_err:
	lge_extra_gpio_free(ctrl_pdata, "avdd");
avdd_gpio_err:
	lge_extra_gpio_free(ctrl_pdata, "vddio");
vddio_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:

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

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);
	if ((mdss_dsi_is_right_ctrl(ctrl_pdata) &&
		mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) ||
			pinfo->is_dba_panel) {
		pr_debug("%s:%d, right ctrl gpio configuration not needed\n",
			__func__, __LINE__);
		return rc;
	}
	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n", __func__, __LINE__);
		return rc;
	}

	pr_info("%s: + enable = %d (override: lv9)\n", __func__, enable);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("request gpio failed, rc=%d\n", rc);
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

			if(lge_get_display_power_ctrl()) {
				pr_info("%s: turn panel power on\n", __func__);
				lge_extra_gpio_set_value(ctrl_pdata, "avdd", 1);
				usleep_range(1000, 1000);
				lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
				usleep_range(1000, 1000);
				lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 1);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
				ext_dsv_mode_change(MODE_NORMAL); // DSV_ENREG enabled
				usleep_range(15000, 15000);
#endif
			} else {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
				ext_dsv_mode_change(MODE_NORMAL); // DSV_ENREG enabled
#endif
				pr_info("%s: skip panel power control\n", __func__);
			}

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
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
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
		ext_dsv_mode_change(POWER_OFF);	//DSV external pin control, active-discharge enable
#endif

		if(lge_get_display_power_ctrl()) {
			pr_info("%s: turn panel power off\n", __func__);

			gpio_set_value(ctrl_pdata->rst_gpio, 0);
			usleep_range(5000, 5000);

			lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 0); // DSV low
			usleep_range(10000, 10000);

			lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0); // vddio low
			usleep_range(1000, 1000);
			lge_extra_gpio_set_value(ctrl_pdata, "avdd", 0); // avdd low
		}

		lge_extra_gpio_free(ctrl_pdata, "dsv_ena");
		lge_extra_gpio_free(ctrl_pdata, "avdd");
		lge_extra_gpio_free(ctrl_pdata, "vddio");
		gpio_free(ctrl_pdata->rst_gpio);
	}

	pr_info("%s: -\n", __func__);
exit:
	return rc;
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

	pr_info("%s: (override: lv9)\n", __func__);

	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	if(lge_get_display_power_ctrl()) {
		ret = msm_dss_enable_vreg(
				ctrl_pdata->panel_power_data.vreg_config,
				ctrl_pdata->panel_power_data.num_vreg, 0);

		if (ret)
			pr_err("%s: failed to disable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
	} else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
	}

end:
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_info("%s: (override: lv9)\n", __func__);

	if(lge_get_display_power_ctrl()) {
		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);

		if (ret) {
			pr_err("%s: failed to enable vregs for %s\n",
					__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
			return ret;
		}
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

#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
int lge_mdss_dsi_parse_reader_mode_cmds(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_dsi_parse_dcs_cmds(np, &normal_mode_initial_gamma_cmds,
		"qcom,panel-normal-mode-initial-gamma-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step1_cmds,
		"qcom,panel-reader-mode-initial-step1-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step2_cmds,
		"qcom,panel-reader-mode-initial-step2-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step3_cmds,
		"qcom,panel-reader-mode-initial-step3-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_ie_enable_cmds,
		"qcom,panel-ie-enable-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step1_cmds,
		"qcom,panel-reader-mode-step1-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step2_cmds,
		"qcom,panel-reader-mode-step2-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step3_cmds,
		"qcom,panel-reader-mode-step3-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_off_cmds,
		"qcom,panel-reader-mode-off-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_ie_enable_cmds,
		"qcom,panel-ie-enable-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_ie_disable_cmds,
		"qcom,panel-ie-disable-command", "qcom,mdss-dsi-reader-mode-command-state");

	return 0;
}

static void lge_mdss_dsi_send_ie_cmds(struct mdss_dsi_ctrl_pdata *ctrl)
{
	pr_debug("%s: ie_status : 0x%x\n", __func__, ie_status);
	if(!(ie_status & 0x03)) {  // DAYLIGHT OFF, MONO OFF
		mdss_dsi_panel_cmds_send(ctrl, &reader_mode_ie_disable_cmds, CMD_REQ_COMMIT);
	}
	else {
		reader_mode_ie_enable_cmds.cmds[IE_MONO_ENABLE_IDX].payload[VALUE_POS] = (ie_status & (1 <<IE_MONO_BIT))?1:0;
		reader_mode_ie_enable_cmds.cmds[IE_DAYLIGHT_ENABLE_IDX].payload[VALUE_POS] = (ie_status & (1 << IE_DAYLIGHT_BIT))?1:0;
		mdss_dsi_panel_cmds_send(ctrl, &reader_mode_ie_enable_cmds, CMD_REQ_COMMIT);
	}
}

int lge_mdss_dsi_panel_send_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *default_on_cmds, int cur_mode)
{
	pr_info("%s: reader_mode (%d).\n", __func__, cur_mode);
	ie_status &= ~BIT(IE_DAYLIGHT_BIT);  // daylight bit clear for display on

	switch(cur_mode)
	{
		case READER_MODE_STEP_1:
			pr_info("%s: reader_mode STEP1\n", __func__);
			if (reader_mode_initial_step1_cmds.cmd_cnt) {
				pr_info("%s: reader_mode_initial_step1_cmds: cnt = %d \n",
					__func__, reader_mode_initial_step1_cmds.cmd_cnt);
				mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_step1_cmds, CMD_REQ_COMMIT);
			}
			break;
		case READER_MODE_STEP_2:
			pr_info("%s: reader_mode STEP2\n", __func__);
			if (reader_mode_initial_step2_cmds.cmd_cnt) {
				pr_info("%s: reader_mode_initial_step2_cmds: cnt = %d \n",
					__func__, reader_mode_initial_step2_cmds.cmd_cnt);
				mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_step2_cmds, CMD_REQ_COMMIT);
			}
			break;
		case READER_MODE_STEP_3:
			pr_info("%s: reader_mode STEP3\n", __func__);
			if (reader_mode_initial_step3_cmds.cmd_cnt) {
				pr_info("%s: reader_mode_initial_step3_cmds: cnt = %d \n",
					__func__, reader_mode_initial_step3_cmds.cmd_cnt);
				mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_step3_cmds, CMD_REQ_COMMIT);
			}
			break;
		case READER_MODE_MONO:
			pr_info("%s: reader_mode MONO \n", __func__);
			/* 5500K */
			if (reader_mode_initial_step2_cmds.cmd_cnt) {
				pr_info("%s: reader_mode_initial_step2_cmds: cnt = %d \n",
					__func__, reader_mode_initial_step2_cmds.cmd_cnt);
				mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_step2_cmds, CMD_REQ_COMMIT);
			}
			if (default_on_cmds->cmd_cnt)
				mdss_dsi_panel_cmds_send(ctrl, default_on_cmds, CMD_REQ_COMMIT);
			if (reader_mode_initial_ie_enable_cmds.cmd_cnt) {
				pr_info("%s: reader_mode_initial_ie_enable_cmds: cnt = %d \n",
					__func__, reader_mode_initial_ie_enable_cmds.cmd_cnt);
				mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_ie_enable_cmds, CMD_REQ_COMMIT);
			}
			break;
		case READER_MODE_OFF:
		default:
			if (normal_mode_initial_gamma_cmds.cmd_cnt) {
				pr_debug("%s: normal_mode_initial_gamma_cmds\n", __func__);
				mdss_dsi_panel_cmds_send(ctrl, &normal_mode_initial_gamma_cmds, CMD_REQ_COMMIT);
			}
			break;
	}

	if (default_on_cmds->cmd_cnt && cur_mode != READER_MODE_MONO) {
		pr_debug("%s: default on cmds\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, default_on_cmds, CMD_REQ_COMMIT);
	}

	return 0;
}

bool lge_change_reader_mode(struct mdss_dsi_ctrl_pdata *ctrl, int old_mode, int new_mode)
{
	if(new_mode == READER_MODE_OFF) {
		switch(old_mode)
		{
			case READER_MODE_STEP_1:
			case READER_MODE_STEP_2:
			case READER_MODE_STEP_3:
				if (reader_mode_off_cmds.cmd_cnt) {
					pr_info("%s: sending reader mode OFF commands: cnt = %d\n",
						__func__, reader_mode_off_cmds.cmd_cnt);
					mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_off_cmds, CMD_REQ_COMMIT);
					mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
				}
			    break;
			case READER_MODE_MONO:
				if (reader_mode_ie_disable_cmds.cmd_cnt || reader_mode_off_cmds.cmd_cnt) {
					mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
				}
				if (reader_mode_off_cmds.cmd_cnt) {
					pr_info("%s: sending reader mode OFF commands: cnt = %d\n",
						__func__, reader_mode_off_cmds.cmd_cnt);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_off_cmds, CMD_REQ_COMMIT);
				}
				ie_status &= ~BIT(IE_MONO_BIT);  // monochrome bit clear
				lge_mdss_dsi_send_ie_cmds(ctrl);
				if (reader_mode_ie_disable_cmds.cmd_cnt || reader_mode_off_cmds.cmd_cnt) {
					mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
				}
				break;
			default:
				pr_err("%s: Invalid old status : %d\n", __func__, old_mode);
				break;
		}
	} else {
		switch(old_mode)
		{
			case READER_MODE_OFF:
			case READER_MODE_STEP_1:
			case READER_MODE_STEP_2:
			case READER_MODE_STEP_3:
				if (old_mode == new_mode) {
					pr_info("%s: Same as older mode\n", __func__);
					break;
				}
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
				switch(new_mode)
				{
					case READER_MODE_STEP_1:
						if (reader_mode_step1_cmds.cmd_cnt) {
							pr_info("%s: sending reader mode STEP1 commands: cnt = %d\n",
								__func__, reader_mode_step1_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step1_cmds, CMD_REQ_COMMIT);
						}
						break;
					case READER_MODE_STEP_2:
						if (reader_mode_step2_cmds.cmd_cnt) {
							pr_info("%s: sending reader mode STEP2 commands: cnt = %d\n",
								__func__, reader_mode_step2_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step2_cmds, CMD_REQ_COMMIT);
						}
						break;
					case READER_MODE_STEP_3:
						if (reader_mode_step3_cmds.cmd_cnt) {
							pr_info("%s: sending reader mode STEP3 commands: cnt = %d\n",
								__func__, reader_mode_step3_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step3_cmds, CMD_REQ_COMMIT);
						}
						break;
					case READER_MODE_MONO:
						/* 5500K */
						if (reader_mode_initial_step2_cmds.cmd_cnt) {
							pr_info("%s: sending reader mode STEP2 commands: cnt = %d\n",
								__func__, reader_mode_step2_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step2_cmds, CMD_REQ_COMMIT);
						}
						ie_status |= BIT(IE_MONO_BIT);  // monochrome bit set
						lge_mdss_dsi_send_ie_cmds(ctrl);
						break;
					default:
						pr_err("%s: Input Invalid parameter: %d \n", __func__, new_mode);
						break;
				}
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
				break;
			case READER_MODE_MONO:
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
				if (new_mode != READER_MODE_MONO) {
					ie_status &= ~BIT(IE_MONO_BIT);  // monochrome bit clear
					lge_mdss_dsi_send_ie_cmds(ctrl);
				}
				switch(new_mode)
				{
					case READER_MODE_STEP_1:
						if (reader_mode_step1_cmds.cmd_cnt) {
							pr_err("%s: sending reader mode STEP1 commands: cnt : %d \n",
								__func__, reader_mode_step1_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step1_cmds, CMD_REQ_COMMIT);
						}
						break;
					case READER_MODE_STEP_2:
						if (reader_mode_step2_cmds.cmd_cnt) {
							pr_err("%s: sending reader mode STEP1 commands: cnt : %d \n",
								__func__, reader_mode_step2_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step2_cmds, CMD_REQ_COMMIT);
						}
						break;
					case READER_MODE_STEP_3:
						if (reader_mode_step3_cmds.cmd_cnt) {
							pr_info("%s: sending reader mode STEP3 commands: cnt = %d \n",
								__func__, reader_mode_step3_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step3_cmds, CMD_REQ_COMMIT);
						}
						break;
					case READER_MODE_MONO:
						break;
					default:
						pr_err("%s: Input Invalid parameter : %d \n", __func__, new_mode);
						break;
				}
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
				break;
			default:
				pr_err("%s: Invalid old status : %d\n", __func__, old_mode);
				break;
		}
	}

	return true;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	if(gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_info("%s: reset to low\n", __func__);
		gpio_set_value((ctrl_pdata->rst_gpio), 0); //Reset low
	}
	usleep_range(10000, 10000);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
	ext_dsv_mode_change(POWER_OFF);      //Disable DSV_EN
	usleep_range(2000, 2000);
#endif
	lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 0);  //DSV low
	usleep_range(10000, 10000);
	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);    //vddio low
	usleep_range(5000, 5000);
	lge_extra_gpio_set_value(ctrl_pdata, "avdd", 0);    //avdd low
	pr_info("%s: turn panel shutdown\n", __func__);

	return;
}
#endif

#if defined(CONFIG_LGE_DISPLAY_DAYLIGHT_MODE)
int lge_mdss_dsi_set_daylight_mode(struct mdss_dsi_ctrl_pdata *ctrl, int mode)
{
	pr_info("%s: daylight mode : %d\n", __func__, mode);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
	switch(mode)
	{
		case DAYLIGHT_MODE_OFF :
			ie_status &= ~BIT(IE_DAYLIGHT_BIT);  // daylight bit clear
			lge_mdss_dsi_send_ie_cmds(ctrl);
			break;
		case DAYLIGHT_MODE_HIGH :
			ie_status |= BIT(IE_DAYLIGHT_BIT);  // daylight bit set
			lge_mdss_dsi_send_ie_cmds(ctrl);
			break;
		default :
			break;
	}
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	return 0;
}
#endif
