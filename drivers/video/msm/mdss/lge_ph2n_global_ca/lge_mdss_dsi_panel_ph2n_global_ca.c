#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "lge/reader_mode.h"
#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
#include <soc/qcom/lge/board_lge.h>
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
extern int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER_3) || IS_ENABLED(CONFIG_LGE_TOUCH_CORE)
extern int mfts_lpwg_on;
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
static struct dsi_panel_cmds reader_mode_initial_step1_cmds;
static struct dsi_panel_cmds reader_mode_initial_step2_cmds;
static struct dsi_panel_cmds reader_mode_initial_step3_cmds;
static struct dsi_panel_cmds reader_mode_initial_mono_enable_cmds;
static struct dsi_panel_cmds reader_mode_step1_cmds;
static struct dsi_panel_cmds reader_mode_step2_cmds;
static struct dsi_panel_cmds reader_mode_step3_cmds;
static struct dsi_panel_cmds reader_mode_off_cmds;
static struct dsi_panel_cmds reader_mode_mono_enable_cmds;
static struct dsi_panel_cmds reader_mode_mono_disable_cmds;
#endif

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
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

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n", __func__, __LINE__);
	}

	pr_info("%s: + enable = %d (override: ph2n_global_ca)\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
		if (rc) {
			pr_err("request reset gpio failed, rc=%d\n", rc);
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
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
		if (lge_get_panel_type() == PH2_SHARP) {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			LGE_MDELAY(10);
		} else if(lge_get_panel_type() == PH2_JDI) {
#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER_3) || IS_ENABLED(CONFIG_LGE_TOUCH_CORE)
			if(lge_get_mfts_mode()) {
				if (!mfts_lpwg_on) {
					gpio_set_value((ctrl_pdata->rst_gpio), 0);
				} else {
					gpio_set_value((ctrl_pdata->rst_gpio), 1);
				}
			} else {
				gpio_set_value((ctrl_pdata->rst_gpio), 1);
			}
#else
		gpio_set_value((ctrl_pdata->rst_gpio), 1);
#endif
		}
		gpio_free(ctrl_pdata->rst_gpio);
	}

	pr_info("%s: -\n", __func__);
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

	pr_info("%s: (override: ph2n_global_ca)\n", __func__);

	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 0);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER_3) || IS_ENABLED(CONFIG_LGE_TOUCH_CORE)
	if(lge_get_mfts_mode()) {
			if (!mfts_lpwg_on) {
				if(lge_get_panel_type() == PH2_SHARP) {
					lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
					LGE_MDELAY(2);

					lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 0);
					LGE_MDELAY(2);
					lge_extra_gpio_set_value(ctrl_pdata, "touch-vdddc", 0);
					lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
				} else if(lge_get_panel_type() == PH2_JDI) {
					LGE_MDELAY(1);
					lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
					lge_extra_gpio_set_value(ctrl_pdata, "touch-vdddc", 0);
					lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
					LGE_MDELAY(1);
					lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 0);
				}
			}
	}
#endif

	if (ret)
		pr_err("%s: failed to disable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));

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

	pr_info("%s: (override: ph2n_global_ca)\n", __func__);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER_3) || IS_ENABLED(CONFIG_LGE_TOUCH_CORE)
		if(lge_get_mfts_mode()) {
				if (!mfts_lpwg_on) {
					if(lge_get_panel_type() == PH2_SHARP) {
						lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
						lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
						lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 1);
						LGE_MDELAY(2);
						lge_extra_gpio_set_value(ctrl_pdata, "touch-vdddc", 1);
						LGE_MDELAY(25);
					} else if(lge_get_panel_type() == PH2_JDI) {
						lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
						lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 1);
						LGE_MDELAY(1);
						lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
						lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 1);
						lge_extra_gpio_set_value(ctrl_pdata, "touch-vdddc", 1);
						LGE_MDELAY(1);
					}
				}
		}
#endif

	ret = msm_dss_enable_vreg(
		ctrl_pdata->panel_power_data.vreg_config,
		ctrl_pdata->panel_power_data.num_vreg, 1);
	if (ret) {
		pr_err("%s: failed to enable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
		return ret;
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
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step1_cmds,
		"qcom,panel-reader-mode-initial-step1-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step2_cmds,
		"qcom,panel-reader-mode-initial-step2-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step3_cmds,
		"qcom,panel-reader-mode-initial-step3-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_mono_enable_cmds,
		"qcom,panel-reader-mode-mono-enable-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step1_cmds,
		"qcom,panel-reader-mode-step1-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step2_cmds,
		"qcom,panel-reader-mode-step2-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step3_cmds,
		"qcom,panel-reader-mode-step3-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_off_cmds,
		"qcom,panel-reader-mode-off-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_mono_enable_cmds,
		"qcom,panel-reader-mode-mono-enable-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_mono_disable_cmds,
		"qcom,panel-reader-mode-mono-disable-command", "qcom,mdss-dsi-reader-mode-command-state");

	return 0;
}

int lge_mdss_dsi_panel_send_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *default_on_cmds, int cur_mode)
{
	if (default_on_cmds->cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, default_on_cmds, CMD_REQ_COMMIT);

	pr_info("%s: reader_mode (%d).\n", __func__, cur_mode);

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
			if(lge_get_panel_type() == PH2_JDI) {
				/* 5500K */
				if (reader_mode_initial_step2_cmds.cmd_cnt) {
					pr_info("%s: reader_mode_initial_step2_cmds: cnt = %d \n",
						__func__, reader_mode_initial_step2_cmds.cmd_cnt);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_step2_cmds, CMD_REQ_COMMIT);
				}
				if (reader_mode_initial_mono_enable_cmds.cmd_cnt) {
					pr_info("%s: reader_mode_mono_enable_cmds: cnt = %d \n",
						__func__, reader_mode_initial_mono_enable_cmds.cmd_cnt);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_mono_enable_cmds, CMD_REQ_COMMIT);
				}
			} else if(lge_get_panel_type() == PH2_SHARP) {
				/* 5500K */
				if (reader_mode_initial_step2_cmds.cmd_cnt) {
					pr_info("%s: reader_mode_initial_step2_cmds: cnt = %d \n",
						__func__, reader_mode_initial_step2_cmds.cmd_cnt);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_step2_cmds, CMD_REQ_COMMIT);
				}
			}
			break;
		case READER_MODE_OFF:
		default:
			break;
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
				if(reader_mode_off_cmds.cmd_cnt) {
					pr_info("%s: sending reader mode OFF commands: cnt = %d\n",
						__func__, reader_mode_off_cmds.cmd_cnt);
					mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_off_cmds, CMD_REQ_COMMIT);
					mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
				}
			    break;
			case READER_MODE_MONO:
				if(reader_mode_mono_disable_cmds.cmd_cnt || reader_mode_off_cmds.cmd_cnt) {
					mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
				}
				if(reader_mode_off_cmds.cmd_cnt) {
					pr_info("%s: sending reader mode OFF commands: cnt = %d\n",
						__func__, reader_mode_off_cmds.cmd_cnt);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_off_cmds, CMD_REQ_COMMIT);
				}
				if(reader_mode_mono_disable_cmds.cmd_cnt) {
					pr_info("%s: sending MONO OFF commands: cnt = %d\n",
						__func__, reader_mode_mono_disable_cmds.cmd_cnt);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_mono_disable_cmds, CMD_REQ_COMMIT);
				}
				if(reader_mode_mono_disable_cmds.cmd_cnt || reader_mode_off_cmds.cmd_cnt) {
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
				if(old_mode == new_mode)
				{
					pr_info("%s: Same as older mode\n", __func__);
					break;
				}
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
				switch(new_mode)
				{
					case READER_MODE_STEP_1:
						if(reader_mode_step1_cmds.cmd_cnt) {
							pr_info("%s: sending reader mode STEP1 commands: cnt = %d\n",
								__func__, reader_mode_step1_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step1_cmds, CMD_REQ_COMMIT);
						}
						break;
					case READER_MODE_STEP_2:
						if(reader_mode_step2_cmds.cmd_cnt) {
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
						if(lge_get_panel_type() == PH2_JDI) {
							/* 5500K */
							if (reader_mode_initial_step2_cmds.cmd_cnt) {
								pr_info("%s: sending reader mode STEP2 commands: cnt = %d\n",
									__func__, reader_mode_step2_cmds.cmd_cnt);
								mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step2_cmds, CMD_REQ_COMMIT);
							}
							if (reader_mode_initial_mono_enable_cmds.cmd_cnt) {
								pr_info("%s: reader_mode_mono_enable_cmds: cnt = %d \n",
									__func__, reader_mode_initial_mono_enable_cmds.cmd_cnt);
								mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_mono_enable_cmds, CMD_REQ_COMMIT);
							}
						} else if(lge_get_panel_type() == PH2_SHARP) {
							/* 5500K */
							if (reader_mode_initial_step2_cmds.cmd_cnt) {
								pr_info("%s: sending reader mode STEP2 commands: cnt = %d\n",
									__func__, reader_mode_step2_cmds.cmd_cnt);
								mdss_dsi_panel_cmds_send(ctrl, &reader_mode_step2_cmds, CMD_REQ_COMMIT);
							}
						}
						break;
					default:
						pr_err("%s: Input Invalid parameter: %d \n", __func__, new_mode);
						break;
				}
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
				break;
			case READER_MODE_MONO:
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
				if(new_mode != READER_MODE_MONO) {
					if (reader_mode_mono_disable_cmds.cmd_cnt) {
						pr_err("%s: sending reader MONO mode OFF commands: FIRST cnt : %d \n",
							__func__, reader_mode_mono_disable_cmds.cmd_cnt);
						mdss_dsi_panel_cmds_send(ctrl, &reader_mode_mono_disable_cmds, CMD_REQ_COMMIT);
					}
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
