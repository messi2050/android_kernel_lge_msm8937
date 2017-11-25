#include <linux/delay.h>
#include "mdss_dsi.h"
#include <soc/qcom/lge/board_lge.h>
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
#include "mdss_mdp.h"
#include "lge/reader_mode.h"
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
#include "lge/lge_mdss_debug.h"
#endif

#define USE_TEMP_HX8394C_PANEL_FOR_HDK 0 //only for test


#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON) || IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DW8768_DSV)
#include <linux/mfd/dw8768.h>


int lge_mdss_dsi_panel_power_dsv_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int rc = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return -EINVAL;
	}

#if USE_TEMP_HX8394C_PANEL_FOR_HDK
	return rc;
#endif

	if(enable) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		if (ctrl_pdata->debug_pwr_always_on[0] > 0) {
			pr_info("%s: not controlled[%d] \n", __func__, enable);
			return rc;
		}
#endif
		usleep_range(5000, 5000);
		lge_extra_gpio_set_value(ctrl_pdata, "dw8768-ena", 1);
		usleep_range(5000, 5000);
		lge_extra_gpio_set_value(ctrl_pdata, "dw8768-enm", 1);
	} else {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		if (ctrl_pdata->debug_pwr_always_on[0] > 0) {
			pr_info("%s: not controlled[%d] \n", __func__, enable);
			return rc;
		}
#endif
		lge_extra_gpio_set_value(ctrl_pdata, "dw8768-enm", 0);
		usleep_range(5000, 5000);
		lge_extra_gpio_set_value(ctrl_pdata, "dw8768-ena", 0);
		usleep_range(5000, 5000);
	}
	pr_err("%s: enabled[%d], result[%d] \n", __func__, enable, rc);

	return rc;
}
#endif

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

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_err("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	if (enable) {
		if (!pinfo->cont_splash_enabled) {
			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
#if USE_TEMP_HX8394C_PANEL_FOR_HDK
				lge_extra_gpio_set_value(ctrl_pdata, "reset3.3", pdata->panel_info.rst_seq[i]);
#else
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
#endif
				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}
			pr_info("%s: LCD reset sequence done \n", __func__);
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		if (ctrl_pdata->debug_pwr_seq_dly[2] > 0) {
			usleep_range(ctrl_pdata->debug_pwr_seq_dly[2] * 1000, ctrl_pdata->debug_pwr_seq_dly[2] * 1000);
			pr_info("%s: %dms delayed after RESET  \n", __func__, ctrl_pdata->debug_pwr_seq_dly[2]);
		}
#endif
	} else {
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}

	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	enum hw_rev_type rev;

	if (pdata == NULL) {
		pr_err("%s: Invalid pdata\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid ctrl_pdata\n", __func__);
		return -EINVAL;
	}

	rev = lge_get_board_revno();

	if (rev < HW_REV_A) {
#if USE_TEMP_HX8394C_PANEL_FOR_HDK
		//do nothing
#else
		//do nothing, use always-on L6 ldo
#endif
	} else if (rev >= HW_REV_A) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
	}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
	if (ctrl_pdata->debug_pwr_seq_dly[0] > 0) {
		usleep_range(ctrl_pdata->debug_pwr_seq_dly[0] * 1000, ctrl_pdata->debug_pwr_seq_dly[0] * 1000);
		pr_info("%s: %dms delayed after I/O  \n", __func__, ctrl_pdata->debug_pwr_seq_dly[0]);
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

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DW8768_DSV)
	lge_mdss_dsi_panel_power_dsv_ctrl(ctrl_pdata, 1);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
	if (ctrl_pdata->debug_pwr_seq_dly[1] > 0) {
		usleep_range(ctrl_pdata->debug_pwr_seq_dly[1] * 1000, ctrl_pdata->debug_pwr_seq_dly[1] * 1000);
		pr_info("%s: %dms delayed after DSV  \n", __func__, ctrl_pdata->debug_pwr_seq_dly[1]);
	}
#endif

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
	enum hw_rev_type rev;

	if (pdata == NULL) {
		pr_err("%s: Invalid pdata\n", __func__);
		ret = -EINVAL;
		goto end;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid ctrl_pdata\n", __func__);
		return -EINVAL;
	}

	rev = lge_get_board_revno();

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
	if (ret)
		pr_err("%s: failed to disable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DW8768_DSV)
	lge_mdss_dsi_panel_power_dsv_ctrl(ctrl_pdata, 0);
#endif


	if (rev < HW_REV_A) {
#if USE_TEMP_HX8394C_PANEL_FOR_HDK
		//do nothing
#else
		//do nothing, use always-on L6 ldo
#endif
	} else if (rev >= HW_REV_A) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	}

end:
	return ret;
}
#endif

int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;

	switch (event) {
	case MDSS_EVENT_RESET:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		lge_debug_event_trigger(pdata, "/etc/debug_pwr_seq_delay", DEBUG_PWR_SEQ_DELAY);
#endif
		break;
	case MDSS_EVENT_POST_PANEL_ON:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		lge_debug_event_trigger(pdata, "/etc/debug_pwr_always_on", DEBUG_PWR_ALWAYS_ON);
		lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_tx", DEBUG_DSI_CMD_TX);
		lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_rx", DEBUG_DSI_CMD_RX);
#endif
		break;
	case MDSS_EVENT_PANEL_OFF:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		lge_debug_event_trigger(pdata, "", INVALID); //NOTE : This is must-do-null-event-trigger for debug_event to escape from unblnak
#endif
		break;
	default:
		break;
	}

	return rc;
}


#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);

static struct dsi_panel_cmds reader_mode_cmds[4];

int lge_mdss_dsi_parse_reader_mode_cmds(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_OFF], "qcom,panel-reader-mode-off-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_STEP_1], "qcom,panel-reader-mode-step1-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_STEP_2], "qcom,panel-reader-mode-step2-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_STEP_3], "qcom,panel-reader-mode-step3-command", "qcom,mdss-dsi-reader-mode-command-state");

    {
        int i;
        for (i=0; i<4; ++i) {
            pr_info("%s: cmd size[%d] = %d\n", __func__, i, reader_mode_cmds[i].cmd_cnt);
        }
    }

	return 0;
}

static bool change_reader_mode(struct mdss_dsi_ctrl_pdata *ctrl, int new_mode)
{
	if (new_mode == READER_MODE_MONO) {
		pr_info("%s: READER_MODE_MONO is not supported. reader mode is going off.\n", __func__);
		new_mode = READER_MODE_STEP_2;
	}

    pr_info("%s ++\n", __func__);
	if(reader_mode_cmds[new_mode].cmd_cnt) {
		pr_info("%s: sending reader mode commands [%d]\n", __func__, new_mode);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
		mdss_dsi_panel_cmds_send(ctrl, &reader_mode_cmds[new_mode], CMD_REQ_COMMIT);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	}
    pr_info("%s --\n", __func__);
	return true;
}

bool lge_change_reader_mode(struct mdss_dsi_ctrl_pdata *ctrl, int old_mode, int new_mode)
{
	if (old_mode == new_mode) {
		pr_info("%s: same mode [%d]\n", __func__, new_mode);
		return true;
	}

	return change_reader_mode(ctrl, new_mode);
}

int lge_mdss_dsi_panel_send_post_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, int cur_mode)
{
	if (cur_mode != READER_MODE_OFF)
		change_reader_mode(ctrl, cur_mode);
	return 0;
}
#endif
