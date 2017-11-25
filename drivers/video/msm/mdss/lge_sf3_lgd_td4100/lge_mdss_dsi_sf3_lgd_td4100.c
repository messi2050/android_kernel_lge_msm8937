#include <linux/delay.h>
#include "mdss_dsi.h"
#include <linux/mfd/sm5107.h>
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
#include "mdss_mdp.h"
#include "lge/reader_mode.h"
#endif
#include "lge/mfts_mode.h"
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
#include "lge/lge_mdss_debug.h"
#endif
#include <soc/qcom/lge/board_lge.h>

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
/* touch irq handle according to display power down in mfts (sf3) */
extern bool mfts_check_shutdown;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON) || IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);

#define IOVCC_POST 0
#define RESET_POST 1
#define DSV_POST 2

#define USLEEP(a) usleep_range(a,a)

static int delay_table[][3] = {
//{iovcc_post,reset_post,dsv_post}
    {   10000, 200000, 10000  },  // for id=0, lgd panel
    {   3000,  20000,  130000 }   // for id=1, tovis panel
};

int get_delay_time(enum lge_panel_type panel_type, int select_time)
{
    switch(panel_type){
        case SF3_LGD_TD4100:
            return delay_table[0][select_time];
        case SF3_TOVIS:
            return delay_table[1][select_time];
        default:
            return 1;
    }
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

	if (lge_mdss_dsi_panel_power_seq_all()) {
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

	if (lge_mdss_dsi_panel_power_seq_all()) {
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

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}

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
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	return rc;
}

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
		rc = mdss_dsi_request_gpios(ctrl_pdata);
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

			if (lge_mdss_dsi_panel_power_seq_all()) {
				pr_info("%s: turn panel power on\n", __func__);

				gpio_set_value((ctrl_pdata->rst_gpio), 0);
				mdelay(10);

				lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 1);
				USLEEP(get_delay_time(lge_get_panel_type(),IOVCC_POST));

				gpio_set_value((ctrl_pdata->rst_gpio), 1);
				USLEEP(get_delay_time(lge_get_panel_type(),RESET_POST));

				lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 1);
				USLEEP(get_delay_time(lge_get_panel_type(),DSV_POST));
			} else{
				pr_info("%s: skip panel power control\n", __func__);

				for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
					gpio_set_value((ctrl_pdata->rst_gpio), pdata->panel_info.rst_seq[i]);

					if (pdata->panel_info.rst_seq[++i])
						usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
				}
			}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
			ext_dsv_mode_change(MODE_NORMAL); // DSV_ENREG enable
#endif
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}

		lge_set_panel_recovery_flag(0);
	} else {
		if (lge_mdss_dsi_panel_power_seq_all()) {
			mfts_check_shutdown = true; /* Variable that can only used sf3 */
			pr_info("%s: turn panel power off\n", __func__);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
			ext_dsv_mode_change(MODE_KNOCKON);  //DSV_ENREG disable
#endif
			pr_info("%s: set panel reset low\n", __func__);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			mdelay(5);

			lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 0); //DSV low
			usleep_range(10000, 10000);

			lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);   //iovcc low
		}

		lge_extra_gpio_free(ctrl_pdata, "dsv_ena");
		lge_extra_gpio_free(ctrl_pdata, "iovcc");
		gpio_free(ctrl_pdata->rst_gpio);

	}

	pr_info("%s: -\n", __func__);
exit:
	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
static struct dsi_panel_cmds reader_mode_cmds[4];

int lge_mdss_dsi_parse_reader_mode_cmds(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_OFF], "qcom,panel-reader-mode-off-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_STEP_1], "qcom,panel-reader-mode-step1-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_STEP_2], "qcom,panel-reader-mode-step2-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_cmds[READER_MODE_STEP_3], "qcom,panel-reader-mode-step3-command", "qcom,mdss-dsi-reader-mode-command-state");

	return 0;
}

static bool change_reader_mode(struct mdss_dsi_ctrl_pdata *ctrl, int new_mode)
{
	if (new_mode == READER_MODE_MONO) {
		pr_info("%s: sending reader mode (MONO) commands [%d]\n", __func__, READER_MODE_STEP_2);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
		mdss_dsi_panel_cmds_send(ctrl, &reader_mode_cmds[READER_MODE_STEP_2], CMD_REQ_COMMIT);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	} else {
		if(reader_mode_cmds[new_mode].cmd_cnt) {
			pr_info("%s: sending reader mode commands [%d]\n", __func__, new_mode);
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
			mdss_dsi_panel_cmds_send(ctrl, &reader_mode_cmds[new_mode], CMD_REQ_COMMIT);
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
		}
	}
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
		gpio_set_value((ctrl_pdata->rst_gpio), 0);		//Reset low
	}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_SM5107_DSV)
	ext_dsv_mode_change(POWER_OFF);  					//Disable DSV_EN
	usleep_range(3000, 3000);
#endif

	lge_extra_gpio_set_value(ctrl_pdata, "dsv_ena", 0);	//DSV low
	usleep_range(10000, 10000);

	lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);	//iovcc low

	pr_info("%s: turn panel shutdown\n", __func__);

	return;

}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	switch (event) {
		case MDSS_EVENT_RESET:
			lge_debug_event_trigger(pdata, "/etc/debug_dsi_timing_change", DEBUG_DSI_TIMING_CHANGE);
			break;
		case MDSS_EVENT_POST_PANEL_ON:
			lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_tx", DEBUG_DSI_CMD_TX);
			lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_rx", DEBUG_DSI_CMD_RX);
			break;
		case MDSS_EVENT_PANEL_OFF:
			lge_debug_event_trigger(pdata, "", INVALID); //NOTE : This is must-do-null-event-trigger for debug_event to escape from unblnak
			break;
		default:
			break;
	}
	return rc;

}
#endif

static int panel_recovery_flag = 0;
int lge_get_panel_recovery_flag()
{
	pr_info("%s: flag=%d", __func__, panel_recovery_flag);
	return panel_recovery_flag;
}

/*void lge_set_panel_recovery_flag(int flag)
{
	pr_info("%s: flag=%d", __func__, flag);
	panel_recovery_flag = flag;
}*/
