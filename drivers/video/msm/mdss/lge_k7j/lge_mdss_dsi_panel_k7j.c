#include <linux/delay.h>
#include "mdss_dsi.h"
#include "../lge/lge_mdss_dsi.h"
#include "../lge/lge_mdss_dsi_panel.h"

#include "../lge/mfts_mode.h"

#include <soc/qcom/lge/board_lge.h>

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
#define EXT_DSV_PRIVILEGED
#include <linux/mfd/external_dsv.h>
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
#include "lge/lge_mdss_debug.h"
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
#include "mdss_mdp.h"
#include "lge/reader_mode.h"
static struct dsi_panel_cmds reader_mode_initial_setting_cmds;
static struct dsi_panel_cmds reader_mode_initial_step1_cmds;
static struct dsi_panel_cmds reader_mode_initial_step2_cmds;
static struct dsi_panel_cmds reader_mode_initial_step3_cmds;
static struct dsi_panel_cmds reader_mode_step1_cmds;
static struct dsi_panel_cmds reader_mode_step2_cmds;
static struct dsi_panel_cmds reader_mode_step3_cmds;
static struct dsi_panel_cmds reader_mode_off_cmds;

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
extern int lge_mdss_report_panel_dead(void);
int esd_detected = 0; // to avoid touch mutex dead lock
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
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

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
	pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_info("%s: enable = %d\n", __func__, enable);

	// Power seq. is done on LPWG and DEEPSLEEP commonly
	if (enable) {
		if(lge_get_display_power_ctrl()) {
			// do nothing, controlled by touch driver
		} else {
			if (!pinfo->cont_splash_enabled) {
				switch (lge_get_panel_type()) {
					case JAPAN_LGD_FT8707_1_0:
					case JAPAN_LGD_FT8707_1_1:
						lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
						gpio_set_value((ctrl_pdata->rst_gpio), 0);
						usleep_range(1000, 1000); // 500us
						lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 1);
						gpio_set_value((ctrl_pdata->rst_gpio), 1);
						pr_info("%s: Touch/LCD reset sequence done \n", __func__);
						usleep_range(20000, 20000); // 15ms
						break;
					case JAPAN_LGD_TD4300:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
						if (esd_detected) {
							gpio_set_value((ctrl_pdata->rst_gpio), 0);
							LGE_MDELAY(10);
							gpio_set_value((ctrl_pdata->rst_gpio), 1);
							pr_info("%s: LCD reset sequence done \n", __func__);
							LGE_MDELAY(150);
						}
						esd_detected = 0;
#endif
						break;
					default:
						break;
				}
			}
		}

		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_info("%s: Reset panel done\n", __func__);
		}
	} else {
		if(lge_get_display_power_ctrl()) {
			// do nothing, controlled by touch driver
		} else {
			if (lge_get_panel_type() == JAPAN_LGD_FT8707_1_0) { //will be removed after check touch f/w 1.0
				gpio_set_value((ctrl_pdata->rst_gpio), 0);
				usleep_range(5000, 5000); // 5ms
				gpio_set_value((ctrl_pdata->rst_gpio), 1);
				pr_info("%s: LCD reset sequence done (w/a for d-ic cut 1.0) \n", __func__);
				usleep_range(105000, 105000); // 105ms
			}
		}
	}
	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_MIPI_DSI_LGD_K7J_FHD_VIDEO_INCELL_LCD_PANEL)
static int ttw_mode_enabled = 0; //ttw mode need to be set just once if the mode is needed.

//For PMI8952 with lab-always-on mode, needs to saperate ttw mode control and labibb regulator control
//because needs to saperate normal boot mode and mfts boot mode.
int lge_mdss_dsi_panel_ttw_mode_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int rc = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return -EINVAL;
	}

	rc = lge_msm_dss_ctrl_ttw_mode(
		ctrl_pdata->lge_extra.extra_power_data.vreg_config,
		ctrl_pdata->lge_extra.extra_power_data.num_vreg, enable);
	if (rc)
		pr_err("%s: failed to ctrl ttw \n",
			__func__);

	pr_debug("%s: enabled = %d\n", __func__, enable);
	return rc;
}

//For K7J LCD, lcd reset is triggered by touch driver for the power sequence.
int lge_mdss_dsi_panel_reset_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int rc = 0;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return -EINVAL;
	}

	if (enable) {
		gpio_set_value((ctrl_pdata->rst_gpio), 1);
	} else {
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}
	return rc;
}
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTERNAL_DSV)
int lge_mdss_dsi_panel_power_dsv_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int rc = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return -EINVAL;
	}

	if(lge_get_boot_mode() != LGE_BOOT_MODE_NORMAL) {
		if(lge_get_display_power_ctrl()) {
			if(enable) {
				lge_extra_gpio_set_value(ctrl_pdata, "dw8768-ena", 1);
				usleep_range(50000, 50000); // 50ms
			} else {
				lge_extra_gpio_set_value(ctrl_pdata, "dw8768-ena", 0);
			}
			pr_info("%s: MFTS pwr ctrl - enabled[%d], result[%d] \n", __func__, enable, rc);

			return rc;
		}
	}

	if(enable) {
		ext_dsv_mode_change(DSV_MODE_NORMAL);
	} else {
		ext_dsv_mode_change(DSV_MODE_LPWG);
	}

	return rc;
}
#endif
int lge_mdss_dsi_panel_power_labibb_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int rc = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return -EINVAL;
	}

	/* Fix me !!!!!!!!!!!!!!!!!!!!!!!!!!!!
		w/a for LCD black-out and only for internal dsv
		for a while, do not control labibb because some devices show LCD black-out with abnormal lab voltage level during off control
	*/
	return rc;

	if(lge_get_boot_mode() != LGE_BOOT_MODE_NORMAL) {
		if(!lge_get_display_power_ctrl()) {
			//do not control for LPWG test in MFTS mode
			return rc;
		}
	} else {
		if(!ttw_mode_enabled) {
			lge_mdss_dsi_panel_ttw_mode_ctrl(ctrl_pdata, 1);
			ttw_mode_enabled = 1;
		}
	}

	rc = msm_dss_enable_vreg(
		ctrl_pdata->lge_extra.extra_power_data.vreg_config,
		ctrl_pdata->lge_extra.extra_power_data.num_vreg, enable);

	pr_info("%s: enabled[%d], result[%d] \n", __func__, enable, rc);
	return rc;
}

int lge_mdss_dsi_panel_power_vddio_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int rc = 0;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (enable) {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
	} else {
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
	}

	pr_info("%s: enabled = %d\n", __func__, enable);
	return rc;
}

int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;

	switch (event) {
	case MDSS_EVENT_RESET:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)

#endif
		break;
	case MDSS_EVENT_POST_PANEL_ON:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
		//lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_tx", DEBUG_DSI_CMD_TX);
		//lge_debug_event_trigger(pdata, "/etc/debug_dsi_cmd_rx", DEBUG_DSI_CMD_RX);
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

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);

int lge_mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int type)
{
	int rc = 0;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (type) {
		if (ctrl_pdata->on_cmds.cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->on_cmds, CMD_REQ_COMMIT);
			pr_info("%s: on cmds sent, link_state[%d] \n", __func__, ctrl_pdata->on_cmds.link_state);
		}
	} else {
		if (ctrl_pdata->off_cmds.cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->off_cmds, CMD_REQ_COMMIT);
			pr_info("%s: off cmds sent, link_state[%d] \n", __func__, ctrl_pdata->off_cmds.link_state);
		}
	}

	return rc;
}

void lge_incell_lcd_external_api(int type, int enable)
{
	enum hw_rev_type rev = lge_get_board_revno();
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	pdata = lge_mdss_dsi_get_ctrl_pdata();
	if (pdata == NULL)
		return;

	switch(type) {
		case VDDI_CTRL:
			lge_mdss_dsi_panel_power_vddio_ctrl(pdata, enable);
			break;
		case AVDD_AVEE_CTRL:
			if (rev <= HW_REV_B)
				lge_mdss_dsi_panel_power_labibb_ctrl(pdata, enable);
			else if (rev >= HW_REV_C)
				lge_mdss_dsi_panel_power_dsv_ctrl(pdata, enable);
			break;
		case LCD_RESET_CTRL:
			lge_mdss_dsi_panel_reset_ctrl(pdata, enable);
			break;
		case LCD_INIT_CMD_TRANSFER:
			lge_mdss_dsi_panel_cmds_send(pdata, type);
			break;
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
		case LCD_ESD_RESET:
			esd_detected = 1;
			if(lge_mdss_report_panel_dead() != 0)
				esd_detected = 0;
			break;
#endif
		case NONE:
		default:
			break;
	}
}
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
int lge_mdss_dsi_parse_reader_mode_cmds(struct device_node *np, struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{

	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_setting_cmds,
		"qcom,panel-reader-mode-initial-setting-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step1_cmds,
		"qcom,panel-reader-mode-initial-step1-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step2_cmds,
		"qcom,panel-reader-mode-initial-step2-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_initial_step3_cmds,
		"qcom,panel-reader-mode-initial-step3-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step1_cmds,
		"qcom,panel-reader-mode-step1-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step2_cmds,
		"qcom,panel-reader-mode-step2-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_step3_cmds,
		"qcom,panel-reader-mode-step3-command", "qcom,mdss-dsi-reader-mode-command-state");
	mdss_dsi_parse_dcs_cmds(np, &reader_mode_off_cmds,
		"qcom,panel-reader-mode-off-command", "qcom,mdss-dsi-reader-mode-command-state");

	return 0;
}

int lge_mdss_dsi_panel_send_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *default_on_cmds, int cur_mode)
{
	if (default_on_cmds->cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, default_on_cmds, CMD_REQ_COMMIT);

	pr_info("%s: reader_mode (%d).\n", __func__, cur_mode);

	if (reader_mode_initial_setting_cmds.cmd_cnt) {
		mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_setting_cmds, CMD_REQ_COMMIT);
	}

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

		case READER_MODE_OFF:
		case READER_MODE_MONO:
			if (lge_get_panel_type() == JAPAN_LGD_TD4300) {
				pr_info("%s: reader_mode OFF\n", __func__);
				if (reader_mode_off_cmds.cmd_cnt) {
					pr_info("%s: reader_mode_off_cmds: cnt = %d \n",
						__func__, reader_mode_off_cmds.cmd_cnt);
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_off_cmds, CMD_REQ_COMMIT);
				}
			}
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
			case READER_MODE_MONO:
				if(old_mode == new_mode)
				{
					pr_info("%s: Same as older mode\n", __func__);
					break;
				}
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

				if (reader_mode_initial_setting_cmds.cmd_cnt) {
					mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_setting_cmds, CMD_REQ_COMMIT);
				}
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
						if (lge_get_panel_type() == JAPAN_LGD_TD4300) {
							pr_info("%s: reader_mode OFF\n", __func__);
							if (reader_mode_off_cmds.cmd_cnt) {
								pr_info("%s: reader_mode_off_cmds: cnt = %d \n",
									__func__, reader_mode_off_cmds.cmd_cnt);
								mdss_dsi_panel_cmds_send(ctrl, &reader_mode_off_cmds, CMD_REQ_COMMIT);
							}
						}
						break;
					default:
						pr_err("%s: Input Invalid parameter: %d \n", __func__, new_mode);
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
