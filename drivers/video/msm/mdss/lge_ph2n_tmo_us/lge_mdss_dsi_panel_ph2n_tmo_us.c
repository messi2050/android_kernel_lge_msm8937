#include <linux/delay.h>
#include <soc/qcom/lge/board_lge.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "lge_mdss_dsi_panel_ph2n_tmo_us.h"
#include "lge/reader_mode.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
extern void MIT300_Reset(int status, int delay);
extern int mfts_lpwg;
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

extern int lge_mdss_fb_get_shutdown_state(void);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

int lge_get_dsv_type(void)
{
	enum hw_rev_type rev = lge_get_board_revno();

	pr_info("%s : board revision = %d\n", __func__, lge_get_board_revno());

	if(rev <= HW_REV_A) {
		return LGE_DSV_PMI8952;
	} else if ( rev >= HW_REV_B && rev < HW_REV_MAX) {
		return LGE_DSV_DW8768;
	} else
		return 0;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int dsv_type;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n", __func__, __LINE__);
	}

	pr_info("%s: + enable = %d (override: ph2n_tmo_us)\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		if (!pinfo->cont_splash_enabled) {
			pr_info("[LCD] Start LCD reset pin down when LCD on\n");
			if (gpio_is_valid(ctrl_pdata->rst_gpio))
				gpio_set_value((ctrl_pdata->rst_gpio), 0);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
			MIT300_Reset(0, 2);
#endif
			if(lge_get_mfts_mode()){
#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
				if (!mfts_lpwg) {
#endif
					pr_info("%s+: lcd power on, lge_get_mfts_mode = %d\n", __func__, lge_get_mfts_mode());
					pr_info("[LCD] turn on VPNL, VDDI in MFTS Mode\n");
					lge_extra_gpio_set_value(ctrl_pdata, "vddio", 1);
					lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 1);
					usleep_range(2000, 2000);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
				}
#endif
			}

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
			}
#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
			MIT300_Reset(1, 50);
#endif

			if(lge_get_mfts_mode()){
#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
				if (!mfts_lpwg) {
#endif
					pr_info("[LCD] set lab/ibb to HIGH when LCD on\n");
					dsv_type = lge_get_dsv_type();

					if(dsv_type == LGE_DSV_PMI8952) {
						rc = msm_dss_enable_vreg(
							ctrl_pdata->lge_extra.extra_power_data.vreg_config,
							ctrl_pdata->lge_extra.extra_power_data.num_vreg, 1);
						if (rc) {
							pr_err("%s: failed to init regulator, rc=%d\n", __func__, rc);
						}
					} else if(dsv_type == LGE_DSV_DW8768) {
						lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 1);
						LGE_MDELAY(5);
						lge_extra_gpio_set_value(ctrl_pdata, "dsv-enm", 1);
					} else {
						pr_err("%s: wrong dsv type.\n", __func__);
					}
#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
				}
#endif
			}
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		/* do nothing */
	}

	pr_info("%s: -\n", __func__);
	return rc;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_OFF)
int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	int rc = 0;
	int dsv_type;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_info("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds, CMD_REQ_COMMIT);

	LGE_MDELAY(140);

	if(lge_get_mfts_mode()){
		pr_info("%s+: lcd power off, lge_get_mfts_mode = %d\n", __func__, lge_get_mfts_mode());

#if defined(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
		if (!mfts_lpwg) {
#endif
			if (gpio_is_valid(ctrl->rst_gpio)) {
				pr_info("[LCD] reset to low in MFTS Mode\n");
				gpio_set_value((ctrl->rst_gpio), 0);
			}
#if defined(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
			pr_info("[TOUCH] T_Reset to low in MFTS Mode\n");
			MIT300_Reset(0, 2);
#endif
			mdelay(20);

			pr_info("[LCD] set lab/ibb to LOW when LCD off && MFTS Mode\n");
			dsv_type = lge_get_dsv_type();

			if(dsv_type == LGE_DSV_PMI8952) {
				rc = msm_dss_enable_vreg(
					ctrl->lge_extra.extra_power_data.vreg_config,
					ctrl->lge_extra.extra_power_data.num_vreg, 0);
				if (rc) {
					pr_err("%s: failed to init regulator, rc=%d\n", __func__, rc);
				}
			} else if(dsv_type == LGE_DSV_DW8768) {
				lge_extra_gpio_set_value(ctrl, "dsv-enm", 0);
				LGE_MDELAY(5);
				lge_extra_gpio_set_value(ctrl, "dsv-ena", 0);
			} else {
				pr_err("%s: wrong dsv type.\n", __func__);
			}

			pr_info("[LCD] turn off VPNL, VDDI in MFTS Mode\n");
			lge_extra_gpio_set_value(ctrl, "touch-avdd", 0);
			usleep_range(2000, 2000);
			lge_extra_gpio_set_value(ctrl, "vddio", 0);
#if defined(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
		}
#endif
	 } else {
		if(lge_mdss_fb_get_shutdown_state()) {
			pr_info("%s: shutdown, just skip off2_cmd\n", __func__);
		} else {
			lge_mdss_dsi_panel_extra_cmds_send(ctrl, "off2");
			LGE_MDELAY(20);
		}
	}

end:
	pr_info("%s:-\n", __func__);
	return 0;
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
			if (reader_mode_initial_mono_enable_cmds.cmd_cnt) {
				pr_info("%s: reader_mode_mono_enable_cmds: cnt = %d \n",
					__func__, reader_mode_initial_mono_enable_cmds.cmd_cnt);
				mdss_dsi_panel_cmds_send(ctrl, &reader_mode_initial_mono_enable_cmds, CMD_REQ_COMMIT);
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
						if (reader_mode_mono_enable_cmds.cmd_cnt) {
							pr_info("%s: sending reader mode mono enable commands: cnt = %d \n",
								__func__, reader_mode_mono_enable_cmds.cmd_cnt);
							mdss_dsi_panel_cmds_send(ctrl, &reader_mode_mono_enable_cmds, CMD_REQ_COMMIT);
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
