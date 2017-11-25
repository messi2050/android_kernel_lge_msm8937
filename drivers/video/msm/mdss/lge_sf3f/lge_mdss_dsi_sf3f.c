#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include <soc/qcom/lge/board_lge.h>
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
#include "lge/reader_mode.h"
#endif
#include "lge_mdss_dsi_sf3f.h"


#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;

	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			ret = sf3f_lgd_td4310_mdss_dsi_panel_power_on(pdata);
			break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			ret = sf3f_lgd_sw49105_mdss_dsi_panel_power_on(pdata);
			break;
#endif
		default:
			break;
	}

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;

	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			ret = sf3f_lgd_td4310_mdss_dsi_panel_power_off(pdata);
			break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			ret = sf3f_lgd_sw49105_mdss_dsi_panel_power_off(pdata);
			break;
#endif
		default:
			break;
	}

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
/*
 * mdss_dsi_request_gpios() should be defined in each panel file
 */
int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
        int rc = 0;

	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			rc = sf3f_lgd_td4310_mdss_dsi_panel_reset(pdata, enable);
			break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			rc = sf3f_lgd_sw49105_mdss_dsi_panel_reset(pdata, enable);
			break;
#endif
		default:
			break;
	}

        return rc;
}
#endif


#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	switch (lge_get_panel_type()) {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
		case SF3F_TD4310:
			sf3f_lgd_td4310_mdss_dsi_ctrl_shutdown(pdev);
		break;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
		case SF3F_SW49105:
			sf3f_lgd_sw49105_mdss_dsi_ctrl_shutdown(pdev);
		break;
#endif
		default:
			break;
	}
}
#endif

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

	return 0;
}

static bool change_reader_mode(struct mdss_dsi_ctrl_pdata *ctrl, int new_mode)
{
	if (new_mode == READER_MODE_MONO) {
		pr_info("%s: READER_MODE_MONO is not supported. reader mode is going off.\n", __func__);
		new_mode = READER_MODE_STEP_2;
	}

	if(reader_mode_cmds[new_mode].cmd_cnt) {
		pr_info("%s: sending reader mode commands [%d]\n", __func__, new_mode);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
		mdss_dsi_panel_cmds_send(ctrl, &reader_mode_cmds[new_mode], CMD_REQ_COMMIT);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
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
