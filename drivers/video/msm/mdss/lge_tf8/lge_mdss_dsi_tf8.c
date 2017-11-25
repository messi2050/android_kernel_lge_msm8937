#define pr_fmt(fmt)	"[DISPLAY]%s: " fmt, __func__

#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include <soc/qcom/lge/board_lge.h>
#include "lge/reader_mode.h"


static int gpio_power_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{

	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (pinfo == NULL) {
		pr_err("invalid pinfo\n");
		return -EINVAL;
	}

	if (enable) {
		rc = lge_extra_gpio_request(ctrl_pdata, "iovcc");
		if (rc) {
			pr_err("gpio request failed, rc = %d\n", rc);
			return rc;
		}

		if (!pinfo->cont_splash_enabled) {
			pr_info("turn panel power on\n");
			lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 1);
		}
	} else {
		pr_info("turn panel power off\n");
		lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);
		lge_extra_gpio_free(ctrl_pdata, "iovcc");
	}

	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_ON)
int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pr_info("++\n");

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	if (ctrl_pdata == NULL) {
		pr_err("invalid ctrl_pdata\n");
		return -EINVAL;
	}

	ret = gpio_power_ctrl(ctrl_pdata, 1);

	if (ret)
		pr_err("panel power on failed, ret = %d\n", ret);

	usleep_range(100, 100);

	ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);
	if (ret)
		pr_err("failed to enable vregs for %s\n",
				__mdss_dsi_pm_name(DSI_PANEL_PM));

	pr_info("--\n");
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_POWER_OFF)
int mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pr_info("++\n");

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	if (ctrl_pdata == NULL) {
		pr_err("invalid ctrl_pdata\n");
		return -EINVAL;
	}

	ret = gpio_power_ctrl(ctrl_pdata, 0);

	if (ret)
		pr_err("panel power off failed, rc=%d\n", ret);

	usleep_range(5000, 5000);

	ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 0);

	if (ret)
		pr_err("failed to disable vregs for %s\n",
				__mdss_dsi_pm_name(DSI_PANEL_PM));

	pr_info("--\n");
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_PANEL_RESET)
/*
 * mdss_dsi_request_gpios() should be defined in each panel file
 */
int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("no ctrl_pdata\n");
		return;
	}

	pr_info("turn panel shutdown\n");
	lge_extra_gpio_set_value(ctrl_pdata, "iovcc", 0);

	return;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
struct dcs_dt_table {
	struct dsi_panel_cmds *dpc;
	char *cmd_key;
	char *link_key;
};

#define IS_SAME_GAMMA(old, new) \
			((old == READER_MODE_STEP_2 && new == READER_MODE_MONO) || \
			(old == READER_MODE_MONO && new == READER_MODE_STEP_2))

#define ck_pre "lge,panel-cmd-"
#define lk_pre "lge,panel-cmd-state-"
#define DCS_PAR_LIST(cmds, idx, ck_sfx, lk_sfx) {&cmds[idx], ck_pre ck_sfx, lk_pre lk_sfx}
#define DCS_PAR(cmds, ck_sfx, lk_sfx) {&cmds, ck_pre ck_sfx, lk_pre lk_sfx}

static struct dsi_panel_cmds fast_gamma_cmds[5];
static struct dsi_panel_cmds mono_on_cmds;
static struct dsi_panel_cmds mono_off_cmds;
static struct dsi_panel_cmds fast_gamma_mono_off_dft_cmds;

#define MAX_READER_MODE (READER_MODE_MONO)
#define READER_MODE_MONO_ON (MAX_READER_MODE + 1)
#define READER_MODE_MONO_OFF (MAX_READER_MODE + 2)
#define READER_MODE_MONO_OFF_DFT_GAMMA (MAX_READER_MODE + 3)
#define READER_MODE_MONO_OFF_NEW_GAMMA (MAX_READER_MODE + 4)


#define ST0 READER_MODE_OFF			/* default gamma */
#define ST1 READER_MODE_STEP_1			/* step1 gamma */
#define ST2 READER_MODE_STEP_2			/* step2 gamma */
#define ST3 READER_MODE_STEP_3			/* step3 gamma */
#define MN2 READER_MODE_MONO			/* mono mode on & step2 gamma */
#define MNE READER_MODE_MONO_ON			/* mono mode on */
#define MND READER_MODE_MONO_OFF		/* mono mode off */
#define MD0 READER_MODE_MONO_OFF_DFT_GAMMA	/* mono mode off & step0 gamma */
#define MDN READER_MODE_MONO_OFF_NEW_GAMMA	/* mono mode off & new  gamma */

static int rm_cmds_id[MAX_READER_MODE+1][MAX_READER_MODE+1] = {
		/* new mode */
      /* ST0  ST1  ST2  ST3  MN2 */
	{ST0, ST1, ST2, ST3, MN2},  /* ST0 */
	{ST0, ST1, ST2, ST3, MN2},  /* ST1 */
	{ST0, ST1, ST2, ST3, MNE},  /* ST2 */ /* old mode */
	{ST0, ST1, ST2, ST3, MN2},  /* ST3 */
	{MD0, MDN, MND, MDN, MN2},  /* MN2 */
};

static char * rm_mode[] = {
	"rm_off",
	"rm_step1",
	"rm_step2",
	"rm_step3",
	"rm_mono"
};
static struct dcs_dt_table ddt[]  = {
	DCS_PAR_LIST(fast_gamma_cmds, READER_MODE_OFF, "fast-gamma-dft", "lp"),
	DCS_PAR_LIST(fast_gamma_cmds, READER_MODE_STEP_1, "fast-gamma-step1", "lp"),
	DCS_PAR_LIST(fast_gamma_cmds, READER_MODE_STEP_2, "fast-gamma-step2", "lp"),
	DCS_PAR_LIST(fast_gamma_cmds, READER_MODE_STEP_3, "fast-gamma-step3", "lp"),
	DCS_PAR_LIST(fast_gamma_cmds, READER_MODE_MONO, "fast-gamma-mono", "lp"),
	DCS_PAR(mono_on_cmds, "mono-on", "lp"),
	DCS_PAR(mono_off_cmds, "mono-off", "lp"),
	DCS_PAR(fast_gamma_mono_off_dft_cmds, "fast-gamma-mtd", "lp"),
};

static void lge_send_dcs_cmds(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *dpc)
{
	if (dpc && dpc->cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, dpc, CMD_REQ_COMMIT);
	else
		pr_info("dcs command not found\n");
}

static void lge_send_dcs_cmds_by_id(struct mdss_dsi_ctrl_pdata *ctrl, int index)
{
	if (index >= 0 && index < ARRAY_SIZE(ddt) && ddt[index].cmd_key) {
		pr_info("sending %s\n", ddt[index].cmd_key);
		lge_send_dcs_cmds(ctrl, ddt[index].dpc);
	} else {
		pr_info("index %d dcs command not found\n", index);
	}
}

int lge_mdss_dsi_parse_reader_mode_cmds(struct device_node *np,
					struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ddt); i++) {
		mdss_dsi_parse_dcs_cmds(np, ddt[i].dpc,
				ddt[i].cmd_key, ddt[i].link_key);
	}
	return 0;
}


int lge_mdss_dsi_panel_send_on_cmds(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *default_on_cmds, int cur_mode)
{
	if(cur_mode > MAX_READER_MODE || cur_mode < 0) {
		pr_err("rm mode: invalid mode(%d)\n", cur_mode);
		return -1;
	}

	pr_info("panel off -> %s\n",rm_mode[cur_mode]);

	pr_info("sending default on cmds\n");
	lge_send_dcs_cmds(ctrl, default_on_cmds);

	lge_send_dcs_cmds_by_id(ctrl, rm_cmds_id[cur_mode][cur_mode]);
	return 0;
}

bool lge_change_reader_mode(struct mdss_dsi_ctrl_pdata *ctrl, int old_mode, int new_mode)
{

	int cmd_id;
	if(new_mode > MAX_READER_MODE || new_mode < 0) {
		pr_err("invalid new_mode: %d\n", new_mode);
		return false;
	}

	if(old_mode > MAX_READER_MODE || old_mode < 0) {
		pr_err("invalid old_mode: %d\n", old_mode);
		return false;
	}

	pr_info("%s -> %s\n", rm_mode[old_mode], rm_mode[new_mode]);

	if (new_mode == old_mode) {
		pr_info("no mode change\n");
		return true;
	}

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

	cmd_id = rm_cmds_id[old_mode][new_mode];

	if (cmd_id == MDN) {
		cmd_id = new_mode;
		lge_send_dcs_cmds_by_id(ctrl, cmd_id);
		lge_send_dcs_cmds_by_id(ctrl, MND);
	} else {
		lge_send_dcs_cmds_by_id(ctrl, cmd_id);
	}

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);


	return true;
}
#endif
