#include <linux/of_platform.h>
#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_dsi.h"

#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
char *lge_blmap_name[] = {
	"lge,blmap",
	"lge,blmap-hl",
};
#endif

extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);

static int parse_dt_extra_dcs_cmds(struct device_node *np,
                        struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc;
	int i;
	const char *name;
	char buf1[256];
	char buf2[256];

	rc = of_property_count_strings(np, "lge,mdss-dsi-extra-command-names");
	if (rc > 0) {
		ctrl_pdata->lge_extra.num_extra_cmds = rc;
		pr_info("%s: num_extra_cmds=%d\n", __func__, ctrl_pdata->lge_extra.num_extra_cmds);
		ctrl_pdata->lge_extra.extra_cmds_array = kmalloc(sizeof(struct lge_cmds_entry)*ctrl_pdata->lge_extra.num_extra_cmds, GFP_KERNEL);
		if (NULL == ctrl_pdata->lge_extra.extra_cmds_array) {
			pr_err("%s: no memory\n", __func__);
			ctrl_pdata->lge_extra.num_extra_cmds = 0;
			return -ENOMEM;
		}
		for (i = 0; i < ctrl_pdata->lge_extra.num_extra_cmds; ++i) {
			of_property_read_string_index(np, "lge,mdss-dsi-extra-command-names", i, &name);
			strlcpy(ctrl_pdata->lge_extra.extra_cmds_array[i].name, name, sizeof(ctrl_pdata->lge_extra.extra_cmds_array[i].name));
			snprintf(buf1, sizeof(buf1), "lge,mdss-dsi-extra-command-%s", name);
			snprintf(buf2, sizeof(buf2), "lge,mdss-dsi-extra-command-state-%s", name);
			mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_extra.extra_cmds_array[i].cmds, buf1, buf2);
		}
	} else {
		ctrl_pdata->lge_extra.num_extra_cmds = 0;
	}

	return 0;
}

int lge_mdss_panel_parse_dt_extra(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc;
	u32 tmp;

	rc = of_property_read_u32(np, "lge,pre-on-cmds-delay", &tmp);
	ctrl_pdata->lge_extra.pre_on_cmds_delay = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "lge,post-ldo-on-delay", &tmp);
	ctrl_pdata->lge_extra.post_ldo_on_delay = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "lge,pre-bl-on-delay", &tmp);
	ctrl_pdata->lge_extra.pre_bl_on_delay = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "lge,esc-clk-rate", &tmp);
	if (rc) {
		pr_info("%s: esc-clk-rate not specified\n", __func__);
	} else {
		ctrl_pdata->lge_extra.esc_clk_rate = tmp;
		pr_info("%s: esc-clk-rate=%d\n", __func__, ctrl_pdata->lge_extra.esc_clk_rate);
	}

	ctrl_pdata->lge_extra.lp11_off = of_property_read_bool(np,
					"lge,mdss-dsi-lp11-off");

	ctrl_pdata->lge_extra.panel_id = 0;
	rc = of_property_read_u32(np, "lge,panel-id", &tmp);
	if(!rc && tmp >=0 && tmp <= 7) {
		ctrl_pdata->lge_extra.panel_id = tmp;
	} else {
		pr_info("%s: failed to parse panel_id\n", __func__);
	}

	parse_dt_extra_dcs_cmds(np, ctrl_pdata);

	return 0;
}

void lge_mdss_dsi_panel_extra_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, const char *name)
{
	int i, index = -1;

	if (ctrl == NULL)
		ctrl = lge_mdss_dsi_get_ctrl_pdata();

	if (ctrl == NULL)
		return;

	for (i = 0; i < ctrl->lge_extra.num_extra_cmds; ++i) {
		if (!strcmp(ctrl->lge_extra.extra_cmds_array[i].name, name)) {
			index = i;
			break;
		}
	}

	if (index != -1) {
		if (ctrl->lge_extra.extra_cmds_array[index].cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl, &ctrl->lge_extra.extra_cmds_array[index].cmds, CMD_REQ_COMMIT);
	} else {
		pr_err("%s: extra cmds %s not found\n", __func__, name);
	}
}

#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
void lge_mdss_panel_parse_dt_blmaps(struct device_node *np,
							struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i, j, rc;
	u32 *array;
	u32 tmp;

	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	rc = of_property_read_u32(np, "lge,blmap-size", &tmp);
	pinfo->blmap_size = (!rc ? tmp : 0);
	array = kzalloc(sizeof(u32) * pinfo->blmap_size, GFP_KERNEL);

	if (!array)
		return;

	for (i = 0; i < LGE_BLMAPMAX; i++) {
		/* check if property exists */
		if (!of_find_property(np, lge_blmap_name[i], NULL))
			continue;

		pr_info("%s: found %s\n", __func__, lge_blmap_name[i]);

		rc = of_property_read_u32_array(np, lge_blmap_name[i], array,
						pinfo->blmap_size);
		if (rc) {
			pr_err("%s:%d, unable to read %s\n",
					__func__, __LINE__, lge_blmap_name[i]);
			goto error;
		}

		pinfo->blmap[i] = kzalloc(sizeof(int) * pinfo->blmap_size,
				GFP_KERNEL);

		if (!pinfo->blmap[i]){
			goto error;
		}

		for (j = 0; j < pinfo->blmap_size; j++)
			pinfo->blmap[i][j] = array[j];

	}
	kfree(array);
	return;

error:
	for (i = 0; i < LGE_BLMAPMAX; i++)
		if (pinfo->blmap[i])
			kfree(pinfo->blmap[i]);
	kfree(array);
}
#endif

#if defined(CONFIG_LGE_DISPLAY_DAYLIGHT_MODE)
__weak int lge_mdss_dsi_set_daylight_mode(struct mdss_dsi_ctrl_pdata *ctrl, int mode)
{
	pr_err("%s is not implemented.\n", __func__);
	return 0;
}
#endif

static struct mdss_dsi_ctrl_pdata *dsi_ctrl_pdata = NULL;

struct mdss_dsi_ctrl_pdata *lge_mdss_dsi_get_ctrl_pdata(void)
{
	if (dsi_ctrl_pdata == NULL)
		pr_err("%s: %pS: dsi ctrl pdata is NULL\n", __func__, __builtin_return_address(0));
	return dsi_ctrl_pdata;
}

void lge_mdss_dsi_store_ctrl_pdata(struct mdss_dsi_ctrl_pdata *pdata)
{
	// store only dsi0
	if (dsi_ctrl_pdata == NULL)
		dsi_ctrl_pdata = pdata;
}

void lge_mdss_dsi_pr_status_buf(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 i, j, k = 0, l = 0, cnt, *lenp;
	char buf[256];

	if (ctrl == NULL)
		return;

	lenp = ctrl->status_valid_params ?: ctrl->status_cmds_rlen;
	cnt = ctrl->status_cmds.cmd_cnt;
	if (ctrl->return_buf == NULL || lenp == NULL || cnt == 0)
		return;

	for (i = 0; i < cnt; ++i) {
		for (j = 0; j < lenp[i]; ++j) {
			snprintf(buf+l, sizeof(buf)-l, "0x%02X ", ctrl->return_buf[k++]);
			l = strlen(buf);
		}
	}
	pr_info("%s: %s\n", __func__, buf);
}

__weak int lge_mdss_panel_select_initial_cmd_set(struct mdss_dsi_ctrl_pdata *ctrl)
{
	return 0;
}