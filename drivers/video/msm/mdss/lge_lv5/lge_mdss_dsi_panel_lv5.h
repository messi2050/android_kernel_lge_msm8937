#ifndef _LGE_MDSS_DSI_PANEL_LV5_H
#define _LGE_MDSS_DSI_PANEL_LV5_H

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);
extern int mdss_dsi_parse_dcs_cmds(struct device_node *np, struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

extern struct mdss_panel_data *lgd_lg4894_pdata_base;

#endif
