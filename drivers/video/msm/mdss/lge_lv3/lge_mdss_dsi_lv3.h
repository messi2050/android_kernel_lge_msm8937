#ifndef LGE_MDSS_DSI_LV3
#define LGE_MDSS_DSI_LV3

#if IS_ENABLED(CONFIG_LGE_DISPLAY_LV3_TIANMA_FT860X)
int tianma_ft860x_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int tianma_ft860x_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
int tianma_ft860x_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
int tianma_ft860x_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int tianma_ft860x_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
void tianma_ft860x_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
enum external_api_type {
	VDDI_CTRL,
	AVDD_AVEE_CTRL,
	LCD_RESET_CTRL,
	LCD_INIT_CMD_TRANSFER,
	NONE,
};
int tianma_ft860x_get_esd_power_recovery(void);
void tianma_ft860x_set_esd_power_recovery(int esd_detection);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_LV3_LGD_LG4894)
int lv3_lgd_lg4894_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int lv3_lgd_lg4894_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
int lv3_lgd_lg4894_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
int lv3_lgd_lg4894_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lv3_lgd_lg4894_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
void lv3_lgd_lg4894_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
#endif
#endif
