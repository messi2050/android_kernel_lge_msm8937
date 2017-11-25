#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310)
int sf3f_lgd_td4310_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
int sf3f_lgd_td4310_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
//int sf3f_lgd_td4310_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int sf3f_lgd_td4310_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int sf3f_lgd_td4310_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)
int sf3f_lgd_sw49105_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata);
int sf3f_lgd_sw49105_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata);
//int sf3f_lgd_sw49105_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int sf3f_lgd_sw49105_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int sf3f_lgd_sw49105_mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
#endif

#if defined (CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
int get_esd_power_recovery(void);
void set_esd_power_recovery(int esd_detection);
#endif
