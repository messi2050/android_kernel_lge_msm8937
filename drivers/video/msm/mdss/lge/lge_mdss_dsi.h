#ifndef LGE_MDSS_DSI_H
#define LGE_MDSS_DSI_H

struct lge_supply_entry {
	char name[32];
};

struct lge_gpio_entry {
	char name[32];
	int gpio;
};

struct lge_cmds_entry {
	char name[32];
	struct dsi_panel_cmds cmds;
};

struct lge_mdss_dsi_ctrl_pdata {
	/* gpio */
	int num_gpios;
	struct lge_gpio_entry *gpio_array;

	/* esc_clk_rate */
	int esc_clk_rate;

	/* lp11 during lcd power off */
	char lp11_off;

	/* delay */
	int pre_on_cmds_delay;
	int post_ldo_on_delay;
	int pre_bl_on_delay;

	/* cmds */
	int num_extra_cmds;
	struct lge_cmds_entry *extra_cmds_array;

	/* extra power */
	struct dss_module_power extra_power_data;
	int extra_power_state;

	/* multi support panel */
	int panel_id;
};

#define LGE_MDELAY(m) do { if ( m > 0) usleep_range((m)*1000,(m)*1000); } while(0)
#define LGE_OVERRIDE_VALUE(x, v) do { if ((v)) (x) = (v); } while(0)

#include "lge/lge_mdss_dsi_panel.h"
#include "lge/lge_mdss_sysfs.h"

int lge_mdss_dsi_parse_extra_params(struct platform_device *ctrl_pdev,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_init_extra_pm(struct platform_device *ctrl_pdev,
        struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void lge_mdss_dsi_deinit_extra_pm(struct platform_device *pdev,
        struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void lge_extra_gpio_set_value(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	const char *name, int value);
int lge_extra_gpio_request(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	const char *name);
void lge_extra_gpio_free(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	const char *name);
#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_TOUCH_NOTIFIER_CALL_CHAIN)
int lge_get_lpwg_off_event(void);
int lge_get_lpwg_on_event(void);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_PRE_ACTIVE_AREA_DELAY)
void lge_mdss_dsi_calc_pre_active_area_delay(struct mdss_panel_info *pinfo);
#endif
int lge_mdss_dsi_post_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int lge_mdss_dsi_panel_power_seq_all(void);
int lge_get_panel_recovery_flag(void);
void lge_set_panel_recovery_flag(int flag);
#endif
