#include <linux/delay.h>
#include "mdss_dsi.h"
#include "../lge/lge_mdss_dsi.h"
#include "lge_mdss_dsi_ph2n_global_ca.h"
#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
#include <soc/qcom/lge/board_lge.h>
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	pr_info("%s: + (override: ph2n_global_ca)\n", __func__);

	if(lge_get_panel_type() == PH2_SHARP) {

		lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
		LGE_MDELAY(2);

		lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 0);
		LGE_MDELAY(2);

		lge_extra_gpio_set_value(ctrl_pdata, "touch-vdddc", 0);
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);

	} else if(lge_get_panel_type() == PH2_JDI) {
		lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
		lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);
		lge_extra_gpio_set_value(ctrl_pdata, "touch-vdddc", 0);
		LGE_MDELAY(1);
		lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 0);
	}
	pr_info("%s: -\n", __func__);
	return;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_TOUCH_NOTIFIER_CALL_CHAIN)
int lge_get_lpwg_on_event(void)
{
	if (lge_get_panel_type() == PH2_SHARP)
		return MDSS_EVENT_BLANK;
	else
		return MDSS_EVENT_PANEL_OFF;
}
#endif
