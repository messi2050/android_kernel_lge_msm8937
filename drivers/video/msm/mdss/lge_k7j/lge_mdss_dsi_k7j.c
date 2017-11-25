#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
#include <linux/delay.h>
#include "mdss_dsi.h"
#include "../lge/lge_mdss_dsi.h"
#include "lge_mdss_dsi_k7j.h"

void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	pr_info("%s: + (override: k7j)\n", __func__);

	lge_extra_gpio_set_value(ctrl_pdata, "touch-reset", 0);
#if 0 //not necessary, because dsv is controlled by touch driver for ttw mode
	lge_mdss_dsi_panel_power_labibb_ctrl(ctrl_pdata, 0);
#endif
	LGE_MDELAY(1);

	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);

	pr_info("%s: -\n", __func__);
	return;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_TOUCH_NOTIFIER_CALL_CHAIN)
int lge_get_lpwg_off_event(void)
{
	return MDSS_EVENT_LINK_READY;
}
#endif
