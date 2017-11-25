#include <linux/delay.h>
#include "mdss_dsi.h"
#include "../lge/lge_mdss_dsi.h"
#include "lge_mdss_dsi_ph2n_tmo_us.h"
#include "lge_mdss_dsi_panel_ph2n_tmo_us.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
extern void MIT300_Reset(int status, int delay);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_OVERRIDE_MDSS_DSI_CTRL_SHUTDOWN)
void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);
	int dsv_type;
	int rc = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	pr_info("%s: + (override: ph2n_tmo_us)\n", __func__);

	if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_info("%s: [LCD] reset to low\n", __func__);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}
#if defined(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER4)
	pr_info("%s: [TOUCH] T_Reset to low\n", __func__);
	MIT300_Reset(0, 2);
#endif
	mdelay(1);

	dsv_type = lge_get_dsv_type();

	if(dsv_type == LGE_DSV_PMI8952) {
		pr_info("%s: [LCD] DDVDH, DDVDL off\n", __func__);
		rc = msm_dss_enable_vreg(
			ctrl_pdata->lge_extra.extra_power_data.vreg_config,
			ctrl_pdata->lge_extra.extra_power_data.num_vreg, 0);
		if (rc) {
			pr_err("%s: failed to init regulator, rc=%d\n", __func__, rc);
		}
	} else if(dsv_type == LGE_DSV_DW8768) {
		pr_info("%s: [LCD] DDVDH, DDVDL off\n", __func__);
		lge_extra_gpio_set_value(ctrl_pdata, "dsv-enm", 0);
		LGE_MDELAY(5);
		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);
	} else {
		pr_err("%s: wrong dsv type.\n", __func__);
	}

	pr_info("%s: [LCD] turn off VPNL, VDDI\n", __func__);
	lge_extra_gpio_set_value(ctrl_pdata, "touch-avdd", 0);
	usleep_range(2000, 2000);
	lge_extra_gpio_set_value(ctrl_pdata, "vddio", 0);

	pr_info("%s: -\n", __func__);
	return;
}
#endif
