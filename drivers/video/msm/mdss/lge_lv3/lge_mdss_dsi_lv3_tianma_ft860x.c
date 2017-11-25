#include <linux/delay.h>
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "lge/mfts_mode.h"
#include <linux/mfd/dw8768.h>
#include "lge_mdss_dsi_lv3.h"
#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
#include "mdss_fb.h"
#endif

int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
					bool active);

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_panel_cmds *pcmds, u32 flags);

static atomic_t boot_fw_recovery = ATOMIC_INIT(0);

int get_boot_fw_recovery(void)
{
	return atomic_read(&boot_fw_recovery);
}

void set_boot_fw_recovery(int recovery)
{
	atomic_set(&boot_fw_recovery, recovery);
}

#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
static atomic_t esd_power_recovery = ATOMIC_INIT(ESD_OK);

int tianma_ft860x_get_esd_power_recovery(void)
{
	return atomic_read(&esd_power_recovery);
}

void tianma_ft860x_set_esd_power_recovery(int esd_detection)
{
	atomic_set(&esd_power_recovery, esd_detection);
}
#endif

int tianma_ft860x_reset_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		int enable)
{
	int rc = 0;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (enable) {
		gpio_set_value((ctrl_pdata->rst_gpio), 1);
	} else {
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}

	return rc;
}

int tianma_ft860x_cmd_transfer(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		int enable)
{
	int rc = 0;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (enable) {
		if (ctrl_pdata->on_cmds.cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->on_cmds, CMD_REQ_COMMIT);
			pr_info("%s: on cmds sent, link_state[%d] \n",
					__func__, ctrl_pdata->on_cmds.link_state);
		}
	} else {
		if (ctrl_pdata->off_cmds.cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->off_cmds, CMD_REQ_COMMIT);
			pr_info("%s: off cmds sent, link_state[%d] \n",
					__func__, ctrl_pdata->off_cmds.link_state);
		}
	}

	return rc;
}

int tianma_ft860x_panel_external_api(int type, int enable)
{
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *pdata = NULL;

	pdata = lge_mdss_dsi_get_ctrl_pdata();
	if (pdata == NULL)
		return -ENODEV;

	switch(type) {
		case VDDI_CTRL:
			break;
		case AVDD_AVEE_CTRL:
			break;
		case LCD_RESET_CTRL:
			rc = tianma_ft860x_reset_ctrl(pdata, enable);
			break;
		case LCD_INIT_CMD_TRANSFER:
			rc = tianma_ft860x_cmd_transfer(pdata, enable);
			break;
		case NONE:
		default:
			break;
	}

	return rc;
}

int tianma_ft860x_firmware_recovery(void)
{
	struct mdss_panel_data *pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	u32 backup_level = 0;
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = lge_mdss_dsi_get_ctrl_pdata();
	if (ctrl_pdata == NULL)
		return -ENODEV;


	pr_info("firmware update detected, LCD recovery function called\n");

	pdata = &(ctrl_pdata->panel_data);
	if (pdata == NULL) {
		pr_err("%s: no panel_data\n", __func__);
		return -EINVAL;
	}

	pinfo = &(ctrl_pdata->panel_data.panel_info);
	if (pinfo->cont_splash_enabled) {
		pr_info("%s: firmware recovery called before dsi init\n", __func__);
		set_boot_fw_recovery(1);
		return -EPERM;
	}

	if (pinfo->blmap_size)
		backup_level = pinfo->blmap[pinfo->blmap_size-1];
	else
		backup_level = pinfo->bl_max;

	if (pdata->set_backlight)
		pdata->set_backlight(pdata, 0);

	rc = tianma_ft860x_panel_external_api(LCD_INIT_CMD_TRANSFER, 0);
	rc |= tianma_ft860x_panel_external_api(LCD_RESET_CTRL, 1);
	LGE_MDELAY(1);
	rc |= tianma_ft860x_panel_external_api(LCD_RESET_CTRL, 0);
	LGE_MDELAY(1);
	rc |= tianma_ft860x_panel_external_api(LCD_RESET_CTRL, 1);
	LGE_MDELAY(200);
	rc |= tianma_ft860x_panel_external_api(LCD_INIT_CMD_TRANSFER, 1);

	if (pdata->set_backlight)
		pdata->set_backlight(pdata, backup_level);

	if (rc) {
		pr_err("%s: panel recovery fail\n", __func__);
		return -1;
	}
	return 0;
}

int tianma_ft860x_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	switch (event) {
	case MDSS_EVENT_POST_PANEL_ON:
		if (get_boot_fw_recovery() == 1) {
			tianma_ft860x_firmware_recovery();
			set_boot_fw_recovery(0);
		}
		break;
	default:
		break;
	}

	return rc;
}

int tianma_ft860x_mdss_dsi_panel_power_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &(ctrl_pdata->panel_data.panel_info);
#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
	if (lge_get_display_power_ctrl() || tianma_ft860x_get_esd_power_recovery() == ESD_NOK) {
#else
	if (lge_get_display_power_ctrl()) {
#endif
		pr_info("%s: turn panel power on\n", __func__);

		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 1);
		if (ret) {
			pr_err("%s: failed to enable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
			return ret;
		}
		if (lge_get_display_power_ctrl()) {
			lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 0);
			pr_err("mfts-power gpio set\n");
		}

		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 1);
		usleep_range(1000, 1000);

        pr_info("%s: esd power on function!!\n", __func__);

		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 1);
		usleep_range(50000, 50000);

		dw8768_register_set(0x05, 0x0F);
		LGE_MDELAY(1);
		if (dw8768_set_output_voltage(0x0E))
			pr_err("%s: setting DSV output voltage failed\n", __func__);
#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
		if (tianma_ft860x_get_esd_power_recovery() == ESD_NOK)
			tianma_ft860x_set_esd_power_recovery(ESD_OK);

	} else if (tianma_ft860x_get_esd_power_recovery() == ESD_POWEROFF_PENDING){

		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);
		pr_info("%s: mfts or esd recovery dsv off [0]\n", __func__);

		mdelay(5);
		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 1);
		usleep_range(50000, 50000);

		dw8768_register_set(0x05, 0x0F);
		LGE_MDELAY(1);
		if (dw8768_set_output_voltage(0x0E))
			pr_err("%s: setting DSV output voltage failed\n", __func__);

		tianma_ft860x_set_esd_power_recovery(ESD_OK);
#endif
	}else{
		pr_info("%s: skip panel power control\n", __func__);
	}

	/*
	 * If continuous splash screen feature is enabled, then we need to
	 * request all the GPIOs that have already been configured in the
	 * bootloader. This needs to be done irresepective of whether
	 * the lp11_init flag is set or not.
	 */
	if (pdata->panel_info.cont_splash_enabled ||
		!pdata->panel_info.mipi.lp11_init) {
		if (mdss_dsi_pinctrl_set_state(ctrl_pdata, true))
			pr_debug("reset enable: pinctrl not enabled\n");

		ret = mdss_dsi_panel_reset(pdata, 1);
		if (ret)
			pr_err("%s: Panel reset failed. rc=%d\n",
					__func__, ret);
	}
	return ret;
}

int tianma_ft860x_mdss_dsi_panel_power_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto end;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_dsi_panel_reset(pdata, 0);
	if (ret) {
		pr_warn("%s: Panel reset failed. rc=%d\n", __func__, ret);
		ret = 0;
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");
#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
	if (lge_get_display_power_ctrl()|| tianma_ft860x_get_esd_power_recovery()) {
#else
	if (lge_get_display_power_ctrl()) {
#endif
		pr_info("%s: turn panel power off\n", __func__);

#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
		if (tianma_ft860x_get_esd_power_recovery())
			tianma_ft860x_set_esd_power_recovery(ESD_NOK);
#endif
		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);

		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);


		if (lge_get_display_power_ctrl()) {
			lge_extra_gpio_set_value(ctrl_pdata, "mfts-power", 1);
			pr_err("mfts-power gpio unset\n");
			//lge_extra_gpio_free(ctrl_pdata, "mfts-power");
		}
		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
	} else {
		pr_info("%s: keep panel power for lpwg mode\n", __func__);
	}
end:
	return ret;
}

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
        int rc = 0;

        rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
        if (rc) {
                pr_err("request reset gpio failed, rc=%d\n",
                        rc);
        }
#if 0
#if IS_ENABLED(CONFIG_LGE_DISPLAY_MFTS)
		if(lge_get_display_power_ctrl())
		{
			rc = lge_extra_gpio_request(ctrl_pdata, "mfts-power");
			if (rc) {
				pr_err("request mfts-power gpio failed, rc=%d\n",
					rc);
			}
			else
				pr_err("MFTS power ctrl gpio is aligned\n");
		}
#endif
#endif
	return rc;
}

int tianma_ft860x_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
        struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
        struct mdss_panel_info *pinfo = NULL;
        int i, rc = 0;

        if (pdata == NULL) {
                pr_err("%s: Invalid input data\n", __func__);
                return -EINVAL;
        }

        ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
                                panel_data);

        pinfo = &(ctrl_pdata->panel_data.panel_info);
        if ((mdss_dsi_is_right_ctrl(ctrl_pdata) &&
                mdss_dsi_is_hw_config_split(ctrl_pdata->shared_data)) ||
                        pinfo->is_dba_panel) {
                pr_debug("%s:%d, right ctrl gpio configuration not needed\n",
                        __func__, __LINE__);
                return rc;
        }

        if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
                pr_debug("%s:%d, reset line not configured\n",
                           __func__, __LINE__);
                return rc;
        }

        pr_debug("%s: enable = %d\n", __func__, enable);

        if (enable) {
                rc = mdss_dsi_request_gpios(ctrl_pdata);
                if (rc) {
                        pr_err("gpio request failed\n");
                        return rc;
                }
            if (!pinfo->cont_splash_enabled) {
				pr_info("%s: panel reset high\n", __func__);
				if (pdata->panel_info.rst_seq_len) {
					rc = gpio_direction_output(ctrl_pdata->rst_gpio,
						pdata->panel_info.rst_seq[0]);
					if (rc) {
						pr_err("%s: unable to set dir for rst gpio\n", __func__);
						goto exit;

					}
				}
				for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
					gpio_set_value((ctrl_pdata->rst_gpio),
						pdata->panel_info.rst_seq[i]);
					if (pdata->panel_info.rst_seq[++i])
						usleep_range(pinfo->rst_seq[i] * 1000, pinfo->rst_seq[i] * 1000);
				}
            }
                if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
                        pr_debug("%s: Panel Not properly turned OFF\n",
                                                __func__);
                        ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
                        pr_debug("%s: Reset panel done\n", __func__);
				}
        } else {
#if IS_ENABLED(CONFIG_LGE_LCD_ESD_PANEL_POWER_RECOVERY)
		if (lge_get_display_power_ctrl()|| tianma_ft860x_get_esd_power_recovery()) {
#else
	    if (lge_get_display_power_ctrl()) {
#endif
			pr_info("%s: panel reset low\n", __func__);
	                gpio_set_value((ctrl_pdata->rst_gpio), 0);
		} else {
			pr_info("%s: skip panel reset low\n", __func__);
		}
                gpio_free(ctrl_pdata->rst_gpio);
        }

exit:
        return rc;
}


void tianma_ft860x_mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_info("%s:reset to low\n", __func__);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

		lge_extra_gpio_set_value(ctrl_pdata, "dsv-ena", 0);
		usleep_range(10000, 10000);
		lge_extra_gpio_set_value(ctrl_pdata, "vdda", 0);

		ret = msm_dss_enable_vreg(
			ctrl_pdata->panel_power_data.vreg_config,
			ctrl_pdata->panel_power_data.num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
		pr_info("%s: turn panel shutdown\n", __func__);
}
