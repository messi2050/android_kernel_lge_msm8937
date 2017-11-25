#ifndef _LGE_MDSS_SYSFS_H
#define _LGE_MDSS_SYSFS_H

#include <linux/of_platform.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <soc/qcom/lge/board_lge.h>
#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_dsi.h"

int lge_mdss_sysfs_init(struct fb_info *fbi);

#if defined(CONFIG_LGE_DISPLAY_BL_SP_MIRRORING)
int sp_link_backlight_status;
int sp_link_backlight_brightness;
int sp_link_backlight_is_ready;
#endif
#endif
