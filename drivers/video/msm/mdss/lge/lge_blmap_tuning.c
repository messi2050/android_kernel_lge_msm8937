/*
 * Copyright (C) 2016 LGE
 * Copyright (c) 2008-2016, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#if IS_ENABLED(CONFIG_LGE_DISPLAY_BL_USE_BLMAP)
#include <linux/string.h>
#include <linux/device.h>
#include "../mdss_fb.h"

static int blmap_index = 0;

ssize_t mdss_fb_set_blmap_index(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t len)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = fbi->par;
	struct mdss_panel_info *pinfo = mfd->panel_info;
	int rc = 0;

	rc = kstrtoint(buf, 10, &blmap_index);
	if (rc) {
		pr_err("%s: failed to set blmap_index (%s)\n", __func__, buf);
		return rc;
	}

	if (blmap_index < 0 || blmap_index >= pinfo->blmap_size) {
		blmap_index = 0;
		pr_err("%s: failed to set blmap_index\n", __func__);
	}

	return len;
}

ssize_t mdss_fb_get_blmap_index(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	ssize_t ret = 0;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", blmap_index);

	return ret;
}

ssize_t mdss_fb_set_blmap_value(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t len)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = fbi->par;
	struct mdss_panel_info *pinfo = mfd->panel_info;
	int rc = 0;
	int value = 0;

	if (blmap_index < 0 || blmap_index >= pinfo->blmap_size) {
		pr_err("%s: invalid blmap_index: %d\n", __func__, blmap_index);
		return -EINVAL;
	}

	rc = kstrtoint(buf, 10, &value);
	if (rc) {
		pr_err("%s: invalid parameter: %s\n", __func__, buf);
		return rc;
	}

	pinfo->blmap[blmap_index] = value;

	return len;
}

ssize_t mdss_fb_get_blmap_value(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = fbi->par;
	struct mdss_panel_info *pinfo = mfd->panel_info;
	ssize_t ret = 0;

	if (blmap_index < 0 || blmap_index >= pinfo->blmap_size) {
		ret = scnprintf(buf, PAGE_SIZE, "invalid blmap_index: %d\n", blmap_index);
		pr_err("%s: %s", __func__, buf);
		return ret;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pinfo->blmap[blmap_index]);
	return ret;
}

ssize_t mdss_fb_get_blmap_size(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = fbi->par;
	struct mdss_panel_info *pinfo = mfd->panel_info;
	ssize_t ret = 0;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pinfo->blmap_size);
	return ret;
}
#endif
