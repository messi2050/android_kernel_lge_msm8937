#include "lge_mdss_sysfs.h"

struct class *panel = NULL;					/* lge common class node "/sys/class/panel/" */
struct device *panel_sysfs_dev = NULL;
struct device *lge_panel_sysfs_dev = NULL;	/* lge common device node "/sys/class/panel/dev0/" */
struct fb_info *fbi = NULL;
struct msm_fb_data_type *mfd = NULL;
struct mdss_panel_data *pdata = NULL;
struct mdss_dsi_ctrl_pdata *ctrl = NULL;

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
extern ssize_t lge_get_multi_panel_support_flag(struct device *dev, struct device_attribute *attr, char *buf);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
extern ssize_t set_reader_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t get_reader_mode(struct device *dev, struct device_attribute *attr, char *buf);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
extern ssize_t get_lge_debug_event(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_lge_debug_event(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
ssize_t mdss_fb_get_panel_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int panel_type = lge_get_panel_type();

	switch(panel_type) {
		case PH2_JDI:
		case LV9_JDI_NT35596:
			ret = snprintf(buf, PAGE_SIZE, "JDI-NT35596S\n");
			break;
		case PH2_SHARP:
			ret = snprintf(buf, PAGE_SIZE, "SHARP-NT35596\n");
			break;
		case PH2_LGD_DB7400:
			ret = snprintf(buf, PAGE_SIZE, "LGD-DB7400\n");
			break;
		case JAPAN_LGD_FT8707_1_0:
		case JAPAN_LGD_FT8707_1_1:
			ret = snprintf(buf, PAGE_SIZE, "LGD-FT8707\n");
			break;
		case JAPAN_LGD_TD4300:
			ret = snprintf(buf, PAGE_SIZE, "LGD-TD4300\n");
			break;
		case LV3_TIANMA:
			ret = snprintf(buf, PAGE_SIZE, "TIANMA-FT8607\n");
			break;
		case LV3_LGD:
		case LV5_LGD:
			ret = snprintf(buf, PAGE_SIZE, "LGD-LG4894\n");
			break;
		case SF3_TOVIS:
		case LV5_TOVIS:
		case LV7_TOVIS:
			ret = snprintf(buf, PAGE_SIZE, "TOVIS-TD4100\n");
			break;
		case SF3_LGD_TD4100:
			ret = snprintf(buf, PAGE_SIZE, "LGD-TD4100\n");
			break;
		case SF3F_TD4310:
			ret = snprintf(buf, PAGE_SIZE, "LGD-TD4310\n");
			break;
		case SF3F_SW49105:
			ret = snprintf(buf, PAGE_SIZE, "LGD-SW49105\n");
			break;
		case TOVIS_HX8394C:
			ret = snprintf(buf, PAGE_SIZE, "TOVIS-HX8394C\n");
			break;
		case TF8_INX_NT51021:
			ret = snprintf(buf, PAGE_SIZE, "INX-NT51021\n");
			break;
		case LGD_INCELL_SW49106:
			ret = snprintf(buf, PAGE_SIZE, "LGD-SW49106\n");
			break;
		default:
			ret = snprintf(buf, PAGE_SIZE, "UNDEFINED\n");
			break;
	}
	return ret;
}

ssize_t lge_get_multi_panel_support_flag(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0, val = 0;

	fbi = dev_get_drvdata(dev);
	if (fbi == NULL) {
		pr_err("%s : uninitialzed fb0\n", __func__);
		return -EINVAL;
	}

	mfd = (struct msm_fb_data_type *)fbi->par;
	if (mfd == NULL) {
		pr_err("%s : uninitialzed mfd\n", __func__);
		return -EINVAL;
	}

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("no panel connected!\n");
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl) {
		pr_err("%s: ctrl is null\n", __func__);
		return -EINVAL;
	}

	val = BIT(ctrl->lge_extra.panel_id);
	ret = scnprintf(buf, PAGE_SIZE, "%d\n", val);

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
extern int lge_mdss_report_panel_dead(void);

static ssize_t mdss_fb_set_esd_lcd_reset(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	lge_mdss_report_panel_dead();
	return count;
}
#endif

#if defined(CONFIG_LGE_DISPLAY_DAYLIGHT_MODE)
ssize_t daylight_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *pinfo;
	int ret;

	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("[daylight_mode] no panel connected!\n");
		return -EINVAL;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pinfo->daylight_mode);
	return ret;
}

ssize_t daylight_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *pinfo;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_data *pdata;
	u32 value;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("[daylight_mode] no panel connected!\n");
		return len;
	}

	value = simple_strtoul(buf, NULL, 10);

	if(pinfo->daylight_mode != value) {
		pinfo->daylight_mode = value;
		lge_mdss_dsi_set_daylight_mode(ctrl, value);
	}

	return len;
}
#endif

#if defined(CONFIG_LGE_HIGH_LUMINANCE_MODE)
ssize_t hl_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *pinfo;
	int ret;

	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("[hl_mode] no panel connected!\n");
		return -EINVAL;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pinfo->hl_mode_on);
	return ret;
}

ssize_t hl_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *pinfo;

	pinfo = mfd->panel_info;

	if (!pinfo) {
		pr_err("[hl_mode] no panel connected!\n");
		return len;
	}

	pinfo->hl_mode_on = simple_strtoul(buf, NULL, 10);

	if(pinfo->hl_mode_on == 1)
		pr_info("[hl_mode] hl_mode on\n");
	else
		pr_info("[hl_mode] hl_mode off\n");

	return len;
}
#endif

#if defined(CONFIG_LGE_DISPLAY_BL_SP_MIRRORING)
static ssize_t mdss_fb_get_sp_link_backlight_off(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "sp link backlight status : %d\n", sp_link_backlight_status);
	return ret;
}

static ssize_t mdss_fb_set_sp_link_backlight_off(struct device *dev,
		struct device_attribute *attr,const char *buf,  size_t count)
{
	if(!count || !sp_link_backlight_is_ready ||!dev) {
		pr_warn("%s invalid value : %d, %d || NULL check\n", __func__,(int) count, sp_link_backlight_is_ready);
		return -EINVAL;
	}

	fbi = dev_get_drvdata(dev);
	mfd = fbi->par;

	sp_link_backlight_status = simple_strtoul(buf, NULL, 10);
	if(sp_link_backlight_status) {
		pr_info("[%s] status : %d, brightness : 0 \n", __func__,
			sp_link_backlight_status);
		mdss_fb_set_backlight(mfd, 0);
	} else {
		pr_info("[%s] status : %d, brightness : %d \n", __func__,
			sp_link_backlight_status, sp_link_backlight_brightness);
		mdss_fb_set_backlight(mfd, sp_link_backlight_brightness);
	}

	return count;
}
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_BL_USE_BLMAP)
ssize_t mdss_fb_set_blmap_index(
        struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t len);
ssize_t mdss_fb_get_blmap_index(
        struct device *dev,
        struct device_attribute *attr,
        char *buf);
ssize_t mdss_fb_set_blmap_value(
        struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t len);
ssize_t mdss_fb_get_blmap_value(
        struct device *dev,
        struct device_attribute *attr,
        char *buf);
ssize_t mdss_fb_get_blmap_size(
        struct device *dev,
        struct device_attribute *attr,
        char *buf);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
static DEVICE_ATTR(panel_flag, S_IRUGO, lge_get_multi_panel_support_flag, NULL);
static DEVICE_ATTR(panel_type, S_IRUGO, mdss_fb_get_panel_type, NULL);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
static DEVICE_ATTR(lcd_esd_reset, S_IRUGO | S_IWUSR | S_IWGRP,  NULL, mdss_fb_set_esd_lcd_reset);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DAYLIGHT_MODE)
static DEVICE_ATTR(daylight_mode, S_IWUSR|S_IRUGO, daylight_mode_show, daylight_mode_store);
#endif
#if IS_ENABLED(CONFIG_LGE_HIGH_LUMINANCE_MODE)
static DEVICE_ATTR(hl_mode, S_IWUSR|S_IRUGO, hl_mode_show, hl_mode_store);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
static DEVICE_ATTR(reader_mode, S_IRUGO | S_IWUSR, get_reader_mode, set_reader_mode);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
static DEVICE_ATTR(lge_debug_event, S_IRUGO | S_IWUSR, get_lge_debug_event, set_lge_debug_event);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_BL_SP_MIRRORING)
static DEVICE_ATTR(sp_link_backlight_off, S_IRUGO | S_IWUSR,
	mdss_fb_get_sp_link_backlight_off, mdss_fb_set_sp_link_backlight_off);
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_BL_USE_BLMAP)
static DEVICE_ATTR(blmap_index, S_IRUGO | S_IWUSR, mdss_fb_get_blmap_index, mdss_fb_set_blmap_index);
static DEVICE_ATTR(blmap_value, S_IRUGO | S_IWUSR, mdss_fb_get_blmap_value, mdss_fb_set_blmap_value);
static DEVICE_ATTR(blmap_size, S_IRUGO, mdss_fb_get_blmap_size, NULL);
#endif


/* "/sys/class/panel/dev0/" */
struct attribute *lge_mdss_panel_sysfs_list[] = {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
	&dev_attr_panel_flag.attr,
#endif
	NULL,
};

/* "/sys/class/graphics/fb0/" */
struct attribute *lge_mdss_fb_sysfs_list[] = {
#if IS_ENABLED(CONFIG_LGE_DISPLAY_COMMON)
	&dev_attr_panel_type.attr,
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_RECOVERY_ESD)
	&dev_attr_lcd_esd_reset.attr,
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_READER_MODE)
	&dev_attr_reader_mode.attr,
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)
	&dev_attr_lge_debug_event.attr,
#endif
#if IS_ENABLED(CONFIG_LGE_HIGH_LUMINANCE_MODE)
	&dev_attr_hl_mode.attr,
#endif
#if defined(CONFIG_LGE_DISPLAY_BL_SP_MIRRORING)
		&dev_attr_sp_link_backlight_off.attr,
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_BL_USE_BLMAP)
	&dev_attr_blmap_index.attr,
	&dev_attr_blmap_value.attr,
	&dev_attr_blmap_size.attr,
#endif
	NULL,
};

static struct attribute_group lge_mdss_fb_sysfs_group = {
	.attrs = lge_mdss_fb_sysfs_list,
};

static struct attribute_group lge_mdss_panel_sysfs_group = {
	.attrs = lge_mdss_panel_sysfs_list,
};

int lge_mdss_sysfs_init(struct fb_info *fbi)
{
	int ret = 0;

	if(!panel) {
		panel = class_create(THIS_MODULE, "panel");
		if (IS_ERR(panel))
			pr_err("%s: Failed to create panel class\n", __func__);
	}

	if(!panel_sysfs_dev) {
		panel_sysfs_dev = device_create(panel, NULL, 0, fbi, "img_tune");
		if(IS_ERR(panel_sysfs_dev)) {
			pr_err("%s: Failed to create dev(panel_sysfs_dev)!", __func__);
		}
		else {
#if defined(CONFIG_LGE_DISPLAY_DAYLIGHT_MODE)
			if (device_create_file(panel_sysfs_dev, &dev_attr_daylight_mode) < 0)
				pr_err("%s: add sre set node fail!", __func__);
#endif
		}
	}

	if(!lge_panel_sysfs_dev) {
		lge_panel_sysfs_dev = device_create(panel, NULL, 0, fbi, "dev0");
		if (IS_ERR(lge_panel_sysfs_dev)) {
			pr_err("%s: Failed to create lge_panel_sysfs_dev class\n", __func__);
		}else{
			ret += sysfs_create_group(&lge_panel_sysfs_dev->kobj, &lge_mdss_panel_sysfs_group);
		}
	}
	ret += sysfs_create_group(&fbi->dev->kobj, &lge_mdss_fb_sysfs_group);

	if (ret)
		return ret;

	return 0;
}
