#if IS_ENABLED(CONFIG_LGE_DISPLAY_DEBUG)

#ifndef LGE_MDSS_DEBUG_H
#define LGE_MDSS_DEBUG_H


#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/path.h>
#include <linux/namei.h>
#include <linux/of_platform.h>

#include "mdss_fb.h"
#include "mdss_dsi.h"

//NOTE : DO NOT USE THESE EVENTS WITH PANEL_OFF DSI EVENT because user data partition could be unmounted during file-access when device power-off
enum {
	DEBUG_DSI_CMD_TX = 0, //same format as dtsi input
	DEBUG_DSI_CMD_RX = 1, //same format as dtsi input and use dcs read(06) or generic read(14) as the data type
	DEBUG_DSI_TIMING_CHANGE = 2, //[ex : hfp hbp hpw vfp vbp vpw phy_timing clk_post clk_pre]
	DEBUG_PWR_SEQ_DELAY = 3, //use these values(ms) in your source code [ex for three powers : 10 10 10]
	DEBUG_PWR_ALWAYS_ON = 4, //use these flags in your source code and use this event only on MDSS_EVENT_POST_PANEL_ON [ex for three powers : 1 1 1]
	DEBUG_BLMAP_CHANGE = 5, //same format as dtsi input
	DEBUG_WLED_CURR_CHANGE = 6, //[ex : 18000]
	DEBUG_MDSS_FUDGE_FACTOR_CHANGE = 7, //to apply this, first see default values for each chipset's mdss.dtsi [ex : 100 100 105]
	DEBUG_DSV_REG_CHANGE = 8, //use this event only for adb interface [ex : echo 8 03 03 00 ff 01 ff ff 28 -> for sm, fd on & max voltage for +/- & knock-on mode]
	INVALID, //for adb interface, use this event to unlock debug event
};

struct debug_event_list {
	uint32_t id;
	char name[256];
};

struct debug_file_info {
	char file_name[256];
	char *cbuf;
	int *ibuf;
	loff_t file_size;
	int data_len;
	int data_type;
	int event;
};

int lge_debug_event_trigger(struct mdss_panel_data *pdata,
	char *debug_file, int debug_event);

#endif

#endif