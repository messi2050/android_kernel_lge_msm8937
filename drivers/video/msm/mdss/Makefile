ccflags-y += -I$(src)

obj-$(CONFIG_FB_MSM_MDSS_MHL3) += mhl3/

mdss-mdp3-objs = mdp3.o mdp3_dma.o mdp3_ctrl.o dsi_status_v2.o
mdss-mdp3-objs += mdp3_ppp.o mdp3_ppp_hwio.o mdp3_ppp_data.o
obj-$(CONFIG_FB_MSM_MDSS_MDP3) += mdss-mdp3.o
ifeq ($(CONFIG_FB_MSM_MDSS_MDP3), y)
ccflags-y += -DTARGET_HW_MDSS_MDP3
endif
mdss-mdp-objs := mdss_mdp.o mdss_mdp_ctl.o mdss_mdp_pipe.o mdss_mdp_util.o dsi_status_6g.o
mdss-mdp-objs += mdss_mdp_pp.o mdss_mdp_pp_debug.o mdss_mdp_pp_cache_config.o
mdss-mdp-objs += mdss_mdp_intf_video.o
mdss-mdp-objs += mdss_mdp_intf_cmd.o
mdss-mdp-objs += mdss_mdp_intf_writeback.o
mdss-mdp-objs += mdss_rotator.o
mdss-mdp-objs += mdss_mdp_overlay.o
mdss-mdp-objs += mdss_mdp_layer.o
mdss-mdp-objs += mdss_mdp_splash_logo.o
mdss-mdp-objs += mdss_mdp_cdm.o
mdss-mdp-objs += mdss_smmu.o
mdss-mdp-objs += mdss_mdp_wfd.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss-mdp.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_mdp_debug.o

mdss-mdp-objs += mdss_mdp_pp_v1_7.o
mdss-mdp-objs += mdss_mdp_pp_v3.o
mdss-mdp-objs += mdss_mdp_pp_common.o

ifeq ($(CONFIG_FB_MSM_MDSS),y)
obj-$(CONFIG_DEBUG_FS) += mdss_debug.o mdss_debug_xlog.o
endif

mdss-dsi-objs := mdss_dsi.o mdss_dsi_host.o mdss_dsi_cmd.o mdss_dsi_status.o
mdss-dsi-objs += mdss_dsi_panel.o
ifeq ($(CONFIG_LGE_DISPLAY_READER_MODE),y)
mdss-dsi-objs += lge/reader_mode.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_COMMON),y)
mdss-dsi-objs += lge/lge_mdss_dsi.o
mdss-dsi-objs += lge/lge_mdss_dsi_panel.o
mdss-dsi-objs += lge/lge_mdss_sysfs.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_PH2_GLOBAL_COM),y)
mdss-dsi-objs += lge_ph2_global_com/lge_mdss_dsi_ph2_global_com.o
mdss-dsi-objs += lge_ph2_global_com/lge_mdss_dsi_panel_ph2_global_com.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_PH2_CMO_CN),y)
mdss-dsi-objs += lge_ph2_cmo_cn/lge_mdss_dsi_ph2_cmo_cn.o
mdss-dsi-objs += lge_ph2_cmo_cn/lge_mdss_dsi_panel_ph2_cmo_cn.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_PH2N_TMO_US),y)
mdss-dsi-objs += lge_ph2n_tmo_us/lge_mdss_dsi_ph2n_tmo_us.o
mdss-dsi-objs += lge_ph2n_tmo_us/lge_mdss_dsi_panel_ph2n_tmo_us.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_PH2N_MPCS_US),y)
mdss-dsi-objs += lge_ph2n_tmo_us/lge_mdss_dsi_ph2n_mpcs_us.o
mdss-dsi-objs += lge_ph2n_tmo_us/lge_mdss_dsi_panel_ph2n_mpcs_us.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_PH2N_GLOBAL_CA),y)
mdss-dsi-objs += lge_ph2n_global_ca/lge_mdss_dsi_ph2n_global_ca.o
mdss-dsi-objs += lge_ph2n_global_ca/lge_mdss_dsi_panel_ph2n_global_ca.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_L5_DCM_JP),y)
mdss-dsi-objs += lge_k7j/lge_mdss_dsi_k7j.o
mdss-dsi-objs += lge_k7j/lge_mdss_dsi_panel_k7j.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_JSG_KDDI_JP),y)
mdss-dsi-objs += lge_k7j/lge_mdss_dsi_k7j.o
mdss-dsi-objs += lge_k7j/lge_mdss_dsi_panel_k7j.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_LGPS29_1),y)
	mdss-dsi-objs += lge_lgps29_1/lge_mdss_dsi_lgps29_1.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_B6),y)
	mdss-dsi-objs += lge_b6/lge_mdss_dsi_b6.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_LGPS33),y)
	mdss-dsi-objs += lge_lgps33/lge_mdss_dsi_lgps33.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_SF340_TOVIS_TD4100),y)
	mdss-dsi-objs += lge_sf340_tovis_td4100/lge_mdss_dsi_sf340_tovis_td4100.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_SF3_LGD_TD4100),y)
	mdss-dsi-objs += lge_sf3_lgd_td4100/lge_mdss_dsi_sf3_lgd_td4100.o
endif
ifneq ($(filter y, &(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310) $(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105)), )
mdss-dsi-objs += lge_sf3f/lge_mdss_dsi_sf3f.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_SF3F_LGD_TD4310),y)
	mdss-dsi-objs += lge_sf3f/lge_mdss_dsi_sf3f_lgd_td4310.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_SF3F_LGD_SW49105),y)
	mdss-dsi-objs += lge_sf3f/lge_mdss_dsi_sf3f_lgd_sw49105.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_MFTS),y)
mdss-dsi-objs += lge/mfts_mode.o
endif
ifneq ($(filter y, &(CONFIG_LGE_DISPLAY_LV3_TIANMA_FT860X) $(CONFIG_LGE_DISPLAY_LV3_LGD_LG4894)), )
mdss-dsi-objs += lge_lv3/lge_mdss_dsi_lv3.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_LV3_TIANMA_FT860X),y)
mdss-dsi-objs += lge_lv3/lge_mdss_dsi_lv3_tianma_ft860x.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_LV3_LGD_LG4894),y)
mdss-dsi-objs += lge_lv3/lge_mdss_dsi_lv3_lgd_lg4894.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_LV7_TOVIS_TD4100),y)
mdss-dsi-objs += lge_lv7_tovis_td4100/lge_mdss_dsi_lv7_tovis_td4100.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_LV5_LGD_LG4894),y)
mdss-dsi-objs += lge_lv5/lge_mdss_dsi_lv5.o
mdss-dsi-objs += lge_lv5/lge_mdss_dsi_panel_lv5.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_LV9_JDI_NT35596),y)
mdss-dsi-objs += lge_lv9_jdi_nt35596/lge_mdss_dsi_lv9_jdi_nt35596.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_TF8_INX_NT51021),y)
mdss-dsi-objs += lge_tf8/lge_mdss_dsi_tf8.o
endif
ifeq ($(CONFIG_LGE_DISPLAY_MH),y)
mdss-dsi-objs += lge_mh/lge_mdss_dsi_mh.o
endif

mdss-dsi-objs += msm_mdss_io_8974.o
mdss-dsi-objs += mdss_dsi_phy.o
mdss-dsi-objs += mdss_dsi_clk.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss-dsi.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_panel.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_hdmi_util.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_hdmi_edid.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_cec_core.o
obj-$(CONFIG_FB_MSM_MDSS) += mdss_dba_utils.o
obj-$(CONFIG_FB_MSM_MDSS_EDP_PANEL) += mdss_edp.o
obj-$(CONFIG_FB_MSM_MDSS_EDP_PANEL) += mdss_edp_aux.o

obj-$(CONFIG_FB_MSM_MDSS) += mdss_io_util.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_tx.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_panel.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_hdcp.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_hdcp2p2.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_cec.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_PANEL) += mdss_hdmi_audio.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8334) += mhl_sii8334.o mhl_msc.o

obj-$(CONFIG_FB_MSM_MDSS_WRITEBACK) += mdss_wb.o

mdss-qpic-objs := mdss_qpic.o mdss_fb.o mdss_qpic_panel.o
obj-$(CONFIG_FB_MSM_QPIC) += mdss-qpic.o
obj-$(CONFIG_FB_MSM_QPIC_ILI_QVGA_PANEL) += qpic_panel_ili_qvga.o

obj-$(CONFIG_FB_MSM_MDSS) += mdss_fb.o mdss_util.o
ifeq ($(CONFIG_LGE_DISPLAY_BL_USE_BLMAP),y)
obj-$(CONFIG_FB_MSM_MDSS) += lge/lge_blmap_tuning.o
endif
obj-$(CONFIG_COMPAT) += mdss_compat_utils.o

ifeq ($(CONFIG_LGE_DISPLAY_DEBUG),y)
mdss-dsi-objs += lge/lge_mdss_debug.o
endif

