#
# Makefile for the QCOM specific video device drivers
# based on V4L2.
#
obj-$(CONFIG_MSM_VIDC_V4L2) += vidc/
obj-y += broadcast/
obj-$(CONFIG_DVB_MPQ) += dvb/
ifeq ($(CONFIG_LGE_DIR_CAMERA),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_lge/
else
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
<<<<<<< HEAD
endif
obj-y += sde/
=======
obj-$(CONFIG_MSM_AIS) += ais/
obj-y += sde/
>>>>>>> LA.UM.6.6.r1-02700-89xx.0
