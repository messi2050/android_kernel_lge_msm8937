#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3330)		+= touch_s3330.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3330)		+= touch_s3330_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3330)		+= touch_s3330_rmi_dev.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3330)		+= touch_s3330_prd.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_S3330)		+= touch_s3330_f54_test.o

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100)		+= touch_td4100.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100)		+= touch_td4100_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100)		+= touch_td4100_rmi_dev.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100)		+= touch_td4100_prd.o
# obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4100)		+= touch_td4100_f54_test.o

obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4310)		+= touch_td4310.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4310)		+= touch_td4310_fw_upgrade.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4310)		+= touch_td4310_rmi_dev.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4310)		+= touch_td4310_prd.o
#obj-$(CONFIG_LGE_TOUCH_SYNAPTICS_TD4310)		+= touch_td4310_f54_test.o # [bringup] need to check. file not exist.
