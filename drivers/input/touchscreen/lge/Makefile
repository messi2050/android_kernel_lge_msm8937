#
# Makefile for lge touch core driver
#

# Each configuration option enables a list of files.

ifdef CONFIG_LGE_TOUCH_CORE_MTK
	subdir-ccflags-y += -Werror
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
endif

ifdef CONFIG_LGE_TOUCH_CORE_MTK_LEGACY
	include $(srctree)/drivers/misc/mediatek/Makefile.custom
	subdir-ccflags-y += -DUSE_I2C_MTK_EXT
endif

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_core.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_core_sysfs.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_common.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_i2c.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_spi.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_model.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_notify.o

obj-$(CONFIG_LGE_TOUCH_CORE)		+= touch_hwif.o
obj-$(CONFIG_LGE_TOUCH_CORE_MTK)	+= touch_hwif_mtk.o
obj-$(CONFIG_LGE_TOUCH_CORE_MTK_LEGACY)	+= touch_hwif_mtk_legacy.o

obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= lgsic/
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= synaptics/
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= focaltech/
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= novatek/
