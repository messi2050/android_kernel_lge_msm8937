#
# Makefile for lge touch vendor driver
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_NOVATEK_NT11206)		+= touch_nt11206.o
obj-$(CONFIG_LGE_TOUCH_NOVATEK_NT11206)		+= touch_nt11206_sysfs.o
obj-$(CONFIG_LGE_TOUCH_NOVATEK_NT11206)		+= touch_nt11206_sd_new.o

