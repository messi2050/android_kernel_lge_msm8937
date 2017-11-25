#
## Makefile for the touchscreen drivers.
#
#
## Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4945)         += touch_lg4945.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4945)         += touch_lg4945_watch.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4945)         += touch_lg4945_prd.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4945)         += touch_lg4945_abt.o

obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4946)		+= touch_lg4946.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4946)		+= touch_lg4946_watch.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4946)		+= touch_lg4946_prd.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4946)		+= touch_lg4946_abt.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_LG4946)		+= touch_lg4946_asc.o

obj-$(CONFIG_LGE_TOUCH_LG4894)		+= touch_lg4894.o
obj-$(CONFIG_LGE_TOUCH_LG4894)		+= touch_lg4894_prd.o
obj-$(CONFIG_LGE_TOUCH_LG4894)		+= touch_lg4894_abt.o

obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49106)		+= touch_sw49106.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49106)		+= touch_sw49106_prd.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49106)		+= touch_sw49106_abt.o
