
obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_platform_api.o
obj-y	+=  lgtp_model_config.o
obj-y	+=  lgtp_touch_notify.o

ifeq ($(CONFIG_TOUCHSCREEN_S3320), y)
obj-y 	+= S3320/
endif

ifeq ($(CONFIG_MTK_TD4191),y)
obj-y	+=  td4191/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= LU201X/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= LU202X/
endif

ifeq ($(CONFIG_TOUCHSCREEN_FOCALTECH), y)
obj-y 	+= FT6X36/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= DUMMY/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LGE_MELFAS), y)
obj-y 	+= MIT200/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LR388K5), y)
obj-y 	+= LR388K5/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LR388K6), y)
obj-y 	+= LR388K6/
endif
