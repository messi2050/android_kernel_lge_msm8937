

obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_model_config_i2c.o
obj-y	+=  lgtp_model_config_misc.o
obj-y	+=  lgtp_platform_api_i2c.o
obj-y	+=  lgtp_platform_api_misc.o
obj-y	+=  lgtp_platform_api_power.o

ifeq ($(CONFIG_JDI_INCELL), y)
obj-y 	+= S3320/
endif

ifeq ($(CONFIG_MTK_TD4191),y)
obj-y	+=  td4191/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= LU202X/
endif


ifeq ($(CONFIG_MACH_MSM8939_M239DS_GLOBAL_COM), y)
obj-y 	+= MIT300/
endif

ifeq ($(CONFIG_MSM8916_P1C), y)
obj-y   += MIT300/
endif

ifeq ($(CONFIG_TOUCHSCREEN_UNIFIED_MELFAS_MIT300_PH1), y)
obj-y   += MIT300_PH1/
endif

ifeq ($(CONFIG_TOUCHSCREEN_UNIFIED_MELFAS_MIT300_PH2), y)
obj-y   += MIT300_PH2/
endif
