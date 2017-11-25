#
# Makefile for the input sensor drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -DCONFIG_BMA_USE_PLATFORM_DATA
ccflags-y += -DBMM_USE_BASIC_I2C_FUNC
ccflags-y += -DCONFIG_LGE_SENSOR

obj-$(CONFIG_SENSOR_ATMF04) += atmf04.o
obj-$(CONFIG_SENSOR_ATMF04_2ND) += atmf04_2nd.o
