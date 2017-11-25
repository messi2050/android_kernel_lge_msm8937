#
# Makefile for the fingerprint drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -Idrivers/misc/

ifeq ($(CONFIG_INPUT_IX_BTP),y)
obj-$(CONFIG_INPUT_IX_BTP)	+= ix_btp_tee.o
endif

ifeq ($(CONFIG_FINGERPRINT_GOODIX_GF3208),y)
obj-$(CONFIG_FINGERPRINT_GOODIX_GF3208) += gf_spi.o platform.o netlink.o gf_common.o
endif

ifeq ($(CONFIG_INPUT_FPC1145),y)
obj-$(CONFIG_INPUT_FPC1145)     += fpc1145_tee.o
endif
