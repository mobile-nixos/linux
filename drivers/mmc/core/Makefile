#
# Makefile for the kernel mmc core.
#

obj-$(CONFIG_MMC)		+= mmc_core.o
mmc_core-y			:= core.o bus.o host.o \
				   mmc.o mmc_ops.o sd.o sd_ops.o \
				   sdio.o sdio_ops.o sdio_bus.o \
				   sdio_cis.o sdio_io.o sdio_irq.o \
				   quirks.o slot-gpio.o

mmc_core-$(CONFIG_DEBUG_FS)	+= debugfs.o
# wangjun@wimd-mobi.com 20170818 begin
ifeq ($(TARGET_BUILD_VARIANT),user)
KBUILD_CFLAGS += -DMMC_USER=1
endif
# wangjun@wimd-mobi.com 20170818 end