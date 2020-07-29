#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#
# Makefile for misc devices that really don't fit anywhere else.
#

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ARCH_MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek


subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/inc
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/lc898122af
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/lc898212xdaf/inc
# zhenglihong@wind-mobi.com 20161013
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af
# zhenglihong@wind-mobi.com 20161013 

# zhenglihong@wind-mobi.com 20170208
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_GZ_E266L_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af_gz_e266l
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_OFG_E266L_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af_ofg_e266l
# zhenglihong@wind-mobi.com 20170208 

# zhaozhensen@wind-mobi.com 20161208 begin
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_GZ_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af_gz
# zhaozhensen@wind-mobi.com 20161208 end
# zhaozhensen@wind-mobi.com 20161208 begin
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_S5K3M2_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af_s5k3m2
# zhaozhensen@wind-mobi.com 20161208 end
# dingyisheng@wind-mobi.com 20170811 begin
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_S5K3P3SX_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af_s5k3p3sx
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_OV16880_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af_ov16880
# dingyisheng@wind-mobi.com 20161208 end
# yeshaobo@wind-mobi.com 20170811 begin
subdir-ccflags-$(CONFIG_MTK_LENS_GT9762AF_OV13855_E266L_SUPPORT) += -I$(MTK_CUSTOM_PATH)/lens/main/common/gt9762af_ov13855_e266l
# yeshaobo@wind-mobi.com 20170331 end
ifeq ($(CONFIG_MTK_LENS_BU63165AF_SUPPORT), y)
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/bu63165af/inc
else
  subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/main/common/bu63169af/inc
endif

obj-$(CONFIG_MTK_LENS)    += mainaf.o

mainaf-y  += main_lens.o
mainaf-y  += common/fm50af/FM50AF.o
mainaf-y  += common/bu6424af/BU6424AF.o
mainaf-y  += common/bu6429af/BU6429AF.o
mainaf-y  += common/dw9714af/DW9714AF.o
mainaf-y  += common/dw9814af/DW9814AF.o
mainaf-y  += common/dw9718af/DW9718AF.o
mainaf-y  += common/dw9718saf/DW9718SAF.o
mainaf-y  += common/dw9719taf/DW9719TAF.o
mainaf-y  += common/lc898212af/LC898212AF.o
mainaf-y  += common/lc898212xdaf/LC898212XDAF.o
mainaf-y  += common/lc898212xdaf/AfSTMV.o
mainaf-y  += common/lc898212xdaf/AfInter.o
mainaf-y  += common/lc898212xdaf/AfInit.o
mainaf-y  += common/lc898212xdaf_tvc700/LC898212XDAF_TVC700.o
mainaf-y  += common/lc898214af/LC898214AF.o
mainaf-y  += common/lc898217af/LC898217AF.o
mainaf-y  += common/lc898122af/LC898122AF.o
mainaf-y  += common/lc898122af/OisCmd.o
mainaf-y  += common/lc898122af/OisIni.o
mainaf-y  += common/ad5820af/AD5820AF.o
mainaf-y  += common/wv511aaf/WV511AAF.o
mainaf-y  += common/ak7371af/AK7371AF.o
ifeq ($(CONFIG_MTK_LENS_BU63165AF_SUPPORT), y)
mainaf-y  += common/bu63165af/BU63165AF.o
mainaf-y  += common/bu63165af/OIS_func.o
mainaf-y  += common/bu63165af/OIS_main.o
mainaf-y  += common/bu63165af/OIS_user.o
else
mainaf-y  += common/bu63169af/BU63169AF.o
mainaf-y  += common/bu63169af/OIS_func.o
mainaf-y  += common/bu63169af/OIS_main.o
mainaf-y  += common/bu63169af/OIS_user.o
endif
#zhenglihong@wind-mobi.com 20161012 begin
obj-$(CONFIG_MTK_LENS_GT9762AF_SUPPORT)    += common/gt9762af/
#zhenglihong@wind-mobi.com 20161012 end

#zhenglihong@wind-mobi.com 20170208 begin
obj-$(CONFIG_MTK_LENS_GT9762AF_GZ_E266L_SUPPORT)    += common/gt9762af_gz_e266l/
obj-$(CONFIG_MTK_LENS_GT9762AF_OFG_E266L_SUPPORT)    += common/gt9762af_ofg_e266l/
#zhenglihong@wind-mobi.com 20170208 end

#yeshaobo@wind-mobi.com 20170331 begin
obj-$(CONFIG_MTK_LENS_GT9762AF_OV13855_E266L_SUPPORT)    += common/gt9762af_ov13855_e266l/
#yeshaobo@wind-mobi.com 20170331 end

#dingyisheng@wind-mobi.com 20170331 begin
obj-$(CONFIG_MTK_LENS_GT9762AF_S5K3P3SX_SUPPORT)    += common/gt9762af_s5k3p3sx/
obj-$(CONFIG_MTK_LENS_GT9762AF_OV16880_SUPPORT)    += common/gt9762af_ov16880/
#dingyisheng@wind-mobi.com 20170331 end

#zhaozhensen@wind-mobi.com 20161208 begin
obj-$(CONFIG_MTK_LENS_GT9762AF_GZ_SUPPORT)    += common/gt9762af_gz/
obj-$(CONFIG_MTK_LENS_GT9762AF_S5K3M2_SUPPORT)    += common/gt9762af_s5k3m2/
#zhaozhensen@wind-mobi.com 20161208 end
