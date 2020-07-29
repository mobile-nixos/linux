# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt9xx_mtk/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt9xx_mtk/include/$(CONFIG_GTP_CONFIG)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
# add by qiangang@wind-mobi.com 20170728 begin
ccflags-y += -I$(srctree)/drivers/misc/mediatek/wind_device_info/
# add by qiangang@wind-mobi.com 20170728 end
$(info CONFIG_WIND_DEVICE_INFO $(CONFIG_WIND_DEVICE_INFO))
obj-y	+=  gt9xx_driver.o
obj-y	+=  gt9xx_extents.o
obj-y	+=  gt9xx_update.o
obj-y	+=  goodix_tool.o

