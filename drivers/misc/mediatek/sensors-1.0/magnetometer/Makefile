# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/magnetometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include
# add by qiangang@wind-mobi.com 20170728 begin
ccflags-y += -I$(srctree)/drivers/misc/mediatek/wind_device_info/
# add by qiangang@wind-mobi.com 20170728 end


obj-y   +=  mag.o
obj-y   +=  mag_factory.o

obj-$(CONFIG_MTK_MAGHUB)        +=  maghub/
obj-$(CONFIG_MTK_AKM09911)         +=  akm09911/
obj-$(CONFIG_MTK_AKM8963)	+=  akm8963/
obj-$(CONFIG_MTK_AFx133)	+=  afx133/
obj-$(CONFIG_MTK_MMC3630X)	+=  mmc3680x/
