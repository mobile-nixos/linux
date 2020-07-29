# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/tl5601c
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/stk3x1x
# add by qiangang@wind-mobi.com 20170728 begin
ccflags-y += -I$(srctree)/drivers/misc/mediatek/wind_device_info/
# add by qiangang@wind-mobi.com 20170728 end


obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-y += PSensor_Calibrator.o LSensor_Calibrator.o

obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
obj-$(CONFIG_MTK_CM36558)	+=  cm36558/
obj-$(CONFIG_MTK_CM36652)	+=  cm36652/
obj-$(CONFIG_MTK_CM36686)	+=  cm36686/
obj-$(CONFIG_MTK_EPL2182)	+=  epl2182/
obj-$(CONFIG_MTK_APDS9930)	+=  APDS9930/
obj-$(CONFIG_MTK_LTR559)	+=  ltr559/
obj-$(CONFIG_MTK_STK3X1X_OT)	+=  stk3x1x/
obj-$(CONFIG_MTK_PA22A)  += pa22a/
