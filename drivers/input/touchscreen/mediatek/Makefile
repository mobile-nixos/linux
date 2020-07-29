subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
# In case the platform does NOT support this type of sensors

ifneq ($(CONFIG_TOUCHSCREEN_MTK),)
obj-y   +=  mtk_tpd.o tpd_button.o tpd_calibrate.o tpd_debug.o \
		tpd_default.o tpd_init.o tpd_misc.o tpd_setting.o   tp_gesture.o 
endif

obj-$(CONFIG_TOUCHSCREEN_MTK)   += tpd_common_probe.o
obj-$(CONFIG_TOUCHSCREEN_MTK_GT910)	+=  GT910/
#ranyanhao@wind-mobi.com 20161010 begin
#obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
#ranyanhao@wind-mobi.com 20161010 end
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151TB)	+=  GT1151TB/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XXTB_HN_NEW)	+=  gt9xxtb_hn_new/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT)	+=  GT9XX_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_PHONE)	+=  GT9XX_hotknot_phone/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_I2C_RMI4)	+=  synaptics_i2c_rmi4/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT928)	+=  GT928/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_SCP)	+=  GT9XX_hotknot_scp/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5X0X)	+=  ft5x0x/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_3320_50) +=  synaptics_3320_50/
obj-$(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER_4)	+=  unified_driver_4/
#ranyanhao@wind-mobi.com 20160928 begin
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICSS2716)	+=  synapticss2716/
#ranyanhao@wind-mobi.com 20160928 end
obj-$(CONFIG_TOUCHSCREEN_MTK_MSG2238A)  +=  msg2238a/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT6336S)   +=  ft6336s/

###################################################################
# must be first: MSG5846
#lihaiyan@wind-mobi.com 20170114 begin
# TP i2c addr 0x26
obj-$(CONFIG_TOUCHSCREEN_MTK_MSG5846)	+=  MSG5846/
#lihaiyan@wind-mobi.com 20170114 end

#hebiao@wind-mobi.com 20161108 begin
obj-$(CONFIG_TOUCHSCREEN_FT3427)	+= ft3427/
## onsell TP  i2C addr 0x38
obj-$(CONFIG_TOUCHSCREEN_FT3427_E266L) += ft3427_E266L_new/

## insell TP  i2C addr 0x3A
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_TS) += focaltech_touch_ft3427/

#wangjun@wind-mobi.com 20170601 begind 
# insell TP i2c addr 0x48
obj-$(CONFIG_TOUCHSCREEN_HX83102_HLT_E266L) += HX83102/
#wangjun@wind-mobi.com 20170601 end 




#liukangping@wind-mobi.com 20170601 begind 
ifeq ($(CONFIG_TOUCHSCREEN_MTK_MSG2836),y)
obj-y	+=  msg2836/
endif
#liukangping@wind-mobi.com 20170601 end 


#qiangang@wind-mobi.com 20170601 begind 
ifeq ($(CONFIG_TOUCHSCREEN_MTK_GT917),y)
obj-y	+=  gt917/
endif
#qiangang@wind-mobi.com 20170601 end 


ifeq ($(CONFIG_TOUCHSCREEN_HIMAX_HX852xES),y)
obj-y	+=  hx852xes/
endif
#hebiao@wind-mobi.com 20161108 end
