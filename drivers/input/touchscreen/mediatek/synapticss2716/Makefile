# hebiao@wind-mobi.com 2016826 begin 
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/synapticss2716/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics_v61/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/wdt/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/wdt/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/i2c/$(MTK_PLATFORM)/


obj-y +=  synaptics_dsx_rmi_dev.o
#obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_TEST_REPORTING) +=  synaptics_dsx_test_reporting.o
obj-y +=  synaptics_dsx_i2c.o
obj-y +=  synaptics_dsx_fw_update.o

# hebiao@wind-mobi.com 2016826 end