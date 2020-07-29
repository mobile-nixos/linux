//liupeng@wind-mobi.com 20160530 begin
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_mtk.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>

#include "tpd.h"

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif //TIMER_DEBUG

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif //CONFIG_MTK_SENSOR_HUB_SUPPORT

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <linux/regulator/consumer.h>
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

#include "mstar_drv_platform_interface.h"

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" or "msg28xx" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */
#define I2C_BUS_ID   (1)       // i2c bus id : 0 or 1

#define TPD_OK (0)

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
extern int g_TpVirtualKey[];

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
extern int g_TpVirtualKeyDimLocal[][4];
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

extern struct tpd_device *tpd;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

struct i2c_client *g_I2cClient = NULL;


#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
struct regulator *g_ReguVdd = NULL;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

/*=============================================================*/
// FUNCTION DECLARATION
/*=============================================================*/

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

/* probe function is used for matching and initializing input device */
#ifdef CONFIG_ENABLE_CHARGER_DETECTION//adair
static struct hrtimer chargerTimer;
static u8 bCancelTimer = 0;
static struct workqueue_struct *chargerWq;
static struct work_struct chargerWork;
extern u8 g_ForceUpdate;
extern void DrvFwCtrlChargerDetection(u8 nChargerStatus);

 static void charger_work_func(struct work_struct *work)
{
	u8 szChargerStatus[20] = {0};
 
        DrvCommonReadFile("/sys/class/power_supply/battery/status", szChargerStatus, 20);
        
        DBG(&g_I2cClient->dev, "*** Battery Status : %s ***\n", szChargerStatus);
        
        g_ForceUpdate = 0; // Set flag to force update charger status

        if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL || strstr(szChargerStatus, "Fully charged") != NULL) // Charging
        {
            DrvFwCtrlChargerDetection(1); // charger plug-in
        }
        else // Not charging
        {
            DrvFwCtrlChargerDetection(0); // charger plug-out
        }

        g_ForceUpdate = 0; // Clear flag after force update charger status
		hrtimer_start(&chargerTimer, ktime_set(0, 125000000), HRTIMER_MODE_REL);
}

static enum hrtimer_restart charger_timer_func(struct hrtimer *timer)
{   
       queue_work(chargerWq, &chargerWork);
	   return HRTIMER_NORESTART;
}
#endif
static int /*__devinit*/ tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    const char *vdd_name = "vtouch";
//    const char *vcc_i2c_name = "vcc_i2c";
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

    printk("[Mstar] probe\n");   
    
    if (client == NULL)
    {
        printk("i2c client is NULL\n");
        return -1;
    }

    g_I2cClient = client;
	
	if(g_I2cClient->addr != 0x26)
	{
		g_I2cClient->addr = 0x26;
		printk("[Mstar]:i2c_client_Mstar->addr=0x%x after --- \n",g_I2cClient->addr);
	}
    
    MsDrvInterfaceTouchDeviceSetIicDataRate(g_I2cClient, 100000); // 100 KHz

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
   g_ReguVdd = regulator_get(tpd->tpd_dev, vdd_name);
  /*   tpd->reg = g_ReguVdd;

    ret = regulator_set_voltage(g_ReguVdd, 2800000, 2800000); 
    if (ret)
    {
        printk("[Mstar]Could not set to 2800mv.\n");
    }else{
		printk("[Mstar]set to 2800mv success.\n");
	}
*/
	ret = regulator_enable(tpd->reg);
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

    ret = MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
    if (ret == 0) // If probe is success, then enable the below flag.
    {
        tpd_load_status = 1;
		#ifdef CONFIG_ENABLE_CHARGER_DETECTION//adair
			chargerWq = create_singlethread_workqueue("charger detection");
			INIT_WORK(&chargerWork, charger_work_func);
	
			hrtimer_init(&chargerTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			chargerTimer.function = charger_timer_func;
			hrtimer_start(&chargerTimer, ktime_set(1, 0), HRTIMER_MODE_REL);
		#endif
    }    


    printk("[Mstar] probe done\n");
    
    return TPD_OK;   
}

static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, TPD_DEVICE);    
    
    return TPD_OK;
}

static int /*__devexit*/ tpd_remove(struct i2c_client *client)
{   
    TPD_DEBUG("TPD removed\n");
    
    MsDrvInterfaceTouchDeviceRemove(client);
    
    return TPD_OK;
}


/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id tpd_device_id[] =
{
    {MSG_TP_IC_NAME, 0},
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, tpd_device_id);

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
const struct of_device_id touch_dt_match_table[] = {
    { .compatible = "mediatek,cap_touch",},
    {},
};

MODULE_DEVICE_TABLE(of, touch_dt_match_table);

static struct device_attribute *msg2xxx_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif //CONFIG_MTK_SENSOR_HUB_SUPPORT
};

#else
static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(MSG_TP_IC_NAME, (0x4C>>1))};
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

static struct i2c_driver tpd_i2c_driver = {

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    .driver = {
        .name = MSG_TP_IC_NAME,
        .of_match_table = of_match_ptr(touch_dt_match_table),
    },
#else
    .driver = {
        .name = MSG_TP_IC_NAME,
    },
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = tpd_device_id,
    .detect = tpd_detect,
};

extern void pmic_ldo_vldo28_sw_en(int en_value);  //lihiayan

static int tpd_local_init(void)
{  

	int ret;
    printk("[Mstar] init device driver\n");

	printk("[Mstar] Start to power on\n");
		//=======regulator power on ==========
	//tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	//tpd->reg = regulator_get(tpd->tpd_dev, "vgp1");
/*
    retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    if (retval != 0) {
    	TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
    	return -1;
    }
*/
	tpd->tpd_dev->of_node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-touch");
	if (tpd->tpd_dev->of_node)
	{
		printk("[Mstar]of_find_compatible_node-->mediatek,mt6755-touch\n");

		tpd->reg = regulator_get(tpd->tpd_dev, "vtouch"); //modified by hebiao 2016907
		ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	//modified by ranyanhao 20161021
		if (ret) {
			printk("[Mstar]regulator_set_voltage(%d) failed!\n", ret);
			return -1;
		}
		ret = regulator_enable(tpd->reg);
		if (ret) {
			printk("[Mstar] regulator_enable ERROR \n");
		}
	}	
	
	//pmic_ldo_vldo28_sw_en(1);
	
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        printk("[Mstar]Unable to add i2c driver.\n");
         
        return -1;
    }
    
    if (tpd_load_status == 0) 
    {
        printk("Add error touch panel driver.\n");

        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    if (tpd_dts_data.use_tpd_button)
    {
        tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
        tpd_dts_data.tpd_key_dim_local);
    }
#else
#ifdef CONFIG_TP_HAVE_KEY
#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE     
    tpd_button_setting(4, g_TpVirtualKey, g_TpVirtualKeyDimLocal); //MAX_KEY_NUM
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE  
#endif //CONFIG_TP_HAVE_KEY  
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

    printk("[Mstar] init done %s, %d\n", __func__, __LINE__);  
        
    return TPD_OK; 
}



//lihaiyan@wind-mobi.com 20160209 begin
/*
 * This function will be performed only once
 */
#if defined(CONFIG_MTK_LENS_GT9762AF_GZ_E266L_SUPPORT) || defined(CONFIG_MTK_LENS_GT9762AF_OFG_E266L_SUPPORT)
extern struct i2c_client *GT9762AF_I2Cclient_common_E266;   //lihaiyan@wind-mobi.com 20160209 add

static void AF_Into_Power_Down_Mode(void)                   // wangbing@wind-mobi.com 20170331 modify
{
	static char AF_flag = 0;
	int ret = 0;
	unsigned short AF_addr_temp = 0;
	char AF_Cmd[2]={(char)(0x80),(char)(0x00)};//GF9762af power down mode
	
	if((AF_flag == 0) && (GT9762AF_I2Cclient_common_E266 != NULL)) {
		AF_addr_temp = GT9762AF_I2Cclient_common_E266->addr;
		GT9762AF_I2Cclient_common_E266->addr = 0x18 >> 1;
		ret = i2c_master_send(GT9762AF_I2Cclient_common_E266, AF_Cmd, 2); 
    	if (ret < 0) {
        	printk("ranyanhao %s,GT9762AF into power down mode fail\n", __func__);
			AF_flag = 0;
		} else {
			printk("ranyanhao %s,GT9762AF into power down mode success\n", __func__);
			AF_flag = 1;
		}
		GT9762AF_I2Cclient_common_E266->addr = AF_addr_temp;
	}
}
#endif
//lihaiyan@wind-mobi.com 20160209 end	



#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
static void tpd_resume(struct device *h)
#else
static void tpd_resume(struct early_suspend *h)
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
{
    printk("[Mstar] wake up\n");
    
    MsDrvInterfaceTouchDeviceResume(h);
#ifdef CONFIG_ENABLE_CHARGER_DETECTION//adair
     if(bCancelTimer)
    {
         bCancelTimer = 0;
	  hrtimer_start(&chargerTimer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }
#endif
    printk("[Mstar] wake up done\n");
}

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
static void tpd_suspend(struct device *h)
#else
static void tpd_suspend(struct early_suspend *h)
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
{
    printk("[Mstar] enter sleep\n");
#ifdef CONFIG_ENABLE_CHARGER_DETECTION//adair
       cancel_work_sync(&chargerWork);
	hrtimer_cancel(&chargerTimer);
       bCancelTimer = 1;
#endif
    MsDrvInterfaceTouchDeviceSuspend(h);
	
#if defined(CONFIG_MTK_LENS_GT9762AF_GZ_E266L_SUPPORT) || defined(CONFIG_MTK_LENS_GT9762AF_OFG_E266L_SUPPORT)
	AF_Into_Power_Down_Mode();   //lihaiyan 20170209 begin
#endif
	
    printk("[Mstar] enter sleep done\n");
} 

static struct tpd_driver_t tpd_device_driver = {
    .tpd_device_name = MSG_TP_IC_NAME,
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    .attrs = {
        .attr = msg2xxx_attrs,
        .num  = ARRAY_SIZE(msg2xxx_attrs),
    },
#else
#ifdef CONFIG_TP_HAVE_KEY
#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
     .tpd_have_button = 1,
#else
     .tpd_have_button = 0,
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE        
#endif //CONFIG_TP_HAVE_KEY        
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
};

static int __init tpd_driver_init(void) 
{
    printk("MStar touch panel driver init\n");
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    tpd_get_dts_info();
#else
    i2c_register_board_info(I2C_BUS_ID, &i2c_tpd, 1);
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    if (tpd_driver_add(&tpd_device_driver) < 0)
    {
        printk("[Mstar] add MStar TP driver failed\n");
    }
     
    return 0;
}
 
static void __exit tpd_driver_exit(void) 
{
    printk("MStar touch panel driver exit\n");
    
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
MODULE_LICENSE("GPL");
//liupeng@wind-mobi.com 20160530 end