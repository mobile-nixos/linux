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
 * @file    ilitek_drv_mtk.c
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
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>

#include "ilitek_drv_common.h"
#include "tpd.h"
/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_CHIP_TYPE_ILI21XX
#define TP_IC_NAME "ili212x" //"ili2120" or "ili2121" /* Please define the ilitek touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */
#else //CONFIG_ENABLE_CHIP_TYPE_MSG22XX || CONFIG_ENABLE_CHIP_TYPE_MSG28XX
#define TP_IC_NAME "msg2xxx" //"msg22xx" or "msg28xx" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */
#endif //CONFIG_ENABLE_CHIP_TYPE_ILI21XX

#define I2C_BUS_ID   (1)       // i2c bus id : 0 or 1

#define TPD_OK (0)
extern void MsDrvInterfaceTouchDeviceResume(struct device *pDevice);
extern void MsDrvInterfaceTouchDeviceSuspend(struct device *pDevice);
/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

int msg_sleep_flag = 0;//add by qiangang
#ifdef CONFIG_TP_HAVE_KEY
extern int g_TpVirtualKey[];

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
extern int g_TpVirtualKeyDimLocal[][4];
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

extern struct tpd_device *tpd;
//add by qiangang 2010823 begin 
extern int tp_gesture_proc_init(void);
extern void tp_gesture_proc_remove(void);
//add by qiangang 2010823 end
/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

struct i2c_client *g_I2cClient = NULL;


#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
struct regulator *g_ReguVdd = NULL;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD


// wangjun@wind-mobi.com 20170922 begin
#ifdef CONFIG_WIND_DEF_PRO_E262L
int TOUCH_SCREEN_X_MAX = 720;
int TOUCH_SCREEN_Y_MAX = 1440;
//wangjun@wind-mobi.com 20170919 begin
static int __init set_tp_size(char *str)
{
	
	if (!strcmp(str,"1080x2160"))
    {
        TOUCH_SCREEN_X_MAX = 1080;
        TOUCH_SCREEN_Y_MAX = 2160;
    }   		
    else
    {
        TOUCH_SCREEN_X_MAX = 720;
        TOUCH_SCREEN_Y_MAX = 1440;
    }
    pr_err("[wjwind] set_tp_size() TOUCH_SCREEN_X_MAX = %d TOUCH_SCREEN_Y_MAX = %d\n",TOUCH_SCREEN_X_MAX,TOUCH_SCREEN_Y_MAX);
	return 0;
}
__setup("androidboot.lcmsize=", set_tp_size);
#endif
// wangjun@wind-mobi.com 20170922 end



/*=============================================================*/
// FUNCTION DECLARATION
/*=============================================================*/

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/
//add by qiangang@wind-mobi.com 20170831 begin
extern struct i2c_client *g_pstAF_I2Cclient;   //lihaiyan@wind-mobi.com 20160209 add

static void AF_Into_Power_Down_Mode(void)                   // wangbing@wind-mobi.com 20170331 modify
{
	static char AF_flag = 0;
	int ret = 0;
	unsigned short AF_addr_temp = 0;
	char AF_Cmd[2]={(char)(0x80),(char)(0x00)};//GF9762af power down mode

	
	
	if((AF_flag == 0) && (g_pstAF_I2Cclient != NULL)) {
		AF_addr_temp = g_pstAF_I2Cclient->addr;
		g_pstAF_I2Cclient->addr = 0x18 >> 1;

		ret = i2c_master_send(g_pstAF_I2Cclient, AF_Cmd, 2); 
    	if (ret < 0) {
        	printk("dingyisheng %s,GT9762AF into power down mode fail\n", __func__);
			AF_flag = 0;
		} else {
			printk("dingyisheng %s,GT9762AF into power down mode success\n", __func__);
			AF_flag = 0;
		}
		g_pstAF_I2Cclient->addr = AF_addr_temp;
	}
}
//add by qiangang@wind-mobi.com 20170831 end

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    const char *vdd_name = "vtouch";
//    const char *vcc_i2c_name = "vcc_i2c";
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

    TPD_DMESG("TPD probe\n");   
    
    if (client == NULL)
    {
        TPD_DMESG("i2c client is NULL\n");
        return -1;
    }
    g_I2cClient = client;
    
    MsDrvInterfaceTouchDeviceSetIicDataRate(g_I2cClient, 100000); // 100 KHz

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    g_ReguVdd = regulator_get(tpd->tpd_dev, vdd_name);
    tpd->reg = g_ReguVdd;

    ret = regulator_set_voltage(g_ReguVdd, 2800000, 2800000); 
    if (ret)
    {
        TPD_DMESG("Could not set to 2800mv.\n");
    }
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

    ret = MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
    if (ret == 0) // If probe is success, then enable the below flag.
    {
        tpd_load_status = 1;
    }    

    TPD_DMESG("TPD probe done\n");
//add by qiangang 20170823 begin	
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	//msg_touch_proc_init();
	msg_sleep_flag = 0;
	tp_gesture_proc_init();
	
#endif 
//add by qiangang 20170823 end
    
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
	//add by qiangang 2010823 begin 	
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    	tp_gesture_proc_remove();
#endif 
//add by qiangang 2010823 end
    return TPD_OK;
}


/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id tpd_device_id[] =
{
    {TP_IC_NAME, 0},
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

#ifdef CONFIG_ENABLE_CHIP_TYPE_ILI21XX
static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(TP_IC_NAME, (0x82>>1))};
#else //CONFIG_ENABLE_CHIP_TYPE_MSG22XX || CONFIG_ENABLE_CHIP_TYPE_MSG28XX
static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(TP_IC_NAME, (0x4C>>1))};
#endif //CONFIG_ENABLE_CHIP_TYPE_ILI21XX

#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD

static struct i2c_driver tpd_i2c_driver = {

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    .driver = {
        .name = TP_IC_NAME,
        .of_match_table = of_match_ptr(touch_dt_match_table),
    },
#else
    .driver = {
        .name = TP_IC_NAME,
    },
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = tpd_device_id,
    .detect = tpd_detect,
};

static int tpd_local_init(void)
{  
    TPD_DMESG("TPD init device driver\n");

    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        TPD_DMESG("Unable to add i2c driver.\n");
         
        return -1;
    }
    
    if (tpd_load_status == 0) 
    {
        TPD_DMESG("Add error touch panel driver.\n");

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

    TPD_DMESG("TPD init done %s, %d\n", __func__, __LINE__);  
        
    return TPD_OK; 
}

//#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
static void tpd_resume(struct device *h)
//#else
//static void tpd_resume(struct early_suspend *h)
//#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
{
    TPD_DMESG("TPD wake up\n");
    
    MsDrvInterfaceTouchDeviceResume(h);
    
    TPD_DMESG("TPD wake up done\n");
}

//#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
static void tpd_suspend(struct device *h)
//#else
//static void tpd_suspend(struct early_suspend *h)
//#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
{
    TPD_DMESG("TPD enter sleep\n");

	AF_Into_Power_Down_Mode();//add by qiangang
    MsDrvInterfaceTouchDeviceSuspend(h);

    TPD_DMESG("TPD enter sleep done\n");
} 

static struct tpd_driver_t tpd_device_driver = {
    .tpd_device_name = TP_IC_NAME,
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
    TPD_DMESG("ILITEK/MStar touch panel driver init\n");

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    tpd_get_dts_info();
#else
    i2c_register_board_info(I2C_BUS_ID, &i2c_tpd, 1);
#endif //CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
    if (tpd_driver_add(&tpd_device_driver) < 0)
    {
        TPD_DMESG("TPD add ILITEK/MStar TP driver failed\n");
    }
     
    return 0;
}
 
static void __exit tpd_driver_exit(void) 
{
    TPD_DMESG("ILITEK/MStar touch panel driver exit\n");
    
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
MODULE_LICENSE("GPL");