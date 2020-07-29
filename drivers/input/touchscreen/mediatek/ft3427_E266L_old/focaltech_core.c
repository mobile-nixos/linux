//hebiao@wind-mobi.com add at 20161112 begin
/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>

#include "include/tpd_ft5x0x_common.h"
/*shenyong@wind-mobi.com 20160721 start ***/
#include <linux/fs.h>
/*shenyong@wind-mobi.com 20160721 end ***/
#include "focaltech_core.h"
/* #include "ft5x06_ex_fun.h" */

#include "tpd.h"
#include "base.h"
/* #define TIMER_DEBUG */


#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif
#ifdef CONFIG_WIND_DEVICE_INFO
#include "wind_device_info.h"
#endif
//hebiao@wind-mobi.com 20161105 end

//lihaiyan@wind-mobi.com 20170412 begin
#if FTS_GESTRUE_EN   
#include "../tp_gesture.h"
extern unsigned int g_GestureWakeupMode_wind;

static char gesture_mode_tmp[6] = {0}; 

extern int tp_gesture_proc_init(void);
extern void tp_gesture_proc_remove(void);
#endif
//lihaiyan@wind-mobi.com 20170412 end


//tuwenzan@wind-mobi.com modify at 20160608 begin
#if CTP_ESD_PROTECT
//define and implement in focaltech_esd_protection.c
extern int  fts_esd_protection_init(void);
extern int  fts_esd_protection_exit(void);
extern int  fts_esd_protection_notice(void);
extern int  fts_esd_protection_suspend(void);
extern int  fts_esd_protection_resume(void);

int apk_debug_flag = 0;
//int  power_switch_gesture = 0;
//#define TPD_ESD_CHECK_CIRCLE        		200
//static struct delayed_work ctp_esd_check_work;
//static struct workqueue_struct *ctp_esd_check_workqueue = NULL;
void ctp_esd_check_func(void);
static int count_irq = 0;
#endif
//tuwenzan@wind-mobi.com modify at 20160608 end






#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
enum DOZE_T {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static DOZE_T doze_status = DOZE_DISABLED;
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client);

enum TOUCH_IPI_CMD_T {
	/* SCP->AP */
	IPI_COMMAND_SA_GESTURE_TYPE,
	/* AP->SCP */
	IPI_COMMAND_AS_CUST_PARAMETER,
	IPI_COMMAND_AS_ENTER_DOZEMODE,
	IPI_COMMAND_AS_ENABLE_GESTURE,
	IPI_COMMAND_AS_GESTURE_SWITCH,
};

struct Touch_Cust_Setting {
	u32 i2c_num;
	u32 int_num;
	u32 io_int;
	u32 io_rst;
};

struct Touch_IPI_Packet {
	u32 cmd;
	union {
		u32 data;
		Touch_Cust_Setting tcs;
	} param;
};

/* static bool tpd_scp_doze_en = FALSE; */
static bool tpd_scp_doze_en = TRUE;
DEFINE_MUTEX(i2c_access);
#endif

#define TPD_SUPPORT_POINTS	5


struct i2c_client *i2c_client = NULL;
struct task_struct *thread_tpd_onsell = NULL;
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client_onsell 				= NULL;
struct input_dev *fts_input_dev_onsell				=NULL;
#ifdef TPD_AUTO_UPGRADE
static bool is_update = false;
#endif
#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;
#endif


static struct kobject *touchscreen_dir=NULL;
static struct kobject *virtual_dir=NULL;
static struct kobject *touchscreen_dev_dir=NULL;
static char *vendor_name=NULL;
static u8 ctp_fw_version;
//static int tpd_keys[TPD_VIRTUAL_KEY_MAX] = { 0 };
//static int tpd_keys_dim[TPD_VIRTUAL_KEY_MAX][4]={0};

#define WRITE_BUF_SIZE  1016
#define PROC_UPGRADE							0
#define PROC_READ_REGISTER						1
#define PROC_WRITE_REGISTER					    2
#define PROC_AUTOCLB							4
#define PROC_UPGRADE_INFO						5
#define PROC_WRITE_DATA						    6
#define PROC_READ_DATA							7
#define PROC_SET_TEST_FLAG						8
static unsigned char proc_operate_mode 			= PROC_UPGRADE;



static DECLARE_WAIT_QUEUE_HEAD(waiter);

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);


static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static int tpd_flag;
/*static int point_num = 0;
static int p_point_num = 0;*/

static unsigned int tpd_rst_gpio_number = 0;
static unsigned int tpd_int_gpio_number = 1;
static unsigned int touch_irq = 0;
#define TPD_OK 0


/* Register define */
#define DEVICE_MODE	0x00
#define GEST_ID		0x01
#define TD_STATUS	0x02

#define TOUCH1_XH	0x03
#define TOUCH1_XL	0x04
#define TOUCH1_YH	0x05
#define TOUCH1_YL	0x06

#define TOUCH2_XH	0x09
#define TOUCH2_XL	0x0A
#define TOUCH2_YH	0x0B
#define TOUCH2_YL	0x0C

#define TOUCH3_XH	0x0F
#define TOUCH3_XL	0x10
#define TOUCH3_YH	0x11
#define TOUCH3_YL	0x12

#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT	3

#ifdef TIMER_DEBUG

static struct timer_list test_timer;

static void timer_func(unsigned long data)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	mod_timer(&test_timer, jiffies + 100*(1000/HZ));
}

static int init_test_timer(void)
{
	memset((void *)&test_timer, 0, sizeof(test_timer));
	test_timer.expires  = jiffies + 100*(1000/HZ);
	test_timer.function = timer_func;
	test_timer.data     = 0;
	init_timer(&test_timer);
	add_timer(&test_timer);
	return 0;
}
#endif


#if defined(CONFIG_TPD_ROTATE_90) || defined(CONFIG_TPD_ROTATE_270) || defined(CONFIG_TPD_ROTATE_180)
/*
static void tpd_swap_xy(int *x, int *y)
{
	int temp = 0;

	temp = *x;
	*x = *y;
	*y = temp;
}
*/
/*
static void tpd_rotate_90(int *x, int *y)
{
//	int temp;

	*x = TPD_RES_X + 1 - *x;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
static void tpd_rotate_180(int *x, int *y)
{
	*y = TPD_RES_Y + 1 - *y;
	*x = TPD_RES_X + 1 - *x;
}
/*
static void tpd_rotate_270(int *x, int *y)
{
//	int temp;

	*y = TPD_RES_Y + 1 - *y;

	*x = (*x * TPD_RES_Y) / TPD_RES_X;
	*y = (*y * TPD_RES_X) / TPD_RES_Y;

	tpd_swap_xy(x, y);
}
*/
#endif
struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

/*dma declare, allocate and release*/
#define __MSG_DMA_MODE__
#ifdef __MSG_DMA_MODE__
	u8 *g_dma_buff_va_onsell = NULL;
	dma_addr_t g_dma_buff_pa_onsell = 0;
#endif

#ifdef __MSG_DMA_MODE__

	static void msg_dma_alloct(void)
	{
	    if (NULL == g_dma_buff_va_onsell)
    		{
       		 tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
       		 g_dma_buff_va_onsell = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa_onsell, GFP_KERNEL);
    		}

	    	if(!g_dma_buff_va_onsell)
		{
	        	TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	    	}
	}
	static void msg_dma_release(void){
		if(g_dma_buff_va_onsell)
		{
	     		dma_free_coherent(NULL, 128, g_dma_buff_va_onsell, g_dma_buff_pa_onsell);
	        	g_dma_buff_va_onsell = NULL;
	        	g_dma_buff_pa_onsell = 0;
			TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	    	}
	}
#endif

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
/* static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX; */
/* static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX; */
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id ft5x0x_tpd_id[] = {{"ft5x0x", 0}, {} };
static const struct of_device_id ft5x0x_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft5x0x_dt_match),
		.name = "ft5x0x",
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5x0x_tpd_id,
	.detect = tpd_i2c_detect,
};

static int of_get_ft5x0x_platform_data(struct device *dev)
{
	/*int ret, num;*/

	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ft5x0x_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}
	
	//tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "rst-gpio", 0);
	//tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "int-gpio", 0);
	/*
	ret = of_property_read_u32(dev->of_node, "rst-gpio", &num);
	if (!ret)
		tpd_rst_gpio_number = num;
	ret = of_property_read_u32(dev->of_node, "int-gpio", &num);
	if (!ret)
		tpd_int_gpio_number = num;
    */
	TPD_DMESG("g_vproc_en_gpio_number %d\n", tpd_rst_gpio_number);
	TPD_DMESG("g_vproc_vsel_gpio_number %d\n", tpd_int_gpio_number);
	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static ssize_t show_scp_ctrl(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t store_scp_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u32 cmd;
	Touch_IPI_Packet ipi_pkt;

	if (kstrtoul(buf, 10, &cmd)) {
		TPD_DEBUG("[SCP_CTRL]: Invalid values\n");
		return -EINVAL;
	}

	TPD_DEBUG("SCP_CTRL: Command=%d", cmd);
	switch (cmd) {
	case 1:
	    /* make touch in doze mode */
	    tpd_scp_wakeup_enable(TRUE);
	    tpd_suspend(NULL);
	    break;
	case 2:
	    tpd_resume(NULL);
	    break;
		/*case 3:
	    // emulate in-pocket on
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 1;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;
	case 4:
	    // emulate in-pocket off
	    ipi_pkt.cmd = IPI_COMMAND_AS_GESTURE_SWITCH,
	    ipi_pkt.param.data = 0;
		md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
	    break;*/
	case 5:
		{
				Touch_IPI_Packet ipi_pkt;

				ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
			    ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
			ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
				ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
			ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;
			if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0) < 0)
				TPD_DEBUG("[TOUCH] IPI cmd failed (%d)\n", ipi_pkt.cmd);

			break;
		}
	default:
	    TPD_DEBUG("[SCP_CTRL] Unknown command");
	    break;
	}

	return size;
}
static DEVICE_ATTR(tpd_scp_ctrl, 0664, show_scp_ctrl, store_scp_ctrl);
#endif

static struct device_attribute *ft5x0x_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif
};


static ssize_t mtk_ctp_firmware_vertion_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

  int ret;
   
  ret=sprintf(buf, "%s:%s:0x%02x:%x\n",vendor_name,"FT5446",ctp_fw_version,0);
			   
			   
			     
  return ret;
 
}
static struct kobj_attribute ctp_firmware_vertion_attr = {
	.attr = {
		 .name = "firmware_version",                    
		 .mode = S_IRUGO,
		 },
	.show = &mtk_ctp_firmware_vertion_show,
};

static ssize_t mtk_ctp_firmware_update_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
    unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
#if FT_ESD_PROTECT
    esd_switch(0);
	apk_debug_flag = 1;
//printk("\n  zax v= %d \n",apk_debug_flag);
#endif
	if (copy_from_user(&writebuf, buf, buflen)) {
		dev_err(&fts_i2c_client_onsell->dev, "%s:copy from user error\n", __func__);
#if FT_ESD_PROTECT
	esd_switch(1);
    apk_debug_flag = 0;
#endif
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			TPD_DEBUG("%s\n", upgrade_file_path);
			//#if FT_ESD_PROTECT
			//	esd_switch(0);apk_debug_flag = 1;
			//#endif
			disable_irq(fts_i2c_client_onsell->irq);
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client_onsell, upgrade_file_path);
			enable_irq(fts_i2c_client_onsell->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client_onsell->dev, "%s:upgrade failed.\n", __func__);
				#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				return ret;
			}
			//#if FT_ESD_PROTECT
			//	esd_switch(1);apk_debug_flag = 0;
			//#endif
		}
		break;
	//case PROC_SET_TEST_FLAG:
	
	//	break;
	case PROC_SET_TEST_FLAG:
		#if FT_ESD_PROTECT
		apk_debug_flag=writebuf[1];
		if(1==apk_debug_flag)
			esd_switch(0);
		else if(0==apk_debug_flag)
			esd_switch(1);
		printk("\n zax flag=%d \n",apk_debug_flag);
		#endif
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write_onsell(fts_i2c_client_onsell, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client_onsell->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write_onsell(fts_i2c_client_onsell, writebuf + 1, writelen);
		if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
			dev_err(&fts_i2c_client_onsell->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		TPD_DEBUG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb_onsell(fts_i2c_client_onsell);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		if(writelen>0)
		{
			ret = fts_i2c_write_onsell(fts_i2c_client_onsell, writebuf + 1, writelen);
			if (ret < 0) {
#if FT_ESD_PROTECT
					esd_switch(1);apk_debug_flag = 0;
				#endif
				dev_err(&fts_i2c_client_onsell->dev, "%s:write iic error\n", __func__);
				return ret;
			}
		}
		break;
	default:
		break;
	}
	
	#if FT_ESD_PROTECT
		//printk("\n  zax proc w 1 \n");		
esd_switch(1);apk_debug_flag = 0;
//printk("\n  zax v= %d \n",apk_debug_flag);
			#endif
	return count; 
 
}


static struct kobj_attribute ctp_firmware_update_attr = {
	.attr = {
		 .name = "firmware_update",
		 .mode = S_IWUGO,
		 },
	.store = &mtk_ctp_firmware_update_store,

};
static ssize_t mtk_ctp_vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
 int ret;
 ret=sprintf(buf,"%s\n",vendor_name); 
 return ret;
}

static struct kobj_attribute ctp_vendor_attr = {
	.attr = {
		 .name = "vendor",
		 .mode = S_IRUGO,
		 },
	.show = &mtk_ctp_vendor_show,
};

static struct attribute *mtk_properties_attrs[] = {
	&ctp_firmware_vertion_attr.attr,
	&ctp_vendor_attr.attr,
	&ctp_firmware_update_attr.attr,
	NULL
};

static struct attribute_group mtk_ctp_attr_group = {
	.attrs = mtk_properties_attrs,
};

#if 1
static int create_ctp_node(void)
{
  int ret;
  virtual_dir = virtual_device_parent(NULL);
  if(!virtual_dir)
  {
   printk("Get virtual dir failed\n");
   return -ENOMEM;
  }
  touchscreen_dir=kobject_create_and_add("touchscreen",virtual_dir);
  if(!touchscreen_dir)
  {
   printk("Create touchscreen dir failed\n");
   return -ENOMEM;
  }
  touchscreen_dev_dir=kobject_create_and_add("touchscreen_dev",touchscreen_dir);
  if(!touchscreen_dev_dir)
  {
   printk("Create touchscreen_dev dir failed\n");
   return -ENOMEM;
  }
  ret=sysfs_create_group(touchscreen_dev_dir, &mtk_ctp_attr_group);
  if(ret)
  {
    printk("create mtk_ctp_firmware_vertion_attr_group error\n");
  } 
 
  return 0;
}


#endif
static void tpd_down(int x, int y, int p, int id)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
        
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
		TPD_DEBUG("%s x:%d y:%d p:%d\n", __func__, x, y, p);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, 720-x);  //modify by qiangang@wind-mobi.com 20170213
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, 1280-y);  //modify by qiangang@wind-mobi.com 20170213
		input_mt_sync(tpd->dev);
	
	}
}

static void tpd_up(int x, int y,int id)
{
#if defined(CONFIG_TPD_ROTATE_90)
	tpd_rotate_90(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_270)
	tpd_rotate_270(&x, &y);
#elif defined(CONFIG_TPD_ROTATE_180)
	tpd_rotate_180(&x, &y);
#endif

#ifdef TPD_SOLVE_CHARGING_ISSUE
	if (0 != x) {
#else
	{
#endif
		TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);


	}
}

/*Coordination mapping*/
/*
static void tpd_calibrate_driver(int *x, int *y)
{
	int tx;

	tx = ((tpd_def_calmat[0] * (*x)) + (tpd_def_calmat[1] * (*y)) + (tpd_def_calmat[2])) >> 12;
	*y = ((tpd_def_calmat[3] * (*x)) + (tpd_def_calmat[4] * (*y)) + (tpd_def_calmat[5])) >> 12;
	*x = tx;
}
*/
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[40] = {0};
	u8 report_rate = 0;
	u16 high_byte, low_byte;
	char writebuf[10]={0};
	//u8 fwversion = 0;

	writebuf[0]=0x00;
	fts_i2c_read_onsell(i2c_client, writebuf,  1, data, 32);
	//fts_read_reg(i2c_client, 0xa6, &fwversion);
	fts_read_reg(i2c_client, 0x88, &report_rate);

	//TPD_DEBUG("FW version=%x]\n", fwversion);

#if 0
	printk("wind_tp received raw data from touch panel as following:\n");
	for (i = 0; i < 8; i++)
		printk("wind_tp data[%d] = 0x%02X ", i, data[i]);
	printk("\n");
	for (i = 8; i < 16; i++)
		printk("wind_tp data[%d] = 0x%02X ", i, data[i]);
	printk("\n");
	for (i = 16; i < 24; i++)
		printk("wind_tp data[%d] = 0x%02X ", i, data[i]);
	printk("\n");
	for (i = 24; i < 32; i++)
		printk("wind_tp data[%d] = 0x%02X ", i, data[i]);
	printk("\n");
#endif
	if (report_rate < 8) {
		report_rate = 0x8;
		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
	}

	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if ((data[0] & 0x70) != 0)
		return false;

	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	//for (i = 0; i < TPD_SUPPORT_POINTS; i++)
		//cinfo->p[i] = 1;	/* Put up */

	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;

	TPD_DEBUG("Number of touch points = %d\n", cinfo->count);

	TPD_DEBUG("Procss raw data...\n");

	for (i = 0; i < cinfo->count; i++) {
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
		cinfo->id[i] = data[3+6*i+2]>>4; 						// touch id

		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;

		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];
		high_byte <<= 8;
		high_byte &= 0x0F00;

		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;
       //printk("cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
		//TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
		//cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}




#ifdef CONFIG_TPD_HAVE_CALIBRATION
	for (i = 0; i < cinfo->count; i++) {
		tpd_calibrate_driver(&(cinfo->x[i]), &(cinfo->y[i]));
		TPD_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
		cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
	}
#endif

	return true;

};


#if defined (CONFIG_MTK_I2C_EXTENSION) && defined (CONFIG_FT_AUTO_UPGRADE_SUPPORT)

int fts_i2c_read_onsell(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret=0;
//tuwenzan@wind-mobi.com modify at 20160602 begin
#if CTP_ESD_PROTECT
		int i = 0;
		//	fts_esd_protection_notice();
		for(i = 0; i < 3; i++)
		{
			ret = fts_esd_protection_notice();
			if(0 == ret){
				printk(" tp_esd %s tp is ok.\n", __func__);
				break;	// can use I2C
			}else
			{
				printk(" tp_esd %s tp is fail return :%d.\n", __func__,ret);
				continue;
			}
		}
		if(3 == i)
		{
		//	FTS_COMMON_DBG("[focal] ESD are still use I2C. \n");
		}
		
		
#endif
//tuwenzan@wind-mobi.com modify at 20160602 end
	mutex_lock(&i2c_rw_access);
	if((NULL!=client) && (writelen>0) && (writelen<=128))
	{
		memcpy(g_dma_buff_va_onsell, writebuf, writelen);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa_onsell, writelen))!=writelen)
			printk("i2c write failed\n");
		client->addr = (client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
	}
	if((NULL!=client) && (readlen>0) && (readlen<=128))
	{
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa_onsell, readlen);
		memcpy(readbuf, g_dma_buff_va_onsell, readlen);
		client->addr = (client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
	}
	mutex_unlock(&i2c_rw_access);
	return ret;	
}
int fts_i2c_write_onsell(struct i2c_client *client, char *writebuf, int writelen)
{
	int i = 0;
	int ret = 0;
//tuwenzan@wind-mobi.com modify at 20160602 begin
	#if CTP_ESD_PROTECT
	fts_esd_protection_notice();
    #endif
//tuwenzan@wind-mobi.com modify at 20160602 end
	if (writelen <= 8) {
	    client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);
		return i2c_master_send(client, writebuf, writelen);
	}
	else if((writelen > 8)&&(NULL != tpd_i2c_dma_va))
	{
		for (i = 0; i < writelen; i++)
			tpd_i2c_dma_va[i] = writebuf[i];
		client->addr = (client->addr & I2C_MASK_FLAG )| I2C_DMA_FLAG;
	    ret = i2c_master_send(client, (unsigned char *)tpd_i2c_dma_pa, writelen);
	    client->addr = client->addr & I2C_MASK_FLAG & ~I2C_DMA_FLAG;
		return ret;
	}
	return 1;
}
#else
int fts_i2c_read_onsell(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
  int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			pr_err("f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("%s:i2c read error.\n", __func__);
	}
	return ret;
}


/************************************************************************
* Name: fts_i2c_write_onsell
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write_onsell(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error.\n", __func__);

	return ret;
}

#endif
/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write_onsell(client, buf, sizeof(buf));
}
/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	printk("wind_tp_test i2c_client->addr = %d ******************",client->addr);
	return fts_i2c_read_onsell(client, &regaddr, 1, regvalue, 1);

}

#if USB_CHARGE_DETECT
//qiumeng@wind-mobi.com 20160503 begin
//extern int FG_charging_status ;
//extern bool gFG_Is_Charging;
extern bool bat_is_charger_exist(void);
//qiumeng@wind-mobi.com 20160503 end
int close_to_ps_flag_value = 1;	// 1: close ; 0: far away
int charging_flag = 0;
#endif
static int touch_event_handler(void *unused)
{
	int i = 0;
	#if FTS_GESTRUE_EN
	int ret = 0;
	u8 state = 0;
	#endif
	#if USB_CHARGE_DETECT
	u8 data;
	#endif
	struct touch_info cinfo, pinfo, finfo;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	/*shenyong@wind-mobi.com 20160721 start ***/
	struct file	*filp;
	u8 check_flag = 0;
	/*shenyong@wind-mobi.com 20160721 end ***/
	if (tpd_dts_data.use_tpd_button) {
		memset(&finfo, 0, sizeof(struct touch_info));
		for (i = 0; i < TPD_SUPPORT_POINTS; i++)
			finfo.p[i] = 1;
	}

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		printk(" wind_tp_test **** i2c_client->addr = %d \n",i2c_client->addr);
		/*enable_irq(touch_irq);*/
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

#if USB_CHARGE_DETECT
//qiumeng@wind-mobi.com 20160503 begin
		//if((FG_charging_status != 0) && (charging_flag == 0))
		//if((gFG_Is_Charging != false) && (charging_flag == 0))
			/*shenyong@wind-mobi.com 20160721 start ***/
	if(check_flag == 0){
		filp = filp_open("/sys/devices/platform/battery/ADC_Charger_Voltage", O_RDONLY, 0);
		if (IS_ERR(filp)) {
            pr_err("open /sys/devices/platform/battery/ADC_Charger_Voltage fail !!!! and check_flag =%d\n",check_flag);
		}else{
			check_flag = 1;
			filp_close(filp,NULL);
		}
	}
	if(check_flag ==1){
		if((bat_is_charger_exist() != false) && (charging_flag == 0))
//qiumeng@wind-mobi.com 20160503 end
		{
			data = 0x1;
			charging_flag = 1;
			fts_write_reg(i2c_client, 0x8B, 0x01);  
		}
		else
		{    
//qiumeng@wind-mobi.com 20160503 begin		    
			//if((FG_charging_status == 0) && (charging_flag == 1))
			//if((gFG_Is_Charging == false) && (charging_flag == 1))
			if((bat_is_charger_exist() == false) && (charging_flag == 1))
//qiumeng@wind-mobi.com 20160503 end
			{
				charging_flag = 0;
				data = 0x0;
				fts_write_reg(i2c_client, 0x8B, 0x00);  					
			}
		}
	}
#endif
/*shenyong@wind-mobi.com 20160721 end ***/
		#if FTS_GESTRUE_EN
			ret = fts_read_reg(fts_i2c_client_onsell, 0xd0,&state);
			if (ret<0)
			{
				printk("[Focal][Touch] read value fail");
				//return ret;
			}
			//printk("tp_esd gesture fts_read_Gestruedata state=%d\n",state);
		     	if(state ==1)
		     	{
			        fts_read_Gestruedata();
			        continue;
		    	}
		 #endif

		//TPD_DEBUG("touch_event_handler start\n");
//tuwenzan@wind-mobi.com modify at 20160602 begin
#if CTP_ESD_PROTECT
					 apk_debug_flag = 1;
#endif
//tuwenzan@wind-mobi.com modify at 20160602 end
		if (tpd_touchinfo(&cinfo, &pinfo)) {
		
			
		
			if (tpd_dts_data.use_tpd_button) {
				if (cinfo.p[0] == 0)
					memcpy(&finfo, &cinfo, sizeof(struct touch_info));
			}
			printk(" wind_tp_test  pinfo.x[0]=%d, pinfo.y[0]=%d,pinfo.id[0]=%d \n",pinfo.x[0], pinfo.y[0],pinfo.id[0]);
			if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
			&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
				TPD_DEBUG("Dummy release --->\n");
				printk(" wind_tp_test  pinfo.x[0]=%d, pinfo.y[0]=%d,pinfo.id[0]=%d \n",pinfo.x[0], pinfo.y[0],pinfo.id[0]);
				tpd_up(pinfo.x[0], pinfo.y[0],pinfo.id[0]);
				input_sync(tpd->dev);
				continue;
			}
			
            #if 0 
			if (tpd_dts_data.use_tpd_button) {
				if ((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y)
				&& ((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
					TPD_DEBUG("Dummy key release --->\n");
					//tpd_button(pinfo.x[0], pinfo.y[0], 0);
					tpd_down(pinfo.x[0], pinfo.y[0], 0)
					input_sync(tpd->dev);
					continue;
				}

			if ((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y)) {
				if (finfo.y[0] > TPD_RES_Y) {
					if ((cinfo.p[0] == 0) || (cinfo.p[0] == 2)) {
							TPD_DEBUG("Key press --->\n");
							tpd_button(pinfo.x[0], pinfo.y[0], 1);
					} else if ((cinfo.p[0] == 1) &&
						((pinfo.p[0] == 0) || (pinfo.p[0] == 2))) {
							TPD_DEBUG("Key release --->\n");
							tpd_down(pinfo.x[0], pinfo.y[0], 0);
					}
					input_sync(tpd->dev);
				}
				continue;
			}
			
			}
             #endif

			if (cinfo.count > 0) {
				for (i = 0; i < cinfo.count; i++)
					tpd_down(cinfo.x[i], cinfo.y[i], i + 1, cinfo.id[i]);
					//tpd_down(cinfo.x[i], cinfo.y[i], , i);
			} else {
#ifdef TPD_SOLVE_CHARGING_ISSUE
				tpd_up(1, 48);
#else
				tpd_up(cinfo.x[0], cinfo.y[0],cinfo.id[0]);
#endif

			}
			input_sync(tpd->dev);

		}
//tuwenzan@wind-mobi.com modify at 20160602 begin
		#if CTP_ESD_PROTECT
			apk_debug_flag = 0;
       #endif
//tuwenzan@wind-mobi.com modify at 20160602 end
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	//tuwenzan@wind-mobi.com modify at 20160602 begin
	#if CTP_ESD_PROTECT
	count_irq ++;
    #endif
	//tuwenzan@wind-mobi.com modify at 20160602 end
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}
static int tpd_irq_registration(void)
{  
    #if 0
 	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if (ret > 0)
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	
	return 0;
	#endif
	
	struct device_node *node = NULL;
		int ret = 0;
	
		//node = of_find_compatible_node(NULL, NULL, "mediatek,cap_touch");
		node = of_find_matching_node(node, touch_of_match);
		if (node) {
			/*touch_irq = gpio_to_irq(tpd_int_gpio_number);*/
			touch_irq = irq_of_parse_and_map(node, 0);
			ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
						IRQF_TRIGGER_NONE, TPD_DEVICE, NULL);
				if (ret > 0)
					TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
		} else {
			TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
		}
		return 0;
	
}
#if 0
int hidi2c_to_stdi2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;

	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;

	fts_i2c_write_onsell(client, auc_i2c_write_buf, 3);

	msleep(10);

	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;

	fts_i2c_read_onsell(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

	if(0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
	{
		bRet = 1;
	}
	else
		bRet = 0;

	return bRet;

}
#endif
#if MTK_CTP_NODE 
#define CTP_PROC_FILE "tp_info"

static int ctp_proc_read_show (struct seq_file* m, void* data)
{
	char temp[40] = {0};
	//char vendor_name[20] = {0};

	/*if(temp_pid == 0x00 || temp_pid == 0x01){	//add by liuzhen
		sprintf(vendor_name,"%s","O-Film");
	}else if(temp_pid == 0x02){
		sprintf(vendor_name,"%s","Mudong");
	}else{
		sprintf(vendor_name,"%s","Reserve");
	}*/
	//sprintf(temp, "[Vendor]O-Film,[Fw]%s,[IC]GT915\n",temp_ver); //changed by cao
	sprintf(temp, "[Vendor]%s,[Fw]%x,[IC]FT5446\n","O-Film",4); 
	seq_printf(m, "%s\n", temp);
	//printk("vid:%s,firmware:0x%04x\n",temp_ver, temp_pid);
	return 0;
}

static int ctp_proc_open (struct inode* inode, struct file* file) 
{
    return single_open(file, ctp_proc_read_show, inode->i_private);
}

static const struct file_operations g_ctp_proc = 
{
    .open = ctp_proc_open,
    .read = seq_read,
};
#endif


static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = TPD_OK;
	u8 report_rate = 0;
	int reset_count = 0;
	char data;
	int err = 0;
	int ret = 0;
//hebiao@wind-mobi.com 20161105 begin
#ifdef CONFIG_WIND_DEVICE_INFO
extern wind_device_info_t wind_device_info;
#endif
//hebiao@wind-mobi.com 20161105 end


	printk(" enter wind_tp xxxxx %s   client=%d \n" ,__func__, client->addr);
	i2c_client = client;
	fts_i2c_client_onsell = client;
	fts_input_dev_onsell=tpd->dev;
	if(i2c_client->addr != 0x38)
	{
		i2c_client->addr = 0x38;
		printk("frank_zhonghua:i2c_client_FT->addr=%d\n",i2c_client->addr);
	}

	

	
	of_get_ft5x0x_platform_data(&client->dev);
	/* configure the gpio pins */

	TPD_DMESG("mtk_tpd: tpd_probe ft5x0x\n");

#if 0
	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
#endif
	tpd->tpd_dev->of_node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-touch");
	if (tpd->tpd_dev->of_node)
	{
		printk("of_find_compatible_node-->mediatek,mt6755-touch\n");

		tpd->reg = regulator_get(tpd->tpd_dev, "vtouch"); //modified by hebiao 2016907
		ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	//modified by ranyanhao 20161021
		if (ret) {
			printk("regulator_set_voltage(%d) failed!\n", ret);
			return -1;
		}
		ret = regulator_enable(tpd->reg);
	}
	else
	{
		printk("of_find_compatible_node-->mediatek,mt6797-touch,error\n");
	}

	/* set INT mode */

	//tpd_gpio_as_int(tpd_int_gpio_number);
	tpd_gpio_as_int(1);// eint 1
//sunjingtao@wind-mobi.com 20160617 begin
	//tpd_irq_registration();
//sunjingtao@wind-mobi.com 20160617 end
	msleep(100);
	msg_dma_alloct();

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT


    if (NULL == tpd_i2c_dma_va)
    {
        tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        tpd_i2c_dma_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 250, &tpd_i2c_dma_pa, GFP_KERNEL);
    }
    if (!tpd_i2c_dma_va)
		TPD_DMESG("TPD dma_alloc_coherent error!\n");
	else
		TPD_DMESG("TPD dma_alloc_coherent success!\n");
#endif

#if FTS_GESTRUE_EN
	fts_Gesture_init(tpd->dev);
#endif

reset_proc:
	/* Reset CTP */
	//printk("hb test reset_proc\n");
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	err = fts_read_reg(i2c_client, 0x00, &data);
	printk("wind_tp fts_i2c: data = %d\n", data);
	if(err< 0 || data!=0)// reg0 data running state is 0; other state is not 0
	{
	    printk("wind_tp I2C transfer error\n");
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
		if ( ++reset_count < TPD_MAX_RESET_COUNT )
		{
			goto reset_proc;
		}
#endif
		retval	= regulator_disable(tpd->reg); //disable regulator
		if(retval)
		{
			printk("focaltech tpd_probe regulator_disable() failed!\n");
		}

		regulator_put(tpd->reg);
		msg_dma_release();
		gpio_free(tpd_rst_gpio_number);
		gpio_free(tpd_int_gpio_number);
		return -1;
	}
//sunjingtao@wind-mobi.com 20160617 begin
	tpd_irq_registration();
//sunjingtao@wind-mobi.com 20160617 end

	tpd_load_status = 1;

	#ifdef  MTK_CTP_NODE
    if((proc_create(CTP_PROC_FILE,0444,NULL,&g_ctp_proc))==NULL)
    {
      printk("proc_create tp vertion node error\n");
	}
	#endif
	#ifdef  MTK_CTP_NODE
	create_ctp_node();
    #endif
	
	//touch_class = class_create(THIS_MODULE,"FT5446");
	#ifdef SYSFS_DEBUG
                fts_create_sysfs_onsell(fts_i2c_client_onsell);
	#endif
	//hidi2c_to_stdi2c(fts_i2c_client_onsell);
	fts_get_upgrade_array();
	#ifdef FTS_CTL_IIC
		 if (fts_rw_iic_drv_init(fts_i2c_client_onsell) < 0)
			 dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
	#endif

	#ifdef FTS_APK_DEBUG
		fts_create_apk_debug_channel_onsell(fts_i2c_client_onsell);
	#endif

		printk(" wind_tp have ............... ");
	#if 0
	/* Reset CTP */
	
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	#endif

	#ifdef TPD_AUTO_UPGRADE
		printk("hb ********************wind_tp Enter CTP Auto Upgrade********************\n");
		is_update = true;
		fts_ctpm_auto_upgrade_onsell(fts_i2c_client_onsell);
		is_update = false;


	#endif

	#if 0
	/* Reset CTP */

	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	#endif
/*#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
	tpd_auto_upgrade(client);
#endif*/
	/* Set report rate 80Hz */
	report_rate = 0x8;
	if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0) {
		if ((fts_write_reg(i2c_client, 0x88, report_rate)) < 0)
			TPD_DMESG("I2C write report rate error, line: %d\n", __LINE__);
			printk("I2C write report rate error, line: %d\n", __LINE__);
	}

#if FTS_GESTRUE_EN
	tp_gesture_proc_init(); //lihaiyan@wind-mobi.com 20170412
#endif
	/* tpd_load_status = 1; */
	//tuwenzan@wind-mobi.com modify at 20160602 begin

    #if CTP_ESD_PROTECT
	fts_esd_protection_init();
     #endif

	 //tuwenzan@wind-mobi.com modify at 20160602 end
	thread_tpd_onsell = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd_onsell)) {
		retval = PTR_ERR(thread_tpd_onsell);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd_onsell: %d\n", retval);
	}

	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef TIMER_DEBUG
	init_test_timer();
#endif

#ifdef CONFIG_WIND_DEVICE_INFO
	{
		u8 fwvr;
		u8 ver;
		fts_read_reg(client, 0xA6, &fwvr);
		fts_read_reg(client, 0xA8, &ver);
		sprintf(wind_device_info.ctp_module_info.ic_name, "%s", "ft3427");
		wind_device_info.ctp_module_info.fwvr = fwvr;
		wind_device_info.ctp_module_info.vendor = ver;
	    
	}
#endif
   
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int ret;

	ret = get_md32_semaphore(SEMAPHORE_TOUCH);
	if (ret < 0)
		pr_err("[TOUCH] HW semaphore reqiure timeout\n");
#endif

	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
#ifdef CONFIG_CUST_FTS_APK_DEBUG
	//ft_rw_iic_drv_exit();
#endif

#ifdef CONFIG_FT_AUTO_UPGRADE_SUPPORT
	if (tpd_i2c_dma_va) {
		dma_free_coherent(NULL, 4096, tpd_i2c_dma_va, tpd_i2c_dma_pa);
		tpd_i2c_dma_va = NULL;
		tpd_i2c_dma_pa = 0;
	}
#endif

#if FTS_GESTRUE_EN
	tp_gesture_proc_remove(); //lihaiyan@wind-mobi.com
#endif

	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

	return 0;
}
//tuwenzan@wind-mobi.com modify at 20160608 begin
#if CTP_ESD_PROTECT
 /************************************************************************
 * Name: force_reset_guitar
 * Brief: reset
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
 static void force_reset_guitar(void)
 {
 //tuwenzan@wind-mobi.com modify at 20150607 begin
	 int retval;
 	/* Reset CTP */
//	printk("twz enter reset guitar\n");
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	 msleep(10);
	 retval = regulator_disable(tpd->reg);
	 if (retval != 0)
		 TPD_DMESG("Failed to disable reg-vgp6: %d\n", retval);
	 msleep(200);
	 retval = regulator_enable(tpd->reg);
	 if (retval != 0)
		 TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	 msleep(10);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	 tpd_gpio_as_int(tpd_int_gpio_number);
	 msleep(300);
 //tuwenzan@wind-mobi.com modify at 20150607 end
 	printk(" tp_esd %s tp is restared.\n", __func__);
 
 }


#define A3_REG_VALUE					0X54 //0x87   //lihaiyan@wind-Mobi.com 20161021 
//#define RESET_91_REGVALUE_SAMECOUNT 	5
//tuwenzan@wind-mobi.com modify at 20160608 end
//tuwenzan@wind-mobi.com modify at 20160608 begin
void ctp_esd_check_func(void)
{
		int i;
		int ret = -1;
	 u8 data;
		int reset_flag = 0;

	 printk("tp_esd enter ctp_esd_check_func for protect tp\n");
		for (i = 0; i < 3; i++)
	 {
		
			ret =  fts_read_reg(i2c_client, 0xA3, &data);
			printk("tp_esd focal--fts_esd_check_func-0xA3:%x\n", data);
			if (ret==1 && A3_REG_VALUE==data) {
				break;
			}
	 }
	
		if (i >= 3) {
		 force_reset_guitar();
			printk("tp_esd focal--tpd reset. i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
			reset_flag = 1;
			goto FOCAL_RESET_A3_REGISTER;
	 }
		
		
FOCAL_RESET_A3_REGISTER:
		data=0;
		// ret = fts_write_reg(i2c_client, 0x8F,data);
	
	 return;
 }
#endif
//tuwenzan@wind-mobi.com modify at 20160608 end

static int tpd_local_init(void)
{
	int retval;

	TPD_DMESG("Focaltech FT5x0x I2C Touchscreen Driver...\n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
     /* tpd_load_status = 1; */
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))

	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);

#endif

	TPD_DMESG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static s8 ftp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	char gestrue_on = 0x01;
	char gestrue_data;
	int i;

	/* TPD_DEBUG("Entering doze mode..."); */
	pr_alert("Entering doze mode...");

	/* Enter gestrue recognition mode */
	ret = fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);
	if (ret < 0) {
		/* TPD_DEBUG("Failed to enter Doze %d", retry); */
		pr_alert("Failed to enter Doze %d", retry);
		return ret;
	}
	msleep(30);

	for (i = 0; i < 10; i++) {
		fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
		if (gestrue_data == 0x01) {
			doze_status = DOZE_ENABLED;
			/* TPD_DEBUG("FTP has been working in doze mode!"); */
			pr_alert("FTP has been working in doze mode!");
			break;
		}
		msleep(20);
		fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, gestrue_on);

	}

	return ret;
}
#endif

static void tpd_resume(struct device *h)
{
	int retval = TPD_OK;

	TPD_DEBUG("TPD wake up\n");

	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);

	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(200);

	#if FTS_GESTRUE_EN
		fts_write_reg(fts_i2c_client_onsell,0xD0,0x00);
	#endif


#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	doze_status = DOZE_DISABLED;
	/* tpd_halt = 0; */
	int data;

	data = 0x00;

	fts_write_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, data);
#else
	enable_irq(touch_irq);
#endif
#if USB_CHARGE_DETECT
//qiumeng@wind-mobi.com 20160503 begin
	//if(FG_charging_status != 0)
	//if(gFG_Is_Charging != false)
	if(bat_is_charger_exist() != false)
//qiumeng@wind-mobi.com 20160503 end
	{
		charging_flag = 0;
	}
	else
	{
		charging_flag = 1;
	}
#endif
//tuwenzan@wind-mobi.com modify at 20160602 begin
#if CTP_ESD_PROTECT
	count_irq = 0;
	fts_esd_protection_resume();
#endif
//tuwenzan@wind-mobi.com modify at 20160602 end
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
void tpd_scp_wakeup_enable(bool en)
{
	tpd_scp_doze_en = en;
}

void tpd_enter_doze(void)
{

}
#endif



// wangbing@wind-mobi.com 20170331 begin
#if defined(CONFIG_MTK_LENS_GT9762AF_GZ_E266L_SUPPORT) || defined(CONFIG_MTK_LENS_GT9762AF_OFG_E266L_SUPPORT)

// this struct hase been defined by lens 
extern  struct i2c_client *GT9762AF_I2Cclient_common_E266;   //lihaiyan@wind-mobi.com 20160209 add

/*
 * This function will be performed only once
 */
static void AF_Into_Power_Down_Mode(void)
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
// wangbing@wind-mobi.com 20170331 end	


//lihaiyan@wind-mobi.bom 20170425 begin
#if FTS_GESTRUE_EN
void gesture_mode_exchange(void)
{
	
	gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_DOUBLE_CLICK_FLAG) ? (gesture_mode_tmp[0] | (1<<4) ) : (gesture_mode_tmp[0] & (~(1<<4)));
	gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_UP_DIRECT_FLAG) 	? (gesture_mode_tmp[0] | (1<<2) ) : (gesture_mode_tmp[0] & (~(1<<2)));
	gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_DOWN_DIRECT_FLAG) 	? (gesture_mode_tmp[0] | (1<<3) ) : (gesture_mode_tmp[0] & (~(1<<3)));
	gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_LEFT_DIRECT_FLAG) 	? (gesture_mode_tmp[0] | (1<<0) ) : (gesture_mode_tmp[0] & (~(1<<0)));
	gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_RIGHT_DIRECT_FLAG) ? (gesture_mode_tmp[0] | (1<<1) ) : (gesture_mode_tmp[0] & (~(1<<1)));
	
	
	//gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_M_FLAG) ? (gesture_mode_tmp[1] | (1<<2) ) : (gesture_mode_tmp[1] & (~(1<<2)));
	gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_W_FLAG) ? (gesture_mode_tmp[1] | (1<<1) ) : (gesture_mode_tmp[1] & (~(1<<1)));
	gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_C_FLAG) ? (gesture_mode_tmp[1] | (1<<4) ) : (gesture_mode_tmp[1] & (~(1<<4)));
	gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_E_FLAG) ? (gesture_mode_tmp[1] | (1<<3) ) : (gesture_mode_tmp[1] & (~(1<<3)));
	//gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_O_FLAG) ? (gesture_mode_tmp[1] | (1<<0) ) : (gesture_mode_tmp[1] & (~(1<<0)));

	gesture_mode_tmp[3] = (g_GestureWakeupMode_wind & GESTURE_MODE_V_FLAG) ? (gesture_mode_tmp[3] | (1<<4) ) : (gesture_mode_tmp[3] & (~(1<<4)));
	gesture_mode_tmp[2] = (g_GestureWakeupMode_wind & GESTURE_MODE_S_FLAG) ? (gesture_mode_tmp[2] | (1<<6) ) : (gesture_mode_tmp[2] & (~(1<<6)));
	//gesture_mode_tmp[2] = (g_GestureWakeupMode_wind & GESTURE_MODE_L_FLAG) ? (gesture_mode_tmp[2] | (1<<4) ) : (gesture_mode_tmp[2] & (~(1<<4)));
	
	gesture_mode_tmp[4] = (g_GestureWakeupMode_wind & GESTURE_MODE_Z_FLAG) ? (gesture_mode_tmp[4] | (1<<5) ) : (gesture_mode_tmp[4] & (~(1<<5)));
	
	printk("[tpd_wind][gesture] %s , g_GestureWakeupMode_wind = 0x%x , gesture_mode_tmp[0]=0x%x [1]=0x%x [2]=0x%x [3]=0x%x [4]=0x%x [5]=0x%x \n", __func__, 
						g_GestureWakeupMode_wind, gesture_mode_tmp[0], gesture_mode_tmp[1], gesture_mode_tmp[2], gesture_mode_tmp[3], gesture_mode_tmp[4], gesture_mode_tmp[5]);

}
#endif
//lihaiyan@wind-mobi.bom 20170425 end


static void tpd_suspend(struct device *h)
{
	int retval = TPD_OK;
	static char data = 0x3;
	#if FTS_GESTRUE_EN
	int i = 0;
	u8 state = 0;
	#endif
	printk("TPD enter sleep\n");

#if defined(CONFIG_MTK_LENS_GT9762AF_GZ_E266L_SUPPORT) || defined(CONFIG_MTK_LENS_GT9762AF_OFG_E266L_SUPPORT)
	AF_Into_Power_Down_Mode();   //lihaiyan 20170209 begin	
#endif


	#if FTS_GESTRUE_EN
        if( 0x00000000 != g_GestureWakeupMode_wind){
			gesture_mode_exchange();  // lihaiyan@wind-mobi.com 
			
			//memset(coordinate_x,0,255);
			//memset(coordinate_y,0,255);

			fts_write_reg(i2c_client, 0xd0, 0x01);
		  	fts_write_reg(i2c_client, 0xd1, gesture_mode_tmp[0]);//0xff
			fts_write_reg(i2c_client, 0xd2, gesture_mode_tmp[1]);//0xff
			fts_write_reg(i2c_client, 0xd5, gesture_mode_tmp[2]);//0xff
			fts_write_reg(i2c_client, 0xd6, gesture_mode_tmp[3]);//0xff
			fts_write_reg(i2c_client, 0xd7, gesture_mode_tmp[4]);//0xff
			fts_write_reg(i2c_client, 0xd8, gesture_mode_tmp[5]);//0xff

			msleep(10);

			for(i = 0; i < 10; i++)
			{
				printk("tpd_suspend4 %d",i);
			  	fts_read_reg(i2c_client, 0xd0, &state);

				if(state == 1)
				{
					TPD_DMESG("TPD gesture write 0x01\n");
	        			return;
				}
				else
				{
					fts_write_reg(i2c_client, 0xd0, 0x01);
					fts_write_reg(i2c_client, 0xd1,	gesture_mode_tmp[0]);//0xff
		 			fts_write_reg(i2c_client, 0xd2,	gesture_mode_tmp[1]);//0xff
				    fts_write_reg(i2c_client, 0xd5,	gesture_mode_tmp[2]);//0xff
					fts_write_reg(i2c_client, 0xd6,	gesture_mode_tmp[3]);//0xff
					fts_write_reg(i2c_client, 0xd7,	gesture_mode_tmp[4]);//0xff
				  	fts_write_reg(i2c_client, 0xd8,	gesture_mode_tmp[5]);//0xff
					msleep(10);
			}
		}

		if(i >= 9)
		{
			TPD_DMESG("TPD gesture write 0x01 to d0 fail \n");
			return;
		}
	}
	#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	int sem_ret;

	tpd_enter_doze();

	int ret;
	char gestrue_data;
	char gestrue_cmd = 0x03;
	static int scp_init_flag;

	/* TPD_DEBUG("[tpd_scp_doze]:init=%d en=%d", scp_init_flag, tpd_scp_doze_en); */

	mutex_lock(&i2c_access);

	sem_ret = release_md32_semaphore(SEMAPHORE_TOUCH);

	if (scp_init_flag == 0) {
		Touch_IPI_Packet ipi_pkt;

		ipi_pkt.cmd = IPI_COMMAND_AS_CUST_PARAMETER;
		ipi_pkt.param.tcs.i2c_num = TPD_I2C_NUMBER;
		ipi_pkt.param.tcs.int_num = CUST_EINT_TOUCH_PANEL_NUM;
		ipi_pkt.param.tcs.io_int = tpd_int_gpio_number;
		ipi_pkt.param.tcs.io_rst = tpd_rst_gpio_number;

		TPD_DEBUG("[TOUCH]SEND CUST command :%d ", IPI_COMMAND_AS_CUST_PARAMETER);

		ret = md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 0);
		if (ret < 0)
			TPD_DEBUG(" IPI cmd failed (%d)\n", ipi_pkt.cmd);

		msleep(20); /* delay added between continuous command */
		/* Workaround if suffer MD32 reset */
		/* scp_init_flag = 1; */
	}

	if (tpd_scp_doze_en) {
		TPD_DEBUG("[TOUCH]SEND ENABLE GES command :%d ", IPI_COMMAND_AS_ENABLE_GESTURE);
		ret = ftp_enter_doze(i2c_client);
		if (ret < 0) {
			TPD_DEBUG("FTP Enter Doze mode failed\n");
	  } else {
			int retry = 5;
	    {
				/* check doze mode */
				fts_read_reg(i2c_client, FT_GESTRUE_MODE_SWITCH_REG, &gestrue_data);
				TPD_DEBUG("========================>0x%x", gestrue_data);
	    }

	    msleep(20);
			Touch_IPI_Packet ipi_pkt = {.cmd = IPI_COMMAND_AS_ENABLE_GESTURE, .param.data = 1};

			do {
				if (md32_ipi_send(IPI_TOUCH, &ipi_pkt, sizeof(ipi_pkt), 1) == DONE)
					break;
				msleep(20);
				TPD_DEBUG("==>retry=%d", retry);
			} while (retry--);

	    if (retry <= 0)
				TPD_DEBUG("############# md32_ipi_send failed retry=%d", retry);

			/*while(release_md32_semaphore(SEMAPHORE_TOUCH) <= 0) {
				//TPD_DEBUG("GTP release md32 sem failed\n");
				pr_alert("GTP release md32 sem failed\n");
			}*/

		}
		/* disable_irq(touch_irq); */
	}

	mutex_unlock(&i2c_access);
#else
	disable_irq(touch_irq);
	fts_write_reg(i2c_client, 0xA5, data);  /* TP enter sleep mode */

	retval = regulator_disable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to disable reg-vgp6: %d\n", retval);
	/***********************************************
	*huyunge@wind-mobi.com 20160608
	*release all touch down event start
	************************************************/
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	input_sync(tpd->dev);	
	/***********************************************
	*huyunge@wind-mobi.com 20160608
	*release all touch down event end
	************************************************/
	
#endif
/* lihaiyan@wind-mobi.com 20161018 begin */
#if CTP_ESD_PROTECT
	fts_esd_protection_suspend();
#endif
/* lihaiyan@wind-mobi.com 20161018 end */

}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "FT5x0x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.attrs = {
		.attr = ft5x0x_attrs,
		.num  = ARRAY_SIZE(ft5x0x_attrs),
	},
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5x0x driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek FT5x0x touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
//hebiao@wind-mobi.com add at 20161112 end
