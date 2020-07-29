/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include "kd_flashlight_type.h"

#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
// zhaozhensen@wind-mobi.com 20161108 begin

#include <linux/proc_fs.h>

// zhaozhensen@wind-mobi.com 20161108 end
// zhaozhensen@wind-mobi.com 20161017 begin
#define FLASHLIGHT_TEST

#ifdef FLASHLIGHT_TEST
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#endif
// zhaozhensen@wind-mobi.com 20161017 end
/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */

// zhaozhensen@wind-mobi.com 20161017 begin
#define KTD2684_REG_ENABLE      0x01
#define KTD2684_REG_TIMING      0x08

#define KTD2684_REG_FLASH_LED1  0x03
#define KTD2684_REG_TORCH_LED1  0x05
#define KTD2684_REG_FLASH_LED2  0x04
#define KTD2684_REG_TORCH_LED2  0x06
#define e_DutyNum 13
int flashlight_vendor_id = 0;
int chip_reg = 0;
// zhaozhensen@wind-mobi.com 20161017 end

static int gIsTorch[e_DutyNum] = {  1,  0,  0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0};
static int gLedDuty[e_DutyNum]    = { 40, 20, 47, 55, 63, 71, 79, 85, 85, 85, 85, 85, 85}; // liuying@wind-mobi.com 20171012
static int gLedDuty_aw[e_DutyNum] = { 20,  8, 23, 28, 31, 35, 39, 43, 47,  51,  55,  59,  63}; // liuying@wind-mobi.com 20171012
        /* current(mA) 50, 94, 141, 188, 281, 375, 469, 563, 656, 750, 844, 938, 1031, 1125, 1220, 1313, 1406, 1500 */
        //           { 16, 31,  47,  63,  23,  31,  39,  47,  55,  63,  71,  79,   87,   95,  103,  111,  119,  127};


/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);
// zhaozhensen@wind-mobi.com 20161017 begin
extern int flashlight_gpio_set(int pin , int state);
static struct i2c_client *KTD2684_i2c_client;
static int KTD2684_write_reg(struct i2c_client *client, u8 reg, u8 val);
static int KTD2684_read_reg(struct i2c_client *client, u8 reg);
int writeReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
    KTD2684_write_reg(KTD2684_i2c_client, buf[0], buf[1]);
   return 0;
}

int FlashIc_Enable(void)
{
    flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_HIGH);
	PK_DBG("FlashIc_Enable!\n");
	return 0;
}
int FlashIc_Disable(void)
{
    flashlight_gpio_set(FLASHLIGHT_PIN_HWEN,STATE_LOW);
	PK_DBG("FlashIc_Disable!\n");
	return 0;
}
//add by qiangang@wind-mobi.com 20170928 begin
static struct proc_dir_entry *asus_proc_flash_file = NULL;
char *asus_string_data = "flash";
unsigned char torch_level = 0;
int flash_flag = 0; // 1:flash open; 2:flash closed
static ssize_t asus_flash_brightness_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	char *ptr = buf;
	printk("zzs asus_flash_brightness_read\n");

	if (*pos)
		return 0;
	
	ptr += sprintf(ptr,"torch_level = %d\n", torch_level);

	*pos += ptr - buf;

	return (ptr -buf);
}

static ssize_t asus_flash_brightness_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	char buf[6] = {0};
	static int reg_level;
	printk("zzs asus_flash_brightness_write begin\n");
	if (len >= 6)
	{
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}
	printk("zzs app send %s\n", buf);
	torch_level = simple_strtol(buf, NULL, 0);
	printk("zzs torch_level = %d\n", torch_level);
	reg_level = torch_level + 28;
	if(torch_level > 0 && torch_level < 100) {
		printk("zzs open torch\n");
		flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_HIGH);
		writeReg(KTD2684_REG_ENABLE, 0x80);//close
		writeReg(KTD2684_REG_TORCH_LED1, reg_level);
	//	//writeReg(KTD2684_REG_TORCH_LED2, reg_level);
      writeReg(KTD2684_REG_ENABLE, 0x89);//open high-0x89 low-0x8A  both-0x8B
		flash_flag = 1;
	} else if (torch_level == 0){
		printk("zzs close torch\n");
		writeReg(KTD2684_REG_ENABLE, 0x80);//close
		flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_LOW);
		flash_flag = 0;
	}
	return len;
}

static struct file_operations Asus_proc_asus_flash_brightness_ops =
{
	.owner = THIS_MODULE,
	.read  = asus_flash_brightness_read,
	.write = asus_flash_brightness_write,
};
//add by qiangang@wind-mobi.com 20170928 end
struct KTD2684_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct KTD2684_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct KTD2684_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int KTD2684_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct KTD2684_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

static int KTD2684_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct KTD2684_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}



/* ========================= */




static int KTD2684_chip_init(struct KTD2684_chip_data *chip)
{


	return 0;
}

static int KTD2684_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct KTD2684_chip_data *chip;
	struct KTD2684_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("KTD2684_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("KTD2684 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct KTD2684_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("KTD2684 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct KTD2684_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (KTD2684_chip_init(chip) < 0)
		goto err_chip_init;

	KTD2684_i2c_client = client;
	//add by qiangang@wind-mobi.com 20170928 begin
	asus_proc_flash_file = proc_create("driver/asus_flash_brightness", (S_IWUSR|S_IRUGO|S_IWUGO), NULL, &Asus_proc_asus_flash_brightness_ops);
	
	if(asus_proc_flash_file == NULL)
	{
		printk("zzs proc asus_proc_flash file create failed!\n");
	}
	//add by qiangang@wind-mobi.com 20170928 end
    // zhaozhensen@wind-mobi.com 20161219 begin
    flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_HIGH);
    chip_reg = KTD2684_read_reg(client, 0x0C);
	pr_err("zzs chip_reg = 0x%x\n", chip_reg);
	pr_debug("zzs chip_reg = 0x%x\n", chip_reg);
    if(chip_reg == 0x04)
    {
        pr_debug("zzs Flashlight IC is TLV61310\n"); // zhaozhensen@wind-mobi.com 20161219
        flashlight_vendor_id = 0;
    } else {
        pr_debug("zzs Flashlight IC is AW3648\n");
        flashlight_vendor_id = 1;
    }
    // HWEN Low
    flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_LOW);
    // zhaozhensen@wind-mobi.com 20161219 end
	PK_DBG("KTD2684 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("KTD2684 probe is failed\n");
	return -ENODEV;
}

static int KTD2684_remove(struct i2c_client *client)
{
	struct KTD2684_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define KTD2684_NAME "leds-KTD2684"
static const struct i2c_device_id KTD2684_id[] = {
	{KTD2684_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id KTD2684_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver KTD2684_i2c_driver = {
	.driver = {
		   .name = KTD2684_NAME,
#ifdef CONFIG_OF
		   .of_match_table = KTD2684_of_match,
#endif
		   },
	.probe = KTD2684_probe,
	.remove = KTD2684_remove,
	.id_table = KTD2684_id,
};

static int __init KTD2684_init(void)
{
	PK_DBG("KTD2684_init\n");
	return i2c_add_driver(&KTD2684_i2c_driver);
}

static void __exit KTD2684_exit(void)
{
	i2c_del_driver(&KTD2684_i2c_driver);
}


module_init(KTD2684_init);
module_exit(KTD2684_exit);

MODULE_DESCRIPTION("Flash driver for KTD2684");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;

	val = KTD2684_read_reg(KTD2684_i2c_client, reg);
	return (int)val;
}

int FL_Enable(void)
{
	int buf[2];

	buf[0] = KTD2684_REG_ENABLE;
#if 0
	if (gDuty == -1)
		buf[1] = 0x00; // when "gDuty = -1", close led
	else if (gIsTorch[gDuty] == 1)
		buf[1] = 0x09;  // torch mode LED1 ENABLE LED2 DISABLE
	else
		buf[1] = 0x0D;  // flash mode LED1 ENABLE LED2 DISABLE
	writeReg(buf[0], buf[1]);
	PK_DBG(" FL_Enable line=%d\n", __LINE__);
#endif
        if (gDuty == -1) {
            buf[1] = 0x00; // when "gDuty = -1", close led
        } else if(gDuty <= 4) {
	    printk("torch mode\n");
            buf[1] = 0x09;  // torch mode LED1 ENABLE LED2 DISABLE
	} else {
	    printk("flash mode\n");
            buf[1] = 0x0D;  // flash mode LED1 ENABLE LED2 DISABLE
	}
        writeReg(buf[0], buf[1]);
        PK_DBG(" FL_Enable line=%d\n", __LINE__);

	return 0;
}



int FL_Disable(void)
{
	int buf[2];

	buf[0] = KTD2684_REG_ENABLE;
	buf[1] = 0x00;
	writeReg(buf[0], buf[1]);//close
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
	return 0;
}

int FL_dim_duty(int duty)
{
	int buf[2];
	gDuty = duty;
	if (gDuty > e_DutyNum)
		gDuty = e_DutyNum;
	else if (gDuty < 0)
		gDuty = -1;
	pr_debug("zzs chip_reg = 0x%x flashlight_vendor_id = %d gDuty = %d\n", chip_reg, flashlight_vendor_id, gDuty);
	if(flashlight_vendor_id == 0)
    	buf[1] = gLedDuty[gDuty];
	else
    	buf[1] = gLedDuty_aw[gDuty];
	if (gIsTorch[gDuty] == 1){
		buf[0] = KTD2684_REG_TORCH_LED1; //set torch duty
		writeReg(buf[0], buf[1]);
		//buf[0] = KTD2684_REG_TORCH_LED2; //set torch duty ONLY USE LED1
		//writeReg(buf[0], buf[1]);
	} else {
		buf[0] = KTD2684_REG_FLASH_LED1; //set flash duty
		writeReg(buf[0], buf[1]);
		//buf[0] = KTD2684_REG_FLASH_LED2; //set flash duty ONLY USE LED1
		//writeReg(buf[0], buf[1]);
	}

	//PK_DBG(" FL_dim_duty line=%d, gIsTorch[%d] = %d\n", __LINE__, gDuty, gLedDuty[gDuty]);
	return 0;
}




int FL_Init(void)
{
	int buf[2];
// zhaozhensen@wind-mobi.com 20161223 begin
	FlashIc_Enable();
    
	buf[0] = 0x01;
	buf[1] = 0x00;
	KTD2684_write_reg(KTD2684_i2c_client, buf[0], buf[1]);

	buf[0] = 0x08;
    if(flashlight_vendor_id == 1){
	    buf[1] = 0x1A;  // AW3644 600ms modified by lifengxiang@wind-mobi.com 20170629
    } else {
	    buf[1] = 0x1F;  // KDT2687 400ms
    }
	KTD2684_write_reg(KTD2684_i2c_client, buf[0], buf[1]);
	KTD2684_write_reg(KTD2684_i2c_client, 0x07, readReg(0x07)|0x01);
// zhaozhensen@wind-mobi.com 20161223 end
	PK_DBG(" FL_Init line=%d\n", __LINE__);
	return 0;
}


int FL_Uninit(void)
{
// zhaozhensen@wind-mobi.com 20161017 begin
	FlashIc_Disable();
// zhaozhensen@wind-mobi.com 20161017 end
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 2000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	PK_DBG
	    ("zzs constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);

	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		if(arg==0)
			g_timeOutTimeMs = arg;
		else{
			g_timeOutTimeMs=2000;
		}
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
EXPORT_SYMBOL(constantFlashlightInit);


/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
// zhaozhensen@wind-mobi.com 20170314 end
