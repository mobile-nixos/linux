/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2016, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#if FTS_GESTURE_EN
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define  KEY_GESTURE_U                          KEY_U
#define  KEY_GESTURE_UP                         KEY_F20  
#define  KEY_GESTURE_DOWN                       KEY_DOWN
#define  KEY_GESTURE_LEFT                       KEY_LEFT
#define  KEY_GESTURE_RIGHT                      KEY_RIGHT
#define  KEY_GESTURE_O                          KEY_O
#define  KEY_GESTURE_E                          KEY_F15
#define  KEY_GESTURE_M                          KEY_M
#define  KEY_GESTURE_L                          KEY_L
#define  KEY_GESTURE_W                          KEY_F13
#define  KEY_GESTURE_S                          KEY_F14
#define  KEY_GESTURE_V                          KEY_F18
#define  KEY_GESTURE_C                          KEY_F19
#define  KEY_GESTURE_Z                          KEY_F17

#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x65
#define GESTURE_C                               0x34
#define FTS_GESTRUE_POINTS                      255
#define FTS_GESTRUE_POINTS_HEADER               8
#define FTS_GESTURE_OUTPUT_ADRESS               0xD3

//hebiao@wind-mobi.com 20160907 begin
#if 1
#define FT_PROC_TOUCH_FOLDER "android_touch"
#define FT_PROC_SMWP_FILE "SMWP"
#define FT_PROC_GESTURE_FILE "GESTURE"
static struct proc_dir_entry *ft_proc_GESTURE_file = NULL;
static struct proc_dir_entry *ft_touch_proc_dir = NULL;
static struct proc_dir_entry *ft_proc_SMWP_file = NULL;
static int GESTURE_SEND_FLAG = 0;
extern int sleep_flag;
#endif
//hebiao@wind-mobi.com 20160907 end
/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct fts_gesture_st {
    u8 header[FTS_GESTRUE_POINTS_HEADER];
    u16 coordinate_x[FTS_GESTRUE_POINTS];
    u16 coordinate_y[FTS_GESTRUE_POINTS];
    u8 mode;
    u8 active;
//hebiao@wind-mobi.com 20160101 begin
	//struct wake_lock ts_SMWP_wake_lock;
	uint8_t gesture_cust_en[16];
//hebiao@wind-mobi.com 20160101 end
	/* 1-enter into gesture(suspend) 0-gesture disable or LCD on */
};


/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(struct device *dev,
                                struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf,
                                 size_t count);
static ssize_t fts_gesture_buf_show(struct device *dev,
                                    struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_buf_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count);

/* sysfs gesture node
 *   read example: cat  fts_gesture_mode        ---read gesture mode
 *   write example:echo 01 > fts_gesture_mode   ---write gesture mode to 01
 *
 */
static DEVICE_ATTR(fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show,
                   fts_gesture_store);
/*
 *   read example: cat  fts_gesture_buf        ---read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR, fts_gesture_buf_show,
                   fts_gesture_buf_store);
static struct attribute *fts_gesture_mode_attrs[] = {

    &dev_attr_fts_gesture_mode.attr,
    &dev_attr_fts_gesture_buf.attr,
    NULL,
};

static struct attribute_group fts_gesture_group = {
    .attrs = fts_gesture_mode_attrs,
};

/************************************************************************
* Name: fts_gesture_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return:
***********************************************************************/
//hebiao@wind-mobi.com 20160101 begin
static ssize_t ft_SMWP_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

	if(!GESTURE_SEND_FLAG)
	{
		ret = snprintf(buf, PAGE_SIZE, "%d\n", fts_gesture_data.mode);
		GESTURE_SEND_FLAG=1;
	}
	else
		GESTURE_SEND_FLAG=0;

	return ret;
}

static ssize_t ft_SMWP_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	char buf[80] = {0};
	int i;

	if (sleep_flag == 1) {
		printk("TP is suspend,can not write gesture\n");
		return len;
	}
		
	if (len >= 80)
	{
		printk("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	if(buf[0] == '0') {
		fts_gesture_data.mode= 0;
	} else if(buf[0] == '1') {
		fts_gesture_data.mode= 1;
	} else {
		return -EINVAL;
	}
		
	if (fts_gesture_data.mode== 0){
		for (i=0;i<8;i++){
			fts_gesture_data.gesture_cust_en[i]= 0;
		}
	} 
		printk("%s: SMART_WAKEUP_enable = %d.\n", __func__, fts_gesture_data.mode);
	
		return len;
}

static struct file_operations ft_proc_SMWP_ops =
{
	.owner = THIS_MODULE,
	.read = ft_SMWP_read,
	.write = ft_SMWP_write,
};

static ssize_t ft_GESTURE_read(struct file *file, char *buf,
	size_t len, loff_t *pos)
{
	size_t ret = 0;

	if(GESTURE_SEND_FLAG<16)
	{
		ret = sprintf(buf, "ges_en[%d]=%d \n",GESTURE_SEND_FLAG ,fts_gesture_data.gesture_cust_en[GESTURE_SEND_FLAG]);
		GESTURE_SEND_FLAG++;
	}
	else
	{
		GESTURE_SEND_FLAG = 0;
		ret = 0;
	}
	return ret;
}

static ssize_t ft_GESTURE_write(struct file *file, const char *buff,
	size_t len, loff_t *pos)
{
	int i =0;
	char buf[80] = {0};
	
	if (len >= 80)
	{
		printk("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
	{
		return -EFAULT;
	}

	for (i=0;i<16;i++)
		{
			if (buf[i] == '0')
	    			fts_gesture_data.gesture_cust_en[i]= 0;
			else if (buf[i] == '1')
	    			fts_gesture_data.gesture_cust_en[i]= 1;
			else
				fts_gesture_data.gesture_cust_en[i]= 0;
			printk("gesture en[%d]=%d \n", i, fts_gesture_data.gesture_cust_en[i]);
		}
	return len;
}

static struct file_operations ft_proc_Gesture_ops =
{
	.owner = THIS_MODULE,
	.read = ft_GESTURE_read,
	.write = ft_GESTURE_write,
};

static int ft_touch_proc_init(void)
{
	ft_touch_proc_dir = proc_mkdir(FT_PROC_TOUCH_FOLDER, NULL);
	if (ft_touch_proc_dir == NULL)
	{

		printk(" %s: ft_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}


	ft_proc_SMWP_file = proc_create(FT_PROC_SMWP_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), 
		ft_touch_proc_dir, &ft_proc_SMWP_ops);
	if(ft_proc_SMWP_file == NULL)
	{
		printk(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_0;
	}
	ft_proc_GESTURE_file = proc_create(FT_PROC_GESTURE_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), 
		ft_touch_proc_dir, &ft_proc_Gesture_ops);
	if(ft_proc_GESTURE_file == NULL)
	{
		printk(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_1;
	}

	return 0 ;

	
	fail_0: remove_proc_entry( FT_PROC_SMWP_FILE, ft_touch_proc_dir );
	fail_1: remove_proc_entry( FT_PROC_GESTURE_FILE, ft_touch_proc_dir );	
	return -ENOMEM;
}

static void ft_touch_proc_deinit(void)
{

		remove_proc_entry( FT_PROC_GESTURE_FILE, ft_touch_proc_dir );
		remove_proc_entry( FT_PROC_SMWP_FILE, ft_touch_proc_dir );
}

//hebia@wind-mobi.com 20161228 end

static ssize_t fts_gesture_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    int count;
    u8 val;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    mutex_lock(&fts_input_dev->mutex);
    fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &val);
    count =
        sprintf(buf, "Gesture Mode: %s\n",
                fts_gesture_data.mode ? "On" : "Off");
    count += sprintf(buf + count, "Reg(0xD0) = %d\n", val);
    mutex_unlock(&fts_input_dev->mutex);

    return count;
}

/************************************************************************
* Name: fts_gesture_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return:
***********************************************************************/
static ssize_t fts_gesture_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf,
                                 size_t count)
{
    unsigned long val;

    FTS_DEBUG("fts_gesture_store buf:%s", buf);
    mutex_lock(&fts_input_dev->mutex);
    val = simple_strtoul(buf, NULL, 10);
    if (val == 1)
        fts_gesture_data.mode = ENABLE;
    else
        fts_gesture_data.mode = DISABLE;
    mutex_unlock(&fts_input_dev->mutex);

    return count;
}

/************************************************************************
* Name: fts_gesture_buf_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    int count;
    int i = 0;

    mutex_lock(&fts_input_dev->mutex);
    count =
        snprintf(buf, PAGE_SIZE, "Gesture ID: 0x%x\n",
                 fts_gesture_data.header[0]);
    count +=
        snprintf(buf + count, PAGE_SIZE, "Gesture PointNum: %d\n",
                 fts_gesture_data.header[1]);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Point Buf:\n");
    for (i = 0; i < fts_gesture_data.header[1]; i++) {
        count +=
            snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i,
                     fts_gesture_data.coordinate_x[i],
                     fts_gesture_data.coordinate_y[i]);
        if ((i + 1) % 4 == 0)
            count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&fts_input_dev->mutex);

    return count;
}

/************************************************************************
* Name: fts_gesture_buf_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    /* place holder for future use */
    return -EPERM;
}

/*****************************************************************************
*   Name: fts_create_gesture_sysfs
*  Brief:
*  Input:
* Output: None
* Return: 0-success or error
*****************************************************************************/
int fts_create_gesture_sysfs(struct i2c_client *client)
{
    int ret = 0;

    ret = sysfs_create_group(&client->dev.kobj, &fts_gesture_group);
    if (ret != 0) {
        FTS_DEBUG("[GESTURE]fts_gesture_mode_group(sysfs) create failed!");
        sysfs_remove_group(&client->dev.kobj, &fts_gesture_group);
        return ret;
    }
    return 0;
}

/*****************************************************************************
*   Name: fts_gesture_recovery
*  Brief: recovery gesture state when reset
*  Input:
* Output: None
* Return:
*****************************************************************************/
void fts_gesture_recovery(struct i2c_client *client)
{
    if (fts_gesture_data.mode && fts_gesture_data.active) {
        fts_i2c_write_reg(client, FTS_REG_GESTURE_EN, ENABLE);
        fts_i2c_write_reg(fts_i2c_client, 0xD1, 0xff);
        fts_i2c_write_reg(fts_i2c_client, 0xD2, 0xff);
        fts_i2c_write_reg(fts_i2c_client, 0xD5, 0xff);
        fts_i2c_write_reg(fts_i2c_client, 0xD6, 0xff);
        fts_i2c_write_reg(fts_i2c_client, 0xD7, 0xff);
        fts_i2c_write_reg(fts_i2c_client, 0xD8, 0xff);
    }
}

/*****************************************************************************
*   Name: fts_gesture_init
*  Brief:
*  Input:
* Output: None
* Return: None
*****************************************************************************/
int fts_gesture_init(struct input_dev *input_dev, struct i2c_client *client)
{
    FTS_FUNC_ENTER();
    input_set_capability(input_dev, EV_KEY, KEY_POWER);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

    __set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
    __set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
    __set_bit(KEY_GESTURE_UP, input_dev->keybit);
    __set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
    __set_bit(KEY_GESTURE_U, input_dev->keybit);
    __set_bit(KEY_GESTURE_O, input_dev->keybit);
    __set_bit(KEY_GESTURE_E, input_dev->keybit);
    __set_bit(KEY_GESTURE_M, input_dev->keybit);
    __set_bit(KEY_GESTURE_W, input_dev->keybit);
    __set_bit(KEY_GESTURE_L, input_dev->keybit);
    __set_bit(KEY_GESTURE_S, input_dev->keybit);
    __set_bit(KEY_GESTURE_V, input_dev->keybit);
    __set_bit(KEY_GESTURE_C, input_dev->keybit);
    __set_bit(KEY_GESTURE_Z, input_dev->keybit);

    fts_create_gesture_sysfs(client);

    ft_touch_proc_init();
    fts_gesture_data.mode = 0;
    fts_gesture_data.active = 0;

    return 0;
}

/************************************************************************
* Name: fts_gesture_exit
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_gesture_exit(struct i2c_client *client)
{
    ft_touch_proc_deinit();
    sysfs_remove_group(&client->dev.kobj, &fts_gesture_group);
    return 0;
}

/*****************************************************************************
*   Name: fts_check_gesture
*  Brief:
*  Input:
* Output: None
* Return: None
*****************************************************************************/
static void fts_check_gesture(struct input_dev *input_dev, int gesture_id)
{
   // char *envp[2];
    int gesture = -1;
    printk("ft3427 gesture_id=0x%x\n", gesture_id);
    FTS_FUNC_ENTER();
    switch (gesture_id) {
    case GESTURE_LEFT:
			// envp[0] = "GESTURE=LEFT";
			gesture = KEY_GESTURE_LEFT;
        break;
    case GESTURE_RIGHT:
			//  envp[0] = "GESTURE=RIGHT";
			gesture = KEY_GESTURE_RIGHT;
        break;
    case GESTURE_UP:
        envp[0] = "GESTURE=UP";
        if (fts_gesture_data.gesture_cust_en[0] == 1) {
       		gesture = KEY_GESTURE_UP;
       	}
        break;
    case GESTURE_DOWN:
		// envp[0] = "GESTURE=DOWN";
			gesture = KEY_GESTURE_DOWN;

        break;
    case GESTURE_DOUBLECLICK:
		if (fts_gesture_data.gesture_cust_en[1] == 1) {
		 envp[0] = "GESTURE=DOUBLE_CLICK";
			gesture = KEY_POWER;
		}
        break;
    case GESTURE_O:
		// envp[0] = "GESTURE=O";
			gesture = KEY_GESTURE_O;
        break;
    case GESTURE_W:
		//if (fts_gesture_data.gesture_cust_en[2] == 1) {
		// envp[0] = "GESTURE=W";
		gesture = KEY_GESTURE_W;
        break;
    case GESTURE_M:
			// envp[0] = "GESTURE=M";
       		gesture = KEY_GESTURE_M;
        break;
    case GESTURE_E:
		if (fts_gesture_data.gesture_cust_en[3] == 1) {
			// envp[0] = "GESTURE=E";
        	gesture = KEY_GESTURE_E;
		}
        break;
    case GESTURE_L:
			// envp[0] = "GESTURE=L";
			gesture = KEY_GESTURE_L;
		break;
    case GESTURE_C:
		if (fts_gesture_data.gesture_cust_en[4] == 1) {
			// envp[0] = "GESTURE=C";
        	gesture = KEY_GESTURE_C;
		}
        break;
     case GESTURE_S:
		if (fts_gesture_data.gesture_cust_en[5] == 1) {
			// envp[0] = "GESTURE=S";
     		gesture = KEY_GESTURE_S;
		}
        break;
      case GESTURE_Z:
	  	if (fts_gesture_data.gesture_cust_en[6] == 1) {
			//  envp[0] = "GESTURE=Z";
       		gesture = KEY_GESTURE_Z;
	  	}
        break;
       case GESTURE_V:
	   	if (fts_gesture_data.gesture_cust_en[7] == 1) {
			// envp[0] = "GESTURE=V";
       		gesture = KEY_GESTURE_V;
	   	}
        break;
    default:
      //  envp[0] = "GESTURE=NONE";
        gesture = -1;
        break;
    }
    FTS_DEBUG("gesture=%d\n", gesture);
    /* report event key */
    if (gesture != -1)
       {
      		input_report_key(input_dev, gesture, 1);
       		input_sync(input_dev);
       		input_report_key(input_dev, gesture, 0);
       		input_sync(input_dev);
       		gesture = -1;
       } 

    //envp[1] = NULL;
    //kobject_uevent_env(&tpd->tpd_dev->kobj, KOBJ_CHANGE, envp);
   // sysfs_notify(&tpd->tpd_dev->kobj, NULL, "GESTURE_ID");

}

/************************************************************************
*   Name: fts_gesture_readdata
* Brief: read data from TP register
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
static int fts_gesture_read_buffer(struct i2c_client *client, u8 * buf,
                                   int read_bytes)
{
    int remain_bytes;
    int ret;
    int i;

    if (read_bytes <= I2C_BUFFER_LENGTH_MAXINUM) {
        ret = fts_i2c_read(client, buf, 1, buf, read_bytes);
    }
    else {
        ret = fts_i2c_read(client, buf, 1, buf, I2C_BUFFER_LENGTH_MAXINUM);
        remain_bytes = read_bytes - I2C_BUFFER_LENGTH_MAXINUM;
        for (i = 1; remain_bytes > 0; i++) {
            if (remain_bytes <= I2C_BUFFER_LENGTH_MAXINUM)
                ret =
                    fts_i2c_read(client, buf, 0,
                                 buf + I2C_BUFFER_LENGTH_MAXINUM * i,
                                 remain_bytes);
            else
                ret =
                    fts_i2c_read(client, buf, 0,
                                 buf + I2C_BUFFER_LENGTH_MAXINUM * i,
                                 I2C_BUFFER_LENGTH_MAXINUM);
            remain_bytes -= I2C_BUFFER_LENGTH_MAXINUM * i;
        }
    }

    return ret;
}

/************************************************************************
*   Name: fts_gesture_readdata
* Brief: read data from TP register
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
int fts_gesture_readdata(struct i2c_client *client)
{
    u8 buf[FTS_GESTRUE_POINTS * 4] = { 0 };
    int ret = -1;
    int i = 0;
    int gestrue_id = 0;
    int read_bytes = 0;
    u8 pointnum;

    FTS_FUNC_ENTER();
    /* init variable before read gesture point */
   // printk("ft3427 gesture read_data\n");
    memset(fts_gesture_data.header, 0, FTS_GESTRUE_POINTS_HEADER);
    memset(fts_gesture_data.coordinate_x, 0, FTS_GESTRUE_POINTS * sizeof(u16));
    memset(fts_gesture_data.coordinate_y, 0, FTS_GESTRUE_POINTS * sizeof(u16));

    buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
    ret = fts_i2c_read(client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
    if (ret < 0) {
        FTS_ERROR("[GESTURE]Read gesture header data failed!!");
        FTS_FUNC_EXIT();
        return ret;
    }

    /* FW recognize gesture */
    if (chip_types.chip_idh == 0x54 || chip_types.chip_idh == 0x58
        || chip_types.chip_idh == 0x86 || chip_types.chip_idh == 0x87
        || chip_types.chip_idh == 0x64) {
        memcpy(fts_gesture_data.header, buf, FTS_GESTRUE_POINTS_HEADER);
        gestrue_id = buf[0];
        pointnum = buf[1];
        read_bytes = ((int)pointnum) * 4 + 2;
        buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
        FTS_DEBUG("[GESTURE]PointNum=%d", pointnum);
        ret = fts_gesture_read_buffer(client, buf, read_bytes);
        if (ret < 0) {
            FTS_ERROR("[GESTURE]Read gesture touch data failed!!");
            FTS_FUNC_EXIT();
            return ret;
        }
	//printk("ft3427 gesture_id=0x%x\n", gestrue_id);
        fts_check_gesture(fts_input_dev, gestrue_id);
        for (i = 0; i < pointnum; i++) {
            fts_gesture_data.coordinate_x[i] =
                (((s16) buf[0 + (4 * i + 2)]) & 0x0F) << 8 |
                (((s16) buf[1 + (4 * i + 2)]) & 0xFF);
            fts_gesture_data.coordinate_y[i] =
                (((s16) buf[2 + (4 * i + 2)]) & 0x0F) << 8 |
                (((s16) buf[3 + (4 * i + 2)]) & 0xFF);
        }
        FTS_FUNC_EXIT();
        return 0;
    }
    /* other IC's gestrue in driver */
    if (0x24 == buf[0]) {
        gestrue_id = 0x24;
        fts_check_gesture(fts_input_dev, gestrue_id);
        FTS_DEBUG("[GESTURE]%d check_gesture gestrue_id", gestrue_id);
        FTS_FUNC_EXIT();
        return -1;
    }

    /* Host Driver recognize gesture */
    pointnum = buf[1];
    read_bytes = ((int)pointnum) * 4 + 2;
    buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
    ret = fts_gesture_read_buffer(client, buf, read_bytes);
    if (ret < 0) {
        FTS_ERROR
            ("[GESTURE]Driver recognize gesture - Read gesture touch data failed!!");
        FTS_FUNC_EXIT();
        return ret;
    }

    /*
     * Host Driver recognize gesture, need gesture lib.a
     * Not use now for compatibility
     gestrue_id = fetch_object_sample(buf, pointnum);
     */
    gestrue_id = 0x24;
    fts_check_gesture(fts_input_dev, gestrue_id);
    FTS_DEBUG("[GESTURE]%d read gestrue_id", gestrue_id);

    for (i = 0; i < pointnum; i++) {
        fts_gesture_data.coordinate_x[i] =
            (((s16) buf[0 + (4 * i + 8)]) & 0x0F) << 8 |
            (((s16) buf[1 + (4 * i + 8)]) & 0xFF);
        fts_gesture_data.coordinate_y[i] =
            (((s16) buf[2 + (4 * i + 8)]) & 0x0F) << 8 |
            (((s16) buf[3 + (4 * i + 8)]) & 0xFF);
    }
    FTS_FUNC_EXIT();
    return -1;
}

/*****************************************************************************
*   Name: fts_gesture_suspend
*  Brief:
*  Input:
* Output: None
* Return: None
*****************************************************************************/
int fts_gesture_suspend(struct i2c_client *i2c_client)
{
    int i;
    u8 state;

    FTS_FUNC_ENTER();

    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == 0)
        return -EPERM;

    for (i = 0; i < 5; i++) {
        fts_i2c_write_reg(i2c_client, FTS_REG_GESTURE_EN, 0x01);
        fts_i2c_write_reg(i2c_client, 0xd1, 0xff);
        fts_i2c_write_reg(i2c_client, 0xd2, 0xff);
        fts_i2c_write_reg(i2c_client, 0xd5, 0xff);
        fts_i2c_write_reg(i2c_client, 0xd6, 0xff);
        fts_i2c_write_reg(i2c_client, 0xd7, 0xff);
        fts_i2c_write_reg(i2c_client, 0xd8, 0xff);

        fts_i2c_read_reg(i2c_client, FTS_REG_GESTURE_EN, &state);
        if (state == 1)
            break;
        else
            msleep(1);
    }

    if (i >= 5) {
        FTS_ERROR("[GESTURE]Enter into gesture(suspend) failed!\n");
        FTS_FUNC_EXIT();
        return -1;
    }

    fts_gesture_data.active = 1;
    FTS_DEBUG("[GESTURE]Enter into gesture(suspend) successfully!");
    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*   Name: fts_gesture_resume
*  Brief:
*  Input:
* Output: None
* Return: None
*****************************************************************************/
int fts_gesture_resume(struct i2c_client *client)
{
    int i;
    u8 state;
    FTS_FUNC_ENTER();

    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == 0)
        return -EPERM;

    fts_gesture_data.active = 0;
    /* Before read/write TP register, need wait TP to valid */
    fts_wait_tp_to_valid(client);
    for (i = 0; i < 5; i++) {
        fts_i2c_write_reg(client, FTS_REG_GESTURE_EN, 0x00);
        msleep(1);
        fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &state);
        if (state == 0)
            break;
    }

    if (i >= 5) {
        FTS_ERROR("[GESTURE]Clear gesture(resume) failed!\n");
    }

    FTS_FUNC_EXIT();
    return 0;
}
#endif
