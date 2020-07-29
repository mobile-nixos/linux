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
#include "tp_gesture.h"

#if FTS_GESTURE_EN
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define  KEY_GESTURE_DOUBLECLICK                KEY_F21   //KEY_POWER
#define  KEY_GESTURE_U                          KEY_U
#define  KEY_GESTURE_UP                         KEY_F21  
#define  KEY_GESTURE_DOWN                       KEY_DOWN
#define  KEY_GESTURE_LEFT                       KEY_LEFT
#define  KEY_GESTURE_RIGHT                      KEY_RIGHT
#define  KEY_GESTURE_O                          KEY_O
#define  KEY_GESTURE_E                          KEY_F15
#define  KEY_GESTURE_M                          KEY_M
#define  KEY_GESTURE_L                          KEY_L
#define  KEY_GESTURE_W                          KEY_F13
#define  KEY_GESTURE_S                          192   //KEY_F14
#define  KEY_GESTURE_V                          186     //KEY_F18
#define  KEY_GESTURE_C                          193  //KEY_F19
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

static char gesture_mode_tmp[6] = {0};          // wangbing@wind-mobi.com 20170503

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct fts_gesture_st {
    u8 header[FTS_GESTRUE_POINTS_HEADER];
    u16 coordinate_x[FTS_GESTRUE_POINTS];
    u16 coordinate_y[FTS_GESTRUE_POINTS];
    u8 mode;
    u8 active;
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
static ssize_t fts_gesture_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    int count;
    u8 val;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    mutex_lock(&fts_input_dev_onsell->mutex);
    fts_i2c_read_reg_onsell(client, FTS_REG_GESTURE_EN, &val);
    count =
        sprintf(buf, "Gesture Mode: %s\n",
                fts_gesture_data.mode ? "On" : "Off");
    count += sprintf(buf + count, "Reg(0xD0) = %d\n", val);
    mutex_unlock(&fts_input_dev_onsell->mutex);

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
    mutex_lock(&fts_input_dev_onsell->mutex);
    val = simple_strtoul(buf, NULL, 10);
    if (val == 1)
        fts_gesture_data.mode = ENABLE;
    else
        fts_gesture_data.mode = DISABLE;
    mutex_unlock(&fts_input_dev_onsell->mutex);

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

    mutex_lock(&fts_input_dev_onsell->mutex);
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
    mutex_unlock(&fts_input_dev_onsell->mutex);

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
    if (fts_gesture_data.active) {
        fts_i2c_write_reg_onsell(client, FTS_REG_GESTURE_EN, ENABLE);
        fts_i2c_write_reg_onsell(fts_i2c_client_onsell, 0xD1, 0xff);
        fts_i2c_write_reg_onsell(fts_i2c_client_onsell, 0xD2, 0xff);
        fts_i2c_write_reg_onsell(fts_i2c_client_onsell, 0xD5, 0xff);
        fts_i2c_write_reg_onsell(fts_i2c_client_onsell, 0xD6, 0xff);
        fts_i2c_write_reg_onsell(fts_i2c_client_onsell, 0xD7, 0xff);
        fts_i2c_write_reg_onsell(fts_i2c_client_onsell, 0xD8, 0xff);
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
    //ft_touch_proc_deinit();
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
    int gesture = -1;
    printk("ft3427 gesture_id=0x%x\n", gesture_id);
    FTS_FUNC_ENTER();
    switch (gesture_id) 
    {
    case GESTURE_LEFT:
        gesture = KEY_GESTURE_LEFT;
        break;
    case GESTURE_RIGHT:
        gesture = KEY_GESTURE_RIGHT;
        break;
    case GESTURE_UP:
        gesture = KEY_GESTURE_UP;
        break;
    case GESTURE_DOWN:
        gesture = KEY_GESTURE_DOWN;
        break;
    case GESTURE_DOUBLECLICK:
        gesture = KEY_GESTURE_DOUBLECLICK;
        break;
    case GESTURE_O:
        gesture = KEY_GESTURE_O;
        break;
    case GESTURE_W:
        gesture = KEY_GESTURE_W;
        break;
    case GESTURE_M:
        gesture = KEY_GESTURE_M;
        break;
    case GESTURE_E:
        gesture = KEY_GESTURE_E;
        break;
    case GESTURE_L:
        gesture = KEY_GESTURE_L;
        break;
    case GESTURE_C:
        gesture = KEY_GESTURE_C;
        break;
     case GESTURE_S:
        gesture = KEY_GESTURE_S;
        break;
    case GESTURE_Z:
        gesture = KEY_GESTURE_Z;
        break;
    case GESTURE_V:
        gesture = KEY_GESTURE_V;
        break;
    default:
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
        ret = fts_i2c_read_onsell(client, buf, 1, buf, read_bytes);
    }
    else {
        ret = fts_i2c_read_onsell(client, buf, 1, buf, I2C_BUFFER_LENGTH_MAXINUM);
        remain_bytes = read_bytes - I2C_BUFFER_LENGTH_MAXINUM;
        for (i = 1; remain_bytes > 0; i++) {
            if (remain_bytes <= I2C_BUFFER_LENGTH_MAXINUM)
                ret =
                    fts_i2c_read_onsell(client, buf, 0,
                                 buf + I2C_BUFFER_LENGTH_MAXINUM * i,
                                 remain_bytes);
            else
                ret =
                    fts_i2c_read_onsell(client, buf, 0,
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
    memset(fts_gesture_data.header, 0, FTS_GESTRUE_POINTS_HEADER);
    memset(fts_gesture_data.coordinate_x, 0, FTS_GESTRUE_POINTS * sizeof(u16));
    memset(fts_gesture_data.coordinate_y, 0, FTS_GESTRUE_POINTS * sizeof(u16));

    buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
    ret = fts_i2c_read_onsell(client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
    if (ret < 0) {
        FTS_ERROR("[GESTURE]Read gesture header data failed!!");
        FTS_FUNC_EXIT();
        return ret;
    }
    
    /* FW recognize gesture */
    if (chip_types_onsell.chip_idh == 0x54 || chip_types_onsell.chip_idh == 0x58 || chip_types_onsell.chip_idh == 0x86 || chip_types_onsell.chip_idh == 0x87 || chip_types_onsell.chip_idh == 0x64)
    {
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
        fts_check_gesture(fts_input_dev_onsell, gestrue_id);
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
        fts_check_gesture(fts_input_dev_onsell, gestrue_id);
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
    fts_check_gesture(fts_input_dev_onsell, gestrue_id);
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

// wangbing@wind-mobi.com 20170503 begin
#if FTS_GESTURE_EN
static void gesture_mode_exchange(void)
{
    /* gesture_mode_tmp[0] will be write into the ft3427 0xd1 register */
    gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_LEFT_DIRECT_FLAG ) ? (gesture_mode_tmp[0] | (1<<0) ) : (gesture_mode_tmp[0] & (~(1<<0)));
    gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_RIGHT_DIRECT_FLAG) ? (gesture_mode_tmp[0] | (1<<1) ) : (gesture_mode_tmp[0] & (~(1<<1)));
    gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_UP_DIRECT_FLAG   ) ? (gesture_mode_tmp[0] | (1<<2) ) : (gesture_mode_tmp[0] & (~(1<<2)));
    gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_DOWN_DIRECT_FLAG ) ? (gesture_mode_tmp[0] | (1<<3) ) : (gesture_mode_tmp[0] & (~(1<<3)));
    gesture_mode_tmp[0] = (g_GestureWakeupMode_wind & GESTURE_MODE_DOUBLE_CLICK_FLAG) ? (gesture_mode_tmp[0] | (1<<4) ) : (gesture_mode_tmp[0] & (~(1<<4)));
    
    /* gesture_mode_tmp[1] will be write into the ft3427 0xd2 register */
    // gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_O_FLAG           ) ? (gesture_mode_tmp[1] | (1<<0) ) : (gesture_mode_tmp[1] & (~(1<<0)));
    gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_W_FLAG           ) ? (gesture_mode_tmp[1] | (1<<1) ) : (gesture_mode_tmp[1] & (~(1<<1)));
    // gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_M_FLAG           ) ? (gesture_mode_tmp[1] | (1<<2) ) : (gesture_mode_tmp[1] & (~(1<<2)));
    gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_E_FLAG           ) ? (gesture_mode_tmp[1] | (1<<3) ) : (gesture_mode_tmp[1] & (~(1<<3)));
    gesture_mode_tmp[1] = (g_GestureWakeupMode_wind & GESTURE_MODE_C_FLAG           ) ? (gesture_mode_tmp[1] | (1<<4) ) : (gesture_mode_tmp[1] & (~(1<<4)));
    
    /* gesture_mode_tmp[2] will be write into the ft3427 0xd5 register */
    // gesture_mode_tmp[2] = (g_GestureWakeupMode_wind & GESTURE_MODE_L_FLAG           ) ? (gesture_mode_tmp[2] | (1<<4) ) : (gesture_mode_tmp[2] & (~(1<<4)));
    gesture_mode_tmp[2] = (g_GestureWakeupMode_wind & GESTURE_MODE_S_FLAG           ) ? (gesture_mode_tmp[2] | (1<<6) ) : (gesture_mode_tmp[2] & (~(1<<6)));
    
    /* gesture_mode_tmp[3] will be write into the ft3427 0xd6 register */
    gesture_mode_tmp[3] = (g_GestureWakeupMode_wind & GESTURE_MODE_V_FLAG           ) ? (gesture_mode_tmp[3] | (1<<4) ) : (gesture_mode_tmp[3] & (~(1<<4)));
    
    /* gesture_mode_tmp[4] will be write into the ft3427 0xd7 register */
    gesture_mode_tmp[4] = (g_GestureWakeupMode_wind & GESTURE_MODE_Z_FLAG           ) ? (gesture_mode_tmp[4] | (1<<5) ) : (gesture_mode_tmp[4] & (~(1<<5)));
    
    printk("[tpd_wind][gesture] %s , g_GestureWakeupMode_wind = 0x%x , gesture_mode_tmp[0]=0x%x [1]=0x%x [2]=0x%x [3]=0x%x [4]=0x%x [5]=0x%x \n", __func__, g_GestureWakeupMode_wind, gesture_mode_tmp[0], gesture_mode_tmp[1], gesture_mode_tmp[2], gesture_mode_tmp[3], gesture_mode_tmp[4], gesture_mode_tmp[5]);
}
#endif

int fts_gesture_suspend(struct i2c_client *i2c_client)
{
    int i = 0;
    u8 state;

    FTS_FUNC_ENTER();
        
    /* gesture not enable, return immediately */
    if( 0x00000000 != g_GestureWakeupMode_wind )
    {
        gesture_mode_exchange();

        do
        {
            fts_i2c_write_reg_onsell(i2c_client, 0xd0, 0x01);
            fts_i2c_write_reg_onsell(i2c_client, 0xd1, gesture_mode_tmp[0]);
            fts_i2c_write_reg_onsell(i2c_client, 0xd2, gesture_mode_tmp[1]);
            fts_i2c_write_reg_onsell(i2c_client, 0xd5, gesture_mode_tmp[2]);
            fts_i2c_write_reg_onsell(i2c_client, 0xd6, gesture_mode_tmp[3]);
            fts_i2c_write_reg_onsell(i2c_client, 0xd7, gesture_mode_tmp[4]);
            fts_i2c_write_reg_onsell(i2c_client, 0xd8, gesture_mode_tmp[5]);
            msleep(10);
            i++;
            printk("tpd write gesture register time = %d", i);
            fts_i2c_read_reg_onsell(i2c_client, 0xd0, &state);
        } while( (state != 1) && (i <= 10) );
            
        if(state == 1)
        {
            TPD_DMESG("TPD gesture write 0x01\n");
            return 0;
        }
        
        if(i > 10)
        {
            TPD_DMESG("TPD gesture write 0x01 to d0 fail \n");
            return -1;
        }
    }
    else
    {
        /* close all gesture */
        fts_i2c_write_reg_onsell(i2c_client, 0xd0, 0x0);
		return -1;
    }

    fts_gesture_data.active = 1;
    FTS_DEBUG("[GESTURE]Enter into gesture(suspend) successfully!");
    FTS_FUNC_EXIT();
    return 0;
}
// wangbing@wind-mobi.com 20170503 end

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

    fts_gesture_data.active = 0;
	
	if( 0x00000000 != g_GestureWakeupMode_wind )
    {
    	/* Before read/write TP register, need wait TP to valid */
    	fts_wait_tp_to_valid_onsell(client);
    	for (i = 0; i < 5; i++) {
       	 	fts_i2c_write_reg_onsell(client, FTS_REG_GESTURE_EN, 0x00);
        	msleep(1);
        	fts_i2c_read_reg_onsell(client, FTS_REG_GESTURE_EN, &state);
        	if (state == 0)
           		break;
    	}
    	if (i >= 5) {
        	FTS_ERROR("[GESTURE]Clear gesture(resume) failed!\n");
    	}
		printk("[GESTURE]gesture (resume) successful \n");
		return 0;
	}else{
		printk("[GESTURE]gesture (resume) gesture mode not open \n");
		return -1;
	}
    FTS_FUNC_EXIT();
    return 0;
}
#endif
