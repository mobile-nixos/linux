/* drivers/i2c/chips/AFX133.c - AFX133 compass driver
 *
 * Copyright (C) 2013 VTC Technology Inc.
 * Author: Gary Huang <gary.huang@voltafield.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <hwmsen_helper.h>
#include <cust_mag.h>
#include <mag.h>

#include "afx133.h"


/* Add for auto detect feature */
static int afx133_local_init(void);
static int afx133_remove(void);
static int afx133_m_open_report_data(int en);
static int afx133_m_set_delay(u64 delay);
static int afx133_m_enable(int en);
static int afx133_o_open_report_data(int en);
static int afx133_o_set_delay(u64 delay);
static int afx133_o_enable(int en);
static int afx133_get_data_m(int *x,int *y, int *z,int *status);
static int afx133_get_data_o(int *x,int *y, int *z,int *status);

static int afx133_init_flag = -1;  //0:ok,,-1:fail

static struct mag_init_info afx133_init_info = {
        .name   = "afx133",	
        .init   = afx133_local_init,
        .uninit = afx133_remove,	
};

/*----------------------------------------------------------------------------*/
#define DEBUG 1
#define AFX133_DEV_NAME         "afx133"
#define DRIVER_VERSION          "1.0.9"
#define DRIVER_RELEASE          "20170303"
/*----------------------------------------------------------------------------*/
#define AFX133_DEFAULT_DELAY    40
#define AFX133_DELAY_MIN     		5
#define AFX133_DELAY_MAX     		100
#define AFX133_DELAY_SW_GYRO    10
/*----------------------------------------------------------------------------*/
#define MSE_TAG                 "MSENSOR"
#define MSE_FUN(f)              printk(MSE_TAG" %s\r\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)   printk(KERN_ERR MSE_TAG" %s %d : \r\n"fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)   printk(MSE_TAG fmt, ##args)
#define MSE_VER(fmt, args...)   ((void)0)
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

/* Maintain  cust info here */
struct mag_hw mag_cust;
/* For  driver get cust info */
static struct mag_hw *hw = &mag_cust;
/*----------------------------------------------------------------------------*/
static struct i2c_client *afx133_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id afx133_i2c_id[] = {{AFX133_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_afx133={ I2C_BOARD_INFO("afx133", AFX133_I2C_ADDRESS)};  //7-bit address
/*----------------------------------------------------------------------------*/
static int afx133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int afx133_i2c_remove(struct i2c_client *client);

static int afx133_suspend(struct i2c_client *client, pm_message_t msg) ;
static int afx133_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
typedef enum {
    VTC_TRC_DEBUG  = 0x01,
} AMI_TRC;
/*----------------------------------------------------------------------------*/
struct _afx133_data {
    rwlock_t lock;
    int mode;
    int rate;
    volatile int updated;
} afx133_data;
/*----------------------------------------------------------------------------*/
struct _afx133_mid_data {
    rwlock_t datalock;
    rwlock_t ctrllock;    
    int controldata[10];
    int pid;
    int offset[3];
    unsigned int debug;
    int ori[3];
    int mag[3];
    int acc[3];
#ifdef SUPPORT_SOFTGYRO_FUNCTION
    int gyr[3]; 
    int rov[3]; 
    int lin[3]; 
    int gra[3]; 
#endif
    int mag_status;
} afx133_mid_data;
/*----------------------------------------------------------------------------*/
struct afx133_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t layout;   
    atomic_t trace;
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,MSENSOR_AF7133"},
	{},
};
#endif
static struct i2c_driver afx133_i2c_driver = {
    .driver = {
     //   .owner = THIS_MODULE, 
        .name  = "MSENSOR_AF7133",
#ifdef CONFIG_OF
        .of_match_table = mag_of_match,
#endif
    },
	.probe      = afx133_i2c_probe,
	.remove     = afx133_i2c_remove,
	.suspend    = afx133_suspend,
	.resume     = afx133_resume,
	.id_table   = afx133_i2c_id,
};

static DEFINE_MUTEX(afx133_mutex);
#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/

//========================================================================
static void afx133_power(struct mag_hw *hw, unsigned int on)
{
}

static int VTC_i2c_Rx(struct i2c_client *client, char *rxData, int length)
{
        uint8_t retry;
        struct i2c_msg msgs[] = 
        {
            {
                    .addr = client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = rxData,
            },
            {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = length,
                    .buf = rxData,
            },
        };

        for (retry = 0; retry < 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msgs, 2) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry >= 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        } 
        else
                return 0;
}

static int VTC_i2c_Tx(struct i2c_client *client, char *txData, int length)
{
        int retry;
        struct i2c_msg msg[] = 
        {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = txData,
                },
        };

        for (retry = 0; retry <= 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msg, 1) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry > 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        }
        else
                return 0;
}
/*----------------------------------------------------------------------------*/
int afx133_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{

	if(i2c_flag == I2C_FLAG_READ)
	{
	  return (VTC_i2c_Rx(client, buf, count>>8));
	}
	else if(i2c_flag == I2C_FLAG_WRITE)
	{
	  return (VTC_i2c_Tx(client, buf, count));
  }
  
  return 0;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int afx133_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

/*----------------------------------------------------------------------------*/
static int afx133_GetOffset(void)
{
  if(afx133_mid_data.pid == AF9133_PID)
  {
    unsigned char databuf[10];
    int S_Data[3], SR_Data[3];
    int i,j;
    int loop=5;
    
 
    if(NULL == afx133_i2c_client)
    {
  	  return -1;
    }   
  	
  	for(i=0;i<3;i++)
      afx133_mid_data.offset[i] = 0;
  	
  	for(i=0;i<loop;i++)
  	{
  		// set data
      databuf[0] = REG_MODE;
      databuf[1] = 0x38; 
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
      databuf[0] = REG_MEASURE;
      databuf[1] = 0x01;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
      mdelay(5);
  	
      databuf[0] = REG_DATA;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
      S_Data[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
      S_Data[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
      S_Data[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);
  
      for(j=0;j<3;j++) S_Data[j] = (S_Data[j] > 32767) ? (S_Data[j] - 65536) : S_Data[j];
  
      // set/reset data
      databuf[0] = REG_MODE;
      databuf[1] = 0x3C; 
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
      databuf[0] = REG_MEASURE;
      databuf[1] = 0x01;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
      mdelay(5);
  	
      databuf[0] = REG_DATA;
      afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
      SR_Data[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
      SR_Data[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
      SR_Data[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);
  
      for(j=0;j<3;j++) SR_Data[j] = (SR_Data[j] > 32767) ? (SR_Data[j] - 65536) : SR_Data[j];
      	
      // offset data
      for(j=0;j<3;j++)
        afx133_mid_data.offset[j] += (S_Data[j] - SR_Data[j]) / loop;
    }   
  	
  	// back to set mode
    databuf[0] = REG_MODE;
    databuf[1] = 0x38; 
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
    mdelay(5);
  }	
  
  return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_Chipset_Init(int mode)
{
  u8 databuf[2];
  
  databuf[0] = REG_SW_RESET;
  databuf[1] = 0x81; 
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  mdelay(15);
  
  databuf[0] = REG_PCODE;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x101, I2C_FLAG_READ); 	
  
  if(AF8133J_PID != databuf[0] && AF9133_PID != databuf[0] && AF5133_PID != databuf[0])
  {
  	MSE_ERR("afx133 PCODE is incorrect: %d\n", databuf[0]);
#ifndef BYPASS_PCODE_CHECK_FOR_MTK_DRL
  	return -3;
#endif
  } 
  else
  {
    afx133_mid_data.pid = databuf[0];  
  	printk("%s chip id:%#x\n",__func__,databuf[0]);
  }
  
  databuf[0] = REG_I2C_CHECK;
  databuf[1] = 0x55; 
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE); 

  if(afx133_mid_data.pid != AF9133_PID)
  {
    databuf[0] = REG_MODE;
    databuf[1] = 0x3E; 
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;
    afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
    mdelay(5);
  }
  
  databuf[0] = REG_MODE;
  databuf[1] = 0x38; 
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  databuf[0] = REG_AVG;
  if(afx133_mid_data.pid == AF9133_PID)
  	databuf[1] = 0x36;
  else if(afx133_mid_data.pid == AF5133_PID)
  	databuf[1] = 0x36;
  else if(afx133_mid_data.pid == AF8133J_PID)
  	databuf[1] = 0x04;	
  else	 
  {
  	databuf[1] = 0x04;	
    MSE_ERR("afx133 PID is incorrect: %d\n", afx133_mid_data.pid);
  }
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  databuf[0] = REG_OSR;
  databuf[1] = 0x15;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  databuf[0] = REG_WAITING;
  databuf[1] = 0x66;     
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
     
  write_lock(&afx133_data.lock);
  afx133_data.mode = mode;
  write_unlock(&afx133_data.lock);
  
  return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_SetMode(int newmode)
{
	int mode;
    mode = 0;
	read_lock(&afx133_data.lock);
	mode = afx133_data.mode;
	read_unlock(&afx133_data.lock);           

	return afx133_Chipset_Init(newmode);
}
/*----------------------------------------------------------------------------*/
static int afx133_Write_Register(unsigned char reg, unsigned char value)
{     
	unsigned char databuf[10];  

	databuf[0] = reg;
  	databuf[1] = value;
  	afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_Read_Regiser(unsigned char reg, char* value)
{     
	unsigned char databuf[10];  
	databuf[0] = reg;  
	afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x101, I2C_FLAG_READ);  
	*value = databuf[0];  
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_ReadSensorData(int *buf)
{
  unsigned char databuf[10];
  int output[3];
  int i;
  
  if(NULL == afx133_i2c_client)
  {
  	*buf = 0;
  	return -2;
  }   
  	
  databuf[0] = REG_DATA;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
  output[0] = ((int) databuf[1]) << 8 | ((int) databuf[0]);
  output[1] = ((int) databuf[3]) << 8 | ((int) databuf[2]);
  output[2] = ((int) databuf[5]) << 8 | ((int) databuf[4]);
  
  for(i=0;i<3;i++) output[i] = (output[i] > 32767) ? (output[i] - 65536) : output[i];
  
  for(i=0;i<3;i++)
    if(afx133_mid_data.pid == AF9133_PID)
      buf[i] = output[i] - afx133_mid_data.offset[i];
    else
    	buf[i] = output[i];
  
  databuf[0] = REG_MEASURE;
  databuf[1] = 0x01;
  afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  return 0;
 }

/*----------------------------------------------------------------------------*/
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[10];
	sprintf(strbuf, "af7133d");
	return sprintf(buf, "%s", strbuf);		
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char databuf[2];
	
	databuf[0] = REG_PCODE;
	afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x101, I2C_FLAG_READ); 	
  	
	if(AF8133J_PID != databuf[0] && AF9133_PID != databuf[0] && AF5133_PID != databuf[0])
	{
		printk("afx133 PCODE is incorrect: %d\n", databuf[0]);
	} 

	return sprintf(buf, "%X\n", databuf[0]);       
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int databuf[3];
	afx133_ReadSensorData(databuf);
	return sprintf(buf, "%d %d %d\n", databuf[0],databuf[1],databuf[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[32];
	
	read_lock(&afx133_mid_data.datalock);
	tmp[0] = afx133_mid_data.ori[0] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = afx133_mid_data.ori[1] * CONVERT_O / CONVERT_O_DIV;;
	tmp[2] = afx133_mid_data.ori[2] * CONVERT_O / CONVERT_O_DIV;;
	read_unlock(&afx133_mid_data.datalock); 
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);	
	
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[32];
	
	read_lock(&afx133_mid_data.datalock);
	tmp[0] = afx133_mid_data.mag[0] * CONVERT_M / CONVERT_M_DIV;
	tmp[1] = afx133_mid_data.mag[1] * CONVERT_M / CONVERT_M_DIV;
	tmp[2] = afx133_mid_data.mag[2] * CONVERT_M / CONVERT_M_DIV;
	read_unlock(&afx133_mid_data.datalock); 
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);	
	
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_midcontrol_value(struct device_driver *ddri, char *buf)
{
	char strbuf[32];
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_midcontrol_value(struct device_driver *ddri, const char *buf, size_t count)
{   
	int p[10];
	if(10 == sscanf(buf, "%d %d %d %d %d %d %d %d %d %d",&p[0], &p[1], &p[2], &p[3], &p[4], 
		&p[5], &p[6], &p[7], &p[8], &p[9]))
	{
		write_lock(&afx133_mid_data.ctrllock);
		memcpy(&afx133_mid_data.controldata[0], &p, sizeof(int)*10);    
		write_unlock(&afx133_mid_data.ctrllock);        
	}
	else
	{
		MSE_ERR("invalid format\n");     
	}
	return sizeof(int)*10;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_middebug_value(struct device_driver *ddri, char *buf)
{
	ssize_t len;
	read_lock(&afx133_mid_data.ctrllock);
	len = sprintf(buf, "0x%08X\n", afx133_mid_data.debug);
	read_unlock(&afx133_mid_data.ctrllock);

	return len;            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_middebug_value(struct device_driver *ddri, const char *buf, size_t count)
{   
	int debug;
	if(1 == sscanf(buf, "0x%x", &debug))
	{
		write_lock(&afx133_mid_data.ctrllock);
		afx133_mid_data.debug = debug;
		write_unlock(&afx133_mid_data.ctrllock);        
	}
	else
	{
		MSE_ERR("invalid format\n");     
	}
	return count;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_mode_value(struct device_driver *ddri, char *buf)
{
	int mode=0;
	read_lock(&afx133_data.lock);
	mode = afx133_data.mode;
	read_unlock(&afx133_data.lock);        
	return sprintf(buf, "%d\n", mode);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_mode_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int mode = 0;
	sscanf(buf, "%d", &mode);    
	afx133_SetMode(mode);
	return count;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
    struct afx133_i2c_data *data;

    if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct afx133_i2c_data *data;
    int layout;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	layout = 0;
	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    struct afx133_i2c_data *data;
    ssize_t len;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}
    
	data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	len = 0;
	if(data->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
		data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
static ssize_t store_status_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct afx133_i2c_data *data;
    int value;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

    value = simple_strtol(buf, NULL, 10);
    
    data->hw->direction = value;
	if(hwmsen_get_convert(value, &data->cvt)<0)
	{
		MSE_ERR("invalid direction: %d\n", value);
	}

	atomic_set(&data->layout, value);
	return count;    

}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    struct afx133_i2c_data *data;
	ssize_t res;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}	

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&data->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct afx133_i2c_data *data;
    int trace = 0;

	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
    
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&data->trace, trace);
	}
	else 
	{
		MSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;    
}

/*----------------------------shipment test------------------------------------------------*/
/*!
 @return If @a testdata is in the range of between @a lolimit and @a hilimit,
 the return value is 1, otherwise -1.
 @param[in] testno   A pointer to a text string.
 @param[in] testname A pointer to a text string.
 @param[in] testdata A data to be tested.
 @param[in] lolimit  The maximum allowable value of @a testdata.
 @param[in] hilimit  The minimum allowable value of @a testdata.
 @param[in,out] pf_total
 */
int AFX133_TEST_DATA(const char testno[], const char testname[], const int testdata,
	const int lolimit, const int hilimit, int *pf_total)
{
	int pf;			/* Pass;1, Fail;-1 */

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		MSE_LOG(" Test No. Test Name	Fail	Test Data	[	 Low	High]\n");
		MSE_LOG("--------------------------------------------------------------------\n");
		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		if (*pf_total == 1)
			MSE_LOG("Factory shipment test was passed.\n\n");
		else
			MSE_LOG("Factory shipment test was failed.\n\n");

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit))
			pf = 1;
		else
			pf = -1;

	/* display result */
	MSE_LOG(" %7s  %-10s	 %c	%9d	[%9d	%9d]\n",
		testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
		lolimit, hilimit);
	}

	/* Pass/Fail check */
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1))
			*pf_total = 1;		/* Pass */
		else
			*pf_total = -1;		/* Fail */
	}
	return pf;
}

/*!
 Execute "Onboard Function Test" (NOT includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int FST_AFX133(void)
{
	int  pf_total;  /* p/f flag for this subtest */
	u8   databuf[6];
	int  value[3];
	int  pos[3];
  int  neg[3];
  int  offset[3];
  int  i;
  int  pid_flag=0;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	pf_total = 1;

	/* *********************************************** */
	/* Step1 */
	/* *********************************************** */

	/* Reset device. */
	afx133_Chipset_Init(0);

	/* Read values from WIA. */
	databuf[0] = REG_PCODE;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x101, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

#ifndef BYPASS_PCODE_CHECK_FOR_MTK_DRL
  if(AF8133J_PID == databuf[0] || AF9133_PID == databuf[0] || AF5133_PID == databuf[0])
    pid_flag = 1;

	/* TEST */
	AFX133_TEST_DATA("T1", "AFX133 Product Code is correct", pid_flag, 1, 1, &pf_total);
#else
  AFX133_TEST_DATA("T1", "Bypass AFX133 Product Code", pid_flag, 0, 0, &pf_total);
#endif


	/* Find offset by SW set_reset */
	// neg data (reset)
  databuf[0] = REG_MODE;
	databuf[1] = 0x34;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
	mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  neg[0] = databuf[0] | (databuf[1] << 8);
  neg[1] = databuf[2] | (databuf[3] << 8);
  neg[2] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) neg[i] = (neg[i] > 32767) ? (neg[i] - 65536) : neg[i];

  // pos data (reset)
  databuf[0] = REG_MODE;
	databuf[1] = 0x38;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
	mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

	pos[0] = databuf[0] | (databuf[1] << 8);
  pos[1] = databuf[2] | (databuf[3] << 8);
  pos[2] = databuf[4] | (databuf[5] << 8);
  for(i=0;i<3;i++) pos[i] = (pos[i] > 32767) ? (pos[i] - 65536) : pos[i];

  // offset
  for(i=0;i<3;i++) offset[i] = (pos[i] + neg[i]) / 2;

	/* TEST */
	AFX133_TEST_DATA("T2_1", "AFX133 x-axis offset", offset[0], AFX133_OFFSET_MIN, AFX133_OFFSET_MAX, &pf_total);
	AFX133_TEST_DATA("T2_2", "AFX133 y-axis offset", offset[1], AFX133_OFFSET_MIN, AFX133_OFFSET_MAX, &pf_total);
	AFX133_TEST_DATA("T2_3", "AFX133 z-axis offset", offset[2], AFX133_OFFSET_MIN, AFX133_OFFSET_MAX, &pf_total);

	/* *********************************************** */
	/* Step2 */
	/* *********************************************** */
	
  /* Set to Self-test mode */
  databuf[0] = REG_I2C_CHECK;
	databuf[1] = 0x55;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}	

  /* Set to Self-test mode */
  databuf[0] = REG_MODE;
	databuf[1] = 0x3C;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  /************************************************************************************/
	// positive BIST function (X and Y)
  databuf[0] = REG_BIST;
	databuf[1] = 0xB1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // positive measurement (X and Y)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

	pos[0] = databuf[0] | (databuf[1] << 8);
  pos[1] = databuf[2] | (databuf[3] << 8); 
  pos[0] = (pos[0] > 32767) ? (pos[0] - 65536) : pos[0];
  pos[1] = (pos[1] > 32767) ? (pos[1] - 65536) : pos[1];

	// negative BIST function  (X and Y)
  databuf[0] = REG_BIST;
	databuf[1] = 0xC1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // negative measurement (X and Y)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  neg[0] = databuf[0] | (databuf[1] << 8);
  neg[1] = databuf[2] | (databuf[3] << 8);
  neg[0] = (neg[0] > 32767) ? (neg[0] - 65536) : neg[0];
  neg[1] = (neg[1] > 32767) ? (neg[1] - 65536) : neg[1];

  /************************************************************************************/
	// positive BIST function (Z)
  databuf[0] = REG_BIST;
	databuf[1] = 0xD1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // positive measurement (Z)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

	pos[2] = databuf[4] | (databuf[5] << 8);
  pos[2] = (pos[2] > 32767) ? (pos[2] - 65536) : pos[2];

	// negative BIST function (Z)
  databuf[0] = REG_BIST;
	databuf[1] = 0xE1;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

  // negative measurement (Z)
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	databuf[0] = REG_DATA;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x601, I2C_FLAG_READ) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  neg[2] = databuf[4] | (databuf[5] << 8);
  neg[2] = (neg[2] > 32767) ? (neg[2] - 65536) : neg[2];

  /************************************************************************************/
  
  // calculate sensitivity
  if(afx133_mid_data.pid == AF8133J_PID)
  {
    value[0] = pos[0] - neg[0];
    value[1] = pos[1] - neg[1];
    value[2] = pos[2] - neg[2]; 
  }
  else if(afx133_mid_data.pid == AF9133_PID)   
  {
    value[0] = neg[0] - pos[0];
    value[1] = neg[1] - pos[1];
    value[2] = pos[2] - neg[2];   	
  }
  else //AF5133
  {
    value[0] = neg[0] - pos[0];
    value[1] = neg[1] - pos[1];
    value[2] = pos[2] - neg[2];  	
  }

	/* TEST */
	AFX133_TEST_DATA("T3_1", "AFX133 BIST test 1", value[0], AFX133_SENS_MIN, AFX133_SENS_MAX, &pf_total);
	AFX133_TEST_DATA("T3_2", "AFX133 BIST test 2", value[1], AFX133_SENS_MIN, AFX133_SENS_MAX, &pf_total);
	AFX133_TEST_DATA("T3_2", "AFX133 BIST test 3", value[2], AFX133_SENS_MIN, AFX133_SENS_MAX, &pf_total);

  // disbale BIST function
  databuf[0] = REG_BIST;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}

  // open clock
  databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	if (afx133_i2c_master_operate(afx133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE) < 0) {
		MSE_ERR("afx133 I2C Error.\n");
		return 0;
	}
  mdelay(5);

	return pf_total;
}

/*!
 Execute "Onboard Function Test" (includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int AFX133_FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	AFX133_TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	/* *********************************************** */
	/* Step 1 to 2 */
	/* *********************************************** */
	pf_total = FST_AFX133();

	/* *********************************************** */
	/* Judge Test Result */
	/* *********************************************** */
	AFX133_TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}

static ssize_t store_shipment_test(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;

	res = AFX133_FctShipmntTestProcess_Body();
	if (1 == res) {
		MSE_LOG("shipment_test pass\n");
		strcpy(result, "y");
	} else if (-1 == res) {
		MSE_LOG("shipment_test fail\n");
		strcpy(result, "n");
	} else {
		MSE_LOG("shipment_test NaN\n");
		strcpy(result, "NaN");
	}

	return sprintf(buf, "%s\n", result);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(calidata,    S_IRUGO, show_calidata_value, NULL);
static DRIVER_ATTR(midcontrol,  S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value );
static DRIVER_ATTR(middebug,    S_IRUGO | S_IWUSR, show_middebug_value, store_middebug_value );
static DRIVER_ATTR(mode,        S_IRUGO | S_IWUSR, show_mode_value, store_mode_value );
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(status,      S_IRUGO | S_IWUSR, show_status_value, store_status_value);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *afx133_attr_list[] = {
  &driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_calidata,
	&driver_attr_midcontrol,
	&driver_attr_middebug,
	&driver_attr_mode,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
	&driver_attr_shipmenttest,
};
/*----------------------------------------------------------------------------*/
static int afx133_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(afx133_attr_list)/sizeof(afx133_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, afx133_attr_list[idx])))
		{            
			MSE_ERR("driver_create_file (%s) = %d\n", afx133_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int afx133_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(afx133_attr_list)/sizeof(afx133_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, afx133_attr_list[idx]);
	}


	return err;
}


/*----------------------------------------------------------------------------*/
static int afx133_open(struct inode *inode, struct file *file)
{    
	struct afx133_i2c_data *obj = i2c_get_clientdata(afx133_i2c_client);    
	int ret = -1;
	atomic_inc(&dev_open_count);

	if(atomic_read(&obj->trace) & VTC_TRC_DEBUG)
	{
		MSE_LOG("Open device node:AFX133\n");
	}
	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int afx133_release(struct inode *inode, struct file *file)
{
	struct afx133_i2c_data *obj = i2c_get_clientdata(afx133_i2c_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & VTC_TRC_DEBUG)
	{
		MSE_LOG("Release device node:AFX133\n");
	}	
	return 0;
}
/*----------------------------------------------------------------------------*/
static long afx133_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
  void __user *argp = (void __user *)arg;
	int valuebuf[22] = {0}; 
	int magbuf[3] = {0};
	void __user *data;
	long retval=0;
	int mode=0;
	int status; 				
	short sensor_status;		
	struct hwm_sensor_data* osensor_data;
	char buff[128];
	char version[10];
    unsigned char reg;
	int mag_layout;

	
#if DEBUG
    struct afx133_i2c_data *obj;
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}
	obj = i2c_get_clientdata(afx133_i2c_client);

	if(NULL == obj)
	{
		MSE_ERR("obj IS NULL !\n");
		return -1;
	}
#endif    
   
	switch (cmd)
	{
		case MSENSOR_IOCTL_INIT:      
			afx133_Chipset_Init(AFX133_MODE_SINGLE);
			break;

		case MSENSOR_IOCTL_SET_POSTURE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				MSE_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			if(copy_from_user(valuebuf, data, sizeof(valuebuf)))
			{
				retval = -EFAULT;
				goto err_out;
			}
#if DEBUG
				if (atomic_read(&obj->trace) & VTC_TRC_DEBUG) {
						MSE_ERR("AFX133 driver: Osensor %d %d %d\n", valuebuf[0], valuebuf[1], valuebuf[2]);
				}
#endif
      
        write_lock(&afx133_mid_data.datalock);
        afx133_mid_data.ori[0] = valuebuf[0];
        afx133_mid_data.ori[1] = valuebuf[1];
        afx133_mid_data.ori[2] = valuebuf[2];
        afx133_mid_data.mag[0] = valuebuf[3];
        afx133_mid_data.mag[1] = valuebuf[4];
        afx133_mid_data.mag[2] = valuebuf[5];
        afx133_mid_data.acc[0] = valuebuf[6];
        afx133_mid_data.acc[1] = valuebuf[7];
        afx133_mid_data.acc[2] = valuebuf[8];
        afx133_mid_data.mag_status = valuebuf[9];
#ifdef SUPPORT_SOFTGYRO_FUNCTION
        afx133_mid_data.rov[0] = valuebuf[10]; 
        afx133_mid_data.rov[1] = valuebuf[11]; 
        afx133_mid_data.rov[2] = valuebuf[12]; 
        afx133_mid_data.gyr[0] = valuebuf[13]; 
        afx133_mid_data.gyr[1] = valuebuf[14]; 
        afx133_mid_data.gyr[2] = valuebuf[15];  
        afx133_mid_data.lin[0] = valuebuf[16]; 
        afx133_mid_data.lin[1] = valuebuf[17]; 
        afx133_mid_data.lin[2] = valuebuf[18]; 
        afx133_mid_data.gra[0] = valuebuf[19]; 
        afx133_mid_data.gra[1] = valuebuf[20]; 
        afx133_mid_data.gra[2] = valuebuf[21]; 
#endif
			write_unlock(&afx133_mid_data.datalock);    
			break;

		case ECOMPASS_IOC_GET_OFLAG:
			sensor_status = atomic_read(&o_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = atomic_read(&m_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MSE_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_OPEN_STATUS:
			status = afx133_GetOpenStatus();			
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MSE_LOG("copy_to_user failed.");
				return -EFAULT;
			}
			break;        

                case ECOMPASS_IOC_GET_DELAY:
			status = afx133_mid_data.controldata[0];						
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MSE_LOG("copy_to_user failed.");
				return -EFAULT;
			}
			break;                     

		case MSENSOR_IOCTL_SET_CALIDATA:
			break;                                

		case MSENSOR_IOCTL_READ_CHIPINFO:          
			data = (void __user *) arg;
			if(data == NULL)
			{
				MSE_ERR("IO parameter pointer is NULL!\r\n");
				break;    
			}
			sprintf(version, "%s-%s", (char*)DRIVER_VERSION, (char*)DRIVER_RELEASE);
			if(copy_to_user(data, version, strlen(version)))
			{
				retval = -EFAULT;
				goto err_out;
			}            
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				MSE_ERR("IO parameter pointer is NULL!\r\n");
				break;    
			}
			afx133_ReadSensorData(magbuf);
#if DEBUG
				if (atomic_read(&obj->trace) & VTC_TRC_DEBUG) {
						MSE_ERR("AFX133 driver: Msensor %d %d %d\n", magbuf[0], magbuf[1], magbuf[2]);
				}
#endif			
			
			sprintf(buff, "%x %x %x", magbuf[0], magbuf[1], magbuf[2]);
			if(copy_to_user(data, buff, strlen(buff)+1))
			{
				retval = -EFAULT;
				goto err_out;
			}                
			break;

		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
			data = (void __user *) arg; 
            if(data == NULL)
			{ 
			    MSE_ERR( "IO parameter pointer is NULL!\r\n");
				break; 
            }
			osensor_data = (struct hwm_sensor_data *)buff;
			read_lock(&afx133_mid_data.datalock);
			osensor_data->values[0] = afx133_mid_data.ori[0];
			osensor_data->values[1] = afx133_mid_data.ori[1];
			osensor_data->values[2] = afx133_mid_data.ori[2];
			osensor_data->status = afx133_mid_data.mag_status;
			read_unlock(&afx133_mid_data.datalock); 
			osensor_data->value_divide = CONVERT_O_DIV;
			//sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1], osensor_data->values[2], osensor_data->status, osensor_data->value_divide);
			if(copy_to_user(data, buff, strlen(buff)+1))
			{ return -EFAULT; } 
			break;                

		case MSENSOR_IOCTL_READ_POSTUREDATA:             
			break;            

		case MSENSOR_IOCTL_READ_CALIDATA:           
			break;

		case MSENSOR_IOCTL_READ_CONTROL:
 			data = (void __user *) arg;
			if(data == NULL)
			{
			  MSE_ERR("IO parameter pointer is NULL!\r\n");
			  break;    
			}
			if(copy_from_user(buff, data, sizeof(buff[0])))
			{
				retval = -EFAULT;
				goto err_out;
			}
			reg = (unsigned char)buff[0];			
			afx133_Read_Regiser(reg, buff);						
			if(copy_to_user(data, buff, sizeof(buff[0])))
			{
				retval = -EFAULT;
				goto err_out;
			}                                                          
			break;

		case MSENSOR_IOCTL_SET_CONTROL:
			data = (void __user *) arg;
			if(data == NULL)
			{
			  MSE_ERR("IO parameter pointer is NULL!\r\n");
			  break;    
			}
			if(copy_from_user(valuebuf, data, sizeof(valuebuf[0])))
			{
				retval = -EFAULT;
				goto err_out;
			}
			MSE_ERR("Grant Debug %X \r\n",valuebuf[0]);
			afx133_Write_Register((unsigned char)(valuebuf[0]>>8), (unsigned char)(valuebuf[0]));
			break;

		case MSENSOR_IOCTL_SET_MODE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				break;
			}
			if(copy_from_user(&mode, data, sizeof(mode)))
			{
				retval = -EFAULT;
				goto err_out;
			}             
			if(afx133_SetMode(mode))
			{
				retval = -EFAULT;
				goto err_out;				
			}
			break;

        case ECOMPASS_IOC_GET_LAYOUT:
			mag_layout = hw->direction;
			if(copy_to_user(argp, &mag_layout, sizeof(mag_layout)))
			{
				retval = -EFAULT;
				goto err_out;
			}  
		    break;

		default:
			MSE_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
		}

	err_out:
	return retval;    
}

#ifdef CONFIG_COMPAT
static long afx133_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
		
	switch (cmd) {
		
		 case COMPAT_MSENSOR_IOCTL_INIT:
	
			 if(arg32 == NULL)
			 {
				 MSE_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_INIT,
							(unsigned long)arg32);
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_INIT unlocked_ioctl failed.");
				return ret;
			 }			 

			 break;
		 case COMPAT_MSENSOR_IOCTL_SET_POSTURE:
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_POSTURE,
							(unsigned long)arg32);
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_SET_POSTURE unlocked_ioctl failed.");
				return ret;
			 }
		     break;		 
		 case COMPAT_ECOMPASS_IOC_GET_OFLAG:
		 	 if(arg32 == NULL)
			 {
				 MSE_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OFLAG,
							(unsigned long)arg32);
			 if (ret){
			 	MSE_ERR("ECOMPASS_IOC_GET_OFLAG unlocked_ioctl failed.");
				return ret;
			 }
			 
			 break;

		case COMPAT_ECOMPASS_IOC_GET_MFLAG:
		 	 ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_MFLAG,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("ECOMPASS_IOC_GET_MFLAG unlocked_ioctl failed.");
				return ret;
			 }

			 break;
			 
		 case COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS:
		 	 ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OPEN_STATUS,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("ECOMPASS_IOC_GET_OPEN_STATUS unlocked_ioctl failed.");
				return ret;
			 }

			 break;
			 	 
		 case COMPAT_MSENSOR_IOCTL_SET_CALIDATA:
		 	 if(arg32 == NULL)
			 {
				 MSE_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_CALIDATA,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_SET_CALIDATA unlocked_ioctl failed.");
				return ret;
			 }
			 break;
		
		 case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_READ_CHIPINFO unlocked_ioctl failed.");
				return ret;
			 }

			 break;
			 
		 case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
		 	 if(arg32 == NULL)
			 {
				 MSE_ERR("invalid argument.");
				 return -EINVAL;
			 }		
			 
			 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_SENSOR_ENABLE unlocked_ioctl failed.");
				return ret;
			 }
			 break;
		
		 case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
				return ret;
			 }

			 break;
			 
		 case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_READ_FACTORY_SENSORDATA unlocked_ioctl failed.");
				return ret;
			 }

			 break;
			 
		 case COMPAT_MSENSOR_IOCTL_READ_POSTUREDATA:
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_POSTUREDATA,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_READ_POSTUREDATA unlocked_ioctl failed.");
				return ret;
			 }

			 break;
			 
		 case COMPAT_MSENSOR_IOCTL_READ_CALIDATA:
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CALIDATA,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_READ_CALIDATA unlocked_ioctl failed.");
				return ret;
			 }
			 
			 break;
		
		 case COMPAT_MSENSOR_IOCTL_READ_CONTROL:
			 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CONTROL,
							(unsigned long)arg32);
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_READ_CONTROL unlocked_ioctl failed.");
				return ret;
			 }
			 
			 break;
		
		 case COMPAT_MSENSOR_IOCTL_SET_CONTROL:
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_CONTROL,
							(unsigned long)arg32);
			 if (ret){
			 	MSE_ERR("SENSOR_IOCTL_SET_CONTROL unlocked_ioctl failed.");
				return ret;
			 }
			 
			 break;
		
		 case COMPAT_MSENSOR_IOCTL_SET_MODE:	
		 	 ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SET_MODE,
							(unsigned long)arg32);
			 if (ret){
			 	MSE_ERR("MSENSOR_IOCTL_SET_MODE unlocked_ioctl failed.");
				return ret;
			 }

			 break;
			 
		 case COMPAT_ECOMPASS_IOC_GET_LAYOUT:
		 	 if(arg32 == NULL)
			 {
				 MSE_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_LAYOUT,
							(unsigned long)(arg32));
			 if (ret){
			 	MSE_ERR("ECOMPASS_IOC_GET_LAYOUT unlocked_ioctl failed.");
				return ret;
			 }
			 
			 break;
			 
		 default:
			 return -ENOIOCTLCMD;
			 break;
	}
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static struct file_operations afx133_fops = {
//	.owner = THIS_MODULE,
	.open = afx133_open,
	.release = afx133_release,
	.unlocked_ioctl = afx133_unlocked_ioctl,//modified
	#ifdef CONFIG_COMPAT
	.compat_ioctl = afx133_compat_ioctl,
	#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice afx133_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "msensor",
    .fops = &afx133_fops,
};
/*----------------------------------------------------------------------------*/
static int afx133_m_open_report_data(int en)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_m_set_delay(u64 delay)
{
	int value = 0;
	int sample_delay = afx133_mid_data.controldata[0];
	struct afx133_i2c_data *data;
	
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	value = (int)(delay / 1000000);
	if(value < AFX133_DELAY_MIN)
	{
		sample_delay = AFX133_DELAY_MIN;
	}
	else if(value > AFX133_DELAY_MAX)
	{
		sample_delay = AFX133_DELAY_MAX;
	}
	else
	{
		sample_delay = value;
	}
	
	afx133_mid_data.controldata[0] = sample_delay;  // Loop Delay

	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_m_enable(int en)
{
	int value = 0;
    struct afx133_i2c_data *data ;
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	value = en;
	read_lock(&afx133_mid_data.ctrllock);
	if(value == 1)
	{
		afx133_mid_data.controldata[7] |= SENSOR_MAGNETIC;
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		afx133_mid_data.controldata[7] &= ~SENSOR_MAGNETIC;
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}	
	}
	wake_up(&open_wq);
	read_unlock(&afx133_mid_data.ctrllock);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_o_open_report_data(int en)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_o_set_delay(u64 delay)
{
	return afx133_m_set_delay(delay);
}
/*----------------------------------------------------------------------------*/
static int afx133_o_enable(int en)
{
	int value = 0;
    struct afx133_i2c_data *data ;
	
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
	
	value = en;
	read_lock(&afx133_mid_data.ctrllock);
	if(value == 1)
	{
		afx133_mid_data.controldata[7] |= SENSOR_ORIENTATION;
		atomic_set(&o_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		afx133_mid_data.controldata[7] &= ~SENSOR_ORIENTATION;
		atomic_set(&o_flag, 0);
		if(atomic_read(&m_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}
	}
	wake_up(&open_wq);
	read_unlock(&afx133_mid_data.ctrllock);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_get_data_m(int *x,int *y, int *z,int *status)
{
  struct afx133_i2c_data *data ;
	int x_tmp = 0;
	int y_tmp = 0;
	int z_tmp = 0;
	
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	read_lock(&afx133_mid_data.datalock);
	x_tmp = afx133_mid_data.mag[0];
	y_tmp = afx133_mid_data.mag[1];
	z_tmp = afx133_mid_data.mag[2];
	*status = afx133_mid_data.mag_status;
	read_unlock(&afx133_mid_data.datalock); 
	
	*x = x_tmp * CONVERT_M;
	*y = y_tmp * CONVERT_M;
	*z = z_tmp * CONVERT_M;

#if DEBUG
	if (atomic_read(&data->trace) & VTC_TRC_DEBUG) {
			MSE_LOG("%s get data: %d, %d, %d. divide %d, status %d!", __func__,
			*x, *y, *z, CONVERT_M_DIV, *status);
	}
#endif

	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_get_data_o(int *x,int *y, int *z,int *status)
{
    struct afx133_i2c_data *data ;
	if(NULL == afx133_i2c_client)
	{
		MSE_ERR("afx133_i2c_client IS NULL !\n");
		return -1;
	}

    data = i2c_get_clientdata(afx133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	read_lock(&afx133_mid_data.datalock);
	*x = afx133_mid_data.ori[0];
	*y = afx133_mid_data.ori[1];
	*z = afx133_mid_data.ori[2];
	*status = afx133_mid_data.mag_status;	
	read_unlock(&afx133_mid_data.datalock); 

#if DEBUG
	if (atomic_read(&data->trace) & VTC_TRC_DEBUG) {
			MSE_ERR("%s get data: %d, %d, %d. divide %d, status %d!", __func__,
			*x, *y, *z,	CONVERT_O_DIV, *status);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef SUPPORT_SOFTGYRO_FUNCTION

int afx133_rotation_vector_operate(void* self, uint32_t command, void* buff_in, int size_in, 
                            void* buff_out, int size_out, int* actualout) 
{ 
  int err = 0; 
  int value; 
  struct hwm_sensor_data* rvsensor_data=NULL; 
  
  MSE_FUN(); 
  
  switch (command) 
  { 
    case SENSOR_DELAY: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      }
      else 
      {
        value = *(int *)buff_in; 

        afx133_mid_data.controldata[0] = AFX133_DELAY_SW_GYRO;  //recommand speed
      } 
      break;
    case SENSOR_ENABLE: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      } 
      else 
      { 
        value = *(int *)buff_in; 
        read_lock(&afx133_mid_data.ctrllock);
		    if(value == 1)
		    {
        	afx133_mid_data.controldata[7] |= SENSOR_ROTATION_VECTOR;
			    atomic_set(&open_flag, 1);
		    }
		    else
		    {
        	afx133_mid_data.controldata[7] &= ~SENSOR_ROTATION_VECTOR;
        	if(afx133_mid_data.controldata[7] == 0)
			    {
				    atomic_set(&open_flag, 0);
			    }
		    }
		    wake_up(&open_wq);
		    read_unlock(&afx133_mid_data.ctrllock);
      } 
      break;
      case SENSOR_GET_DATA: 
        if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data))) 
        { 
          err = -EINVAL; 
        } 
        else 
        { 
          rvsensor_data = (struct hwm_sensor_data *)buff_out; 
          read_lock(&afx133_mid_data.datalock); 
          rvsensor_data->values[0] = afx133_mid_data.rov[0];
          rvsensor_data->values[1] = afx133_mid_data.rov[1];
          rvsensor_data->values[2] = afx133_mid_data.rov[2];
          rvsensor_data->status = afx133_mid_data.mag_status; 
          rvsensor_data->value_divide = CONVERT_RV_DIV; 
          read_unlock(&afx133_mid_data.datalock);  
        } 
        break; 
      default: 
        err = -1; 
        break; 
  }
  return err; 
} 
/*----------------------------------------------------------------------------*/
int afx133_gyroscope_operate(void* self, uint32_t command, void* buff_in, int size_in, 
                            void* buff_out, int size_out, int* actualout) 
{ 
  int err = 0; 
  int value; 
  struct hwm_sensor_data* gysensor_data=NULL; 
  
  MSE_FUN(); 
  
  switch (command) 
  { 
    case SENSOR_DELAY: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      }
      else 
      {
        value = *(int *)buff_in; 

        afx133_mid_data.controldata[0] = AFX133_DELAY_SW_GYRO;  //recommand speed
      } 
      break;
    case SENSOR_ENABLE: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      } 
      else 
      { 
        value = *(int *)buff_in; 
			read_lock(&afx133_mid_data.ctrllock);
			if(value == 1)
			{
				afx133_mid_data.controldata[7] |= SENSOR_GYROSCOPE;
				atomic_set(&open_flag, 1);
			}
			else
			{
				afx133_mid_data.controldata[7] &= ~SENSOR_GYROSCOPE;
				if(afx133_mid_data.controldata[7] == 0)
				{
					atomic_set(&open_flag, 0);
				}
			}
			wake_up(&open_wq);
			read_unlock(&afx133_mid_data.ctrllock);
      } 
      break;
      case SENSOR_GET_DATA: 
        if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data))) 
        { 
          err = -EINVAL; 
        } 
        else 
        { 
          gysensor_data = (struct hwm_sensor_data *)buff_out; 
          read_lock(&afx133_mid_data.datalock); 
          gysensor_data->values[0] = afx133_mid_data.gyr[0];
          gysensor_data->values[1] = afx133_mid_data.gyr[1];
          gysensor_data->values[2] = afx133_mid_data.gyr[2];
          gysensor_data->status = afx133_mid_data.mag_status; 
          gysensor_data->value_divide = CONVERT_GY_DIV; 
          read_unlock(&afx133_mid_data.datalock);  
        } 
        break; 
      default: 
        err = -1; 
        break; 
  }
  return err; 
} 
/*----------------------------------------------------------------------------*/
int afx133_linear_acceleration_operate(void* self, uint32_t command, void* buff_in, int size_in, 
                            void* buff_out, int size_out, int* actualout) 
{ 
  int err = 0; 
  int value; 
  struct hwm_sensor_data* lasensor_data=NULL; 
  
  MSE_FUN(); 
  
  switch (command) 
  { 
    case SENSOR_DELAY: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      }
      else 
      {
        value = *(int *)buff_in; 

        afx133_mid_data.controldata[0] = AFX133_DELAY_SW_GYRO;  //recommand speed
      } 
      break;
    case SENSOR_ENABLE: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      } 
      else 
      { 
        value = *(int *)buff_in; 
        read_lock(&afx133_mid_data.ctrllock);
        if(value == 1)
        {
        	afx133_mid_data.controldata[7] |= SENSOR_LINEAR_ACCELERATION;
        	atomic_set(&open_flag, 1);
        }
        else
        {
        	afx133_mid_data.controldata[7] &= ~SENSOR_LINEAR_ACCELERATION;
        	if(afx133_mid_data.controldata[7] == 0)
        	{
        		atomic_set(&open_flag, 0);
        	}
        }
        wake_up(&open_wq);
        read_unlock(&afx133_mid_data.ctrllock);
      } 
      break;
    case SENSOR_GET_DATA: 
      if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data))) 
      { 
        err = -EINVAL; 
      } 
      else 
      { 
        lasensor_data = (struct hwm_sensor_data *)buff_out; 
        read_lock(&afx133_mid_data.datalock); 
        lasensor_data->values[0] = afx133_mid_data.lin[0];
        lasensor_data->values[1] = afx133_mid_data.lin[1];
        lasensor_data->values[2] = afx133_mid_data.lin[2];
        lasensor_data->status = afx133_mid_data.mag_status; 
        lasensor_data->value_divide = CONVERT_LA_DIV; 
        read_unlock(&afx133_mid_data.datalock);  
      } 
      break; 
    default: 
      err = -1; 
      break; 
  }
  return err; 
} 
/*----------------------------------------------------------------------------*/
int afx133_gravity_operate(void* self, uint32_t command, void* buff_in, int size_in, 
                            void* buff_out, int size_out, int* actualout) 
{ 
  int err = 0; 
  int value; 
  struct hwm_sensor_data* gvsensor_data=NULL; 
  
  MSE_FUN(); 
  
  switch (command) 
  { 
    case SENSOR_DELAY: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      }
      else 
      {
        value = *(int *)buff_in; 

        afx133_mid_data.controldata[0] = AFX133_DELAY_SW_GYRO;  //recommand speed
      } 
      break;
    case SENSOR_ENABLE: 
      if((buff_in == NULL) || (size_in < sizeof(int))) 
      { 
        err = -EINVAL; 
      } 
      else 
      { 
        value = *(int *)buff_in; 
        read_lock(&afx133_mid_data.ctrllock);
        if(value == 1)
        {
        	afx133_mid_data.controldata[7] |= SENSOR_GRAVITY;
        	atomic_set(&open_flag, 1);
        }
        else
        {
        	afx133_mid_data.controldata[7] &= ~SENSOR_GRAVITY;
        	if(afx133_mid_data.controldata[7] == 0)
        	{
        		atomic_set(&open_flag, 0);
        	}
        }
        wake_up(&open_wq);
        read_unlock(&afx133_mid_data.ctrllock);
      } 
      break;
    case SENSOR_GET_DATA: 
      if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data))) 
      { 
        err = -EINVAL; 
      } 
      else 
      { 
        gvsensor_data = (struct hwm_sensor_data *)buff_out; 
        read_lock(&afx133_mid_data.datalock); 
        gvsensor_data->values[0] = afx133_mid_data.gra[0];
        gvsensor_data->values[1] = afx133_mid_data.gra[1];
        gvsensor_data->values[2] = afx133_mid_data.gra[2];
        gvsensor_data->status = afx133_mid_data.mag_status; 
        gvsensor_data->value_divide = CONVERT_GV_DIV; 
        read_unlock(&afx133_mid_data.datalock);  
      } 
      break; 
    default: 
      err = -1; 
      break; 
  }
  return err; 
} 
#endif //ifdef SUPPORT_SOFTGYRO_FUNCTION
/*----------------------------------------------------------------------------*/
static int afx133_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct afx133_i2c_data *obj = i2c_get_clientdata(client);
	MSE_FUN();    
#if 1
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		afx133_power(obj->hw, 0);   
			}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_resume(struct i2c_client *client)
{
	int err;
	struct afx133_i2c_data *obj = i2c_get_clientdata(client);
	MSE_FUN();
#if 1
	afx133_power(obj->hw, 1);

	if((err = afx133_Chipset_Init(AFX133_MODE_SINGLE))!=0)
	{
		MSE_ERR("initialize client fail!!\n");
		return err;        
	}
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
static int afx133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct afx133_i2c_data *data;
	int err = 0;

    struct mag_control_path ctl_path ={0};
    struct mag_data_path dat_path = {0};
	
#ifdef SUPPORT_SOFTGYRO_FUNCTION
	struct hwmsen_object sobj_rov;
	struct hwmsen_object sobj_gyr; 
	struct hwmsen_object sobj_lin;
	struct hwmsen_object sobj_gra; 
#endif

  printk("%s start\n",__func__);

	if (!(data = kmalloc(sizeof(struct afx133_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct afx133_i2c_data));

   data->hw = hw;
	
	if((err = hwmsen_get_convert(data->hw->direction, &data->cvt)))
	{
		MSE_ERR("invalid direction: %d\n", data->hw->direction);
		goto exit;
	}

	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);
	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	afx133_i2c_client = new_client;	

	if((err = afx133_Chipset_Init(AFX133_MODE_IDLE)))
	{
		MSE_ERR("afx133 register initial fail\n");
		goto exit_init_failed;
	}
	
	if((err = afx133_GetOffset()))
	{
		MSE_ERR("afx133 get offset initial fail\n");
		goto exit_init_failed;
	}

	/* Register sysfs attribute */
	if((err = afx133_create_attr(&afx133_init_info.platform_diver_addr->driver)))
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	if((err = misc_register(&afx133_device)))
	{
		MSE_ERR("afx133_device register failed\n");
		goto exit_misc_device_register_failed;	}    

		ctl_path.m_open_report_data = afx133_m_open_report_data;
		ctl_path.m_enable 					= afx133_m_enable;
		ctl_path.m_set_delay 				= afx133_m_set_delay;
		
		ctl_path.o_open_report_data = afx133_o_open_report_data;
		ctl_path.o_enable 					= afx133_o_enable;
		ctl_path.o_set_delay 				= afx133_o_set_delay;

		ctl_path.is_report_input_direct = false;
		ctl_path.is_support_batch 		= data->hw->is_batch_supported;
		
		err = mag_register_control_path(&ctl_path);

		if(err < 0)
		{
			MSE_ERR("mag_register_control_path failed!\n");
			goto exit_misc_device_register_failed;
		}

		dat_path.div_m = CONVERT_M_DIV;
		dat_path.div_o = CONVERT_O_DIV;

		dat_path.get_data_m = afx133_get_data_m;
		dat_path.get_data_o = afx133_get_data_o;

		err = mag_register_data_path(&dat_path);
		if(err < 0)
		{
			MSE_ERR("mag_register_control_path failed!\n");
			goto exit_misc_device_register_failed;
		}

#ifdef SUPPORT_SOFTGYRO_FUNCTION
        sobj_rov.self = data; 
        sobj_rov.polling = 1; 
        sobj_rov.sensor_operate = afx133_rotation_vector_operate; 
        if((err = hwmsen_attach(ID_GEOMAGNETIC_ROTATION_VECTOR, &sobj_rov)))
        { 
          MSE_ERR("VTC ROTATION_VECTOR attach fail = %d\n", err); 
          goto exit_kfree; 
        } 
        
        sobj_gyr.self = data; 
        sobj_gyr.polling = 1; 
        sobj_gyr.sensor_operate = afx133_gyroscope_operate; 
        if((err = hwmsen_attach(ID_GYROSCOPE, &sobj_gyr)))
        { 
          MSE_ERR("VTC GYROSCOPE attach fail = %d\n", err); 
          goto exit_kfree; 
        }
        
        sobj_lin.self = data; 
        sobj_lin.polling = 1; 
        sobj_lin.sensor_operate = afx133_linear_acceleration_operate; 
        if((err = hwmsen_attach(ID_LINEAR_ACCELERATION, &sobj_lin)))
        { 
          MSE_ERR("VTC LINEAR ACCELERATION attach fail = %d\n", err); 
          goto exit_kfree; 
        }
        
        sobj_gra.self = data; 
        sobj_gra.polling = 1; 
        sobj_gra.sensor_operate = afx133_gravity_operate; 
        if((err = hwmsen_attach(ID_GRAVITY, &sobj_gra)))
        { 
          MSE_ERR("VTC GRAVITY attach fail = %d\n", err); 
          goto exit_kfree; 
        }
#endif  

	MSE_LOG("%s: OK\n", __func__);

	afx133_init_flag=0;

	return 0;

	exit_sysfs_create_group_failed:   
	exit_init_failed:
	exit_misc_device_register_failed:
#ifdef SUPPORT_SOFTGYRO_FUNCTION
  exit_kfree:
#endif
  	kfree(data);
	exit:
	MSE_ERR("%s: err = %d\n", __func__, err);

	afx133_init_flag=-1;

	return err;
}
/*----------------------------------------------------------------------------*/
static int afx133_i2c_remove(struct i2c_client *client)
{
	int err;	

	if((err = afx133_delete_attr(&afx133_init_info.platform_diver_addr->driver)))
	{
		MSE_ERR("afx133_delete_attr fail: %d\n", err);
	}

	afx133_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));	
	misc_deregister(&afx133_device);    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_local_init(void)
{
	printk("afx133_local_init");
	rwlock_init(&afx133_mid_data.ctrllock);
	rwlock_init(&afx133_mid_data.datalock);
	rwlock_init(&afx133_data.lock);
	memset(&afx133_mid_data.controldata[0], 0, sizeof(int)*10);    
	afx133_mid_data.controldata[0] =    AFX133_DEFAULT_DELAY;  // Loop Delay
	afx133_mid_data.controldata[1] =     0;  // Run   
	afx133_mid_data.controldata[2] =     0;  // Disable Start-AccCali
	afx133_mid_data.controldata[3] =     1;  // Enable Start-Cali
	afx133_mid_data.controldata[4] =   350;  // MW-Timout
	afx133_mid_data.controldata[5] =    10;  // MW-IIRStrength_M
	afx133_mid_data.controldata[6] =    10;  // MW-IIRStrength_G   
	afx133_mid_data.controldata[7] =     0;  // Active Sensors
	afx133_mid_data.controldata[8] =     0;  // Wait for define
	afx133_mid_data.controldata[9] =     0;  // Wait for define   
	atomic_set(&dev_open_count, 0);
  
	if(i2c_add_driver(&afx133_i2c_driver))
	{
		MSE_ERR("add driver error\n");
		return -1;
	} 


	if(-1 == afx133_init_flag)	
	{	   
		return -1;	
	}

    printk("%s done\n",__func__);
    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int afx133_remove(void)
{
	MSE_FUN();    
	atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&afx133_i2c_driver);
	afx133_init_flag = -1;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init afx133_init(void)
{
	const char *name = "mediatek,afx133";

	MSE_FUN();

	hw = get_mag_dts_func(name, hw);
	if (!hw){
		MSE_ERR("get_mag_dts_func() fail, and call get_cust_mag_hw()\n");
		//hw = get_cust_mag_hw();
	}

	if(mag_driver_add(&afx133_init_info) < 0)
	{
		MSE_ERR("mag_driver_add failed!\n");
	}

	return 0;   
	
}
/*----------------------------------------------------------------------------*/
static void __exit afx133_exit(void)
{
	MSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(afx133_init);
module_exit(afx133_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Gary Huang");
MODULE_DESCRIPTION("AFX133 m-Sensor driver");
MODULE_LICENSE("VTC");
MODULE_VERSION(DRIVER_VERSION);

