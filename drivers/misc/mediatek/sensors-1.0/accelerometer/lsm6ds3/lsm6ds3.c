//liukangping@wind-mobi.com 20161108 begin
/* ST LSM6DS3 Accelerometer and Gyroscope sensor driver
 *
 *
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
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>

//#include <mach/mt_gpio.h>

//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
#include "lsm6ds3.h"
//#include <linux/hwmsen_helper.h>
#include <linux/kernel.h>
//#include <mach/mt_pm_ldo.h>
//#include <cust_eint.h>
//#include <mach/eint.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
//#include "core.h"
#include <asm/atomic.h>
#include <cust_acc.h>
#include <accel.h>
#include "step_counter.h"

#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif				/* #ifdef CUSTOM_KERNEL_SENSORHUB */

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define LSM6DS3_NEW_ARCH				//kk and L compatialbe
#define LSM6DS3_STEP_COUNTER 	//liuakangping		//it depends on the MACRO LSM6DS3_NEW_ARCH
//#define LSM6DS3_TILT_FUNC 				//dependency on LSM6DS3_STEP_COUNTER
#define LSM6DS3_SIGNIFICANT_MOTION  	//dependency on LSM6DS3_STEP_COUNTER

#ifndef LSM6DS3_NEW_ARCH		//new sensor type depend on new arch
#undef LSM6DS3_STEP_COUNTER
#undef LSM6DS3_TILT_FUNC
#undef LSM6DS3_SIGNIFICANT_MOTION
#endif


#ifndef LSM6DS3_STEP_COUNTER		//significant_motion depend on step_counter
#undef LSM6DS3_SIGNIFICANT_MOTION
#endif

#include <cust_acc.h>

#ifdef LSM6DS3_NEW_ARCH
#include <accel.h>

#if (defined(LSM6DS3_SIGNIFICANT_MOTION) ||defined(LSM6DS3_STEP_COUNTER))
#include "step_counter.h"
#endif
#ifdef LSM6DS3_TILT_FUNC //tilt detector
#include <tilt_detector.h>
#endif

#endif
/****************************************************************/
/* oujiacheng@wind-mobi.com 20160126 begin */
#ifdef KERNEL_NVRAM_READ
#include "../../../misc/mediatek/nvram_ctl/nvram_ctl.h"
#endif
/* oujiacheng@wind-mobi.com 20160126 end */

#if (defined(LSM6DS3_TILT_FUNC) ||defined(LSM6DS3_SIGNIFICANT_MOTION) ||defined(LSM6DS3_STEP_COUNTER))

#define GPIO_LSM6DS3_EINT_PIN 1
#define GPIO_LSM6DS3_EINT_PIN_M_EINT 1
#define CUST_EINT_LSM6DS3_NUM 1

#define CUST_EINT_LSM6DS3_DEBOUNCE_CN 1 //debounce time
#define CUST_EINT_LSM6DS3_TYPE 1	//eint trigger type

#define GPIO_LSM6DS3_EINT2_PIN 1
#define GPIO_LSM6DS3_EINT2_PIN_M_EINT 1
#define CUST_EINT2_LSM6DS3_NUM 1
#define CUST_EINT2_LSM6DS3_DEBOUNCE_CN 1 //debounce time

#endif
struct platform_device *stepPltFmDev;
int step_irq;
#if defined(LSM6DS3_TILT_FUNC) //|| defined(LSM6DS3_SIGNIFICANT_MOTION)	
#define GPIO_LSM6DS3_EINT_PIN GPIO_GSE_1_EINT_PIN		//eint gpio pin num
#define GPIO_LSM6DS3_EINT_PIN_M_EINT GPIO_GSE_1_EINT_PIN_M_EINT	//eint mode
#define CUST_EINT_LSM6DS3_NUM 67		//eint num
#define CUST_EINT_LSM6DS3_DEBOUNCE_CN CUST_EINT_GSE_1_DEBOUNCE_EN //debounce time
//#define CUST_EINT_LSM6DS3_TYPE CUST_EINT_GSE_1_TYPE	//eint trigger type
#endif

/*---------------------------------------------------------------------------*/
#define DEBUG 1
//yangzhigang@wind-mobi.com start 20160407
#define DEBUG_log 1
//yangzhigang@wind-mobi.com end
/*----------------------------------------------------------------------------*/
#define CONFIG_LSM6DS3_LOWPASS   /*apply low pass filter on output*/       
/*----------------------------------------------------------------------------*/
#define LSM6DS3_AXIS_X          0
#define LSM6DS3_AXIS_Y          1
#define LSM6DS3_AXIS_Z          2
#define LSM6DS3_ACC_AXES_NUM        3
#define LSM6DS3_GYRO_AXES_NUM       3
#define LSM6DS3_ACC_DATA_LEN        6   
#define LSM6DS3_GYRO_DATA_LEN       6   
#define LSM6DS3_ACC_DEV_NAME        "LSM6DS33"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lsm6ds3_i2c_id[] = {{LSM6DS3_ACC_DEV_NAME,0},{}};
// static struct i2c_board_info __initdata i2c_lsm6ds3={ I2C_BOARD_INFO(LSM6DS3_ACC_DEV_NAME, (0xD6>>1))};//liukangpin


/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lsm6ds3_i2c_remove(struct i2c_client *client);
static int lsm6ds3_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
//static int lsm6ds3_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int LSM6DS3_init_client(struct i2c_client *client, bool enable);
static int LSM6DS3_acc_SetPowerMode(struct i2c_client *client, bool enable);

static int LSM6DS3_ReadAccRawData(struct i2c_client *client, s16 data[LSM6DS3_ACC_AXES_NUM]);
#ifndef CONFIG_HAS_EARLYSUSPEND

static int lsm6ds3_acc_suspend(struct i2c_client *client, pm_message_t msg);
static int lsm6ds3_acc_resume(struct i2c_client *client);
#endif
//oujiacheng@wind-mobi.com 20150126 begin
/* oujiacheng@wind-mobi.com 20160117 modify begin */
#ifdef KERNEL_NVRAM_READ
static int NVRAM_ACC_DATA_SET=1;
static int acc_x_cali_value =0;
static int acc_y_cali_value =0;
static int acc_z_cali_value =0;
#endif
/* oujiacheng@wind-mobi.com 20160117 modify end */


//liukangping  add gsensor factory calibration 20161208 begin
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include "../../../../arch/powerpc/boot/stdlib.h"

#define GSENSOR_PROC_FOLDER "gsensor"
#define GSENSOR_PROC_DATA_FILE1 "value_x"
#define GSENSOR_PROC_DATA_FILE2 "value_y"
#define GSENSOR_PROC_DATA_FILE3 "value_z"

static struct proc_dir_entry *gsensor_proc_dir = NULL;
static struct proc_dir_entry *proc_data_file1 = NULL;
static struct proc_dir_entry *proc_data_file2 = NULL;
static struct proc_dir_entry *proc_data_file3 = NULL;

static int x=0;
static int y=0;
static int z=0;

//liukangping  add gsensor factory calibration 20161208 end

static ssize_t qiku_acc_calibration(acc_offset* qiku_acc_offset);  //liukangping
extern long ACC_NVRAM_RW(int cmd, __user acc_offset* buf, int length,int bit);
//oujiacheng@wind-mobi.com 20150126 end
static int LSM6DS3_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate);

static int LSM6DS3_acc_Enable_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_FUNC_EN_t newValue); //liukangping  qiangang
static int LSM6DS3_Int_Ctrl(struct i2c_client *client, LSM6DS3_ACC_GYRO_INT_ACTIVE_t int_act, LSM6DS3_ACC_GYRO_INT_LATCH_CTL_t int_latch);//liukangping  qiangang

#ifdef LSM6DS3_STEP_COUNTER //step counter
static int LSM6DS3_acc_Enable_Pedometer_Func(struct i2c_client *client, bool enable);

static int LSM6DS3_Write_PedoThreshold(struct i2c_client *client, u8 newValue);
//modify by qiangang@wind-mobi.com 20170920 begin
static int LSM6DS3_Reset_Pedo_Data(struct i2c_client *client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue);
//modify by qiangang@wind-mobi.com 20170920 end
#ifdef LSM6DS3_SIGNIFICANT_MOTION

//static int LSM6DS3_Enable_SigMotion_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_SIGN_MOT_t newValue);
#endif
#endif
#ifdef LSM6DS3_TILT_FUNC //tilt detector

static int LSM6DS3_Enable_Tilt_Func(struct i2c_client *client, bool enable);
static int LSM6DS3_Enable_Tilt_Func_On_Int(struct i2c_client *client, LSM6DS3_ACC_GYRO_ROUNT_INT_t tilt_int, bool enable);
#endif

#if (defined(LSM6DS3_TILT_FUNC) ||defined(LSM6DS3_SIGNIFICANT_MOTION) ||defined(LSM6DS3_STEP_COUNTER))
static int lsm6ds3_setup_eint(void);
#endif

//static struct acc_hw accel_cust;		// wangjun@wind-mobi.com 20170814
//static struct acc_hw *hw = &accel_cust;

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
    ACCEL_TRC_FILTER  = 0x01,
    ACCEL_TRC_RAWDATA = 0x02,
    ACCEL_TRC_IOCTL   = 0x04,
    ACCEL_TRC_CALI	= 0X08,
    ACCEL_TRC_INFO	= 0X10,
    ACCEL_TRC_DATA	= 0X20,
} ACCEL_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_ACC_AXES_NUM];
    int sum[LSM6DS3_ACC_AXES_NUM];
    int num;
    int idx;
};
struct gyro_data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_GYRO_AXES_NUM];
    int sum[LSM6DS3_GYRO_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct lsm6ds3_i2c_data {
    struct i2c_client *client;
	struct acc_hw hw;
    struct hwmsen_convert   cvt;
    atomic_t 				layout;
    /*misc*/
    //struct data_resolution *reso;
    struct work_struct	eint_work;				
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[LSM6DS3_GYRO_AXES_NUM+1];

    /*data*/
	s8                      offset[LSM6DS3_ACC_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[LSM6DS3_ACC_AXES_NUM+1];
	
	int 					sensitivity;
	int 					sample_rate;
	
	atomic_t    int1_request_num;

#if defined(CONFIG_LSM6DS3_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};


#ifdef LSM6DS3_NEW_ARCH
static int lsm6ds3_probe(struct platform_device *pdev) 
{
	stepPltFmDev = pdev;
	printk("qiangang is in 000\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_remove(struct platform_device *pdev)
{
    return 0;
}
#endif
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id sig_sensor_of_match[] = {
	{ .compatible = "mediatek,st_step_counter", },
	//{ .compatible = "mediatek,lsm6ds3_acc", },
	{},
};
#endif

#ifdef LSM6DS3_NEW_ARCH
static struct platform_driver lsm6ds3_driver = {
	.probe      = lsm6ds3_probe,
	.remove     = lsm6ds3_remove,    
	.driver     = {
			.name  = "stepcounter",
		//	.owner	= THIS_MODULE,
	#ifdef CONFIG_OF
			.of_match_table = sig_sensor_of_match,
	#endif
	}
};
#endif


/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,gsensor_lsm6da", },
	{},
};
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_driver lsm6ds3_i2c_driver = {
    .driver = {
        .owner          = THIS_MODULE,
        .name           = LSM6DS3_ACC_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
#endif
    },
	.probe      		= lsm6ds3_i2c_probe,
	.remove    			= lsm6ds3_i2c_remove,
	.detect				= lsm6ds3_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = lsm6ds3_acc_suspend,
    .resume             = lsm6ds3_acc_resume,
#endif
	.id_table = lsm6ds3_i2c_id,
};
#ifdef LSM6DS3_NEW_ARCH
static int lsm6ds3_local_init(void);
static int lsm6ds3_local_uninit(void);
static int lsm6ds3_acc_init_flag = -1;
static unsigned long lsm6ds3_init_flag_test = 0; //initial state
static DEFINE_MUTEX(lsm6ds3_init_mutex);
static DEFINE_MUTEX(lsm6ds3_acc_mutex);
typedef enum {
	LSM6DS3_ACC = 1,
	LSM6DS3_STEP_C = 2,
	LSM6DS3_TILT = 3,
}LSM6DS3_INIT_TYPE;
static struct acc_init_info  lsm6ds3_init_info = {
	.name   = LSM6DS3_ACC_DEV_NAME,
	.init   = lsm6ds3_local_init,
	.uninit = lsm6ds3_local_uninit,
};

#ifdef LSM6DS3_STEP_COUNTER
static int lsm6ds3_step_c_local_init(void);
static int lsm6ds3_step_c_local_uninit(void);
static struct step_c_init_info  lsm6ds3_step_c_init_info = {
	.name   = "LSM6DS3_STEP_C",
	.init   = lsm6ds3_step_c_local_init,
	.uninit = lsm6ds3_step_c_local_uninit,
};
#endif
#ifdef LSM6DS3_TILT_FUNC
static int lsm6ds3_tilt_local_init(void);
static int lsm6ds3_tilt_local_uninit(void);
static struct tilt_init_info  lsm6ds3_tilt_init_info = {
	.name   = "LSM6DS3_TILT",
	.init   = lsm6ds3_tilt_local_init,
	.uninit = lsm6ds3_tilt_local_uninit,
};
#endif

#endif
/*----------------------------------------------------------------------------*/
static struct i2c_client *lsm6ds3_i2c_client = NULL;

static struct lsm6ds3_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static bool enable_status = false;
static bool pedo_enable_status = false;
static bool tilt_enable_status = false;
static bool stepd_enable_status = false;
#ifdef LSM6DS3_SIGNIFICANT_MOTION
static bool sigm_enable_status = false;
#endif

/*----------------------------------------------------------------------------*/

#define GSE_TAG                  "[accel] "

#define GSE_FUN(f)               pr_debug(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    pr_err(KERN_ERR GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_debug(KERN_INFO GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)

/*----------------------------------------------------------------------------*/

static void LSM6DS3_dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x10;
  u8 regdata=0;
  for(i=0; i<80; i++)
  {
    //dump all
    hwmsen_read_byte(client,addr,&regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;	
  }
}

#if 0
static void LSM6DS3_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;
#if 0
	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "LSM6DS3"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "LSM6DS3"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
#endif
	power_on = on;    
}
#endif

/*----------------------------------------------------------------------------*/

static int LSM6DS3_acc_write_rel_calibration(struct lsm6ds3_i2c_data *obj, int dat[LSM6DS3_GYRO_AXES_NUM])
{
    obj->cali_sw[LSM6DS3_AXIS_X] = obj->cvt.sign[LSM6DS3_AXIS_X]*dat[obj->cvt.map[LSM6DS3_AXIS_X]];
    obj->cali_sw[LSM6DS3_AXIS_Y] = obj->cvt.sign[LSM6DS3_AXIS_Y]*dat[obj->cvt.map[LSM6DS3_AXIS_Y]];
    obj->cali_sw[LSM6DS3_AXIS_Z] = obj->cvt.sign[LSM6DS3_AXIS_Z]*dat[obj->cvt.map[LSM6DS3_AXIS_Z]];
#if DEBUG		
		if(atomic_read(&obj->trace) & ACCEL_TRC_CALI)
		{
			GSE_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n", 
				obj->cvt.sign[LSM6DS3_AXIS_X],obj->cvt.sign[LSM6DS3_AXIS_Y],obj->cvt.sign[LSM6DS3_AXIS_Z],
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				obj->cvt.map[LSM6DS3_AXIS_X],obj->cvt.map[LSM6DS3_AXIS_Y],obj->cvt.map[LSM6DS3_AXIS_Z]);
			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)\n", 
				obj->cali_sw[LSM6DS3_AXIS_X],obj->cali_sw[LSM6DS3_AXIS_Y],obj->cali_sw[LSM6DS3_AXIS_Z]);
		}
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_ResetCalibration(struct i2c_client *client)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;    
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_ReadCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
    struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->cali_sw[LSM6DS3_AXIS_X];
    dat[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->cali_sw[LSM6DS3_AXIS_Y];
    dat[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->cali_sw[LSM6DS3_AXIS_Z];

#if DEBUG		
		if(atomic_read(&obj->trace) & ACCEL_TRC_CALI)
		{
			GSE_LOG("Read gyro calibration data  (%5d, %5d, %5d)\n", 
				dat[LSM6DS3_AXIS_X],dat[LSM6DS3_AXIS_Y],dat[LSM6DS3_AXIS_Z]);
		}
#endif
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/

static int LSM6DS3_acc_WriteCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[LSM6DS3_GYRO_AXES_NUM];
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end
	if(!obj || ! dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        		
		cali[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->cali_sw[LSM6DS3_AXIS_X];
		cali[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->cali_sw[LSM6DS3_AXIS_Y];
		cali[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->cali_sw[LSM6DS3_AXIS_Z]; 
		cali[LSM6DS3_AXIS_X] += dat[LSM6DS3_AXIS_X];
		cali[LSM6DS3_AXIS_Y] += dat[LSM6DS3_AXIS_Y];
		cali[LSM6DS3_AXIS_Z] += dat[LSM6DS3_AXIS_Z];
#if DEBUG		
		if(atomic_read(&obj->trace) & ACCEL_TRC_CALI)
		{
			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n", 
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				cali[LSM6DS3_AXIS_X],cali[LSM6DS3_AXIS_Y],cali[LSM6DS3_AXIS_Z]);
		}
#endif
		return LSM6DS3_acc_write_rel_calibration(obj, cali);
	} 

	return err;
}
/*----------------------------------------------------------------------------*/
static int LSM6DS3_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = LSM6DS3_FIXED_DEVID;    

	res = hwmsen_read_byte(client,LSM6DS3_WHO_AM_I,databuf);
    GSE_LOG(" LSM6DS3  id %x!\n",databuf[0]);
	
	if(databuf[0]!=LSM6DS3_FIXED_DEVID)
	{
		return LSM6DS3_ERR_IDENTIFICATION;
	}
   
	if (res < 0)
	{
		return LSM6DS3_ERR_I2C;
	}
	
	return LSM6DS3_SUCCESS;
}

#ifdef LSM6DS3_TILT_FUNC //tilt detector
static int LSM6DS3_enable_tilt(struct i2c_client *client, bool enable)
{
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);//obj_i2c_data;
	
	if(enable)
	{
		//set ODR to 26 hz		
		//res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_26HZ);
		res = LSM6DS3_acc_SetSampleRate(client, obj->sample_rate);
		if(LSM6DS3_SUCCESS == res)
		{
			GSE_LOG(" %s set 26hz odr to acc\n", __func__);
		}			
		
		res = LSM6DS3_Enable_Tilt_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Enable_Tilt_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		
		res = LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);	
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}		
		
		res = LSM6DS3_Enable_Tilt_Func_On_Int(client, LSM6DS3_ACC_GYRO_INT1, true);  //default route to INT1 	
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Enable_Tilt_Func_On_Int failed!\n");
			return LSM6DS3_ERR_STATUS;
		}	
		while ( atomic_read(&obj->int1_request_num) >=1){
		
		    enable_irq(step_irq);
		    atomic_dec(&obj->int1_request_num);
			GSE_LOG(" sigMotion enable with2 sigm_enable_status=%d,stepd_enable_status = %d,irq times %d  \n", sigm_enable_status,stepd_enable_status, atomic_read(&obj->int1_request_num));
		}
	}
	else
	{
		res = LSM6DS3_Enable_Tilt_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Enable_Tilt_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		if(!enable_status && !pedo_enable_status && !sigm_enable_status && !stepd_enable_status)
		{   
		    mutex_lock(&lsm6ds3_acc_mutex);
			res = LSM6DS3_acc_SetPowerMode(client, false);
		    mutex_unlock(&lsm6ds3_acc_mutex);
			if(res != LSM6DS3_SUCCESS)
			{
				GSE_LOG(" LSM6DS3_acc_SetPowerMode failed!\n");
				return LSM6DS3_ERR_STATUS;
			}
		}
		//disable_irq(CUST_EINT_LSM6DS3_NUM);
	if (( sigm_enable_status ==false) &&( stepd_enable_status ==false) )
		{
			cancel_work_sync(&obj->eint_work);
			disable_irq(step_irq);
			atomic_inc(&obj->int1_request_num);
		}
	}
//	tilt_enable_status = enable;
	return LSM6DS3_SUCCESS;
}
#endif




/******************************liukangping  add gsensor factory calibration 20161208 begin********************************/

static ssize_t data_write3(struct file *file, const char *buf, size_t len, loff_t *pos)
{
    int data_read_enable = 0;	
    sscanf(buf, "%d", &data_read_enable);
	z = data_read_enable;	
  /*  if(0<=X1&&X1<=300 && 300<=X2&&X2<=1000 && 1000<=X3&&X3<=3000)
    {
		//printk("%d\n",);
	    dynk_low_offset = X2-X1; 
        dynk_high_offset = X3-X1;
		printk("wind-liukangping dynk_low_offset= %d\n",dynk_low_offset);
		printk("wind-liukangping dynk_low_offset=%d\n",dynk_high_offset);
    }
	*/
	return len;
		
}
static ssize_t data_read3(struct file *file, char *buf, size_t len, loff_t *pos)
{
	size_t size = 0;
	int count = 0 ;
	static int DATA_SEND_FLAG = 0;
	
	count = z;
	if (!DATA_SEND_FLAG)
	{	
	    size = sprintf(buf, "%d\n", count);
	    DATA_SEND_FLAG=1;
	}
	
	else
	{
		DATA_SEND_FLAG=0;
	}
	return size;
		
}

static struct file_operations proc_data_file3_ops =
{
	.owner = THIS_MODULE,
	.write = data_write3,
	.read = data_read3,
};


static ssize_t data_write2(struct file *file, const char *buf, size_t len, loff_t *pos)
{
    int data_read_enable = 0;	 
    sscanf(buf, "%d", &data_read_enable);
	y = data_read_enable;
	return len;
	
	
		
}

static ssize_t data_read2(struct file *file, char *buf, size_t len, loff_t *pos)
{

	size_t size = 0;
	int count = 0 ;
	static int DATA_SEND_FLAG = 0;
	
	count = y;
	if (!DATA_SEND_FLAG)
	{	
	    size = sprintf(buf, "%d\n", count);
	    DATA_SEND_FLAG=1;
	}
	
	else
	{
		DATA_SEND_FLAG=0;
	}
	return size;
		
}
static struct file_operations proc_data_file2_ops =
{
	.owner = THIS_MODULE,
	.write = data_write2,
	.read = data_read2,
};


static ssize_t data_write1(struct file *file, const char *buf, size_t len, loff_t *pos)
{
	//size_t size = 0;
     int data_read_enable = 0;	 
    sscanf(buf, "%d", &data_read_enable);

	x = data_read_enable;
	//size = sizeof(buf);
	return len;
		
}

static ssize_t data_read1(struct file *file, char *buf, size_t len, loff_t *pos)
{
	size_t size = 0;
	int count = 0 ;
	static int DATA_SEND_FLAG = 0;
	
	count = x;
	
	if (!DATA_SEND_FLAG)
	{	
	    size = sprintf(buf, "%d\n", count);
	    DATA_SEND_FLAG=1;
	}
	
	else
	{
		DATA_SEND_FLAG=0;
	}
	return size;
		
}

static struct file_operations proc_data_file1_ops =
{
	.owner = THIS_MODULE,
	.write = data_write1,
	.read = data_read1,
};


static int gsensor_proc_init(void)
{
	gsensor_proc_dir = proc_mkdir(GSENSOR_PROC_FOLDER, NULL);
	if (gsensor_proc_dir == NULL)
	{
		printk(" %s: gsensor_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
/////////////////////add data 1
	proc_data_file1 = proc_create(GSENSOR_PROC_DATA_FILE1, (S_IWUSR|S_IRUGO|S_IWUGO), 
		gsensor_proc_dir, &proc_data_file1_ops);
	if(proc_data_file1 == NULL)
	{
		printk(" %s: proc data file1 create failed!\n", __func__);
		remove_proc_entry( GSENSOR_PROC_DATA_FILE1, gsensor_proc_dir );
		return -ENOMEM;
	}
	
 ////////////////////add data 2
	proc_data_file2 = proc_create(GSENSOR_PROC_DATA_FILE2, (S_IWUSR|S_IRUGO|S_IWUGO), 
		gsensor_proc_dir, &proc_data_file2_ops);
	if(proc_data_file2 == NULL)
	{
		printk(" %s: proc data file2 create failed!\n", __func__);
		remove_proc_entry( GSENSOR_PROC_DATA_FILE2, gsensor_proc_dir );
		return -ENOMEM;
	}
	//////////////////add data 3
	proc_data_file3 = proc_create(GSENSOR_PROC_DATA_FILE3, (S_IWUSR|S_IRUGO|S_IWUGO), 
		gsensor_proc_dir, &proc_data_file3_ops);
		
	if(proc_data_file3 == NULL)
	{
		printk(" %s: proc data file3 create failed!\n", __func__);
		remove_proc_entry( GSENSOR_PROC_DATA_FILE3, gsensor_proc_dir );
		return -ENOMEM;
	}
	
	return 0 ;
}

static void gsensor_proc_remove(void)
{
    //remove_proc_entry( GSENSOR_PROC_FOLDER, gsensor_proc_dir );
	remove_proc_entry( GSENSOR_PROC_DATA_FILE1, gsensor_proc_dir );
	remove_proc_entry( GSENSOR_PROC_DATA_FILE2, gsensor_proc_dir );
	remove_proc_entry( GSENSOR_PROC_DATA_FILE2, gsensor_proc_dir );
}

/******************************liukangping  add gsensor factory calibration 20161208 end********************************/







#ifdef LSM6DS3_STEP_COUNTER //step counter
static int LSM6DS3_enable_pedo(struct i2c_client *client, bool enable)
{
//	u8 databuf[2] = {0};    
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);//obj_i2c_data;

	if(true == enable)
	{	
		//software reset
		//set ODR to 26 hz		
		//res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_26HZ);
		res = LSM6DS3_acc_SetSampleRate(client, obj->sample_rate);
		if(LSM6DS3_SUCCESS == res)
		{
			GSE_LOG(" %s set 26hz odr to acc\n", __func__);
		}
		//enable tilt feature and pedometer feature
		res = LSM6DS3_acc_Enable_Pedometer_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Pedometer_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		
		res = LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);	
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		printk("fengqian LSM6DS3_enable_pedo\n");

		
		//modify by qiangang@wind-mobi.com 20170911 begin				
   #if 0
		res = LSM6DS3_Write_PedoThreshold(client, 0x0f);// set threshold to a certain value here  0x11
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Write_PedoThreshold failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		res = LSM6DS3_Reset_Pedo_Data(client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
		
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Reset_Pedo_Data failed!\n");
			return LSM6DS3_ERR_STATUS;
		}  
	#endif
	//modify by qiangang@wind-mobi.com 20170911 end	
	}
	else
	{
		res = LSM6DS3_acc_Enable_Pedometer_Func(client, enable);
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Func failed at disable pedo!\n");
			return LSM6DS3_ERR_STATUS;
		}
		//do not turn off the func
		if(!enable_status && !tilt_enable_status)
		{
			res = LSM6DS3_acc_SetPowerMode(client,false);
			if(res != LSM6DS3_SUCCESS)
			{
				GSE_LOG(" LSM6DS3_acc_SetPowerMode failed at disable pedo!\n");
				return LSM6DS3_ERR_STATUS;
			}
		}
	}
	return LSM6DS3_SUCCESS;
}
#endif

static int LSM6DS3_acc_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);//obj_i2c_data;

	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return LSM6DS3_SUCCESS;
	}

	if(hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf))
	{
		GSE_ERR("read lsm6ds3 power ctl register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	GSE_LOG("LSM6DS3_CTRL1_XL:databuf[0] =  %x!\n", databuf[0]);


	if(true == enable)
	{
		databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;//clear lsm6ds3 gyro ODR bits
		databuf[0] |= obj->sample_rate;//LSM6DS3_ACC_ODR_104HZ; //default set 100HZ for LSM6DS3 acc
	}
	else
	{
		// do nothing
		databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;//clear lsm6ds3 acc ODR bits
		databuf[0] |= LSM6DS3_ACC_ODR_POWER_DOWN;
	}
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL;    
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_LOG("LSM6DS3 set power mode: ODR 100hz failed!\n");
		return LSM6DS3_ERR_I2C;
	}	
	else
	{
		GSE_LOG("set LSM6DS3 gyro power mode:ODR 100HZ ok %d!\n", enable);
	}	

	sensor_power = enable;
	
	return LSM6DS3_SUCCESS;    
}


/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_SetFullScale(struct i2c_client *client, u8 acc_fs)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end     
		
	if(hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf))
	{
		GSE_ERR("read LSM6DS3_CTRL1_XL err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_CTRL1_XL register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_RANGE_MASK;//clear 
	databuf[0] |= acc_fs;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL; 
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write full scale register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	switch(acc_fs)
	{
		case LSM6DS3_ACC_RANGE_2g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_2G;
			break;
		case LSM6DS3_ACC_RANGE_4g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_4G;
			break;
		case LSM6DS3_ACC_RANGE_8g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_8G;
			break;
		case LSM6DS3_ACC_RANGE_16g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_16G;
			break;
		default:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_2G;
			break;
	}

	if(hwmsen_read_byte(client, LSM6DS3_CTRL9_XL, databuf))
	{
		GSE_ERR("read LSM6DS3_CTRL9_XL err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_CTRL9_XL register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_ENABLE_AXIS_MASK;//clear 
	databuf[0] |= LSM6DS3_ACC_ENABLE_AXIS_X | LSM6DS3_ACC_ENABLE_AXIS_Y| LSM6DS3_ACC_ENABLE_AXIS_Z;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL9_XL; 
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write full scale register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
// set the acc sample rate
static int LSM6DS3_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end
	res = LSM6DS3_acc_SetPowerMode(client, true);	//set Sample Rate will enable power and should changed power status
	if(res != LSM6DS3_SUCCESS)	
	{
		return res;
	}

	if(hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;//clear 
	databuf[0] |= sample_rate;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL; 
		
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write sample rate register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}

#ifdef LSM6DS3_TILT_FUNC //tilt detector
static int LSM6DS3_Enable_Tilt_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(hwmsen_read_byte(client, LSM6DS3_TAP_CFG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LSM6DS3_TILT_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_TILT_EN_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_TILT_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_TILT_EN_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_TAP_CFG; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS; 
}
#endif

#ifdef LSM6DS3_SIGNIFICANT_MOTION
static int LSM6DS3_Enable_SigMotion_Func_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
	
	//LSM6DS3_ACC_GYRO_FUNC_EN_t func_enable;
	//LSM6DS3_ACC_GYRO_SIGN_MOT_t sigm_enable;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    

#if 0
	if(enable)
	{
		func_enable = LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED;
		sigm_enable = LSM6DS3_ACC_GYRO_SIGN_MOT_ENABLED;
		
		res = LSM6DS3_acc_Enable_Func(client, func_enable);	   //bug here need notice
		if(res != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}	
	}
	else
	{
		//func_enable = LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED;
		sigm_enable = LSM6DS3_ACC_GYRO_SIGN_MOT_DISABLED;
	}		
	
	res = LSM6DS3_Enable_SigMotion_Func(client, sigm_enable);	
	if(res != LSM6DS3_SUCCESS)
	{
		GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
		return LSM6DS3_ERR_STATUS;
	}	
	
	//Config interrupt for significant motion

	op_reg = LSM6DS3_INT1_CTRL;
		
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}
	
	if(enable)
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_SIGN_MOT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_SIGN_MOT_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_SIGN_MOT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_SIGN_MOT_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}	
#endif

    //Config interrupt for significant motion  , enable/disable WU bit IN MD1

	op_reg = LSM6DS3_MD1_CFG;
		
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_MD1_CFG register: 0x%x\n", databuf[0]);
	}
	
	if(enable)
	{
		databuf[0] &= ~0x20;//clear 
		databuf[0] |= 0x20;			
	}
	else
	{
		databuf[0] &= ~0x20;//clear 
		databuf[0] |= 0x00;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}	
	
	res = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_HIGH, LSM6DS3_ACC_GYRO_INT_LATCH);    //default low
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}	
	return LSM6DS3_SUCCESS; 
}
#endif

static int LSM6DS3_Int_Ctrl(struct i2c_client *client, LSM6DS3_ACC_GYRO_INT_ACTIVE_t int_act, LSM6DS3_ACC_GYRO_INT_LATCH_CTL_t int_latch)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    

	//config latch int or no latch
	op_reg = LSM6DS3_TAP_CFG;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}
	
	databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_LATCH_CTL_MASK;//clear 
	databuf[0] |= int_latch;			
		
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	// config high or low active
	op_reg = LSM6DS3_CTRL3_C;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}
	
	databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_ACTIVE_MASK;//clear 
	databuf[0] |= int_act;			
		
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS; 
}

#ifdef LSM6DS3_TILT_FUNC //tilt detector
static int LSM6DS3_Enable_Tilt_Func_On_Int(struct i2c_client *client, LSM6DS3_ACC_GYRO_ROUNT_INT_t tilt_int, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(LSM6DS3_ACC_GYRO_INT1 == tilt_int)
	{
		op_reg = LSM6DS3_MD1_CFG;
	}
	else if(LSM6DS3_ACC_GYRO_INT2 == tilt_int)
	{
		op_reg = LSM6DS3_MD2_CFG;
	}
	
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}
	
	if(enable)
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_TILT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_TILT_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_TILT_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_TILT_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = op_reg; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	res = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_LOW, LSM6DS3_ACC_GYRO_INT_LATCH);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS; 
}
#endif

#ifdef LSM6DS3_STEP_COUNTER //step counter
static int LSM6DS3_acc_Enable_Pedometer_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(hwmsen_read_byte(client, LSM6DS3_TAP_CFG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LSM6DS3_PEDO_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED;			
	}
	else
	{
		databuf[0] &= ~LSM6DS3_PEDO_EN_MASK;//clear 
		databuf[0] |= LSM6DS3_ACC_GYRO_PEDO_EN_DISABLED;		
	}
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_TAP_CFG; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable pedometer func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}

#ifdef LSM6DS3_SIGNIFICANT_MOTION
static int LSM6DS3_Set_SigMotion_Threshold(struct i2c_client *client, u8 SigMotion_Threshold)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    

    
	if(hwmsen_read_byte(client, LSM6DS3_WAKE_UP_THS, databuf))
		{
			GSE_ERR("read acc data format register err!\n");
			return LSM6DS3_ERR_I2C;
		}
		else
		{
			GSE_LOG("read  LSM6DS3_WAKE_UP_THS register: 0x%x\n", databuf[0]);
		}
		
	databuf[1] &= ~0x3f;//clear 
	databuf[1] |= SigMotion_Threshold; 	
	databuf[0] = LSM6DS3_WAKE_UP_THS; 
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_WAKE_UP_THS register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}


	if(hwmsen_read_byte(client, LSM6DS3_WAKE_UP_DUR, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_WAKE_UP_DUR register: 0x%x\n", databuf[0]);
	}

	databuf[1] &= ~0x60;//clear 
	databuf[1] |= 0x40;			
	databuf[0] = LSM6DS3_WAKE_UP_DUR;//clear 
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_WAKE_UP_DUR register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

    if(hwmsen_read_byte(client, LSM6DS3_TAP_CFG, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_TAP_CFG register: 0x%x\n", databuf[0]);
	}

	databuf[1] &= ~0x10;//clear  
	databuf[1] |= 0x00;			 // slope filter
	databuf[0] = LSM6DS3_TAP_CFG;//clear 
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_TAP_CFG register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	return LSM6DS3_SUCCESS;    
}

#if 0
static int LSM6DS3_Enable_SigMotion_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_SIGN_MOT_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_ACC_GYRO_SIGN_MOT_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}
#endif 
#endif
#endif

static int LSM6DS3_acc_Enable_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_FUNC_EN_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	
	databuf[0] &= ~0x02;//always clear reset bit, add by qiangang 
	databuf[0] &= ~LSM6DS3_ACC_GYRO_FUNC_EN_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;    
}

#ifdef LSM6DS3_STEP_COUNTER //step counter
static int LSM6DS3_W_Open_RAM_Page(struct i2c_client *client, LSM6DS3_ACC_GYRO_RAM_PAGE_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(hwmsen_read_byte(client, LSM6DS3_RAM_ACCESS, databuf))
	{
		GSE_ERR("%s read LSM6DS3_RAM_ACCESS register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_RAM_PAGE_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_RAM_ACCESS; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_RAM_ACCESS register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Write_PedoThreshold(struct i2c_client *client, u8 newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_ENABLED);
	if(LSM6DS3_SUCCESS != res)
	{
		return res;
	}
	if(hwmsen_read_byte(client, LSM6DS3_CONFIG_PEDO_THS_MIN, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	
	databuf[0] &= ~0x1F; 
	databuf[0] |= ((newValue & 0x1F) | 0x80);      //force set to 4g mode,add by qiangang
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CONFIG_PEDO_THS_MIN; 	
	res = i2c_master_send(client, databuf, 0x2);

	GSE_LOG("%s write threshold value 0x%x\n", __func__, databuf[0]);	//add by qiangang@wind-mobi.com 20170911 
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	
	databuf[0] = 0x14;
	databuf[1] = 0x4f;                   // change debounce time from 1101 （ 0xd ） to 1010 （0xa） , debouce  0x0a = 10 , 11* 80ms  = 800ms, 可以改变debounce time 的时间，减小debouce time 来进一步约束公交车状态
                                          //	7step debounce , and 1101    default 0x6e
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED);
	if(LSM6DS3_SUCCESS != res)
	{
		GSE_ERR("%s write LSM6DS3_W_Open_RAM_Page failed!\n", __func__);
		return res;
	}
	
	return LSM6DS3_SUCCESS; 
}
//modify by qiangang@wind-mobi.com 20170920 begin

static int LSM6DS3_Reset_Pedo_Data(struct i2c_client *client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue)
{
	u8 databuf[2] = {0}; 
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc LSM6DS3_CTRL10_C data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_PEDO_RST_STEP_MASK;//clear 
	databuf[0] |= newValue;
	
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

//modify by qiangang@wind-mobi.com 20170920 end
static int LSM6DS3_Get_Pedo_DataReg(struct i2c_client *client, u16 *Value)
{
	u8 databuf[2] = {0}; 
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    
	
	if(hwmsen_read_block(client, LSM6DS3_STEP_COUNTER_L, databuf, 2))
	{
		GSE_ERR("LSM6DS3 read acc data  error\n");
		return -2;
	}

	*Value = (databuf[1]<<8)|databuf[0];	
	printk("fengqian LSM6DS3_Get_Pedo_DataReg %d \n",*Value);

	return LSM6DS3_SUCCESS;
}
#endif

//chenjiehua@wind-mobi.com change start 20160316
#ifdef CONFIG_MEITU_DRV_CAMERA
static int LSM6DS3_CompassReadAccData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LSM6DS3_ACC_AXES_NUM];
	int res = 0;
	
/* oujiacheng@wind-mobi.com 20160126  begin */
	#ifdef KERNEL_NVRAM_READ
	//void __user *data = (void __user*) arg;
	acc_offset offset;
	memset(&offset,0,sizeof(acc_offset));
	acc_offset *qk_acc_offset; 
	qk_acc_offset=&offset;
	#endif
/* oujiacheng@wind-mobi.com 20160126  end */
	
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = LSM6DS3_acc_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on lsm6ds3 error %d!\n", res);
		}
		msleep(20);
	}
	
	res = LSM6DS3_ReadAccRawData(client, obj->data);
	if(res < 0)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//obj->data[LSM6DS3_AXIS_X] += obj->cali_sw[LSM6DS3_AXIS_X];
		//obj->data[LSM6DS3_AXIS_Y] += obj->cali_sw[LSM6DS3_AXIS_Y];
		//obj->data[LSM6DS3_AXIS_Z] += obj->cali_sw[LSM6DS3_AXIS_Z];
		printk("qiangang calibration 111");
		obj->data[LSM6DS3_AXIS_X] += x;
		obj->data[LSM6DS3_AXIS_Y] += y;
		obj->data[LSM6DS3_AXIS_Z] += z;
		
		/*remap coordinate*/
		acc[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->data[LSM6DS3_AXIS_X];
		acc[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->data[LSM6DS3_AXIS_Y];
		acc[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->data[LSM6DS3_AXIS_Z];
	printk("qiangang calibration 222");
		sprintf(buf, "%d %d %d", (s16)acc[LSM6DS3_AXIS_X], (s16)acc[LSM6DS3_AXIS_Y], (s16)acc[LSM6DS3_AXIS_Z]);
	//	sprintf(buf, "%04x %04x %04x", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);
	
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
		{
			//GSE_LOG("gsensor data: %s!\n", buf);
			GSE_LOG("raw data:obj->data:%04x %04x %04x\n", obj->data[LSM6DS3_AXIS_X], obj->data[LSM6DS3_AXIS_Y], obj->data[LSM6DS3_AXIS_Z]);
			GSE_LOG("acc:%04x %04x %04x\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);
			printk("qiangang calibration 333");
			//LSM6DS3_dumpReg(client);
		}
	}
	
	return 0;
}
#endif
//chenjiehua@wind-mobi.com change end
/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadAccData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LSM6DS3_ACC_AXES_NUM];
	int res = 0;
	u8 databuf_gensor[2] = {0}; 
/* oujiacheng@wind-mobi.com 20160126  begin */
	#ifdef KERNEL_NVRAM_READ
	//void __user *data = (void __user*) arg;
	acc_offset offset;
	memset(&offset,0,sizeof(acc_offset));
	acc_offset *qk_acc_offset; 
	qk_acc_offset=&offset;
	#endif
/* oujiacheng@wind-mobi.com 20160126  end */
	
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = LSM6DS3_acc_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on lsm6ds3 error %d!\n", res);
		}
		msleep(20);
	}

	/////liukangping read LSM6DS3_CTRL1_XL begin
	  if((hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf_gensor)==0))
	{
	
	   
	   if( ((databuf_gensor[0] &= 0x08) !=0x08 ))
	   	{
	      printk("wind-liukangping LSM6DS3 set LSM6DS3_CTRL1_XL  value 11=%x\n",databuf_gensor[0]);
		  res = LSM6DS3_acc_SetFullScale(client,LSM6DS3_ACC_RANGE_4g);  //set oly this choice 
		  hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf_gensor);
		  printk("wind-liukangping LSM6DS3 set LSM6DS3_CTRL1_XL  value 22=%x\n",databuf_gensor[0]);
		
	   	}
	    
	}	
	/////liukangping LSM6DS3_CTRL1_XL read end
	res = LSM6DS3_ReadAccRawData(client, obj->data);
	if(res < 0)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
	#if 1
		obj->data[LSM6DS3_AXIS_X] = (long)(obj->data[LSM6DS3_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000); //NTC
		obj->data[LSM6DS3_AXIS_Y] = (long)(obj->data[LSM6DS3_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		obj->data[LSM6DS3_AXIS_Z] = (long)(obj->data[LSM6DS3_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		
		obj->data[LSM6DS3_AXIS_X] += obj->cali_sw[LSM6DS3_AXIS_X];
		obj->data[LSM6DS3_AXIS_Y] += obj->cali_sw[LSM6DS3_AXIS_Y];
		obj->data[LSM6DS3_AXIS_Z] += obj->cali_sw[LSM6DS3_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->data[LSM6DS3_AXIS_X];
		acc[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->data[LSM6DS3_AXIS_Y];
		acc[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->data[LSM6DS3_AXIS_Z];
		
		//printk("qiangang calibration 111");
		//printk("qiangang calibration GSEBSOR NO value=%d\n", x);
		//printk("qiangang calibration GSEBSOR NO  value=%d\n", y);
		//printk("qiangang calibration GSEBSOR NO value=%d\n", z);
		//acc[obj->cvt.map[LSM6DS3_AXIS_X]] += (-x);
		//acc[obj->cvt.map[LSM6DS3_AXIS_Y]] += (-y);
		//acc[obj->cvt.map[LSM6DS3_AXIS_Z]] += (-z);
//add by qiangang 20170824 begin		
		acc[LSM6DS3_AXIS_X] -= x;
		acc[LSM6DS3_AXIS_Y] -= y;
		acc[LSM6DS3_AXIS_Z] -= z;
//add by qiangang 20170824 end		
		printk("GSEBSOR X=%d,Y=%d,Z=%d\n", acc[obj->cvt.map[LSM6DS3_AXIS_X]],acc[obj->cvt.map[LSM6DS3_AXIS_Y]],acc[obj->cvt.map[LSM6DS3_AXIS_Z]]);
		

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);

		//Out put the mg
		/*
		acc[LSM6DS3_AXIS_X] = acc[LSM6DS3_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LSM6DS3_AXIS_Y] = acc[LSM6DS3_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LSM6DS3_AXIS_Z] = acc[LSM6DS3_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
		*/
	#endif


		sprintf(buf, "%04x %04x %04x", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);
	
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
		{
			//GSE_LOG("gsensor data: %s!\n", buf);
			GSE_LOG("raw data:obj->data:%04x %04x %04x\n", obj->data[LSM6DS3_AXIS_X], obj->data[LSM6DS3_AXIS_Y], obj->data[LSM6DS3_AXIS_Z]);
			GSE_LOG("acc:%04x %04x %04x\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);
	
			//LSM6DS3_dumpReg(client);
		}
/* oujiacheng@wind-mobi.com 20160126  begin */
		#ifdef KERNEL_NVRAM_READ
		if(NVRAM_ACC_DATA_SET)
		{	
			NVRAM_ACC_DATA_SET=0;
			if(!(ACC_NVRAM_RW(NVRAM_READ,(__user acc_offset*)qk_acc_offset,ACC_RW_DATA_LEN,ACC_RW_DATA_START)))
			{
				//acc_x_cali_value=qk_acc_offset->x;
				//acc_y_cali_value=qk_acc_offset->y;
				//acc_z_cali_value=qk_acc_offset->z;
				printk("qiangang111 ACC get NVRAM data error!\n");
				acc_x_cali_value=x;
				acc_y_cali_value=y;
				acc_z_cali_value=z;
				
				printk("[kevin]ACC get NVRAM data succes!\n");
			}
			else
				printk("[kevin]ACC get NVRAM data error!\n");
		
		}
printk("qiangang2222 ACC get NVRAM data error!\n");
		acc[obj->cvt.map[LSM6DS3_AXIS_X]] +=acc_x_cali_value;
		acc[obj->cvt.map[LSM6DS3_AXIS_Y]] +=acc_y_cali_value;
		acc[obj->cvt.map[LSM6DS3_AXIS_Z]] +=acc_z_cali_value;
		printk("qiangang3333  ACC get NVRAM data error!\n");
		#endif
/* oujiacheng@wind-mobi.com 20160126  end */
	}
	
	return 0;
}
static int LSM6DS3_ReadAccRawData(struct i2c_client *client, s16 data[LSM6DS3_ACC_AXES_NUM])
{
	int err = 0;
	char databuf[6] = {0};

	if(NULL == client)
	{
		err = -EINVAL;
	}	
	else
	{
		if(hwmsen_read_block(client, LSM6DS3_OUTX_L_XL, databuf, 6))
		{
			GSE_ERR("LSM6DS3 read acc data  error\n");
			return -2;
		}
		else
		{
			data[LSM6DS3_AXIS_X] = (s16)((databuf[LSM6DS3_AXIS_X*2+1] << 8) | (databuf[LSM6DS3_AXIS_X*2]));
			data[LSM6DS3_AXIS_Y] = (s16)((databuf[LSM6DS3_AXIS_Y*2+1] << 8) | (databuf[LSM6DS3_AXIS_Y*2]));
			data[LSM6DS3_AXIS_Z] = (s16)((databuf[LSM6DS3_AXIS_Z*2+1] << 8) | (databuf[LSM6DS3_AXIS_Z*2]));	
		}      
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LSM6DS3 Chip");
	return 0;
}

//oujiacheng@wind-mobi.com 20160126 begin
/*If use wind-mobi cali must close KIONIX_AUTO_CAL*/
//#define KIONIX_AUTO_CAL     //Setup AUTO-Cali parameter
//oujiacheng@wind-mobi.com 20160126 end
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LSM6DS3_ReadChipInfo(client, strbuf, LSM6DS3_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];
	int x,y,z;
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LSM6DS3_ReadAccData(client, strbuf, LSM6DS3_BUFSIZE);
	sscanf(strbuf, "%x %x %x", &x, &y, &z);	
	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n", x,y,z);            
}
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	s16 data[LSM6DS3_ACC_AXES_NUM] = {0};
	
	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LSM6DS3_ReadAccRawData(client, data);
	return snprintf(buf, PAGE_SIZE, "%x,%x,%x\n", data[0],data[1],data[2]);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	
	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	if (1 == sscanf(buf, "0x%x", &trace)) {
		atomic_set(&obj->trace, trace);
	} else {
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	
	return count;    
}
static ssize_t show_chipinit_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;

	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	LSM6DS3_init_client(obj->client, true);
	LSM6DS3_dumpReg(obj->client);
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	//if (obj->hw) {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
	            obj->hw.i2c_num, obj->hw.direction, obj->sensitivity, obj->hw.power_id, obj->hw.power_vol);   
	LSM6DS3_dumpReg(obj->client);
	//} else {
	//	len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	//}
	return len;    
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct lsm6ds3_i2c_data *data = obj_i2c_data;
	if(NULL == data)
	{
		printk(KERN_ERR "lsm6ds3_i2c_data is null!!\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw.direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	struct lsm6ds3_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		printk(KERN_ERR "lsm6ds3_i2c_data is null!!\n");
		return count;
	}

	

	if (1 == sscanf(buf, "%d", &layout)) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw.direction, &data->cvt)) {
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw.direction);
		} else {
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw.direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}

//chenjiehua@wind-mobi.com change start 20160310
#ifdef CONFIG_MEITU_DRV_CAMERA //added by evanzeng @20150506
ssize_t get_gsensor_value_to_m12mo(char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];
	if(NULL == client)
	{
		ACC_ERR("i2c client is null!!\n");
		return 0;
	}
	if(NULL == buf)
	{
		ACC_ERR("buf point is null!!\n");
		return 0;
	}
	LSM6DS3_CompassReadAccData(client, strbuf, LSM6DS3_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
#endif
//chenjiehua@wind-mobi.com change end

/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensorrawdata,           S_IRUGO, show_sensorrawdata_value,    NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(chipinit,      S_IWUSR | S_IRUGO, show_chipinit_value,         store_chipinit_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *LSM6DS3_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/	
	&driver_attr_sensorrawdata,   /*dump sensor raw data*/	
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,  
	&driver_attr_chipinit,
	&driver_attr_layout,
};
/*----------------------------------------------------------------------------*/
static int lsm6ds3_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(LSM6DS3_attr_list)/sizeof(LSM6DS3_attr_list[0]));
	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		if (0 != (err = driver_create_file(driver,  LSM6DS3_attr_list[idx]))) {            
			GSE_ERR("driver_create_file (%s) = %d\n",  LSM6DS3_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof( LSM6DS3_attr_list)/sizeof( LSM6DS3_attr_list[0]));

	if (driver == NULL) {
		return -EINVAL;
	}	

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver,  LSM6DS3_attr_list[idx]);
	}
	return err;
}
static int LSM6DS3_Set_RegInc(struct i2c_client *client, bool inc)
{
	u8 databuf[2] = {0};    
	int res = 0;
	//GSE_FUN();     
	
	if(hwmsen_read_byte(client, LSM6DS3_CTRL3_C, databuf))
	{
		GSE_ERR("read LSM6DS3_CTRL3_XL err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LSM6DS3_CTRL3_C register: 0x%x\n", databuf[0]);
	}
	if(inc)
	{
		databuf[0] |= LSM6DS3_CTRL3_C_IFINC;
		
		databuf[1] = databuf[0];
		databuf[0] = LSM6DS3_CTRL3_C; 
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GSE_ERR("write full scale register err!\n");
			return LSM6DS3_ERR_I2C;
		}
	}

	return LSM6DS3_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_init_client(struct i2c_client *client, bool enable)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end	
    GSE_LOG(" lsm6ds3 addr %x!\n",client->addr);
	res = LSM6DS3_CheckDeviceID(client);
	if(res != LSM6DS3_SUCCESS)
	{
		return res;
	}

	res = LSM6DS3_Set_RegInc(client, true);
	if(res != LSM6DS3_SUCCESS) 
	{
		return res;
	}

	res = LSM6DS3_acc_SetFullScale(client,LSM6DS3_ACC_RANGE_4g);//we have only this choice //oujiacheng@wind-mobi.com 20160303 add 
	if(res != LSM6DS3_SUCCESS) 
	{
		return res;
	}

	//res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_104HZ);
	res = LSM6DS3_acc_SetSampleRate(client, obj->sample_rate);
	if(res != LSM6DS3_SUCCESS ) 
	{
		return res;
	}
	
	res = LSM6DS3_acc_SetPowerMode(client, enable);
	if(res != LSM6DS3_SUCCESS)
	{
		return res;
	}

	GSE_LOG("LSM6DS3_init_client OK!\n");
	//acc setting
		
#ifdef CONFIG_LSM6DS3_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
#ifdef LSM6DS3_NEW_ARCH
static int lsm6ds3_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
	
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lsm6ds3_enable_nodata(int en)
{
	int value = en;
	int err = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if(value == 1)
	{
		enable_status = true;
	}
	else
	{
		enable_status = false;
		priv->sample_rate = LSM6DS3_ACC_ODR_104HZ; //default rate
	}
//yangzhigang@wind-mbi.com 20160407 start
#if DEBUG_log
	GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
#endif
//yangzhigang@wind-mobi.com end
	if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
	{
		GSE_LOG("Gsensor device have updated!\n");
	}
	else if(!pedo_enable_status && !tilt_enable_status)
	{
	    mutex_lock(&lsm6ds3_acc_mutex);
		err = LSM6DS3_acc_SetPowerMode( priv->client, enable_status);					
		mutex_unlock(&lsm6ds3_acc_mutex);
	}
//yangzhigang@wind-mbi.com 20160407 start
#if DEBUG_log
    GSE_LOG("%s OK!\n",__FUNCTION__);
#endif
//yangzhigang@wind-mobi.com end
    return err;
}

static int lsm6ds3_set_delay(u64 ns)
{
    int value =0;
    int sample_delay;
	int err = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data; //liukangping
    value = (int)ns/1000/1000; //liukangping
    //value = (int)((int)(ns)/1000/000);
	
	

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
					
	if(value <= 5)
	{
		sample_delay = LSM6DS3_ACC_ODR_208HZ;
	}
	else if(value <= 10)
	{
		sample_delay = LSM6DS3_ACC_ODR_104HZ;
	}
	else
	{
		sample_delay = LSM6DS3_ACC_ODR_52HZ;
	}
	priv->sample_rate = sample_delay;
	err = LSM6DS3_acc_SetSampleRate(priv->client, sample_delay);
	if(err != LSM6DS3_SUCCESS ) 
	{
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50)
	{
		atomic_set(&priv->filter, 0);
	}
	else
	{					
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[LSM6DS3_AXIS_X] = 0;
		priv->fir.sum[LSM6DS3_AXIS_Y] = 0;
		priv->fir.sum[LSM6DS3_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	}

    GSE_LOG("%s (%d), chip only use 1024HZ \n",__FUNCTION__, value);
    return 0;
}

static int lsm6ds3_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;

	GSE_LOG("mpu6515 acc set delay = (%d) ok.\n", value);
	return lsm6ds3_set_delay(samplingPeriodNs);
}
static int lsm6ds3_flush(void)
{
	return acc_flush_report();
}


static int lsm6ds3_get_data(int* x ,int* y,int* z, int* status)
{
   char buff[LSM6DS3_BUFSIZE];
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
		
	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	if(atomic_read(&priv->trace) & ACCEL_TRC_DATA)
	{
		GSE_LOG("%s (%d),  \n",__FUNCTION__,__LINE__);
	}
	memset(buff, 0, sizeof(buff));
	LSM6DS3_ReadAccData(priv->client, buff, LSM6DS3_BUFSIZE);
	
	sscanf(buff, "%x %x %x", x, y, z);				
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;				

    return 0;
}
#ifdef LSM6DS3_TILT_FUNC
static int lsm6ds3_tilt_open_report_data(int open)
{
	int res = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	
	if(1 == open)
	{
		//GSE_FUN();
		tilt_enable_status = true;
		res = LSM6DS3_enable_tilt(priv->client, true);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_ERR("%s run LSM6DS3_enable_tilt to true failed!\n", __func__);
		}
	}
	else if(0 == open)
	{
		tilt_enable_status = false;
		res = LSM6DS3_enable_tilt(priv->client, false);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_ERR("%s run LSM6DS3_enable_tilt to false failed!\n", __func__);
		}
	}
	
	return res;
}
#endif

#ifdef LSM6DS3_SIGNIFICANT_MOTION
static int lsm6ds3_step_c_enable_significant(int en)
{
	int res =0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	GSE_FUN();
	
	if(1 == en)
	{
	          if ( true == sigm_enable_status)
		     return 0;
		sigm_enable_status = true;
		res = LSM6DS3_Set_SigMotion_Threshold(priv->client, 0x02);     // 2 = 125mg
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_ERR("%s run LSM6DS3_Set_SigMotion_Threshold to fail!\n", __func__);
		}
		//res = LSM6DS3_acc_SetSampleRate(priv->client, LSM6DS3_ACC_ODR_26HZ);
		res = LSM6DS3_acc_SetSampleRate(priv->client, priv->sample_rate);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_ERR("%s run LSM6DS3_Set_SigMotion_Threshold to fail!\n", __func__);
		}

		msleep(50);
		
		res = LSM6DS3_Enable_SigMotion_Func_On_Int(priv->client, true); //default route to INT2
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_ERR("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}
		
		GSE_LOG(" sigMotion enable with1 int1_count=%d\n", atomic_read(&priv->int1_request_num));
		while ( atomic_read(&priv->int1_request_num) >=1){
			//if (( tilt_enable_status ==false) &&( stepd_enable_status ==false) )
		    		enable_irq(step_irq);
			atomic_dec(&priv->int1_request_num);
			GSE_LOG(" sigMotion enable with2 tilt_enable_status=%d,stepd_enable_status = %d,irq times %d  \n", tilt_enable_status,stepd_enable_status, atomic_read(&priv->int1_request_num));
			}
	}
	else if(0 == en)
	{
	    if ( false == sigm_enable_status)
		   return 0;
		sigm_enable_status = false;
		res = LSM6DS3_Enable_SigMotion_Func_On_Int(priv->client, false);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_ERR("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}

		if(!enable_status && !tilt_enable_status && !pedo_enable_status && !stepd_enable_status)
		{   
		    mutex_lock(&lsm6ds3_acc_mutex);
			res = LSM6DS3_acc_SetPowerMode(priv->client, false);
			mutex_unlock(&lsm6ds3_acc_mutex);
			if(LSM6DS3_SUCCESS != res)
			{
				GSE_ERR("%s run LSM6DS3_acc_SetPowerMode to fail!\n", __func__);
			}
		}
		GSE_LOG(" sigMotion disable with1 int1_count=%d\n", atomic_read(&priv->int1_request_num));

		
		if (( tilt_enable_status ==false) &&( stepd_enable_status ==false) )
		{
			cancel_work_sync(&priv->eint_work);
			disable_irq(step_irq);
			atomic_inc(&priv->int1_request_num);
		}
	        GSE_LOG(" sigMotion enable with2 tilt_enable_status=%d,stepd_enable_status = %d \n", tilt_enable_status,stepd_enable_status);
	}
	
	return res;
}
#endif

#ifdef LSM6DS3_STEP_COUNTER //step counter
static int lsm6ds3_step_c_open_report_data(int open)
{
	
	return LSM6DS3_SUCCESS;
}
static int lsm6ds3_step_c_enable_nodata(int en)
{
	int res =0;
	int value = en;
	int err = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	if(priv == NULL)
	{
		GSE_ERR("%s obj_i2c_data is NULL!\n", __func__);
		return -1;
	}

	if(value == 1)
	{
		pedo_enable_status = true;
		res = LSM6DS3_enable_pedo(priv->client, true);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("LSM6DS3_enable_pedo failed at open action!\n");
			return res;
		}
	}
	else
	{
		pedo_enable_status = false;
		res = LSM6DS3_enable_pedo(priv->client, false);
		if(LSM6DS3_SUCCESS != res)
		{
			GSE_LOG("LSM6DS3_enable_pedo failed at close action!\n");
			return res;
		}

	}
	
	GSE_LOG("lsm6ds3_step_c_enable_nodata OK!\n");
    return err;
}
static int lsm6ds3_step_c_enable_step_detect(int en)
{
	return lsm6ds3_step_c_enable_nodata(en);
}

static int lsm6ds3_step_c_set_delay(u64 delay)
{
	
	return 0;
}

static int lsm6ds3_step_d_set_delay(u64 delay)
{
	
	return 0;
}

static int lsm6ds3_step_c_flush(void)
{
	
	return step_c_flush_report();
}

static int lsm6ds3_step_d_flush(void)
{
	
	return step_d_flush_report();
}

static int lsm6ds3_step_c_get_data(uint32_t *value, int *status)
{
	int err = 0;
	u16 pedo_data = 0;
	
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	err = LSM6DS3_Get_Pedo_DataReg(priv->client, &pedo_data);
	*value = (uint32_t)pedo_data;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	printk("fengqian LSM6DS3_Get_Pedo_DataReg222 %d \n",*value);	
	return err;
}
static int lsm6ds3_step_c_get_data_step_d(uint32_t *value, int *status)
{
	return 0;
}
static int lsm6ds3_step_c_get_data_significant(uint32_t *value, int *status)
{
	return 0;
}
#endif
#else
static int LSM6DS3_acc_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct lsm6ds3_i2c_data *priv = (struct lsm6ds3_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[LSM6DS3_BUFSIZE];
	
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = LSM6DS3_ACC_ODR_208HZ;
				}
				else if(value <= 10)
				{
					sample_delay = LSM6DS3_ACC_ODR_104HZ;
				}
				else
				{
					sample_delay = LSM6DS3_ACC_ODR_52HZ;
				}
				
				priv->sample_rate = sample_delay;
				LSM6DS3_acc_SetSampleRate(priv->client, sample_delay);
				if(err != LSM6DS3_SUCCESS ) 
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{					
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[LSM6DS3_AXIS_X] = 0;
					priv->fir.sum[LSM6DS3_AXIS_Y] = 0;
					priv->fir.sum[LSM6DS3_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				}
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
			    
				value = *(int *)buff_in;
				if(value == 1)
				{
					enable_status = true;
				}
				else
				{
					enable_status = false;
					priv->sample_rate = LSM6DS3_ACC_ODR_104HZ; //default rate
				}
				GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
				
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else if(!pedo_enable_status && !tilt_enable_status)
				{
					err = LSM6DS3_acc_SetPowerMode( priv->client, enable_status);					
				}

			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				LSM6DS3_ReadAccData(priv->client, buff, LSM6DS3_BUFSIZE);
				
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int lsm6ds3_open(struct inode *inode, struct file *file)
{
	file->private_data = lsm6ds3_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long lsm6ds3_acc_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[LSM6DS3_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];
	acc_offset offset; //liukangping
	acc_offset set_offset;  //liukangping
	acc_offset *set_qk_acc_offset; //liukangpings
	acc_offset *qk_acc_offset;  //liukangping

	//GSE_FUN(f);
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;	  
			}
						
			LSM6DS3_ReadChipInfo(client, strbuf, LSM6DS3_BUFSIZE);
			
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;	  
			}
			
			LSM6DS3_ReadAccData(client, strbuf, LSM6DS3_BUFSIZE);
			
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;	  
			}			

			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;	  
			}

			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if (data == NULL) {
				err = -EINVAL;
				break;	  
			}
			
			LSM6DS3_ReadAccRawData(client, (s16 *)strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
		#if 0
			cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000); //NTC
			cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
			cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
		#else
			cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x);
			cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y);	
			cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z);	
		#endif
				err = LSM6DS3_acc_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = LSM6DS3_acc_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			err = LSM6DS3_acc_ReadCalibration(client, cali);
			if(err < 0)
			{
				break;
			}
					
		#if 0
			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000); //NTC
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		#else
			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]);
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]);
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]);
		#endif
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
//oujiacheng@wind-mobi.com 20151216 begin

		case ACC_CALIBRATE:
			data = (void __user*)arg;
			memset(&offset,0,sizeof(acc_offset));
			
		
			qk_acc_offset=&offset;			
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			printk("[kevin]qk_acc_offset.x=%d,qk_acc_offset.y=%d,qk_acc_offset.z=%d\n",qk_acc_offset->x,qk_acc_offset->y,qk_acc_offset->z);
			if(qiku_acc_calibration(qk_acc_offset))
			{
			       /* oujiacheng@wind-mobi.com 20160117 modify begin */
				#ifdef KERNEL_NVRAM_READ
				ACC_NVRAM_RW(NVRAM_WRITE,(__user acc_offset*)qk_acc_offset,ACC_RW_DATA_LEN,ACC_RW_DATA_START);
				#endif
				/* oujiacheng@wind-mobi.com 20160117 modify end */
				if(copy_to_user(data, qk_acc_offset, sizeof(acc_offset)))
				{
					err = -EFAULT;
					printk("[kevin]ERROR\n");
				}
				else
				{
					printk("[kevin]qiku_acc_calibration ok\n");
				}
			        /* oujiacheng@wind-mobi.com 20160222 modify begin */
				err = LSM6DS3_acc_ResetCalibration(client);
					
				//if(atomic_read(&obj->suspend)){
				//	printk("[Bily]Perform calibration in suspend state!!\n");
				//	err = -EINVAL;
				//}else{
				printk("[oujiacheng]----%d,%d,%d----\n",offset.x,offset.y,offset.z);
					cali[LSM6DS3_AXIS_X] = offset.x;// * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
					cali[LSM6DS3_AXIS_Y] = offset.y;// * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
					cali[LSM6DS3_AXIS_Z] = offset.z;// * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);		  
					err = LSM6DS3_acc_WriteCalibration(client, cali);			 
				//}
				 /* oujiacheng@wind-mobi.com 20160222 modify end */
			}
			else
			{
				printk("[kevin]qiku_acc_calibration error\n");
			}
			break;
/* oujiacheng@wind-mobi.com 20160222 add begin */
		case ACC_SET_CALIBRATE:
			printk("[Bily]set acc calibrate  begin\n");
			data = (void __user*)arg;
			
			memset(&set_offset,0,sizeof(acc_offset));
			
			set_qk_acc_offset=&set_offset;	
			if(data == NULL){
				err = -EINVAL;
				break;	  
			}
			
			if(copy_from_user(set_qk_acc_offset, data, sizeof(acc_offset))){
				err = -EFAULT;
				printk("[Bily]get caili data ERROR\n");
				break;	  
			}
			
			err = LSM6DS3_acc_ResetCalibration(client);
			
			if(atomic_read(&obj->suspend)){
				printk("[Bily]Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}else{
				cali[LSM6DS3_AXIS_X] = set_offset.x * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
				cali[LSM6DS3_AXIS_Y] = set_offset.y * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
				cali[LSM6DS3_AXIS_Z] = set_offset.z * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);		  
				err = LSM6DS3_acc_WriteCalibration(client, cali);			 
			}
			printk("[Bily]set acc calibrate  end\n");
			break;
/* oujiacheng@wind-mobi.com 20160222 add end */
			
//oujiacheng@wind-mobi.com 20151216 end
		
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}
#ifdef CONFIG_COMPAT
static long lsm6ds3_acc_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL) {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL) {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif

/*----------------------------------------------------------------------------*/
static struct file_operations lsm6ds3_acc_fops = {
	.owner = THIS_MODULE,
	.open = lsm6ds3_open,
	.release = lsm6ds3_release,
	.unlocked_ioctl = lsm6ds3_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = lsm6ds3_acc_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice lsm6ds3_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lsm6ds3_acc_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);       
	int err = 0;
//yangzhigang@wind-mobi.com 20160407 start	
	#if DEBUG_log
	GSE_FUN();
    #endif 
//yangzhigang@wind-mobi.com end	
	if (msg.event == PM_EVENT_SUSPEND) {   
		if (obj == NULL) {
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);

		if(pedo_enable_status  || tilt_enable_status || sigm_enable_status ||stepd_enable_status )
		{
			return 0;
		}
		mutex_lock(&lsm6ds3_acc_mutex);
		err = LSM6DS3_acc_SetPowerMode(obj->client, false);
		mutex_unlock(&lsm6ds3_acc_mutex);
		if(err)
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}
		
		sensor_power = false;
		//LSM6DS3_power(obj->hw, 0);

	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_resume(struct i2c_client *client)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -1;
	}

	if(pedo_enable_status  || tilt_enable_status || sigm_enable_status ||stepd_enable_status )
	{
		atomic_set(&obj->suspend, 0);
		return 0;
	}
	//LSM6DS3_power(obj->hw, 1);
	mutex_lock(&lsm6ds3_acc_mutex);
	err = LSM6DS3_acc_SetPowerMode(obj->client, enable_status);
	mutex_unlock(&lsm6ds3_acc_mutex);
	if (err) {
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return err ;        
	}
	atomic_set(&obj->suspend, 0);  

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void lsm6ds3_early_suspend(struct early_suspend *h) 
{
	struct lsm6ds3_i2c_data *obj = container_of(h, struct lsm6ds3_i2c_data, early_drv);   
	int err;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);
	
	if(pedo_enable_status  || tilt_enable_status || sigm_enable_status)
	{
		return;
	}
	mutex_lock(&lsm6ds3_acc_mutex);
	err = LSM6DS3_acc_SetPowerMode(obj->client, false);
	mutex_unlock(&lsm6ds3_acc_mutex);
	if(err)
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;
	
	//LSM6DS3_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void lsm6ds3_late_resume(struct early_suspend *h)
{
	struct lsm6ds3_i2c_data *obj = container_of(h, struct lsm6ds3_i2c_data, early_drv);         
	int err;
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}
	
	if (pedo_enable_status  || tilt_enable_status || sigm_enable_status)
		atomic_set(&obj->suspend, 0);
		return;
	}

	//LSM6DS3_power(obj->hw, 1);
	mutex_lock(&lsm6ds3_acc_mutex);
	err = LSM6DS3_acc_SetPowerMode(obj->client, enable_status);
	mutex_unlock(&lsm6ds3_acc_mutex);

	if (err) {
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return;        
	}
	atomic_set(&obj->suspend, 0);    
}
#endif /*CONFIG_HAS_EARLYSUSPEND*/


#if (defined(LSM6DS3_TILT_FUNC) ||defined(LSM6DS3_SIGNIFICANT_MOTION) ||defined(LSM6DS3_STEP_COUNTER))
static void lsm6ds3_eint_work(struct work_struct *work)
{
	u8 src_value = 0;
	//u8 src_fifo = 0;
	u8 wake_src = 0;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
 //yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
printk("fengqian lsm6ds3_eint_work\n");


//yangzhigang@wind-mobi.com end
	if(obj == NULL)
	{
		GSE_ERR("obj_i2c_data is null pointer!!\n");
		goto lsm6ds3_eint_work_exit;
	}	
	
	if(hwmsen_read_byte(obj->client, LSM6DS3_FUNC_SRC, &src_value))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		goto lsm6ds3_eint_work_exit;
	}
	
		if(hwmsen_read_byte(obj->client, LSM6DS3_WAKE_UP_SRC, &wake_src))
	{
		GSE_ERR("%s read LSM6DS3_WAKE_UP_SRC register err!\n", __func__);
		goto lsm6ds3_eint_work_exit;
	}

   	if(hwmsen_read_byte(obj->client, LSM6DS3_FUNC_SRC, &src_value))
	{
		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		goto lsm6ds3_eint_work_exit;
	}
	
#ifdef LSM6DS3_STEP_COUNTER
	/* 16bit step count overflow, need to reset */
	#if 0
	if(LSM6DS3_STEP_OVERFLOW & src_value)
	{
		int res;
		u16 steps = 0;

		res = lsm6ds3_Get_Pedo_DataReg(obj->client, &steps);
		if(res == LSM6DS3_SUCCESS)
		{
			if(obj->boot_deb == 1)
			{
				/* state debounce */
				obj->boot_deb = 0;
				GSE_ERR("%s overflow intr first boot time with steps=%d.\n", __func__, steps);
			}
			else if((steps == 0) || (steps == 65535))
			{
				res = lsm6ds3_Reset_Pedo_Data(obj->client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
				if(res != LSM6DS3_SUCCESS)
				{
					GSE_ERR(" LSM6DS3_Reset_Pedo_Data failed!\n");
				}
				else
				{
					obj->overflow += 1;
					atomic_set(&obj->reset_sc, 1);
					GSE_LOG("%s overflow intr done steps=%d en=%d.\n", __func__, steps, pedo_enable_status);
				}
			}
		}
	}
	#endif
	/* at least 1 step recognized in delta time, readback & send to uplayer */
	/*if(LSM6DS3_STEP_COUNT_DELTA_IA & src_value)
	{
		if(0 == atomic_read(&obj->suspend))
		{
			step_notify(TYPE_STEP_COUNTER);
		}
		obj->boot_deb = 0;
	} */


	if((stepd_enable_status == true) && (LSM6DS3_STEP_DETECT_INT_STATUS & src_value))
	{
		//add the action when receive step detection interrupt
		step_notify(TYPE_STEP_DETECTOR);
	}
#endif

#ifdef LSM6DS3_SIGNIFICANT_MOTION
	if((sigm_enable_status == true) && (LSM6DS3_SIGNICANT_MOTION_INT_STATUS & wake_src))
	{
		//add the action when receive sigm interrupt
		step_notify(TYPE_SIGNIFICANT);
	}
#endif

#ifdef LSM6DS3_TILT_FUNC
	if(LSM6DS3_TILT_INT_STATUS & src_value)
	{
		//add the action when receive the tilt interrupt
		tilt_notify();
	}
#endif
lsm6ds3_eint_work_exit:
    enable_irq(step_irq);
	atomic_dec(&obj->int1_request_num);

	GSE_LOG(" enable detect with pedo=%d tilt_enable_status=%d,sigm_enable_status = %d,irq times %d\n", pedo_enable_status, tilt_enable_status,sigm_enable_status, atomic_read(&obj->int1_request_num));
#ifdef LSM6DS3_FIFO_SUPPORT
	if(0 != atomic_read(&obj->int2_request_num))
	{
		mt_eint_unmask(CUST_EINT2_LSM6DS3_NUM);
	}
#endif
}
#endif

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lsm6ds3_i2c_data *obj; //liukangping

#ifdef LSM6DS3_NEW_ARCH
	//struct acc_control_path ctl = { 0 }; //liukangping
	//struct acc_data_path data = { 0 }; //liukangping
#else
	struct hwmsen_object gyro_sobj;
	struct hwmsen_object acc_sobj;
#endif
	int err = 0;

	/* oujiacheng@wind-mobi.com 20160126  begin */
	#ifdef KERNEL_NVRAM_READ
	NVRAM_ACC_DATA_SET=1;
	#endif

	if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct lsm6ds3_i2c_data));

	err = get_accel_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		GSE_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_kfree;
	}
	
#if(defined(LSM6DS3_TILT_FUNC) || defined(LSM6DS3_SIGNIFICANT_MOTION) || defined(LSM6DS3_STEP_COUNTER))
	INIT_WORK(&obj->eint_work, lsm6ds3_eint_work);
#endif
	
	obj->sample_rate = LSM6DS3_ACC_ODR_104HZ;
	printk("fengqian direction %d \n",obj->hw.direction);
	obj->hw.direction = 1; 
		
	atomic_set(&obj->layout, obj->hw.direction);
	err = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->int1_request_num, 0);
	
	lsm6ds3_i2c_client = new_client;	
	err = LSM6DS3_init_client(new_client, false);
	if (err) {
		goto exit_init_failed;
	}
	 printk("wind-101gsensor_probe\n");
	err = misc_register(&lsm6ds3_acc_device);
	if (err) {
		GSE_ERR("lsm6ds3_gyro_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
#ifdef LSM6DS3_NEW_ARCH
	err = lsm6ds3_create_attr(&lsm6ds3_init_info.platform_diver_addr->driver);
#else
	err = lsm6ds3_create_attr(&lsm6ds3_driver.driver);
#endif
	if (err) {
		GSE_ERR("lsm6ds3 create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
		//modify by qiangang@wind-mobi.com 20170911 begin			
        err = LSM6DS3_Write_PedoThreshold(client, 0x0d);//  in 4g mode 0x0b = 10* 32 = 320mg  set threshold to a certain value here  0x11  
		if(err != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Write_PedoThreshold failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		
	    GSE_LOG(" LSM6DS3_PEDOMETER RESET!\n");
		err = LSM6DS3_Reset_Pedo_Data(client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
		
		if(err != LSM6DS3_SUCCESS)
		{
			GSE_LOG(" LSM6DS3_Reset_Pedo_Data failed!\n");
			return LSM6DS3_ERR_STATUS;
		}  
		//modify by qiangang@wind-mobi.com 20170911 end	
#ifdef LSM6DS3_NEW_ARCH

#else
	acc_sobj.self = obj;
    acc_sobj.polling = 1;
    acc_sobj.sensor_operate = LSM6DS3_acc_operate;
	err = hwmsen_attach(ID_ACCELEROMETER, &acc_sobj);
	if(err)
	{
		GSE_ERR("hwmsen_attach Accelerometer fail = %d\n", err);
		goto exit_kfree;
	}
#endif	

	//liukangping  add gsensor factory calibration 20161220 begin
    gsensor_proc_init(); 
	printk("liukangping gsensor_proc_init \n");
	//liukangping  add gsensor factory calibration 20161220 end

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = lsm6ds3_early_suspend,
	obj->early_drv.resume   = lsm6ds3_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 
#if (defined(LSM6DS3_TILT_FUNC) ||defined(LSM6DS3_SIGNIFICANT_MOTION) ||defined(LSM6DS3_STEP_COUNTER))
	lsm6ds3_setup_eint();
#endif 
#ifdef LSM6DS3_NEW_ARCH
	lsm6ds3_acc_init_flag = 0;
#endif
	GSE_LOG("%s: OK\n", __func__);
	printk("wind-666gsensor_probe success");

	return 0;

exit_create_attr_failed:
	misc_deregister(&lsm6ds3_acc_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
#ifdef LSM6DS3_NEW_ARCH
	lsm6ds3_acc_init_flag = -1;
#endif
	GSE_ERR("%s: err = %d\n", __func__, err);     
	return err;
}
static int lsm6ds3_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, LSM6DS3_ACC_DEV_NAME);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_remove(struct i2c_client *client)
{
	int err = 0;	

#ifdef LSM6DS3_NEW_ARCH		
	if (test_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test)) {
		err = lsm6ds3_delete_attr(&(lsm6ds3_init_info.platform_diver_addr->driver));
	}
	lsm6ds3_acc_init_flag = -1;
#else
	err = lsm6ds3_delete_attr(&lsm6ds3_driver.driver);
#endif
	if (err) {
		GSE_ERR("lsm6ds3_i2c_remove fail: %d\n", err);
	}

	err = misc_deregister(&lsm6ds3_acc_device);
	if (err) {
		GSE_ERR("misc_deregister lsm6ds3_gyro_device fail: %d\n", err);
	}

	lsm6ds3_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	
	//liukangping  add psensor factory calibration begin
	gsensor_proc_remove();
	//liukangping  add psensor factory calibration end
	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef LSM6DS3_NEW_ARCH
static int lsm6ds3_local_init_common(void)
{
#if 0
	const char *name = "mediatek,lsm6ds33a";
	printk("qiangang 222 lsm6ds3_local_init");

	hw = get_accel_dts_func(name, hw);
	LSM6DS3_power(hw, 1);
	printk("qiangang 333 lsm6ds3_local_init");
#endif 	
	if (i2c_add_driver(&lsm6ds3_i2c_driver)) {
		printk("qiangang 444 lsm6ds3_local_init");
		GSE_ERR("add driver error\n");
		return -1;
	}

	return 0;
}
static int lsm6ds3_local_init(void)
{
	int res = 0;
	struct acc_control_path ctl={0};
    struct acc_data_path data={0};
	struct lsm6ds3_i2c_data *obj = NULL; 

	mutex_lock(&lsm6ds3_init_mutex);
	printk("wind-444 lsm6ds3_local_init");
	
	set_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test);

	if ((0==test_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test)) \
		&& (0 == test_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test))) {
		printk("qiangang 111 lsm6ds3_local_init\n");
		res = lsm6ds3_local_init_common();
		if (res < 0) {
			goto lsm6ds3_local_init_failed;
		}
	}
	printk("qiangang 555 lsm6ds3_local_init\n");

	if (lsm6ds3_acc_init_flag == -1) {
		mutex_unlock(&lsm6ds3_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	} else {
		obj = obj_i2c_data;
		if (NULL == obj) {
			GSE_ERR("i2c_data obj is null!!\n");
			goto lsm6ds3_local_init_failed;
		}
		printk("qiangang 666 lsm6ds3_local_init\n");
		//res = lsm6ds3_create_attr(&(lsm6ds3_init_info.platform_diver_addr->driver));
		//if (res < 0) {
			//printk("qiangang error here lsm6ds3_local_init\n");
			//goto lsm6ds3_local_init_failed;
		//}
		printk("qiangang 777 lsm6ds3_local_init\n");
		ctl.open_report_data= lsm6ds3_open_report_data;
	    ctl.enable_nodata = lsm6ds3_enable_nodata;
	    ctl.set_delay  = lsm6ds3_set_delay;
	    ctl.is_report_input_direct = false;
		ctl.batch = lsm6ds3_batch;
		ctl.flush = lsm6ds3_flush;
		//modify by qiangang@wind-mobi.com 20170920 begin
	    //ctl.is_support_batch = obj->hw->is_batch_supported;
		ctl.is_support_batch = false;
		//modify by qiangang@wind-mobi.com 20170920 begin
        printk("qiangang wind-999 lsm6ds3_local_init\n");
	    res = acc_register_control_path(&ctl);
	    if (res) {
	         GSE_ERR("register acc control path err\n");
			 goto lsm6ds3_local_init_failed;
	    }

        printk("qiangang wind-777 lsm6ds3_local_init\n");
	    data.get_data = lsm6ds3_get_data;
		printk("qiangang  wind-888 lsm6ds3_local_init\n");
	    data.vender_div = 1000;
	    res = acc_register_data_path(&data);
	    if (res) {
	        GSE_ERR("register acc data path err= %d\n", res);
			goto lsm6ds3_local_init_failed;
	    }
	}
	mutex_unlock(&lsm6ds3_init_mutex);
	return 0;
lsm6ds3_local_init_failed:
	GSE_ERR("%s init failed\n", __FUNCTION__);
	mutex_unlock(&lsm6ds3_init_mutex);
	return res;
}
static int lsm6ds3_local_uninit(void)
{
	//struct acc_hw *accel_hw = get_cust_acc_hw();
	//struct acc_hw *hw = get_cust_acc();
	//const char *name = "mediatek,lsm6ds33a";

	//hw = get_accel_dts_func(name, hw);
	clear_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test);

    //GSE_FUN();    
    //LSM6DS3_power(hw, 0);  	
    i2c_del_driver(&lsm6ds3_i2c_driver);
    return 0;
}
#ifdef LSM6DS3_TILT_FUNC
static int lsm6ds3_tilt_get_data(u16 *value, int *status)
{
  return 0;
}
#endif

#ifdef CUST_EINT_LSM6DS3_TYPE
//static void lsm6ds3_eint_func(void)
irqreturn_t lsm6ds3_eint_func(int irq , void *desc)
{
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	disable_irq_nosync(step_irq);
	atomic_inc(&priv->int1_request_num);
	printk("fengqian enable detect with pedo=%d tilt_enable_status=%d,sigm_enable_status = %d,irq times %d\n", pedo_enable_status, tilt_enable_status,sigm_enable_status, atomic_read(&priv->int1_request_num));
	GSE_FUN();
	if(!priv)
	{
		return -1;
	}
	schedule_work(&priv->eint_work);	

	return IRQ_HANDLED;
}
#endif

#if (defined(LSM6DS3_TILT_FUNC) ||defined(LSM6DS3_SIGNIFICANT_MOTION) ||defined(LSM6DS3_STEP_COUNTER))
static int lsm6ds3_setup_eint(void)
{
	int ret;
	struct device_node *node = NULL;
	struct pinctrl *pinctrl;
	//struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};
        struct lsm6ds3_i2c_data *priv = obj_i2c_data;
/* gpio setting */

 // alspsPltFmDev = get_alsps_platformdev();

	node = of_find_compatible_node(NULL, NULL, "mediatek,st_step_counter");
	printk("qiangang node = %s\n",node->name);
	//printk("qiangang stepPltFmDev->dev = %s\n",stepPltFmDev->dev);
	printk("qiangang run here 111\n");
	pinctrl = devm_pinctrl_get(&stepPltFmDev->dev);
	printk("qiangang run here 222\n");
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		GSE_ERR("Cannot find step pinctrl!\n");
		return ret;
	}
	printk("qiangang run here 333\n");
	//pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	//printk("qiangang pins_default = %c\n",pins_default->name);
	//if (IS_ERR(pins_default)) {
	//	ret = PTR_ERR(pins_default);
	//	GSE_ERR("Cannot find step qiangang pinctrl default!\n");
	//}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	//printk("qiangang pins_cfg = %c\n",pins_cfg->name);
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		GSE_ERR("Cannot find step qiangang pinctrl pin_cfg!\n");
		return ret;
	}
	printk("qiangang here is 1\n");
	pinctrl_select_state(pinctrl, pins_cfg);
	printk("qiangang here is 2\n");

/* eint request */
	if (node) {
		//GSE_LOG("irq node qiangang is ok!");
		printk("qiangang here is 3\n");
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		//GSE_LOG("irq node qiangang is ok!");
		printk("qiangang here is 4\n");
		//gpio_set_debounce(ints[0], ints[1]);
		//GSE_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		printk("qiangang here is 5 ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		step_irq = irq_of_parse_and_map(node, 0);
		printk("qiangang here is 6 step_irq = %d\n", step_irq);
		//GSE_LOG("step_irq = %d\n", step_irq);
		if (!step_irq) {
			GSE_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}

		//if (request_irq(step_irq, lsm6ds3_eint_func, IRQF_TRIGGER_NONE, "gyroscope", NULL)) {
        if (request_irq(step_irq, lsm6ds3_eint_func, IRQ_TYPE_LEVEL_HIGH, "gyroscope", NULL)) {	   //	IRQ_TYPE_LEVEL_LOW
			GSE_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		disable_irq(step_irq);
		atomic_inc(&priv->int1_request_num);    // after disable irq depth should be 1 add by jonny 
	} else {
		GSE_ERR("null irq node!!\n");
		printk("qiangang here is 7\n");
		return -EINVAL;
	}
printk("qiangang here is 5\n");
	return 0;
}
#endif

#ifdef LSM6DS3_TILT_FUNC

static int lsm6ds3_tilt_local_init(void)
{
	int res = 0;

	struct tilt_control_path tilt_ctl={0};
	struct tilt_data_path tilt_data={0};

	mutex_lock(&lsm6ds3_init_mutex);
	set_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test);
	
	if((0==test_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test)) \
		&& (0==test_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test)))
	{
		res = lsm6ds3_local_init_common();
		if(res < 0)
		{
			goto lsm6ds3_tilt_local_init_failed;
		}
	}

	if(lsm6ds3_acc_init_flag == -1)
	{
		mutex_unlock(&lsm6ds3_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}
	else
	{
		//res = lsm6ds3_setup_eint();
		tilt_ctl.open_report_data= lsm6ds3_tilt_open_report_data;	
		res = tilt_register_control_path(&tilt_ctl);

		tilt_data.get_data = lsm6ds3_tilt_get_data;
		res = tilt_register_data_path(&tilt_data);
	}
	mutex_unlock(&lsm6ds3_init_mutex);
	return 0;
	
lsm6ds3_tilt_local_init_failed:
	mutex_unlock(&lsm6ds3_init_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return -1;
}
static int lsm6ds3_tilt_local_uninit(void)
{
	clear_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test);
    return 0;
}
#endif

#endif

#ifdef LSM6DS3_STEP_COUNTER
static int lsm6ds3_step_c_local_init(void)
{
	int res = 0;

	struct step_c_control_path step_ctl={0};
	struct step_c_data_path step_data={0};	
	printk("Kaka lsm6ds3_step_c_local_init\n");
	mutex_lock(&lsm6ds3_init_mutex);
		
	set_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test);
	
	if ((0==test_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test)) \
		&& (0 == test_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test))) {
		res = lsm6ds3_local_init_common();
		if (res < 0) {
			goto lsm6ds3_step_c_local_init_failed;
		}	
	}

	if (lsm6ds3_acc_init_flag == -1) {
		mutex_unlock(&lsm6ds3_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	} else {
		printk("Kaka lsm6ds3_step_c_local_init else \n");
	
		
		step_ctl.open_report_data= lsm6ds3_step_c_open_report_data;
		step_ctl.enable_nodata = lsm6ds3_step_c_enable_nodata;
		step_ctl.enable_step_detect  = lsm6ds3_step_c_enable_step_detect;
		step_ctl.step_c_set_delay = lsm6ds3_step_c_set_delay;//qiangang
		step_ctl.step_d_set_delay = lsm6ds3_step_d_set_delay;
		step_ctl.step_c_flush = lsm6ds3_step_c_flush;//qiangang
		step_ctl.step_d_flush = lsm6ds3_step_d_flush;
		step_ctl.is_report_input_direct = false;
		step_ctl.is_smd_support_batch = false;		
#ifdef LSM6DS3_SIGNIFICANT_MOTION
		step_ctl.enable_significant = lsm6ds3_step_c_enable_significant;
#endif

		res = step_c_register_control_path(&step_ctl);
		if (res) {
			 GSE_ERR("register step counter control path err\n");
			goto lsm6ds3_step_c_local_init_failed;
		}
	
		step_data.get_data = lsm6ds3_step_c_get_data;
		step_data.get_data_step_d = lsm6ds3_step_c_get_data_step_d;
		step_data.get_data_significant = lsm6ds3_step_c_get_data_significant;
		
		step_data.vender_div = 1;
		res = step_c_register_data_path(&step_data);
		if (res) {
			GSE_ERR("register step counter data path err= %d\n", res);
			goto lsm6ds3_step_c_local_init_failed;
		}
	}
	mutex_unlock(&lsm6ds3_init_mutex);
	return 0;
	
lsm6ds3_step_c_local_init_failed:
	mutex_unlock(&lsm6ds3_init_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return res;

}
static int lsm6ds3_step_c_local_uninit(void)
{
	clear_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test);
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int __init lsm6ds3_init(void)
{

#if 0	
	char *name = "mediatek,lsm6ds33a";

	printk("wind-222 gsensor init");

	hw = get_accel_dts_func(name, hw);
	printk("wind-333 gsensor read dts");
	if (!hw) {
		GSE_ERR("get cust_accel dts info fail\n");
	}
#endif

#ifdef LSM6DS3_NEW_ARCH
	acc_driver_add(&lsm6ds3_init_info);	
	#ifdef LSM6DS3_STEP_COUNTER //step counter
		step_c_driver_add(&lsm6ds3_step_c_init_info); //step counter
	#endif
	
	#ifdef LSM6DS3_TILT_FUNC
	tilt_driver_add(&lsm6ds3_tilt_init_info);
	#endif
	if (platform_driver_register(&lsm6ds3_driver)) {
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
   printk("wind-333 gsensor init  end");
	return 0;    
}

//oujiacheng@wind-mobi.com 20150129 begin
static ssize_t qiku_acc_calibration(acc_offset* qiku_acc_offset)
{
	// int x, y, z; //liukangping
	int sumx = 0;
	int sumy = 0;
	int sumz = 0;
	int count = 0,count_fail=0;
	int acc[LSM6DS3_ACC_AXES_NUM];
    int acc_enable = 0;
	struct i2c_client *client = lsm6ds3_i2c_client;
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data*)i2c_get_clientdata(client);
	int av_x=0,av_y=0,av_z=0;
//	struct kxtj2_1009_i2c_data *obj = (struct kxtj2_1009_i2c_data*)i2c_get_clientdata(client);
	printk("[kevin]--qiku_acc_calibration--qiku_acc_offset.x=%d,qiku_acc_offset.y=%d,qiku_acc_offset.z=%d\n",qiku_acc_offset->x,qiku_acc_offset->y,qiku_acc_offset->z);
	    if(!sensor_power)
		{
		   acc_enable = 1;
		   lsm6ds3_enable_nodata(1);
		}
		msleep(1000);
		while(count < 20)
		{
			msleep(20);
			LSM6DS3_ReadAccRawData(client, obj->data);
/*oujiacheng@wind-mobi.com 20160526 begin*/
		obj->data[LSM6DS3_AXIS_X] = (long)(obj->data[LSM6DS3_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000); //NTC
		obj->data[LSM6DS3_AXIS_Y] = (long)(obj->data[LSM6DS3_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		obj->data[LSM6DS3_AXIS_Z] = (long)(obj->data[LSM6DS3_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);		
			printk("[kevin]obj->cvt.map[LSM6DS3_AXIS_X]=%d\n",obj->cvt.map[LSM6DS3_AXIS_X]);
			printk("[kevin]obj->cvt.sign[LSM6DS3_AXIS_X]=%d\n",obj->cvt.sign[LSM6DS3_AXIS_X]);
			printk("[kevin]obj->data[LSM6DS3_AXIS_X]=%d\n",obj->data[LSM6DS3_AXIS_X]);
			printk("[kevin]acc[obj->cvt.map[LSM6DS3_AXIS_X]] =%d\n",acc[obj->cvt.map[LSM6DS3_AXIS_X]]);
			printk("[kevin]acc[LSM6DS3_AXIS_X] =%d\n",acc[LSM6DS3_AXIS_X]);
			printk("[kevin]obj->reso->sensitivity =%d\n",obj->sensitivity);	
		acc[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->data[LSM6DS3_AXIS_X];
		acc[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->data[LSM6DS3_AXIS_Y];
		acc[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->data[LSM6DS3_AXIS_Z];	
		
		

			printk("[kevin]acc[LSM6DS3_AXIS_X,Y,Z] =[%d,%d,%d]\n",acc[LSM6DS3_AXIS_X],acc[LSM6DS3_AXIS_Y],acc[LSM6DS3_AXIS_Z]);
			if ((abs(acc[LSM6DS3_AXIS_X]) < 600) && (abs(acc[LSM6DS3_AXIS_Y]) < 600) && ((abs(acc[LSM6DS3_AXIS_Z]) > 8600) && (abs(acc[LSM6DS3_AXIS_Z]) < 11300)))
			{
			sumx = sumx+acc[LSM6DS3_AXIS_X];
			sumy = sumy+acc[LSM6DS3_AXIS_Y];
			sumz = sumz+acc[LSM6DS3_AXIS_Z];
			
			} else {
			                count_fail ++;
					count = count - 1;
					if (count_fail == 10) {
						qiku_acc_offset->key = -1;
						return 0;
					}
					continue;				
			}			
			count++;
		}
		
		av_x = sumx/20;
		av_y = sumy/20;
		av_z = sumz/20;		
		if ((abs(av_x) < 600) && (abs(av_y) < 600) && ((abs(av_z) > 8600) && (abs(av_z) < 11300)))
		{
			qiku_acc_offset->x=0-av_x;
			qiku_acc_offset->y=0-av_y;
			qiku_acc_offset->z=9807-av_z;
			qiku_acc_offset->key=1;
		} else {
			qiku_acc_offset->key = -1;
		}
/*oujiacheng@wind-mobi.com 20160526 end*/		
		/* oujiacheng@wind-mobi.com 20160117 modify begin */
		#ifdef KERNEL_NVRAM_READ
		acc_x_cali_value =qiku_acc_offset->x;
		acc_y_cali_value =qiku_acc_offset->y;
		acc_z_cali_value =qiku_acc_offset->z;
		#endif
		/* oujiacheng@wind-mobi.com 20160117 modify end */
		if(acc_enable)
		{
		   acc_enable = 0;
		   lsm6ds3_enable_nodata(0);
		}
//		qiku_acc_offset->key=1;
		printk("[kevin]qiku_acc_offset.x=%d,qiku_acc_offset.y=%d,qiku_acc_offset.z=%d,qiku_acc_offset.key=%d\n",qiku_acc_offset->x,qiku_acc_offset->y,qiku_acc_offset->z,qiku_acc_offset->key);

	return 1;
      	
}

//oujiacheng@wind-mobi.com 20150129 end

/*----------------------------------------------------------------------------*/
static void __exit lsm6ds3_exit(void)
{
//yangzhigang@wind-mobi.com 20160407 start
#if DEBUG_log
	GSE_FUN();
#endif
//yangzhigang@wind-mobi.com end

	platform_driver_unregister(&lsm6ds3_driver);

}
/*----------------------------------------------------------------------------*/
module_init(lsm6ds3_init);
module_exit(lsm6ds3_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSM6DS3 Accelerometer");
MODULE_AUTHOR("xj.wang@mediatek.com, darren.han@st.com");
//liukangping@wind-mobi.com 20161108 end

/*----------------------------------------------------------------- LSM6DS3 ------------------------------------------------------------------*/
