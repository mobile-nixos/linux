//liukangping beign
/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information and source code
 *  contained herein is confidential. The software including the source code
 *  may not be copied and the information contained herein may not be used or
 *  disclosed except with the written permission of MEMSIC Inc. (C) 2017
 *****************************************************************************/
/*
 *
 * mmc3680x.c - mmc3680x magnetic sensor chip driver.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <linux/types.h>


#include <hwmsen_helper.h>
#include <batch.h>

#include <cust_mag.h>
#include "mmc3680x.h"

#include "mag.h"

/*-------------------------MT6516&MT6573 define-------------------------------*/
#define POWER_NONE_MACRO MT65XX_POWER_NONE

/*----------------------------------------------------------------------------*/
#define MMC3680X_DEV_NAME       "mmc3680x"
#define DRIVER_VERSION          "V60.97.01.10"
/*----------------------------------------------------------------------------*/
#define MEMSIC_DEBUG_ON          		0
#define MEMSIC_DEBUG_FUNC_ON     		1
/* Log define */
#define MEMSIC_INFO(fmt, arg...)      	pr_warn("<<-MMC3680X INFO->> "fmt"\n", ##arg)
#define MEMSIC_ERR(fmt, arg...)          	pr_err("<<-MMC3680X ERROR->> [line=%d]"fmt"\n",__LINE__,##arg)
#define MEMSIC_DEBUG(fmt, arg...)		do {\
						if (MEMSIC_DEBUG_ON)\
							pr_warn("<<-MMC3680X DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
					} while (0)
#define MEMSIC_DEBUG_FUNC()		do {\
						if (MEMSIC_DEBUG_FUNC_ON)\
							pr_debug("<<-MMC3680X FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
					} while (0)



#define MEMSIC_SINGLE_POWER 0

#define MMC3680X_RETRY_COUNT	3
#define MMC3680X_DEFAULT_DELAY	100
#define MMC3680X_BUFSIZE  	0x50
#define READMD			0


/* Define Delay time */
#define MMC3680X_DELAY_TM		10	/* ms */
#define MMC3680X_DELAY_SET		50	/* ms */
#define MMC3680X_DELAY_RESET		50  	/* ms */
#define MMC3680X_DELAY_STDN		1	/* ms */

#define MMC3680X_RESET_INTV		250
#define MEMSIC_ABS(a)	((a) > 0 ? (a):(-a))

static struct i2c_client *this_client = NULL;

static u32 read_idx = 0;

// calibration msensor and orientation data
static int sensor_data[CALIBRATION_DATA_SIZE];
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;


/* static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq); */
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static int mmcd_delay = MMC3680X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

static const struct i2c_device_id mmc3680x_i2c_id[] = {{MMC3680X_DEV_NAME,0},{}};

/* Maintain  cust info here */
extern struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

/*----------------------------------------------------------------------------*/
static int mmc3680x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mmc3680x_i2c_remove(struct i2c_client *client);
static int mmc3680x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int mmc3680x_suspend(struct i2c_client *client, pm_message_t msg);
static int mmc3680x_resume(struct i2c_client *client);
static int mmc3680x_local_init(void);
static int mmc3680x_remove(void);

typedef enum {
	MEMSIC_FUN_DEBUG  = 0x01,
	MEMSIC_DATA_DEBUG = 0X02,
	MEMSIC_HWM_DEBUG  = 0X04,
	MEMSIC_CTR_DEBUG  = 0X08,
	MEMSIC_I2C_DEBUG  = 0x10,
} MMC_TRC;

/*----------------------------------------------------------------------------*/
struct mmc3680x_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    atomic_t layout;
    atomic_t trace;
	struct hwmsen_convert   cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

static int mmc3680x_init_flag = 0; // 0<==>OK -1 <==> fail



static struct mag_init_info mmc3680x_init_info = {
	 .name = "mmc3680x",
	 .init = mmc3680x_local_init,
	 .uninit = mmc3680x_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{},
};
#endif

static struct i2c_driver mmc3680x_i2c_driver = {
    .driver = {
     //   .owner = THIS_MODULE,
        .name  = MMC3680X_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = mag_of_match,
#endif
    },
	.probe      = mmc3680x_i2c_probe,
	.remove     = mmc3680x_i2c_remove,
	.detect     = mmc3680x_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = mmc3680x_suspend,
	.resume     = mmc3680x_resume,
#endif
	.id_table = mmc3680x_i2c_id,
};

static atomic_t dev_open_count;

static DEFINE_MUTEX(mmc3680x_i2c_mutex);

#ifndef CONFIG_MTK_I2C_EXTENSION
int mag_read_byte(struct i2c_client *client, u8 addr, u8 *data)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,	.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = 1,	.buf = data,
		}
	};

	if (!client)
		return -EINVAL;

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		MEMSIC_ERR("i2c_transfer error: (%d %p) %d\n", addr, data, err);
		err = -EIO;
	}

	err = 0;

	return err;
}
static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	if(len == 1){
		return mag_read_byte(client, addr, data);
	}
	mutex_lock(&mmc3680x_i2c_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&mmc3680x_i2c_mutex);
		MEMSIC_ERR("Client is Empty\n");
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc3680x_i2c_mutex);
		MEMSIC_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		MEMSIC_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&mmc3680x_i2c_mutex);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	mutex_lock(&mmc3680x_i2c_mutex);
	if (!client) {
		mutex_unlock(&mmc3680x_i2c_mutex);
		MEMSIC_ERR("Client is Empty\n");
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc3680x_i2c_mutex);
		MEMSIC_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&mmc3680x_i2c_mutex);
		MEMSIC_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&mmc3680x_i2c_mutex);
	return err;
}
#endif

static void mmc3680x_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
#if 0
	if(hw->power_id != MT65XX_POWER_NONE)
	{
		//MEMSIC_DEBUG("power %s\n", on ? "on" : "off");
		if(power_on == on)
		{
			//MEMSIC_DEBUG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "mmc3680x"))
			{
				MEMSIC_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "mmc3680x"))
			{
				MEMSIC_ERR("power off fail!!\n");
			}
		}
	}
#endif
	power_on = on;
}

static int I2C_RxData(char *rxData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = rxData[0];

	if ((rxData == NULL) || (length < 1))
	{
		MEMSIC_ERR("Invalid param\n");
		return -EINVAL;
	}
	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
	{
		MEMSIC_ERR("mag_i2c_read_block error\n");
		return -1;
	}
	return 0;
#else
	uint8_t loop_i = 0;

	int i;
	struct i2c_client *client = this_client;
	struct mmc3680x_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];

	/* Caller should check parameter validity.*/

	if((rxData == NULL) || (length < 1))
	{
		MEMSIC_ERR("Invalid param\n");
		return -EINVAL;
	}

	for(loop_i = 0; loop_i < MMC3680X_RETRY_COUNT; loop_i++)
	{
		this_client->addr = (this_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG;
		if(i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01)))
		{
			break;
		}
		MEMSIC_DEBUG("I2C_RxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC3680X_RETRY_COUNT)
	{
		MEMSIC_ERR("%s retry over %d\n", __func__, MMC3680X_RETRY_COUNT);
		return -EIO;
	}

	if(atomic_read(&data->trace) == MEMSIC_I2C_DEBUG)
	{
		MEMSIC_INFO("RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for(i = 0; i < length; i++)
		{
			MEMSIC_INFO(" %02x", rxData[i]);
		}
	    MEMSIC_INFO("\n");
	}

	return 0;
#endif
}

static int I2C_TxData(char *txData, int length)
{
#ifndef CONFIG_MTK_I2C_EXTENSION
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = txData[0];
	u8 *buff = &txData[1];

	if ((txData == NULL) || (length < 2))
	{
		MEMSIC_ERR("Invalid param\n");
		return -EINVAL;
	}
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;
#else
	uint8_t loop_i;
	int i;
	struct i2c_client *client = this_client;
	struct mmc3680x_i2c_data *data = i2c_get_clientdata(client);

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		return -EINVAL;
	}
	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < MMC3680X_RETRY_COUNT; loop_i++)
	{
		if(i2c_master_send(this_client, (const char*)txData, length) > 0)
		{
			break;
		}
		MEMSIC_DEBUG("I2C_TxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= MMC3680X_RETRY_COUNT)
	{
		MEMSIC_ERR("%s retry over %d\n", __func__, MMC3680X_RETRY_COUNT);
		return -EIO;
	}

	if(atomic_read(&data->trace) == MEMSIC_I2C_DEBUG)
	{
		MEMSIC_INFO("TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++)
		{
			MEMSIC_INFO(" %02x", txData[i + 1]);
		}
		MEMSIC_INFO("\n");
	}

	return 0;
#endif
}

static int ECS_SaveData(int buf[12])
{
	struct i2c_client *client = this_client;
	struct mmc3680x_i2c_data *data = i2c_get_clientdata(client);

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);

	if(atomic_read(&data->trace) & MEMSIC_HWM_DEBUG)
	{
		MEMSIC_INFO("Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
	}

	return 0;
}

static int ECS_ReadXYZData(int *vec, int size)
{
	unsigned char data[6] = {0,0,0,0,0,0};
#if READMD
	int MD_times = 0;
#endif
	struct i2c_client *client = this_client;
	struct mmc3680x_i2c_data *clientdata = i2c_get_clientdata(client);

	if(size < 3)
	{
		MEMSIC_ERR("Invalid size value\n");
		return -1;
	}
	mutex_lock(&read_i2c_xyz);

	if (!(read_idx % MMC3680X_RESET_INTV))
	{
		/* Reset Sensor Periodly SET */
#if MEMSIC_SINGLE_POWER
		data[0] = MMC3680X_REG_CTRL;
		data[1] = MMC3680X_CTRL_REFILL;
		/* not check return value here, assume it always OK */
		I2C_TxData(data, 2);
		/* wait external capacitor charging done for next RM */
		msleep(MMC3680X_DELAY_SET);
#endif
		data[0] = 0x0F;
		data[1] = 0xE1;
		I2C_TxData(data, 2);

		data[0]=0x20;
		I2C_RxData(data, 1);

		data[1] = data[0] & 0xE7;
		data[0] = 0x20;
		I2C_TxData(data, 2);

		data[0] = MMC3680X_REG_CTRL;
		data[1] = MMC3680X_CTRL_SET;
		I2C_TxData(data, 2);
		msleep(MMC3680X_DELAY_STDN);
	}

	/* send TM cmd before read */
	data[0] = MMC3680X_REG_CTRL;
	data[1] = MMC3680X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);
	msleep(MMC3680X_DELAY_TM);

#if READMD
	/* Read MD */
	data[0] = MMC3680X_REG_DS;
	I2C_RxData(data, 1);
	while (!(data[0] & 0x01)) {
		msleep(1);
		/* Read MD again*/
		data[0] = MMC3680X_REG_DS;
		I2C_RxData(data, 1);
		if (data[0] & 0x01) break;
		MD_times++;
		if (MD_times > 3) {
			MEMSIC_ERR("TM not work!!");
			mutex_unlock(&read_i2c_xyz);
			return -EFAULT;
		}
	}
#endif
	read_idx++;
	data[0] = MMC3680X_REG_DATA;
	if(I2C_RxData(data, 6) < 0)
	{
		mutex_unlock(&read_i2c_xyz);
		MEMSIC_ERR("i2c rxdata failed\n");
		return -EFAULT;
	}
	vec[0] = data[1] << 8 | data[0];
	vec[1] = data[3] << 8 | data[2];
	vec[2] = data[5] << 8 | data[4];

	if(atomic_read(&clientdata->trace) == MEMSIC_DATA_DEBUG)
	{
		MEMSIC_INFO("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
	}

	mutex_unlock(&read_i2c_xyz);
	return 0;
}

static int ECS_GetRawData(int data[3])
{
	int err = 0;
	err = ECS_ReadXYZData(data, 3);
	if(err !=0 )
	{
		MEMSIC_ERR("MMC3680X_IOC_TM failed\n");
		return -1;
	}

	// sensitivity 1024 count = 1 Guass = 100uT
	data[0] = (data[0] - MMC3680X_OFFSET_X) * 100 / MMC3680X_SENSITIVITY_X;
	data[1] = (data[1] - MMC3680X_OFFSET_X) * 100 / MMC3680X_SENSITIVITY_X;
	data[2] = (data[2] - MMC3680X_OFFSET_X) * 100 / MMC3680X_SENSITIVITY_X;

	return err;
}

static int ECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int mmc3680x_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= MMC3680X_BUFSIZE -1))
	{
		MEMSIC_ERR("Invalid buff size\n");
		return -1;
	}
	if(!this_client)
	{
		*buf = 0;
		MEMSIC_ERR("Invalid client\n");
		return -2;
	}

	sprintf(buf, "mmc3680x Chip");
	return 0;
}

static int mmc3680x_self_test(void)
{
	unsigned char data[6] = {0};
	int pre_data[3] = {0};
	int cur_data[3] = {0};
	int delta_value[3] = {0};

	data[0] = 0x0F;
	data[1] = 0xE1;
	I2C_TxData(data, 2);

	data[0]=0x20;
	I2C_RxData(data, 1);

	data[1] = data[0] & 0xE7;
	data[0] = 0x20;
	I2C_TxData(data, 2);

	data[0] = MMC3680X_REG_CTRL;
	data[1] = MMC3680X_CTRL_RESET;
	I2C_TxData(data, 2);
	msleep(MMC3680X_DELAY_STDN);
	
	/* send TM cmd before read */
	data[0] = MMC3680X_REG_CTRL;
	data[1] = MMC3680X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);
	msleep(MMC3680X_DELAY_TM);


	data[0] = MMC3680X_REG_DATA;
	if(I2C_RxData(data, 6) < 0)
	{
		MEMSIC_ERR("read data failed\n");
		return 0;
	}
	pre_data[0] = data[1] << 8 | data[0];
	pre_data[1] = data[3] << 8 | data[2];
	pre_data[2] = data[5] << 8 | data[4];


	data[0] = 0x0F;
	data[1] = 0xE1;
	I2C_TxData(data, 2);

	data[0]=0x20;
	I2C_RxData(data, 1);

	data[1] = data[0] & 0xE7;
	data[0] = 0x20;
	I2C_TxData(data, 2);
	data[0] = MMC3680X_REG_CTRL;
	data[1] = MMC3680X_CTRL_SET;
	I2C_TxData(data, 2);
	msleep(MMC3680X_DELAY_STDN);

	/* send TM cmd before read */
	data[0] = MMC3680X_REG_CTRL;
	data[1] = MMC3680X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);
	msleep(MMC3680X_DELAY_TM);

	data[0] = MMC3680X_REG_DATA;
	if(I2C_RxData(data, 6) < 0)
	{
		MEMSIC_ERR("read data failed\n");
		return 0;
	}
	cur_data[0] = data[1] << 8 | data[0];
	cur_data[1] = data[3] << 8 | data[2];
	cur_data[2] = data[5] << 8 | data[4];

	delta_value[0] = pre_data[0] - cur_data[0];
	delta_value[1] = pre_data[1] - cur_data[1];
	delta_value[2] = pre_data[2] - cur_data[2];

	delta_value[0] = MEMSIC_ABS(delta_value[0]);
	delta_value[1] = MEMSIC_ABS(delta_value[1]);
	delta_value[2] = MEMSIC_ABS(delta_value[2]);

	if(delta_value[0] > 100 || delta_value[1] > 100 || delta_value[2] > 100)
	{
		return 1;
	}
	MEMSIC_ERR("Selftest failed delta_value[0]=%d,delta_value[1]=%d,delta_value[2]=%d\n",delta_value[0],delta_value[1],delta_value[2]);
	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3680X_BUFSIZE];
	mmc3680x_ReadChipInfo(strbuf, MMC3680X_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[MMC3680X_BUFSIZE];

	ECS_GetRawData(sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_orientation_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[MMC3680X_BUFSIZE];
	tmp[0] = sensor_data[8] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[9] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[10] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);

	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct mmc3680x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct mmc3680x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MEMSIC_ERR("hwmsen_get_convert function error!\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			MEMSIC_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			MEMSIC_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MEMSIC_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mmc3680x_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		MEMSIC_ERR("mmc3680x_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mmc3680x_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		MEMSIC_ERR("mmc3680x_i2c_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		MEMSIC_ERR("invalid content: '%s', length = %ld\n", buf, count);
	}

	return count;
}


static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3680X_BUFSIZE];
	sprintf(strbuf, "memsicd3680x");
	return sprintf(buf, "%s", strbuf);
}

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	u8 uData = 0;
	struct mmc3680x_i2c_data *obj = i2c_get_clientdata(this_client);

	if (obj == NULL) {
		MAG_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	uData = atomic_read(&m_flag);

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", uData);
	return res;
}

static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3680X_BUFSIZE];
	int ret = 0;
	ret = mmc3680x_self_test();
	sprintf(strbuf, "%d\n", ret);
	return sprintf(buf, "%s", strbuf);
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,	 S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,	 S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,   S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(orientation,  S_IRUGO, show_orientation_value, NULL);
static DRIVER_ATTR(layout,	 S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(trace,	 S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
static DRIVER_ATTR(powerstatus,        S_IRUGO, show_power_status, NULL);
static DRIVER_ATTR(selftest,    S_IRUGO, show_selftest_value, NULL);

static struct driver_attribute *mmc3680x_attr_list[] = {
    	&driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_orientation,
	&driver_attr_layout,
	&driver_attr_trace,
	&driver_attr_powerstatus,
	&driver_attr_selftest,
};

static int mmc3680x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mmc3680x_attr_list)/sizeof(mmc3680x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mmc3680x_attr_list[idx])))
		{
			MEMSIC_ERR("driver_create_file (%s) = %d\n", mmc3680x_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

static int mmc3680x_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(mmc3680x_attr_list)/sizeof(mmc3680x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mmc3680x_attr_list[idx]);
	}

	return 0;
}

static int mmc3680x_open(struct inode *inode, struct file *file)
{
	struct mmc3680x_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if(atomic_read(&obj->trace) & MEMSIC_CTR_DEBUG)
	{
		MEMSIC_INFO("Open device node:mmc3680x\n");
	}
	ret = nonseekable_open(inode, file);

	return ret;
}

static int mmc3680x_release(struct inode *inode, struct file *file)
{
	struct mmc3680x_i2c_data *obj = i2c_get_clientdata(this_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & MEMSIC_CTR_DEBUG)
	{
		MEMSIC_INFO("Release device node:mmc3680x\n");
	}
	return 0;
}

static long mmc3680x_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)

{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char buff[MMC3680X_BUFSIZE];	/* for chip information */

	int value[CALIBRATION_DATA_SIZE];			/* for SET_YPR */
	int delay;			/* for GET_DELAY */
	int status; 			/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	unsigned char data[16] = {0};
	int vec[3] = {0};
	struct i2c_client *client = this_client;
	struct mmc3680x_i2c_data *clientdata = i2c_get_clientdata(client);
	struct hwm_sensor_data* osensor_data;
	uint32_t enable;
	unsigned char reg_addr;
	unsigned char reg_value;
	switch (cmd)
	{
		case MMC31XX_IOC_TM:
			data[0] = MMC3680X_REG_CTRL;
			data[1] = MMC3680X_CTRL_TM;
			if (I2C_TxData(data, 2) < 0)
			{
				MEMSIC_ERR("MMC3680X_IOC_TM failed\n");
				return -EFAULT;
			}
			/* wait TM done for coming data read */
			msleep(MMC3680X_DELAY_TM);
			break;

		case MMC31XX_IOC_SET:
		case MMC31XX_IOC_RM:
#if MEMSIC_SINGLE_POWER
			data[0] = MMC3680X_REG_CTRL;
			data[1] = MMC3680X_CTRL_REFILL;
			if(I2C_TxData(data, 2) < 0)
			{
				MEMSIC_ERR("MMC3680X_IOC_SET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3680X_DELAY_SET);
#endif
			data[0] = 0x0F;
			data[1] = 0xE1;
			I2C_TxData(data, 2);

			data[0] = 0x20;
			I2C_RxData(data, 1);

			data[1] = data[0] & 0xE7;
			data[0] = 0x20;
			I2C_TxData(data, 2);

			data[0] = MMC3680X_REG_CTRL;
			data[1] = MMC3680X_CTRL_SET;
			if(I2C_TxData(data, 2) < 0)
			{
				MEMSIC_ERR("MMC3680X_IOC_SET failed\n");
				return -EFAULT;
			}

			msleep(MMC3680X_DELAY_STDN);
			break;

		case MMC31XX_IOC_RESET:
		case MMC31XX_IOC_RRM:
#if MEMSIC_SINGLE_POWER
			data[0] = MMC3680X_REG_CTRL;
			data[1] = MMC3680X_CTRL_REFILL;
			if(I2C_TxData(data, 2) < 0)
			{
				MEMSIC_ERR("MMC3680X_CTRL_REFILL failed\n");
				return -EFAULT;
			}

			msleep(MMC3680X_DELAY_RESET);
#endif
			data[0] = 0x0F;
			data[1] = 0xE1;
			I2C_TxData(data, 2);

			data[0] = 0x20;
			I2C_RxData(data, 1);

			data[1] = data[0] & 0xE7;
			data[0] = 0x20;
			I2C_TxData(data, 2);

			data[0] = MMC3680X_REG_CTRL;
			data[1] = MMC3680X_CTRL_RESET;
			if(I2C_TxData(data, 2) < 0)
			{
				MEMSIC_ERR("MMC3680X_CTRL_RESET failed\n");
				return -EFAULT;
			}
			/* wait external capacitor charging done for next SET/RESET */
			msleep(MMC3680X_DELAY_STDN);
			break;

		case MMC31XX_IOC_READ:
			data[0] = MMC3680X_REG_DATA;
			if(I2C_RxData(data, 6) < 0)
			{
				MEMSIC_ERR("MMC3680X_IOC_READ failed\n");
				return -EFAULT;
			}
			vec[0] = data[1] << 8 | data[0];
			vec[1] = data[3] << 8 | data[2];
			vec[2] = data[5] << 8 | data[4];

			if(atomic_read(&clientdata->trace) == MEMSIC_DATA_DEBUG)
			{
				MEMSIC_INFO("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
			}
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				MEMSIC_ERR("MMC3680X_IOC_READ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MMC31XX_IOC_READXYZ:
			ECS_ReadXYZData(vec, 3);
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				MEMSIC_ERR("MMC3680X_IOC_READXYZ: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_DELAY:
			delay = mmcd_delay;
			if(copy_to_user(argp, &delay, sizeof(delay)))
			{
				MEMSIC_ERR("copy_to_user failed.\n");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_SET_YPR:
			if(argp == NULL)
			{
				MEMSIC_ERR("invalid argument.\n");
				return -EINVAL;
			}
			if(copy_from_user(value, argp, sizeof(value)))
			{
				MEMSIC_ERR("copy_from_user failed.\n");
				return -EFAULT;
			}
			ECS_SaveData(value);
			break;

		case ECOMPASS_IOC_GET_OPEN_STATUS:
			status = ECS_GetOpenStatus();
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MEMSIC_ERR("copy_to_user failed.\n");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = atomic_read(&m_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MEMSIC_ERR("copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_OFLAG:
			sensor_status = atomic_read(&o_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				MEMSIC_ERR("copy_to_user failed.\n");
				return -EFAULT;
			}
			break;


		case MSENSOR_IOCTL_READ_CHIPINFO:
			if(argp == NULL)
			{
				MEMSIC_ERR("IO parameter pointer is NULL!\n");
				break;
			}

			mmc3680x_ReadChipInfo(buff, MMC3680X_BUFSIZE);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:
			if(argp == NULL)
			{
				MEMSIC_ERR("IO parameter pointer is NULL!\n");
				break;
			}
			ECS_GetRawData(vec);
			sprintf(buff, "%x %x %x", vec[0], vec[1], vec[2]);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_LAYOUT:
			status = atomic_read(&clientdata->layout);
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:

			if(argp == NULL)
			{
				MEMSIC_ERR("IO parameter pointer is NULL!\n");
				break;
			}
			if(copy_from_user(&enable, argp, sizeof(enable)))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			else
			{
			    	MEMSIC_INFO( "MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\n",enable);
				if(1 == enable)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);

			}

			break;

		case MMC3680X_IOC_READ_REG:
			if (copy_from_user(&reg_addr, argp, sizeof(reg_addr)))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			data[0] = reg_addr;
			if (I2C_RxData(data, 1) < 0)
			{
				MEMSIC_ERR("I2C TX failed\n");
				return -EFAULT;
			}
			reg_value = data[0];
			if (copy_to_user(argp, &reg_value, sizeof(reg_value)))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MMC3680X_IOC_WRITE_REG:
			if (copy_from_user(&data, argp, sizeof(data)))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			if (I2C_TxData(data, 2) < 0)
			{
				return -EFAULT;
			}

		    break;

		case MMC3680X_IOC_READ_REGS:
			if (copy_from_user(&data, argp, sizeof(data)))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			if (I2C_RxData(data, data[1]) < 0)
			{
				MEMSIC_ERR("I2C TX failed\n");
				return -EFAULT;
			}
			MEMSIC_DEBUG("data: %x %x %x \n%x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);
			if (copy_to_user(argp, data, sizeof(data)))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
			if(argp == NULL)
			{
				MEMSIC_ERR("IO parameter pointer is NULL!\n");
				break;
			}

			osensor_data = (struct hwm_sensor_data *)buff;
		    	mutex_lock(&sensor_data_mutex);

			osensor_data->values[0] = sensor_data[8] * CONVERT_O;
			osensor_data->values[1] = sensor_data[9] * CONVERT_O;
			osensor_data->values[2] = sensor_data[10] * CONVERT_O;
			osensor_data->status = sensor_data[11];
			osensor_data->value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);

			sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
				osensor_data->values[2],osensor_data->status,osensor_data->value_divide);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				MEMSIC_ERR("copy to user failed\n");
				return -EFAULT;
			}

			break;

		default:
			MEMSIC_ERR("%s not supported = 0x%04x\n", __FUNCTION__, cmd);
			return -ENOIOCTLCMD;
			break;
		}

	return 0;
}
#ifdef CONFIG_COMPAT
static long mmc3680x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

		void __user *arg64 = compat_ptr(arg);

		if(!file->f_op || !file->f_op->unlocked_ioctl)
		{
			MEMSIC_ERR("file->f_op OR file->f_op->unlocked_ioctl is null!\n");
			return -ENOTTY;
		}

		switch(cmd)
		{
			case COMPAT_MMC31XX_IOC_TM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_TM, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC31XX_IOC_TM is failed!\n");
				}
				break;

			case COMPAT_MMC31XX_IOC_SET:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_SET, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC31XX_IOC_SET is failed!\n");
				}
				break;

			case COMPAT_MMC31XX_IOC_RM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RM, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC31XX_IOC_RM is failed!\n");
				}

				break;

			case COMPAT_MMC31XX_IOC_RESET:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RESET, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC31XX_IOC_RESET is failed!\n");
				}

			case COMPAT_MMC31XX_IOC_RRM:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_RRM, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC31XX_IOC_RRM is failed!\n");
				}

				break;

			case COMPAT_MMC31XX_IOC_READ:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READ, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC31XX_IOC_READ is failed!\n");
				}

				break;

			case COMPAT_MMC31XX_IOC_READXYZ:
				ret = file->f_op->unlocked_ioctl(file, MMC31XX_IOC_READXYZ, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC31XX_IOC_READXYZ is failed!\n");
				}

				break;

			case COMPAT_MMC3680X_IOC_READ_REG:
				ret = file->f_op->unlocked_ioctl(file, MMC3680X_IOC_READ_REG, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC3680X_IOC_READ_REG is failed!\n");
				}
				break;

			case COMPAT_MMC3680X_IOC_WRITE_REG:
				ret = file->f_op->unlocked_ioctl(file, MMC3680X_IOC_WRITE_REG, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC3680X_IOC_WRITE_REG is failed!\n");
				}

				break;

			case COMPAT_MMC3680X_IOC_READ_REGS:
				ret = file->f_op->unlocked_ioctl(file, MMC3680X_IOC_READ_REGS, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MMC3680X_IOC_READ_REGS is failed!\n");
				}
				break;

			case COMPAT_ECOMPASS_IOC_GET_DELAY:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_DELAY, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_ECOMPASS_IOC_GET_DELAY is failed!\n");
				}

				break;

			case COMPAT_ECOMPASS_IOC_SET_YPR:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_SET_YPR, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_ECOMPASS_IOC_SET_YPR is failed!\n");
				}

				break;

			case COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OPEN_STATUS, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_ECOMPASS_IOC_GET_OPEN_STATUS is failed!\n");
				}

				break;

			case COMPAT_ECOMPASS_IOC_GET_MFLAG:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_MFLAG, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_ECOMPASS_IOC_GET_MFLAG is failed!\n");
				}

				break;

			case COMPAT_ECOMPASS_IOC_GET_OFLAG:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_OFLAG, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_ECOMPASS_IOC_GET_OFLAG is failed!\n");
				}

				break;


			case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
				if(arg64 == NULL)
				{
					MEMSIC_ERR("IO parameter pointer is NULL!\n");
					break;
				}

				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_CHIPINFO, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MSENSOR_IOCTL_READ_CHIPINFO is failed!\n");
				}

				break;

			case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
				if(arg64 == NULL)
				{
					MEMSIC_ERR("IO parameter pointer is NULL!\n");
					break;
				}

				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MSENSOR_IOCTL_READ_SENSORDATA is failed!\n");
				}

				break;

			case COMPAT_ECOMPASS_IOC_GET_LAYOUT:
				ret = file->f_op->unlocked_ioctl(file, ECOMPASS_IOC_GET_LAYOUT, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_ECOMPASS_IOC_GET_LAYOUT is failed!\n");
				}

				break;

			case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
				if(arg64 == NULL)
				{
					MEMSIC_ERR("IO parameter pointer is NULL!\n");
					break;
				}

				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_SENSOR_ENABLE, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE is failed!\n");
				}

				break;

			case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
				if(arg64 == NULL)
				{
					MEMSIC_ERR("IO parameter pointer is NULL!\n");
					break;
				}

				ret = file->f_op->unlocked_ioctl(file, MSENSOR_IOCTL_READ_FACTORY_SENSORDATA, (unsigned long)arg64);
				if(ret < 0)
				{
					MEMSIC_ERR("COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA is failed!\n");
				}

				break;

		 default:
			 MEMSIC_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			 return -ENOIOCTLCMD;
			 break;
	}
    return 0;
}

#endif
/*----------------------------------------------------------------------------*/
static struct file_operations mmc3680x_fops = {
	//.owner = THIS_MODULE,
	.open = mmc3680x_open,
	.release = mmc3680x_release,
	.unlocked_ioctl = mmc3680x_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mmc3680x_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mmc3680x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "msensor",
    .fops = &mmc3680x_fops,
};
/*----------------------------------------------------------------------------*/

#ifndef	CONFIG_HAS_EARLYSUSPEND
static int mmc3680x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct mmc3680x_i2c_data *obj = i2c_get_clientdata(client);


	if(msg.event == PM_EVENT_SUSPEND)
	{
		mmc3680x_power(obj->hw, 0);
	}
	return 0;
}
static int mmc3680x_resume(struct i2c_client *client)
{
	struct mmc3680x_i2c_data *obj = i2c_get_clientdata(client);


	mmc3680x_power(obj->hw, 1);


	return 0;
}
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
static void mmc3680x_early_suspend(struct early_suspend *h)
{
	struct mmc3680x_i2c_data *obj = container_of(h, struct mmc3680x_i2c_data, early_drv);

	if(NULL == obj)
	{
		MEMSIC_ERR("null pointer!!\n");
		return;
	}
	
	mmc3680x_power(obj->hw, 0);

}

static void mmc3680x_late_resume(struct early_suspend *h)
{
	struct mmc3680x_i2c_data *obj = container_of(h, struct mmc3680x_i2c_data, early_drv);


	if(NULL == obj)
	{
		MEMSIC_ERR("null pointer!!\n");
		return;
	}

	mmc3680x_power(obj->hw, 1);

}
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int mmc3680x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, MMC3680X_DEV_NAME);
	return 0;
}

static int mmc3680x_m_enable(int en)
{
	int value = 0;
	int err = 0;
	value = en;

	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}
	}
	wake_up(&open_wq);
	return err;
}

static int mmc3680x_m_set_delay(u64 ns)
{
    int value=0;
	value = (int)ns/1000/1000;
	if(value <= 10)
    {
        mmcd_delay = 10;
    }
    else{
        mmcd_delay = value;
	}

	return 0;
}

static int mmc3680x_m_open_report_data(int open)
{
	return 0;
}

static int mmc3680x_o_enable(int en)
{
  	int value =en;
	if(value == 1)
	{
		atomic_set(&o_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&o_flag, 0);
		if(atomic_read(&m_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}
	}
	wake_up(&open_wq);
	return en;
}

static int mmc3680x_o_set_delay(u64 ns)
{
	int value=0;
	value = (int)ns/1000/1000;
	if(value <= 10)
	{
		mmcd_delay = 10;
	}
	else{
		mmcd_delay = value;
	}

	return 0;

}

static int mmc3680x_o_open_report_data(int open)
{
	return 0;
}

static int mmc3680x_o_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);

	*x = sensor_data[8] * CONVERT_O;
	*y = sensor_data[9] * CONVERT_O;
	*z = sensor_data[10] * CONVERT_O;
	*status = sensor_data[11];

	mutex_unlock(&sensor_data_mutex);

	return 0;
}

static int mmc3680x_m_get_data(int* x ,int* y,int* z, int* status)
{
	mutex_lock(&sensor_data_mutex);

	*x = sensor_data[4] * CONVERT_M;
	*y = sensor_data[5] * CONVERT_M;
	*z = sensor_data[6] * CONVERT_M;
	*status = sensor_data[7];
	mutex_unlock(&sensor_data_mutex);
	return 0;
}

static int check_device(void)
{
	char product_id[2] = {0};

	product_id[0] = MMC3680X_REG_PRODUCTID;
	if(I2C_RxData(product_id, 1) < 0)
	{
		MEMSIC_ERR("[mmc3680x] read id fail\n");
		//read again
		I2C_RxData(product_id, 1);
	}

	MEMSIC_INFO("[mmc3680x] product_id[0] = %d\n",product_id[0]);
	if(product_id[0] != MMC3680x_DEVICE_ID)
	{
		MEMSIC_ERR("Got memsic mmc3680x id failed");
		return -1;
	}
	return 0;
}
static int init_device(void)
{
	char tmp[2] = {0};

#if MEMSIC_SINGLE_POWER
    tmp[0] = MMC3680X_REG_CTRL;
	tmp[1] = MMC3680X_CTRL_REFILL;
	I2C_TxData(tmp, 2);
	msleep(MMC3680X_DELAY_SET);
#endif
	tmp[0] = 0x0F;
	tmp[1] = 0xE1;
	I2C_TxData(tmp, 2);

	tmp[0]=0x20;
	I2C_RxData(tmp, 1);

	tmp[1] = tmp[0] & 0xE7;
	tmp[0] = 0x20;
	I2C_TxData(tmp, 2);

	tmp[0] = MMC3680X_REG_CTRL;
	tmp[1] = MMC3680X_CTRL_SET;
	I2C_TxData(tmp, 2);
	msleep(MMC3680X_DELAY_STDN);

	tmp[0] = MMC3680X_REG_BITS;
	tmp[1] = MMC3680X_BITS_SLOW_16;
	I2C_TxData(tmp, 2);
	msleep(1);

	tmp[0] = MMC3680X_REG_CTRL;
	tmp[1] = MMC3680X_CTRL_TM;
	I2C_TxData(tmp, 2);
	return 0;
}

static int mmc3680x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mmc3680x_i2c_data *data;
	int err = 0;
	struct mag_control_path ctl={0};
	struct mag_data_path mag_data={0};
    
	MEMSIC_INFO("%s: enter probe,driver version=%s\n", __func__,DRIVER_VERSION);
	printk("wind-liukangping entra mmc3680x_i2c_probe");

	if(!(data = kmalloc(sizeof(struct mmc3680x_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		MEMSIC_ERR("Allocate memory to struct failed\n");
		goto exit;
	}
	memset(data, 0, sizeof(struct mmc3680x_i2c_data));

	data->hw = hw;

	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);

	/*init_waitqueue_head(&data_ready_wq);*/
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;
    msleep(10);

	err = check_device();
	if(err < 0)
	{
		MEMSIC_ERR("mmc3680x probe: check device connect error\n");
		goto exit_kfree;
	}

    err = init_device();
	if(err < 0)
	{
		MEMSIC_ERR("mmc3680x probe: init_device error\n");
		goto exit_kfree;
	}
	printk("wind-liukangping entra init_device");


	/* Register sysfs attribute */
	if((err = mmc3680x_create_attr(&(mmc3680x_init_info.platform_diver_addr->driver))))
	{
		MEMSIC_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	if((err = misc_register(&mmc3680x_device)))
	{
		MEMSIC_ERR("mmc3680x_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	ctl.is_use_common_factory = false;
	ctl.m_enable = mmc3680x_m_enable;
	ctl.m_set_delay  = mmc3680x_m_set_delay;
	ctl.m_open_report_data = mmc3680x_m_open_report_data;
	ctl.o_enable = mmc3680x_o_enable;
	ctl.o_set_delay  = mmc3680x_o_set_delay;
	ctl.o_open_report_data = mmc3680x_o_open_report_data;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw->is_batch_supported;

	err = mag_register_control_path(&ctl);
	if(err)
	{
		MEMSIC_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

	mag_data.div_m = CONVERT_M_DIV;
	mag_data.div_o = CONVERT_O_DIV;
	mag_data.get_data_o = mmc3680x_o_get_data;
	mag_data.get_data_m = mmc3680x_m_get_data;

	err = mag_register_data_path(&mag_data);
	if(err)
	{
		MEMSIC_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#if defined CONFIG_HAS_EARLYSUSPEND
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	data->early_drv.suspend  = mmc3680x_early_suspend,
	data->early_drv.resume   = mmc3680x_late_resume,
	register_early_suspend(&data->early_drv);
#endif

	MEMSIC_INFO("mmc3680X IIC probe successful !");

	mmc3680x_init_flag = 1;
	return 0;

	exit_sysfs_create_group_failed:
	exit_misc_device_register_failed:
	exit_kfree:
	kfree(data);
	exit:
	MEMSIC_ERR("%s: err = %d\n", __func__, err);
	mmc3680x_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int mmc3680x_i2c_remove(struct i2c_client *client)
{
	int err;

	if((err = mmc3680x_delete_attr(&(mmc3680x_init_info.platform_diver_addr->driver))))
	{
		MEMSIC_ERR("mmc3680x_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&mmc3680x_device);
	return 0;
}
/*----------------------------------------------------------------------------*/

static int mmc3680x_local_init(void)
{

	mmc3680x_power(hw, 1);

	atomic_set(&dev_open_count, 0);

    printk("wind-liukangping entra mmc3680x_local_init");
	if(i2c_add_driver(&mmc3680x_i2c_driver))
	{
		MEMSIC_ERR("add driver error\n");
		return -1;
	}

	printk("wind-liukangping entra mmc3680x_init_flag=%d\n",mmc3680x_init_flag);
	if(-1 == mmc3680x_init_flag)
	{
	   MEMSIC_ERR("mmc3680x init failed\n");
	   return -1;
	}

	return 0;
}

static int mmc3680x_remove(void)
{

	mmc3680x_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&mmc3680x_i2c_driver);
	return 0;
}



/*----------------------------------------------------------------------------*/
static int __init mmc3680x_init(void)
{
	const char *name = "mediatek,mmc3680x";

	hw = get_mag_dts_func(name, hw);

	MEMSIC_INFO("mmc3680x_init addr0 = 0x%x,addr1 = 0x%x,i2c_num = %d \n",hw->i2c_addr[0],hw->i2c_addr[1],hw->i2c_num);

	if (!hw)
	{
		MEMSIC_ERR("get dts info fail\n");
	}

  printk("wind-liukangping entra mmc3680x_init");
	mag_driver_add(&mmc3680x_init_info);

	return 0;
}


/*----------------------------------------------------------------------------*/
static void __exit mmc3680x_exit(void)
{
	//platform_driver_unregister(&mmc3680x_platform_driver);
}
/*----------------------------------------------------------------------------*/
module_init(mmc3680x_init);
module_exit(mmc3680x_exit);

MODULE_AUTHOR("Aaron Peng<hcpeng@memsic.cn>");
MODULE_DESCRIPTION("MEMSIC MMC3680KJ Magnetic Sensor Chip Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
//liukangping end

