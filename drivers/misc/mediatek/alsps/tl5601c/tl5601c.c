/*
 * This file is part of the tl5601c sensor driver for MTK platform.
 * tl5601c is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: Seon-haeng Lee <shlee@sensonia.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: tl5601c.c
 *
 * Summary:
 *	tl5601c sensor dirver.
 *
 * android version : 6.0
 * linux version : 3.18.22
 *
 * Modification History:
 * Version	Date		By		Summary
 * 1.3.0	01/02/17	SHLEE	modify power managerment process for compatiblity
 * 1.2.3	12/26/16	SHLEE	add sunlight depensive code 
 * 1.2.2	09/23/16	SHLEE	always init crosstalk at ps enable
 * 1.2.1	09/22/16	SHLEE	void ps enable 0 at ps enable
 * 1.2.0	09/22/16	SHLEE	add void sunlight noise
 * 1.1.9	09/19/16	SHLEE	add calculate init crosstalk work
 * 1.1.8	08/11/16	SHLEE	modify auto calibration bug
 * 1.1.7	08/03/16	SHLEE	update auto calibration for 6.25ms_x64
 * 1.1.6	07/19/16	SHLEE	add led_off_auto_calibration
 * 1.1.5	06/29/16	SHLEE	update auto_calibration, attribute debug
 * 1.1.4	04/06/16	SHLEE	update_ps_auto_range, auto_calibration
 * 1.1.3   	01/13/16  	SHLEE 	update tl5601c source
 * 1.1.2	04/07/15	SHLEE	add tl5601cR4 source
 * 1.1.1	10/06/14	SHLEE	shutdown bug fix
 * 1.1.0	03/14/14	SHLEE	Original Creation (initial version:1.1.0)
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/ioctl.h>
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

//#define PLATFORM_VERSION_AND_5

/*----------------------------------*/
#ifdef PLATFORM_VERSION_AND_5
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include "mach/eint.h"
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <cust_eint.h>
#endif
/*-----------------------------------*/

#include <linux/io.h>
#include <linux/gpio.h>
#include <cust_alsps.h>
#include "tl5601c.h"

#include <alsps.h>


#include "alsps_calibrator.h"    //lihaiyan@wind-mobi.com 20170327


/******************************************************************************
 * configuration
 *******************************************************************************/
/*----------------------------------------------------------------------------*/

#define DRV_VER						"1.3.0"
#define DRV_DESC					"tl5601c ambient & proximity sensor driver"
#define VENDOR_NAME					"SENSONIA"
#define DEV_NAME					"tl5601c"
#define SUCCESS						0
#define ERR_I2C						-1
#define ERR_STATUS					-3
#define ERR_SETUP_FAILURE			-4
#define ERR_GETGSENSORDATA			-5
#define ERR_IDENTIFICATION			-6


/*----------------------------------------------------------------------------*/
// debug
#define APS_TAG					"[ALSPS_SENSO]"
//#define APS_FUN(f)				pr_info(APS_TAG"%s\n", __func__)
#define APS_FUN(f)				pr_err(APS_TAG"%s\n", __func__)
#define APS_FUN_ERR(f)			pr_err(APS_TAG"%s ERROR LINE = %d\n", __func__, __LINE__)
#define APS_ERR(fmt, args...)	pr_err(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	pr_info(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	pr_debug(APS_TAG fmt, ##args)
#define ABS_LIGHT				ABS_MISC
/*----------------------------------------------------------------------------*/
#define ENABLE	1
#define DISABLE	0
#define ALS_MAX	16383

/*
 * Registers
 */
#define I2C_SLAVE_ADDR		0x39	/* ADDR pin is ground */
//#define I2C_SLAVE_ADDR	0x44	/* ADDR pin is high */
#define CHIPID				0x62

// 		REGISTER_NAME	 	ADDRESS
#define ADDR_CHIPID			0x00
#define ADDR_MODE			0x01
#define ADDR_INT_STATUS		0x02
#define ADDR_INT_FILTER		0x03
#define ADDR_RES_ALS		0x04
#define ADDR_RANGE_ALS		0x05
#define ADDR_RES_PS			0x06
#define ADDR_RANGE_PS		0x07
#define ADDR_LED_CTL		0x08
#define ADDR_INT_EN			0x09
#define ADDR_MODE_CTL		0x0A
#define ADDR_SOFT_RESET		0x0B
#define ADDR_ALS_LTH_LSB	0X0D
#define ADDR_ALS_LTH_MSB	0X0E
#define ADDR_ALS_HTH_LSB	0X0F
#define ADDR_ALS_HTH_MSB	0x10
#define ADDR_PS_LTH_LSB		0x11
#define ADDR_PS_LTH_MSB		0x12
#define ADDR_PS_HTH_LSB		0x13
#define ADDR_PS_HTH_MSB		0x14
#define ADDR_ALS0_DATA_LSB	0x15
#define ADDR_ALS1_DATA_LSB	0x17
#define ADDR_PS_DATA_LSB		0x19
#define ADDR_PS_DATA_MSB		0x1A
#define ADDR_PS_OFFSET			0x1B
#define ADDR_LED_HIGH_TH_LSB	0x1C
#define ADDR_LED_HIGH_TH_MSB	0x1D
#define ADDR_LED_LOW_TH_LSB		0x1E
#define ADDR_LED_LOW_TH_MSB		0x1F
#define ADDR_PS_LED_OFF			0x25
#define ADDR_LED_OFF_DATA_MSB	0x50
#define ADDR_LED_OFF_DATA_LSB	0x51

#define MODE_PMODE_MASK		0x03
#define MODE_OPER_MASK		0x0C
#define MODE_OPER_ALSPS			0
#define MODE_OPER_PS			8
#define MODE_OPER_ALS			4
#define MODE_PMODE_NORMAL		0
#define MODE_PMODE_LOWPOWER		1
#define MODE_PMODE_AUTOSHUTDOWN	2
#define MODE_PMODE_SHUTDOWN		3

#define SOFT_RESET				0xA5

#define MODE_CTL_ALS_DATA_SEL_MASK	0x0C
#define MODE_CTL_INT_ACTIVE_MASK	0x02
#define MODE_CTL_INT_TYPE_MASK		0x01
#define MODE_CTL_SLEEP_PERIOD_MASK	0x70

// PS_DISTANCE_TABLE IN WHITE BOARD
// unit : mm, res = 6.25ms, range = x1/2
#define PS_DISTANCE_100		698
#define PS_DISTANCE_90		879
#define PS_DISTANCE_80		1124
#define PS_DISTANCE_70		1476
#define PS_DISTANCE_60		2006
#define PS_DISTANCE_50		2880
#define PS_DISTANCE_40		4030
#define PS_DISTANCE_30		4033
#define PS_DISTANCE_20		4040
#define PS_DISTANCE_10		4060
#define PS_DISTANCE_CLOSE	PS_DISTANCE_60
#define PS_DISTANCE_FAR		PS_DISTANCE_70

#define PS_INT_FILTER_0		0
#define PS_INT_FILTER_1		16
#define PS_INT_FILTER_2		32
#define PS_INT_FILTER_3		48
#define PS_INT_FILTER_4		64
#define PS_INT_FILTER_5		80
#define PS_INT_FILTER_DEFAULT	PS_INT_FILTER_2

#define ALS_INT_FILTER_0	0
#define ALS_INT_FILTER_1	1
#define ALS_INT_FILTER_2	2
#define ALS_INT_FILTER_3	3
#define ALS_INT_FILTER_DEFAULT	ALS_INT_FILTER_2

#define PS_OFFSET		0		// unit : 6.25ms, x4
#define LED_OFF_DATA_DEFAULT	3000
#define NVM_DELAY				10

#define COEF_A			1825	// 0.1825
#define COEF_B			120		// 0.012(-0.012)
#define COEF_C			4073	// 0.4073
#define COEF_D			378		// 0.0378(-0.0378)
//#define G_A				1000	// in open air
//#define G_A				5457	// 5.457 // i8730
//#define G_A				3132	// 3.132 // i437
#define G_A					600	// 3.132 // i437     
// G_A = LED_S0 / LED_S0' * 1000

#define LED_CTL_LFREQ_81				1
#define LED_CTL_LFREQ_327				0
#define LED_CTL_LPEAK_16				0
#define LED_CTL_LPEAK_32				1
#define LED_CTL_LPEAK_65				2
#define LED_CTL_LPEAK_130				3
#define LED_CTL_LMOD_25					0
#define LED_CTL_LMOD_8					1
#define LED_CTL_LMOD_5					2
#define LED_CTL_LMOD_2					3
#define RES_ALS_MASK					0x07
#define RES_ALS_19BIT					0
#define RES_ALS_18BIT					1
#define RES_ALS_17BIT					2
#define RES_ALS_16BIT					3
#define RES_ALS_14BIT					4	// 25ms
#define RES_ALS_12BIT					5
#define RES_ALS_10BIT					6
#define RES_ALS_8BIT					7

#define RANGE_ALS_REG					0x05
#define RANGE_ALS_MASK					0x07
#define RANGE_ALS_SHIFT					0
#define RANGE_ALS_X1					0
#define RANGE_ALS_X2					1
#define RANGE_ALS_X4					2
#define RANGE_ALS_X8					3
#define RANGE_ALS_X16					4
#define RANGE_ALS_X32					5
#define RANGE_ALS_X64					6
#define RANGE_ALS_X128					7

#define RES_PS_REG						0x06
#define RES_PS_MASK						0x07
#define RES_PS_SHIFT					0
#define RES_PS_19BIT					7
#define RES_PS_18BIT					6
#define RES_PS_17BIT					5
#define RES_PS_16BIT					0
#define RES_PS_14BIT					1
#define RES_PS_12BIT					2	// 6.25ms
#define RES_PS_10BIT					3	// 1.56ms
#define RES_PS_8BIT						4

#define RANGE_PS_REG					0x07
#define RANGE_PS_MASK					0x07
#define RANGE_PS_SHIFT					0
#define RANGE_PS_X1						0
#define RANGE_PS_X2						1
#define RANGE_PS_X4						2
#define RANGE_PS_X8						3
#define RANGE_PS_X16					4
#define RANGE_PS_X32					5
#define RANGE_PS_X64					6
#define RANGE_PS_X128					7

#define SLEEP_DURATION_2				0
#define SLEEP_DURATION_4				16
#define SLEEP_DURATION_8				32
#define SLEEP_DURATION_16				48
#define SLEEP_DURATION_32				64
#define SLEEP_DURATION_64				80
#define SLEEP_DURATION_128				96
#define SLEEP_DURATION_256				112
#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

#define PS_OPERATION					0x08	// enable ps only, normal mode
#define ALS_OPERATION					0x04	// enable als only, normal mode
#define ALS_PS_OPERATION				0x00	// enable als & ps, normal mode
#define MODE_OPERATION_MASK				0x0C
#define ALS_LOW_THRESHOLD_DEFAULT		0
#define ALS_HIGH_THRESHOLD_DEFAULT		0
#define ALS_POLL_DELAY					10
#define ALS_NEW_INT_ENABLE				0x01
#define PS_PROX_INT_ENABLE				0x40
#define PS_LOW_INT_ENABLE				0x10
#define PS_HIGH_INT_ENABLE				0x20
#define PS_INT_FILTER					3

#define ALS_RES_DEFAULT					RES_ALS_14BIT
#define ALS_RANGE_LOW					RANGE_ALS_X4
#define ALS_RANGE_HIGH					RANGE_ALS_X128
#define ALS_RANGE_DEFAULT				ALS_RANGE_LOW

#define PS_RANGE_DEFAULT				RANGE_PS_X4
#define PS_RES_DEFAULT					RES_PS_12BIT

#define LOW_LUX_MAX						20400
#define HIGH_LUX_MIN					13600
#define MAX_LUX							34000
#define HIGH_LUX_RANGE					RANGE_ALS_X8
#define LOW_LUX_RANGE					RANGE_ALS_X4
#define PS_DISABLE_SUNLIGHT_LUX			10000

#define PS_AIRGAP						220
#define PS_TRANS_RATIO					100
// PS_TRANS_RATIO = S2' / S2 * 100

#define PS_LOW_THRESHOLD_DEFAULT		500     
#define PS_LOW_THRESHOLD_MIN			0
#define PS_HIGH_THRESHOLD_MAX			65535
#define PS_HIGH_THRESHOLD_DEFAULT		600

// ioctl
#define ALSPS							0X84
#define ALSPS_SET_PS_MODE				_IOW(ALSPS, 0x01, int)
#define ALSPS_GET_PS_MODE				_IOR(ALSPS, 0x02, int)
#define ALSPS_GET_PS_DATA				_IOR(ALSPS, 0x03, int)
#define ALSPS_GET_PS_RAW_DATA			_IOR(ALSPS, 0x04, int)
#define ALSPS_SET_ALS_MODE				_IOW(ALSPS, 0x05, int)
#define ALSPS_GET_ALS_MODE				_IOR(ALSPS, 0x06, int)
#define ALSPS_GET_ALS_DATA				_IOR(ALSPS, 0x07, int)
#define ALSPS_GET_ALS_RAW_DATA          _IOR(ALSPS, 0x08, int)
#define ALSPS_GET_ALS0_DATA				_IOR(ALSPS, 0x37, int)
#define ALSPS_GET_ALS1_DATA				_IOR(ALSPS, 0x38, int)
#define ALSPS_PS_CAL_MODE				_IOW(ALSPS, 0x39, int)
#define ALS_RANGEUP							1
#define ALS_RANGEDOWN						-1

#define MODE_ALS_PS 0x00
#define MODE_ALS    0x04
#define PS_INT_FILTER_0	0
#define PS_INT_FILTER_1	16
#define PS_INT_FILTER_2	32
#define PS_INT_FILTER_3	48
#define PHT_INT	0x20
#define PLT_INT	0x10
#define MODE_PMODE_NORMAL	0
#define MODE_PMODE_LOWPOWER	1
#define MODE_PMODE_AUTOSHUTDOWN	2
#define MODE_PMODE_SHUTDOWN	3
#define MODE_DEFAULT	MODE_ALS_PS
#define RES_ALS_25	4
#define RES_PS_6    2 
#define RANGE_PS_4  2
#define RANGE_PS_8  3
#define RANGE_PS_16 4
#define RANGE_PS_32 5
#define PS_INT_ENABLE_PH_PL 0x30
#define PS_THRESHOLD_MAX	0xFFFF
#define PS_THRESHOLD_MIN	0
#define LHT_INT	0x02
#define LLT_INT	0x01
#define CLOSE				0
#define FAR					1
#define PROX_AVG_COUNT		40
#define PROX_MIN			0
#define PROX_MAX			0xFFFF
#define WORK_PROX_AVG_SLEEP	40
#define PS_OFFSET_TEST_COUNT	30

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

#define POWER_DOWN				0
#define SET_ALS_PS				1
#define SET_ALS					2
#define SET_PS					3

#define LIGHT_NORMAL		0
#define LIGHT_OVER		1
#define AMB_LIGHT_DEFAULT	0
#define AMB_LIGHT_MIN		1
#define AMB_LIGHT_SMALL		2
#define AMB_LIGHT_MEDIAN	3
#define AMB_LIGHT_MAX		4
#define PS_MAX_16BIT			65535
#define PS_MAX_14BIT			16383
#define PS_MAX_12BIT			4095
#define PS_MAX_10BIT			1023

#define INIT_CRT_REPEAT_COUNT	200
#define OFF					0
#define ON					1

/* #define SENSONIA_COMPILE */
#define F_DEBUG

/******************************************************************************
 * extern functions
 *******************************************************************************/
//static DEFINE_MUTEX(tl5601c_mutex);
/*----------------------------------------------------------------------------*/

static struct i2c_client *tl5601c_i2c_client;
static struct wake_lock ps_lock;
#ifdef PLATFORM_VERSION_AND_5
static struct i2c_board_info i2c_tl5601c __initdata = {I2C_BOARD_INFO(DEV_NAME, I2C_SLAVE_ADDR)};
#endif
//static int tl5601c_eint_type; /* 0:pht,plt 1:proxi */

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id tl5601c_i2c_id[] = {{DEV_NAME, 0}, {} };
/*the adapter id & i2c address will be available in customization*/
//static struct i2c_board_info i2c_tl5601c __initdata = { I2C_BOARD_INFO(DEV_NAME, I2C_SLAVE_ADDR)};
/*----------------------------------------------------------------------------*/
static int tl5601c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tl5601c_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static int tl5601c_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int tl5601c_i2c_resume(struct i2c_client *client);
#if defined(CONFIG_OF)
static irqreturn_t tl5601c_eint_handler(int irq, void *desc);
#endif
static int tl5601c_local_init(void);
static int tl5601c_remove(void);
static int tl5601c_init_flag = -1; /* 0<==>OK -1 <==> fail */
static struct alsps_init_info tl5601c_init_info = {
	.name = DEV_NAME,
	.init = tl5601c_local_init,
	.uninit = tl5601c_remove,
};

static struct platform_device *alspsPltFmDev;

#ifndef PLATFORM_VERSION_AND_5
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

struct alsps_hw *get_cust_alsps_hw(void)
{
	return &alsps_cust;
}
#endif

/*----------------------------------------------------------------------------*/
enum {
	CMC_BIT_ALS	= 1,
	CMC_BIT_PS	= 2,
};

/*----------------------------------------------------------------------------*/
struct tl5601c_i2c_addr {	/*define a series of i2c slave address*/
	u8 write_addr;
	u8 ps_thd;	/*PS INT threshold*/
};

int ps_array_count = 0;
//unsigned long ps_data_array[PS_ARRAY_COUNT] = {0};
//unsigned long led_off_data_array[PS_ARRAY_COUNT] = {0};
bool cal_init_crosstalk = false;
int init_crt_repeat_count = 0;
int work_crt_repeat_count = 0;
int saturation_count = 0;

/*----------------------------------------------------------------------------*/

struct tl5601c_obj {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct eint_work;
	struct work_struct work_prox_poll;
	struct work_struct work_prox_init_crt;
	struct workqueue_struct *wq_prox_poll;
	struct mutex i2c_mutex;
	struct mutex nvm_lock;
	struct mutex als_mutex;
	struct mutex lock;
	/*i2c address group*/
	struct tl5601c_i2c_addr addr;
	struct hrtimer prox_poll_timer;
	struct hrtimer prox_init_crt_timer;
	ktime_t prox_poll_delay;
	ktime_t prox_init_crt_delay;

#if defined(CONFIG_OF)
	struct device_node *irq_node;
	int irq;
#endif
	/*misc*/
	u16			als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on;	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t	als_suspend;

	/*data*/
	u16		als;
	u16			ps;
	u16 als0_data;
	u16 als1_data;
	u16 ps_data;
//	u16 led_high_threshold;
//	u16 led_low_threshold;
	u8			_align;
	u8 power_state;
	u16		als_level_num;
	u16		als_value_num;
	u32		als_level[C_CUST_ALS_LEVEL-1];
	u32		als_value[C_CUST_ALS_LEVEL];

	unsigned int lux;
	unsigned int res_als;
	unsigned int range_als;
	unsigned int res_ps;
	unsigned int range_ps;
	unsigned int res_als_tb;
	unsigned int range_als_tb;
	unsigned int ps_max;

	int ps_enable_interrupt;
	int ps_auto_calibration;
	int resume_als_enable;
	int oper_mode;
	int	led_freq;
	int led_peak_cur;
	int led_mode;
	int ps_debug;
	int power_mode;
	int sleep_period;
	int ps_int_filter;
	int als_int_filter;
	int proximity_status;
	int ps_offset;
	int auto_ps_en;
	int	ps_count_far;
	int	ps_count_far_sunlight;
	int	ps_count_close;
	int	ps_count_close_sunlight;
	int ps_high_threshold_normal;
	int ps_low_threshold_normal;
	int ps_dyn_high_threshold;
	int ps_dyn_low_threshold;
	int saturation;
	struct mutex prox_poll_mutex;
	int amb_light_state; 

	int led_off_noise_threshold;
	int ps_high_threshold;
	int ps_low_threshold;
	u32 abs_als0_data;
	u32 abs_als1_data;
	int ps_crosstalk;
	int ps_crosstalk_min;
	int ps_crosstalk_max;
	int ps_en_cnt;
	int ps_close_cnt;
	int saturation_threshold;

	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val;	/*the cmd value can't be read, stored in ram*/
//	atomic_t	ps_threshold_high;	/*the cmd value can't be read, stored in ram*/
//	atomic_t	ps_threshold_low;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_threshold_high;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_threshold_low;	/*the cmd value can't be read, stored in ram*/
	ulong		enable;		/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
	atomic_t	trace;
	atomic_t	debug_als;
	atomic_t	debug_ps;
	
	int ps_crosstalk_def;
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

static int tl5601c_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, DEV_NAME);
	return 0;
}

static unsigned int tl5601c_res_als_tb[] = {80000, 40000, 20000, 10000, 2500, 625, 156, 39};
static unsigned int tl5601c_range_als_tb[] = {625, 1250, 2500, 5000, 10000, 20000, 40000, 80000};
//static unsigned int tl5601c_ps_res_tb[] = {0, 64, 16, 4, 1}; //16
static unsigned int tl5601c_ps_max_tb[] = {65535, 16383, 4095, 1023};
static unsigned int tl5601c_als_max_tb[] = {65535, 65535, 65535, 65535, 16383, 4095, 1023, 255};
static struct tl5601c_obj *tl5601c_obj;

/*----------------------------------------------------------------------------*/
static struct i2c_driver tl5601c_i2c_driver = {
        .driver = {
		.name			= DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},

	.probe		= tl5601c_i2c_probe,
	.remove		= tl5601c_i2c_remove,
	.detect     = tl5601c_i2c_detect,
	.suspend	= tl5601c_i2c_suspend,
	.resume		= tl5601c_i2c_resume,
	.id_table	= tl5601c_i2c_id,
};

/*----------------------------------------------------------------------------*/
// read & write register function start
/*
static int tl5601c_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = {{0}, {0} };

	mutex_lock(&tl5601c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&tl5601c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&tl5601c_mutex);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		APS_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else
		err = 0;

	mutex_unlock(&tl5601c_mutex);
	return err;

}
*/
//static int tl5601c_hwmsen_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
//{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
//	int err, idx, num;
//	char buf[C_I2C_FIFO_SIZE];
//
//	err = 0;
//	mutex_lock(&tl5601c_mutex);
//	if (!client) {
//		mutex_unlock(&tl5601c_mutex);
//		return -EINVAL;
//	} else if (len >= C_I2C_FIFO_SIZE) {
//		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
//		mutex_unlock(&tl5601c_mutex);
//		return -EINVAL;
//	}
//
//	num = 0;
//	buf[num++] = addr;
//	for (idx = 0; idx < len; idx++)
//		buf[num++] = data[idx];
//
//	err = i2c_master_send(client, buf, num);
//	if (err < 0) {
//		APS_LOG("send command error!!\n");
//		mutex_unlock(&tl5601c_mutex);
//		return -EFAULT;
//	}
//	err = 0;
//
//	mutex_unlock(&tl5601c_mutex);
//	return err;
//}

static int tl5601c_read_byte(struct tl5601c_obj *obj, u8 addr)
{
	int res;
	u8 reg_value[1] = {0};

	mutex_lock(&obj->i2c_mutex);
	//res = tl5601c_hwmsen_read_block(obj->client, addr, reg_value, 0x1);
	res = hwmsen_read_block(obj->client, addr, reg_value, 0x1);
	mutex_unlock(&obj->i2c_mutex);
	if (res < 0)
	{
		APS_DBG("%s Send command error!, res = %d\n", __func__, res);
		return -EFAULT;
	}

	return reg_value[0];
}
/*
static int tl5601c_read_word(struct tl5601c_obj *obj, u8 addr)
{
	int res, data;
	u8 reg_value[2] = {0, 0};

	mutex_lock(&obj->i2c_mutex);
	res = hwmsen_read_block(obj->client, addr, reg_value, 0x2);
	mutex_unlock(&obj->i2c_mutex);
	if(res < 0)
	{
		APS_ERR("%s Send command error!, res = %d\n", __func__, res);
		return -EFAULT;
	}

	data = reg_value[0] + (reg_value[1] * 256);

	return data;
}
*/
static int tl5601c_write_byte(struct tl5601c_obj *obj, u8 addr, u8 val)
{
	u8 databuf[2];
	int res = 0;
	databuf[0] = addr;
	databuf[1] = val;

	mutex_lock(&obj->i2c_mutex);
	res = i2c_master_send(obj->client, databuf, 0x2);
	mutex_unlock(&obj->i2c_mutex);
	if (res != 2) 
	{
		APS_ERR("%s failed. res = %d\n", __func__, res);
		return -EFAULT;
	}

	return 0;
}
/*
static int tl5601c_write_word(struct tl5601c_obj *obj, u8 addr, u8 data_lsb, u8 data_msb)
{
	int res;
	res = tl5601c_write_byte(obj, addr, data_lsb);
	if (res < 0)
	{
		APS_ERR("%s failed, res = %d\n", __func__, res);
		return -EFAULT;
	}
	res = tl5601c_write_byte(obj, addr+1, data_msb);
	if (res < 0)
	{
		APS_ERR("%s failed, res = %d\n", __func__, res);
		return -EFAULT;
	}

	return 0;
}
*/
/*
static int aeon_read_nvm(struct tl5601c_obj *obj, int addr, int *nvm_high_byte, int *nvm_low_byte)
{
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	mutex_lock(&obj->nvm_lock);
	ret = tl5601c_write_byte(obj, ADDR_MODE, MODE_OPER_ALSPS | MODE_PMODE_NORMAL);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	msleep(NVM_DELAY);

	ret = tl5601c_write_byte(obj, 0x33, 0x10);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x22, 0xF8);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	msleep(NVM_DELAY);
	ret = tl5601c_read_byte(obj, 0x34);
	if(ret != 0x01)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x30, addr);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x33, 0x14);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	msleep(NVM_DELAY);
	ret = tl5601c_read_byte(obj, 0x34);
	if(ret != 0x01)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}

	ret = tl5601c_write_byte(obj, 0x33, 0x00);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x22, 0xFC);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}

	*nvm_high_byte = tl5601c_read_byte(obj, 0x31);
	*nvm_low_byte = tl5601c_read_byte(obj, 0x32);

	mutex_unlock(&obj->nvm_lock);
	APS_ERR("nvm_address = %d, high_byte = %d\t, low_byte = %d\n", addr, *nvm_high_byte, *nvm_low_byte);

	return 0; 

I2C_ERROR:
	mutex_unlock(&obj->nvm_lock);
	return -1;
}

static int aeon_write_nvm(struct tl5601c_obj *obj, int addr, int high_byte, int low_byte)
{
	int ret;

	mutex_lock(&obj->nvm_lock);
	ret = tl5601c_write_byte(obj, ADDR_MODE, MODE_OPER_ALSPS | MODE_PMODE_NORMAL);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	msleep(NVM_DELAY);

	ret = tl5601c_write_byte(obj, 0x33, 0x10);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x22, 0xF8);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x30, addr);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x33, 0x11);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	msleep(NVM_DELAY);
	ret = tl5601c_read_byte(obj, 0x34);
	if(ret != 0x01)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR; 
	}
	ret = tl5601c_write_byte(obj, 0x31, high_byte);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x32, low_byte);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x33, 0x12);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	msleep(NVM_DELAY);
	ret = tl5601c_read_byte(obj, 0x34);
	if(ret != 0x01)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR; 
	}

	ret = tl5601c_write_byte(obj, 0x33, 0x00);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}
	ret = tl5601c_write_byte(obj, 0x22, 0xFC);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		goto I2C_ERROR;
	}

	mutex_unlock(&obj->nvm_lock);
	return 0;

I2C_ERROR:
	mutex_unlock(&obj->nvm_lock);
	return -1;
}
*/

static int tl5601c_power_on(struct tl5601c_obj *obj)
{
	int val, ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	val = tl5601c_read_byte(obj, ADDR_MODE);
	val &= MODE_OPER_MASK;
	val |= obj->power_mode;
	ret = tl5601c_write_byte(obj, ADDR_MODE, val);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	return 0;
}

static int tl5601c_power_off(struct tl5601c_obj *obj)
{
	int val, ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	val = tl5601c_read_byte(obj, ADDR_MODE);
	val &= MODE_OPER_MASK;
	val |= MODE_PMODE_SHUTDOWN;
	ret = tl5601c_write_byte(obj, ADDR_MODE, val);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int tl5601c_read_led_off(struct tl5601c_obj *obj)
{
	unsigned int led_off_msb, led_off_lsb, led_off_data;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	led_off_msb = tl5601c_read_byte(obj, ADDR_LED_OFF_DATA_MSB);
	if(led_off_msb < 0)
	{
		APS_ERR("tl5601c_read_byte error! led_off_msb =  %d\n", led_off_msb);
		return -EFAULT;
	}
	led_off_lsb = tl5601c_read_byte(obj, ADDR_LED_OFF_DATA_LSB);
	if(led_off_lsb < 0)
	{
		APS_ERR("tl5601c_read_byte error! led_off_lsb =  %d\n", led_off_lsb);
		return -EFAULT;
	}
	led_off_data = (led_off_msb * 256) + led_off_lsb;

	return led_off_data;
}

static int tl5601c_set_res_als(struct tl5601c_obj *obj, int res_als)
{
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	ret = tl5601c_write_byte(obj, ADDR_RES_ALS, res_als);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	return 0;
}

static int tl5601c_set_range_als(struct tl5601c_obj *obj, int range_als)
{
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	ret = tl5601c_write_byte(obj, ADDR_RANGE_ALS, range_als);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	return 0;
}

static int tl5601c_st_sleep_period(struct tl5601c_obj *obj, int sleep_period)
{
	u8 ret, val;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	ret = tl5601c_read_byte(obj, ADDR_MODE_CTL);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}
	ret &= 0x0F;
	ret |= sleep_period;
	val = tl5601c_write_byte(obj, ADDR_MODE_CTL, ret);
	if(val < 0)
	{
		APS_ERR("val = %d\n", val);
		return -EFAULT;
	}

	return 0;
}

static int tl5601c_check_chipid(struct tl5601c_obj *obj)
{
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret = tl5601c_read_byte(obj, ADDR_CHIPID);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}
	if(ret == CHIPID)
		return 0;
	else
	{
		APS_ERR("read value of ADDR_CHIPID = %x, can't recognize\n", ret);
		return -1;
	}

	return 0;
}

/*
static int tl5601c_check_version(struct tl5601c_obj *obj)
{
    int ret;
	int nvm_high_byte = 0, nvm_low_byte = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
    ret = aeon_read_nvm(obj, 0x04, &nvm_high_byte, &nvm_low_byte);
    if(ret < 0)
    {
		APS_ERR("aeon_read_nvm error! ret = %d\n", ret);
		return -1;
    }

    if(nvm_low_byte != 5)
    {
		ret = aeon_write_nvm(obj, 0x04, 0, 5);
		if(ret < 0)
		{
			APS_ERR("aeon_read_nvm error! ret = %d\n", ret);
			return -1;
		}
		ret = aeon_read_nvm(obj, 0x04, &nvm_high_byte, &nvm_low_byte);
		if(ret < 0)
		{
			APS_ERR("aeon_read_nvm error! ret = %d\n", ret);
			return -1;
		}
    }
    
    return 0;
}
*/

static int tl5601c_set_ps_threshold(struct tl5601c_obj *obj, int ps_high_threshold, int ps_low_threshold) 
{
	int ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	ret = tl5601c_write_byte(obj, ADDR_PS_LTH_LSB, ps_low_threshold & 0xFF);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	ret = tl5601c_write_byte(obj, ADDR_PS_LTH_MSB, (ps_low_threshold >> 8) & 0xFF);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	ret = tl5601c_write_byte(obj, ADDR_PS_HTH_LSB, ps_high_threshold & 0xFF);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	ret = tl5601c_write_byte(obj, ADDR_PS_HTH_MSB, (ps_high_threshold >> 8) & 0xFF);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	ps_threshold_5cm = ps_low_threshold;   //lihaiyan@wind-mobi.com 20170515 modified
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_set_led_high_threshold(struct tl5601c_obj *obj, u16 led_high_threshold)
{
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret = tl5601c_write_byte(obj, ADDR_LED_HIGH_TH_MSB, led_high_threshold / 256);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}
	ret = tl5601c_write_byte(obj, ADDR_LED_HIGH_TH_LSB, led_high_threshold % 256);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_set_led_low_threshold(struct tl5601c_obj *obj, u16 led_low_threshold)
{
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	ret = tl5601c_write_byte(obj, ADDR_LED_LOW_TH_MSB, led_low_threshold / 256);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}
	ret = tl5601c_write_byte(obj, ADDR_LED_LOW_TH_LSB, led_low_threshold % 256);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_set_ps_int_filter(struct tl5601c_obj *obj, int ps_int_filter)
{
	int ret, write_data;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret = tl5601c_read_byte(obj, ADDR_INT_FILTER);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}
	ret &= 0x0F;
	write_data = ret | ps_int_filter;
	ret = tl5601c_write_byte(obj, ADDR_INT_FILTER, write_data);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	return 0;
}
// read & write register function end
/*----------------------------------------------------------------------------*/
static unsigned int tl5601c_pow(int x, int y)
{
	unsigned int value = x;
	int i;
//	if(atomic_read(&obj->trace) == 1)
//		APS_FUN(f);

	if(y == 0)
	{
		value = 1;
	}
	else if(y == 1)
	{
		value = x;
	}
	else
	{
		for(i=1; i<y; i++)
		{
			value *= x;
		}
	}
	return value;
}

int tl5601c_get_addr(struct alsps_hw *hw, struct tl5601c_i2c_addr *addr)
{
	if (!hw || !addr)
		return -EFAULT;

	addr->write_addr = hw->i2c_addr[0];

	return 0;
}

/*----------------------------------------------------------------------------*/
static void tl5601c_power(struct alsps_hw *hw, unsigned int on)
{
/*
	static unsigned int power_on;

#ifdef __USE_LINUX_REGULATOR_FRAMEWORK__
#else
	if (hw->power_id != POWER_NONE_MACRO) {
		if (power_on == on)
			APS_LOG("ignore power control: %d\n", on);
		else if (on) {
			if (!hwPowerOn(hw->power_id, hw->power_vol, "tl5601c"))
				APS_ERR("power on fails!!\n");

		} else{
			if (!hwPowerDown(hw->power_id, "tl5601c"))
				APS_ERR("power off fail!!\n");
		}
	}
#endif
	power_on = on;
*/
}
/*----------------------------------------------------------------------------*/
/*static void tl5601c_read_all_register(struct i2c_client *client)
{
	int i = 0;
	u8 buffer[32] = {0};
	struct tl5601c_obj *obj = i2c_get_clientdata(client);
#ifdef F_DEBUG
	APS_FUN(f);
#endif
	for(i=0; i<32; i++)
	{
		buffer[i] = tl5601c_read_byte(obj, i);
	}
	for(i=0; i<32; i++)
	{
		printk("reg %d = %d\n", i, buffer[i]);
	}

}*/
/*----------------------------------------------------------------------------*/
static int tl5601c_set_auto_ps_en(struct tl5601c_obj *obj, int auto_ps_en)
{
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	ret = tl5601c_write_byte(obj, ADDR_PS_LED_OFF, auto_ps_en);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}
	obj->auto_ps_en = auto_ps_en;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_set_led_ctl(struct tl5601c_obj *obj, int led_freq, int led_peak_cur, int led_mode)
{
	int ret, led_ctl;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	led_ctl = (led_freq << 4) + (led_peak_cur << 2) + led_mode;
	ret = tl5601c_write_byte(obj, ADDR_LED_CTL, led_ctl);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_set_range_ps(struct tl5601c_obj *obj, int range_ps)
{
	int ret, read_byte, write_byte;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	read_byte = tl5601c_read_byte(obj, ADDR_RANGE_PS);
	if(read_byte < 0)
	{
		APS_ERR("read_byte = %d\n", read_byte);
		return -EFAULT;
	}
	write_byte = (read_byte & 0x70) | range_ps;
	ret = tl5601c_write_byte(obj, ADDR_RANGE_PS, write_byte);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_set_res_ps(struct tl5601c_obj *obj, int res_ps)
{
	int ret, read_byte, write_byte;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	read_byte = tl5601c_read_byte(obj, ADDR_RES_PS);
	if(read_byte < 0)
	{
		APS_ERR("read_byte = %d\n", read_byte);
		return -EFAULT;
	}
	write_byte = (read_byte & 0x70) | res_ps;
	ret = tl5601c_write_byte(obj, ADDR_RES_PS, write_byte);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
/*
static int tl5601c_set_ps_offset(struct tl5601c_obj *obj, int ps_offset)
{	
	int ret;
#ifdef F_DEBUG
	APS_FUN(f);
#endif
	ret = tl5601c_write_byte(obj, ADDR_PS_OFFSET, ps_offset);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	return 0;
}*/
/*----------------------------------------------------------------------------*/
static int tl5601c_clear_interrupt(struct tl5601c_obj *obj)
{
	int ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret = tl5601c_read_byte(obj, ADDR_INT_STATUS);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_enable_ps_interrupt(struct tl5601c_obj *obj)
{
	int ret;
	u8 write_data;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret = tl5601c_read_byte(obj, ADDR_INT_EN);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	write_data = (ret | PS_INT_ENABLE_PH_PL);
	ret = tl5601c_write_byte(obj, ADDR_INT_EN, write_data);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return ret;
	}
	
	ret = tl5601c_set_ps_int_filter(obj, obj->ps_int_filter);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int tl5601c_disable_ps_interrupt(struct tl5601c_obj *obj)
{
	int ret, val;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	val = tl5601c_read_byte(obj, ADDR_INT_EN);
	if(val < 0)
	{
		APS_ERR("val = %d\n", val);
		return -1;
	}
	val &= ~PS_INT_ENABLE_PH_PL;
	ret = tl5601c_write_byte(obj, ADDR_INT_EN, val);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
/*
static int tl5601c_setup(struct tl5601c_obj *obj)
{
	int ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	ret += tl5601c_set_res_als(obj, obj->res_als);
	ret += tl5601c_set_range_als(obj, obj->range_als);
	ret += tl5601c_set_res_ps(obj, obj->res_ps);
	ret += tl5601c_set_range_ps(obj, obj->ps_range);
	ret += tl5601c_write_byte(obj, ADDR_INT_FILTER, obj->ps_int_filter);
	ret += tl5601c_clear_interrupt(obj);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
*/

/*----------------------------------------------------------------------------*/
static int tl5601c_check_and_clear_intr(struct i2c_client *client)
{
	struct tl5601c_obj *obj = i2c_get_clientdata(client);
	int int_status;

	if (NULL == obj)
		goto EXIT_ERR;

	if (atomic_read(&(obj->trace)) <= 1)
		APS_FUN(f);

	/* Get Int status */
	int_status = tl5601c_read_byte(obj, ADDR_INT_STATUS);
	if (int_status < 0)
		goto EXIT_ERR;

	return int_status;

EXIT_ERR:
	APS_ERR("tl5601c_check_and_clear_intr fail\n");
	return -EINVAL;
}

/*----------------------------------------------------------------------------*/
void tl5601c_eint_func(void)
{
	struct tl5601c_obj *obj = tl5601c_obj;
//	if(atomic_read(&obj->trace) == 1)
//		APS_FUN(f);

	if (!obj)
		return;

	if (atomic_read(&(obj->trace)) <= 1)
		APS_FUN(f);

	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
/* This function depends the real hw setting, customers should modify it. 2012/5/10 YC. */
int tl5601c_setup_eint(struct i2c_client *client)
{
	int ret;
	u32 ints[2] = {0, 0};
#ifndef PLATFORM_VERSION_AND_5
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
#endif

	if(atomic_read(&tl5601c_obj->trace) == 1)
		APS_FUN(f);
	/*configure to GPIO function, external interrupt*/

	if(tl5601c_obj == NULL)
	{
		APS_ERR("tl5601c_obj is null!\n");
		return -EINVAL;
	}

#ifndef	PLATFORM_VERSION_AND_5 // android 6.0 code
    alspsPltFmDev = get_alsps_platformdev();
/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");

	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
/* eint request */
	if (tl5601c_obj->irq_node) 
	{
		of_property_read_u32_array(tl5601c_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		tl5601c_obj->irq = irq_of_parse_and_map(tl5601c_obj->irq_node, 0);
		APS_LOG("tl5601c_obj->irq = %d\n", tl5601c_obj->irq);
		if (!tl5601c_obj->irq) 
		{
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
        printk("====================================================== %s ,%d\n", __func__, __LINE__);
		if (request_irq(tl5601c_obj->irq, tl5601c_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
#if defined(CONFIG_OF)
			enable_irq(tl5601c_obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
			mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
			mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
	} 
	else 
	{
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

#else	// android 5.0 code
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
//	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
#if defined(MT6575) || defined(MT6571) || defined(MT6589)
	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_EDGE_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLRITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tl5601c_eint_func, 0);
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#elif defined(CONFIG_OF)	
	if (tl5601c_obj->irq_node != NULL) 
	{
		of_property_read_u32_array(tl5601c_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		APS_LOG("ints[0]=%d,ints[1]=%d\n", ints[0], ints[1]);
		mt_gpio_set_debounce(ints[0], ints[1]);

		tl5601c_obj->irq = irq_of_parse_and_map(tl5601c_obj->irq_node, 0);
		if (tl5601c_obj->irq == 0) 
		{
			APS_ERR("irq_of_parse_and_map failed!\n");
			return -EINVAL;
		}

		ret = request_irq(tl5601c_obj->irq, tl5601c_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL);
		if (ret != 0) 
		{
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq(tl5601c_obj->irq);
	}
	else 
	{
		APS_ERR("tl5601c_obj->irq_node is null!!\n");
		return -EINVAL;
	}
#else
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, tl5601c_eint_func, 0);
	mt_eint_mask(CUST_EINT_ALS_NUM);
#endif

#endif	// PLATFORM_VERSION_AND_5 



	return 0;
}
/*----------------------------------------------------------------------------*/
/*
static int tl5601c_disable_eint(struct i2c_client *client)
{
#ifdef F_DEBUG
	APS_FUN(f);
#endif
//	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
//	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
//	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, FALSE);
//	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_DOWN);

//	mt_eint_mask(CUST_EINT_ALS_NUM);

	return 0;
}
*/
/*----------------------------------------------------------------------------*/
static void tl5601c_cal_lr(struct tl5601c_obj *obj)
{
	u8 read_data = 0x00;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	read_data = tl5601c_read_byte(obj, ADDR_RES_ALS);
	obj->res_als = read_data & 0x07;
	read_data = tl5601c_read_byte(obj, ADDR_RANGE_ALS);
	obj->range_als = read_data & 0x07;
	if(obj->res_als <= RES_ALS_17BIT)
	{
		obj->res_als = RES_ALS_14BIT;
		APS_ERR("if RES_ALS >= 17BIT, lux can't calculate\n");
	}
	obj->res_als_tb = tl5601c_res_als_tb[obj->res_als];
	obj->range_als_tb = tl5601c_range_als_tb[obj->range_als];
}
/*----------------------------------------------------------------------------*/
static int tl5601c_get_lux(struct tl5601c_obj *obj)
{
	unsigned long long calculated_lux = 0;
	unsigned int lux = 0;
	unsigned long long lr;
	u32 als0_data, als1_data, ps_data;
	unsigned long long light_a, light_b, light_c;
	int range_als, res_als, range_ps, res_ps;
	int led_off_msb, led_off_lsb, led_off;
	unsigned int ps_high_threshold, ps_low_threshold;
	unsigned int led_off_high_threshold, led_off_low_threshold;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	mutex_lock(&obj->als_mutex);
	als0_data = i2c_smbus_read_word_data(obj->client, ADDR_ALS0_DATA_LSB);
	als1_data = i2c_smbus_read_word_data(obj->client, ADDR_ALS1_DATA_LSB);
	ps_data = i2c_smbus_read_word_data(obj->client, ADDR_PS_DATA_LSB);
	mutex_unlock(&obj->als_mutex);

	tl5601c_cal_lr(obj);

	light_a = (2005 * als1_data) - (60 * als0_data);
	light_b = (1455 * als1_data) + (578 * als0_data);
	light_c = (867 * als1_data) + (1129 * als0_data);

	if(light_a > light_b)
		calculated_lux = light_a;
	else
		calculated_lux = light_b;
	if(calculated_lux > light_c)
		;
	else
		calculated_lux = light_c;

	if((int)calculated_lux <= 0)
		calculated_lux = 0;
	else
		do_div(calculated_lux, 100);

	lr = (G_A * obj->range_als_tb) / obj->res_als_tb;
	calculated_lux *= lr;

	if(calculated_lux != 0)
		do_div(calculated_lux, 10000);

	if(atomic_read(&obj->debug_als) == 1)
	{
		range_als = tl5601c_read_byte(obj, ADDR_RANGE_ALS);
		res_als = tl5601c_read_byte(obj, ADDR_RES_ALS);
		range_ps = tl5601c_read_byte(obj, ADDR_RANGE_PS);
		res_ps = tl5601c_read_byte(obj, ADDR_RES_PS);
		ps_low_threshold = i2c_smbus_read_word_data(obj->client, ADDR_PS_LTH_LSB);
		ps_high_threshold = i2c_smbus_read_word_data(obj->client, ADDR_PS_HTH_LSB);
		led_off_high_threshold = i2c_smbus_read_word_data(obj->client, ADDR_LED_HIGH_TH_LSB);
		led_off_low_threshold = i2c_smbus_read_word_data(obj->client, ADDR_LED_LOW_TH_LSB);
		led_off_msb = tl5601c_read_byte(obj, ADDR_LED_OFF_DATA_MSB);
		led_off_lsb = tl5601c_read_byte(obj, ADDR_LED_OFF_DATA_LSB);
		led_off = (led_off_msb * 256) + led_off_lsb;
		APS_DBG("als0_data = %d, als1_data = %d, ps_data = %d, ps_low_th = %d, ps_high_th = %d, led_off = %d, led_high_th = %d, led_low_th = %d\n",
				als0_data, als1_data, ps_data, ps_low_threshold, ps_high_threshold, led_off, led_off_high_threshold, led_off_low_threshold);
		APS_DBG("range_als = %d, res_als = %d, range_ps = %d, res_ps =%d\n",range_als, res_als, range_ps, res_ps);
	}

	lux = (int)calculated_lux;

	if(lux < 0)
	{
		lux = 0;
	}
	else if(lux > 25000)

	{
		lux = 25000;
	}

	if(atomic_read(&obj->debug_als) == 1)
	{
		APS_DBG("lux = %d\n", lux);
	}

	obj->als0_data = als0_data;
	obj->als1_data = als1_data;
	obj->lux = lux;
	if(obj->range_als == RANGE_ALS_X1)
	{
		obj->abs_als0_data = als0_data;
		obj->abs_als1_data = als1_data;
	}
	else
	{
		obj->abs_als0_data = als0_data * tl5601c_pow(2, obj->range_als);
		obj->abs_als1_data = als1_data * tl5601c_pow(2, obj->range_als);
	}

	if((als0_data >= ((tl5601c_als_max_tb[obj->res_als] * 8) / 10)) || (als1_data >= ((tl5601c_als_max_tb[obj->res_als] * 8) / 10)))
	{
		if(obj->range_als < 6)
		{
			tl5601c_set_range_als(obj, obj->range_als+1);
			obj->range_als += 1;
		}
	}
	else if((als0_data <= ((tl5601c_als_max_tb[obj->res_als] * 2) / 10)) || (als1_data <= ((tl5601c_als_max_tb[obj->res_als] *2) / 10)))
	{
		if((obj->range_als == 1) && (als1_data >= (tl5601c_als_max_tb[obj->res_als]/2)))
		{
			;
		}
		else if(obj->range_als > 0)
		{
			tl5601c_set_range_als(obj, obj->range_als-1);
			obj->range_als -= 1;
		}
	}
	return lux;
}
/*----------------------------------------------------------------------------*/
int tl5601c_read_als(struct i2c_client *client, u16 *data)
{
	struct tl5601c_obj *obj = i2c_get_clientdata(client);
	int lux;

	if (obj == NULL) {
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -EINVAL;
	}
	if (atomic_read(&obj->trace) == 1)
		APS_FUN();

	lux = tl5601c_get_lux(obj);

	*data = (u16)lux;

	if (*data < 0) 
	{
		*data = 0;
		APS_DBG("als_value is invalid!!\n");
		return -EINVAL;
	}
	if (atomic_read(&obj->trace) == 2)
		APS_DBG("als_data = %d!\n", *data);
	
	ps_threshold_5cm = obj->ps_dyn_low_threshold;   //lihaiyan@wind-mobi.com 20170515 modified	
		
	return 0;
}

/*----------------------------------------------------------------------------*/
static int tl5601c_get_als_value(struct tl5601c_obj *obj, u16 als)
{

	int invalid = 0;
	
	als = filter_als_value(als); //lihaiyan@wind-mobi.com 20170327


	if (obj == NULL) {
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -EINVAL;
	}
	if (atomic_read(&obj->trace) == 1)
		APS_FUN();


	if (1 == atomic_read(&obj->als_deb_on)) {
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if (time_after(jiffies, endt))
			atomic_set(&obj->als_deb_on, 0);

		if (1 == atomic_read(&obj->als_deb_on))
			invalid = 1;
	}

	if (!invalid) {
		if (atomic_read(&obj->trace) <= 2)
			APS_DBG("ALS: %05d => %05d\n", als, als);

		return als;
	} else{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, als);
		return -EINVAL;
	}
}

static int32_t tl5601c_read_ps(struct tl5601c_obj *obj)
{
	u16 buffer;
	int32_t ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	buffer = tl5601c_read_byte(obj, ADDR_PS_DATA_LSB);
	if(buffer < 0)
	{
		APS_ERR("tl5601c_read_byte error! buffer =  %d\n", buffer);
		goto EXIT_ERR;
	}
	ret = tl5601c_read_byte(obj, ADDR_PS_DATA_MSB);
	if(ret < 0)
	{
		APS_ERR("tl5601c_read_byte error! ret = %d\n", ret);
		goto EXIT_ERR;
	}
	ret *= 256;
	ret += buffer;

	ps_threshold_5cm = obj->ps_dyn_low_threshold;   //lihaiyan@wind-mobi.com 20170515 modified
	return ret;

EXIT_ERR:
	APS_ERR("tl5601c_read_ps fail!\n");
	return -EFAULT;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_get_ps_value(struct tl5601c_obj *obj, u16 ps)
{
	int val = 0;
	int invalid = 0;
	int int_status;

	if (obj == NULL) {
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -EINVAL;
	}
	if (atomic_read(&obj->trace) == 1)
		APS_FUN();

	int_status = tl5601c_check_and_clear_intr(obj->client);
	if(int_status < 0)
	{
		APS_ERR("can't read int status = %d\n", int_status);
		return -EINVAL;
	}

	if(ps > obj->ps_dyn_low_threshold)
		val = 0;
	else
		val = 1;

//	if (ps > atomic_read(&obj->ps_threshold_high))
//		val = 0;	/*close*/
//	else if (ps < atomic_read(&obj->ps_threshold_low))
//		val = 1;	/*far away*/

	if (atomic_read(&obj->ps_suspend))
		invalid = 1;
	else if (1 == atomic_read(&obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if (time_after(jiffies, endt))
			atomic_set(&obj->ps_deb_on, 0);

		if (1 == atomic_read(&obj->ps_deb_on))
			invalid = 1;

	}

	if (!invalid) {
		if (atomic_read(&obj->trace) <= 2)
			APS_DBG("PS:	%05d => %05d(0:close,1:far away)\n", ps, val);

		return val;
	} else{
		APS_ERR("PS invalid:	%d\n", invalid);
		return -EINVAL;
	}
}

/*
static unsigned long tl5601c_mean(unsigned long *array, int array_count)
{
	int i;
	unsigned long mean, sum = 0;
	
	for(i=0; i< array_count; i++)
	{
		sum += array[i];
	}
	mean = sum / array_count;
	
	return mean;
}

static unsigned int tl5601c_stdev(unsigned long *array, int array_count)
{
	int i;
	unsigned long mean, multi_mean;
	unsigned int stdev;
	unsigned long multi_array[array_count];
	
	for(i = 0; i< array_count; i++)
	{
		multi_array[i] = array[i] * array[i];
	}	
	multi_mean = tl5601c_mean(multi_array, array_count);
	mean = tl5601c_mean(array, array_count);
	stdev = multi_mean - (mean * mean);
	stdev = int_sqrt(stdev);

	return stdev;
}
*/

static int tl5601c_check_saturation(struct tl5601c_obj *obj, int led_off_data)
{
	int ret = 0;
	if(led_off_data > obj->saturation_threshold)
		ret = LIGHT_OVER;
	else
		ret = LIGHT_NORMAL;

	return ret;
}

/*----------------------------------------------------------------------------*/
static enum hrtimer_restart tl5601c_prox_poll_timer_func(struct hrtimer *timer)
{
	struct tl5601c_obj *obj = container_of(timer, struct tl5601c_obj, prox_poll_timer);
	queue_work(obj->wq_prox_poll, &obj->work_prox_poll);
	hrtimer_forward_now(&obj->prox_poll_timer, obj->prox_poll_delay);
	return HRTIMER_RESTART;
}

static enum hrtimer_restart tl5601c_prox_init_crt_timer_func(struct hrtimer *timer)
{
	struct tl5601c_obj *obj = container_of(timer, struct tl5601c_obj, prox_init_crt_timer);
	queue_work(obj->wq_prox_poll, &obj->work_prox_init_crt);
	hrtimer_forward_now(&obj->prox_init_crt_timer, obj->prox_init_crt_delay);
	return HRTIMER_RESTART;
}

static int tl5601c_enable_mode_als(struct tl5601c_obj *obj, int enable_als)
{
	u8 write_byte;
	int ret, enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	if(enable_als)
	{
		if(enable_ps)
		{
			write_byte = MODE_OPER_ALSPS | MODE_PMODE_NORMAL;
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
		}
		else
		{
			write_byte = MODE_OPER_ALS | MODE_PMODE_NORMAL;
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
		}
		obj->power_mode = MODE_PMODE_NORMAL;
	}
	else
	{
		if(enable_ps)
		{
			write_byte = MODE_OPER_ALSPS | MODE_PMODE_NORMAL;
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
			obj->power_mode = MODE_PMODE_NORMAL;
		}
		else
		{
			write_byte = MODE_OPER_ALSPS | MODE_PMODE_SHUTDOWN;
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
			obj->power_mode = MODE_PMODE_SHUTDOWN;
		}
	}
	return 0;
}

static int tl5601c_enable_mode_ps(struct tl5601c_obj *obj, int enable_ps)
{
	u8 write_byte;
	int ret, enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	if(enable_ps)
	{
		if(enable_als)
		{
			write_byte = MODE_OPER_ALSPS | MODE_PMODE_NORMAL;
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
		}
		else
		{
			//shlee 170403   //mahongyu@wind-mobi.com
			write_byte = MODE_OPER_ALSPS | MODE_PMODE_NORMAL;
			//
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
		}
		obj->power_mode = MODE_PMODE_NORMAL;
	}
	else
	{
		if(enable_als)
		{
			write_byte = MODE_OPER_ALS | MODE_PMODE_NORMAL;
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
			obj->power_mode = MODE_PMODE_NORMAL;
		}
		else
		{
			write_byte = MODE_OPER_ALSPS | MODE_PMODE_SHUTDOWN;
			ret = tl5601c_write_byte(obj, ADDR_MODE, write_byte);
			if(ret < 0)
			{
				APS_ERR("tl5601c_write_byte ERR, ret = %d\n", ret);
				return -EFAULT;
			}
			obj->power_mode = MODE_PMODE_SHUTDOWN;
		}
	}

	return 0;
}

static int tl5601c_light_enable(struct tl5601c_obj *obj)
{
	int ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret += tl5601c_set_res_als(obj, obj->res_als);
	ret += tl5601c_set_range_als(obj, obj->range_als);
	ret += tl5601c_enable_mode_als(obj, ENABLE);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

//	APS_LOG("starting poll timer, delay %lldns\n",
//			ktime_to_ns(obj->light_poll_delay));
//	hrtimer_start(&obj->light_timer, obj->light_poll_delay, HRTIMER_MODE_REL);

	return 0;
}

static int tl5601c_light_disable(struct tl5601c_obj *obj)
{
	int ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

//	APS_LOG("cancelling poll timer\n");
//	hrtimer_cancel(&obj->light_timer);
//	cancel_work_sync(&obj->work_light);
	ret = tl5601c_enable_mode_als(obj, DISABLE);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_proximity_enable(struct tl5601c_obj *obj)
{
	int ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret += tl5601c_set_auto_ps_en(obj, obj->auto_ps_en);
	ret += tl5601c_set_led_ctl(obj, obj->led_freq, obj->led_peak_cur, obj->led_mode);
	ret += tl5601c_set_res_ps(obj, obj->res_ps);
	ret += tl5601c_set_range_ps(obj, obj->range_ps);
	ret += tl5601c_set_ps_int_filter(obj, obj->ps_int_filter);
	ret += tl5601c_clear_interrupt(obj);
	ret += tl5601c_set_led_high_threshold(obj, (obj->ps_max * 90) / 100);
	// shlee 170403    //mahongyu@wind-mobi.com
	ret += tl5601c_set_res_als(obj, obj->res_als);
	ret += tl5601c_set_range_als(obj, obj->range_als);
	//
	ret += tl5601c_enable_mode_ps(obj, ENABLE);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	obj->ps_crosstalk += 30;
	if(obj->ps_crosstalk > obj->ps_crosstalk_max)
		obj->ps_crosstalk = obj->ps_crosstalk_max;
	obj->ps_dyn_high_threshold = obj->ps_crosstalk + obj->ps_high_threshold;
	obj->ps_dyn_low_threshold = obj->ps_crosstalk + obj->ps_low_threshold;
	if(obj->ps_dyn_high_threshold > obj->ps_max)
		obj->ps_dyn_high_threshold = obj->ps_max;
	if(obj->ps_dyn_low_threshold > obj->ps_max)
		obj->ps_dyn_low_threshold = obj->ps_max;
	tl5601c_set_ps_threshold(obj, obj->ps_dyn_high_threshold, 0);
	obj->proximity_status = FAR;
	
	work_crt_repeat_count = 10;
	if(obj->ps_auto_calibration == 1)
		hrtimer_start(&obj->prox_poll_timer, obj->prox_poll_delay, HRTIMER_MODE_REL);
	if(obj->ps_enable_interrupt == 1)
		tl5601c_enable_ps_interrupt(obj);

	obj->ps_en_cnt++;
	APS_DBG("Ps_En_Cnt = %d, Ps_Close_Cnt = %d \n",obj->ps_en_cnt,obj->ps_close_cnt);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_proximity_disable(struct tl5601c_obj *obj)
{
	int ret = 0;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	hrtimer_cancel(&obj->prox_poll_timer);
	cancel_work_sync(&obj->work_prox_poll);
	ret += tl5601c_disable_ps_interrupt(obj);
	ret += tl5601c_set_led_high_threshold(obj, 0);
	ret += tl5601c_set_led_low_threshold(obj, 0);
	obj->proximity_status = FAR;
	ret += tl5601c_enable_mode_ps(obj, DISABLE);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}
    obj->ps_close_cnt++;
	APS_DBG("Ps_En_Cnt = %d, Ps_Close_Cnt = %d \n",obj->ps_en_cnt,obj->ps_close_cnt);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_enable_als(struct i2c_client *client, int enable)
{
	struct tl5601c_obj *obj = i2c_get_clientdata(client);

	if (client == NULL) 
	{
		APS_DBG("CLIENT CAN'T EQUAL NULL\n");
		return -EINVAL;
	}
	if (atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	if (enable)
	{
//		tl5601c_setup(obj);
//		tl5601c_enable_als(obj);
		tl5601c_light_enable(obj);
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		APS_DBG("tl5601c ALS enable\n");
	} 
	else
	{
		tl5601c_light_disable(obj);
//		tl5601c_disable_als(obj);
		atomic_set(&obj->als_deb_on, 0);
		APS_DBG("tl5601c ALS disable\n");
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_enable_ps(struct i2c_client *client, int enable)
{
	struct tl5601c_obj *obj = i2c_get_clientdata(client);

	if (client == NULL) 
	{
		APS_DBG("CLIENT CAN'T EQUAL NULL\n");
		return -EINVAL;
	}

	if (atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	if (enable) 
	{
		tl5601c_proximity_enable(obj);
	
		if (obj->hw->polling_mode_ps != 0) 
		{	/* set debounce just with polling */
			wake_lock(&ps_lock);
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		} 
		else 
		{
#if defined(CONFIG_OF)
			enable_irq(obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
			mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
			mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
		}
		APS_DBG("tl5601c PS enable\n");
	} 
	else
	{
		tl5601c_proximity_disable(obj);

		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tl5601c PS disable\n");

		if (0 == obj->hw->polling_mode_ps) 
		{
#ifdef MT6516        
			MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  
#elif ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572))
			mt65xx_eint_mask(CUST_EINT_ALS_NUM);    
#else				
	//mt_eint_mask(CUST_EINT_ALS_NUM);
			disable_irq_nosync(obj->irq);
#endif	
			cancel_work_sync(&obj->eint_work);
		} 
		else
			wake_unlock(&ps_lock);

	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
// init structure
static int tl5601c_init_data(struct tl5601c_obj *obj)
{
	int ret;
	obj->led_freq = LED_CTL_LFREQ_327;
	obj->led_peak_cur = LED_CTL_LPEAK_130;
	obj->led_mode = LED_CTL_LMOD_25;
	obj->ps_debug = 0;
	obj->power_mode = MODE_PMODE_SHUTDOWN;
	obj->sleep_period = SLEEP_DURATION_2;
	obj->ps_int_filter = PS_INT_FILTER_5;
	obj->als_int_filter = ALS_INT_FILTER_2;
	obj->auto_ps_en = ENABLE;
	obj->res_als = RES_ALS_14BIT;
	obj->range_als = RANGE_ALS_X4;
	obj->res_ps = RES_PS_12BIT;
	obj->ps_max = tl5601c_ps_max_tb[obj->res_ps];
	obj->range_ps = RANGE_PS_X64;
	obj->proximity_status = FAR;
//	obj->ps_crosstalk_def = 300;
	obj->ps_crosstalk = obj->ps_max;
//	obj->ps_crosstalk = obj->ps_crosstalk_def;
	obj->ps_enable_interrupt = 1;
	obj->ps_auto_calibration = 1;
//	obj->led_off_noise_threshold = ((obj->ps_max * 50)/100);
	obj->ps_high_threshold = atomic_read(&obj->ps_threshold_high);
	obj->ps_low_threshold = atomic_read(&obj->ps_threshold_low);
	obj->saturation_threshold = (obj->ps_max * 25) / 100;
	obj->resume_als_enable = 0;
	atomic_set(&obj->debug_als, 0);
	atomic_set(&obj->debug_ps, 1);
    obj->ps_en_cnt = 0;
	obj->ps_close_cnt = 0;
#ifdef F_DEBUG
	atomic_set(&obj->trace, 1);
#else
	atomic_set(&obj->trace, 2);
#endif
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret = tl5601c_st_sleep_period(obj, obj->sleep_period);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}

	init_crt_repeat_count = INIT_CRT_REPEAT_COUNT;
	cal_init_crosstalk = false;
	obj->ps_crosstalk_min = 0;
	obj->ps_crosstalk_max = 1000;   // 160  wangjun@wind-mobi.com
// pdata power on
// pdata led power on
	ret += tl5601c_power_on(obj);
	ret += tl5601c_set_auto_ps_en(obj, obj->auto_ps_en);
	ret += tl5601c_set_led_ctl(obj, obj->led_freq, obj->led_peak_cur, obj->led_mode);
	ret += tl5601c_set_res_ps(obj, obj->res_ps);
	ret += tl5601c_set_range_ps(obj, obj->range_ps);
	ret += tl5601c_set_ps_int_filter(obj, obj->ps_int_filter);
	ret += tl5601c_clear_interrupt(obj);
	ret += tl5601c_set_led_high_threshold(obj, (obj->ps_max * 90) / 100);
	//
//	tl5601c_setup(obj);
//	tl5601c_disable_als(obj);
//	clear_bit(CMC_BIT_ALS, &obj->enable);
	//
//	tl5601c_enable_ps(obj);
	ret += tl5601c_enable_mode_ps(obj, ENABLE);
	if(ret < 0)
	{
		APS_ERR("ret = %d\n", ret);
		return -EFAULT;
	}
	set_bit(CMC_BIT_PS, &obj->enable);
	usleep_range(15000, 20000);
//	msleep(200);
	//
//	tl5601c_set_range_ps(obj, obj->ps_range);
//	tl5601c_set_res_ps(obj, obj->res_ps);
	//
//	tl5601c_set_led_high_threshold(obj, (obj->ps_max * 90) / 100);
	hrtimer_start(&obj->prox_init_crt_timer, 
			obj->prox_init_crt_delay, HRTIMER_MODE_REL);
	return 0;
}

static int tl5601c_init_client(struct i2c_client *client)
{
	struct tl5601c_obj *obj = (struct tl5601c_obj *)i2c_get_clientdata(client);
	int ret;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	ret = tl5601c_check_chipid(obj);
	if(ret < 0)
	{
		APS_ERR("%s: tl5601c_check_chipid failed, can't acknowledge ADDR = 0 (CHIPID)\n", __func__);
		return -ENODEV;
	}

	ret = tl5601c_init_data(obj);
	if (ret < 0) {
		APS_ERR("tl5601c_init_data ERR, ret = %d\n", ret);
		return ret;
	}
	ret = tl5601c_setup_eint(client);
	if (ret < 0) {
		APS_ERR("init dev: set up eint ERR = %d\n", ret);
		return ret;
	}
	ret = tl5601c_check_and_clear_intr(client);
	if (ret < 0) {
		APS_ERR("init dev: check/clear intr ERR = %d\n", ret);
		return ret;
	}

	return SUCCESS;

//EXIT_ERR_I2C:
//	APS_ERR("init dev: I2C ERR = %d\n", ret);
//	return ret;
}
/*----------------------------------------------------------------------------*/
static void tl5601c_work_func_prox_poll(struct work_struct *work)
{
	struct tl5601c_obj *obj = container_of(work, struct tl5601c_obj, work_prox_poll);
	u16 ps_data, led_off_data;
	int enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	APS_ERR("enable_ps = %d\n", enable_ps);
	if(!enable_ps)
		return;

	mutex_lock(&obj->prox_poll_mutex);
	ps_data = tl5601c_read_ps(obj);
	led_off_data = tl5601c_read_led_off(obj);
	mutex_unlock(&obj->prox_poll_mutex);
	obj->ps_data = ps_data;
	obj->saturation = tl5601c_check_saturation(obj, led_off_data);
	if(obj->saturation == LIGHT_OVER)
		saturation_count = 10;
	else
		saturation_count -= 1;
	if(saturation_count < 0)
		saturation_count = 0;


	if(atomic_read(&obj->debug_ps) == 1)
	{
		APS_LOG("ps_data = %d, led_off_data = %d, saturation = %d, ps_crosstalk = %d,ps_high_th = %d,ps_low_th = %d, proximity_status = %d, ps_enable_cnt = %d, ps_disable_cnt = %d\n",
				ps_data, led_off_data, obj->saturation, obj->ps_crosstalk,obj->ps_dyn_high_threshold,obj->ps_dyn_low_threshold, obj->proximity_status, obj->ps_en_cnt, obj->ps_close_cnt);
	}

	if(work_crt_repeat_count > 0)
	{
		work_crt_repeat_count--;
		return;
	}

	if((obj->proximity_status == FAR)
			&& (obj->saturation == LIGHT_NORMAL)
			&& ((ps_data + led_off_data) < obj->ps_crosstalk)
			&& (saturation_count == 0))
	{
		tl5601c_disable_ps_interrupt(obj);
		obj->ps_crosstalk = ps_data + led_off_data;
		obj->ps_dyn_high_threshold = obj->ps_crosstalk + obj->ps_high_threshold;
		obj->ps_dyn_low_threshold = obj->ps_crosstalk + obj->ps_low_threshold;
		tl5601c_set_ps_threshold(obj, obj->ps_dyn_high_threshold, PROX_MIN);
		if(obj->ps_enable_interrupt == 1)
			tl5601c_enable_ps_interrupt(obj);
		if(atomic_read(&obj->debug_ps) == 1)
		{
			APS_ERR("ps_crosstalk = %d, ps_dyn_high_th = %d, ps_dyn_low_th = %d\n",
					obj->ps_crosstalk, obj->ps_dyn_high_threshold, obj->ps_dyn_low_threshold);
		}
	}
}

static void tl5601c_work_func_prox_init_crt(struct work_struct *work)
{
	struct tl5601c_obj *obj = container_of(work, struct tl5601c_obj, work_prox_init_crt);
	int ps_data, led_off_data;
	int enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	if(!enable_ps)
		return;

	mutex_lock(&obj->prox_poll_mutex);
	ps_data = tl5601c_read_ps(obj);
	led_off_data = tl5601c_read_led_off(obj);
	mutex_unlock(&obj->prox_poll_mutex);

	obj->saturation = tl5601c_check_saturation(obj, led_off_data);
	if(atomic_read(&obj->debug_ps) == 1)
	{
		APS_ERR("ps_data = %d, led_off_data = %d, saturation = %d\n",
				ps_data, led_off_data, obj->saturation);
	}

	if(init_crt_repeat_count >= INIT_CRT_REPEAT_COUNT - 10)
	{
		init_crt_repeat_count--;
		return;
	}

	if(init_crt_repeat_count <= 0)
	{
		APS_ERR("end calculate init crosstalk\n");
		hrtimer_cancel(&obj->prox_init_crt_timer);
		tl5601c_set_led_high_threshold(obj, 0);
		obj->proximity_status = FAR;
		tl5601c_enable_mode_ps(obj, DISABLE);
#ifdef GALAXY_EXPRESS
		obj->pdata->led_on(false);
		obj->pdata->power(false);
#endif
		clear_bit(CMC_BIT_PS, &obj->enable);
	}
	else
	{
		if((obj->saturation == LIGHT_NORMAL)
				&& (ps_data + led_off_data < obj->ps_crosstalk_max)
				&& (ps_data + led_off_data > obj->ps_crosstalk_min)
				&& ((ps_data + led_off_data) < obj->ps_crosstalk))
		{
			obj->ps_crosstalk = ps_data + led_off_data;
			APS_ERR("obj->ps_crosstalk = %d\n", obj->ps_crosstalk);
		}
	}
	init_crt_repeat_count--;
}
/*----------------------------------------------------------------------------*/
static void tl5601c_eint_work(struct work_struct *work)
{
	struct tl5601c_obj *obj = (struct tl5601c_obj *)container_of(work, struct tl5601c_obj, eint_work);
	int proximity_value = -1, int_status, led_off_data;
	int res;
//	hwm_sensor_data sensor_data;
	int enable_ps;
	u16 ps_data;
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);
	
	enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
	APS_ERR("enable_ps = %d\n", enable_ps);

	if (!enable_ps)
		goto exit_err;

	if (obj == NULL) 
	{
		APS_DBG("CLIENT CAN'T EQUAL NULL\n");
		goto exit_err;
	}

	int_status = tl5601c_read_byte(obj, ADDR_INT_STATUS);
	if (int_status < 0) 
	{
		APS_ERR("can't read int status = %d\n", int_status);
		goto exit_err;
	}

	led_off_data = tl5601c_read_led_off(obj);
	if(led_off_data < 0)
	{
		APS_ERR("can't read led_off_data = %d\n", led_off_data);
		goto exit_err;
	}

	if(int_status & PHT_INT && (led_off_data < 2000))
	{
		proximity_value = 0;
		res = ps_report_interrupt_data(proximity_value);
		if (res < 0)
			APS_ERR("call ps_report_interrupt_data fail = %d\n", res);

		APS_LOG("tl5601c_eint_work close\n");
		APS_LOG("close [%s] prox value = %d\n", __func__, proximity_value);
		obj->proximity_status = CLOSE;
		tl5601c_set_ps_threshold(obj, PROX_MAX, obj->ps_dyn_low_threshold);
		if(atomic_read(&obj->debug_ps) == 1)
		{
			ps_data = i2c_smbus_read_word_data(obj->client, ADDR_PS_DATA_LSB);
			APS_ERR("ps_data = %d, ps_crosstalk = %d, ps_high_threshold = %d, ps_low_threshold = %d, int_status = %d\n",
					ps_data, obj->ps_crosstalk, obj->ps_dyn_high_threshold, obj->ps_dyn_low_threshold, int_status);
		}
		work_crt_repeat_count = 10;
	}
	else if(int_status & PLT_INT)
	{
		proximity_value = 1;
		res = ps_report_interrupt_data(proximity_value);
		if (res < 0)
			APS_ERR("call ps_report_interrupt_data fail = %d\n", res);

		APS_LOG("tl5601c_eint_work far away\n");
		APS_LOG("far [%s] prox value = %d\n", __func__, proximity_value);
		obj->proximity_status = FAR;
		tl5601c_set_ps_threshold(obj, obj->ps_dyn_high_threshold, PROX_MIN);
		if(atomic_read(&obj->debug_ps) == 1)
		{
			ps_data = i2c_smbus_read_word_data(obj->client, ADDR_PS_DATA_LSB);
			APS_ERR("ps_data = %d, ps_crosstalk = %d, ps_high_threshold = %d, ps_low_threshold = %d, int_status = %d\n",
				ps_data, obj->ps_crosstalk, obj->ps_dyn_high_threshold, obj->ps_dyn_low_threshold, int_status);
		}
		work_crt_repeat_count = 10;
	}
	else
	{
		if(int_status != 0)
			APS_ERR("can't operate int_status = %d\n", int_status);
	}

	if (atomic_read(&obj->debug_ps) == 1)
		APS_LOG("tl5601c eint_work distance = %d\n", proximity_value);

exit_err:
#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
	return;
}
/*----------------------------------------------------------------------------*/
#if defined(CONFIG_OF)
static irqreturn_t tl5601c_eint_handler(int irq, void *desc)
{
	struct tl5601c_obj *obj = tl5601c_obj;
//	if(atomic_read(&obj->trace) == 1)
//		APS_FUN(f);
	if (unlikely(obj == NULL)) {
		APS_ERR("%s--%d tl5601c_obj is NULL!\n", __func__, __LINE__);
		return IRQ_HANDLED;
	}

	if (atomic_read(&obj->trace) == 1)
		APS_LOG("%s--%d\n", __func__, __LINE__);


	disable_irq_nosync(obj->irq);
	tl5601c_eint_func();

	return IRQ_HANDLED;
}

#endif

/******************************************************************************
 * Function Configuration
 ******************************************************************************/
static int tl5601c_open(struct inode *inode, struct file *file)
{
	if (tl5601c_i2c_client == NULL) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	file->private_data = tl5601c_i2c_client;

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tl5601c_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long tl5601c_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct tl5601c_obj *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user *) arg;
	int dat = 0;
	uint32_t enable = 0;
	int enable_als, enable_ps;

	if (NULL == obj) {
		err = -EINVAL;
		goto err_out;
	}
	if (atomic_read(&(obj->trace)) <= 1)
		APS_FUN(f);

	switch (cmd) 
	{
	case ALSPS_SET_PS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) 
		{
			err = -EFAULT;
			goto err_out;
		}
		if (enable != 0) 
		{
			enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
			if(!enable_ps)
			{
				err = tl5601c_enable_ps(obj->client, 1);
				if (err < 0) 
				{
					APS_ERR("enable ps fail: %d\n", err);
					goto err_out;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
			}
		} 
		else
		{
			enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
			if(enable_ps)
			{
				err = tl5601c_enable_ps(obj->client, 0);
				if (err < 0) 
				{
					APS_ERR("disable ps fail: %d\n", err);
					goto err_out;
				}
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
		}
		break;

	case ALSPS_GET_PS_MODE:
		enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_DATA:
//		err = tl5601c_read_ps(obj->client, &obj->ps);
		obj->ps = tl5601c_read_ps(obj);
		if (obj->ps < 0)
			goto err_out;

		dat = tl5601c_get_ps_value(obj, obj->ps);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_PS_RAW_DATA:
		obj->ps = tl5601c_read_ps(obj);
		if (obj->ps < 0)
			goto err_out;

		dat = obj->ps;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_MODE:
		enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_SET_ALS_MODE:
		if(copy_from_user(&enable, ptr, sizeof(enable)))
		{
			err = -EFAULT;
			goto err_out;
		}
		enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
		if(enable) 
		{
			if(!enable_als)
			{
				err = tl5601c_enable_als(obj->client, 1);
				if (err < 0)
				{
					APS_ERR("enable als fail: %d\n", err);
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
		} 
		else
		{
			if(enable_als)
			{
				err = tl5601c_enable_als(obj->client, 0);
				if(err < 0) 
				{
					APS_ERR("disable als fail: %d\n", err);
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
		}
		break;

	case ALSPS_GET_ALS_DATA:
		err = tl5601c_read_als(obj->client, &obj->als);
		if (err < 0)
			goto err_out;

		dat = tl5601c_get_als_value(obj, obj->als);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_RAW_DATA:
		err = tl5601c_read_als(obj->client, &obj->als);
		if (err < 0)
			goto err_out;

		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;
	}

err_out:
	return err;
}

/*----------------------------------------------------------------------------*/
static const struct file_operations tl5601c_fops = {
	.owner = THIS_MODULE,
	.open = tl5601c_open,
	.release = tl5601c_release,
	.unlocked_ioctl = tl5601c_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tl5601c_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &tl5601c_fops,
};
/*----------------------------------------------------------------------------*/
static int tl5601c_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
// shlee 170407
static ssize_t tl5601c_show_enable(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	int addr_mode, addr_res_als, addr_res_ps;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}
	addr_mode = tl5601c_read_byte(tl5601c_obj, ADDR_MODE);
	APS_ERR("ADDR_MODE = %d\n", addr_mode);
	addr_res_als = tl5601c_read_byte(tl5601c_obj, ADDR_RES_ALS);
	APS_ERR("ADDR_RES_ALS = %d\n", addr_res_als);
	addr_res_ps = tl5601c_read_byte(tl5601c_obj, ADDR_RES_PS);
	APS_ERR("ADDR_RES_PS = %d\n", addr_res_ps);

	res = scnprintf(buf, PAGE_SIZE, 
			"mode = %d, res_als = %d, res_ps = %d\n",addr_mode,addr_res_als, addr_res_ps);
	return res;
}

static ssize_t tl5601c_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	res = scnprintf(buf, PAGE_SIZE,
		"(i2c_retry:%d als_debounce:%d ps_mask:%d ps_threshold_high:%d ps_threshold_low:%d ps_debounce:%d)\n",
		atomic_read(&tl5601c_obj->i2c_retry), atomic_read(&tl5601c_obj->als_debounce),
		atomic_read(&tl5601c_obj->ps_mask), atomic_read(&tl5601c_obj->ps_threshold_high),
		atomic_read(&tl5601c_obj->ps_threshold_low), atomic_read(&tl5601c_obj->ps_debounce));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t tl5601c_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres_h, thres_l;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	if (5 == sscanf(buf, "%d %d %d %d %d %d", &retry, &als_deb, &mask, &thres_h, &thres_l, &ps_deb)) {
		atomic_set(&tl5601c_obj->i2c_retry, retry);
		atomic_set(&tl5601c_obj->als_debounce, als_deb);
		atomic_set(&tl5601c_obj->ps_mask, mask);
		atomic_set(&tl5601c_obj->ps_threshold_high, thres_h);
		atomic_set(&tl5601c_obj->ps_threshold_low, thres_l);
		atomic_set(&tl5601c_obj->ps_debounce, ps_deb);
	} else
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t tl5601c_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&tl5601c_obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t tl5601c_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&tl5601c_obj->trace, trace);
	else
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t tl5601c_show_als(struct device_driver *ddri, char *buf)
{
	int res;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	res = tl5601c_read_als(tl5601c_obj->client, &tl5601c_obj->als);
	if (res < 0)
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);

	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", tl5601c_obj->als);
}
/*----------------------------------------------------------------------------*/
static ssize_t tl5601c_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

//	res = tl5601c_read_ps(tl5601c_obj->client, &tl5601c_obj->ps);
	res = tl5601c_read_ps(tl5601c_obj);
	tl5601c_obj->ps = res;
	if (res < 0)
		return scnprintf(buf, PAGE_SIZE, "ERROR: %zd\n", res);

	return scnprintf(buf, PAGE_SIZE, "0x%04x\n", tl5601c_obj->ps);
}

static ssize_t proximity_ps_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int ps_enable;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	if (1 == sscanf(buf, "%d", &ps_enable)){
		printk("wind_tl5601c PS opening \n");
		tl5601c_proximity_disable(tl5601c_obj);
	}else if (0 == sscanf(buf,  "%d", &ps_enable)){
		printk("wind_tl5601c PS close \n");
		tl5601c_proximity_enable(tl5601c_obj);
	}else
		APS_ERR("wind_tl5601c invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t light_debug_als_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}
	
	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&tl5601c_obj->debug_als));
	return res;
}

static ssize_t light_debug_als_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int debug_als;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	if (1 == sscanf(buf, "0x%x", &debug_als))
		atomic_set(&tl5601c_obj->debug_als, debug_als);
	else
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

static ssize_t proximity_debug_ps_show(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}
	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&tl5601c_obj->debug_ps));
	return res;
}

static ssize_t proximity_debug_ps_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int debug_ps;
	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	if (1 == sscanf(buf, "0x%x", &debug_ps))
		atomic_set(&tl5601c_obj->debug_ps, debug_ps);
	else
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t tl5601c_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	if (tl5601c_obj->hw) {
		len += scnprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n",
			tl5601c_obj->hw->i2c_num, tl5601c_obj->hw->power_id, tl5601c_obj->hw->power_vol);
	} else
		len += scnprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");


	len += scnprintf(buf+len, PAGE_SIZE-len, "REGS: %02x %02x %02x %02x %02lx %02lx\n",
			atomic_read(&tl5601c_obj->als_cmd_val), atomic_read(&tl5601c_obj->ps_cmd_val),
			atomic_read(&tl5601c_obj->ps_threshold_high), atomic_read(&tl5601c_obj->ps_threshold_low),
			tl5601c_obj->enable, tl5601c_obj->pending_intr);

	len += scnprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n",
			atomic_read(&tl5601c_obj->als_suspend), atomic_read(&tl5601c_obj->ps_suspend));

	return len;
}
/*-------------------------------------*/
static ssize_t tl5601c_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	for (idx = 0; idx < tl5601c_obj->als_level_num; idx++)
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", tl5601c_obj->hw->als_level[idx]);

	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/

static ssize_t tl5601c_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	for (idx = 0; idx < tl5601c_obj->als_value_num; idx++)
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", tl5601c_obj->hw->als_value[idx]);

	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*---------------------------------------------------------------------------------------*/
static ssize_t tl5601c_enable_interrupt_show(struct device_driver *ddri, char *buf)
{
	if (NULL == tl5601c_obj) 
	{
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}
	return scnprintf(buf, PAGE_SIZE, "obj->ps_enable_interrupt = %d\n", tl5601c_obj->ps_enable_interrupt);
}
/*----------------------------------------------------------------------------*/
static ssize_t tl5601c_enable_interrupt_store(struct device_driver *ddri, const char *buf, size_t count)
{
	int ps_enable_interrupt;

	if (NULL == tl5601c_obj)
	{
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	if (1 == sscanf(buf, "%d", &ps_enable_interrupt))
		tl5601c_obj->ps_enable_interrupt = ps_enable_interrupt;
	else if (0 == sscanf(buf, "%d", &ps_enable_interrupt))
		tl5601c_obj->ps_enable_interrupt = ps_enable_interrupt;
	else
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}
/*----alsps_factory_calibration debug----*/
/** lihaiyan@wind-mobi.com 20170327 begin **/
static ssize_t alsps_show_calibrate_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!!\n");
		return -EINVAL;
	}

	res = scnprintf(buf, PAGE_SIZE,
		"(ps_low_threshold:%d  ps_high_threshold:%d  als_calibration_factor:%d)\n",
			tl5601c_obj->ps_low_threshold, tl5601c_obj->ps_high_threshold , get_als_calibration_factor() 
		);
	return res;
}	

void write_ps_value(int ps_low_threshold, int ps_high_threshold)
{
	tl5601c_obj->ps_low_threshold = ps_low_threshold; 
    tl5601c_obj->ps_high_threshold = ps_high_threshold;
}

void start_als_value(void)
{
	tl5601c_read_als(tl5601c_obj->client, &tl5601c_obj->als);
	tl5601c_get_als_value(tl5601c_obj, tl5601c_obj->als);
}
/** lihaiyan@wind-mobi.com 20170327 end **/

/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,	S_IRUGO, tl5601c_show_als, NULL);
static DRIVER_ATTR(ps,	S_IRUGO | S_IWUSR | S_IWGRP, tl5601c_show_ps, proximity_ps_store);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, tl5601c_show_config,	tl5601c_store_config);
static DRIVER_ATTR(alslv,	S_IRUGO, tl5601c_show_alslv, NULL);
static DRIVER_ATTR(alsval, S_IRUGO, tl5601c_show_alsval, NULL);
static DRIVER_ATTR(trace,	S_IWUSR | S_IRUGO, tl5601c_show_trace,		tl5601c_store_trace);
static DRIVER_ATTR(status, S_IRUGO, tl5601c_show_status, NULL);
static DRIVER_ATTR(enable_interrupt, S_IWUSR | S_IRUGO, tl5601c_enable_interrupt_show, tl5601c_enable_interrupt_store);
static DRIVER_ATTR(debug_als, S_IRUGO | S_IWUSR | S_IWGRP,
			light_debug_als_show, light_debug_als_store);
static DRIVER_ATTR(debug_ps, S_IRUGO | S_IWUSR | S_IWGRP,
			proximity_debug_ps_show, proximity_debug_ps_store);
			
static DRIVER_ATTR(test, S_IRUGO, tl5601c_show_enable, NULL);	// shlee 170407
static DRIVER_ATTR(alsps_fac_cal, S_IWUSR | S_IRUGO, alsps_show_calibrate_value, NULL);   //lihaiyan@wind-mobi.com 20170327

/*----------------------------------------------------------------------------*/
static struct driver_attribute *tl5601c_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_trace,		/*trace log*/
	&driver_attr_config,
	&driver_attr_alslv,
	&driver_attr_alsval,
	&driver_attr_status,
	&driver_attr_enable_interrupt,
	&driver_attr_debug_als,
	&driver_attr_debug_ps,
	&driver_attr_test,	// shlee 170407

	&driver_attr_alsps_fac_cal    //lihaiyan@wind-mobi.com 20170327

};

/*----------------------------------------------------------------------------*/
static int tl5601c_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
//	int num = (int)ARRAY_SIZE(tl5601c_attr_list);/* (sizeof(tl5601c_attr_list)/sizeof(tl5601c_attr_list[0])); */
	int num = (int)(sizeof(tl5601c_attr_list)/sizeof(tl5601c_attr_list[0])); 
//	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	if (driver == NULL)
		return -EINVAL;

#ifndef PLATFORM_VERSION_AND_5
	for (idx = 0; idx < num; idx++) 
	{
		err = driver_create_file(driver, tl5601c_attr_list[idx]);
		if (err < 0) {
			APS_ERR("driver_create_file (%s) = %d\n", tl5601c_attr_list[idx]->attr.name, err);
			break;
		}
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
static int tl5601c_delete_attr(struct device_driver *driver)
{
	int idx , err = 0;
	int num = (int)ARRAY_SIZE(tl5601c_attr_list);/* (sizeof(tl5601c_attr_list)/sizeof(tl5601c_attr_list[0])); */

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, tl5601c_attr_list[idx]);

	return err;
}
/*----------------------------------------------------------------------------*/

static int tl5601c_als_open_report_data(int open)
{
	return 0;
}
static int tl5601c_als_enable_nodata(int en)
{
	int err = 0;
	struct tl5601c_obj *obj = tl5601c_obj;
	int enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
	if(atomic_read(&obj->trace) == 1)
		APS_FUN(f);

	if (NULL == obj) 
	{
		APS_ERR("tl5601c_obj is null!\n");
		return -EINVAL;
	}

	if (en) 
	{
		if(!enable_als)
		{
			err = tl5601c_enable_als(obj->client, 1);
			if (err < 0) {
				APS_ERR("enable als fail: %d\n", err);
				return -EINVAL;
			}
			set_bit(CMC_BIT_ALS, &obj->enable);
		}
	} 
	else
	{
		if(enable_als)
		{
			err = tl5601c_enable_als(obj->client, 0);
			if (err < 0) {
				APS_ERR("disable als fail: %d\n", err);
				return -EINVAL;
			}
			clear_bit(CMC_BIT_ALS, &obj->enable);
		}
	}
	return 0;
}
static int tl5601c_als_set_delay(u64 ns)
{
	return 0;
}
static int tl5601c_als_get_data(int *value, int *status)
{
	int err = 0;
	int als_tmp = 0;
	struct tl5601c_obj *obj = tl5601c_obj;

	if (NULL == obj) {
		APS_ERR("tl5601c_obj is null!\n");
		return -EINVAL;
	}

	err = tl5601c_read_als(obj->client, &obj->als);
	if (err < 0) {
		APS_ERR("tl5601c_read_als fail: %d\n", err);
		return -EINVAL;
	} else{
		als_tmp = tl5601c_get_als_value(obj, obj->als);
		if (als_tmp < 0) {
			APS_ERR("tl5601c_get_als_value fail: %d\n", err);
			return -EINVAL;
		}
		*value = als_tmp;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return 0;
}
static int tl5601c_ps_open_report_data(int open)
{
	return 0;
}
static int tl5601c_ps_enable_nodata(int en)
{
	int err = 0;
	struct tl5601c_obj *obj = tl5601c_obj;
	int enable_ps = test_bit(CMC_BIT_PS, &obj->enable);

	if (NULL == obj) 
	{
		APS_ERR("tl5601c_obj is null!\n");
		return -EINVAL;
	}

	if (en) 
	{
		if(!enable_ps)
		{
			err = tl5601c_enable_ps(obj->client, 1);
			if (err < 0) {
				APS_ERR("enable ps fail: %d\n", err);
				return -EINVAL;
			}
			set_bit(CMC_BIT_PS, &obj->enable);
		}
	} 
	else
	{
		if(enable_ps)
		{
			err = tl5601c_enable_ps(obj->client, 0);
			if (err < 0) {
				APS_ERR("disable ps fail: %d\n", err);
				return -EINVAL;
			}
			clear_bit(CMC_BIT_PS, &obj->enable);
		}
	}

	return 0;
}
static int tl5601c_ps_set_delay(u64 ns)
{
	return 0;
}

static int tl5601c_ps_get_data(int *value, int *status)
{
	int ps_tmp = 0;
	struct tl5601c_obj *obj = NULL;

	if (NULL == tl5601c_obj) {
		APS_ERR("tl5601c_obj is null!\n");
		return -EINVAL;
	}
	obj = tl5601c_obj;

//	err = tl5601c_read_ps(obj->client, &obj->ps);
	obj->ps = tl5601c_read_ps(obj);

	if (obj->ps < 0) {
		APS_ERR("tl5601c_read_ps failed!\n");
		return -EINVAL;
	} else{
		ps_tmp = tl5601c_get_ps_value(obj, obj->ps);
		if (ps_tmp < 0) {
			APS_ERR("tl5601c_get_ps_value failed!\n");
			return -EINVAL;
		}
		*value = ps_tmp;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int tl5601c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tl5601c_obj *obj = NULL;
	int ret = 0;

	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};

	int err = 0;
	
#ifndef PLATFORM_VERSION_AND_5
	client->addr = hw->i2c_addr[0]; //sanford.lin
	printk("%s: driver version: %s\n", __FUNCTION__, DRV_VER);
#endif

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (obj == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	/* memset(obj, 0, sizeof(*obj)); */
	tl5601c_obj = obj;

	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

     
#ifdef PLATFORM_VERSION_AND_5
	obj->hw = get_cust_alsps_hw();
//	tl5601c_get_addr(obj->hw, &obj->addr);
#else
	obj->hw = hw;
#endif
	obj->client = client;
	client->addr = 0x39;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->als_debounce, 0);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 0);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0); 
	atomic_set(&obj->ps_cmd_val, 0); 
	atomic_set(&obj->ps_threshold_high, obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_threshold_low, obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	obj->als_modulus = (400*100*40)/(1*1500);

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);

#if defined(CONFIG_OF)
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
#endif

	tl5601c_i2c_client = client;

	mutex_init(&obj->i2c_mutex);
	mutex_init(&obj->nvm_lock);
	mutex_init(&obj->als_mutex);
	mutex_init(&obj->lock);
	mutex_init(&obj->prox_poll_mutex);

	err = misc_register(&tl5601c_device);
	if (err < 0) {
		APS_ERR("tl5601c_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	obj->wq_prox_poll = create_singlethread_workqueue("tl5601c_wq_prox_poll");
	if(!obj->wq_prox_poll)
	{
		ret = -ENOMEM;
		APS_ERR("%s: count not create workqueue\n", __func__);
	}
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);

	err = tl5601c_create_attr(&(tl5601c_init_info.platform_diver_addr->driver));
	if (err < 0) {
		APS_ERR("tl5601c_create_attr fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_ctl.open_report_data = tl5601c_ps_open_report_data;
	ps_ctl.enable_nodata = tl5601c_ps_enable_nodata;
	ps_ctl.set_delay = tl5601c_ps_set_delay;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
	//ps_ctl.is_support_barch = false;
#endif

	if (0 != obj->hw->polling_mode_ps) 
	{/*polling mode*/
		wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
		ps_ctl.is_polling_mode = true;
		ps_ctl.is_report_input_direct = false;
	} 
	else
	{							/*interrupt mode*/
		ps_ctl.is_polling_mode = false;
		ps_ctl.is_report_input_direct = true;

		hrtimer_init(&obj->prox_poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//		obj->prox_poll_delay = ns_to_ktime(40 * NSEC_PER_MSEC);
		obj->prox_poll_delay = ns_to_ktime(15 * NSEC_PER_MSEC);
		obj->prox_poll_timer.function = tl5601c_prox_poll_timer_func;

		hrtimer_init(&obj->prox_init_crt_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		obj->prox_init_crt_delay = ns_to_ktime(15 * NSEC_PER_MSEC);
		obj->prox_init_crt_timer.function = tl5601c_prox_init_crt_timer_func;

		INIT_WORK(&obj->eint_work, tl5601c_eint_work);
		INIT_WORK(&obj->work_prox_poll, tl5601c_work_func_prox_poll);
		INIT_WORK(&obj->work_prox_init_crt, tl5601c_work_func_prox_init_crt);
		
	}
	err = tl5601c_init_client(client);
	if (err < 0) 
	{
		APS_ERR("tl5601c_init_client() failed!\n");
		goto exit_kfree;
	}
	APS_LOG("tl5601c_init_client() OK!\n");
	
	err = ps_register_control_path(&ps_ctl);
	if (err < 0) 
	{
		APS_ERR("ps_register_control_path fail = %d\n", err);
		goto exit_register_path_failed;
	}

	ps_data.get_data = tl5601c_ps_get_data;
//	ps_data.vender_div = 1;
	ps_data.vender_div = 100;

	err = ps_register_data_path(&ps_data);
	if (err < 0) {
		APS_ERR("ps_register_data_path fail = %d\n", err);
		goto exit_register_path_failed;
	}

	als_ctl.open_report_data = tl5601c_als_open_report_data;
	als_ctl.enable_nodata = tl5601c_als_enable_nodata;
	als_ctl.set_delay = tl5601c_als_set_delay;
//	als_ctl.is_polling_mode = true;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#else
	als_ctl.is_support_batch = false;
#endif

/** lihaiyan@wind-mobi.com 20170327 begin **/
	 write_ps_value_func = write_ps_value;
	 start_als_value_func = start_als_value;
     psensor_proc_init();
     lightsensor_proc_init();
/** lihaiyan@wind-mobi.com 20170327 end **/


	err = als_register_control_path(&als_ctl);
	if (err < 0) {
		APS_ERR("als_register_control_path fail = %d\n", err);
		goto exit_register_path_failed;
	}

	als_data.get_data = tl5601c_als_get_data;
//	als_data.vender_div = 1;
	als_data.vender_div = 100;

	err = als_register_data_path(&als_data);
	if (err < 0) {
		APS_ERR("als_register_data_path fail = %d\n", err);
		goto exit_register_path_failed;
	}

	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
	}

	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
	}

	tl5601c_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_register_path_failed:
	tl5601c_delete_attr(&(tl5601c_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
	misc_deregister(&tl5601c_device);
exit_misc_device_register_failed:
//exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	tl5601c_i2c_client = NULL;
	tl5601c_init_flag = -1;

	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int tl5601c_i2c_remove(struct i2c_client *client)
{
	struct tl5601c_obj *obj = i2c_get_clientdata(client);
	int err;

	err = tl5601c_delete_attr(&(tl5601c_init_info.platform_diver_addr->driver));
	if (err < 0)
		APS_ERR("tl5601c_delete_attr fail: %d\n", err);

	err = misc_deregister(&tl5601c_device);
	if (err < 0)
		APS_ERR("misc_deregister fail: %d\n", err);

/** lihaiyan@wind-mobi.com 20170327 begin **/
	psensor_proc_remove();
    lightsensor_proc_deinit();
/** lihaiyan@wind-mobi.com 20170327 end **/


	destroy_workqueue(obj->wq_prox_poll);
	tl5601c_power_off(obj);

	tl5601c_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/

static int tl5601c_local_init(void)
{
#ifdef PLATFORM_VERSION_AND_5
	struct alsps_hw *hw = get_cust_alsps_hw();
#endif
	printk("%s ,%d\n", __func__, __LINE__);
	tl5601c_power(hw, 1);
	if (i2c_add_driver(&tl5601c_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -EINVAL;
	}
	printk("%s ,%d\n", __func__, __LINE__);
	if (-1 == tl5601c_init_flag)
		return -EINVAL;

	return 0;
}

static int tl5601c_remove(void)
{
#ifdef PLATFORM_VERSION_AND_5
	struct alsps_hw *hw = get_cust_alsps_hw();
#endif
	APS_FUN();
	tl5601c_power(hw, 0);
	i2c_del_driver(&tl5601c_i2c_driver);

	return 0;
}


/*----------------------------------------------------------------------------*/

static int __init tl5601c_init(void)
{
#ifndef PLATFORM_VERSION_AND_5 // android 6.0
	const char *name = "mediatek,tl5601c";
	APS_FUN();
	printk("%s ,%d\n", __func__, __LINE__);
	hw = get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("get dts info fail\n");
	alsps_driver_add(&tl5601c_init_info);
#else // android 5.0
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();
	i2c_register_board_info(hw->i2c_num, &i2c_tl5601c, 1);

	alsps_driver_add(&tl5601c_init_info);
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit tl5601c_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(tl5601c_init);
module_exit(tl5601c_exit);
/*----------------------------------------------------------------------------*/
MODULE_VERSION(DRV_VER);
MODULE_DESCRIPTION(DRV_DESC " " DRV_VER);
MODULE_AUTHOR("Seonhaeng Lee");
MODULE_LICENSE("GPL");
