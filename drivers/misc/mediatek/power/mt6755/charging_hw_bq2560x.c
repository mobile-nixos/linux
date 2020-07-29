/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
 * Revision:   1.0
 * Modtime:   11 Aug 2005 10:28:16
 * Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_common.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include "bq2560x.h"
#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mach/diso.h>
#include "cust_diso.h"
#include <linux/of.h>
#include <linux/of_irq.h>
#ifdef MTK_DISCRETE_SWITCH
#include <mach/eint.h>
#include <cust_eint.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#endif
#endif

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

/* ============================================================ // */
/* Global variable */
/* ============================================================ // */
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

#if defined(GPIO_PWR_AVAIL_WLC)
kal_uint32 wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC;
#else
kal_uint32 wireless_charger_gpio_number = 0;
#endif

#endif

static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

static kal_bool charging_type_det_done = KAL_TRUE;
/* modify begin by chengronghe@wind-mobi.com, 2017-06-30 */
const unsigned int BQ2560x_VBAT_CV_VTH[] = {
	3856000, 3888000, 3920000, 3952000, 
	3984000, 4016000, 4048000, 4080000, 
	4112000, 4144000, 4176000, 4208000, 
	4240000, 4272000, 4304000, 4336000, 
	4368000, 4400000, 4432000, 4464000,
	4496000, 4528000, 4560000, 4592000,
	4624000,
};
//yangzhigang@wind-mobi.com 20170620 begin modify for bq25601 reg
const unsigned int BQ2560x_CS_VTH[] = {
	0, 6000, 12000, 18000, 
	24000, 30000, 36000, 42000, 
	48000, 54000, 60000, 66000,
	72000, 78000, 84000, 90000,
	96000, 102000, 108000, 114000,
	120000, 126000, 132000, 138000,
	144000, 150000, 156000, 162000,
	168000, 174000, 180000, 186000,
	192000, 198000, 204000, 210000,
	216000, 222000, 228000, 234000,
	240000, 246000, 252000, 258000,
	264000, 270000, 276000, 282000,
	288000, 294000, 300000,
};

const unsigned int BQ2560x_INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_200_00_MA, CHARGE_CURRENT_300_00_MA,
	CHARGE_CURRENT_400_00_MA, CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_600_00_MA,
	CHARGE_CURRENT_700_00_MA, CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1100_00_MA, CHARGE_CURRENT_1200_00_MA,
	CHARGE_CURRENT_1300_00_MA, CHARGE_CURRENT_1400_00_MA, CHARGE_CURRENT_1500_00_MA,
	CHARGE_CURRENT_1600_00_MA, CHARGE_CURRENT_1700_00_MA, CHARGE_CURRENT_1800_00_MA,
	CHARGE_CURRENT_1900_00_MA, CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_2100_00_MA,
	CHARGE_CURRENT_2200_00_MA, CHARGE_CURRENT_2300_00_MA, CHARGE_CURRENT_2400_00_MA,
	CHARGE_CURRENT_2500_00_MA, CHARGE_CURRENT_2600_00_MA, CHARGE_CURRENT_2700_00_MA,
	CHARGE_CURRENT_2800_00_MA, CHARGE_CURRENT_2900_00_MA, CHARGE_CURRENT_3000_00_MA,
	CHARGE_CURRENT_MAX
};
//yangzhigang@wind-mobi.com 20170620 end

const unsigned int BQ2560x_VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#ifndef CUST_GPIO_VIN_SEL
#define CUST_GPIO_VIN_SEL 18
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#define SW_POLLING_PERIOD 100	/* 100 ms */
#define MSEC_TO_NSEC(x)		(x * 1000000UL)

static DEFINE_MUTEX(diso_polling_mutex);
static DECLARE_WAIT_QUEUE_HEAD(diso_polling_thread_wq);
static struct hrtimer diso_kthread_timer;
static kal_bool diso_thread_timeout = KAL_FALSE;
static struct delayed_work diso_polling_work;
static void diso_polling_handler(struct work_struct *work);
static DISO_Polling_Data DISO_Polling;
#else
DISO_IRQ_Data DISO_IRQ;
#endif
int g_diso_state = 0;
int vin_sel_gpio_number = (CUST_GPIO_VIN_SEL | 0x80000000);

static char *DISO_state_s[8] = {
	"IDLE",
	"OTG_ONLY",
	"USB_ONLY",
	"USB_WITH_OTG",
	"DC_ONLY",
	"DC_WITH_OTG",
	"DC_WITH_USB",
	"DC_USB_OTG",
};
#endif

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */


/* ============================================================ // */
/* extern variable */
/* ============================================================ // */

/* ============================================================ // */
/* extern function */
/* ============================================================ // */
/*extern unsigned int upmu_get_reg_value(unsigned int reg);
extern bool mt_usb_is_device(void);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern int hw_charging_get_charger_type(void);
extern void mt_power_off(void);
extern unsigned int mt6311_get_chip_id(void);
extern int is_mt6311_exist(void);
extern int is_mt6311_sw_ready(void);
*/
static unsigned int g_input_current;
static unsigned int charging_error;
static unsigned int charging_get_error_state(void);
static unsigned int charging_set_error_state(void *data);
/* ============================================================ // */
unsigned int charging_value_to_parameter_x(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	if (val < array_size) {
		return parameter[val];
	} else {
		pr_debug("Can't find the parameter \r\n");
		return parameter[0];
	}
}

unsigned int charging_parameter_to_value_x(const unsigned int *parameter, const unsigned int array_size,
				       const unsigned int val)
{
	unsigned int i;

	pr_debug("array_size = %d \r\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;

	}

	pr_debug("NO register value match. val=%d\r\n", val);
	/* TODO: ASSERT(0);      // not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					 unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {/* max value in the last element */

			if (pList[i] <= level)
				return pList[i];

		}

		pr_debug("Can't find closest level, small value first \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {/* max value in the first element */

			if (pList[i] <= level)
				return pList[i];

		}

		pr_debug("Can't find closest level, large value first \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

static unsigned int is_chr_det(void)
{
	unsigned int val = 0;
	val = pmic_get_register_value(PMIC_RGS_CHRDET);//yangzhigang@wind-mobi.com 20170508 modify begin
	battery_log(BAT_LOG_CRTI, "[is_chr_det] %d\n", val);

	return val;
}

static unsigned int charging_hw_init(void *data)
{
	unsigned int status = STATUS_OK;

	bq2560x_set_en_hiz(0x0);
	bq2560x_set_vindpm(0x7);	/* VIN DPM check 4.6V */
	bq2560x_set_reg_rst(0x0);
	bq2560x_set_wdt_rst(0x1);	/* Kick watchdog */
	bq2560x_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	bq2560x_set_iprechg(0x8);	/* Precharge current 540mA */
	bq2560x_set_iterm(0x1);	/* Termination current 120mA *///zhounengwen@wind-mobi.com 20170823 modify
	bq2560x_set_low_temp_jieta(0x0);  //liulinsheng@wind-mobi.com 20171001
	bq2560x_set_high_temp_jieta(0x1);  //liulinsheng@wind-mobi.com 20171001
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	bq2560x_set_vreg(0x11);	/* VREG 4.4V *///zhounengwen@wind-mobi.com 20170724 modify
#else
	bq2560x_set_vreg(0x0B);	/* VREG 4.208V */
#endif

	bq2560x_set_batlowv(0x1);	/* BATLOWV 3.0V */
	bq2560x_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	bq2560x_set_en_term(0x1);	/* Enable termination */
//	bq2560x_set_stat_ctrl(0x0);	/* Enable STAT pin function */
	bq2560x_set_watchdog(0x1);	/* WDT 40s */
	bq2560x_set_en_timer(0x0);	/* Enable charge timer */
	//bq2560x_set_chg_timer(0x02);	/*set charge time 12h*/
	bq2560x_set_int_mask(0x0);	/* Disable fault interrupt */

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	if (wireless_charger_gpio_number != 0) {
		mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
		mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
	}
#endif

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
	mt_set_gpio_mode(vin_sel_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(vin_sel_gpio_number, 0);	/* 0: input, 1: output */
#endif
	return status;
}

static unsigned int charging_dump_register(void *data)
{
	unsigned int status = STATUS_OK;

	printk("charging_dump_register\r\n");
	bq2560x_dump_register();

	return status;
}

static unsigned int charging_enable(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int *) (data);
	kal_bool static hiz_flag = 0;

	if (KAL_TRUE == enable) {
		bq2560x_set_en_hiz(0x0);
		bq2560x_set_chg_config(0x1);	/* charger enable */
		hiz_flag = 0;
	} 
	else if(2 == enable)//wangchao add for runin test 2016-4-27
	{
	    bq2560x_set_chg_config(0x0);
	    bq2560x_set_en_hiz(0x1);
        printk("charging_enable = 2,close hiz & disabale charging for runin test\n");
	    hiz_flag = 1;
	} else {
#if defined(CONFIG_USB_MTK_HDRC_HCD)
if (mt_usb_is_device())
#endif
		{
			bq2560x_set_chg_config(0x0);
			if(hiz_flag == 1){
				bq2560x_set_en_hiz(0x1);
			}
			if (charging_get_error_state()) {
				pr_debug("[charging_enable] bq2560x_set_hz_mode(0x1)\n");
				bq2560x_set_en_hiz(0x1);	/* disable power path */
			}
		}
#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		bq2560x_set_chg_config(0x0);
		bq2560x_set_en_hiz(0x1);	/* disable power path */
#endif
	}

	return status;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int array_size;
	unsigned int set_cv_voltage;
	unsigned short register_value;
	unsigned int cv_value = *(unsigned int *) (data);
	static short pre_register_value = -1;
   printk("wind-charging_set_cv_voltage:bq25601");
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	/* highest of voltage will be 4.3V, because powerpath limitation */
	//liulinsheng@wind-mobi.com 20171001 begin
	//if (cv_value >= BATTERY_VOLT_04_300000_V)
	//	cv_value = 4400000;
    //liulinsheng@wind-mobi.com 20171001 end
#endif

	/* use nearest value */
	if (BATTERY_VOLT_04_200000_V == cv_value)
		cv_value = 4208000;
/* modify begin by chengronghe@wind-mobi.com, 2017-06-30 */
	array_size = GETARRAYNUM(BQ2560x_VBAT_CV_VTH);
	battery_log(BAT_LOG_CRTI, "charging_set_cv_voltage set_cv_voltage=%d\n",
		    *(unsigned int *) data);
	set_cv_voltage = bmt_find_closest_level(BQ2560x_VBAT_CV_VTH, array_size, cv_value);
	//battery_set_cv_voltage(set_cv_voltage);
	register_value = charging_parameter_to_value_x(BQ2560x_VBAT_CV_VTH, array_size, set_cv_voltage);
/* modify end by chengronghe@wind-mobi.com, 2017-06-30 */
	battery_log(BAT_LOG_FULL, "charging_set_cv_voltage register_value=0x%x\n", register_value);
	bq2560x_set_vreg(register_value);
	if (pre_register_value != register_value)
		bq2560x_set_chg_config(1);
	pre_register_value = register_value;
	return status;
}

static unsigned int charging_get_current(void *data)
{
	unsigned int status = STATUS_OK;
	/* unsigned int array_size; */
	/* unsigned char reg_value; */
	unsigned char ret_val = 0;
	//unsigned char ret_force_20pct = 0;

	/* Get current level */
	bq2560x_read_interface(bq2560x_CON2, &ret_val, CON2_ICHG_MASK, CON2_ICHG_SHIFT);

	/* Parsing */
	ret_val = (ret_val * 60) ;

		/* Get current level */
		/* array_size = GETARRAYNUM(CS_VTH); */
		/* *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value); */
	*(unsigned int *) data = ret_val;

	return status;
}

static unsigned int charging_set_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	unsigned int current_value = *(unsigned int *) data;
/* modify begin by chengronghe@wind-mobi.com, 2017-06-30 */
	array_size = GETARRAYNUM(BQ2560x_CS_VTH);
	printk("yzg-b char charging_set_current current_value:%d\n",current_value);//yangzhigang@wind-mobi.com 20170508 modify 
	set_chr_current = bmt_find_closest_level(BQ2560x_CS_VTH, array_size, current_value);
	//yangzhigang@wind-mobi.com 20170508 modify begin
	printk("yzg-b char charging_set_current set_chr_current:%d\n",set_chr_current);
	//set_chr_current = 224000;
	//yangzhigang@wind-mobi.com 20170508 modify end
	register_value = charging_parameter_to_value_x(BQ2560x_CS_VTH, array_size, set_chr_current);
/* modify end by chengronghe@wind-mobi.com, 2017-06-30 */
	//register_value = 0x39;
	printk("yzg-b char charging_set_current register_value:0x%x\n",register_value);	
	bq2560x_set_ichg(register_value);
	//charging_dump_register(1);

	return status;
}

static unsigned int charging_set_input_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int current_value = *(unsigned int *) data;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	
	battery_log(BAT_LOG_CRTI, "charging_set_cc_input_current set_cc_current=%d\n",
		    *(unsigned int *) data);

	printk("yzg-b char charging_set_input_current current_value:%d\n",current_value);//yangzhigang@wind-mobi.com 20170508 modify 
/* modify begin by chengronghe@wind-mobi.com, 2017-06-30 */
	array_size = GETARRAYNUM(BQ2560x_INPUT_CS_VTH);
	set_chr_current = bmt_find_closest_level(BQ2560x_INPUT_CS_VTH, array_size, current_value);
	g_input_current = set_chr_current;
	printk("yzg-b char charging_set_input_current set_chr_current:%d\n",set_chr_current);//yangzhigang@wind-mobi.com 20170508 modify	
	register_value = charging_parameter_to_value_x(BQ2560x_INPUT_CS_VTH, array_size, set_chr_current);
/* modify end by chengronghe@wind-mobi.com, 2017-06-30 */
	//yangzhigang@wind-mobi.com 20170508 modify begin
	printk("yzg-b char charging_set_input_current register_value:0x%x\n",register_value);	
	//register_value = 0x1f;
	//yangzhigang@wind-mobi.com 20170508 modify end
	bq2560x_set_iinlim(register_value);

	return status;
}

static unsigned int charging_get_charging_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int ret_val;

	ret_val = bq2560x_get_chrg_stat();

	if (ret_val == 0x3)
		*(unsigned int *) data = KAL_TRUE;
	else
		*(unsigned int *) data = KAL_FALSE;

	return status;
}

static unsigned int charging_get_input_current(void *data)
{
	int ret = 0;

	*((u32 *)data) = g_input_current;

	return ret;
}
//liulinsheng@wind-mobi.com 20171016 begin
static unsigned int charging_set_suspend_charge(void *data)
{
	int status = STATUS_OK;
	bq2560x_set_watchdog(0x0);	/* disable  watch dog */
	bq2560x_set_iinlim(0x0);	/* Iinlim=100mA  */
	bq2560x_set_en_hiz(0x1);	/* close charge  */
	printk("wind-logcharging_set_suspend_chargebq2560x");
	return status;
}

static unsigned int charging_set_recover_charge(void *data)
{
	int status = STATUS_OK;
	bq2560x_set_watchdog(0x1);	/*  enable watch dog */
	bq2560x_set_en_hiz(0x0);	/* recover charge  */
	printk("wind-logcharging_set_recover_chargebq2560x");
	return status;
}
//liulinsheng@wind-mobi.com 20171016 end

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	unsigned int status = STATUS_OK;

	pr_debug("charging_reset_watch_dog_timer\r\n");

	bq2560x_set_wdt_rst(0x1);	/* Kick watchdog */

	return status;
}

static unsigned int charging_set_hv_threshold(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short register_value;
	unsigned int voltage = *(unsigned int *) (data);
/*chengronghe@wind-mobi.com add for XXX 2017-06-30  begin*/
	array_size = GETARRAYNUM(BQ2560x_VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(BQ2560x_VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value_x(BQ2560x_VCDT_HV_VTH, array_size, set_hv_voltage);
/* modify end by chengronghe@wind-mobi.com, 2017-06-30 */
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);//yangzhigang@wind-mobi.com 20170508 modify 

	return status;
}

static unsigned int charging_get_hv_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);//yangzhigang@wind-mobi.com 20170508 modify 

	return status;
}

static unsigned int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	pr_debug("[charging_get_battery_status] battery exist for bring up.\n");
#else
	unsigned int val = 0;
//yangzhigang@wind-mobi.com 20170508 modify begin
	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif
//yangzhigang@wind-mobi.com 20170508 modify end
	return status;
}

//yangzhigang@wind-mobi.com 20170508 modify begin
static unsigned int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int val = 0;

#if defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_log(BAT_LOG_CRTI, "[charging_get_charger_det_status] chr exist for fpga.\n");
#else
#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
#else
	if (((g_diso_state >> 1) & 0x3) != 0x0 || pmic_get_register_value(PMIC_RGS_CHRDET))
		val = KAL_TRUE;
	else
		val = KAL_FALSE;
#endif
#endif

	*(kal_bool *) (data) = val;
	return status;
}
//yangzhigang@wind-mobi.com 20170508 modify end

/*
kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}
*/

static unsigned int charging_get_charger_type(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	int wireless_state = 0;
	if (wireless_charger_gpio_number != 0) {
		wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
		if (wireless_state == WIRELESS_CHARGER_EXIST_STATE) {
			*(CHARGER_TYPE *) (data) = WIRELESS_CHARGER;
			pr_debug("WIRELESS_CHARGER!\n");
			return status;
		}
	} else {
		pr_debug("wireless_charger_gpio_number=%d\n", wireless_charger_gpio_number);
	}

	if (g_charger_type != CHARGER_UNKNOWN && g_charger_type != WIRELESS_CHARGER) {
		*(CHARGER_TYPE *) (data) = g_charger_type;
		pr_debug("return %d!\n", g_charger_type);
		return status;
	}
#endif

	if (is_chr_det() == 0) {
		g_charger_type = CHARGER_UNKNOWN;
		*(CHARGER_TYPE *) (data) = CHARGER_UNKNOWN;
		pr_debug("[charging_get_charger_type] return CHARGER_UNKNOWN\n");
		return status;
	}

	charging_type_det_done = KAL_FALSE;
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
	charging_type_det_done = KAL_TRUE;
	g_charger_type = *(CHARGER_TYPE *) (data);

#endif

	return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;
	/*
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = KAL_FALSE;
#else

	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	pr_debug("slp_get_wake_reason=%d\n", slp_get_wake_reason());
#endif
*/
	return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	pr_debug("charging_set_platform_reset\n");
	/* arch_reset(0,NULL); */
#endif
	return status;
}

static unsigned int charging_get_platfrom_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *) (data) = get_boot_mode();
	pr_debug("get_boot_mode=%d\n", get_boot_mode());
#endif
	return status;
}

static unsigned int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	pr_debug("charging_set_power_off\n");
	mt_power_off();
#endif

	return status;
}

static unsigned int charging_get_power_source(void *data)
{
	unsigned int status = STATUS_OK;

#if 0				/* #if defined(MTK_POWER_EXT_DETECT) */
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *) data = KAL_FALSE;
	else
		*(kal_bool *) data = KAL_TRUE;
#else
	*(kal_bool *) data = KAL_FALSE;
#endif

	return status;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
	unsigned int increase = *(unsigned int *) (data);
	unsigned int charging_status = KAL_FALSE;

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_400000_V;//BATTERY_VOLT_04_340000_V;
#else
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_200000_V;
#endif

	charging_get_charging_status(&charging_status);
	if (KAL_FALSE == charging_status) {
		charging_set_cv_voltage(&cv_voltage);	/* Set CV */
		bq2560x_set_ichg(0x0);	/* Set charging current 500ma */
		bq2560x_set_chg_config(0x1);	/* Enable Charging */
	}

	if (increase == KAL_TRUE) {
		bq2560x_set_iinlim(0x0);	/* 100mA */
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 1");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 1");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 2");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 2");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 3");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 3");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 4");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 4");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 5");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 5");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_increase() on 6");
		msleep(485);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_increase() off 6");
		msleep(50);

		pr_debug("mtk_ta_increase() end\n");

		bq2560x_set_iinlim(0x2);	/* 500mA */
		msleep(200);
	} else {
		bq2560x_set_iinlim(0x0);	/* 100mA */
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 1");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 1");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 2");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 2");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 3");
		msleep(281);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 3");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 4");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 4");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 5");
		msleep(85);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 5");
		msleep(85);

		bq2560x_set_iinlim(0x2);	/* 500mA */
		pr_debug("mtk_ta_decrease() on 6");
		msleep(485);

		bq2560x_set_iinlim(0x0);	/* 100mA */
		pr_debug("mtk_ta_decrease() off 6");
		msleep(50);

		pr_debug("mtk_ta_decrease() end\n");

		bq2560x_set_iinlim(0x2);	/* 500mA */
	}

	return STATUS_OK;
}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
void set_diso_otg(bool enable)
{
	g_diso_otg = enable;
}

void set_vusb_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vusb_polling_measure.notify_irq_en = enable;
	DISO_Polling.vusb_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)),
		      HRTIMER_MODE_REL);
#else
	unsigned short threshold = 0;
	if (enable) {
		if (flag == 0)
			threshold = DISO_IRQ.vusb_measure_channel.falling_threshold;
		else
			threshold = DISO_IRQ.vusb_measure_channel.rising_threshold;

		threshold =
		    (threshold * R_DISO_VBUS_PULL_DOWN) / (R_DISO_VBUS_PULL_DOWN +
							   R_DISO_VBUS_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vusb_measure_channel.number, threshold,
						  DISO_IRQ.vusb_measure_channel.period,
						  DISO_IRQ.vusb_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vusb_measure_channel.number);
	}
#endif
	pr_debug(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

void set_vdc_auxadc_irq(bool enable, bool flag)
{
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_cancel(&diso_kthread_timer);

	DISO_Polling.reset_polling = KAL_TRUE;
	DISO_Polling.vdc_polling_measure.notify_irq_en = enable;
	DISO_Polling.vdc_polling_measure.notify_irq = flag;

	hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)),
		      HRTIMER_MODE_REL);
#else
	unsigned short threshold = 0;
	if (enable) {
		if (flag == 0)
			threshold = DISO_IRQ.vdc_measure_channel.falling_threshold;
		else
			threshold = DISO_IRQ.vdc_measure_channel.rising_threshold;

		threshold =
		    (threshold * R_DISO_DC_PULL_DOWN) / (R_DISO_DC_PULL_DOWN + R_DISO_DC_PULL_UP);
		mt_auxadc_enableBackgroundDection(DISO_IRQ.vdc_measure_channel.number, threshold,
						  DISO_IRQ.vdc_measure_channel.period,
						  DISO_IRQ.vdc_measure_channel.debounce, flag);
	} else {
		mt_auxadc_disableBackgroundDection(DISO_IRQ.vdc_measure_channel.number);
	}
#endif
	pr_debug(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
static void diso_polling_handler(struct work_struct *work)
{
	int trigger_channel = -1;
	int trigger_flag = -1;

	if (DISO_Polling.vdc_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VDC_CHANNEL;
	else if (DISO_Polling.vusb_polling_measure.notify_irq_en)
		trigger_channel = AP_AUXADC_DISO_VUSB_CHANNEL;

	pr_debug("[DISO]auxadc handler triggered\n");
	switch (trigger_channel) {
	case AP_AUXADC_DISO_VDC_CHANNEL:
		trigger_flag = DISO_Polling.vdc_polling_measure.notify_irq;
		pr_debug("[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel,
			 trigger_flag);
#ifdef MTK_DISCRETE_SWITCH	/*for DSC DC plugin handle */
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[2]);
		}
#else				/* for load switch OTG leakage handle */
		set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[5]);
		} else if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_otg_state = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[1]);
		} else
			pr_debug("[%s] wrong trigger flag!\n", __func__);
#endif
		break;
	case AP_AUXADC_DISO_VUSB_CHANNEL:
		trigger_flag = DISO_Polling.vusb_polling_measure.notify_irq;
		pr_debug("[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel,
			 trigger_flag);
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[4]);
		} else if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[6]);
		} else
			pr_debug("[%s] wrong trigger flag!\n", __func__);
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		break;
	default:
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
		return;		/* in error or unexecpt state just return */
	}

	g_diso_state = *(int *)&DISO_data.diso_state;
	pr_debug("[DISO]g_diso_state: 0x%x\n", g_diso_state);
	DISO_data.irq_callback_func(0, NULL);

	return;
}
#else
static irqreturn_t diso_auxadc_irq_handler(int irq, void *dev_id)
{
	int trigger_channel = -1;
	int trigger_flag = -1;
	trigger_channel = mt_auxadc_getCurrentChannel();
	pr_debug("[DISO]auxadc handler triggered\n");
	switch (trigger_channel) {
	case AP_AUXADC_DISO_VDC_CHANNEL:
		trigger_flag = mt_auxadc_getCurrentTrigger();
		pr_debug("[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel,
			 trigger_flag);
#ifdef MTK_DISCRETE_SWITCH	/*for DSC DC plugin handle */
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[2]);
		}
#else				/* for load switch OTG leakage handle */
		set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		if (trigger_flag == DISO_IRQ_RISING) {
			DISO_data.diso_state.pre_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_otg_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[5]);
		} else {
			DISO_data.diso_state.pre_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_otg_state = DISO_ONLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[1]);
		}
#endif
		break;
	case AP_AUXADC_DISO_VUSB_CHANNEL:
		trigger_flag = mt_auxadc_getCurrentTrigger();
		pr_debug("[DISO]VUSB IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel,
			 trigger_flag);
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		if (trigger_flag == DISO_IRQ_FALLING) {
			DISO_data.diso_state.pre_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[4]);
		} else {
			DISO_data.diso_state.pre_vusb_state = DISO_OFFLINE;
			DISO_data.diso_state.pre_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.pre_otg_state = DISO_OFFLINE;
			DISO_data.diso_state.cur_vusb_state = DISO_ONLINE;
			DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
			DISO_data.diso_state.cur_otg_state = DISO_OFFLINE;
			pr_debug(" cur diso_state is %s!\n", DISO_state_s[6]);
		}

		set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
		break;
	default:
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
		return IRQ_HANDLED;	/* in error or unexecpt state just return */
	}
	g_diso_state = *(int *)&DISO_data.diso_state;
	return IRQ_WAKE_THREAD;
}
#endif

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
void vdc_eint_handler(void)
{
	pr_debug("[diso_eint] vdc eint irq triger\n");
	DISO_data.diso_state.cur_vdc_state = DISO_ONLINE;
	mt_eint_mask(CUST_EINT_VDC_NUM);
	do_chrdet_int_task();
}
#endif

static unsigned int diso_get_current_voltage(int Channel)
{
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, times = 5;

	if (IMM_IsAdcInitReady() == 0) {
		pr_debug("[DISO] AUXADC is not ready");
		return 0;
	}

	i = times;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);

		if (ret_value == 0) {
			ret += ret_temp;
		} else {
			times = times > 1 ? times - 1 : 1;
			pr_debug("[diso_get_current_voltage] ret_value=%d, times=%d\n",
				 ret_value, times);
		}
	}

	ret = ret * 1500 / 4096;
	ret = ret / times;

	return ret;
}

static void _get_diso_interrupt_state(void)
{
	int vol = 0;
	int diso_state = 0;
	int check_times = 30;
	kal_bool vin_state = KAL_FALSE;

#ifndef VIN_SEL_FLAG
	mdelay(AUXADC_CHANNEL_DELAY_PERIOD);
#endif

	vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vol = (R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN) * 100 * vol / (R_DISO_DC_PULL_DOWN) / 100;
	pr_debug("[DISO]  Current DC voltage mV = %d\n", vol);

#ifdef VIN_SEL_FLAG
	/* set gpio mode for kpoc issue as DWS has no default setting */
	mt_set_gpio_mode(vin_sel_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(vin_sel_gpio_number, 0);	/* 0: input, 1: output */

	if (vol > VDC_MIN_VOLTAGE / 1000 && vol < VDC_MAX_VOLTAGE / 1000) {
		/* make sure load switch already switch done */
		do {
			check_times--;
#ifdef VIN_SEL_FLAG_DEFAULT_LOW
			vin_state = mt_get_gpio_in(vin_sel_gpio_number);
#else
			vin_state = mt_get_gpio_in(vin_sel_gpio_number);
			vin_state = (~vin_state) & 0x1;
#endif
			if (!vin_state)
				mdelay(5);
		} while ((!vin_state) && check_times);
		pr_debug("[DISO] i==%d  gpio_state= %d\n",
			 check_times, mt_get_gpio_in(vin_sel_gpio_number));

		if (0 == check_times)
			diso_state &= ~0x4;	/* SET DC bit as 0 */
		else
			diso_state |= 0x4;	/* SET DC bit as 1 */
	} else {
		diso_state &= ~0x4;	/* SET DC bit as 0 */
	}
#else
	mdelay(SWITCH_RISING_TIMING + LOAD_SWITCH_TIMING_MARGIN);
	/* force delay for switching as no flag for check switching done */
	if (vol > VDC_MIN_VOLTAGE / 1000 && vol < VDC_MAX_VOLTAGE / 1000)
		diso_state |= 0x4;	/* SET DC bit as 1 */
	else
		diso_state &= ~0x4;	/* SET DC bit as 0 */
#endif


	vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vol =
	    (R_DISO_VBUS_PULL_UP +
	     R_DISO_VBUS_PULL_DOWN) * 100 * vol / (R_DISO_VBUS_PULL_DOWN) / 100;
	pr_debug("[DISO]  Current VBUS voltage  mV = %d\n", vol);

	if (vol > VBUS_MIN_VOLTAGE / 1000 && vol < VBUS_MAX_VOLTAGE / 1000) {
		if (!mt_usb_is_device()) {
			diso_state |= 0x1;	/* SET OTG bit as 1 */
			diso_state &= ~0x2;	/* SET VBUS bit as 0 */
		} else {
			diso_state &= ~0x1;	/* SET OTG bit as 0 */
			diso_state |= 0x2;	/* SET VBUS bit as 1; */
		}

	} else {
		diso_state &= 0x4;	/* SET OTG and VBUS bit as 0 */
	}
	pr_debug("[DISO] DISO_STATE==0x%x\n", diso_state);
	g_diso_state = diso_state;
	return;
}

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
int _get_irq_direction(int pre_vol, int cur_vol)
{
	int ret = -1;

	/* threshold 1000mv */
	if ((cur_vol - pre_vol) > 1000)
		ret = DISO_IRQ_RISING;
	else if ((pre_vol - cur_vol) > 1000)
		ret = DISO_IRQ_FALLING;

	return ret;
}

static void _get_polling_state(void)
{
	int vdc_vol = 0, vusb_vol = 0;
	int vdc_vol_dir = -1;
	int vusb_vol_dir = -1;

	DISO_polling_channel *VDC_Polling = &DISO_Polling.vdc_polling_measure;
	DISO_polling_channel *VUSB_Polling = &DISO_Polling.vusb_polling_measure;

	vdc_vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
	vdc_vol =
	    (R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN) * 100 * vdc_vol / (R_DISO_DC_PULL_DOWN) / 100;

	vusb_vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
	vusb_vol =
	    (R_DISO_VBUS_PULL_UP +
	     R_DISO_VBUS_PULL_DOWN) * 100 * vusb_vol / (R_DISO_VBUS_PULL_DOWN) / 100;

	VDC_Polling->preVoltage = VDC_Polling->curVoltage;
	VUSB_Polling->preVoltage = VUSB_Polling->curVoltage;
	VDC_Polling->curVoltage = vdc_vol;
	VUSB_Polling->curVoltage = vusb_vol;

	if (DISO_Polling.reset_polling) {
		DISO_Polling.reset_polling = KAL_FALSE;
		VDC_Polling->preVoltage = vdc_vol;
		VUSB_Polling->preVoltage = vusb_vol;

		if (vdc_vol > 1000)
			vdc_vol_dir = DISO_IRQ_RISING;
		else
			vdc_vol_dir = DISO_IRQ_FALLING;

		if (vusb_vol > 1000)
			vusb_vol_dir = DISO_IRQ_RISING;
		else
			vusb_vol_dir = DISO_IRQ_FALLING;
	} else {
		/* get voltage direction */
		vdc_vol_dir = _get_irq_direction(VDC_Polling->preVoltage, VDC_Polling->curVoltage);
		vusb_vol_dir =
		    _get_irq_direction(VUSB_Polling->preVoltage, VUSB_Polling->curVoltage);
	}

	if (VDC_Polling->notify_irq_en && (vdc_vol_dir == VDC_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10 * HZ / 1000);	/* 10ms */
		pr_debug("[%s] ready to trig VDC irq, irq: %d\n",
			 __func__, VDC_Polling->notify_irq);
	} else if (VUSB_Polling->notify_irq_en && (vusb_vol_dir == VUSB_Polling->notify_irq)) {
		schedule_delayed_work(&diso_polling_work, 10 * HZ / 1000);
		pr_debug("[%s] ready to trig VUSB irq, irq: %d\n",
			 __func__, VUSB_Polling->notify_irq);
	} else if ((vdc_vol == 0) && (vusb_vol == 0)) {
		VDC_Polling->notify_irq_en = 0;
		VUSB_Polling->notify_irq_en = 0;
	}

	return;
}

enum hrtimer_restart diso_kthread_hrtimer_func(struct hrtimer *timer)
{
	diso_thread_timeout = KAL_TRUE;
	wake_up(&diso_polling_thread_wq);

	return HRTIMER_NORESTART;
}

int diso_thread_kthread(void *x)
{
	/* Run on a process content */
	while (1) {
		wait_event(diso_polling_thread_wq, (diso_thread_timeout == KAL_TRUE));

		diso_thread_timeout = KAL_FALSE;

		mutex_lock(&diso_polling_mutex);

		_get_polling_state();

		if (DISO_Polling.vdc_polling_measure.notify_irq_en ||
		    DISO_Polling.vusb_polling_measure.notify_irq_en)
			hrtimer_start(&diso_kthread_timer,
				      ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)),
				      HRTIMER_MODE_REL);
		else
			hrtimer_cancel(&diso_kthread_timer);

		mutex_unlock(&diso_polling_mutex);
	}

	return 0;
}
#endif
#endif

static unsigned int charging_diso_init(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;

	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;
	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_init(&diso_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	diso_kthread_timer.function = diso_kthread_hrtimer_func;
	INIT_DELAYED_WORK(&diso_polling_work, diso_polling_handler);

	kthread_run(diso_thread_kthread, NULL, "diso_thread_kthread");
	pr_debug("[%s] done\n", __func__);
#else
	struct device_node *node;
	int ret;

	/* Initial AuxADC IRQ */
	DISO_IRQ.vdc_measure_channel.number = AP_AUXADC_DISO_VDC_CHANNEL;
	DISO_IRQ.vusb_measure_channel.number = AP_AUXADC_DISO_VUSB_CHANNEL;
	DISO_IRQ.vdc_measure_channel.period = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vusb_measure_channel.period = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vdc_measure_channel.debounce = AUXADC_CHANNEL_DEBOUNCE;
	DISO_IRQ.vusb_measure_channel.debounce = AUXADC_CHANNEL_DEBOUNCE;

	/* use default threshold voltage, if use high voltage,maybe refine */
	DISO_IRQ.vusb_measure_channel.falling_threshold = VBUS_MIN_VOLTAGE / 1000;
	DISO_IRQ.vdc_measure_channel.falling_threshold = VDC_MIN_VOLTAGE / 1000;
	DISO_IRQ.vusb_measure_channel.rising_threshold = VBUS_MIN_VOLTAGE / 1000;
	DISO_IRQ.vdc_measure_channel.rising_threshold = VDC_MIN_VOLTAGE / 1000;

	node = of_find_compatible_node(NULL, NULL, "mediatek,AUXADC");
	if (!node) {
		pr_debug("[diso_adc]: of_find_compatible_node failed!!\n");
	} else {
		pDISO_data->irq_line_number = irq_of_parse_and_map(node, 0);
		pr_debug("[diso_adc]: IRQ Number: 0x%x\n", pDISO_data->irq_line_number);
	}

	mt_irq_set_sens(pDISO_data->irq_line_number, MT_EDGE_SENSITIVE);
	mt_irq_set_polarity(pDISO_data->irq_line_number, MT_POLARITY_LOW);

	ret = request_threaded_irq(pDISO_data->irq_line_number, diso_auxadc_irq_handler,
				   pDISO_data->irq_callback_func, IRQF_ONESHOT, "DISO_ADC_IRQ",
				   NULL);

	if (ret) {
		pr_debug("[diso_adc]: request_irq failed.\n");
	} else {
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		pr_debug("[diso_adc]: diso_init success.\n");
	}
#endif

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
	pr_debug("[diso_eint]vdc eint irq registitation\n");
	mt_eint_set_hw_debounce(CUST_EINT_VDC_NUM, CUST_EINT_VDC_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_VDC_NUM, CUST_EINTF_TRIGGER_LOW, vdc_eint_handler, 0);
	mt_eint_mask(CUST_EINT_VDC_NUM);
#endif
#endif

	return status;
}

static unsigned int charging_get_diso_state(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *) data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	pr_debug("[do_chrdet_int_task] current diso state is %s!\n", DISO_state_s[diso_state]);
	if (((diso_state >> 1) & 0x3) != 0x0) {
		switch (diso_state) {
		case USB_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
#ifdef MTK_DSC_USE_EINT
			mt_eint_unmask(CUST_EINT_VDC_NUM);
#else
			set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
#endif
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_ONLY:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_USB:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
			pDISO_data->diso_state.cur_vusb_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_OFFLINE;
			break;
		case DC_WITH_OTG:
			set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
			set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_ONLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			break;
		default:	/* OTG only also can trigger vcdt IRQ */
			pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_vdc_state = DISO_OFFLINE;
			pDISO_data->diso_state.cur_otg_state = DISO_ONLINE;
			pr_debug(" switch load vcdt irq triggerd by OTG Boost!\n");
			break;	/* OTG plugin no need battery sync action */
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
#endif

	return status;
}


static unsigned int charging_get_error_state(void)
{
	return charging_error;
}

static unsigned int charging_set_error_state(void *data)
{
	unsigned int status = STATUS_OK;
	charging_error = *(unsigned int *) (data);

	return status;
}

static unsigned int charging_set_vindpm(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_set_vbus_ovp_en(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_get_bif_vbat(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_set_chrind_ck_pdn(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_sw_init(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_enable_safetytimer(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_set_hiz_swchr(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

static unsigned int charging_get_bif_tbat(void *data)
{
	unsigned int status = STATUS_OK;
	return status;
}

//zhounengwen@wind-mobi.com 20170512 begin
static unsigned int charging_enable_otg(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int enable = 0;
	enable = *((unsigned int *)data);
	if(enable)
	{
		bq2560x_set_otg_config(0x1);
		bq2560x_set_boost_lim(0x1);
	}
	else
	{
		bq2560x_set_otg_config(0);
	}
	printk("[kevin_dump]charging_enable_otg\n");
	bq2560x_dump_register();
	return status;
}
//zhounengwen@wind-mobi.com 20170512 end

//yangzhigang@wind-mobi.com 20170522 begin
static unsigned int(* charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
charging_hw_init,
charging_dump_register,
charging_enable,
charging_set_cv_voltage,
    charging_get_current,
    charging_set_current,
    charging_set_input_current,
    charging_get_charging_status,
    charging_reset_watch_dog_timer,
    charging_set_hv_threshold,
    charging_get_hv_status,
    charging_get_battery_status,
    charging_get_charger_det_status,
    charging_get_charger_type,
    charging_get_is_pcm_timer_trigger,
    charging_set_platform_reset,
    charging_get_platfrom_boot_mode,
    charging_set_power_off,
    charging_get_power_source,
    charging_get_csdac_full_flag,
    charging_set_ta_current_pattern,
    charging_set_error_state,
    charging_diso_init,
    charging_get_diso_state,
    charging_set_vindpm,
    charging_set_vbus_ovp_en,
    charging_get_bif_vbat,
    charging_set_chrind_ck_pdn,
    charging_sw_init,
    charging_enable_safetytimer,
    charging_set_hiz_swchr,
    charging_get_bif_tbat,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    charging_enable_otg,
    NULL,
    NULL,
    charging_get_input_current,
    NULL,
    NULL,
	NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
	charging_set_suspend_charge,       //liulinsheng 20171016
	charging_set_recover_charge,       //liulinsheng 20171016
    };

//yangzhigang@wind-mobi.com 20170522 end
/*
* FUNCTION
*		Internal_chr_control_handler
*
* DESCRIPTION
*		 This function is called to set the charger hw
*
* CALLS
*
* PARAMETERS
*		None
*
* RETURNS
*
*
* GLOBALS AFFECTED
*	   None
*/
signed int bq2560x_chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	signed int status;
	//yangzhigang@wind-mobi.com 20170508 modify begin
	printk("yzg-b char chr_control_interface\n");
	if (cmd < CHARGING_CMD_NUMBER) {
		if (charging_func[cmd] != NULL)
			status = charging_func[cmd] (data);
		else {
			battery_log(BAT_LOG_CRTI, "yzg-b char [chr_control_interface]cmd:%d not supported\n", cmd);
			status = STATUS_UNSUPPORTED;
		}
	} else
		status = STATUS_UNSUPPORTED;
	//yangzhigang@wind-mobi.com 20170508 modify end

	return status;
}
