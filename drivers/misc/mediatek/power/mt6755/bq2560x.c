#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include "bq2560x.h"


/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define BQ2560x_SLAVE_ADDR_WRITE   0xD6
#define BQ2560x_SLAVE_ADDR_READ    0xD7


static struct i2c_client *new_client;
static const struct i2c_device_id bq2560x_i2c_id[] = { {"bq2560x", 0}, {} };

//yangzhigang@wind-mobi.com 20170602 begin
extern kal_bool chargin_hw_init_done;
extern int bq25896_exist_flag;/* chengronghe@wind-mobi.com add  2017-07-07 */
//extern int compatible_flag; 
//extern int regist_flag;
//yangzhigang@wind-mobi.com 20170602 end

static int bq2560x_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
#if 0
static void bq2560x_shutdown(struct platform_device *dev)
{
	pr_debug("[bq2560x_shutdown] driver shutdown\n");
	bq2560x_set_chg_config(0x0);	/* charger disable */
}

#endif
#ifdef CONFIG_OF
static const struct of_device_id bq2560x_of_match[] = {
	{.compatible = "mediatek,SWITHING_CHARGER",},
	{},
};

MODULE_DEVICE_TABLE(of, bq2560x_of_match);
#endif

static struct i2c_driver bq2560x_driver = {
	.driver = {
		   .name = "bq25601",
#ifdef CONFIG_OF
		   .of_match_table = bq2560x_of_match,
#endif
		   },
	.probe = bq2560x_driver_probe,
	.id_table = bq2560x_i2c_id,
	//.shutdown = bq2560x_shutdown,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq2560x_reg[bq2560x_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq2560x_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write bq2560x]
  *
  *********************************************************/
int bq2560x_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&bq2560x_i2c_access);
	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;
		mutex_unlock(&bq2560x_i2c_access);

		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;
	mutex_unlock(&bq2560x_i2c_access);

	return 1;
}

int bq2560x_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&bq2560x_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&bq2560x_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&bq2560x_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq2560x_read_interface(unsigned char RegNum, unsigned char*val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq2560x_reg = 0;
	int ret = 0;

	//pr_debug("--------------------------------------------------\n");

	ret = bq2560x_read_byte(RegNum, &bq2560x_reg);
	//pr_debug("[bq2560x_read_interface] Reg[%x]=0x%x\n", RegNum, bq2560x_reg);

	bq2560x_reg &= (MASK << SHIFT);
	*val = (bq2560x_reg >> SHIFT);
	//pr_debug("[bq2560x_read_interface] Val=0x%x\n", *val);

	return ret;
}

unsigned int bq2560x_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq2560x_reg = 0;
	int ret = 0;

	printk("----------------------bq2560x_config_interface----------------------------\n");

	ret = bq2560x_read_byte(RegNum, &bq2560x_reg);
	//printk("[bq2560x_config_interface] Reg[%x]=0x%x\n", RegNum, bq2560x_reg);

	bq2560x_reg &= ~(MASK << SHIFT);
	bq2560x_reg |= (val << SHIFT);

	ret = bq2560x_write_byte(RegNum, bq2560x_reg);
	//printk("[bq2560x_config_interface] Write Reg[%x]=0x%x\n", RegNum, bq2560x_reg);

	/* Check */
	 bq2560x_read_byte(RegNum, &bq2560x_reg); 
	 //printk("[bq2560x_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq2560x_reg); 

	return ret;
}

/* write one register directly */
unsigned int bq2560x_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned char ret = 0;

	ret = bq2560x_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */

void bq2560x_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}

void bq2560x_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
	    );
}

//yangzhigang@wind-mobi.com 20170508 add begin
#if 0
void bq2560x_set_stat_ctrl(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_STAT_IMON_CTRL_MASK),
				       (unsigned char) (CON0_STAT_IMON_CTRL_SHIFT)
	    );
}
#endif
//yangzhigang@wind-mobi.com 20170508 add end
/* CON1---------------------------------------------------- */

void bq2560x_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON11),
				       (unsigned char) (val),
				       (unsigned char) (CON11_REG_RST_MASK),
				       (unsigned char) (CON11_REG_RST_SHIFT)
	    );
}

void bq2560x_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK),
				       (unsigned char) (CON1_WDT_RST_SHIFT)
	    );
}

void bq2560x_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK),
				       (unsigned char) (CON1_OTG_CONFIG_SHIFT)
	    );
}


void bq2560x_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT)
	    );
}


void bq2560x_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK),
				       (unsigned char) (CON1_SYS_MIN_SHIFT)
	    );
}

void bq2560x_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_MIN_VBAT_SEL_MASK),
				       (unsigned char) (CON1_MIN_VBAT_SEL_SHIFT)
	    );
}



/* CON2---------------------------------------------------- */

void bq2560x_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BOOST_LIM_MASK),
				       (unsigned char) (CON2_BOOST_LIM_SHIFT)
	    );
}

void bq2560x_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICHG_MASK), (unsigned char) (CON2_ICHG_SHIFT)
	    );
}

#if 0 //this function does not exist on bq2560x
void bq2560x_set_force_20pct(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_20PCT_MASK),
				       (unsigned char) (CON2_FORCE_20PCT_SHIFT)
	    );
}
#endif
/* CON3---------------------------------------------------- */

void bq2560x_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK),
				       (unsigned char) (CON3_IPRECHG_SHIFT)
	    );
}

void bq2560x_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_ITERM_MASK), (unsigned char) (CON3_ITERM_SHIFT)
	    );
}

/* CON4---------------------------------------------------- */

void bq2560x_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VREG_MASK), (unsigned char) (CON4_VREG_SHIFT)
	    );
}

void bq2560x_set_topoff_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_TOPOFF_TIMER_MASK), (unsigned char) (CON4_TOPOFF_TIMER_SHIFT)
	    );

}


void bq2560x_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VRECHG_MASK),
				       (unsigned char) (CON4_VRECHG_SHIFT)
	    );
}

/* CON5---------------------------------------------------- */

void bq2560x_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK),
				       (unsigned char) (CON5_EN_TERM_SHIFT)
	    );
}



void bq2560x_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK),
				       (unsigned char) (CON5_WATCHDOG_SHIFT)
	    );
}

void bq2560x_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK),
				       (unsigned char) (CON5_EN_TIMER_SHIFT)
	    );
}

void bq2560x_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK),
				       (unsigned char) (CON5_CHG_TIMER_SHIFT)
	    );
}
//liulinsheng@wind-mobi.com 20171001 begin
void bq2560x_set_low_temp_jieta(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_JEITA_MASK),
				       (unsigned char) (CON5_JEITA_SHIFT)
	    );
}
//liulinsheng@wind-mobi.com 20171001 end
/* CON6---------------------------------------------------- */
//yangzhigang@wind-mobi.com 20170508 add begin
#if 0
void bq2560x_set_treg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_TREG_MASK), (unsigned char) (CON6_TREG_SHIFT)
	    );
}
#endif
//yangzhigang@wind-mobi.com 20170508 add end

void bq2560x_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VINDPM_MASK),
				       (unsigned char) (CON6_VINDPM_SHIFT)
	    );
}


void bq2560x_set_ovp(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_OVP_MASK),
				       (unsigned char) (CON6_OVP_SHIFT)
	    );

}

void bq2560x_set_boostv(unsigned int val)
{

	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
	    );



}



/* CON7---------------------------------------------------- */

void bq2560x_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_TMR2X_EN_MASK),
				       (unsigned char) (CON7_TMR2X_EN_SHIFT)
	    );
}

void bq2560x_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_Disable_MASK),
				       (unsigned char) (CON7_BATFET_Disable_SHIFT)
	    );
}


void bq2560x_set_batfet_delay(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_DLY_MASK),
				       (unsigned char) (CON7_BATFET_DLY_SHIFT)
	    );
}

void bq2560x_set_batfet_reset_enable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_RST_EN_MASK),
				       (unsigned char) (CON7_BATFET_RST_EN_SHIFT)
	    );
}

//liulinsheng@wind-mobi.com 20171001 begin
void bq2560x_set_high_temp_jieta(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_JTETA_MASK),
				       (unsigned char) (CON7_JIETA_SHIFT)
	    );
}
//liulinsheng@wind-mobi.com 20171001 end
/* CON8---------------------------------------------------- */

unsigned int bq2560x_get_system_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val), (unsigned char) (0xFF), (unsigned char) (0x0)
	    );
	return val;
}

unsigned int bq2560x_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_VBUS_STAT_MASK),
				     (unsigned char) (CON8_VBUS_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq2560x_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_CHRG_STAT_MASK),
				     (unsigned char) (CON8_CHRG_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq2560x_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_VSYS_STAT_MASK),
				     (unsigned char) (CON8_VSYS_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq2560x_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq2560x_read_interface((unsigned char) (bq2560x_CON8),
				     (&val),
				     (unsigned char) (CON8_PG_STAT_MASK),
				     (unsigned char) (CON8_PG_STAT_SHIFT)
	    );
	return val;
}


/*CON10----------------------------------------------------------*/

void bq2560x_set_int_mask(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq2560x_config_interface((unsigned char) (bq2560x_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_INT_MASK_MASK),
				       (unsigned char) (CON10_INT_MASK_SHIFT)
	    );
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void bq2560x_dump_register(void)
{
	unsigned char i = 0;
	printk("[bq2560x] ");
	for (i = 0; i < bq2560x_REG_NUM; i++) {
		bq2560x_read_byte(i, &bq2560x_reg[i]);
		printk("[0x%x]=0x%x ", i, bq2560x_reg[i]);
	}
	pr_debug("\n");
}
/*chengronghe@wind-mobi.com add for XXX 2017-06-30  begin*/
extern CHARGING_CONTROL battery_charging_control;
extern signed int bq2560x_chr_control_interface(CHARGING_CTRL_CMD cmd, void *data);

/*chengronghe@wind-mobi.com add for XXX 2017-06-30 end*/

static int bq2560x_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	/*liulinsheng@wind-mobi.com 20170924 bengin */
	unsigned char check_returnData = 0;
	int reg2 = 0;
	/*liulinsheng@wind-mobi.com 20170924 end */
	pr_debug("[bq2560x_driver_probe]\n");
	printk("yzg char bq2560x_driver_probe addr:0x%x\n",client->addr);
	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));
	
	client->addr = 0x6B;//yangzhigang@wind-mobi.com change 20170602 
	new_client = client;
	
/* modify begin by chengronghe@wind-mobi.com, 2017-07-01 */
	
	bq2560x_dump_register();
	reg2 = bq2560x_read_byte(0x0B,&check_returnData);
	check_returnData = check_returnData & 0x70;
	if(check_returnData == 0x10){
	battery_log(BAT_LOG_CRTI, "%s:[chengronghe] BQ2560x exists\n", __func__);
	/* bq2560x_hw_init(); //move to charging_hw_xxx.c */
	chargin_hw_init_done = true;
	/* Hook chr_control_interface with battery's interface */
	battery_charging_control = bq2560x_chr_control_interface;
	printk("[chengronghe] set bq2560x_chr_control_interface\n");
	battery_log(BAT_LOG_CRTI, "%s:[chengronghe] bq2560x_driver_probe suc\n", __func__);
	}
	else{
		battery_log(BAT_LOG_CRTI, "%s:[chengronghe] NO BQ2560x exists\n", __func__);
		return -1;
	}
/* modify end by chengronghe@wind-mobi.com, 2017-07-01 */
	
	return 0;

exit:
	return err;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq2560x = 0;
static ssize_t show_bq2560x_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_debug("[show_bq2560x_access] 0x%x\n", g_reg_value_bq2560x);
	return sprintf(buf, "%u\n", g_reg_value_bq2560x);
}

static ssize_t store_bq2560x_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_debug("[store_bq2560x_access]\n");

	if (buf != NULL && size != 0) {
		pr_debug("[store_bq2560x_access] buf is %s and size is %zu\n", buf, size);
		reg_address = simple_strtoul(buf, &pvalue, 16);

		if (size > 3) {
			reg_value = simple_strtoul((pvalue + 1), NULL, 16);
			pr_debug
			    ("[store_bq2560x_access] write bq2560x reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = bq2560x_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq2560x_read_interface(reg_address, &g_reg_value_bq2560x, 0xFF, 0x0);
			pr_debug("[store_bq2560x_access] read bq2560x reg 0x%x with value 0x%x !\n",
				 reg_address, g_reg_value_bq2560x);
			pr_debug
			    ("[store_bq2560x_access] Please use \"cat bq2560x_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(bq2560x_access, 0664, show_bq2560x_access, store_bq2560x_access);	/* 664 */

static int bq2560x_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	pr_debug("******** bq2560x_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq2560x_access);

	return 0;
}
//yangzhigang@wind-mobi.com 20170612 begin
void otg_shutdown_2(struct platform_device *dev)
{
	bq2560x_set_otg_config(0);
	printk("franca-otg otg_shutdown_1 \n");
}
//yangzhigang@wind-mobi.com 20170612 end

struct platform_device bq2560x_user_space_device = {
	.name = "bq2560x-user",
	.id = -1,
};

static struct platform_driver bq2560x_user_space_driver = {
	.probe = bq2560x_user_space_probe,
	.driver = {
		   .name = "bq2560x-user",
		   },
	.shutdown = otg_shutdown_2,
};


static int __init bq2560x_subsys_init(void)
{
	int ret = 0;
//yangzhigang@wind-mobi.com 20170602 begin
	if(!bq25896_exist_flag){/* modify by chengronghe@wind-mobi.com, 2017-07-07 */

		if (i2c_add_driver(&bq2560x_driver) != 0)
			pr_debug("[bq2560x_init] failed to register bq2560x i2c driver.\n");
		else
			pr_debug("[bq2560x_init] Success to register bq2560x i2c driver.\n");


		/* bq2560x user space access interface */
		ret = platform_device_register(&bq2560x_user_space_device);
		if (ret) {
			pr_debug("****[bq2560x_init] Unable to device register(%d)\n", ret);
			return ret;
		}
		ret = platform_driver_register(&bq2560x_user_space_driver);
		if (ret) {
			pr_debug("****[bq2560x_init] Unable to register driver (%d)\n", ret);
			return ret;
		}

		return 0;
	}else{
		printk("bq2560x_probe not loader,bq25896_exist_flag = %d\n",bq25896_exist_flag);/* modify by chengronghe@wind-mobi.com, 2017-07-07 */
		return 0;
	}
//yangzhigang@wind-mobi.com 20170602 end
}

static void __exit bq2560x_exit(void)
{
	i2c_del_driver(&bq2560x_driver);
}

/* module_init(bq2560x_init); */
/* module_exit(bq2560x_exit); */
module_init(bq2560x_subsys_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq2560x Driver");
MODULE_AUTHOR("YT Lee<yt.lee@mediatek.com>");
