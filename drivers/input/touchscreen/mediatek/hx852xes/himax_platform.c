//ranyanhao@wind-mobi.com 20160215 begin
/* Himax Android Driver Sample Code for Himax chipset
*
* Copyright (C) 2014 Himax Corporation.
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

#include "himax_platform.h"

#define CONFIG_TOUCHSCREEN_HIMAX_DEBUG 
//qiumeng@wind-mobi.com 20160525 begin
struct himax_i2c_platform_data *private_pdata;
//qiumeng@wind-mobi.com 20160525 end
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
#define D(x...) printk("[HXTP] " x)
#define I(x...) printk("[HXTP] " x)
#define W(x...) printk("[HXTP][WARNING] " x)
#define E(x...) printk("[HXTP][ERROR] " x)
#endif

//qiumeng@wind-mobi.com 20160513 begin
//int irq_enable_count = 0;
unsigned int irq_enable_count = 0;
//qiumeng@wind-mobi.com 20160513 end

DEFINE_MUTEX(hx_wr_access);

#ifdef QCT
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry/*, loop_i*/;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	memcpy(buf+1, data, length);
	
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
		       __func__, toRetry);
		return -EIO;
	}
	return 0;
}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry/*, loop_i*/;
	uint8_t buf[length];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

	memcpy(buf, data, length);
	
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
		       __func__, toRetry);
		return -EIO;
	}
	return 0;
}

void himax_int_enable(int irqnum, int enable, int log_print)
{
	if (enable == 1 && irq_enable_count == 0) {
		enable_irq(irqnum);
		//qiumeng@wind-mobi.com 20160513 begin
		//irq_enable_count++;
		irq_enable_count=1;
		//qiumeng@wind-mobi.com 20160513 end	
	} else if (enable == 0 && irq_enable_count == 1) {
		disable_irq_nosync(irqnum);
		//qiumeng@wind-mobi.com 20160513 begin
		//irq_enable_count--;
		irq_enable_count=0;
	}
	else if (enable == 1 && irq_enable_count == 1)
		irq_enable_count=1;
	else if (enable == 0 && irq_enable_count == 0)
		irq_enable_count=0;
	else//Not match all of these 4 cases
		{
			disable_irq_nosync(irqnum);
			irq_enable_count=0;
		}
	   //qiumeng@wind-mobi.com 20160513 end	
	if(log_print)
		I("irq_enable_count = %d\n", irq_enable_count);
}

void himax_rst_gpio_set(int pinnum, uint8_t value)
{
	gpio_direction_output(pinnum, value);
}

uint8_t himax_int_gpio_read(int pinnum)
{
	return gpio_get_value(pinnum);
}

#if defined(CONFIG_HMX_DB)
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int himax_power_on(struct himax_i2c_platform_data *pdata, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	rc = reg_set_optimum_mode_check(pdata->vcc_ana, HX_ACTIVE_LOAD_UA);
	if (rc < 0) {
		E("Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(pdata->vcc_ana);
	if (rc) {
		E("Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

	if (pdata->digital_pwr_regulator) {
		rc = reg_set_optimum_mode_check(pdata->vcc_dig,
					HX_ACTIVE_LOAD_DIG_UA);
		if (rc < 0) {
			E("Regulator vcc_dig set_opt failed rc=%d\n",
				rc);
			goto error_reg_opt_vcc_dig;
		}

		rc = regulator_enable(pdata->vcc_dig);
		if (rc) {
			E("Regulator vcc_dig enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_dig;
		}
	}

	if (pdata->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(pdata->vcc_i2c, HX_I2C_LOAD_UA);
		if (rc < 0) {
			E("Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(pdata->vcc_i2c);
		if (rc) {
			E("Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	}

	msleep(130);

	return 0;

error_reg_en_vcc_i2c:
	if (pdata->i2c_pull_up)
		reg_set_optimum_mode_check(pdata->vcc_i2c, 0);
error_reg_opt_i2c:
	if (pdata->digital_pwr_regulator)
		regulator_disable(pdata->vcc_dig);
error_reg_en_vcc_dig:
	if (pdata->digital_pwr_regulator)
		reg_set_optimum_mode_check(pdata->vcc_dig, 0);
error_reg_opt_vcc_dig:
	regulator_disable(pdata->vcc_ana);
error_reg_en_vcc_ana:
	reg_set_optimum_mode_check(pdata->vcc_ana, 0);
	return rc;

power_off:
	reg_set_optimum_mode_check(pdata->vcc_ana, 0);
	regulator_disable(pdata->vcc_ana);
	if (pdata->digital_pwr_regulator) {
		reg_set_optimum_mode_check(pdata->vcc_dig, 0);
		regulator_disable(pdata->vcc_dig);
	}
	if (pdata->i2c_pull_up) {
		reg_set_optimum_mode_check(pdata->vcc_i2c, 0);
		regulator_disable(pdata->vcc_i2c);
	}
	msleep(50);
	return 0;
}

static int himax_regulator_configure(struct i2c_client *client,struct himax_i2c_platform_data *pdata, bool on)
{
	int rc;

	if (on == false)
		goto hw_shutdown;

	pdata->vcc_ana = regulator_get(&client->dev, "vdd_ana");
	if (IS_ERR(pdata->vcc_ana)) {
		rc = PTR_ERR(pdata->vcc_ana);
		E("Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vcc_ana) > 0) {
		rc = regulator_set_voltage(pdata->vcc_ana, HX_VTG_MIN_UV,
							HX_VTG_MAX_UV);
		if (rc) {
			E("regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
	if (pdata->digital_pwr_regulator) {
		pdata->vcc_dig = regulator_get(&client->dev, "vdd_dig");
		if (IS_ERR(pdata->vcc_dig)) {
			rc = PTR_ERR(pdata->vcc_dig);
			E("Regulator get dig failed rc=%d\n", rc);
			goto error_get_vtg_vcc_dig;
		}

		if (regulator_count_voltages(pdata->vcc_dig) > 0) {
			rc = regulator_set_voltage(pdata->vcc_dig,
				HX_VTG_DIG_MIN_UV, HX_VTG_DIG_MAX_UV);
			if (rc) {
				E("regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_vcc_dig;
			}
		}
	}
	if (pdata->i2c_pull_up) {
		pdata->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR(pdata->vcc_i2c)) {
			rc = PTR_ERR(pdata->vcc_i2c);
			E("Regulator get failed rc=%d\n",	rc);
			goto error_get_vtg_i2c;
		}
		if (regulator_count_voltages(pdata->vcc_i2c) > 0) {
			rc = regulator_set_voltage(pdata->vcc_i2c,
				HX_I2C_VTG_MIN_UV, HX_I2C_VTG_MAX_UV);
			if (rc) {
				E("regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	}

	return 0;

error_set_vtg_i2c:
	regulator_put(pdata->vcc_i2c);
error_get_vtg_i2c:
	if (pdata->digital_pwr_regulator)
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
				HX_VTG_DIG_MAX_UV);
error_set_vtg_vcc_dig:
	if (pdata->digital_pwr_regulator)
		regulator_put(pdata->vcc_dig);
error_get_vtg_vcc_dig:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, HX_VTG_MAX_UV);
error_set_vtg_vcc_ana:
	regulator_put(pdata->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, HX_VTG_MAX_UV);
	regulator_put(pdata->vcc_ana);
	if (pdata->digital_pwr_regulator) {
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
						HX_VTG_DIG_MAX_UV);
		regulator_put(pdata->vcc_dig);
	}
	if (pdata->i2c_pull_up) {
		if (regulator_count_voltages(pdata->vcc_i2c) > 0)
			regulator_set_voltage(pdata->vcc_i2c, 0,
						HX_I2C_VTG_MAX_UV);
		regulator_put(pdata->vcc_i2c);
	}
	return 0;
}

int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata)
{
	int error;
	
	error = himax_regulator_configure(client, pdata, true);
	if (error) {
		E("Failed to intialize hardware\n");
		goto err_regulator_not_on;
	}

	if (gpio_is_valid(pdata->gpio_reset)) {
		/* configure touchscreen reset out gpio */
		error = gpio_request(pdata->gpio_reset, "hmx_reset_gpio");
		if (error) {
			E("unable to request gpio [%d]\n",
						pdata->gpio_reset);
			goto err_regulator_on;
		}

		error = gpio_direction_output(pdata->gpio_reset, 0);
		if (error) {
			E("unable to set direction for gpio [%d]\n",
				pdata->gpio_reset);
			goto err_gpio_reset_req;
		}		
	}

	error = himax_power_on(pdata, true);
	if (error) {
		E("Failed to power on hardware\n");
		goto err_gpio_reset_req;
	}

	if (gpio_is_valid(pdata->gpio_irq)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->gpio_irq, "hmx_gpio_irq");
		if (error) {
			E("unable to request gpio [%d]\n",
						pdata->gpio_irq);
			goto err_power_on;
		}
		error = gpio_direction_input(pdata->gpio_irq);
		if (error) {
			E("unable to set direction for gpio [%d]\n",
				pdata->gpio_irq);
			goto err_gpio_irq_req;
		}
		client->irq = gpio_to_irq(pdata->gpio_irq);
	} else {
		E("irq gpio not provided\n");
		goto err_power_on;
	}

	msleep(20);
	if (gpio_is_valid(pdata->gpio_reset)) {
		error = gpio_direction_output(pdata->gpio_reset, 1);
		if (error) {
			E("unable to set direction for gpio [%d]\n",
				pdata->gpio_reset);
				goto err_gpio_irq_req;
		}
	}
	msleep(20);

	return 0;
	
err_gpio_irq_req:
	if (gpio_is_valid(pdata->gpio_irq))
		gpio_free(pdata->gpio_irq);
err_power_on:
		himax_power_on(pdata, false);
err_gpio_reset_req:
	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
err_regulator_on:
		himax_regulator_configure(client, pdata, false);
err_regulator_not_on:

return error;
}

#else
int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata)
{
	int error=0;

	if (pdata->gpio_reset >= 0) {
		error = gpio_request(pdata->gpio_reset, "himax-reset");
		if (error < 0){
				E("%s: request reset pin failed\n", __func__);
				return error;
		}
		error = gpio_direction_output(pdata->gpio_reset, 0);
		if (error) {
			E("unable to set direction for gpio [%d]\n",
				pdata->gpio_reset);
			return error;
		}
	}
	if (pdata->gpio_3v3_en >= 0) {
		error = gpio_request(pdata->gpio_3v3_en, "himax-3v3_en");
		if (error < 0) {
				E("%s: request 3v3_en pin failed\n", __func__);
				return error;
			}
		gpio_direction_output(pdata->gpio_3v3_en, 1);
		I("3v3_en pin =%d\n", gpio_get_value(pdata->gpio_3v3_en));
	}
	if (gpio_is_valid(pdata->gpio_irq)) {
	/* configure touchscreen irq gpio */
	error = gpio_request(pdata->gpio_irq, "himax_gpio_irq");
	if (error) {
			E("unable to request gpio [%d]\n",pdata->gpio_irq);
			return error;
		}
		error = gpio_direction_input(pdata->gpio_irq);
		if (error) {
			E("unable to set direction for gpio [%d]\n",pdata->gpio_irq);
			return error;
		}
		client->irq = gpio_to_irq(pdata->gpio_irq);
	} else {
		E("irq gpio not provided\n");
		return error;
	}
	msleep(20);

	if (pdata->gpio_reset >= 0) {
		error = gpio_direction_output(pdata->gpio_reset, 1);
		if (error) {
			E("unable to set direction for gpio [%d]\n",
				pdata->gpio_reset);
			return error;
		}
	}

	msleep(20);
	
return error;
}

#endif

#endif

#ifdef MTK
u8 *gpDMABuf_va = NULL;
u8 *gpDMABuf_pa = NULL;
static int himax_tpd_int_gpio = 5;
extern int himax_tpd_rst_gpio_number;
extern int himax_tpd_int_gpio_number;

int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int ret=0;
	s32 retry = 0;
	u8 buffer[1];

	struct i2c_msg msg[] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buffer,
			.len = 1,
			.timing = 400
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = gpDMABuf_pa,
			.len = length,
			.timing = 400
		},
	};
	mutex_lock(&hx_wr_access);
	buffer[0] = command;

	if (data == NULL){
		mutex_unlock(&hx_wr_access);
		return -1;
	}
	for (retry = 0; retry < toRetry; ++retry)
	{
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
		{
			continue;
		}
		memcpy(data, gpDMABuf_va, length);
		mutex_unlock(&hx_wr_access);
		return 0;
	}
	E("Dma I2C Read Error: %d byte(s), err-code: %d", length, ret);
	mutex_unlock(&hx_wr_access);
	return ret;
}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
	int rc=0,retry=0;
	u8 *pWriteData = gpDMABuf_va;

	struct i2c_msg msg[]={
		{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = gpDMABuf_pa,
		.len = len+1,
		.timing = 400
		},
	};
	mutex_lock(&hx_wr_access);
	if(!pWriteData)
	{
		E("dma_alloc_coherent failed!\n");
		mutex_unlock(&hx_wr_access);
		return -1;
	}

	gpDMABuf_va[0] = command;

	memcpy(gpDMABuf_va+1, buf, len);

	for (retry = 0; retry < toRetry; ++retry)
	{
		rc = i2c_transfer(client->adapter, &msg[0], 1);
		if (rc < 0)
		{
			continue;
		}
		mutex_unlock(&hx_wr_access);
		return 0;
	}

	E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
	mutex_unlock(&hx_wr_access);
	return rc;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t len, uint8_t *buf, uint8_t *readlength, uint8_t toRetry)
{
	return 0;
}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
	int rc=0, retry=0;
	u8 *pWriteData = gpDMABuf_va;

	struct i2c_msg msg[] ={
		{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = gpDMABuf_pa,
		.len = len,
		.timing = 400
		},
	};

	mutex_lock(&hx_wr_access);
	if(!pWriteData)
	{
		E("dma_alloc_coherent failed!\n");
		mutex_unlock(&hx_wr_access);
		return -1;
	}

	memcpy(gpDMABuf_va, buf, len);
	for (retry = 0; retry < toRetry; ++retry)
	{
		rc = i2c_transfer(client->adapter, &msg[0], 1);
		if (rc < 0)
		{
			continue;
		}
		mutex_unlock(&hx_wr_access);
		return 0;
	}
	E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
	mutex_unlock(&hx_wr_access);
	return rc;
}

void himax_int_enable(int irqnum, int enable, int log_print)
{
//qiumeng@wind-mobi.com 20160525 begin
	if (enable == 1 && (!atomic_read(&private_pdata->int_state))) {
		irq_enable_count=1;
		atomic_set(&private_pdata->int_state, 1);
//qiumeng@wind-mobi.com 20160525 end
#ifdef CONFIG_OF_TOUCH
		enable_irq(irqnum);
#else
		mt_eint_unmask(irqnum);
#endif
//qiumeng@wind-mobi.com 20160525 begin
        //irq_enable_count++;
		//irq_enable_count=1;
	//} else if (enable == 0 && irq_enable_count == 1) {
	} else if (enable == 0 && (atomic_read(&private_pdata->int_state))) {
#if 0
#ifdef CONFIG_OF_TOUCH
		disable_irq_nosync(irqnum);
#else
		mt_eint_mask(irqnum);
#endif
#endif
//qiumeng@wind-mobi.com 20160525 end
//qiumeng@wind-mobi.com 20160513 begin
        //irq_enable_count--;
		irq_enable_count=0;
//qiumeng@wind-mobi.com 20160525 begin
	    atomic_set(&private_pdata->int_state, 0);
	//}
	//else if (enable == 1 && irq_enable_count == 1)
		//irq_enable_count=1;
	//else if (enable == 0 && irq_enable_count == 0)
	//	irq_enable_count=0;
	//else//Not match all of these 4 cases
	//	{
//qiumeng@wind-mobi.com 20160525 end
#ifdef CONFIG_OF_TOUCH
			disable_irq_nosync(irqnum);
#else
			mt_eint_mask(irqnum);
#endif
//qiumeng@wind-mobi.com 20160525 begin
			//irq_enable_count=0;
//qiumeng@wind-mobi.com 20160525 end
		}
//qiumeng@wind-mobi.com 20160513 end
if(log_print)
//qiumeng@wind-mobi.com 20160525 begin
	//I("irq_enable_count = %d\n", irq_enable_count);
	I("enable=%d, irq_enable_count = %d, int_state =%d\n", enable, irq_enable_count,atomic_read(&private_pdata->int_state));
//qiumeng@wind-mobi.com 20160525 end
}

void himax_rst_gpio_set(int pinnum, uint8_t value)
{
		
	/*
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	if(value)
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO); */
	/*	
        int retval = 0;
	retval = gpio_request_one(pinnum, GPIOF_OUT_INIT_LOW,
				 "touchp_reset");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio reset_pin\n");
	}*/
	
	if(value)
		tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
	else
		tpd_gpio_output(himax_tpd_rst_gpio_number, 0);			
}

uint8_t himax_int_gpio_read(int pinnum)
{
	//return mt_get_gpio_in(GPIO_CTP_EINT_PIN);
	return  gpio_get_value(himax_tpd_int_gpio);
}


int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata)
{
	int ret = 0;

	private_pdata=pdata;
#if 0
	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	msleep(100);
#endif
	printk("hb  power_on\n");
/*
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
*/	
	ret = regulator_enable(tpd->reg);
	if (ret != 0)
		printk("Failed to disable reg-vgp6: %d\n", ret);
	
		tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
		msleep(30);
		tpd_gpio_output(himax_tpd_rst_gpio_number, 0);
		msleep(30);
		tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
		msleep(120);

	TPD_DMESG("mtk_tpd: himax reset over \n");
	
	/* set INT mode */

	tpd_gpio_as_int(himax_tpd_int_gpio_number);

	/*
	 
	retval = gpio_request_one(pdata->gpio_reset, GPIOF_OUT_INIT_LOW,
				 "touchp_reset");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio reset_pin\n");
	}
	
	
	

	//Himax: SET Interrupt GPIO, no setting PULL LOW or PULL HIGH  
	//mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	//mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
	//mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	retval = gpio_request_one(pdata->gpio_irq, GPIOF_IN,
				 "tpd_int");
	if (retval < 0) {
		TPD_DMESG("Unable to request gpio int_pin\n");
		gpio_free(pdata->gpio_irq);
	}
	
	
	
	// TODO Power Pin Setup
	//hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
	//hwPowerOn(MT65XX_POWER_LDO_VGP5, VOL_1800, "TP_EINT");
	msleep(10);

	// HW Reset
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	gpio_direction_output(pdata->gpio_reset, 1);
	msleep(20);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	gpio_direction_output(pdata->gpio_reset, 0);
	msleep(20);
	//mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	gpio_direction_output(pdata->gpio_reset, 1);	
	msleep(20);  */

return 0;
}

#endif
//ranyanhao@wind-mobi.com 20160215 end