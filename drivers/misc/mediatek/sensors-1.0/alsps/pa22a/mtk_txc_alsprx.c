/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.  
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* 
 * Author: Simon Hsueh <simonhsueh@txc.com.tw>
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
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h> 
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <hwmsensor.h>
#include "cust_alsps.h"
#include "alsps.h"
//#include <batch.h>
#include "mtk_txc_alsprx.h"
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif


#define TXC_ALSPRX_DEV_NAME "txc_alsprx"

#define ALSPRX_DRIVER_VERSION_H "1.0.0"
#define ALSPRX_DRIVER_VERSION_C "1.0.0"
#define PA22A_PRX_ENABLE 0x02
#define PA22A_ALS_ENABLE 0x01
#define PA22A_CFG2_ADDR 0x02
/*----------------------------------------------------------------------------*/
// static struct alsps_hw alsps_cust;
// static struct alsps_hw *hw = &alsps_cust;
/* For alsp driver get cust info */
// struct alsps_hw *get_cust_alsps(void){    
 //    return &alsps_cust;
// }
static int  txc_alsprx_init_flag = -1;
static int  txc_alsprx_local_init(void);
static int  txc_alsprx_local_uninit(void);
static struct alsps_init_info txc_alsprx_init_info = {        
    .name = "txc_alsprx",        
    .init = txc_alsprx_local_init,        
    .uninit = txc_alsprx_local_uninit,
};
/*----------------------------------------------------------------------------*/
static struct i2c_client *txc_alsprx_i2c_client = NULL;
static struct txc_alsprx_priv *txc_alsprx_obj = NULL;
struct platform_device *alspsPltFmDev;

static int txc_alsprx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int txc_alsprx_i2c_remove(struct i2c_client *client);
static int txc_alsprx_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int txc_alsprx_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int txc_alsprx_i2c_resume(struct i2c_client *client);
static void txc_alsprx_eint_func(void);

/*----------------------------------------------------------------------------*/
#if 1
int i2c_read_byte(uint8_t reg, uint8_t* byte_val);
int i2c_read_buf(uint8_t reg, uint8_t byteCnt, uint8_t* buf);
int i2c_write_byte(uint8_t reg, uint8_t byte_val);
void ms_delay(uint32_t ms);
#endif
alsprx_interface *txc_alsprx_if = {0};
/*===========================================================================
						FUNCTION      Sensor Device functions defintion start
 ===========================================================================*/
int alsprx_set_reg_default(alsprx_state* state, alsprx_nvdb *nvdb)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_set_reg_default) {
		return txc_alsprx_if->alsprx_set_reg_default(state,nvdb);
        printk("[wjwind] %s \n",__FUNCTION__);
	} else {
		return 0;
	}
}
int alsprx_enable_als(alsprx_state* state,int enable)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_enable_als) {
		return txc_alsprx_if->alsprx_enable_als(state,enable);
	} else {
		return 0;
	}
}
int alsprx_enable_ps(alsprx_state* state,int enable)
{
    if(txc_alsprx_if && txc_alsprx_if->alsprx_enable_ps) {
        return txc_alsprx_if->alsprx_enable_ps(state,enable);
    } else {
        return 0;
    }
}

int alsprx_als_calibration(alsprx_state* state,alsprx_nvdb* nvdb)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_als_calibration) {
		return txc_alsprx_if->alsprx_als_calibration(state,nvdb);
	} else {
		return 0;
	}
}
int alsprx_ps_calibration(alsprx_state* state,alsprx_nvdb* nvdb)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_ps_calibration) {
		return txc_alsprx_if->alsprx_ps_calibration(state,nvdb);
	} else {
		return 0;
	}
}
int alsprx_process_als_data(alsprx_state* state)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_process_als_data) {
		return txc_alsprx_if->alsprx_process_als_data(state);
	} else {
		return 0;
	}
}
int alsprx_process_ps_data(alsprx_state* state)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_process_ps_data) {
		return txc_alsprx_if->alsprx_process_ps_data(state);
	} else {
		return 0;
	}
}
int alsprx_read_als(alsprx_state* state)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_read_als) {
		return txc_alsprx_if->alsprx_read_als(state);
	} else {
		return 0;
	}
}
int alsprx_read_ps(alsprx_state* state)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_read_ps) {
		return txc_alsprx_if->alsprx_read_ps(state);
	} else {
		return 0;
	}
}
int alsprx_show_registers(uint8_t* regs_arry, uint8_t arry_size)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_show_registers) {
		return txc_alsprx_if->alsprx_show_registers(regs_arry, arry_size);
	} else {
		return 0;
	}
}
//wangjun@wind-mobi.com 20170927 begin
int alsprx_set_ps_threshold(alsprx_state* state,uint16_t low_thres,uint16_t high_thres,uint16_t cross_talk)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_set_ps_threshold) {
		return txc_alsprx_if->alsprx_set_ps_threshold(state,low_thres,high_thres,cross_talk);
	} else {
		return 0;
	}
}
//wangjun@wind-mobi.com 20170927 end
int alsprx_set_register(uint8_t addr,uint8_t data)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_set_register) {
		return txc_alsprx_if->alsprx_set_register(addr,data);
	} else {
		return 0;
	}
}
int alsprx_get_deviceid(alsprx_state* state)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_get_deviceid) {
		return txc_alsprx_if->alsprx_get_deviceid(state);
	} else {
		return 0;
	}
}
int alsprx_get_vendorid(alsprx_state* state)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_get_vendorid) {
		return txc_alsprx_if->alsprx_get_vendorid(state);
	} else {
		return 0;
	}
}
device_information alsprx_get_device_info(sensor_type_e type)
{
	device_information dev_info;

	if(txc_alsprx_if && txc_alsprx_if->alsprx_get_device_info) {
		return txc_alsprx_if->alsprx_get_device_info(type);
	} else {
		return dev_info;
	}
}
int alsprx_enable_interrupt(sensor_type_e type, alsprx_state* state, int enable)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_enable_interrupt) {
		return txc_alsprx_if->alsprx_enable_interrupt(type,state,enable);
	} else {
		return 0;
	}
}
int alsprx_clear_interrupt(sensor_type_e type)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_clear_interrupt) {
		return txc_alsprx_if->alsprx_clear_interrupt(type);
	} else {
		return 0;
	}
}
int is_alsprx_enabled(sensor_type_e type)
{
	if(txc_alsprx_if && txc_alsprx_if->is_alsprx_enabled) {
		return txc_alsprx_if->is_alsprx_enabled(type);
	} else {
		return 0;
	}
}
int alsprx_get_default_nv(alsprx_nvdb* nvdb)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_get_default_nv) {
		return txc_alsprx_if->alsprx_get_default_nv(nvdb);
	} else {
		return 0;
	}
}
int alsprx_get_registers_count(void)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_get_registers_count) {
		return txc_alsprx_if->alsprx_get_registers_count();
	} else {
		return 0;
	}
}
int alsprx_ps_calibrate_specific(alsprx_nvdb* nvdb, sensor_calibration_type_e cal_type)
{
	if(txc_alsprx_if && txc_alsprx_if->alsprx_ps_calibrate_specific) {
		return txc_alsprx_if->alsprx_ps_calibrate_specific(nvdb,cal_type);
	} else {
		return 0;
	}
}
/*===========================================================================
	 					FUNCTION      Sensor Device functions defintion end
 ===========================================================================*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id txc_alsprx_i2c_id[] = {{TXC_ALSPRX_DEV_NAME,0},{}};
/*----------------------------------------------------------------------------*/
struct txc_alsprx_priv {
    /* MTK Platform */
    struct alsps_hw  hw;
    struct i2c_client *client;
    struct work_struct    eint_work;

#ifdef CUSTOM_KERNEL_SENSORHUB
	struct work_struct init_done_work;
#endif

    /* misc */    
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;    /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;        /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;    
    atomic_t    init_done;
    struct      device_node *irq_node;
    int         irq;
    atomic_t    als_cmd_val;        /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;            /*the cmd value can't be read, stored in ram*/
    atomic_t    als_thd_val_high;    /*the cmd value can't be read, stored in ram*/
    atomic_t    als_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;				/*enable mask*/
    ulong       pending_intr;		/*pending interrupt*/
    atomic_t	trace;
	/* data */
    u16     als_level_num;
    u16     als_value_num;
    u32     als_level[C_CUST_ALS_LEVEL-1];
    u32     als_value[C_CUST_ALS_LEVEL];
    alsprx_nvdb 	txc_nv;
   	alsprx_state 	txc_state;
    u8      load_nv_flag;
    /* Mutex */
	struct mutex	update_lock;
#if defined(CONFIG_GN_BSP_PS_CALIBRATE_INCALL)
    uint16_t ps_thd_val_max;
#endif

    /* early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     

};
/* ------------------ Platform Interface Function ------------------ */
int i2c_read_byte(uint8_t reg, uint8_t* byte_val)
{
    int i,res = 0;
    mutex_lock(&txc_alsprx_obj->update_lock);
    
    for ( i=0; i < 3 ; i++)
    {
        res = hwmsen_read_byte((txc_alsprx_obj->client),(u8)reg,(u8* )byte_val);
        if (res >= 0)
            break;
    }
    mutex_unlock(&txc_alsprx_obj->update_lock);
    return res;
}
int i2c_read_buf(uint8_t reg, uint8_t byteCnt, uint8_t* buf)
{
    int i,res = 0;
    mutex_lock(&txc_alsprx_obj->update_lock);
    for ( i=0; i < 3 ; i++) {
        res = hwmsen_read_block((txc_alsprx_obj->client),(u8)(reg),(u8*)(buf),(u8)byteCnt);
        if (res >= 0)
            break;
    }
    mutex_unlock(&txc_alsprx_obj->update_lock);    
    
    return res;
}
int i2c_write_byte(uint8_t reg, uint8_t byte_val)
{
    int res = 0;
    mutex_lock(&txc_alsprx_obj->update_lock);
    
    res = hwmsen_write_byte(txc_alsprx_obj->client,(u8)reg,(u8)byte_val);
    
    mutex_unlock(&txc_alsprx_obj->update_lock);
    return res;
}
void ms_delay(uint32_t ms)
{
    msleep(ms);
    return;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
    {.compatible = "mediatek,alsps"},    
    {},
};
#endif

static struct i2c_driver txc_alsprx_i2c_driver = {
.driver = {
    .name = TXC_ALSPRX_DEV_NAME,
#ifdef CONFIG_OF    
    .of_match_table = alsps_of_match,
#endif
    },
    .probe      = txc_alsprx_i2c_probe,
    .remove     = txc_alsprx_i2c_remove,
    .detect     = txc_alsprx_i2c_detect,
    .suspend    = txc_alsprx_i2c_suspend,
    .resume     = txc_alsprx_i2c_resume,
    .id_table   = txc_alsprx_i2c_id,
};


/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS    = 2,
}CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
    CMC_TRC_ALS_DATA= 0x0001,
    CMC_TRC_PS_DATA = 0x0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;

/*-------------------------------attribute file for debugging----------------------------------*/

static int txc_alsprx_read_file(char *filename,uint8_t* param) 
{
	struct file  *fop;
	mm_segment_t old_fs;

	fop = filp_open(filename,O_RDONLY,0);
	if(IS_ERR(fop))
	{
		APS_LOG("Filp_open error!! Path = %s\n",filename);
		return -1;
	}

	old_fs = get_fs();  
	set_fs(get_ds()); //set_fs(KERNEL_DS);  
	     
	fop->f_op->llseek(fop,0,0);
	fop->f_op->read(fop, param, NVFILE_MAX_SIZE, &fop->f_pos);     

	set_fs(old_fs);  

	filp_close(fop,NULL);

	return 0;

}
static int txc_alsprx_write_file(char *filename,uint8_t* param) 
{
	struct file  *fop;
	mm_segment_t old_fs;	 

	fop = filp_open(filename,O_CREAT | O_RDWR,0666);
	if(IS_ERR(fop))
	{
		APS_LOG("Create file error!! Path = %s\n",filename);       
		return -1;
	}    
	old_fs = get_fs();  
	set_fs(get_ds()); //set_fs(KERNEL_DS);  
	fop->f_op->write(fop, (char *)param, sizeof(param) * NVFILE_MAX_SIZE, &fop->f_pos);       
	set_fs(old_fs);  

	filp_close(fop,NULL);

	return 0;
}
int txc_alsprx_save_nv(alsprx_nvdb* nvdb)
{
    int i = 0,index=0;
    uint8_t bufftemp[NVFILE_MAX_SIZE] = {0};
    if ( nvdb->nv_size <= 0 )
        APS_LOG("No Calibration! Save NV fail\n");
    bufftemp[index++] = (uint8_t)nvdb->nv_size;
    bufftemp[index++] = (uint8_t)nvdb->version_num ;    
    bufftemp[index++] = (uint8_t)nvdb->Reg0;
    bufftemp[index++] = (uint8_t)nvdb->Reg1;
    bufftemp[index++] = (uint8_t)nvdb->Reg2;
    bufftemp[index++] = (uint8_t)nvdb->Reg3;
    bufftemp[index++] = (uint8_t)nvdb->Reg11;
    bufftemp[index++] = (uint8_t)nvdb->Reg12;
    bufftemp[index++] = (uint8_t)nvdb->Reg13;
    bufftemp[index++] = (uint8_t)nvdb->prxXtalk;
    bufftemp[index++] = (uint8_t)(nvdb->als_caliLUX & 0xFF);
    bufftemp[index++] = (uint8_t)((nvdb->als_caliLUX >> 8) & 0xFF);
	bufftemp[index++] = (uint8_t)((nvdb->als_caliLUX >> 16) & 0xFF);
	bufftemp[index++] = (uint8_t)((nvdb->als_caliLUX >> 24) & 0xFF);
	for(i=0; i<4; i++)
	{
		bufftemp[index++] = (uint8_t)((nvdb->als_caliADC_IR[i]) & 0xFF);
		bufftemp[index++] = (uint8_t)((nvdb->als_caliADC_IR[i] >> 8) & 0xFF);
		bufftemp[index++] = (uint8_t)((nvdb->als_caliADC_IR[i] >> 16) & 0xFF);
		bufftemp[index++] = (uint8_t)((nvdb->als_caliADC_IR[i] >> 24) & 0xFF);
	}
	for(i=0; i<4; i++)
	{	
		bufftemp[index++] = (uint8_t)(nvdb->als_caliADC[i] & 0xFF );
		bufftemp[index++] = (uint8_t)((nvdb->als_caliADC[i] >> 8) & 0xFF);
		bufftemp[index++] = (uint8_t)((nvdb->als_caliADC[i] >> 16) & 0xFF);
		bufftemp[index++] = (uint8_t)((nvdb->als_caliADC[i] >> 24) & 0xFF);
	}
    
    if (txc_alsprx_write_file(NV_FILE_PATH,bufftemp) < 0 ) {
        APS_LOG("Create/Write NV file error!!\n");
        return -1;
    } else {
        APS_LOG("Write NV file Success!!\n");
    }

    return 0;
}
int txc_alsprx_get_nv( alsprx_state* state, alsprx_nvdb* nvdb)
{
    uint8_t bufftemp[NVFILE_MAX_SIZE] = {0};
    int i=0 , index=0;
    if(txc_alsprx_read_file(NV_FILE_PATH,bufftemp) < 0 ) {
        APS_LOG("Get NV Fail!!\n");  
        return 0;
    } else {
        nvdb->nv_size =     (uint8_t)bufftemp[index++];
        nvdb->version_num = (uint8_t)bufftemp[index++];
        
        nvdb->Reg0 =       (uint16_t)bufftemp[index++];
        nvdb->Reg1 =       (uint16_t)bufftemp[index++];
        nvdb->Reg2 =       (uint16_t)bufftemp[index++];
        nvdb->Reg3 =       (uint16_t)bufftemp[index++];
        nvdb->Reg11 =      (uint16_t)bufftemp[index++];
        nvdb->Reg12 =      (uint32_t)bufftemp[index++];
        nvdb->Reg13 =      (uint32_t)bufftemp[index++];
        nvdb->prxXtalk =   (uint32_t)bufftemp[index++];
        nvdb->als_caliLUX   =((uint32_t)(bufftemp[index++]));
		nvdb->als_caliLUX |=((uint32_t)(bufftemp[index++]<<8));
		nvdb->als_caliLUX |=((uint32_t)(bufftemp[index++]<<16));
		nvdb->als_caliLUX |=((uint32_t)(bufftemp[index++]<<24));
		
		for(i=0; i<4; i++)
		{
			nvdb->als_caliADC_IR[i] = (uint32_t)(bufftemp[index++]);
			nvdb->als_caliADC_IR[i] |= (uint32_t)(bufftemp[index++] << 8);
			nvdb->als_caliADC_IR[i] |= (uint32_t)(bufftemp[index++] << 16);
			nvdb->als_caliADC_IR[i] |= (uint32_t)(bufftemp[index++] << 24);
		}
		for(i=0; i<4; i++)
		{
			nvdb->als_caliADC[i] = (uint32_t)(bufftemp[index++]);
			nvdb->als_caliADC[i] |= (uint32_t)(bufftemp[index++] << 8);
			nvdb->als_caliADC[i] |= (uint32_t)(bufftemp[index++] << 16);
			nvdb->als_caliADC[i] |= (uint32_t)(bufftemp[index++] << 24);
		}
        APS_LOG("Get NV Success!!\n");  
    }
    
    state->als_db.als_caliLUX = nvdb->als_caliLUX;
    state->prx_db.ps_cross_talk = nvdb->prxXtalk;
    
    return 1;
}

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t txc_alsprx_show_version(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    
    if(!txc_alsprx_obj) {
        APS_ERR("txc_alsprx_obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, ".H Ver: %s\n.C Ver: %s\n",ALSPRX_DRIVER_VERSION_H,ALSPRX_DRIVER_VERSION_C); 
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_show_als(struct device_driver *ddri, char *buf)
{
    int res = 0;    
    alsprx_read_als(&txc_alsprx_obj->txc_state);
    if( res < 0)
        return snprintf(buf, PAGE_SIZE, "ERROR\n");    
    else
        return snprintf(buf, PAGE_SIZE, "ALS data:%d Lux:%d\n", txc_alsprx_obj->txc_state.als_db.als_data,
        														txc_alsprx_obj->txc_state.als_db.als_mlux);    
}
/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_show_ps(struct device_driver *ddri, char *buf)
{
    int res = 0;
    res = alsprx_read_ps(&txc_alsprx_obj->txc_state);
    if( res < 0 )
        return snprintf(buf, PAGE_SIZE, "ERROR\n");    
    else    
	//add by wangjun begin 
        return snprintf(buf, PAGE_SIZE, "%x\n", txc_alsprx_obj->txc_state.prx_db.prx_data);    
	//add by wangjun end
}
/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_show_reg(struct device_driver *ddri, char *buf)
{
    int i,res;
    uint8_t regdata;
    uint32_t count = 0;

	APS_FUN();

    for(i=0;i <= 0x20 ;i++) {
        res = i2c_read_byte(i,&regdata);
        if(res<0)        
           break;    
        else
            count+=sprintf(buf+count,"[%x] = (%x)\n",0x00+i,regdata);
    }  
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
    int addr, cmd;
    APS_FUN();
    if(2 != sscanf(buf, "%x %x", &addr, &cmd)) {
        APS_ERR("invalid format: '%s'\n", buf);
        return 0;
    }
    //count = txc_sensor_set_register(addr,cmd);    
    ALSPRX_MSG_2(HIGH,"REG[0x%x]:0x%x",addr,cmd);
	alsprx_set_register(addr, cmd);
    return count;
}

/*---Offset At-------------------------------------------------------------------------*/
static ssize_t txc_alsprx_show_ps_cal(struct device_driver *ddri, char *buf)
{
    if(!txc_alsprx_obj) {
        APS_ERR("txc_alsprx_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", txc_alsprx_obj->txc_nv.prxXtalk);     
}
static ssize_t txc_alsprx_show_als_cal(struct device_driver *ddri, char *buf)
{
    if(!txc_alsprx_obj) {
        APS_ERR("txc_alsprx_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "LUX:%d\nADC_IR[0]:%d ADC_IR[1]:%d ADC_IR[2]:%d ADC_IR[3]:%d\nADC[0]:%d ADC[1]:%d ADC[2]:%d ADC[3]:%d\n", 
											txc_alsprx_obj->txc_nv.als_caliLUX,
											txc_alsprx_obj->txc_nv.als_caliADC_IR[0],txc_alsprx_obj->txc_nv.als_caliADC_IR[1],
											txc_alsprx_obj->txc_nv.als_caliADC_IR[2],txc_alsprx_obj->txc_nv.als_caliADC_IR[3],
											txc_alsprx_obj->txc_nv.als_caliADC[0],txc_alsprx_obj->txc_nv.als_caliADC[1],
											txc_alsprx_obj->txc_nv.als_caliADC[2],txc_alsprx_obj->txc_nv.als_caliADC[3]);     
}
/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_set_ps_cal(struct device_driver *ddri, const char *buf, size_t count)
{
    int ret;
    ret = alsprx_ps_calibration(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv);
    txc_alsprx_save_nv(&txc_alsprx_obj->txc_nv);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_set_als_cal(struct device_driver *ddri, const char *buf, size_t count)
{
    int ret;
    uint32_t cal_lux = 0;
    if(1 != sscanf(buf, "%d", &cal_lux)) {
        APS_ERR("invalid format: '%s'\n", buf);
        return -1;
    }
    txc_alsprx_obj->txc_nv.als_caliLUX = cal_lux;
    APS_LOG("Ambient Light Start Calibarte at %d lux\n",txc_alsprx_obj->txc_nv.als_caliLUX);
    ret = alsprx_als_calibration(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv);
    txc_alsprx_save_nv(&txc_alsprx_obj->txc_nv);
    return count;
}

static ssize_t txc_alsprx_store_dev_init(struct device_driver *ddri, const char *buf, size_t count)
{
    int ret;
    ret = alsprx_set_reg_default(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv);
    return count;
}

static ssize_t txc_alsprx_show_enable_ps(struct device_driver *ddri, char *buf)
{

    if ((txc_alsprx_obj->txc_state.enable_mode & 0x02 ) == PA22A_PRX_ENABLE) {	
        return snprintf(buf, PAGE_SIZE, "Enable Proximity Sensor !\n");
	}
    else {
        return snprintf(buf, PAGE_SIZE, "Disable Proximity Sensor !\n");
	}
	return 0; 

}
/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_set_enable_ps(struct device_driver *ddri, const char *buf, size_t count)
{
    int enable,res;
    APS_FUN();
    if(1 != sscanf(buf, "%x", &enable)) {
        APS_ERR("invalid format: '%s'\n", buf);
        return 0;
    }
    /* Load NV Item */
    if ( txc_alsprx_obj->load_nv_flag == 0 ) {
        if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
            alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
            txc_alsprx_obj->load_nv_flag = 1;
        }
    }
    
    res = alsprx_enable_ps(&txc_alsprx_obj->txc_state,enable);    
    if (enable) {
        atomic_set(&txc_alsprx_obj->ps_deb_on, 1);
		atomic_set(&txc_alsprx_obj->ps_deb_end, jiffies+atomic_read(&txc_alsprx_obj->ps_debounce)/(1000/HZ));
    }
    else
        atomic_set(&txc_alsprx_obj->ps_deb_on, 0);
	return count;
}
static ssize_t txc_alsprx_show_enable_als(struct device_driver *ddri, char *buf)
{
    if ((txc_alsprx_obj->txc_state.enable_mode & 0x01 ) == PA22A_ALS_ENABLE ) {
        return snprintf(buf, PAGE_SIZE, "Enable Ambient Light Sensor !\n");
	}
    else {
        return snprintf(buf, PAGE_SIZE, "Disable Ambient Light Sensor !\n");
	}
	return 0; 	

}
/*----------------------------------------------------------------------------*/
static ssize_t txc_alsprx_set_enable_als(struct device_driver *ddri, const char *buf, size_t count)
{
    int enable,res;
    APS_FUN();
    if(1 != sscanf(buf, "%x", &enable)) {
        APS_ERR("invalid format: '%s'\n", buf);
        return 0;
    }      
    /* Load NV Item */
    if ( txc_alsprx_obj->load_nv_flag == 0 ) {
        if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
            alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
            txc_alsprx_obj->load_nv_flag = 1;
        }
    }    
    res = alsprx_enable_als(&txc_alsprx_obj->txc_state,enable);    
    if (enable) {        
        atomic_set(&txc_alsprx_obj->als_deb_on, 1);
        atomic_set(&txc_alsprx_obj->als_deb_end, jiffies+atomic_read(&txc_alsprx_obj->als_debounce)/(1000/HZ));
    }
    else
        atomic_set(&txc_alsprx_obj->als_deb_on, 0);
	  return count;
}
static ssize_t txc_alsprx_show_nv_parm(struct device_driver *ddri, char *buf)
{
    txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv);
    return 0; 	

}
static DRIVER_ATTR(version,     S_IWUSR | S_IRUGO, txc_alsprx_show_version, NULL);
static DRIVER_ATTR(psdata,      S_IWUSR | S_IRUGO, txc_alsprx_show_ps, NULL);
static DRIVER_ATTR(alsdata,     S_IWUSR | S_IRUGO, txc_alsprx_show_als, NULL);
static DRIVER_ATTR(enable_ps,   S_IWUSR | S_IRUGO, txc_alsprx_show_enable_ps, txc_alsprx_set_enable_ps);
static DRIVER_ATTR(enable_als,  S_IWUSR | S_IRUGO, txc_alsprx_show_enable_als, txc_alsprx_set_enable_als);
static DRIVER_ATTR(reg,         S_IWUSR | S_IRUGO, txc_alsprx_show_reg, txc_alsprx_store_reg);
static DRIVER_ATTR(pscalibration, S_IWUSR | S_IRUGO, txc_alsprx_show_ps_cal,txc_alsprx_set_ps_cal);
static DRIVER_ATTR(alscalibration, S_IWUSR | S_IRUGO, txc_alsprx_show_als_cal,txc_alsprx_set_als_cal);
static DRIVER_ATTR(show_nv,     S_IWUSR | S_IRUGO, txc_alsprx_show_nv_parm,NULL);
static DRIVER_ATTR(dev_init,    S_IWUSR | S_IRUGO, NULL, txc_alsprx_store_dev_init);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *txc_alsprx_attr_list[] = {
    &driver_attr_version,
    &driver_attr_psdata,
    &driver_attr_alsdata,
    &driver_attr_enable_ps,
    &driver_attr_enable_als,
    &driver_attr_reg,
    &driver_attr_pscalibration,
    &driver_attr_alscalibration,
    &driver_attr_show_nv,
    &driver_attr_dev_init,    
};

/*----------------------------------------------------------------------------*/
static int txc_alsprx_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(txc_alsprx_attr_list)/sizeof(txc_alsprx_attr_list[0]));
    if (driver == NULL)    
        return -EINVAL;

    for(idx = 0; idx < num; idx++) {
        if((err = driver_create_file(driver, txc_alsprx_attr_list[idx]))) {            
            APS_ERR("driver_create_file (%s) = %d\n", txc_alsprx_attr_list[idx]->attr.name, err);
            break;
        }
    }
    
    return err;
}
/*----------------------------------------------------------------------------*/
static int txc_alsprx_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(txc_alsprx_attr_list)/sizeof(txc_alsprx_attr_list[0]));

    if (!driver)
    return -EINVAL;

    for (idx = 0; idx < num; idx++)     
        driver_remove_file(driver, txc_alsprx_attr_list[idx]);    
    
    return err;
}

/*----------------------------------------------------------------------------*/
static void txc_alsprx_eint_work(struct work_struct *work)
{
	struct txc_alsprx_priv *obj = (struct txc_alsprx_priv *)container_of(work, struct txc_alsprx_priv, eint_work);

#ifdef CUSTOM_KERNEL_SENSORHUB
	int res = 0;
	
	res = ps_report_interrupt_data(obj->txc_state.prx_db.event);
	if (res != 0)
		APS_ERR("TXC_ALSPRX_eint_work err: %d\n", res);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */

    struct hwm_sensor_data sensor_data;
    int res = 0;
    uint8_t regdata = 0;
    int als_intr_active = 0, ps_intr_active = 0;
    memset(&sensor_data, 0, sizeof(sensor_data));

    /* Read interrput flag */
    i2c_read_byte(PA22A_CFG2_ADDR,&regdata);
    als_intr_active = regdata  & 0x01;
    ps_intr_active = regdata & 0x02;
    
    APS_FUN();
    if(!(obj->hw.polling_mode_ps) && ps_intr_active) {

        // Process PS data
        res = alsprx_process_ps_data((void*)&obj->txc_state);
    
        if(res != 0){
            goto EXIT_INTR_ERR;
        } else {
            sensor_data.values[0] = obj->txc_state.prx_db.event;
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;    
        }        

        if(ps_report_interrupt_data(sensor_data.values[0])) {    
            APS_ERR("call ps_report_interrupt_data fail\n");
        }         
    }
    
    enable_irq(obj->irq);
    return;
#endif 
EXIT_INTR_ERR:
	enable_irq(obj->irq);

    APS_ERR("txc_alsprx_eint_work err: %d\n", res);
}

/*----------------------------------------------------------------------------*/

#ifdef CUSTOM_KERNEL_SENSORHUB
static void txc_alsprx_init_done_work(struct work_struct *work)
{
	struct txc_alsprx_priv *obj = (struct txc_alsprx_priv *)container_of(work, struct txc_alsprx_priv, init_done_work);
	union TXC_ALSPRX_CUST_DATA *p_cust_data;
	SCP_SENSOR_HUB_DATA data;
	int max_cust_data_size_per_packet;
	int i;
	uint sizeOfCustData;
	uint len;
	char *p = (char *)obj->hw;

	APS_FUN();

	p_cust_data = (union TXC_ALSPRX_CUST_DATA *)data.set_cust_req.custData;
	sizeOfCustData = sizeof(*(obj->hw));
	max_cust_data_size_per_packet = sizeof(data.set_cust_req.custData) - offsetof(struct TXC_ALSPRX_SET_CUST, data);

	for (i = 0; sizeOfCustData > 0; i++) {
		data.set_cust_req.sensorType = ID_PROXIMITY;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		p_cust_data->setCust.action = TXC_ALSPRX_CUST_ACTION_SET_CUST;
		p_cust_data->setCust.part = i;

		if (sizeOfCustData > max_cust_data_size_per_packet)
			len = max_cust_data_size_per_packet;
		else
			len = sizeOfCustData;

		memcpy(p_cust_data->setCust.data, p, len);
		sizeOfCustData -= len;
		p += len;

		len += offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(struct TXC_ALSPRX_SET_CUST, data);
		SCP_sensorHub_req_send(&data, &len, 1);
	}
/*
	data.set_cust_req.sensorType = ID_PROXIMITY;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	p_cust_data->setEintInfo.action = TXC_ALSPRX_CUST_ACTION_SET_EINT_INFO;
	p_cust_data->setEintInfo.gpio_mode = GPIO_ALS_EINT_PIN_M_EINT;
	p_cust_data->setEintInfo.gpio_pin = GPIO_ALS_EINT_PIN;
	p_cust_data->setEintInfo.eint_num = CUST_EINT_ALS_NUM;
	p_cust_data->setEintInfo.eint_is_deb_en = CUST_EINT_ALS_DEBOUNCE_EN;
	p_cust_data->setEintInfo.eint_type = CUST_EINT_ALS_TYPE;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(p_cust_data->setEintInfo);
	SCP_sensorHub_req_send(&data, &len, 1);
*/
	data.activate_req.sensorType = ID_PROXIMITY;
	data.activate_req.action = SENSOR_HUB_ACTIVATE;
	if (test_bit(CMC_BIT_PS, &obj->enable))
		data.activate_req.enable = 1;
	else
		data.activate_req.enable = 0;

	len = sizeof(data.activate_req);
	SCP_sensorHub_req_send(&data, &len, 1);
	data.activate_req.sensorType = ID_LIGHT;
	if (test_bit(CMC_BIT_ALS, &obj->enable))
		data.activate_req.enable = 1;
	else
		data.activate_req.enable = 0;

	len = sizeof(data.activate_req);
	SCP_sensorHub_req_send(&data, &len, 1);

	atomic_set(&obj->init_done,  1);
}
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

#ifdef CUSTOM_KERNEL_SENSORHUB
static int txc_alsprx_irq_handler(void *data, uint len)
{
	struct txc_alsprx_priv *obj = txc_alsprx_obj;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P)data;
	if(!txc_alsprx_obj) {
        APS_ERR("txc sensor is null!!\n");
        return -1;
    }
    
	APS_ERR("len = %d, type = %d, action = %d, errCode = %d\n",
		len, rsp->rsp.sensorType, rsp->rsp.action, rsp->rsp.errCode);

	switch (rsp->rsp.action) {
	case SENSOR_HUB_NOTIFY:
		switch (rsp->notify_rsp.event) {
		case SCP_INIT_DONE:
			schedule_work(&obj->init_done_work);
			/* schedule_delayed_work(&obj->init_done_work, HZ); */
			break;
		case SCP_NOTIFY:
			if (TXC_ALSPRX_NOTIFY_PROXIMITY_CHANGE == rsp->notify_rsp.data.int32_Data[0]) {
				obj->txc_state.prx_db.event = rsp->notify_rsp.data.int32_Data[1];
				txc_alsprx_eint_func();
			} else
				APS_ERR("Unknown notify");
			break;
		default:
			APS_ERR("Error sensor hub notify");
			break;
		}
		break;
	default:
		APS_ERR("Error sensor hub action");
		break;
	}

	return 0;
}
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

static void txc_alsprx_eint_func(void)
{   
    schedule_work(&txc_alsprx_obj->eint_work);
}

#ifndef CUSTOM_KERNEL_SENSORHUB
static irqreturn_t txc_alsprx_eint_handler(int irq, void *desc)
{
    printk("txc_alsprx_eint_handle\n");
    txc_alsprx_eint_func();
    disable_irq_nosync(txc_alsprx_obj->irq);

    return IRQ_HANDLED;
}
#endif

int txc_alsprx_setup_eint(struct i2c_client *client)
{

#ifdef CUSTOM_KERNEL_SENSORHUB
	int err = 0;
	
	err = SCP_sensorHub_rsp_registration(ID_PROXIMITY, txc_alsprx_irq_handler);
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */

    int ret;
    u32 ints[2] = {0, 0};
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_default;
    struct pinctrl_state *pins_cfg;
    APS_FUN();
    /*configure to GPIO function, external interrupt*/

   alspsPltFmDev = get_alsps_platformdev();
   txc_alsprx_obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
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
    if (txc_alsprx_obj->irq_node) {
        of_property_read_u32_array(txc_alsprx_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_request(ints[0], "p-sensor");
        gpio_set_debounce(ints[0], ints[1]);
        pinctrl_select_state(pinctrl, pins_cfg);
        ALSPRX_MSG_2(LOW,"ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        txc_alsprx_obj->irq = irq_of_parse_and_map(txc_alsprx_obj->irq_node, 0);
        ALSPRX_MSG_1(LOW,"txc_alsprx_obj->irq = %d\n", txc_alsprx_obj->irq);
        if (!txc_alsprx_obj->irq) {
            APS_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        if (request_irq(txc_alsprx_obj->irq, txc_alsprx_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
            APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
        enable_irq(txc_alsprx_obj->irq);
    } else {
        APS_ERR("null irq node!!\n");
        return -EINVAL;
    }

#endif
    return 0;
}
/*-------------------------------MISC device related------------------------------------------*/

/************************************************************/
int txc_alsprx_open(struct inode *inode, struct file *file)
{
    file->private_data = txc_alsprx_i2c_client;

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}
/************************************************************/

int txc_alsprx_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/************************************************************/
long txc_alsprx_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct txc_alsprx_priv *obj = i2c_get_clientdata(client);  
    long err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    //int ps_result;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	union TXC_ALSPRX_CUST_DATA *pCustData;
	int len;

	data.set_cust_req.sensorType = ID_PROXIMITY;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData = (union TXC_ALSPRX_CUST_DATA *)(&data.set_cust_req.custData);
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */    
    switch (cmd)
    {

        case ALSPS_SET_PS_MODE:
        
            if(copy_from_user(&enable, ptr, sizeof(enable))) {
                err = -EFAULT;
                goto err_out;
            }
            /* Load NV Item */
            if ( txc_alsprx_obj->load_nv_flag == 0 ) {
                if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
                    alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
                    txc_alsprx_obj->load_nv_flag = 1;
                }
            }            
            err = alsprx_enable_ps(&obj->txc_state,enable);
            if (err < 0) {
                APS_ERR("enable ps fail: %ld\n", err); 
                goto err_out;
            }
                
            if(enable)
                set_bit(CMC_BIT_PS, &obj->enable);
            else                
                clear_bit(CMC_BIT_PS, &obj->enable);
            
        break;

        case ALSPS_GET_PS_MODE:
            /* Load NV Item */
            if ( txc_alsprx_obj->load_nv_flag == 0 ) {
                if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
                    alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
                    txc_alsprx_obj->load_nv_flag = 1;
                }
            }            
            enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable))) {
                err = -EFAULT;
                goto err_out;
            }
        break;

        case ALSPS_GET_PS_DATA:    

            if((err = alsprx_read_ps(&obj->txc_state))) {
                goto err_out;
            }        
            if((err = alsprx_process_ps_data(&obj->txc_state))) {
                goto err_out;
            }

            dat = obj->txc_state.prx_db.event;
            if(copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            
        break;

        case ALSPS_GET_PS_RAW_DATA:    
        
            if((err = alsprx_read_ps(&obj->txc_state))) {
                goto err_out;
            }

            dat = obj->txc_state.prx_db.prx_data;
            if(copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            
        break;              

        case ALSPS_SET_ALS_MODE:

            if(copy_from_user(&enable, ptr, sizeof(enable))) {
                err = -EFAULT;
                goto err_out;
            }            
            /* Load NV Item */
            if ( txc_alsprx_obj->load_nv_flag == 0 ) {
                if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
                    alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
                    txc_alsprx_obj->load_nv_flag = 1;
                }
            }            
            
            err = alsprx_enable_als((void*)&obj->txc_state,enable);
            if (err < 0) {
                APS_ERR("enable ps fail: %ld\n", err); 
                goto err_out;
            }
                
            if(enable)
                set_bit(CMC_BIT_ALS, &obj->enable);
            else                
                clear_bit(CMC_BIT_ALS, &obj->enable);            
            
        break;

        case ALSPS_GET_ALS_MODE:
        
            enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable))) {
                err = -EFAULT;
                goto err_out;
            }
            
        break;

        case ALSPS_GET_ALS_DATA: 
                
            alsprx_read_als(&obj->txc_state);
            dat = obj->txc_state.als_db.als_mlux;
            if(copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            
        break;

        case ALSPS_GET_ALS_RAW_DATA:    
        
            if((err = alsprx_read_als(&obj->txc_state))) {
                goto err_out;
            }            
            dat = obj->txc_state.als_db.als_data;
            if(copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            
        break;
            
#if !defined(MT6589) && !defined(MT6572)
        case ALSPS_IOCTL_CLR_CALI:
        
            APS_ERR("%s ALSPS_IOCTL_CLR_CALI\n", __func__);
            if(copy_from_user(&dat, ptr, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
#ifdef CUSTOM_KERNEL_SENSORHUB
			pCustData->clearCali.action = TXC_ALSPRX_CUST_ACTION_CLR_CALI;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->clearCali);
			err = SCP_sensorHub_req_send(&data, &len, 1);
#endif
        break;

        case ALSPS_IOCTL_GET_CALI:
        
            dat = obj->txc_nv.prxXtalk;
            APS_ERR("%s set ps_cali %x\n", __func__, dat);
            if(copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            
        break;

/*        case ALSPS_IOCTL_SET_CALI:
            if((err = alsprx_ps_calibration(&txc_alsprx_obj->txc_state,&obj->txc_nv))) {
                goto err_out;
            }            
            txc_alsprx_save_nv(&obj->txc_nv);
#ifdef CUSTOM_KERNEL_SENSORHUB
			pCustData->setCali.action 				= TXC_ALSPRX_CUST_ACTION_SET_CALI;
			pCustData->setCali.Reg0					= (uint8_t)obj->txc_nv.Reg0;
			pCustData->setCali.Reg1 				= (uint8_t)obj->txc_nv.Reg1;
			pCustData->setCali.Reg2 				= (uint8_t)obj->txc_nv.Reg2;
			pCustData->setCali.Reg3 				= (uint8_t)obj->txc_nv.Reg3;
			pCustData->setCali.Reg11				= (uint8_t)obj->txc_nv.Reg11;
			pCustData->setCali.Reg12 				= (uint8_t)obj->txc_nv.Reg12;
			pCustData->setCali.Reg13 				= (uint8_t)obj->txc_nv.Reg13;
			pCustData->setCali.prxXtalk 			= (uint8_t)obj->txc_nv.prxXtalk;
			pCustData->setCali.prxXtalk_base 		= (uint8_t)obj->txc_nv.prxXtalk_base;
			pCustData->setCali.ps_high_threshold 	= (uint8_t)obj->txc_nv.ps_high_threshold;
			pCustData->setCali.ps_low_threshold 	= (uint8_t)obj->txc_nv.ps_low_threshold;
			pCustData->setCali.als_caliADC 			= obj->txc_nv.als_caliADC;
			pCustData->setCali.als_caliADC_IR 		= obj->txc_nv.als_caliADC_IR;
			pCustData->setCali.als_caliLUX 			= obj->txc_nv.als_caliLUX;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);
			err = SCP_sensorHub_req_send(&data, &len, 1);
#endif
            APS_ERR("%s set ps_cali %x\n", __func__, obj->txc_nv.prxXtalk); 
        break;
*/
#endif
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
        return err;    
}

/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations txc_alsprx_fops = {
    .owner = THIS_MODULE,
    .open = txc_alsprx_open,
    .release = txc_alsprx_release,
    .unlocked_ioctl = txc_alsprx_unlocked_ioctl,
};

static struct miscdevice txc_alsprx_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &txc_alsprx_fops,
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
/*--------------------------------------------------------------------------------------*/
void txc_alsprx_early_suspend(struct early_suspend *h)
{
    struct txc_alsprx_priv *obj = container_of(h, struct txc_alsprx_priv, early_drv);    
    int err;
    APS_FUN();      

    if(!obj) {
        APS_ERR("null pointer!!\n");
        return;
    }    
    
    atomic_set(&obj->als_suspend, 1);
    if((err = alsprx_enable_als(&obj->txc_state,0)))    
        APS_ERR("disable als fail: %d\n", err);    
}

void txc_alsprx_late_resume(struct early_suspend *h) 
{
    /*early_suspend is only applied for ALS*/
    struct txc_alsprx_priv *obj = container_of(h, struct txc_alsprx_priv, early_drv);          
    int err;
    APS_FUN();      

    if(!obj) {
        APS_ERR("null pointer!!\n");
        return;
    }
    atomic_set(&obj->als_suspend, 0);
    if ( txc_alsprx_obj->load_nv_flag == 0 ) {
        if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
            alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
            txc_alsprx_obj->load_nv_flag = 1;
        }
    }    
    if(test_bit(CMC_BIT_ALS, &obj->enable)) {
        if((err = alsprx_enable_als(&obj->txc_state,1)))
            APS_ERR("enable als fail: %d\n", err);          
    }
}
#endif
/* ---------------------------------- MTK Function ---------------------------------- */
int als_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

int als_enable_nodata(int en)
{
	int res = 0;

#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

	if(!txc_alsprx_obj) {
		APS_ERR("txc sensor is null!!\n");
		return -1;
	}

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&txc_alsprx_obj->init_done)) {
		req.activate_req.sensorType = ID_LIGHT;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = en;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
	} else
		APS_ERR("sensor hub has not been ready!!\n");
	
	if (en)
		set_bit(CMC_BIT_ALS, &txc_alsprx_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &txc_alsprx_obj->enable);

#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
    if ( txc_alsprx_obj->load_nv_flag == 0 ) {
        if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
            alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
            txc_alsprx_obj->load_nv_flag = 1;
        }
    }
    
    res = alsprx_enable_als(&txc_alsprx_obj->txc_state,en);
    if(res < 0) {
        APS_ERR("enable ps fail: %d\n", res); 
        return -1;
    }
    
    if(en)     
        set_bit(CMC_BIT_PS, &txc_alsprx_obj->enable);
    else
        clear_bit(CMC_BIT_PS, &txc_alsprx_obj->enable);    

    if(res) {
        APS_ERR("ps_enable_nodata is failed!!\n");
        return -1;
    }
#endif
    return 0;
}

int als_set_delay(u64 ns)
{
    return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

int als_get_data(int* value, int* status)
{
    int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;

	if (atomic_read(&txc_alsprx_obj->init_done)) {
		req.get_data_req.sensorType = ID_LIGHT;
		req.get_data_req.action = SENSOR_HUB_GET_DATA;
		len = sizeof(req.get_data_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
		if (res)
			APS_ERR("SCP_sensorHub_req_send fail!\n");
		else {
			*value = req.get_data_rsp.data.int16_Data[0];
			*status = SENSOR_STATUS_ACCURACY_MEDIUM;
		}

		if (atomic_read(&txc_alsprx_obj->trace) & CMC_TRC_PS_DATA)
			APS_LOG("value = %d\n", *value);
	} else {
		APS_ERR("sensor hub hat not been ready!!\n");
		res = -1;
	}
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */    
    if(!txc_alsprx_obj ) {
        APS_ERR("txc sensor is null!!\n");
        return -1;
    }
        
    alsprx_read_als(&txc_alsprx_obj->txc_state);
    *value = txc_alsprx_obj->txc_state.als_db.als_mlux;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
#endif    
    return res;
}

int ps_open_report_data(int open)
{    
    return 0;
}

int ps_enable_nodata(int en)
{
    int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
		SCP_SENSOR_HUB_DATA req;
		int len;
#endif
    if(!txc_alsprx_obj) {
        APS_ERR("txc sensor is null!!\n");
        return -1;
    }

#ifdef CUSTOM_KERNEL_SENSORHUB
	if (atomic_read(&txc_alsprx_obj->init_done)) {
		req.activate_req.sensorType = ID_PROXIMITY;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = en;
		len = sizeof(req.activate_req);
		res = SCP_sensorHub_req_send(&req, &len, 1);
	} else
		APS_ERR("sensor hub has not been ready!!\n");

	if (en)
		set_bit(CMC_BIT_PS, &txc_alsprx_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &txc_alsprx_obj->enable);

#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */
    /* Load NV Item */
    if ( txc_alsprx_obj->load_nv_flag == 0 ) {
        if (txc_alsprx_get_nv(&txc_alsprx_obj->txc_state,&txc_alsprx_obj->txc_nv) > 0) {
            alsprx_set_reg_default(&txc_alsprx_obj->txc_state, &txc_alsprx_obj->txc_nv);
            txc_alsprx_obj->load_nv_flag = 1;
        }
    }
    
    res = alsprx_enable_ps(&txc_alsprx_obj->txc_state,en);
    if(res < 0) {
        APS_ERR("enable ps fail: %d\n", res); 
        return -1;
    }
    
    if(en)     
        set_bit(CMC_BIT_PS, &txc_alsprx_obj->enable);
    else
        clear_bit(CMC_BIT_PS, &txc_alsprx_obj->enable);    
#endif
    if(res) {
        APS_ERR("ps_enable_nodata is failed!!\n");
        return -1;
    }
    return 0;
}

int ps_set_delay(u64 ns)
{
    return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

int ps_get_data(int* value, int* status)
{
    int res = 0;
	
#ifdef CUSTOM_KERNEL_SENSORHUB
		SCP_SENSOR_HUB_DATA req;
		int len;
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

    if(!txc_alsprx_obj) {
        APS_ERR("txc sensor is null!!\n");
        return -1;
    }
	
#ifdef CUSTOM_KERNEL_SENSORHUB
		if (atomic_read(&txc_alsprx_obj->init_done)) {
			req.get_data_req.sensorType = ID_PROXIMITY;
			req.get_data_req.action = SENSOR_HUB_GET_DATA;
			len = sizeof(req.get_data_req);
			res = SCP_sensorHub_req_send(&req, &len, 1);
			if (res) {
				APS_ERR("SCP_sensorHub_req_send fail!\n");
				*value = -1;
				res = -1;
			} else {
				*value = req.get_data_rsp.data.int8_Data[0];
				*status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
	
			if (atomic_read(&txc_alsprx_obj->trace) & CMC_TRC_PS_DATA)
				APS_LOG("value = %d\n", *value);
		} else {
			APS_ERR("sensor hub has not been ready!!\n");
			res = -1;
		}
#else /* #ifdef CUSTOM_KERNEL_SENSORHUB */

    res = alsprx_process_ps_data(&txc_alsprx_obj->txc_state);
    
    *value = txc_alsprx_obj->txc_state.prx_db.event;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
#endif   
    return res;
}

// wangjun@wind-mobi.com 20170728 begin
void pa22a_start_als_value(void)
{
   alsprx_read_als(&txc_alsprx_obj->txc_state); // 核心回调，主要用来读原始光感值用的，会调用 filter_als_value() 将校准原始数据传入给 LSensor_Calibrator.c
   APS_LOG("[wjwind] start get als original value \n");
}

void pa22a_write_ps_value(int ps_low_threshold, int ps_high_threshold)
{

     if(!txc_alsprx_obj) {
        APS_LOG("txc sensor is null!!\n");
        return;
        
    }

    APS_LOG("[wjwind]: %s start ps_high_thd_val = %d, ps_low_thd_val = %d cross_talk = %d \n", __FUNCTION__,txc_alsprx_obj->txc_state.prx_db.ps_high_threshold, txc_alsprx_obj->txc_state.prx_db.ps_low_threshold, txc_alsprx_obj->txc_state.prx_db.ps_cross_talk);
   
    alsprx_set_ps_threshold(&txc_alsprx_obj->txc_state, ps_threshold_low,ps_threshold_high,ps_crosstalk); 
    
    //////////////////////////////////////////////////////////////
    // 添加 5cm 门限值
    ps_threshold_5cm = txc_alsprx_obj->txc_state.prx_db.ps_low_threshold;  
    APS_LOG("[wjwind]: %s end ps_high_thd_val = %d, ps_low_thd_val = %d cross_talk = %d \n", __FUNCTION__,txc_alsprx_obj->txc_state.prx_db.ps_high_threshold, txc_alsprx_obj->txc_state.prx_db.ps_low_threshold, txc_alsprx_obj->txc_state.prx_db.ps_cross_talk);

    
}
// wangjun@wind-mobi.com 20170728 end

/*-----------------------------------i2c operations----------------------------------*/
static int txc_alsprx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct txc_alsprx_priv *obj;
    struct als_control_path als_ctl={0};
    struct als_data_path als_data={0};
    struct ps_control_path ps_ctl={0};
    struct ps_data_path ps_data={0};
    
    int err = 0;
    printk("fengqian ==mlk=== enter  %s\n",__func__);
    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
	
	err = get_alsps_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		APS_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit_init_failed;
	}
	printk("fengqian polling_mode_als %d\n",obj->hw.polling_mode_als);
	obj->hw.polling_mode_ps = 0;
	 obj->hw.polling_mode_als = 1;

    // obj->hw = hw;
    txc_alsprx_obj = obj;    
    
    mutex_init(&obj->update_lock); 
    if( obj->hw.polling_mode_ps == 0 )
        INIT_WORK(&obj->eint_work, txc_alsprx_eint_work);
#ifdef CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->init_done_work, txc_alsprx_init_done_work);
#endif /* #ifdef CUSTOM_KERNEL_SENSORHUB */

    obj->client = client;
    i2c_set_clientdata(client, obj);

	//Get the device interface
    txc_alsprx_if = (alsprx_interface*)device_get_interface();
	if(!txc_alsprx_if)
		APS_ERR("Failed to get device interface");
	
    /*-----------------------------value need to be confirmed-----------------------------------------*/
    atomic_set(&obj->als_debounce, 200);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 200);
    atomic_set(&obj->ps_deb_on, 0);
    atomic_set(&obj->ps_deb_end, 0);
    atomic_set(&obj->ps_mask, 0);
    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->als_cmd_val, 0xDF);
    atomic_set(&obj->ps_cmd_val,  0xC1);
    obj->txc_nv.ps_high_threshold = obj->hw.ps_threshold_high;
    obj->txc_nv.ps_low_threshold = obj->hw.ps_threshold_low;
    //atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
    //atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
    
    
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->als_level_num = sizeof(obj->hw.als_level)/sizeof(obj->hw.als_level[0]);
    obj->als_value_num = sizeof(obj->hw.als_value)/sizeof(obj->hw.als_value[0]);
    
    /*-----------------------------value need to be confirmed-----------------------------------------*/

    memcpy(obj->als_level, obj->hw.als_level, sizeof(obj->als_level));
    memcpy(obj->als_value, obj->hw.als_value, sizeof(obj->als_value));
    atomic_set(&obj->i2c_retry, 3);
    set_bit(CMC_BIT_ALS, &obj->enable);
    set_bit(CMC_BIT_PS, &obj->enable);
    client->addr = 0x1e;
    txc_alsprx_i2c_client = client;      

    //wangjun@wind-mobi.com 20170930 begin
    als_calibration_factor = 849;   
    ps_crosstalk      = 10;
    ps_threshold_high = 37;
    ps_threshold_low  = 20;
	ps_threshold_5cm = 150; 	//add by wangjun 
    //wangjun@wind-mobi.com 20170930 end
    
    if((err = alsprx_set_reg_default(&obj->txc_state, &obj->txc_nv))) {
        goto exit_init_failed;
    }
    APS_LOG("txc_sensor_set_reg_default() OK!\n");

    obj->load_nv_flag = 0;

    /* Set IRQ*/
    if (obj->hw.polling_mode_ps == 0 ) {
        err = txc_alsprx_setup_eint(client);        
    }
    
    obj->txc_state.ps_polling = obj->hw.polling_mode_ps;
    
    if (err < 0 )
        APS_LOG("txc_alsprx_setup_eint fail!\n");
    
    if((err = misc_register(&txc_alsprx_device))) {
        APS_ERR("txc_alsprx_device register failed\n");
        goto exit_misc_device_register_failed;
    }
    APS_LOG("txc_alsprx_device misc_register OK!\n");

    /*------------------------ attribute file for debug --------------------------------------*/
    err = txc_alsprx_create_attr(&(txc_alsprx_init_info.platform_diver_addr->driver));
    if(err < 0) {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    /*------------------------s12201001 attribute file for debug--------------------------------------*/

    als_ctl.open_report_data= als_open_report_data;
    als_ctl.enable_nodata = als_enable_nodata;
    als_ctl.set_delay  = als_set_delay;
    als_ctl.is_report_input_direct = false;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
#ifdef CUSTOM_KERNEL_SENSORHUB
    als_ctl.is_support_batch = obj->hw.is_batch_supported_als;
#else
    als_ctl.is_support_batch = false;
#endif
    
    err = als_register_control_path(&als_ctl);
    if(err) {
        APS_ERR("register fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }

    als_data.get_data = als_get_data;
    als_data.vender_div = 100;
    err = als_register_data_path(&als_data);    
    if(err) {
        APS_ERR("tregister fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }
    
    ps_ctl.open_report_data= ps_open_report_data;
    ps_ctl.enable_nodata = ps_enable_nodata;
    ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
//    ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
    ps_ctl.is_support_batch = obj->hw.is_batch_supported_ps;
#else
    ps_ctl.is_support_batch = false;
#endif

    if(1 == obj->hw.polling_mode_ps) {
        ps_ctl.is_polling_mode = 1;
        ps_ctl.is_report_input_direct = false;
    } else {
        ps_ctl.is_polling_mode = 0;//PS interrupt mode
        ps_ctl.is_report_input_direct = true;
    }
    
    err = ps_register_control_path(&ps_ctl);
    if(err) {
        APS_ERR("register fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }

    ps_data.get_data = ps_get_data;
    ps_data.vender_div = 100;
    err = ps_register_data_path(&ps_data);    
    if(err) {
        APS_ERR("tregister fail = %d\n", err);
        goto exit_sensor_obj_attach_fail;
    }
	#if 0
    err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
    if(err) 
        APS_ERR("register light batch support err = %d\n", err);    
    
    err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
    if(err)
        APS_ERR("register proximity batch support err = %d\n", err);
	#endif

    //wangjun@wind-mobi.com 20170728 begin
    write_ps_value_func = pa22a_write_ps_value;       // 更新距感高低门限回调 
    start_als_value_func = pa22a_start_als_value;     // 启动光感校准回调
    psensor_proc_init();
    lightsensor_proc_init();
    //wangjun@wind-mobi.com 20170728 end
#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
    obj->early_drv.suspend  = txc_alsprx_early_suspend,
    obj->early_drv.resume   = txc_alsprx_late_resume,    
    register_early_suspend(&obj->early_drv);
#endif
    printk("fengqian ==mlk== %s  is ok !",__func__);
    txc_alsprx_init_flag = 0;
    return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
        misc_deregister(&txc_alsprx_device);
exit_init_failed:
    kfree(obj);
exit:
    txc_alsprx_i2c_client = NULL;   
    txc_alsprx_init_flag = -1;    
    APS_ERR("%s: err = %d\n", __func__, err);
    return err;
}

static int txc_alsprx_i2c_remove(struct i2c_client *client)
{
    int err;    
    /*------------------------ txc alsprx sensor attribute file for debug--------------------------------------*/    
    if((err = txc_alsprx_delete_attr(&txc_alsprx_i2c_driver.driver)))
        APS_ERR("txc_alsprx_delete_attr fail: %d\n", err);    
    /*----------------------------------------------------------------------------------------*/
    
    if((err = misc_deregister(&txc_alsprx_device)))
        APS_ERR("misc_deregister fail: %d\n", err);        
        
    txc_alsprx_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    //wangjun@wind-mobi.com 20170728 begin
    psensor_proc_remove();
    lightsensor_proc_deinit();
    //wangjun@wind-mobi.com 20170728 end
    return 0;

}

static int txc_alsprx_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TXC_ALSPRX_DEV_NAME);
    return 0;

}

static int txc_alsprx_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    APS_FUN();
    return 0;
}

static int txc_alsprx_i2c_resume(struct i2c_client *client)
{
    APS_FUN();
    return 0;
}
// void txc_alsprx_power(struct alsps_hw *hw, unsigned int on)
// {
 //    return;
// }
/*----------------------------------------------------------------------------*/
static int txc_alsprx_local_uninit(void)
{
    
    // txc_alsprx_power(hw, 0);
    
    i2c_del_driver(&txc_alsprx_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/

static int txc_alsprx_local_init(void) {
    
    // txc_alsprx_power(hw, 1);   
    if(i2c_add_driver(&txc_alsprx_i2c_driver)) {
        printk("add driver error\n");        
        return -1;
    }
    
    if(-1 == txc_alsprx_init_flag) {        
        return -1;
	}
    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init txc_alsprx_init(void)
{
#if 0
    const char *name = "mediatek,txc_alsprx";    
    hw = get_alsps_dts_func(name, hw);    
    if (!hw)        
        APS_ERR("get dts info fail\n");
    // Fouce set ALS in polling mode
    hw->polling_mode_als = 1;
#endif    
    alsps_driver_add(&txc_alsprx_init_info);
    
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit txc_alsprx_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(txc_alsprx_init);
module_exit(txc_alsprx_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("TXC Corp");
MODULE_DESCRIPTION("ALS/PRX driver");
MODULE_LICENSE("GPL");

