/********************************************************************************
* Copyright (c) 2016, "TXC"
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     1. Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*     2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     3. Neither the name of "TXC" nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
//#define QCT
//#define MTK_SENSORHUB
//#define SPRD
#define MTK
//#define SENSOR_TESTER
//#define QUALCOMM_DTS

#ifdef QCT
#include "sns_dd_alsprx.h"
#elif defined(SPRD)
#include "sprdm_alsprx.h"
#elif defined(MTK_SENSORHUB)
#include "mtk_sensorhub.h"
#elif defined(SENSOR_TESTER)
#include "txc_sensor.h"
#elif defined(QUALCOMM_DTS)
#include "qualcomm_txc_alsprx.h"
#else
#include "mtk_txc_alsprx.h"
#endif


/*=======================================================================
 INTERNAL DEFINITIONS
 ========================================================================*/

//#define uint8_t     unsigned int
//#define uint16_t    unsigned int
//#define uint32_t    unsigned int
#define PA22A_I2C_ADDRESS           0x1E      /* 7 bit Address */
/* REGISTER ADDRESS */
#define PA22A_CFG0_ADDR             0x00
#define PA22A_CFG1_ADDR             0x01
#define PA22A_CFG2_ADDR             0x02
#define PA22A_CFG3_ADDR             0x03
#define PA22A_AILTL_ADDR            0x04
#define PA22A_AILTH_ADDR            0x05
#define PA22A_AIHTL_ADDR            0x06
#define PA22A_AIHTH_ADDR            0x07
#define PA22A_PILT_ADDR             0x08
#define PA22A_PIHT_ADDR             0x0A
#define PA22A_ADATAL_ADDR           0x0B
#define PA22A_ADATAH_ADDR           0x0C
#define PA22A_PDATA_ADDR            0x0E
#define PA22A_POFS1_ADDR            0x10
#define PA22A_POFS2_ADDR            0x11    /* set 0x82 */
#define PA22A_CFG4_ADDR             0x12    /* set 0x0C */
#define PA22A_CFG5_ADDR             0x13
#define PA22A_DEVICEID_ADDR         0x7F
#define PA22A_MAX_REG               20
/* REG0:ALS GAIN */
#define PA22A_AGAIN_x1              0x00
#define PA22A_AGAIN_x8              0x10
#define PA22A_AGAIN_x16             0x20
#define PA22A_AGAIN_x80             0x30

/* REG0:SENSOR ENABLE */
#define PA22A_PWR_DOWN              0x00
#define PA22A_ALS_ENABLE            0x01  /* ALS EN */
#define PA22A_PRX_ENABLE            0x02  /* PS EN */

/* REG1:DRIVE CURRENT */
#define PA22A_PDRVIE_15MA           0x40  /* PRX  15mA VCSEL drive */
#define PA22A_PDRVIE_12MA           0x50  /* PRX  12mA VCSEL drive */
#define PA22A_PDRVIE_10MA           0x60  /* PRX  10mA VCSEL drive */
#define PA22A_PDRVIE_7MA            0x70  /* PRX   7mA VCSEL drive */

/* REG1:PS INTERRUPT PERSISTENCE */
#define PA22A_PPERS_1               0x00  /* 1 consecutive proximity value out of range */
#define PA22A_PPERS_2               0x04  /* 2 consecutive proximity value out of range */
#define PA22A_PPERS_4               0x08  /* 4 consecutive proximity value out of range */
#define PA22A_PPERS_8               0x0C  /* 8 consecutive proximity value out of range */

/* REG1:ALS INTERRUPT PERSISTENCE */
#define PA22A_APERS_1               0x00  /* 1 consecutive ambient light value out of range */
#define PA22A_APERS_2               0x01  /* 2 consecutive ambient light value out of range */
#define PA22A_APERS_4               0x02  /* 4 consecutive ambient light value out of range */
#define PA22A_APERS_8               0x03  /* 8 consecutive ambient light value out of range */

/* REG1:ALS INTERNAL PERSISTENCE MODE*/
#define PA22A_INPERS_DISABLE        0x00  /* Internal Persistence DISABLE */
#define PA22A_INPERS_ENABLE         0x10  /* Internal Persistence ENABLE */

/* REG2:OUTPUT MODE */
#define PA22A_PSMODE_NORMAL         0x40  /* PRX Normal Mode */
#define PA22A_PSMODE_OFFSET         0x00  /* PRX Offset Mode */

/* REG2:REGISTER RESET */
#define PA22A_REG_RESET             0x10  /* Register reset */

/* REG2:INTERRUPT SOURCE */
#define PA22A_INTR_ALS              0x00    /* Interrupted only by ALS       */
#define PA22A_INTR_PS               0x04    /* Interrupted only by PS        */
#define PA22A_INTR_NONE             0x08    /* No Interrupt                  */
#define PA22A_INTR_ALS_PS           0x0C    /* Interrupted  by ALS and PS    */

/* REG2: INTERRUPT FLAG MASK */
#define PA22A_INTR_FLAG_ALS_ACTIVE  0x01
#define PA22A_INTR_FLAG_PS_ACTIVE   0x02

/* REG3:INTERRUPT TYPE */
#define PA22A_INTR_TYPE_WINDOW      0x00
#define PA22A_INTR_TYPE_HYSTERESIS  0x40

/* REG3:PS SLEEP TIME */
#define PA22A_PS_SLEEP_6_25MS       0x00
#define PA22A_PS_SLEEP_12_5MS       0x08
#define PA22A_PS_SLEEP_25MS         0x10
#define PA22A_PS_SLEEP_50MS         0x18
#define PA22A_PS_SLEEP_100MS        0x20
#define PA22A_PS_SLEEP_200MS        0x28
#define PA22A_PS_SLEEP_400MS        0x30
#define PA22A_PS_SLEEP_800MS        0x38

/* REG3:ALS SLEEP TIME */
#define PA22A_ALS_SLEEP_0MS         0x00
#define PA22A_ALS_SLEEP_100MS       0x01
#define PA22A_ALS_SLEEP_300MS       0x02
#define PA22A_ALS_SLEEP_700MS       0x03
#define PA22A_ALS_SLEEP_1500MS      0x04

/*=======================================================================
 PA22A REGISTERS DEFINITIONS
 ========================================================================*/
/************************** USER SETTING AREA ************************************/
#define PA22A_VERSION               3                     /* NV Item version */
/* SENSOR SETTING */
#define PA22A_AGAIN                 PA22A_AGAIN_x8
#define PA22A_PDRIVE_CURR           PA22A_PDRVIE_12MA
#define PA22A_PS_READ_MODE          PA22A_PSMODE_OFFSET    
#define PA22A_INTR                  PA22A_INTR_PS           
#define PA22A_INTR_TYPE             PA22A_INTR_TYPE_WINDOW    
#define PA22A_PPERS_NEAR            PA22A_PPERS_2
#define PA22A_PPERS_FAR             PA22A_PPERS_2
#define PA22A_PS_SLEEP_TIME         PA22A_PS_SLEEP_6_25MS
#define PA22A_ALS_SLEEP_TIME        PA22A_ALS_SLEEP_0MS
#define PA22A_APERS                 PA22A_APERS_1
#define PA22A_ALSPS_ENABLE          PA22A_ALS_ENABLE | PA22A_PRX_ENABLE

/* PS SETTING */
#define PA22A_PRX_OFFSET_MAX        180
#define PA22A_PRX_OFFSET_MIN        10
#define PA22A_PRX_OFFSET_EXTRA      4
#define PA22A_PRX_OFFSET_DEFAULT    65
#define PA22A_PRX_DEBOUNCE          5
#define PA22A_PRX_ODR               10   /* Report 10Hz */
#define THRESHOLD_MAX_LIMIT         180

/* PS THRESHOLD MAX VALUE */
#define PA22A_PRX_NEAR_MAX          0
#define PA22A_PRX_FAR_MAX           255 /* ADC 8 bit Max */

/* 20170829_TXC_Modify
#define PA22A_PRX_FAR_DIFF          7   
#define PA22A_PRX_NEAR_DIFF         15  
*/
static int PA22A_PRX_FAR_DIFF = 10;
static int PA22A_PRX_NEAR_DIFF = 24;

/* ALS SETTING */
#define PA22A_ALS_CALIBRATION_LUX_TARGET    400
#define PA22A_ALS_DATA_AVG          10   /* AVG Times     */
#define PA22A_ALS_DATA_AVG_EN       0
#define PA22A_ALS_ODR               10   /* Coversion Time fix 100 ms */
// wangbing@wind-mobi.com 20171025 begin
#define PA22A_ALS_DYNAMIC_RANGE     1    /*Set 1 for dymanic range; 0 for fixed gain */
// wangbing@wind-mobi.com 20171025 end
/*=======================================================================
 TXC REGISTERS DEFINITIONS
 ========================================================================*/
 /************************** USER SETTING AREA ************************************/
#define PA22A_ALS_TH_MAX            65535
#define PA22A_ALS_TH_MIN            0
#define PA22A_PS_TH_MAX             255 /* 8 bit MAX */
#define PA22A_PS_TH_MIN             0   /* Minimun value */

/* ATTRIBUTES INFO */
#define SNS_DD_ALS_PWR              167  /* unit of uA */
#define SNS_DD_ALS_RES              FX_FLTTOFIX_Q16(0.1)
#define SNS_DD_ALS_LO_PWR           1   /* unit of uA */
#define SNS_DD_ALS_ACCURACY         10
#define SNS_DD_ALS_BIT              16
#define SNS_DD_ALS_FREQ             10

#define SNS_DD_PRX_PWR              1790    /* unit of uA */
#define SNS_DD_PRX_RES              FX_FLTTOFIX_Q16(0.001)  /* unit of this data type is meter */
#define SNS_DD_PRX_LO_PWR           1   /* unit of uA */
#define SNS_DD_PRX_ACCURACY         10  /* unit of this data type unit which is mm */
#define SNS_DD_PRX_BIT              8
#define SNS_DD_PRX_FREQ             100

/* OTHER */
#define SNS_DD_ALS_DATA_READY_US    100000  /* 100ms for ALS */
#define SNS_DD_PRX_DATA_READY_US    15000   /* 15ms  for PRX */
#define SNS_DD_1st_DATA_READY_US    250000  /* 250ms for 1st ALS Data */
#define POLLING_MAX_DELAY_COUNT     30

/*ALS GAIN RANGES*/
#define ALS_LUX_MIN_X1				0
#define ALS_LUX_MAX_X1				1000
#define ALS_LUX_MIN_X8				1000
#define ALS_LUX_MAX_X8				10000
#define ALS_LUX_MIN_X16				10000
#define ALS_LUX_MAX_X16				40000
#define ALS_LUX_MIN_X80				40000
/*=======================================================================
 Macros
 ========================================================================*/
/* Negative ADC counts will be treated as zero */
#define ALSPRX_CONV_TO_UNSIGNED(var, bits) ((var & (1<<(bits-1))) ? (0) : (var))
#define TXC_ABS(x) (x) >= 0 ? (x):(x)*(-1)
#define TXC_ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))

#define ARRAY_SUM(arr, sum) \
do { \
    int i = 0; \
    int size = TXC_ARRAY_SIZE(arr); \
    for (i=0; i<size; i++) \
        sum += arr[i]; \
} while (0)

#define ARRAY_ABS_SUM(arr, sum) \
do { \
    int i = 0; \
    int size = TXC_ARRAY_SIZE(arr); \
    for (i=0; i<size; i++) \
        sum += TXC_ABS(arr[i]); \
} while (0)


#define SWAP(x, y) int32_t tmp = x; x = y; y = tmp;
#define MS_TO_US(t) (t * 1000)
#define US_TO_MS(t) (t / 1000)
#define NUM_OF_LUX_TO_AVE  8
#define LUX_AVG_DIVIDER (NUM_OF_LUX_TO_AVE * 1000)
#define MLUX_SAMPLE_SIZE 3


int pa22a_set_reg_default(alsprx_state* state, alsprx_nvdb *nvdb);
int pa22a_enable_als(alsprx_state* state,int enable);
int pa22a_enable_ps(alsprx_state* state,int enable);
int pa22a_als_calibration(alsprx_state* state,alsprx_nvdb* nvdb);
int pa22a_ps_calibration(alsprx_state* state,alsprx_nvdb* nvdb);
int pa22a_process_als_data(alsprx_state* state);
int pa22a_process_ps_data(alsprx_state* state);
int pa22a_read_als(alsprx_state* state);
int pa22a_read_ps(alsprx_state* state);
int pa22a_show_registers(uint8_t* regs_arry, uint8_t arry_size);
int pa22a_set_ps_threshold(uint16_t low_thres,uint16_t high_thres);
int pa22a_set_register(uint8_t addr,uint8_t data);
int pa22a_get_deviceid(alsprx_state* state);
int pa22a_get_vendorid(alsprx_state* state);
device_information pa22a_get_device_info(sensor_type_e type);
int pa22a_enable_interrupt(sensor_type_e type, alsprx_state* state, int enable);
int pa22a_clear_interrupt(sensor_type_e type);
int is_pa22a_enabled(sensor_type_e type);
int pa22a_get_default_nv(alsprx_nvdb* nvdb);
int txc_set_ps_low_thres(uint16_t val);
int txc_set_ps_high_thres(uint16_t val);
int txc_clear_interrupt(sensor_type_e sensor_type);
int pa22a_get_registers_count(void);

//wangjun@wind-mobi.com 20170927 begin
int pa22a_to_set_ps_threshold(alsprx_state* state,uint16_t low_thres,uint16_t high_thres,uint16_t cross_talk)
{

    state->prx_db.ps_high_threshold = high_thres;//ps_threshold_high;
    state->prx_db.ps_low_threshold = low_thres;//ps_threshold_low;
    state->prx_db.ps_cross_talk = cross_talk;//ps_crosstalk;
    // 20170829_TXC
    PA22A_PRX_FAR_DIFF = low_thres - cross_talk;
    PA22A_PRX_NEAR_DIFF = high_thres - cross_talk;
    APS_LOG("[wjwind]: %s ps_high_thd_val = %d, ps_low_thd_val = %d cross_talk = %d \n", __FUNCTION__, state->prx_db.ps_high_threshold, state->prx_db.ps_low_threshold, state->prx_db.ps_cross_talk);
    txc_set_ps_low_thres(state->prx_db.ps_low_threshold);
    txc_set_ps_high_thres(state->prx_db.ps_high_threshold); 
    return 1;

}
//wangjun@wind-mobi.com 20170927 end

#if PA22A_ALS_DYNAMIC_RANGE
static int pa22a_setDynamic_Range(alsprx_state* state);
#endif

alsprx_interface sensor_alsprx_if = {
    .alsprx_set_reg_default     = &pa22a_set_reg_default,
    .alsprx_enable_als          = &pa22a_enable_als,
    .alsprx_enable_ps           = &pa22a_enable_ps,
    .alsprx_als_calibration     = &pa22a_als_calibration,
    .alsprx_ps_calibration      = &pa22a_ps_calibration,
    .alsprx_process_als_data    = &pa22a_process_als_data,
    .alsprx_process_ps_data     = &pa22a_process_ps_data,
    .alsprx_read_als            = &pa22a_read_als,
    .alsprx_read_ps             = &pa22a_read_ps,
    .alsprx_show_registers      = &pa22a_show_registers,
    //wangjun@wind-mobi.com 20170927 begin
    .alsprx_set_ps_threshold    = &pa22a_to_set_ps_threshold,
    //wangjun@wind-mobi.com 20170927 end
    .alsprx_set_register        = &pa22a_set_register,
    .alsprx_get_deviceid        = &pa22a_get_deviceid,
    .alsprx_get_vendorid        = &pa22a_get_vendorid,
    .alsprx_get_device_info     = &pa22a_get_device_info,
    .alsprx_enable_interrupt    = &pa22a_enable_interrupt,
    .alsprx_clear_interrupt     = &pa22a_clear_interrupt,
    .is_alsprx_enabled          = &is_pa22a_enabled,
    .alsprx_get_default_nv      = &pa22a_get_default_nv,
    .alsprx_get_registers_count = &pa22a_get_registers_count
};

static device_information dev_info = {
    .name       = "Proximity",
    .model      = "PA22A",
    .vendor     = "TXC",
    .version    = 1,
    .i2c_addr   = PA22A_I2C_ADDRESS
};

/* For oil-proof algorithm */
#define OIL_EFFECT 40
#define PS_ARY_SIZE 5

static const int ps_steady = PS_ARY_SIZE + 4;
static int saturation_flag = 0;
static int oil_occurred = 0;
static int delay_count = 0;
static uint8_t alsSampleInitialize = 0;
//static int bCal_flag=0;

//alsprx_state device;
struct ps_sample{
    int ps_sample[PS_ARY_SIZE];
    int cur_index;
    int sample_avg;
    int is_ps_steady;
};
/* ALS ADC TO LUX COEF @ 400 lux*/
static uint32_t als_adc_coefs_IR[] = {
    11139,
    1403,
    347,
    68
};

static uint32_t als_adc_coefs[] = {
    11139,
    1403,
    347,
    68
};
struct ps_sample ps_seq_far =    {
                                        {255, 255, 255, 255, 255}, //PS samples
                                        0,        //Current Index
                                        255,    //Samples avg
                                        0
                                };
struct ps_sample ps_seq_oil =    {
                                        {255, 255, 255, 255, 255}, //PS samples
                                        0,        //Current Index
                                        255,    //Samples avg
                                        0
                                };

void* device_get_interface()
{
    return (void*)(&sensor_alsprx_if);
}

static void txc_u8_swap(uint8_t *x, uint8_t *y)
{
    uint8_t temp = *x;
    *x = *y;
    *y = temp;
}
void txc_bubble_sort_u8(uint8_t* array, int arysize)
{
    int i = 0;
    int j = 0;
    for (i = 0; i < arysize; i++)
        for (j = i+1; j < arysize; j++)
            if (array[i] > array[j])
                txc_u8_swap(array + i, array + j);
}

static void txc_u32_swap(uint32_t *x, uint32_t *y)
{
    uint32_t temp = *x;
    *x = *y;
    *y = temp;
}
void txc_bubble_sort_u32(uint32_t* array, int arysize)
{
    int i = 0;
    int j = 0;
    for (i = 0; i < arysize; i++)
        for (j = i+1; j < arysize; j++)
            if (array[i] > array[j])
                txc_u32_swap(array + i, array + j);
}

device_information pa22a_get_device_info(sensor_type_e type)
{
     return dev_info;
}
int pa22a_enable_interrupt(sensor_type_e type, alsprx_state* state, int enable)
{
    int status = 0;
    uint8_t byteVal;
    if(type == TXC_SENSOR_PROXIMITY || type == TXC_SENSOR_ALSPRX){
          state->ps_polling = false;
  
           if(enable)
               byteVal = PA22A_PS_READ_MODE | PA22A_INTR_PS;
           else
             byteVal = PA22A_PS_READ_MODE | PA22A_INTR_NONE;

           txc_set_ps_high_thres(state->prx_db.ps_high_threshold);
           txc_set_ps_low_thres(state->prx_db.ps_low_threshold);
      
           status = i2c_write_byte(PA22A_CFG2_ADDR, byteVal);  //set PS INT disable
           status = txc_clear_interrupt(TXC_SENSOR_PROXIMITY);      //clear flag
        return status;
    }
    else
        return 0;
}
int pa22a_clear_interrupt(sensor_type_e type)
{
    return txc_clear_interrupt(type);
}
int is_pa22a_enabled(sensor_type_e type)
{
    int status = 0;
    uint8_t byteVal;
    status = i2c_read_byte(PA22A_CFG0_ADDR, &byteVal);
    if(type == TXC_SENSOR_AMBIENT)
        return (byteVal & PA22A_ALS_ENABLE);
    else
        return (byteVal & PA22A_PRX_ENABLE);
    
    return status;
}
int pa22a_get_default_nv(alsprx_nvdb* nvdb)
{
	int i=0;
    ALSPRX_MSG_0(HIGH, "is_pa22a_enabled");

    nvdb->nv_size              		= 128;  //defined as 128 max
    nvdb->version_num          		= PA22A_VERSION;
    nvdb->Reg0						= PA22A_AGAIN;
	nvdb->Reg1						= PA22A_PDRIVE_CURR | PA22A_PPERS_NEAR | PA22A_APERS;
	nvdb->Reg2						= PA22A_PS_READ_MODE | PA22A_INTR;
	nvdb->Reg3						= PA22A_INTR_TYPE | PA22A_PS_SLEEP_TIME | PA22A_ALS_SLEEP_TIME;
	nvdb->Reg11						= 0x82;
	nvdb->Reg12						= 0x0C;
	nvdb->Reg13						= 0x00;
    nvdb->prxXtalk             		= 0;
	nvdb->prxXtalk_base				= 0;
	nvdb->ps_low_threshold 			= PA22A_PRX_OFFSET_MAX; 
	nvdb->ps_high_threshold			= PA22A_PRX_OFFSET_MAX -1;
    nvdb->als_caliLUX				= PA22A_ALS_CALIBRATION_LUX_TARGET;
	for(i=0; i<4; i++)
	{
		nvdb->als_caliADC_IR[i]		= als_adc_coefs_IR[i];
		nvdb->als_caliADC[i]			= als_adc_coefs[i];
	}

    return 0;
}
    

static void txc_process_ps_sample(uint8_t ps)
{
    int i=0, ps_sum = 0, abs_slope_sum = 0;
    struct ps_sample *ps_sample = 0;

    //TODO: Need to have single array
    if(oil_occurred)
        ps_sample = &ps_seq_oil;
    else
        ps_sample = &ps_seq_far;

    ps_sample->ps_sample[ps_sample->cur_index] = ps;

    ++ps_sample->cur_index;
    if (ps_sample->cur_index > PS_ARY_SIZE-1)
        ps_sample->cur_index = 0;

    //Caluclate the average and slope absolute sum for collected ps samples
    for (i=0; i<PS_ARY_SIZE; i++) {
        ps_sum += ps_sample->ps_sample[i];
    }

    for (i=0; i<PS_ARY_SIZE-1; i++) {
        abs_slope_sum += TXC_ABS(ps_sample->ps_sample[i] - ps_sample->ps_sample[i+1]);
    }
        
    ps_sample->sample_avg = ps_sum / PS_ARY_SIZE;

    if(abs_slope_sum < ps_steady)
        ps_sample->is_ps_steady = true;
    else
        ps_sample->is_ps_steady = false;

    ALSPRX_MSG_3(LOW,"average PS:%d, abs_slope_sum:%d, is_ps_steady:%d\n", 
        ps_sample->sample_avg, abs_slope_sum, ps_sample->is_ps_steady);

}

static void txc_reset_ps_samples(void)
{
    int i = 0;
    for (i=0; i<PS_ARY_SIZE; i++) {
        ps_seq_oil.ps_sample[i] = 255;
        ps_seq_far.ps_sample[i] = 255;
    }
    ps_seq_oil.is_ps_steady = false;
    ps_seq_oil.sample_avg = 255;
    ps_seq_oil.cur_index = 0;

    ps_seq_far.is_ps_steady = false;
    ps_seq_far.sample_avg = 255;
    ps_seq_far.cur_index = 0;

}

static bool txc_is_ps_data_unsaturated(prx_db_type* prx_db)
{
    bool state = false;
    //If saturation happened, the average ps value must be greater than (far_ps_min-10) and also  steady
    if ( (saturation_flag && ps_seq_far.sample_avg >= ( prx_db->ps_cross_talk>10 ? (prx_db->ps_cross_talk-10) : prx_db->ps_cross_talk ))
          || !saturation_flag || (saturation_flag && prx_db->ps_cross_talk >= PA22A_PRX_OFFSET_MAX))
            state = true;
    else
            state = false;
    return state;
}

int txc_clear_interrupt(sensor_type_e sensor_type)
{

    uint8_t regdata;
    int status = 0;

    ALSPRX_MSG_1(MED,"Clear interrupt flag, sensor = %x\n",sensor_type);

    status = i2c_read_byte(PA22A_CFG2_ADDR,&regdata);

    switch (sensor_type)
    {
      case TXC_SENSOR_AMBIENT:
        
        regdata &= 0xFE;
        break;
        
     case TXC_SENSOR_PROXIMITY:
        
        regdata &= 0xFD;
        break;
        
     default:
        
        regdata &= 0xFC;
        break;
    }

    status = i2c_write_byte(PA22A_CFG2_ADDR, regdata);
    return status;

}
int txc_set_ps_low_thres(uint16_t val)
{
    uint8_t msb,err;
    msb = val;
    err = i2c_write_byte(PA22A_PILT_ADDR, msb);
    return err;
}

int txc_set_ps_high_thres(uint16_t val)
{
    uint8_t  msb,err;
       msb = val;
    err = i2c_write_byte(PA22A_PIHT_ADDR, msb);
    return err;
}

int pa22a_show_registers(uint8_t* regs_arry, uint8_t arry_size)
{
    uint8_t reg_vals;
    int status;
    int i = 0;

    for(i=0; i<PA22A_MAX_REG; i++)
    {    
        status = i2c_read_byte(0x00+i, &reg_vals);

        if(status<0) {
               break;
        } else {
            ALSPRX_MSG_2(HIGH,"REG[0x%02x] = 0x%02x\n", i, reg_vals);
            if (regs_arry && i < arry_size)
                regs_arry[i] = reg_vals;
        }
    }

    return status;
}

int pa22a_set_reg_default(alsprx_state* state, alsprx_nvdb *nvdb)
{
    uint8_t byteVal;
    int status = 0;    
    ALSPRX_MSG_0(HIGH,"Set Register Default\n");
    ALSPRX_MSG_1(HIGH,"nvdb->nv_size:%d \n",nvdb->nv_size);
    APS_LOG("[wjwind] %s \n",__FUNCTION__);
	/* Set default ALS gain coefficients */
	state->als_db.als_caliLUX = PA22A_ALS_CALIBRATION_LUX_TARGET;
	state->als_db.als_gain = PA22A_AGAIN;
	state->prx_db.ps_cross_talk = PA22A_PRX_OFFSET_MAX;
#if 0	
    if (nvdb->nv_size > 0) {
		/*Set the Calibrated ADC Coefficients*/
		if(nvdb->als_caliADC_IR[0]){
			for(i=0; i<4; i++)
			{
				als_adc_coefs_IR[i] = nvdb->als_caliADC_IR[i];
				als_adc_coefs[i] = nvdb->als_caliADC[i];
			}
		}
        /* Set CFG0 */
        byteVal = nvdb->Reg0;
        status = i2c_write_byte(PA22A_CFG0_ADDR, byteVal);
        CHECK_STATUS(status);

        /* Set CFG1: LEDC,PS PRST,ALS PRST */
        byteVal = nvdb->Reg1;
        status = i2c_write_byte(PA22A_CFG1_ADDR, byteVal);
        CHECK_STATUS(status);

        /* Set CFG2: PS READ MODE,INT MODE */
        byteVal = nvdb->Reg2;
        status = i2c_write_byte(PA22A_CFG2_ADDR, byteVal);
        CHECK_STATUS(status);

        /* Set CFG3: INT TYPE,PS SLEEP TIME,ALS SLEEP TIME */
        byteVal = nvdb->Reg3;
        status = i2c_write_byte(PA22A_CFG3_ADDR, byteVal);
        CHECK_STATUS(status);

        state->prx_db.ps_high_threshold = PA22A_PRX_OFFSET_MAX;
        state->prx_db.ps_low_threshold = PA22A_PRX_OFFSET_MAX-1;
        state->prx_db.ps_cross_talk = PA22A_PRX_OFFSET_MAX;

        // wangjun@wind-mobi.com 20170728 begin
        if(((0 != ps_threshold_low)||(0 != ps_threshold_high))&&(ps_threshold_high > ps_threshold_low))
        {
            state->prx_db.ps_high_threshold = ps_threshold_high;
            state->prx_db.ps_low_threshold = ps_threshold_low;
            state->prx_db.ps_cross_talk = ps_crosstalk;
            // 20170829_TXC
            PA22A_PRX_FAR_DIFF = ps_threshold_low - ps_crosstalk;
            PA22A_PRX_NEAR_DIFF = ps_threshold_high - ps_crosstalk;
            APS_LOG("[wjwind]: %s ps_high_thd_val = %d, ps_low_thd_val = %d cross_talk = %d \n", __FUNCTION__, state->prx_db.ps_high_threshold, state->prx_db.ps_low_threshold, state->prx_db.ps_cross_talk);
        }
        // wangjun@wind-mobi.com 20170728 end

        txc_set_ps_low_thres(state->prx_db.ps_low_threshold);
        txc_set_ps_high_thres(state->prx_db.ps_high_threshold); 

        /* Set ALS threshold */
        status = i2c_write_byte( PA22A_AIHTH_ADDR, 0xFF);
        CHECK_STATUS(status);
        status = i2c_write_byte( PA22A_AIHTL_ADDR, 0xFF);
        CHECK_STATUS(status);
        status = i2c_write_byte( PA22A_AILTH_ADDR, 0x00);
        CHECK_STATUS(status);
        status = i2c_write_byte( PA22A_AILTL_ADDR, 0x00);
        CHECK_STATUS(status);

        /* Set PS threshold */
        status = i2c_write_byte( PA22A_POFS1_ADDR, 0x00);
        CHECK_STATUS(status);
        
        /* Set OFFSET2: 0x82 */   
        byteVal = nvdb->Reg11;
        status = i2c_write_byte(PA22A_POFS2_ADDR, 0x82);
        CHECK_STATUS(status);

        /* Set CFG4: 0x0C */        
        byteVal = nvdb->Reg12;
        status = i2c_write_byte(PA22A_CFG4_ADDR, 0x1C);
        CHECK_STATUS(status);       
        
        /* Set CFG5: 0x00 */        
        byteVal = nvdb->Reg13;
        status = i2c_write_byte(PA22A_CFG5_ADDR, 0x00);
        CHECK_STATUS(status);

        /* Set ALS */
		if(nvdb->als_caliLUX)
			state->als_db.als_caliLUX = nvdb->als_caliLUX;
		
    } else {
#endif
        /* Set CFG0: ALS Gain */
        byteVal = PA22A_AGAIN;
        nvdb->Reg0 = byteVal;
        status = i2c_write_byte(PA22A_CFG0_ADDR, byteVal);
        CHECK_STATUS(status);

        /* Set CFG1: LEDC,PS PRST,ALS PRST */
        byteVal = PA22A_PDRIVE_CURR | PA22A_PPERS_NEAR | PA22A_APERS;
        nvdb->Reg1 = byteVal;
        status = i2c_write_byte(PA22A_CFG1_ADDR, byteVal);
        CHECK_STATUS(status);

        /* Set CFG2: PS READ MODE,INT MODE */
        byteVal = PA22A_PS_READ_MODE | PA22A_INTR;
        nvdb->Reg2 = byteVal;
        status = i2c_write_byte(PA22A_CFG2_ADDR, byteVal);
        CHECK_STATUS(status);

        /* Set CFG3: INT TYPE,PS SLEEP TIME,ALS SLEEP TIME */
        byteVal = PA22A_INTR_TYPE | PA22A_PS_SLEEP_TIME | PA22A_ALS_SLEEP_TIME;
        nvdb->Reg3 = byteVal;
        status = i2c_write_byte(PA22A_CFG3_ADDR, byteVal);
        CHECK_STATUS(status);

        state->prx_db.ps_high_threshold = PA22A_PRX_OFFSET_MAX;
        state->prx_db.ps_low_threshold = PA22A_PRX_OFFSET_MAX-1;
        state->prx_db.ps_cross_talk = PA22A_PRX_OFFSET_MAX;

        // wangjun@wind-mobi.com 20170728 begin
        if(((0 != ps_threshold_low)||(0 != ps_threshold_high))&&(ps_threshold_high > ps_threshold_low))
        {
            state->prx_db.ps_high_threshold = ps_threshold_high;
            state->prx_db.ps_low_threshold = ps_threshold_low;
            state->prx_db.ps_cross_talk = ps_crosstalk;
            // 20170829_TXC
            PA22A_PRX_FAR_DIFF = ps_threshold_low - ps_crosstalk;
            PA22A_PRX_NEAR_DIFF = ps_threshold_high - ps_crosstalk;
            APS_LOG("[wjwind]: %s ps_high_thd_val = %d, ps_low_thd_val = %d cross_talk = %d \n", __FUNCTION__, state->prx_db.ps_high_threshold, state->prx_db.ps_low_threshold, state->prx_db.ps_cross_talk);
        }
        // wangjun@wind-mobi.com 20170728 end

        txc_set_ps_low_thres(state->prx_db.ps_low_threshold);
        txc_set_ps_high_thres(state->prx_db.ps_high_threshold); 

        /* Set ALS threshold */
        status = i2c_write_byte( PA22A_AIHTH_ADDR, 0xFF);
        CHECK_STATUS(status);
        status = i2c_write_byte( PA22A_AIHTL_ADDR, 0xFF);
        CHECK_STATUS(status);
        status = i2c_write_byte( PA22A_AILTH_ADDR, 0x00);
        CHECK_STATUS(status);
        status = i2c_write_byte( PA22A_AILTL_ADDR, 0x00);
        CHECK_STATUS(status);

        /* Set PS threshold */
        status = i2c_write_byte( PA22A_POFS1_ADDR, 0x00);
        CHECK_STATUS(status);
        
        /* Set OFFSET 2: 0x82 */        
        status = i2c_write_byte(PA22A_POFS2_ADDR    , 0x82);
        nvdb->Reg11 = 0x82;
        CHECK_STATUS(status);

        /* Set CFG4: 0x0C */        
        status = i2c_write_byte(PA22A_CFG4_ADDR    , 0x1C);
        nvdb->Reg12 = 0x0C;
        CHECK_STATUS(status); 
        
        /* Set CFG5: 0x00 */
        status = i2c_write_byte(PA22A_CFG5_ADDR, 0x00);        
        nvdb->Reg13 = 0x00;
   // }
    //Show reg settings
    pa22a_show_registers(0, 0);

    return status;
}

int pa22a_enable_ps(alsprx_state* state, int enable)
{
    uint8_t cfg0;
    int status = 0;

    /* Read back CFG0 setting */
    i2c_read_byte(PA22A_CFG0_ADDR,&cfg0);
    cfg0 &= 0xFC;

    if (enable) {
        txc_reset_ps_samples();
        state->enable_mode |= PA22A_PRX_ENABLE;
        state->ps_odr = PA22A_PRX_ODR;
        state->prx_db.ps_high_threshold = state->prx_db.ps_cross_talk + OIL_EFFECT;
        state->prx_db.ps_low_threshold = state->prx_db.ps_cross_talk + OIL_EFFECT-1;
        //state->prx_db.ps_cross_talk = PA22A_PRX_OFFSET_MAX;
        saturation_flag = 0;
        oil_occurred = 0;
        if (state->ps_polling == false) {
				pa22a_enable_interrupt(TXC_SENSOR_PROXIMITY,state, 1);
        }

        //Must reset sleep time 
        i2c_write_byte(PA22A_CFG3_ADDR,
                    PA22A_INTR_TYPE | PA22A_PS_SLEEP_TIME | PA22A_ALS_SLEEP_TIME);    

    } else {
        state->enable_mode &= ~PA22A_PRX_ENABLE;
        state->ps_odr = 0;
    }

    /* CFG0 setting */
    cfg0 = cfg0 | state->enable_mode;
    
    status = i2c_write_byte(PA22A_CFG0_ADDR, cfg0);
    ALSPRX_MSG_1(HIGH, "PS enable:%d\n", enable);
    return status;

}

int pa22a_enable_als(alsprx_state* state, int enable)
{
    uint8_t cfg0;
    int status = 0;

    /* Read back CFG0 setting */
    i2c_read_byte(PA22A_CFG0_ADDR,&cfg0);
    cfg0 &= 0xFC;

    if (enable) {
        state->enable_mode |=    PA22A_ALS_ENABLE;
        state->als_odr = PA22A_ALS_ODR;
	alsSampleInitialize = 1;
    } else {
        state->enable_mode &= ~PA22A_ALS_ENABLE;
        state->als_odr = 0;
		alsSampleInitialize = 0;
    }

    /* CFG0 setting */
    cfg0 = cfg0 | state->enable_mode;
    status = i2c_write_byte(PA22A_CFG0_ADDR, cfg0);
    /* Read back CFG0 setting */
    i2c_read_byte(PA22A_CFG0_ADDR,&cfg0);
    cfg0 &= 0xFC;
    ALSPRX_MSG_1(HIGH, "ALS enable:%d\n", enable);
    return status;
}

int pa22a_als_calibration(alsprx_state* state,alsprx_nvdb* nvdb)
{
    int status;
    int i = 0,k=0;
    uint8_t byteVal = 0, cfg0 = 0;
    int arysize = 10;
    uint32_t als_data[arysize], sum_of_data = 0;    
    uint8_t  regdata;
    uint16_t als_adc = 0;
    //device_nv_db *nv_db;
    if(!nvdb) {
        ALSPRX_MSG_1(HIGH,"%s ERROR\n",__FUNCTION__);
        return -1;
    }
    i2c_read_byte(PA22A_CFG0_ADDR, &cfg0);
    for(k=0;k<4;k++) {
		sum_of_data = 0;
		byteVal = k<<4;
	    /* ALSPS On */
	    byteVal = PA22A_ALSPS_ENABLE | byteVal;
	    status = i2c_write_byte(PA22A_CFG0_ADDR, byteVal);
	    ms_delay(250);
	    
	    for (i=0; i < arysize; i++) {
            i2c_read_buf(PA22A_ADATAL_ADDR, 2,(uint8_t *) &als_adc);
	        als_data[i] = als_adc;
	        ALSPRX_MSG_2(HIGH,"als_temp_data=%d als raw data:%d\n",als_data[i],als_adc);
	        ms_delay(100);
	    }

	    /* als adc sorting */
	    txc_bubble_sort_u32(als_data, arysize);    

	    /* als data averaging */
	    for (i = 3; i < 8; i++)
	        sum_of_data = sum_of_data + als_data[i];
	    
	    als_adc_coefs_IR[k] = sum_of_data/(arysize-5);
	    
	    ALSPRX_MSG_2(HIGH,"als_cal_IR_data[%d]=%d\n",k,als_adc_coefs_IR[k]);
	    
	    /* Only ALS On */
		byteVal = k<<4;
	    byteVal = PA22A_ALS_ENABLE | byteVal;
	    status = i2c_write_byte(PA22A_CFG0_ADDR, byteVal);
	    ms_delay(250);
	    for (i=0; i<arysize; i++) {
	        i2c_read_buf(PA22A_ADATAL_ADDR, 2,(uint8_t *) &als_adc);
	        als_data[i] = als_adc;
	        ALSPRX_MSG_2(HIGH,"als_temp_data=%d als raw data:%d\n",als_data[i],als_adc);
	        ms_delay(100);
	    }
	    
	    /* als adc sorting */
	    txc_bubble_sort_u32(als_data, arysize);
	    
	    /* als data averaging */
	    sum_of_data = 0 ;
	    for (i = 3; i < 8; i++)    
	        sum_of_data += als_data[i];

	    als_adc_coefs[k] = sum_of_data/(arysize-5);
	    ALSPRX_MSG_2(HIGH,"als_cal_data[%d]=%d\n",k,als_adc_coefs[k]);
    }

	for(i=0; i<4; i++)
	{
		nvdb->als_caliADC_IR[i] = als_adc_coefs_IR[i];
		nvdb->als_caliADC[i] = als_adc_coefs[i];
	}

	/* Restore CFG0 data */
    status = i2c_write_byte(PA22A_CFG0_ADDR, cfg0);
    
    /* save data into NV */
    nvdb->nv_size = 128;
    nvdb->version_num = NV_VERSION;
    nvdb->Reg0 = cfg0;
    status = i2c_read_byte(PA22A_CFG1_ADDR,&regdata);
    nvdb->Reg1 = regdata;
    status = i2c_read_byte(PA22A_CFG2_ADDR,&regdata);
    nvdb->Reg2 = regdata;
    status = i2c_read_byte(PA22A_CFG3_ADDR,&regdata);
    nvdb->Reg3 = regdata;
    status = i2c_read_byte(PA22A_POFS2_ADDR,&regdata);
    nvdb->Reg11 = regdata;
    status = i2c_read_byte(PA22A_CFG4_ADDR,&regdata);
    nvdb->Reg12 = regdata;
    status = i2c_read_byte(PA22A_CFG5_ADDR,&regdata);
    nvdb->Reg13 = regdata;
    //nvdb->als_caliLUX = PA22A_ALS_CALIBRATION_LUX_TARGET;
    
    /* Save in device */
	state->als_db.als_gain = cfg0 & 0xF0;
    state->als_db.als_caliLUX = nvdb->als_caliLUX;
    return status;
}

int pa22a_ps_calibration(alsprx_state* state,alsprx_nvdb* nvdb)
{
    int status;
    int i = 0;
    uint8_t byteVal = 0, cfg0 = 0;
    int arysize = 10;
    uint32_t sum_of_data = 0;
    uint8_t ps_data[10];
    uint8_t reg_vals;    
    
    status = i2c_read_byte(PA22A_CFG0_ADDR, &cfg0);

    if(!nvdb) {
        ALSPRX_MSG_1(HIGH,"%s ERROR\n",__FUNCTION__);
        return -1;
    }

    /* PS On */
    byteVal = PA22A_PRX_ENABLE | nvdb->Reg0;
    status = i2c_write_byte(PA22A_CFG0_ADDR, byteVal);
    ms_delay(250);
    
    for (i=0; i < arysize; i++) {
        status = i2c_read_byte(PA22A_PDATA_ADDR,&(ps_data[i]));
        ALSPRX_MSG_1(HIGH,"ps_temp_data=%d\n",ps_data[i]);
        ms_delay(100);
    }

    /* als adc sorting */
    txc_bubble_sort_u8(ps_data, arysize);    
    
    /* ps data averaging */
    sum_of_data = 0 ;
    for (i = 3; i < 8; i++)    
        sum_of_data += ps_data[i];

    nvdb->prxXtalk = sum_of_data/(arysize-5);
    
    ALSPRX_MSG_1(HIGH,"ps_cal_data=%d\n", nvdb->prxXtalk);
    
    /* Restore CFG0 data */
    status = i2c_write_byte(PA22A_CFG0_ADDR, cfg0);
    
    /* save data into NV */
    nvdb->nv_size = 128;
    nvdb->version_num = NV_VERSION;
    status = i2c_read_byte(PA22A_CFG0_ADDR,&reg_vals);
    nvdb->Reg0 = reg_vals;
    status = i2c_read_byte(PA22A_CFG1_ADDR,&reg_vals);
    nvdb->Reg1 = reg_vals;
    status = i2c_read_byte(PA22A_CFG2_ADDR,&reg_vals);
    nvdb->Reg2 = reg_vals;
    status = i2c_read_byte(PA22A_CFG3_ADDR,&reg_vals);
    nvdb->Reg3 = reg_vals;
    status = i2c_read_byte(PA22A_POFS2_ADDR,&reg_vals);
    nvdb->Reg11 = reg_vals;
    status = i2c_read_byte(PA22A_CFG4_ADDR,&reg_vals);
    nvdb->Reg12 = reg_vals;
    status = i2c_read_byte(PA22A_CFG5_ADDR,&reg_vals);
    nvdb->Reg13 = reg_vals;
    
    return status;    
}

int pa22a_process_als_data(alsprx_state* state) 
{    

#if PA22A_ALS_DYNAMIC_RANGE	
	//Set the dymanic ALS Gain
	pa22a_setDynamic_Range(state);
#endif

    return 0;
}

int pa22a_process_ps_data(alsprx_state* state)
{
    int status = 0;
    uint32_t ps_data;
    uint32_t ps_high_threshold = 0;
    uint32_t ps_low_threshold = 0;
    uint8_t CFG3_data = 0;
    pa22a_read_ps(state);

    ps_data = state->prx_db.prx_data;
    ps_high_threshold = state->prx_db.ps_high_threshold;
    ps_low_threshold = state->prx_db.ps_low_threshold;    
    CFG3_data = PA22A_INTR_TYPE | PA22A_PS_SLEEP_TIME | PA22A_ALS_SLEEP_TIME;
    APS_LOG("[wjwind]: %s start ps_high_thd_val = %d, ps_low_thd_val = %d cross_talk = %d \n", __FUNCTION__,state->prx_db.ps_high_threshold, state->prx_db.ps_low_threshold, state->prx_db.ps_cross_talk);
    APS_LOG("[wjwind]: %s ps_cali PH = %d, PL = %d CT = %d \n", __FUNCTION__,ps_threshold_high, ps_threshold_low, ps_crosstalk);
    /* SUNLIGHT */
    if (ps_data == 0) {
        saturation_flag = 1;

        if (oil_occurred && state->prx_db.ps_cross_talk < PA22A_PRX_OFFSET_MAX) {
            ps_high_threshold = state->prx_db.ps_cross_talk + OIL_EFFECT + PA22A_PRX_NEAR_DIFF;
            ps_low_threshold = state->prx_db.ps_cross_talk + OIL_EFFECT;
        } else if (!oil_occurred && state->prx_db.ps_cross_talk < PA22A_PRX_OFFSET_MAX) {
            ps_high_threshold = state->prx_db.ps_cross_talk + PA22A_PRX_NEAR_DIFF;
            ps_low_threshold = state->prx_db.ps_cross_talk + PA22A_PRX_FAR_DIFF;
        } else if (state->prx_db.ps_cross_talk >= PA22A_PRX_OFFSET_MAX) {
            ps_high_threshold = PA22A_PRX_OFFSET_MAX;
            ps_low_threshold = PA22A_PRX_OFFSET_MAX-1;
        }
        
        ALSPRX_MSG_0(HIGH,"Sun light State\n");
        state->prx_db.event = PRX_FAR_AWAY;
        goto exit_prx_binary;
    }

    /* FARTHER AWAY */
    if (ps_data < ps_low_threshold && state->prx_db.event == PRX_FAR_AWAY) {

        txc_process_ps_sample(ps_data);

        if ( txc_is_ps_data_unsaturated(&(state->prx_db))) {

            ALSPRX_MSG_0(HIGH,"FARTHER AWAY state\n");
            
            /* STEADY */
            if (ps_seq_far.is_ps_steady) {
                if (saturation_flag)
                    saturation_flag = 0;

                state->prx_db.event = PRX_FAR_AWAY;

                oil_occurred = 0;
                
                state->prx_db.ps_cross_talk = ps_seq_far.sample_avg;
                if (state->prx_db.ps_cross_talk > PA22A_PRX_OFFSET_MAX+OIL_EFFECT)
                    state->prx_db.ps_cross_talk = PA22A_PRX_OFFSET_MAX+OIL_EFFECT;    
                ALSPRX_MSG_1(LOW,"New cross talk : %d\n", state->prx_db.ps_cross_talk);
                ps_high_threshold = state->prx_db.ps_cross_talk + PA22A_PRX_NEAR_DIFF;
                ps_low_threshold = state->prx_db.ps_cross_talk>5? (state->prx_db.ps_cross_talk-5):1;
            }
        }

    } else if (ps_data > ps_high_threshold)    {/* NEAR */

        ALSPRX_MSG_0(HIGH,"FAR to NEAR state\n");
        state->prx_db.event = PRX_NEAR_BY;
        delay_count = 0;
        if (ps_data >= 254){
            oil_occurred = 1;
            ps_low_threshold  = state->prx_db.ps_cross_talk + OIL_EFFECT ;
            ps_high_threshold = 0xFF;
            txc_reset_ps_samples();
        } else     {
            ps_low_threshold  = state->prx_db.ps_cross_talk + PA22A_PRX_FAR_DIFF ;
            ps_high_threshold = 254;
        }
    }

    /* NEAR to FAR */
    if (ps_data < ps_low_threshold && state->prx_db.event == PRX_NEAR_BY) {
        ALSPRX_MSG_0(HIGH,"NEAR to FAR state\n");
        if (oil_occurred) {
            txc_process_ps_sample(ps_data);            
            /* STEADY */
            if (ps_seq_oil.is_ps_steady /*&& (delay_count > POLLING_MAX_DELAY_COUNT)*/) {
                state->prx_db.event = PRX_FAR_AWAY;
                
                oil_occurred = 0;                
                state->prx_db.ps_cross_talk = ps_seq_oil.sample_avg;
                if (state->prx_db.ps_cross_talk > PA22A_PRX_OFFSET_MAX+OIL_EFFECT)
                    state->prx_db.ps_cross_talk = PA22A_PRX_OFFSET_MAX+OIL_EFFECT;    
                
                ALSPRX_MSG_1(LOW,"STEADY cross talk : %d\n", state->prx_db.ps_cross_talk);
                ps_high_threshold = state->prx_db.ps_cross_talk + PA22A_PRX_NEAR_DIFF;
                ps_low_threshold = state->prx_db.ps_cross_talk > 5 ? (state->prx_db.ps_cross_talk-5): 1;
                
            } /*else if(ps_data < state->prx_db.ps_cross_talk + PA22A_PRX_FAR_DIFF ) {
                state->prx_db.event = PRX_FAR_AWAY;
                oil_occurred = 0;
#if 0                
                state->prx_db.ps_cross_talk = ps_data;
                if (state->prx_db.ps_cross_talk > PA22A_PRX_OFFSET_MAX+OIL_EFFECT)
                    state->prx_db.ps_cross_talk = PA22A_PRX_OFFSET_MAX+OIL_EFFECT;    
                
                ALSPRX_MSG_1(LOW,"Far cross talk : %d\n", state->prx_db.ps_cross_talk);
#endif
                ps_high_threshold = state->prx_db.ps_cross_talk + PA22A_PRX_NEAR_DIFF;
                ps_low_threshold = state->prx_db.ps_cross_talk > 5 ? (state->prx_db.ps_cross_talk-5): 1;
                ALSPRX_MSG_0(LOW,"Far over diff\n");
                
            } else {
                if (state->ps_polling == false)
                    CFG3_data = PA22A_INTR_TYPE | PA22A_PS_SLEEP_400MS | PA22A_ALS_SLEEP_TIME;
                else {
                    delay_count++;
                    ALSPRX_MSG_1(LOW,"Delay count:%d\n",delay_count);
                }
            }*/

        } else{
            state->prx_db.event = PRX_FAR_AWAY;
            ps_high_threshold = state->prx_db.ps_cross_talk + PA22A_PRX_NEAR_DIFF;
            ps_low_threshold = state->prx_db.ps_cross_talk > 5 ? (state->prx_db.ps_cross_talk-5):1;
        }
    }    

exit_prx_binary:
    
    state->prx_db.ps_high_threshold = ps_high_threshold <= PA22A_PRX_FAR_MAX? ps_high_threshold: PA22A_PRX_FAR_MAX;
    state->prx_db.ps_low_threshold = ps_low_threshold <= PA22A_PRX_FAR_MAX? ps_low_threshold: PA22A_PRX_FAR_MAX - PA22A_PRX_FAR_DIFF;
    printk("[wjwind]: %s update ps_high_thd_val = %d, ps_low_thd_val = %d cross_talk = %d \n", __FUNCTION__,state->prx_db.ps_high_threshold, state->prx_db.ps_low_threshold, state->prx_db.ps_cross_talk);
    
    
    if (state->ps_polling == false) {        
        /*Update new thresholds*/
        i2c_write_byte(PA22A_CFG3_ADDR, CFG3_data);
        txc_set_ps_high_thres(state->prx_db.ps_high_threshold);
        txc_set_ps_low_thres(state->prx_db.ps_low_threshold);
        txc_clear_interrupt(TXC_SENSOR_PROXIMITY);
	//add by wangjun begin 
		ps_threshold_5cm = (state->prx_db.ps_low_threshold + state->prx_db.ps_cross_talk) + 10;  // wangjun@wimd-mobi.com 20170823  
	//add by wangjun end
    }

    if (!oil_occurred)
        ALSPRX_MSG_5(LOW,"Samples : {%d %d %d %d %d}\n", ps_seq_far.ps_sample[4], ps_seq_far.ps_sample[3], ps_seq_far.ps_sample[2], ps_seq_far.ps_sample[1], ps_seq_far.ps_sample[0]);
    else
        ALSPRX_MSG_5(LOW,"Oil Samples : {%d %d %d %d %d}\n", ps_seq_oil.ps_sample[4], ps_seq_oil.ps_sample[3], ps_seq_oil.ps_sample[2], ps_seq_oil.ps_sample[1], ps_seq_oil.ps_sample[0]);

    ALSPRX_MSG_5(MED,"CT:%d, LT:%d, HT:%d, PS:%d, Event:%d\n",  state->prx_db.ps_cross_talk, state->prx_db.ps_low_threshold,
                                                            state->prx_db.ps_high_threshold,
                                                            ps_data, state->prx_db.event);
    ALSPRX_MSG_2(MED,"Oil_occurred:%d, Saturation:%d\n",oil_occurred, saturation_flag);
                                                            
    return status;

}//function end

int pa22a_read_als(alsprx_state* state)
{
    uint16_t als_adc = 0;
    uint8_t gain = 0;
    int res = 0;
    res = i2c_read_buf(PA22A_ADATAL_ADDR, 2,(uint8_t *) &als_adc);
    state->als_db.als_data = als_adc;
    
    i2c_read_byte(PA22A_CFG0_ADDR, &gain);
    state->als_db.als_gain = gain & 0xF0;

    if(state->enable_mode & PA22A_PRX_ENABLE)
        state->als_db.als_mlux = (state->als_db.als_data * state->als_db.als_caliLUX)/als_adc_coefs_IR[state->als_db.als_gain >> 4];
    else
        state->als_db.als_mlux = (state->als_db.als_data * state->als_db.als_caliLUX)/als_adc_coefs[state->als_db.als_gain >> 4];

    pa22a_process_als_data(state);

    // wangjun@wind-mobi.com 20170728 begin 
	// APS_LOG("[wjwind] als original value = %d\n",state->als_db.als_mlux);
    state->als_db.als_mlux = filter_als_value((u16)state->als_db.als_mlux);
    //ps_threshold_5cm = (state->prx_db.ps_low_threshold + state->prx_db.ps_cross_talk) + 10;  // wangjun@wimd-mobi.com 20170823  
    //APS_ERR("[wjwind] ps_threshold_5cm = %d !\n",ps_threshold_5cm);
    //APS_LOG("[wjwind] als filter value = %d\n",state->als_db.als_mlux);
    // wangjun@wind-mobi.com 20170728 end  
    
    ALSPRX_MSG_2(HIGH,"ALS COUNT:%d, MLUX:%d\n",state->als_db.als_data,state->als_db.als_mlux);
    ALSPRX_MSG_1(HIGH,"ALS Gain:%d\n",state->als_db.als_gain);
    return res;
}

int pa22a_read_ps(alsprx_state* state)
{    
    uint8_t ps_adc = 0;    
    int res = 0;
    res = i2c_read_byte(PA22A_PDATA_ADDR, &ps_adc);
    state->prx_db.prx_data = ps_adc;    
    return res;
}

int pa22a_set_ps_threshold(uint16_t low_thres,uint16_t high_thres)
{    
    int res = 0;
    
    res = i2c_write_byte(PA22A_PILT_ADDR, (uint8_t)low_thres);    
    res = i2c_write_byte(PA22A_PIHT_ADDR, (uint8_t)high_thres);    

    return res;
}

int pa22a_set_register(uint8_t addr,uint8_t data)
{    
    return i2c_write_byte(addr, data);
}

int pa22a_get_deviceid(alsprx_state* state)
{
    int res;
    uint8_t id;
    res = i2c_read_byte(PA22A_DEVICEID_ADDR, &id);
    if ( res < 0 || id != 0x11) {
        ALSPRX_MSG_1(HIGH,"PA22A Device id:%d is error!!\n",id);
        return res;
    }
    ALSPRX_MSG_1(HIGH,"PA22A Device id:%d!!\n",id);
    return id;
}

int pa22a_get_registers_count(void)
{
    return 20;
}
int pa22a_get_vendorid(alsprx_state* state)
{
    return PA22A_I2C_ADDRESS;
}

#if PA22A_ALS_DYNAMIC_RANGE
static int pa22a_setDynamic_Range(alsprx_state* state)
{
	uint8_t cfg0 = 0;
	int actual_als_gain = 0, expected_als_gain=0;
	int status = 0, i;
	static uint32_t mlux_samples[MLUX_SAMPLE_SIZE];
	static uint8_t index = 0;
	uint32_t sum=0;
	
	if(alsSampleInitialize)
	{
		//Initialize the mlux sample array
		for(i=0; i < MLUX_SAMPLE_SIZE; i++)
		{
			mlux_samples[i] = state->als_db.als_mlux;
		}
		alsSampleInitialize = 0;
	}

	mlux_samples[index++] = state->als_db.als_mlux;
	//ALSPRX_MSG_3(HIGH,"mlux_samples={%d,%d,%d}\n",mlux_samples[0],mlux_samples[1],mlux_samples[2]);
	if(index > MLUX_SAMPLE_SIZE){
		index = 0;
	}

	for(i=0; i < MLUX_SAMPLE_SIZE; i++)
	{
		sum += mlux_samples[i];
	}
	state->als_db.als_mlux = sum / MLUX_SAMPLE_SIZE;
	//ALSPRX_MSG_1(HIGH,"AvgLux:%d\n",state->als_db.als_mlux);
	
	actual_als_gain = state->als_db.als_gain;	
	if(state->als_db.als_mlux > ALS_LUX_MIN_X1 && state->als_db.als_mlux <= ALS_LUX_MAX_X1){
		expected_als_gain = PA22A_AGAIN_x1;
		if(expected_als_gain == actual_als_gain)
			return status;
	}else if(state->als_db.als_mlux > ALS_LUX_MIN_X8 && state->als_db.als_mlux <= ALS_LUX_MAX_X8){
		expected_als_gain = PA22A_AGAIN_x8;
		if(expected_als_gain == actual_als_gain)
			return status;
	}else if(state->als_db.als_mlux > ALS_LUX_MIN_X16 && state->als_db.als_mlux <= ALS_LUX_MAX_X16){
		expected_als_gain = PA22A_AGAIN_x16;
		if(expected_als_gain == actual_als_gain)
			return status;
	}else if(state->als_db.als_mlux > ALS_LUX_MIN_X80){
		expected_als_gain = PA22A_AGAIN_x80;
		if(expected_als_gain == actual_als_gain)
			return status;
	}
	
	//Need to change the gain
	status = i2c_read_byte(PA22A_CFG0_ADDR, &cfg0);
	cfg0 = cfg0 & 0xCF;
	cfg0 = cfg0 | expected_als_gain;
	status = i2c_write_byte(PA22A_CFG0_ADDR, cfg0);
	
	state->als_db.als_gain = actual_als_gain = expected_als_gain;

	ALSPRX_MSG_2(HIGH,"Change ALS_GAIN:0x%02x, CalibLux:%d\n",actual_als_gain,state->als_db.als_caliLUX);

	return status;
}
#endif
