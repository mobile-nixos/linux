#ifndef __TXC_MTK_PLATFORM_H__
#define __TXC_MTK_PLATFORM_H__
#define NV_FILE_PATH    "/data/.txc_nv_file"
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include "linux/types.h"

// wangjun@wind-mobi.com 20170728 begin
#include "alsps_calibrator.h"
// wangjun@wind-mobi.com 20170728 end
#define ERROR   
#define HIGH   
#define MED    
#define LOW    
#define APS_TAG                "[ALS/PS] "
#define ALSPRX_DEBUG
#if defined(ALSPRX_DEBUG)
#define APS_FUN(f)				printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	printk(KERN_INFO APS_TAG fmt, ##args)    

#define ALSPRX_MSG_0(level,msg)                 printk(level  APS_TAG"%s %d : "msg, __FUNCTION__, __LINE__)
#define ALSPRX_MSG_1(level,msg, p1)             printk(level  APS_TAG"%s %d : "msg, __FUNCTION__, __LINE__, p1)
#define ALSPRX_MSG_2(level,msg,p1,p2)           printk(level  APS_TAG"%s %d : "msg, __FUNCTION__, __LINE__, p1,p2)
#define ALSPRX_MSG_3(level,msg,p1,p2,p3)        printk(level  APS_TAG"%s %d : "msg, __FUNCTION__, __LINE__, p1,p2,p3)
#define ALSPRX_MSG_4(level,msg,p1,p2,p3,p4)     printk(level  APS_TAG"%s %d : "msg, __FUNCTION__, __LINE__, p1,p2,p3,p4) 
#define ALSPRX_MSG_5(level,msg,p1,p2,p3,p4,p5)  printk(level  APS_TAG"%s %d : "msg, __FUNCTION__, __LINE__, p1,p2,p3,p4,p5) 

#else
#define APS_FUN(f)				
#define APS_ERR(fmt, args...)	
#define APS_LOG(fmt, args...)	
#define APS_DBG(fmt, args...)	
#define ALSPRX_MSG_0(level,msg)				
#define ALSPRX_MSG_1(level,msg,p1)			
#define ALSPRX_MSG_2(level,msg,p1,p2)		
#define ALSPRX_MSG_3(level,msg,p1,p2,p3)
#define ALSPRX_MSG_4(level,msg,p1,p2,p3,p4)
#define ALSPRX_MSG_5(level,msg,p1,p2,p3,p4,p5)

#endif

#define CHECK_STATUS(status)             \
	if ( status != 0 )						 \
	{										 \
	return status;							 \
	}

/* TXC */
typedef enum
{
    CROSSTALK_CALIBRATION,
    THRESHOLD_CALIBRATION,
    ALL_CALIBRATION
}sensor_calibration_type_e;

typedef enum
{
    TXC_SENSOR_PROXIMITY,
    TXC_SENSOR_AMBIENT,
    TXC_SENSOR_ALSPRX
}sensor_type_e;

/* PRX STATUS */
typedef enum
{
    PRX_NEAR_BY,
    PRX_FAR_AWAY,
    
    PRX_NEAR_BY_UNKNOWN
} dd_prx_nearby_type;

typedef struct
{
	char *name; //Sensor Type
	char *model;	//Model eg PA22A
	char *vendor;	// TXC
	int	 version;
	int  i2c_addr;	// 0x1E
	int deviceId;	// for PA22A 0x11
} device_information;


/* DATA STRUCT FOR ALS */
typedef struct
{  
    uint32_t    als_mlux;
    uint32_t    als_data;  
    uint32_t    als_caliLUX;
	uint32_t	als_gain;
} als_db_type;

typedef struct
{  
    uint32_t    als_mlux;
    uint32_t    als_data[5];
	uint32_t	colorTemp;
} rgb_db_type;

/* DATA STRUCT FOR PRX */
typedef struct
{  
    uint32_t    prx_data;
    uint8_t     ps_cross_talk;
	uint8_t		ps_sunlight_xtalk;
    uint8_t     ps_low_threshold;
    uint8_t     ps_high_threshold;        
    uint8_t     old_event;    
    uint32_t    event;
    uint8_t     delay_count;
} prx_db_type;

	
#define NV_VERSION  3
#define NVFILE_MAX_SIZE 64
/* DATA STRUCT FOR NV ITEM */
typedef struct
{
    uint8_t     nv_size;            /* 100: size of NV item */
    uint8_t     version_num;        /* 101: Version of NV Database */
	uint16_t    Reg0;               /* 102: Reigster 0x00 setting */
    uint16_t    Reg1;               /* 103: Reigster 0x01 setting */
    uint16_t    Reg2;               /* 104: Reigster 0x02 setting */
    uint16_t    Reg3;               /* 105: Reigster 0x03 setting */
    uint16_t    Reg11;              /* 106: Reigster 0x10 setting */
    uint32_t    Reg12;              /* 107: Reigster 0x11 setting */
    uint32_t    Reg13;              /* 108: Reigster 0x12 setting */
    uint32_t    prxXtalk;           /* 109: Proximity Xtalk   */
	uint32_t    prxXtalk_base;      /* 109: Proximity Xtalk   */
	uint8_t     ps_low_threshold;
	uint8_t     ps_high_threshold;
    uint32_t    als_caliLUX;        /* 110: ALS calibration lux */
    uint32_t    als_caliADC_IR[4];     /* 111: ALS calibration value with IR effect */
    uint32_t    als_caliADC[4];        /* 112: ALS calibration value*/

} alsprx_nvdb;

typedef struct 
{
    als_db_type als_db;
    prx_db_type prx_db;
    uint8_t     enable_mode;
    uint32_t    als_odr;    /*ALS ODR*/
    uint32_t    ps_odr;     /*PS ODR*/
    bool        ps_polling;
    bool        als_polling;
    bool        alsps_polling; 
} alsprx_state;

//Device Interface
typedef struct{
    int (*alsprx_set_reg_default)(alsprx_state* state, alsprx_nvdb *nvdb);
    int (*alsprx_enable_als)(alsprx_state* state,int enable);
    int (*alsprx_enable_ps)(alsprx_state* state,int enable);
    int (*alsprx_als_calibration)(alsprx_state* state,alsprx_nvdb* nvdb);
    int (*alsprx_ps_calibration)(alsprx_state* state,alsprx_nvdb* nvdb);
    int (*alsprx_process_als_data)(alsprx_state* state);
    int (*alsprx_process_ps_data)(alsprx_state* state);
    int (*alsprx_read_als)(alsprx_state* state);
    int (*alsprx_read_ps)(alsprx_state* state);
    int (*alsprx_show_registers)(uint8_t* regs_arry, uint8_t arry_size);
    //wangjun@wind-mobi.com 20170927 begin
    int (*alsprx_set_ps_threshold)(alsprx_state* state,uint16_t low_thres,uint16_t high_thres,uint16_t cross_talk);
    //wangjun@wind-mobi.com 20170927 end
    int (*alsprx_set_register)(uint8_t addr,uint8_t data);
    int (*alsprx_get_deviceid)(alsprx_state* state);
    int (*alsprx_get_vendorid)(alsprx_state* state);
	device_information (*alsprx_get_device_info)(sensor_type_e type);
	int (*alsprx_enable_interrupt)(sensor_type_e type, alsprx_state* state, int enable);
	int (*alsprx_clear_interrupt)(sensor_type_e type);
	int (*is_alsprx_enabled)(sensor_type_e type);
	int (*alsprx_get_default_nv)(alsprx_nvdb* nvdb);
	int (*alsprx_get_registers_count)(void);
	int (*alsprx_ps_calibrate_specific)(alsprx_nvdb* nvdb, sensor_calibration_type_e cal_type);
} alsprx_interface;
void* device_get_interface(void);

extern int i2c_write_byte(uint8_t reg, uint8_t byte_val);
extern int i2c_read_byte(uint8_t reg, uint8_t* byte_val);
extern int i2c_read_buf(uint8_t reg, uint8_t byteCnt, uint8_t* buf);
extern void ms_delay(uint32_t nsDelay);

/*----------------------------------------------------------------------------*/
enum TXC_ALSPRX_NOTIFY_TYPE {
	TXC_ALSPRX_NOTIFY_PROXIMITY_CHANGE = 1,
};
/*----------------------------------------------------------------------------*/
enum TXC_ALSPRX_CUST_ACTION {
	TXC_ALSPRX_CUST_ACTION_SET_CUST = 1,
	TXC_ALSPRX_CUST_ACTION_CLR_CALI,
	TXC_ALSPRX_CUST_ACTION_SET_CALI,
	TXC_ALSPRX_CUST_ACTION_SET_PS_THRESHODL,
	TXC_ALSPRX_CUST_ACTION_SET_EINT_INFO,
	TXC_ALSPRX_CUST_ACTION_GET_ALS_RAW_DATA,
	TXC_ALSPRX_CUST_ACTION_GET_PS_RAW_DATA,
};
/*----------------------------------------------------------------------------*/
struct TXC_ALSPRX_CUST {
	uint16_t	action;
};
/*----------------------------------------------------------------------------*/
struct TXC_ALSPRX_SET_CUST {
	uint16_t action;
	uint16_t part;
	int32_t	data[0];
};
/*----------------------------------------------------------------------------*/
struct TXC_ALSPRX_SET_CALI {
	uint16_t	action;
	uint8_t    	Reg0;               /* 102: Reigster 0x00 setting */
    uint8_t    	Reg1;               /* 103: Reigster 0x01 setting */
    uint8_t   	Reg2;               /* 104: Reigster 0x02 setting */
    uint8_t    	Reg3;               /* 105: Reigster 0x03 setting */
    uint8_t    	Reg11;              /* 106: Reigster 0x10 setting */
    uint8_t    	Reg12;              /* 107: Reigster 0x11 setting */
    uint8_t    	Reg13; 
	uint8_t		ps_low_threshold;
	uint8_t		ps_high_threshold;
	uint8_t    	prxXtalk;           /* 109: Proximity Xtalk   */
	uint8_t    	prxXtalk_base;      /* 109: Proximity Xtalk   */
    uint32_t    als_caliLUX;        /* 110: ALS calibration lux */
    uint32_t    als_caliADC_IR;     /* 111: ALS calibration value with IR effect */
    uint32_t    als_caliADC;        /* 112: ALS calibration value*/
};
/*----------------------------------------------------------------------------*/
struct TXC_ALSPRX_SET_PS_THRESHOLD {
	uint16_t	action;
	int32_t	 threshold[2];
};
/*----------------------------------------------------------------------------*/
struct TXC_ALSPRX_SET_EINT_INFO {
	uint16_t	action;
	uint32_t	gpio_pin;
	uint32_t	gpio_mode;
	uint32_t	eint_num;
	uint32_t	eint_is_deb_en;
	uint32_t	eint_type;
};
/*----------------------------------------------------------------------------*/
struct TXC_ALSPRX_GET_ALS_RAW_DATA {
	uint16_t	action;
	uint16_t	als;
};
/*----------------------------------------------------------------------------*/
struct TXC_ALSPRX_GET_PS_RAW_DATA {
	uint16_t	action;
	uint16_t	ps;
};
/*----------------------------------------------------------------------------*/
union TXC_ALSPRX_CUST_DATA {
	uint32_t					data[10];
	struct TXC_ALSPRX_CUST				cust;
	struct TXC_ALSPRX_SET_CUST			setCust;
	struct TXC_ALSPRX_CUST			clearCali;
	struct TXC_ALSPRX_SET_CALI			setCali;
	struct TXC_ALSPRX_SET_PS_THRESHOLD	setPSThreshold;
	struct TXC_ALSPRX_SET_EINT_INFO	   setEintInfo;
	struct TXC_ALSPRX_GET_ALS_RAW_DATA	getALSRawData;
	struct TXC_ALSPRX_GET_PS_RAW_DATA	 getPSRawData;
};
/*----------------------------------------------------------------------------*/

extern struct platform_device *get_alsps_platformdev(void);
#endif
