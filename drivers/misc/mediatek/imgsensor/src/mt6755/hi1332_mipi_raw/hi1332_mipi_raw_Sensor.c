/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 HI1332mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi1332_mipi_raw_Sensor.h"

#define PFX "HI1332_camera_sensor"

#define LOG_DEBUG(format, args...)	pr_err(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

//#define HI1332_BURST_ENABLE		0
//#define HI1332_OTP

extern void kdSetI2CSpeed(u16 i2cSpeed);
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = HI1332_SENSOR_ID,
	
	.checksum_value = 0x17affdd,//0x5dcf01ae,//0x1dc60471,            //0x6d01485c // Auto Test Mode 추후..
	.pre = {
		.pclk = 480000000,				//record different mode's pclk
		.linelength = 4800,				//record different mode's linelength
		.framelength = 3330,			//record different mode's framelength
		.startx =0,					    //record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},

	.cap = {
		.pclk = 480000000,
		.linelength = 4800,	
		.framelength = 3330,
		.startx = 0,	
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 480000000,
		.linelength = 4800,	
		.framelength = 3330,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
    },
    .cap2 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 480000000/2,
		//.linelength = 9600,	
		//.framelength = 3330,
		
		.linelength = 4800,	
		.framelength = 3330,
		//.linelength = 4800,	
		//.framelength = 6660,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
    },
	.normal_video = {
		.pclk = 480000000,
		.linelength = 4800,	
		.framelength = 3330,
		.startx = 0,	
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
        .pclk = 480000000,
        .linelength = 4800,				
		.framelength = 833,			
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640 ,		
		.grabwindow_height = 480 ,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
	},
    .slim_video = {
		.pclk = 480000000,
		.linelength = 4800,
		.framelength = 833,
		.startx = 0,
		.starty = 0,
    	.grabwindow_width = 1280,
    	.grabwindow_height = 720,
    	.mipi_data_lp2hs_settle_dc = 85,//unit , ns
    	.max_framerate = 1200,
    },

		.margin = 6,
		.min_shutter = 6,
		.max_frame_length = 0xffff,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
    	.ihdr_support = 0,        //1, support; 0,not support
   		.ihdr_le_firstline = 0,   //1,le first ; 0, se first
		.sensor_mode_num = 5,	  //support sensor mode num
	
		.cap_delay_frame = 3, 
		.pre_delay_frame = 3,
		.video_delay_frame = 3,
    	.hs_video_delay_frame = 3,    //enter high speed video  delay frame num
    	.slim_video_delay_frame = 3,  //enter slim video delay frame num
	
		.isp_driving_current = ISP_DRIVING_6MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	    .mipi_settle_delay_mode = 0, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
		.mclk = 24,
		.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x40, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
		.i2c_speed = 400,
};


static imgsensor_struct imgsensor = {
		.mirror = IMAGE_V_MIRROR,			//mirrorflip information
		.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
		.shutter = 0x0100,					//current shutter
		.gain = 0xe0,						//current gain
		.dummy_pixel = 0,					//current dummypixel
		.dummy_line = 0,					//current dummyline
		.current_fps = 300,                 //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
		.autoflicker_en = KAL_FALSE,        //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
 		.test_pattern = KAL_FALSE, 
		.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
 		.ihdr_en = 0, 
		.i2c_write_id = 0x40,
};
 
/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 4240, 3120,    16, 0, 4208, 3120,     2104, 1560,     0, 0, 2104, 1560,     0, 0, 2104, 1560},      // preview (2104 x 1560)
 { 4240, 3120,    16, 0, 4208, 3120,     4208, 3120,     0, 0, 4208, 3120,     0, 0, 4208, 3120},      // capture (4208 x 3120)
 { 4240, 3120,    16, 0, 4208, 3120,     4208, 3120,     0, 0, 4208, 3120,     0, 0, 4208, 3120},      // video   (4208 x 3120)
 { 4240, 2880,     0, 0, 4240, 2880,  	  706,  480,    33, 0,  640,  480,     0, 0,  640,  480},      // hight speed video (640 x 480)
 { 4240, 2160,     0, 0, 4240, 2160,     1414,  720,    67, 0, 1280,  720,     0, 0, 1280,  720},       // slim video (1280 x 720)
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{	
	kal_uint16 get_byte=0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	kdSetI2CSpeed(imgsensor_info.i2c_speed);
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor1D(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
 
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id); 
}

#ifdef HI1332_OTP

extern read_Hi1332_otp_mtk_fmt(void);

kal_uint16 read_Hi1332otp_8(kal_uint32 addr)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed);
    kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C_OTP(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

void write_HynixOtp_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
 
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id); 
}

void write_HynixOtp_sensor_16(kal_uint16 addr, kal_uint16 para)
{
    char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

#endif //HI1332_OTP

static void set_dummy(void)
{

	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0006, imgsensor.frame_length); 
	write_cmos_sensor(0x0008, imgsensor.line_length/8);
	LOG_INF("frame_length = %d, line_length = %d \n", imgsensor.frame_length, imgsensor.line_length);

}	/*	set_dummy  */

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	//kal_uint32 frame_length = 0;

	LOG_INF("E\n");
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else{
			
		    write_cmos_sensor(0x0006, imgsensor.frame_length);
	        }
	} else {
		// Extend frame length
		    write_cmos_sensor(0x0006, imgsensor.frame_length);
	}
	// Update Shutter
    write_cmos_sensor1D(0x0003, (shutter & 0x0F0000) >> 16 );
	write_cmos_sensor1D(0x0004, (shutter & 0x00FF00) >> 8 );
	write_cmos_sensor1D(0x0005, (shutter & 0x0000FF) );
	LOG_INF("shutter =%d, framelength =%d, line_length=%d\n", shutter,imgsensor.frame_length, imgsensor.line_length);

}	/*	write_shutter  */

#ifdef HI1332_BURST_ENABLE // add new code 
static void Hynix_i2c_burst(unsigned short setting_table[], u32 nSetSize)
{
    #define HYNIX_SIZEOF_I2C_BUF 254

 	char Hynix_i2c_buf[HYNIX_SIZEOF_I2C_BUF] = {0,};
 	int err = 0;
 	u32 total_set_count = nSetSize;
	u16 set_count = 0;
 	u16 buf_count = 0;

	for (set_count = 0; set_count < (total_set_count-2); set_count+=2) 
  	{
			if (!buf_count) 
			{ 	
				Hynix_i2c_buf[buf_count] = setting_table[set_count] >> 8; 
				buf_count++;
				Hynix_i2c_buf[buf_count] = setting_table[set_count] & 0xFF; 
				buf_count++;

			}

			Hynix_i2c_buf[buf_count] = setting_table[set_count+1] >> 8; 
			buf_count++;
			Hynix_i2c_buf[buf_count] = setting_table[set_count+1] & 0xFF; 
			buf_count++;
 
			if((setting_table[set_count+2] == setting_table[set_count] + 2) && // check i2c address. if address is add +2 , add data only. 
			(buf_count < HYNIX_SIZEOF_I2C_BUF) )
    			{  
				continue;
			}	
		
			err = iBurstWriteReg(Hynix_i2c_buf, buf_count, imgsensor.i2c_write_id); // i2c write fuction

			if (err)  return err;

			buf_count = 0;
		
	}

	write_cmos_sensor1D(setting_table[total_set_count-2], setting_table[total_set_count-1] >> 8);
	write_cmos_sensor1D(setting_table[total_set_count-2] + 1, setting_table[total_set_count-1]&0xFF);

	return 0;
}
#endif

/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0000;
    reg_gain = gain / 4 - 16;

    return (kal_uint16)reg_gain;

}
/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

    LOG_INF("set_gain is %d\n",gain);
  
	if( gain < BASEGAIN || gain > 16 * BASEGAIN ){
        LOG_INF("Error gain setting");
		if(gain < BASEGAIN )
			gain = BASEGAIN;
		else if( gain > 16 * BASEGAIN )
			gain = 16 * BASEGAIN;
	}
    
	reg_gain = gain2reg(gain);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; // set current gain to global 
	spin_unlock(&imgsensor_drv_lock);

    reg_gain = reg_gain & 0x00FF;
    write_cmos_sensor(0x003A,reg_gain);


	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_DEBUG("image_mirror = %d", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0000, 0x0000);
			write_cmos_sensor(0x003c, 0x0000);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0000, 0x0100);
			write_cmos_sensor(0x003c, 0x0001);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0000, 0x0200);
			write_cmos_sensor(0x003c, 0x0002);
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0000, 0x0300);
			write_cmos_sensor(0x003c, 0x0003);
			break;
		default:
			LOG_DEBUG("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/


/*************************************************************************
* FUNCTION
*	Initial, preview, capture(normal_video), hs_video, slim_video
*
* DESCRIPTION
*	Normal write setting
*
* RETURNS
*	None
*
*************************************************************************/

void hi1332_init_setting_normal(void)
{
	
	write_cmos_sensor(0x0a00, 0x0000); //stream off

 //LGE X-Flip
	write_cmos_sensor(0x0000, 0x0100);
	write_cmos_sensor(0x003c, 0x0001);
//	
	write_cmos_sensor(0x2ffe, 0xe000);
	
	write_cmos_sensor(0x3048, 0x0460);
	write_cmos_sensor(0x404a, 0x0000);
	write_cmos_sensor(0x304c, 0x4b0b);
	write_cmos_sensor(0x404c, 0xf808);
	write_cmos_sensor(0x304e, 0x0088);
	write_cmos_sensor(0x404e, 0x0000);
	write_cmos_sensor(0x3050, 0xce80);
	write_cmos_sensor(0x4050, 0x0000);
	write_cmos_sensor(0x0c00, 0x1190);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x0c04, 0x0000);
	write_cmos_sensor(0x0c06, 0x01b0);
	
	write_cmos_sensor(0x0c10, 0x0040);//OB
	write_cmos_sensor(0x0c12, 0x0040);//OB
	write_cmos_sensor(0x0c14, 0x0040);//OB
	write_cmos_sensor(0x0c16, 0x0040);//OB
	write_cmos_sensor(0x0c18, 0x8000);
	write_cmos_sensor(0x0000, 0x0100);
	write_cmos_sensor(0x003c, 0x0001);
	write_cmos_sensor(0x000a, 0x0000);
	write_cmos_sensor(0x0322, 0x0101);
	write_cmos_sensor(0x0012, 0x0000);
	write_cmos_sensor(0x0018, 0x108f);
	write_cmos_sensor(0x0804, 0x0008);
	write_cmos_sensor(0x0008, 0x0258);
	write_cmos_sensor(0x0024, 0x002e);
	write_cmos_sensor(0x002a, 0x003d);
	write_cmos_sensor(0x0026, 0x0040);
	write_cmos_sensor(0x002c, 0x0c6f);
	write_cmos_sensor(0x005c, 0x0202);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0006, 0x0c5d);
	write_cmos_sensor(0x0a0e, 0x0001);
	write_cmos_sensor(0x0a10, 0x400c);
	write_cmos_sensor(0x0a12, 0x1070);
	write_cmos_sensor(0x0a14, 0x0c30);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0004, 0x0bb8);
	write_cmos_sensor(0x0054, 0x012c);
	write_cmos_sensor(0x0a02, 0x0100);
	//write_cmos_sensor(0x0a04, 0x014e);//otp
	write_cmos_sensor(0x0a04, 0x014a);
	write_cmos_sensor(0x0046, 0x0000);
	write_cmos_sensor(0x003a, 0x0000);
	write_cmos_sensor(0x0050, 0x4300); //0x4300 enable, 0x0300 disable of per-frame ctl 
	write_cmos_sensor(0x0722, 0x0300);
	write_cmos_sensor(0x0756, 0x003f);
	write_cmos_sensor(0x005e, 0xf000);
	write_cmos_sensor(0x0060, 0x0000);
	write_cmos_sensor(0x0062, 0x0000);
	write_cmos_sensor(0x0208, 0x0c30);
	write_cmos_sensor(0x021c, 0x0003);
	write_cmos_sensor(0x021e, 0x0535);
	write_cmos_sensor(0x051a, 0x0100);
	write_cmos_sensor(0x0518, 0x0200);
	write_cmos_sensor(0x0900, 0x0300);
	write_cmos_sensor(0x0928, 0x0000);
	write_cmos_sensor(0x0902, 0xc31a);
	write_cmos_sensor(0x0914, 0xc105);
	write_cmos_sensor(0x0916, 0x051a);
	write_cmos_sensor(0x0918, 0x0205);
	write_cmos_sensor(0x091a, 0x0a09);
	write_cmos_sensor(0x091c, 0x0d08);
	write_cmos_sensor(0x091e, 0x0a0a);
	write_cmos_sensor(0x090c, 0x0855);
	write_cmos_sensor(0x090e, 0x0026);
	write_cmos_sensor(0x004c, 0x0100);
	write_cmos_sensor(0x0800, 0x0000);
	
	write_cmos_sensor(0x0a00, 0x0000);	//sleep on
}

void hi1332_preview_setting_normal(void)
{
	write_cmos_sensor(0x0a00, 0x0000);	//Sleep on
	write_cmos_sensor(0x404c, 0xf828); 
	write_cmos_sensor(0x000a, 0x0000);
	write_cmos_sensor(0x0012, 0x0000); 
	write_cmos_sensor(0x0018, 0x108f); 
	write_cmos_sensor(0x0804, 0x0004); 
	write_cmos_sensor(0x0024, 0x002c); 
	write_cmos_sensor(0x002a, 0x003b); 
	write_cmos_sensor(0x0026, 0x0040); 
	write_cmos_sensor(0x002c, 0x0c6f); 
	write_cmos_sensor(0x005c, 0x0404); 
	write_cmos_sensor(0x002e, 0x3311); 
	write_cmos_sensor(0x0032, 0x3311); 
	write_cmos_sensor(0x0a0e, 0x0002); 
	write_cmos_sensor(0x0a12, 0x0838); 
	write_cmos_sensor(0x0a14, 0x0618); 
	write_cmos_sensor(0x0062, 0x0000); 
	//write_cmos_sensor(0x0a04, 0x016e);//otp
	write_cmos_sensor(0x0a04, 0x016a);
	
	write_cmos_sensor(0x0050, 0x4300); //0x4300 enable, 0x0300 disable of per-frame ctl 
	write_cmos_sensor(0x0722, 0x0301); 
	write_cmos_sensor(0x0756, 0x003f); 
	write_cmos_sensor(0x0208, 0x0618); 
	write_cmos_sensor(0x0006, 0x0D02); 
	write_cmos_sensor(0x0008, 0x0258); 
	write_cmos_sensor(0x0928, 0x0100); 
	write_cmos_sensor(0x0914, 0xc105); 
	write_cmos_sensor(0x0916, 0x030b); 
	write_cmos_sensor(0x0918, 0x0203); 
	write_cmos_sensor(0x091a, 0x0505); 
	write_cmos_sensor(0x091c, 0x0603); 
	write_cmos_sensor(0x091e, 0x0506); 
	write_cmos_sensor(0x090c, 0x041d); 
	write_cmos_sensor(0x090e, 0x0008);
	
	write_cmos_sensor(0x0800, 0x0400); //fmt ctrl	
	
	write_cmos_sensor(0x0a00, 0x0100);	//sleep off         
}

static void hi1332_capture_setting_normal(kal_uint16 currefps)
{
	write_cmos_sensor(0x0a00, 0x0000);	//sleep on
	write_cmos_sensor(0x404c, 0xf808);
	write_cmos_sensor(0x000a, 0x0000); 
	write_cmos_sensor(0x0012, 0x0000);
	write_cmos_sensor(0x0018, 0x108f);
	write_cmos_sensor(0x0804, 0x0008);
	write_cmos_sensor(0x0024, 0x002e);
	write_cmos_sensor(0x002a, 0x003d);
	write_cmos_sensor(0x0026, 0x0040);
	write_cmos_sensor(0x002c, 0x0c6f);
	write_cmos_sensor(0x005c, 0x0202);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0a0e, 0x0001);
	write_cmos_sensor(0x0a12, 0x1070);
	write_cmos_sensor(0x0a14, 0x0c30);
	write_cmos_sensor(0x0062, 0x0000);
	//write_cmos_sensor(0x0a04, 0x014e);//otp
	write_cmos_sensor(0x0a04, 0x014a);
	write_cmos_sensor(0x0050, 0x4300); //0x4300 enable, 0x0300 disable of per-frame ctl 
	write_cmos_sensor(0x0722, 0x0300);
	write_cmos_sensor(0x0756, 0x003f); 
	write_cmos_sensor(0x0208, 0x0c30);
	write_cmos_sensor(0x0006, 0x0D02);
	write_cmos_sensor(0x0008, 0x0258);

/////
if(currefps==150)
{
	LOG_DEBUG("capture 15fps : frame_length = %x, line_length = %x \n", imgsensor.frame_length, imgsensor.line_length);

	write_cmos_sensor(0x304c, 0x250b);//0x25 = 34

//write_cmos_sensor(0x0006, imgsensor.frame_length); 
//write_cmos_sensor(0x0008, imgsensor.line_length);
}
else
{
		LOG_DEBUG("capture not 15fps : frame_length = %x, line_length = %x \n", imgsensor.frame_length, imgsensor.line_length);
	
		write_cmos_sensor(0x304c, 0x4b0b);//0x4b 75
	
	//write_cmos_sensor(0x0006, imgsensor.frame_length); 
	//write_cmos_sensor(0x0008, imgsensor.line_length);

}

///

	
	write_cmos_sensor(0x0928, 0x0000);
	write_cmos_sensor(0x0914, 0xc105);
	write_cmos_sensor(0x0916, 0x0514);
	write_cmos_sensor(0x0918, 0x0205);
	write_cmos_sensor(0x091a, 0x0508);
	write_cmos_sensor(0x091c, 0x0c06);
	write_cmos_sensor(0x091e, 0x0a0a);
	write_cmos_sensor(0x090c, 0x0855);
	write_cmos_sensor(0x090e, 0x0026);
	
	write_cmos_sensor(0x0800, 0x0000);
	
	write_cmos_sensor(0x0a00, 0x0100);	//sleep off
}

void hi1332_hs_video_setting_normal(void)
{
	write_cmos_sensor(0x0a00, 0x0000);	// sleep on
	write_cmos_sensor(0x404c, 0xf888);
	write_cmos_sensor(0x000a, 0x0000); 
	write_cmos_sensor(0x0012, 0x0000);
	write_cmos_sensor(0x0018, 0x108f);
	write_cmos_sensor(0x0804, 0x0020);
	write_cmos_sensor(0x0024, 0x009C);
	write_cmos_sensor(0x002a, 0x00ab);
	write_cmos_sensor(0x0026, 0x00b8);
	write_cmos_sensor(0x002c, 0x0bf7);
	write_cmos_sensor(0x005c, 0x040c);
	write_cmos_sensor(0x002e, 0x3311);
	write_cmos_sensor(0x0032, 0x6666);
	write_cmos_sensor(0x0a0e, 0x0006);
	write_cmos_sensor(0x0a12, 0x0280);
	write_cmos_sensor(0x0a14, 0x01e0);
	write_cmos_sensor(0x0062, 0x0000);	
	//write_cmos_sensor(0x0a04, 0x016e);//otp
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x0050, 0x4720); 	//0x4720 enable, 0x0720 disable of per-frame ctl 
	write_cmos_sensor(0x0722, 0x0301);
	write_cmos_sensor(0x0756, 0x003f); 
	write_cmos_sensor(0x0208, 0x01e0);
	write_cmos_sensor(0x0006, 0x0341);
	write_cmos_sensor(0x0008, 0x0258);
	
	write_cmos_sensor(0x0928, 0x0500);
	write_cmos_sensor(0x0914, 0xc102);
	write_cmos_sensor(0x0916, 0x0103);
	write_cmos_sensor(0x0918, 0x0201);
	write_cmos_sensor(0x091a, 0x0202);
	write_cmos_sensor(0x091c, 0x0501);
	write_cmos_sensor(0x091e, 0x0102);
	write_cmos_sensor(0x090c, 0x015b);
	write_cmos_sensor(0x090e, 0x0004);
	
	write_cmos_sensor(0x0800, 0x1400); 
	
	write_cmos_sensor(0x0a00, 0x0100);	//sleep off
}

void hi1332_slim_video_setting_normal(void)
{
	write_cmos_sensor(0x0a00, 0x0000);	//sleep on
	write_cmos_sensor(0x404c, 0xf848);
	write_cmos_sensor(0x000a, 0x0000); 
	write_cmos_sensor(0x0012, 0x0000);
	write_cmos_sensor(0x0018, 0x108f);
	write_cmos_sensor(0x0804, 0x0040);
	write_cmos_sensor(0x0024, 0x020a);
	write_cmos_sensor(0x002a, 0x0219);
	write_cmos_sensor(0x0026, 0x0220);
	write_cmos_sensor(0x002c, 0x0a8f);
	write_cmos_sensor(0x005c, 0x0206);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0032, 0x3333);
	write_cmos_sensor(0x0a0e, 0x0003);
	write_cmos_sensor(0x0a12, 0x0500);
	write_cmos_sensor(0x0a14, 0x02d0);
	write_cmos_sensor(0x0062, 0x0000);
	//write_cmos_sensor(0x0a04, 0x016e);//otp
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x0050, 0x4720); 	//0x4720 enable, 0x0720 disable of per-frame ctl 
	write_cmos_sensor(0x0722, 0x0300);
	write_cmos_sensor(0x0756, 0x003f); 
	write_cmos_sensor(0x0208, 0x02d0);
	write_cmos_sensor(0x0006, 0x0341);
	write_cmos_sensor(0x0008, 0x0258);
	
	write_cmos_sensor(0x0928, 0x0200);
	write_cmos_sensor(0x0914, 0xc103);
	write_cmos_sensor(0x0916, 0x0207);
	write_cmos_sensor(0x0918, 0x0202);
	write_cmos_sensor(0x091a, 0x0304);
	write_cmos_sensor(0x091c, 0x0803);
	write_cmos_sensor(0x091e, 0x0303);
	write_cmos_sensor(0x090c, 0x02bf);
	write_cmos_sensor(0x090e, 0x0019);
	
	write_cmos_sensor(0x0800, 0x0800);

	write_cmos_sensor(0x0a00, 0x0100);	//sleep off
}

/*************************************************************************
* FUNCTION
*	Initial, preview, capture(normal_video), hs_video, slim_video
*
* DESCRIPTION
*	Burst write setting
*
* RETURNS
*	None
*
*************************************************************************/

unsigned short hi1332_init_setting[] =  {

0x0a00, 0x0000, //stream off

0x2ffe, 0xe000,

0x3048, 0x0460,
0x404a, 0x0000,
0x304c, 0x4b0b,
0x404c, 0xf808,
0x304e, 0x0088,
0x404e, 0x0000,
0x3050, 0xce80,
0x4050, 0x0000,
0x0c00, 0x1190,
0x0c02, 0x0011,
0x0c04, 0x0000,
0x0c06, 0x01b0,
0x0c10, 0x0040,
0x0c12, 0x0040,
0x0c14, 0x0040,
0x0c16, 0x0040,
0x0c18, 0x8000,
0x0000, 0x0100,
0x003c, 0x0001,
0x000a, 0x0000,
0x0322, 0x0101,
0x0012, 0x0000,
0x0018, 0x108f,
0x0804, 0x0008,
0x0008, 0x0258,
0x0024, 0x002e,
0x002a, 0x003d,
0x0026, 0x0040,
0x002c, 0x0c6f,
0x005c, 0x0202,
0x002e, 0x1111,
0x0032, 0x1111,
0x0006, 0x0c5d,
0x0a0e, 0x0001,
0x0a10, 0x400c,
0x0a12, 0x1070,
0x0a14, 0x0c30,
0x003e, 0x0000,
0x0004, 0x0bb8,
0x0054, 0x012c,
0x0a02, 0x0100,
0x0a04, 0x014e,
0x0046, 0x0000,
0x003a, 0x0000,
0x0050, 0x4300,   //0x4300 enable, 0x0300 disable of per-frame ctl 
0x0722, 0x0300,
0x0756, 0x003f,
0x005e, 0xf000,
0x0060, 0x0000,
0x0062, 0x0000,
0x0208, 0x0c30,
0x021c, 0x0003,
0x021e, 0x0535,
0x051a, 0x0100,
0x0518, 0x0200,
0x0900, 0x0300,
0x0928, 0x0000,
0x0902, 0xc31a,
0x0914, 0xc105,
0x0916, 0x051a,
0x0918, 0x0205,
0x091a, 0x0a09,
0x091c, 0x0d08,
0x091e, 0x0a0a,
0x090c, 0x0855,
0x090e, 0x0026,
0x004c, 0x0100,
0x0800, 0x0000,

0x0a00, 0x0000,	//sleep on

};

unsigned short hi1332_preview_setting[] = {  
	
0x0a00, 0x0000,	//Sleep on
	
//0x0000, 0x0100, 
//0x003c, 0x0001, 
0x404c, 0xf828, 
0x0012, 0x0000, 
0x0018, 0x108f, 
0x0804, 0x0004, 
0x0024, 0x002c, 
0x002a, 0x003b, 
0x0026, 0x0040, 
0x002c, 0x0c6f, 
0x005c, 0x0404, 
0x002e, 0x3311, 
0x0032, 0x3311, 
0x0a0e, 0x0002, 
0x0a12, 0x0838, 
0x0a14, 0x0618, 
0x0062, 0x0000, 
0x0a04, 0x016e, 
0x0050, 0x4300,   //0x4300 enable, 0x0300 disable of per-frame ctl 
0x0722, 0x0301, 
0x0208, 0x0618, 
0x0006, 0x0D02, 
0x0008, 0x0258, 
0x0928, 0x0100, 
0x0914, 0xc105, 
0x0916, 0x030b, 
0x0918, 0x0203, 
0x091a, 0x0505, 
0x091c, 0x0603, 
0x091e, 0x0506, 
0x090c, 0x041d, 
0x090e, 0x0008,

0x0800, 0x0400, //fmt ctrl	

0x0a00, 0x0100,	//sleep off

};

unsigned short hi1332_capture_setting[] = {
	
0x0a00, 0x0000,	//sleep on

//0x0000, 0x0100,
//0x003c, 0x0001,
0x404c, 0xf808,
0x0012, 0x0000,
0x0018, 0x108f,
0x0804, 0x0008,
0x0024, 0x002e,
0x002a, 0x003d,
0x0026, 0x0040,
0x002c, 0x0c6f,
0x005c, 0x0202,
0x002e, 0x1111,
0x0032, 0x1111,
0x0a0e, 0x0001,
0x0a12, 0x1070,
0x0a14, 0x0c30,
0x0062, 0x0000,
0x0a04, 0x014e,
0x0050, 0x4300,   //0x4300 enable, 0x0300 disable of per-frame ctl 
0x0722, 0x0300,
0x0208, 0x0c30,
0x0006, 0x0D02,
0x0008, 0x0258,

0x0928, 0x0000,
0x0914, 0xc105,
0x0916, 0x0514,
0x0918, 0x0205,
0x091a, 0x0507,
0x091c, 0x0c06,
0x091e, 0x0a0a,
0x090c, 0x0855,
0x090e, 0x0026,

0x0800, 0x0000,

0x0a00, 0x0100,	//sleep off
};

unsigned short hi1332_hs_video_setting[]= {
	
0x0a00, 0x0000,	// sleep on

//0x0000, 0x0100,
//0x003c, 0x0001,
0x404c, 0xf888,
0x0012, 0x0000,
0x0018, 0x108f,
0x0804, 0x0020,
0x0024, 0x009C,
0x002a, 0x00ab,
0x0026, 0x00b8,
0x002c, 0x0bf7,
0x005c, 0x040c,
0x002e, 0x3311,
0x0032, 0x6666,
0x0a0e, 0x0006,
0x0a12, 0x0280,
0x0a14, 0x01e0,
0x0062, 0x0000,
0x0a04, 0x016e,
0x0050, 0x4720,   //0x4720 enable, 0x0720 disable of per-frame ctl 
0x0722, 0x0301,
0x0208, 0x01e0,
0x0006, 0x0341,
0x0008, 0x0258,

0x0928, 0x0500,
0x0914, 0xc102,
0x0916, 0x0103,
0x0918, 0x0201,
0x091a, 0x0202,
0x091c, 0x0501,
0x091e, 0x0102,
0x090c, 0x015b,
0x090e, 0x0004,

0x0800, 0x1400, 

0x0a00, 0x0100,	//sleep off

};

unsigned short hi1332_slim_video_setting[] = {
	
0x0a00, 0x0000,	//sleep on
	
//0x0000, 0x0100,
//0x003c, 0x0001,
0x404c, 0xf848,
0x0012, 0x0000,
0x0018, 0x108f,
0x0804, 0x0040,
0x0024, 0x020a,
0x002a, 0x0219,
0x0026, 0x0220,
0x002c, 0x0a8f,
0x005c, 0x0206,
0x002e, 0x1111,
0x0032, 0x3333,
0x0a0e, 0x0003,
0x0a12, 0x0500,
0x0a14, 0x02d0,
0x0062, 0x0000,
0x0a04, 0x016e,
0x0050, 0x4720,    //0x4720 enable, 0x0720 disable of per-frame ctl 
0x0722, 0x0300,
0x0208, 0x02d0,
0x0006, 0x0341,
0x0008, 0x0258,

0x0928, 0x0200,
0x0914, 0xc103,
0x0916, 0x0207,
0x0918, 0x0202,
0x091a, 0x0304,
0x091c, 0x0803,
0x091e, 0x0303,
0x090c, 0x02bf,
0x090e, 0x0019,

0x0800, 0x0800,

0x0a00, 0x0100,	//sleep off

};


/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	LOG_DEBUG("E\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
			spin_unlock(&imgsensor_drv_lock);
			do {
				*sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
				
				if (*sensor_id == imgsensor_info.sensor_id) {				
					LOG_DEBUG("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
#ifdef HI1332_OTP

					LOG_DEBUG("get_imgsensor_id -> call read_Hi1332_otp_mtk_fmt func\n");

                    read_Hi1332_otp_mtk_fmt();
#endif 
                    return ERROR_NONE;
			}	
				LOG_DEBUG("get_imgsensor_id Read sensor id fail, id: 0x%x 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				retry--;
			} while(retry > 0);
			i++;
			retry = 2;
}
    
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 

	LOG_DEBUG("E\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		LOG_DEBUG("SP\n");
		do {
			sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_DEBUG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  

#ifdef HI1332_OTP
					LOG_DEBUG("open -> call read_Hi1332_otp_mtk_fmt func\n");
                    read_Hi1332_otp_mtk_fmt();
#endif 				
				break;
			}	
			LOG_DEBUG("Read sensor id fail, i2c_write_id 0x%x id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}	
    
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
#ifdef HI1332_BURST_ENABLE
  	Hynix_i2c_burst(hi1332_init_setting, sizeof(hi1332_init_setting) / 2);
#else
	hi1332_init_setting_normal();
#endif

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
   	imgsensor.test_pattern = KAL_FALSE;
  
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_DEBUG("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//u16 nLscRegData = 0;
	LOG_DEBUG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

#ifdef HI1332_BURST_ENABLE
	Hynix_i2c_burst(hi1332_preview_setting, sizeof( hi1332_preview_setting) / 2 );    
#else
	hi1332_preview_setting_normal();
#endif
	set_mirror_flip(imgsensor.mirror);
	
	return ERROR_NONE;
}	/*	preview   */


static void capture_setting(kal_uint16 currefps)
{
  LOG_DEBUG(" capture, currefps = %d\n",currefps);  

#ifdef HI1332_BURST_ENABLE
  	LOG_DEBUG("burst enable\n");
	Hynix_i2c_burst(hi1332_capture_setting, sizeof(hi1332_capture_setting) / 2);
#else
  	LOG_DEBUG("burst disable\n");
	hi1332_capture_setting_normal(currefps);
#endif
}

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_DEBUG("E\n");
    
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}  else if(imgsensor.current_fps == imgsensor_info.cap2.max_framerate){
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_DEBUG("Warning: current_fps %d fps is not support, so use cap2's setting: %d fps!\n",imgsensor.current_fps, imgsensor_info.cap2.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_DEBUG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps, imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;        
	}
	spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps); 
	set_mirror_flip(imgsensor.mirror);
	
	LOG_DEBUG("capture: frame_length = %d, line_length = %d \n", imgsensor.frame_length, imgsensor.line_length);
	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E");
	LOG_DEBUG("Normal_Video\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	//normal_video_setting(imgsensor.current_fps);

#ifdef HI1332_BURST_ENABLE
	Hynix_i2c_burst(hi1332_capture_setting, sizeof(hi1332_capture_setting) / 2);
#else
	hi1332_capture_setting_normal(imgsensor.current_fps);
#endif
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_DEBUG("E\n");
    
	spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

#ifdef HI1332_BURST_ENABLE
    Hynix_i2c_burst(hi1332_hs_video_setting, sizeof(hi1332_hs_video_setting) / 2);	//[mook] [20160217] 640 480 사용한다  
#else
	hi1332_hs_video_setting_normal();
#endif
	set_mirror_flip(imgsensor.mirror);
	LOG_DEBUG("hs_video: frame_length = %d, line_length = %d \n", imgsensor.frame_length, imgsensor.line_length);

    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_DEBUG("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

#ifdef HI1332_BURST_ENABLE
    Hynix_i2c_burst(hi1332_slim_video_setting, sizeof(hi1332_slim_video_setting) / 2);	
#else
	hi1332_slim_video_setting_normal();
#endif
	set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */


static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->PDAF_Support = 0;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("[contrlo]scenario_id = %d", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
       		slim_video(image_window, sensor_config_data);
       		break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_DEBUG("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
         //   set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
                frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_DEBUG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            LOG_DEBUG("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
 	UINT16 enable_TP = 0; 
	enable_TP = ((read_cmos_sensor(0x0A04) << 8) | read_cmos_sensor(0x0A05));

	LOG_INF("enable: %d", enable);
    // 0x0A05[0]: 1 enable,  0 disable               
    // 0x020A[1:0]; 0x01:BLACK, 0x02:COLOR BAR, 0x03:GREY, 0x04:RANDOM
    
	if (enable) {
        enable_TP |= 0x0001;  
        enable_TP &= ~0x0002; 
	    LOG_INF("mook 0x0A04: 0x%x\n", enable_TP);

        write_cmos_sensor(0x0A04,enable_TP);
		write_cmos_sensor1D(0x020A,0x02);

	} else {
        enable_TP &= 0xFFFE;  
        LOG_INF("mook 0x0A04: 0x%x\n", enable_TP);
        write_cmos_sensor(0x0A04,enable_TP);
		write_cmos_sensor1D(0x020A,0x00);
	}	 
	return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;

    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
                     set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
                     night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:
                     set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			LOG_INF("write addr= %x, data = %x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			LOG_INF("read addr = %x, data = %x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;


        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;

        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;

        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
	    break;

        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_SET_HDR_SHUTTER:
            LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */



#if 0 // back up original code 
static void Hynix_i2c_burst(unsigned short setting_table[], u32 nSetSize)
{

    #define HYNIX_SIZEOF_I2C_BUF 8
#define HYNIX_SIZEOF_I2C_BUF 254

 	char Hynix_i2c_buf[HYNIX_SIZEOF_I2C_BUF] = {0,};
 	int err = 0;
 	u32 total_set_count = nSetSize;
	u16 set_count = 0;
 	u16 buf_count = 0;

	for (set_count = 0; set_count < (total_set_count-2); set_count+=2) 
  	{
			if (!buf_count) 
			{ 	
				Hynix_i2c_buf[buf_count] = setting_table[set_count] >> 8; 
				buf_count++;
				Hynix_i2c_buf[buf_count] = setting_table[set_count] & 0xFF; 
				buf_count++;

			}

			Hynix_i2c_buf[buf_count] = setting_table[set_count+1] >> 8; 
			buf_count++;
			Hynix_i2c_buf[buf_count] = setting_table[set_count+1] & 0xFF; 
			buf_count++;
 
			if((setting_table[set_count+2] == setting_table[set_count] + 2) && // check i2c address. if address is add +2 , add data only. 
			(buf_count < HYNIX_SIZEOF_I2C_BUF) )
    			{  
				continue;
			}	
		
			err = iWriteRegI2C(Hynix_i2c_buf, buf_count, imgsensor.i2c_write_id); // i2c write fuction

			if (err)  return err;

			buf_count = 0;
		
	}

	write_cmos_sensor1D(setting_table[total_set_count-2], setting_table[total_set_count-1] >> 8);
	write_cmos_sensor1D(setting_table[total_set_count-2] + 1, setting_table[total_set_count-1]&0xFF);

	return 0;
}
#endif 

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control, 
	control,
	close
};

UINT32 HI1332_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	HI1332_MIPI_RAW_SensorInit	*/
