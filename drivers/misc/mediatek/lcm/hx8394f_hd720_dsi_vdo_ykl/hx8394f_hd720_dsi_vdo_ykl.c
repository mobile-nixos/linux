//liukangping@wind-mobi.com 20170314 begin

//#ifdef BUILD_LK
//	#include <platform/mt_gpio.h>
//#elif defined(BUILD_UBOOT)
//	#include <asm/arch/mt_gpio.h>
//#else
//	#include <mach/mt_gpio.h>
//#endif

#ifdef BUILD_LK
#include <platform/gpio_const.h>
#include <platform/mt_gpio.h>
#include <platform/upmu_common.h>
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mt_gpio.h>
    #endif
#endif


#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1440)
//#define FRAME_HEIGHT 										(1280) //liukangping
#define LCM_ID_HX8394                                                              (0x0F)
#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

extern int esd_check_alarm;
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define __SAME_IC_COMPATIBLE__

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)



static struct platform_device * pltfm_dev ;
struct pinctrl *lcmbiasctrl = NULL;
struct pinctrl_state *lcmbias_enable= NULL;
struct pinctrl_state *lcmbias_disable= NULL;
//static struct platform_device * pltfm_dev ; //liukangping

//extern struct pinctrl *lcmbiasctrl; //liukangping
//extern struct pinctrl_state *lcmbias_enable; //liukangping
//extern struct pinctrl_state *lcmbias_disable; //liukangping

static int lcmbias_probe(struct platform_device *dev)
{
	pr_debug("[lcm]lcmbias_probe begin!\n");
	pltfm_dev = dev;
	
	lcmbiasctrl = devm_pinctrl_get(&pltfm_dev->dev);
	if (IS_ERR(lcmbiasctrl)) {
		dev_err(&pltfm_dev->dev, "Cannot find  lcmbias pinctrl!");
		return -1;
	}

	lcmbias_enable = pinctrl_lookup_state(lcmbiasctrl, "lcd_bias_enp1_gpio");
	if (IS_ERR(lcmbias_enable)) {
		pr_err("%s : pinctrl err, lcmbias_enable\n", __func__);
		return -1;
	}

	lcmbias_disable = pinctrl_lookup_state(lcmbiasctrl, "lcd_bias_enp0_gpio");
	if (IS_ERR(lcmbias_disable)) {
		pr_err("%s : pinctrl err, lcmbias_disable\n", __func__);
		return -1;
	}
	pr_debug("[lcm]lcmbias_probe done!\n");
	return 0;
}

static int lcmbias_remove(struct platform_device *dev)
{

	return 0;
}


struct of_device_id lcmbias_of_match[] = {
	{ .compatible = "mediatek,lcmbias", },
	{},
};

static struct platform_driver lcmbias_driver = {
	.probe = lcmbias_probe,
	.remove = lcmbias_remove,
	.driver = {
			.name = "lcmbias_drv",
			.of_match_table = lcmbias_of_match,
		   },
};
static int lcmbias_mod_init(void)
{
	int ret = 0;

	pr_debug("[lcmbias]lcmbias_mod_init begin!\n");
	ret = platform_driver_register(&lcmbias_driver);
	if (ret)
		pr_debug("[lcmbias]platform_driver_register error:(%d)\n", ret);
	else
		pr_debug("[lcmbias]platform_driver_register done!\n");

		pr_debug("[lcmbias]lcmbias_mod_init done!\n");
	return ret;

}

static void lcmbias_mod_exit(void)
{
	pr_debug("[lcmdet]lcmbias_mod_exit\n");
	platform_driver_unregister(&lcmbias_driver);
	pr_debug("[lcmdet]lcmbias_mod_exit Done!\n");
}

module_init(lcmbias_mod_init);
module_exit(lcmbias_mod_exit);


 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
/*
static struct LCM_setting_table lcm_initialization_setting1[] = 
{
    //HX8394F+BOE5.2_20160301
	{0xB9, 3,{0xFF,0x83,0x94}},
	
	//0xSet0xPower,0x                              //0x54
	{0xB1, 11,{0XB1,0x50,0x13,0x73,0x09,0x32,0x44,0x71,0x31,0x55,0x2F}},

	//0xSet0xMIPI,0x
	{0xBA, 6,{0x63,0x03,0x68,0x6B,0xB2,0xC0}},

	//0xSet0xDisplay,0x
	{0xB2, 6,{0x00,0x80,0x64,0x0E,0x0A,0x2F}},

	//0xSet0xCYC,0x
	{0xB4, 22,{0XB4,0x01,0x74,0x01,0x74,0x01,0x74,0x01,0x0C,0x86,0x75,0x00,0x3F,0x01,0x74,0x01,0x74,0x01,0x74,0x01,0x0C,0x86}},

	//0xSet0xD3,0x
	{0xD3, 34,{0XD3,0x00,0x00,0x07,0x07,0x40,0x1E,0x08,0x00,0x32,0x10,0x08,0x00,0x08,0x54,0x15,0x10,0x05,0x04,0x02,0x12,0x10,0x05,0x07,0x23,0x23,0x0C,0x0C,0x27,0x10,0x07,0x07,0x10,0x40}},

	//0xSet0xGIP,0x
	{0xD5, 45,{0XD5,0x19,0x19,0x18,0x18,0x1B,0x1B,0x1A,0x1A,0x04,0x05,0x06,0x07,0x00,0x01,0x02,0x03,0x20,0x21,0x18,0x18,0x22,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

	//0xSet0xD6,0x
	{0xD6, 45,{0XD6,0x18,0x18,0x19,0x19,0x1B,0x1B,0x1A,0x1A,0x03,0x02,0x01,0x00,0x07,0x06,0x05,0x04,0x23,0x22,0x18,0x18,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},

	//0xSet0xVCOM,0x
	//SSD2828_Gen_1A_2P(0xB6,0x4E,0x4E);//3c 3c

	// Set Gamma
	{0xE0, 59,{0XE0,0x00,0x03,0x08,0x0d,0x0e,0x11,0x13,0x12,0x26,0x35,0x48,0x49,0x54,0x69,0x70,0x78,0x87,0x8b,0x89,0x98,0xAa,0x54,0x54,0x58,0x5c,0x5f,0x68,0x76,0x7F,0x00,0x03,0x08,0x0d,0x0e,0x11,0x13,0x12,0x26,0x35,0x48,0x49,0x54,0x69,0x71,0x78,0x87,0x8b,0x89,0x99,0xAa,0x54,0x53,0x57,0x5c,0x5f,0x69,0x76,0x7F}},

	//0xSet0xPanel,0x
	{0xCC, 1,{0x0B}},  //ranyanhao@wind-mobi.com 20160302 modify the display direction

	//0xSet0xC0,0x
	{0xC0, 2,{0x1F,0x31}},

	//0xSet0xD4h,0x
	{0xD4, 1,{0x02}},

	//0xSet0xBD,0x
	{0xBD, 1,{0x01}},

	//0xSet0xGAS,0x
	{0xB1, 1,{0x00}},
	
	//0xSet0xBD,0x
	{0xBD, 1,{0x00}},
	

	{0x11, 1,{0x00}},
	
	{REGFLAG_DELAY, 120, {0}},	
			  
	{0x29, 1,{0x00}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};
*/
#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    //Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    // Sleep Mode On
	
	{0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
	
	
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	//zhangaifeng@wind-mobi.com begin
   // {0xFF,	3,		{0x98,0x81,0x01}},
   // {0x58, 1, {0x01}},
//{REGFLAG_DELAY, 20, {}},
		//zhangaifeng@wind-mobi.com end
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;


		//dingyisheng@wind-mobi.com 20160413 beign
		/*The following two values added are always uesd for CTS check */
		/*in order to match density and screen size*/
		params->physical_width = 65; //modify by qiangang@wind-mobi.com 20170329 from 65 to 62
	    params->physical_height = 130 ; //modify by qiangang@wind-mobi.com 20170329 from 115 to 110
             //dingyisheng@wind-mobi.com 20160413 end

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//LCM_DBI_TE_MODE_DISABLED;
		//LCM_DBI_TE_MODE_VSYNC_ONLY;  
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING; 
		/////////////////////   
		//if(params->dsi.lcm_int_te_monitor)  
		//params->dsi.vertical_frontporch *=2;  
		//params->dsi.lcm_ext_te_monitor= 0;//TRUE; 
	//	params->dsi.noncont_clock= TRUE;//FALSE;   
	//	params->dsi.noncont_clock_period=2;
//		params->dsi.cont_clock=1;
		////////////////////          
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;  
		// DSI    /* Command mode setting */  
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;      
		//The following defined the fomat for data coming from LCD engine.  
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;   
		params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST; 
		params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;    
		params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;       
		// Video mode setting		   
		params->dsi.intermediat_buffer_num = 2;  
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;  
		params->dsi.packet_size=256;    
		// params->dsi.word_count=480*3;	
		//DSI CMD mode need set these two bellow params, different to 6577   
		// params->dsi.vertical_active_line=800;   
		params->dsi.vertical_sync_active				= 2; //4   
		params->dsi.vertical_backporch				       = 16;  //14  
		params->dsi.vertical_frontporch				       = 9;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 30;   //4
		params->dsi.horizontal_backporch				= 45;  //60   liukangping update
		params->dsi.horizontal_frontporch				= 45;    //60  liukangping update
//		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  

	//	params->dsi.HS_TRAIL=14;
	//	params->dsi.pll_div1=1;		   
	//	params->dsi.pll_div2=1;		   
	//	params->dsi.fbk_div =28;//28	
//zhounengwen@wind-mobi.com 20150327 beign
// To fix lcm rf
        params->dsi.PLL_CLOCK = 243; //modify by qiangang
		params->dsi.ssc_disable = 1; //add by qiangang
//		params->dsi.CLK_TRAIL = 17;
//qiangang@wind-mobi.com 20170912 begin	
	    params->dsi.cont_clock=0;   
	    params->dsi.esd_check_enable = 0;
	    params->dsi.customization_esd_check_enable      = 0;
	    params->dsi.lcm_esd_check_table[0].cmd          = 0xd9;
	    params->dsi.lcm_esd_check_table[0].count        = 1;
	    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80; //80

	    params->dsi.lcm_esd_check_table[1].cmd          = 0x09;
	    params->dsi.lcm_esd_check_table[1].count        = 3;
	    params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80; 
	    params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73; 
	    params->dsi.lcm_esd_check_table[1].para_list[2] = 0x06;
		
	 
	    params->dsi.lcm_esd_check_table[2].cmd          = 0x45;
       	    params->dsi.lcm_esd_check_table[2].count        = 2;
	    params->dsi.lcm_esd_check_table[2].para_list[0] = 0x0f;
	    params->dsi.lcm_esd_check_table[2].para_list[1] = 0xf0;
/*
	    params->dsi.lcm_esd_check_table[3].cmd          = 0xE0;
       	    params->dsi.lcm_esd_check_table[3].count        = 10;
	    params->dsi.lcm_esd_check_table[3].para_list[0] = 0x00;
	    params->dsi.lcm_esd_check_table[3].para_list[1] = 0x16;
	    params->dsi.lcm_esd_check_table[3].para_list[2] = 0x24;
	    params->dsi.lcm_esd_check_table[3].para_list[3] = 0x2D;
            params->dsi.lcm_esd_check_table[3].para_list[4] = 0x30;
	    params->dsi.lcm_esd_check_table[3].para_list[5] = 0x35;
	    params->dsi.lcm_esd_check_table[3].para_list[6] = 0x38;
	    params->dsi.lcm_esd_check_table[3].para_list[7] = 0x37;
	    params->dsi.lcm_esd_check_table[3].para_list[8] = 0x71;
	    params->dsi.lcm_esd_check_table[3].para_list[9] = 0x82;
*/
//qiangang@wind-mobi.com 20170912 end
// wangjun@wind-mobi.com 20170828 begin 
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 40;
#endif
// wangjun@wind-mobi.com 20170828 end 

}
void init_lcm_registers(void)
{
unsigned int data_array[16];

data_array[0]=0x00043902;
data_array[1]=0x9483FFB9;
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00073902;
data_array[1]=0x680363BA;
data_array[2]=0x00C0B26B;
dsi_set_cmdq(data_array,3, 1);
//modify by qiangang@wind-mobi.com 20170918 begin
data_array[0]=0x000B3902;
data_array[1]=0x6F0F50B1;
data_array[2]=0x71233209;
data_array[3]=0x00394E51;
dsi_set_cmdq(data_array,4, 1);

data_array[0]=0x00073902;
data_array[1]=0x788000B2;
data_array[2]=0x00220D0E;
dsi_set_cmdq(data_array,3, 1);

data_array[0]=0x00163902;
data_array[1]=0x037203B4;
data_array[2]=0x01720372;
data_array[3]=0x00957C0D;
data_array[4]=0x036B033F;
data_array[5]=0x056B036B;
data_array[6]=0x00007C0D;
dsi_set_cmdq(data_array,7, 1);

data_array[0]=0x003B3902;
data_array[1]=0x0E0500E0;
data_array[2]=0x231E1A16;
data_array[3]=0x74614D23;
data_array[4]=0x8F8D7D74;
data_array[5]=0xA0A29E90;
data_array[6]=0x5F60C2B1;
data_array[7]=0x7F756964;
data_array[8]=0x05007F7F;
data_array[9]=0x1E1A160E;
data_array[10]=0x614D2323;
data_array[11]=0x8D7D7474;
data_array[12]=0xA29E908F;
data_array[13]=0x60C2B1A0;
data_array[14]=0x7569645F;
data_array[15]=0x007F7F7F;
dsi_set_cmdq(data_array,16, 1);

data_array[0]=0x00223902;
data_array[1]=0x000000D3;
data_array[2]=0x08000000;
data_array[3]=0x09103208;
data_array[4]=0x15320900;
data_array[5]=0x32AD05AD;
data_array[6]=0x00000810;
data_array[7]=0x0C0C3337;
data_array[8]=0x27020227;
data_array[9]=0x0000400E;
dsi_set_cmdq(data_array,10, 1);

data_array[0]=0x002D3902;
data_array[1]=0x252627D5;
data_array[2]=0x05060724;
data_array[3]=0x01020304;
data_array[4]=0x18181800;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x18181818;
data_array[10]=0x23202118;
data_array[11]=0x18181822;
data_array[12]=0x00000018;
dsi_set_cmdq(data_array,13, 1);

data_array[0]=0x002D3902;
data_array[1]=0x222120D6;
data_array[2]=0x06050423;
data_array[3]=0x02010007;
data_array[4]=0x18181803;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x18181818;
data_array[10]=0x24272618;
data_array[11]=0x18181825;
data_array[12]=0x00000018;
dsi_set_cmdq(data_array,13, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000BCC;
dsi_set_cmdq(data_array,2, 1);
//add by qiangang begin
data_array[0]=0x00023902;
data_array[1]=0x000033D2;
dsi_set_cmdq(data_array,2, 1);
//add by qiangang end
data_array[0]=0x00033902;
data_array[1]=0x00311FC0;
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00033902;
//modify by qiangang@wind-mobi.com 20170908 begin
data_array[1]=0x002323B6;
//modify by qiangang@wind-mobi.com 20170908 end
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000002D4;
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000001BD;
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000000B1;
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000000BD;
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000EDC6;
dsi_set_cmdq(data_array,2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000035;
dsi_set_cmdq(data_array,2, 1);

data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(120);
//modify by qiangang@wind-mobi.com 20170918 begin
data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(20);

/*
data_array[0] = 0x00043902;
data_array[1] = 0x9483FFB9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x000B3902;
data_array[1] = 0x701050B1;  //0x751550B1 modify by qiangang@wind-mobi.com 20170619
data_array[2] = 0x71443209;
data_array[3] = 0x00274631;  //0x002F5531 modify by qiangang@wind-mobi.com 20170619
dsi_set_cmdq(data_array, 4, 1);
MDELAY(1);

data_array[0] = 0x00073902;
data_array[1] = 0x680363BA;
data_array[2] = 0x00C0B26B;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000088D2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00063902;
data_array[1] = 0x648000B2;
data_array[2] = 0x00000710;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x00163902;
data_array[1] = 0x017401B4;
data_array[2] = 0x01740174;
data_array[3] = 0x0075860C;
data_array[4] = 0x0174013F;
data_array[5] = 0x01740174;
data_array[6] = 0x0000860C;
dsi_set_cmdq(data_array, 7, 1);
MDELAY(1);

data_array[0] = 0x00223902;
data_array[1] = 0x070000D3;
data_array[2] = 0x081E4007;
data_array[3] = 0x08103200;
data_array[4] = 0x15540800;
data_array[5] = 0x02040510;
data_array[6] = 0x07051012;
data_array[7] = 0x0C0C2323;
data_array[8] = 0x07071027;
data_array[9] = 0x00004010;
dsi_set_cmdq(data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x00203902;
data_array[1] = 0x181919D5;
data_array[2] = 0x1A1B1B18;
data_array[3] = 0x0605041A;
data_array[4] = 0x02010007;
data_array[5] = 0x18212003;
data_array[6] = 0x18232218;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
dsi_set_cmdq(data_array, 9, 1);
MDELAY(1);

data_array[0] = 0x002D3902;
data_array[1] = 0x191818D6;
data_array[2] = 0x1A1B1B19;
data_array[3] = 0x0102031A;
data_array[4] = 0x05060700;
data_array[5] = 0x18222304;
data_array[6] = 0x18202118;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x18181818;
data_array[10] = 0x18181818;
data_array[11] = 0x18181818;
data_array[12] = 0x00000018;
dsi_set_cmdq(data_array, 13, 1);
MDELAY(1);

data_array[0] = 0x003B3902;
//modify by qiangang@wind-mobi.com 20170619 begin 
data_array[1] = 0x040000E0;
data_array[2] = 0x0A090706;
data_array[3] = 0x27180C07;
data_array[4] = 0x51483327;
data_array[5] = 0x79786E59;
data_array[6] = 0x5555A68F;
data_array[7] = 0x7867615B;
data_array[8] = 0x00007F7F;
data_array[9] = 0x07060604;
data_array[10] = 0x180C0709;
data_array[11] = 0x48332727;
data_array[12] = 0x786E5A52;
data_array[13] = 0x55A79079;
data_array[14] = 0x68615B55;
data_array[15] = 0x007F7F78;
//modify by qiangang@wind-mobi.com 20170619 end
dsi_set_cmdq(data_array, 16, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000007CC;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x00311FC0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x009292B6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000001BD;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000000B1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000000BD;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x0000EFC6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000002D4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(150);

data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(20);
*/
}

static void lcm_init(void)
{
#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif

#if 1
	if(!IS_ERR(lcmbiasctrl) && !IS_ERR(lcmbias_enable))
	pinctrl_select_state(lcmbiasctrl, lcmbias_enable);  //liukangping
#endif
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(5); //modified by wangjun at 20170928
        if(esd_check_alarm)
        {
                MDELAY(115);
        }
	//printf(" gemingming hx8394f init  \n");
	//push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1); 
        init_lcm_registers();
		  
}

static void lcm_suspend(void) 
{
        printk("hx8394f suspend\n");
	// when phone sleep , config output low, disable backlight drv chip  
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
// zhaozhensen@wind-mobi.com 20160322 begin
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
// zhaozhensen@wind-mobi.com 20160322 end
#endif
#if 1
		if(!IS_ERR(lcmbiasctrl) && !IS_ERR(lcmbias_disable))
		{
			printk("lcmbiasctrl + lcmbias_disable ok");
        	pinctrl_select_state(lcmbiasctrl, lcmbias_disable);  //liukangping
		}
#endif
	MDELAY(10);

}

static void lcm_resume(void)
{
        printk("hx8394f resume\n");
        lcm_init();
}

static unsigned int lcm_compare_id(void)
{
        unsigned int id=0,id1=0;
        // unsigned int id=0,id1=0,id2=0; //liukangping
        unsigned char buffer[3];
        unsigned int data_array[16];  
		 if(!IS_ERR(lcmbiasctrl) && !IS_ERR(lcmbias_enable))
         pinctrl_select_state(lcmbiasctrl, lcmbias_enable);  //liukangping
#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50); 

	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00023902;
	data_array[1]=0x000063ba;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0] = 0x00013700;// return byte number
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xDC, buffer, 1);
	id1= buffer[0]; //should be 0x0F
	//read_reg_v2(0xDB, buffer, 1);
	//id2= buffer[0]; //should be 0x94
    printk(" liukangping  buffer kernel =%x\n", buffer[0]); 
	id= id1;
	
	

#if defined(BUILD_LK)||defined(BUILD_UBOOT)
	printf(" hzs %s id=%x id1=%x id2=%x \n",__func__,id, id1, id2);
#else
	printk("dzl------------- %s id=%x  \n",__func__,id);
#endif	

	if(LCM_ID_HX8394==id)
		return 1;
	else
		return 0;
}
//static int err_count = 0;
static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
    unsigned char buffer[8] = {0};

    unsigned int array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

    array[0] = 0x00013700;    

    dsi_set_cmdq(array, 1,1);

    read_reg_v2(0x0A, buffer,8);

	printk( "ili9881 lcm_esd_check: buffer[0] = %d,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

    if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/

    {

        printk( "ili9881 lcm_esd_check buffer[0] = %d\n",buffer[0]);

        return TRUE;

    }

    else

    {

#if 0
        if(buffer[3] != 0x02) //error data type is 0x02

        {
		//  is not 02, 
             err_count = 0;

        }

        else

        {
		// is 02, so ,
             if((buffer[4] == 0x40) || (buffer[5] == 0x80))
             {
			 // buffer[4] is not 0, || (huo),buffer[5] is 0x80.
			   err_count = 0;
             }
             else

             {
			// is  0,0x80,  
			   err_count++;
             }             

             if(err_count >=2 )
             {
			
                 err_count = 0;

                 printk( "ili9881 lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);

                 return TRUE;

             }

        }
#endif
        return FALSE;

    }
#endif
	
}
static unsigned int lcm_esd_recover(void)
{
    #ifndef BUILD_LK
    printk( "ili9881 lcm_esd_recover\n");
    #endif
	lcm_init();
//	lcm_resume();

	return TRUE;
}

//dingyisheng@wind-mobi.com 20160704 begin 	
extern atomic_t ESDCheck_byCPU;
static unsigned int lcm_ata_check(unsigned char *buf)		
{
	#ifndef BUILD_LK
	unsigned int id=0,id1=0;	
	//unsigned int id=0,id1=0,id2=0;	//liukangping
	unsigned char buffer[3];	
	unsigned int data_array[16];  		

	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00023902;
	data_array[1]=0x000013ba;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	
	data_array[0] = 0x00023700;
	dsi_set_cmdq(data_array, 1, 1);	
	MDELAY(10);

	atomic_set(&ESDCheck_byCPU,1);	
	read_reg_v2(0xDC, buffer, 1); //0xDA	
	atomic_set(&ESDCheck_byCPU,0);	
	id1 = buffer[0]; 

	//atomic_set(&ESDCheck_byCPU,1);	
	//read_reg_v2(0xDB, buffer, 1); 	
	//atomic_set(&ESDCheck_byCPU,0);	
	//id2 = buffer[0]; 

	id = id1;	

	if(LCM_ID_HX8394==id)	
		return 1;	
	else	
		return 0;	
#else 
		return 0;
#endif
}	
//dingyisheng@wind-mobi.com 20160704 end 

#ifdef WIND_LCD_POWER_SUPPLY_SUPPORT
extern void lcm_init_power(void);
extern void lcm_resume_power(void);
extern void lcm_suspend_power(void);
#endif

LCM_DRIVER hx8394f_hd720_dsi_vdo_ykl_lcm_drv =
{
	.name           	= "hx8394f_hd720_dsi_vdo_ykl",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           	= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,
	.compare_id     	= lcm_compare_id,
	.esd_check = lcm_esd_check,
	//dingyisheng@wind-mobi.com 20160704 begin 
	.ata_check    =   lcm_ata_check,
	//dingyisheng@wind-mobi.com 20160704 end
	.esd_recover = lcm_esd_recover,
#ifdef WIND_LCD_POWER_SUPPLY_SUPPORT
	.init_power		= lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
#endif
};
//late_initcall(lcm_init);

//liukangping@wind-mobi.com 20170314 end
