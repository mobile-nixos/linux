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
#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(2160)
//#define FRAME_HEIGHT 										(1280) //liukangping
#define LCM_ID_HX8399                                       (0x99) //qiangang
#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
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


extern struct pinctrl *lcmbiasctrl;
extern struct pinctrl_state *lcmbias_enable;
extern struct pinctrl_state *lcmbias_disable;
/*
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
	}

	lcmbias_enable = pinctrl_lookup_state(lcmbiasctrl, "lcd_bias_enp1_gpio");
	if (IS_ERR(lcmbias_enable)) {
		pr_debug("%s : pinctrl err, lcmbias_enable\n", __func__);
	}

	lcmbias_disable = pinctrl_lookup_state(lcmbiasctrl, "lcd_bias_enp0_gpio");
	if (IS_ERR(lcmbias_disable)) {
		pr_debug("%s : pinctrl err, lcmbias_disable\n", __func__);
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
*/

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting1[] = 
{ 
    {0xB9,3,{0xFF,0x83,0x99}}, 
    {0xBA,2,{0x63,0x23}}, 
    {0xD2,1,{0x44}}, 
    {0xB1,12,{0x00,0x04,0x71,0x91,0x01,0x32,0x33,0x11,0x11,0x4D, 
              0x57,0x06}}, 
    {0xB2,11,{0x00,0x88,0x80,0xCC,0x05,0x07,0x5A,0x11,0x00,0x00, 
              0x10}}, 
    {0xB4,44,{0x00,0xFF,0x02,0xA7,0x02,0xA7,0x02,0xA7,0x02,0x00, 
              0x03,0x05,0x00,0x2D,0x03,0x09,0x0A,0x21,0x03,0x02, 
              0x00,0x06,0xA5,0x87,0x02,0xA7,0x02,0xA7,0x02,0xA7, 
              0x02,0x00,0x03,0x05,0x00,0x2D,0x03,0x09,0x0A,0x02, 
              0x00,0x06,0xA5,0x01}}, 
    {0xD3,33,{0x00,0x0C,0x03,0x01,0x00,0x00,0x14,0x04,0x32,0x10, 
              0x09,0x00,0x09,0x32,0x10,0x00,0x00,0x00,0x00,0x00, 
              0x00,0x00,0x00,0x11,0x00,0x02,0x02,0x03,0x00,0x00, 
              0x00,0x0A,0x40}}, 
    {0xD5,32,{0x18,0x18,0x18,0x18,0x03,0x02,0x01,0x00,0x18,0x18, 
              0x18,0x18,0x18,0x18,0x19,0x19,0x21,0x20,0x18,0x18, 
              0x18,0x18,0x18,0x18,0x18,0x18,0x2F,0x2F,0x30,0x30, 
              0x31,0x31}}, 
    {0xD6,32,{0x18,0x18,0x18,0x18,0x00,0x01,0x02,0x03,0x18,0x18, 
              0x40,0x40,0x19,0x19,0x18,0x18,0x20,0x21,0x40,0x40, 
              0x18,0x18,0x18,0x18,0x18,0x18,0x2F,0x2F,0x30,0x30, 
              0x31,0x31}}, 
    {0xD8,16,{0xAF,0xAA,0xEA,0xAA,0xAF,0xAA,0xEA,0xAA,0x00,0x00, 
              0x00,0x00,0x00,0x00,0x00,0x00}}, 
    {0xBD,1,{0x01}}, 
    {0xD8,16,{0xFF,0xEF,0xEA,0xBF,0xFF,0xEF,0xEA,0xBF,0x00,0x00, 
              0x00,0x00,0x00,0x00,0x00,0x00}}, 
    {0xBD,1,{0x02}}, 
    {0xD8,8,{0xFF,0xEF,0xEA,0xBF,0xFF,0xEF,0xEA,0xBF}}, 
    {0xBD,1,{0x00}}, 
    {0xE0,54,{0x01,0x1A,0x29,0x26,0x59,0x64,0x73,0x6F,0x77,0x80, 
              0x87,0x8D,0x92,0x9A,0xA2,0xA6,0xAA,0xB7,0xB9,0xC0, 
              0xB4,0xC1,0xC3,0x63,0x5E,0x68,0x73, 
              0x01,0x1A,0x29,0x26,0x59,0x64,0x73,0x6F,0x77,0x80, 
              0x87,0x8D,0x92,0x9A,0xA2,0xA6,0xAA,0xB1,0xB4,0xC0, 
              0xB4,0xC1,0xC3,0x63,0x5E,0x68,0x73}}, 
    //{0xB6,2,{0x97,0x97}}, 
    {0xCC,1,{0x08}}, 
    //{0xE3,1,{0x01}}, 
    //{0xCF,4,{0x00,0x14,0x00,0x80}}, 
    {0xC6,2,{0xFF,0xFA}}, 
    //{0xC9,8,{0x07,0x00,0x0F,0x1E,0xB1,0x1E,0x00,0x91}},
    
    {0x35,1,{0x00}}, 
    
    {0x11,1,{0x00}}, 
    {REGFLAG_DELAY, 120, {0}},         
    {0x29,1,{0x00}}, 
    {REGFLAG_DELAY, 20, {0}}, 
    {REGFLAG_END_OF_TABLE, 0x00, {}} 

}; 

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
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_PULSE_VDO_MODE;  
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
		params->dsi.vertical_sync_active				= 4; //4   
		params->dsi.vertical_backporch				       = 3;  //14  
		params->dsi.vertical_frontporch				       = 9;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 44;   //4
		params->dsi.horizontal_backporch				= 44;  //60   liukangping update
		params->dsi.horizontal_frontporch				= 30;    //60  liukangping update
//		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  

	//	params->dsi.HS_TRAIL=14;
	//	params->dsi.pll_div1=1;		   
	//	params->dsi.pll_div2=1;		   
	//	params->dsi.fbk_div =28;//28	
    
        // wangjun@wind-mobi.com 201013 begin: mipi per line reture lp
        //params->dsi.clk_lp_per_line_enable = 1;
        params->dsi.noncont_clock= TRUE;//FALSE;  
        // wangjun@wind-mobi.com 201013 end
    
//zhounengwen@wind-mobi.com 20150327 beign
// To fix lcm rf
        params->dsi.PLL_CLOCK = 494; //modify by qiangang
		params->dsi.ssc_disable = 1; //add by qiangang
//		params->dsi.CLK_TRAIL = 17;
//qiangang@wind-mobi.com 20170912 begin	
	    params->dsi.cont_clock=0;   
	    params->dsi.esd_check_enable = 0;
	    params->dsi.customization_esd_check_enable      = 1;
	    params->dsi.lcm_esd_check_table[0].cmd          = 0xd9;
	    params->dsi.lcm_esd_check_table[0].count        = 1;
	    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80; //80

	    params->dsi.lcm_esd_check_table[1].cmd          = 0x09;
	    params->dsi.lcm_esd_check_table[1].count        = 3;
	    params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80; 
	    params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73; 
	    params->dsi.lcm_esd_check_table[1].para_list[2] = 0x06;
		
	 
	  
//qiangang@wind-mobi.com 20170912 end
// wangjun@wind-mobi.com 20170828 begin 
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 40;
#endif
// wangjun@wind-mobi.com 20170828 end 

}


static void lcm_init(void)
{
#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif
	pinctrl_select_state(lcmbiasctrl, lcmbias_enable);  //liukangping

	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50); //modified by hebiao at 20170601
	//printf(" gemingming hx8394f init  \n");
	push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1); 
//init_lcm_registers();
		  
}

static void lcm_suspend(void) 
{


   printk("hx8399 liukangping lcm_suspend\n");
	// when phone sleep , config output low, disable backlight drv chip  
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
// zhaozhensen@wind-mobi.com 20160322 begin
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
// zhaozhensen@wind-mobi.com 20160322 end
#endif
pinctrl_select_state(lcmbiasctrl, lcmbias_disable);  //liukangping
	MDELAY(10);

}

static void lcm_resume(void)
{
	printk("hx8399f liukangping lcm_resume\n");
	lcm_init();

}

#if 0
static unsigned int lcm_compare_id(void)
{
	unsigned int id=0,id1=0;
	//unsigned int id=0,id1=0,id2=0; //liukangping
	unsigned char buffer[3];
	unsigned int data_array[16];  
//	pinctrl_select_state(lcmbiasctrl, lcmbias_enable);  //liukangping
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

	if(LCM_ID_HX8399==id)
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
#endif
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
	data_array[1]=0x9983FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00033902;
	data_array[1]=0x002363ba;
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

	if(LCM_ID_HX8399==id)	
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

LCM_DRIVER hx8399_fhd_dsi_vdo_hlt_lcm_drv =
{
	.name           	= "hx8399_fhd_dsi_vdo_hlt",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           	= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,
	//.compare_id     	= lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//dingyisheng@wind-mobi.com 20160704 begin 
	.ata_check    =   lcm_ata_check,
	//dingyisheng@wind-mobi.com 20160704 end
	//.esd_recover = lcm_esd_recover,
#ifdef WIND_LCD_POWER_SUPPLY_SUPPORT
	.init_power		= lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
#endif
};
//late_initcall(lcm_init);

//liukangping@wind-mobi.com 20170314 end
