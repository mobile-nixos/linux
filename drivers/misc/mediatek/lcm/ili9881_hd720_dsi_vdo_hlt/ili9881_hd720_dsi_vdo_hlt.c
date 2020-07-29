//hebiao@wind-mobi.com 20160822 begin

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
#define FRAME_HEIGHT 										(1280)
#define LCM_ID_ILI9881                                      (0x9881)

#define REGFLAG_DELAY             							0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

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


//ranyanhao@wind-mobi.com 20161018 begin
static struct platform_device * pltfm_dev ;
struct pinctrl *lcmbiasctrl = NULL;
struct pinctrl_state *lcmbias_enable= NULL;
struct pinctrl_state *lcmbias_disable= NULL;

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
//ranyanhao@wind-mobi.com 20161018 end


 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting1[] = 
{

 {0xFF,3,{0x98,0x81,0x03}},
	 {0x01,1,{0x00}},
	 {0x02,1,{0x00}},
	 {0x03,1,{0x73}},
	 {0x04,1,{0x00}},
	 {0x05,1,{0x00}},
	 {0x06,1,{0x0A}},
	 {0x07,1,{0x00}},
	 {0x08,1,{0x00}},
	 {0x09,1,{0x01}},
	 {0x0a,1,{0x01}},
	 {0x0b,1,{0x00}},
	 {0x0c,1,{0x00}},
	 {0x0d,1,{0x00}},
	 {0x0e,1,{0x00}},
	 {0x0f,1,{0x01}},
	 {0x10,1,{0x01}},
	 {0x11,1,{0x00}},
	 {0x12,1,{0x00}},
	 {0x13,1,{0x00}},
	 {0x14,1,{0x00}},
	 {0x15,1,{0x00}},
	 {0x16,1,{0x00}},
	 {0x17,1,{0x00}},
	 {0x18,1,{0x00}},
	 {0x19,1,{0x00}},
	 {0x1a,1,{0x00}},
	 {0x1b,1,{0x00}},
	 {0x1c,1,{0x00}},
	 {0x1d,1,{0x00}},
	 {0x1e,1,{0x40}},
	 {0x1f,1,{0xC1}},
	 {0x20,1,{0x06}},
	 {0x21,1,{0x01}},
	 {0x22,1,{0x06}},
	 {0x23,1,{0x03}},
	 {0x24,1,{0x8A}},
	 {0x25,1,{0x8A}},
	 {0x26,1,{0x00}},
	 {0x27,1,{0x00}},
	 {0x28,1,{0x3B}},
	 {0x29,1,{0x03}},
	 {0x2a,1,{0x00}},
	 {0x2b,1,{0x00}},
	 {0x2c,1,{0x00}},
	 {0x2d,1,{0x00}},
	 {0x2e,1,{0x00}},
	 {0x2f,1,{0x00}},
	 {0x30,1,{0x00}},
	 {0x31,1,{0x00}},
	 {0x32,1,{0x00}},
	 {0x33,1,{0x00}},
	 {0x34,1,{0x00}},
	 {0x35,1,{0x00}},
	 {0x36,1,{0x00}},
	 {0x37,1,{0x00}},
	 {0x38,1,{0x00}},
	 {0x39,1,{0x00}},
	 {0x3a,1,{0x00}},
	 {0x3b,1,{0x00}},
	 {0x3c,1,{0x00}},
	 {0x3d,1,{0x00}},
	 {0x3e,1,{0x00}},
	 {0x3f,1,{0x00}},
	 {0x40,1,{0x00}},
	 {0x41,1,{0x00}},
	 {0x42,1,{0x00}},
	 {0x43,1,{0x00}},
	 {0x44,1,{0x00}},
	 {0x50,1,{0x01}},
	 {0x51,1,{0x23}},
	 {0x52,1,{0x45}},
	 {0x53,1,{0x67}},
	 {0x54,1,{0x89}},
	 {0x55,1,{0xab}},
	 {0x56,1,{0x01}},
	 {0x57,1,{0x23}},
	 {0x58,1,{0x45}},
	 {0x59,1,{0x67}},
	 {0x5a,1,{0x89}},
	 {0x5b,1,{0xAB}},
	 {0x5c,1,{0xCD}},
	 {0x5d,1,{0xEF}},
	 {0x5e,1,{0x00}},
	 {0x5f,1,{0x0C}},
	 {0x60,1,{0x0D}},
	 {0x61,1,{0x0E}},
	 {0x62,1,{0x0F}},
	 {0x63,1,{0x06}},
	 {0x64,1,{0x07}},
	 {0x65,1,{0x02}},
	 {0x66,1,{0x02}},
	 {0x67,1,{0x02}},
	 {0x68,1,{0x02}},
	 {0x69,1,{0x02}},
	 {0x6a,1,{0x02}},
	 {0x6b,1,{0x02}},
	 {0x6c,1,{0x02}},
	 {0x6d,1,{0x02}},
	 {0x6e,1,{0x05}},
	 {0x6f,1,{0x05}},
	 {0x70,1,{0x05}},
	 {0x71,1,{0x02}},
	 {0x72,1,{0x02}},
	 {0x73,1,{0x00}},
	 {0x74,1,{0x01}},
	 {0x75,1,{0x0C}},
	 {0x76,1,{0x0D}},
	 {0x77,1,{0x0E}},
	 {0x78,1,{0x0F}},
	 {0x79,1,{0x06}},
	 {0x7a,1,{0x07}},
	 {0x7b,1,{0x02}},
	 {0x7c,1,{0x02}},
	 {0x7d,1,{0x02}},
	 {0x7e,1,{0x02}},
	 {0x7f,1,{0x02}},
	 {0x80,1,{0x02}},
	 {0x81,1,{0x02}},
	 {0x82,1,{0x02}},
	 {0x83,1,{0x02}},
	 {0x84,1,{0x05}},
	 {0x85,1,{0x05}},
	 {0x86,1,{0x05}},
	 {0x87,1,{0x02}},
	 {0x88,1,{0x02}},
	 {0x89,1,{0x00}},
	 {0x8A,1,{0x01}},
	
	 {0xFF,3,{0x98,0x81,0x04}},
//	 {0x00,1,{0x00}},    //3lane  
	 {0x6C,1,{0x15}},
	 {0x6E,1,{0x15 }}, //modified by hebiao at 20161221 for Tp noise
	 {0x6F,1,{0x25}},
	 {0x8D,1,{0x19}},
	 {0x87,1,{0xBA}},
	 {0x26,1,{0x76}},
	 {0xB2,1,{0xD1}},
	 {0x3A,1,{0x24}}, //modified by hebiao at 20161221 for Tp noise
	 {0x35,1,{0x1F}}, //delete 
	 {0x33,1,{0x00}}, //modified by hebiao at 20161221 for Tp noise
//	 {0x33,1,{0x00}}, //delete
	 {0x7A,1,{0x0F}}, //modified by hebiao at 20161221 for Tp noise
	
	 {0xFF,3,{0x98,0x81,0x01}},
	 {0x22,1,{0x09}},
	 {0x52,1,{0x00}},
	 {0x53,1,{0x63}},
	 {0x54,1,{0x00}},
	 {0x55,1,{0x60}},
	 {0x50,1,{0x7C}},
	 {0x51,1,{0x77}},
	 {0x31,1,{0x00}},
	 {0x60,1,{0x19}},
	 {0x61,1,{0x01}},
     {0x62,1,{0x0C}}, //delete
	 {0x63,1,{0x00}}, //delete
	 
	 //{0x56,1,{0x00}},  //hebiao modified at 20161206 for vcom otp
	 
	 {0xA0,1,{0x3a}},
	 {0xA1,1,{0x48}},
	 {0xA2,1,{0x53}},
	 {0xA3,1,{0x13}},
	 {0xA4,1,{0x13}},
	 {0xA5,1,{0x26}},
	 {0xA6,1,{0x1a}},
	 {0xA7,1,{0x1e}},
	 {0xA8,1,{0xBa}},
	 {0xA9,1,{0x1d}},
	 {0xAA,1,{0x29}},
	 {0xAB,1,{0x9E}},
	 {0xAC,1,{0x1e}},
	 {0xAD,1,{0x19}},
	 {0xAE,1,{0x4e}},
	 {0xAF,1,{0x20}},
	 {0xB0,1,{0x66}},
	 {0xB1,1,{0x55}},
	 {0xB2,1,{0x65}},
	 {0xB3,1,{0x34}},
	 {0xC0,1,{0x3a}},
	 {0xC1,1,{0x48}},
	 {0xC2,1,{0x53}},
	 {0xC3,1,{0x12}},
	 {0xC4,1,{0x13}},
	 {0xC5,1,{0x26}},
	 {0xC6,1,{0x19}},
	 {0xC7,1,{0x1D}},
	 {0xC8,1,{0xBb}},
	 {0xC9,1,{0x1B}},
	 {0xCA,1,{0x28}},
	 {0xCB,1,{0x9E}},
	 {0xCC,1,{0x1A}},
	 {0xCD,1,{0x1A}},
	 {0xCE,1,{0x4e}},
	 {0xCF,1,{0x21}},
	 {0xD0,1,{0x66}},
	 {0xD1,1,{0x58}},
	 {0xD2,1,{0x65}},
	 {0xD3,1,{0x39}},

	//CMD_P 0
	{0xFF,3,{0x98,0x81,0x00}},
	{0x35,1,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 20, {}}

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
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0xFF,	3,		{0x98,0x81,0x01}},
    {0x58, 1, {0x01}},
    {REGFLAG_DELAY, 20, {}},
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
		printk("lcm ili9881_hd720_dsi_vdo_yassy %s \n",__func__);
		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;


		//dingyisheng@wind-mobi.com 20160413 beign
		/*The following two values added are always uesd for CTS check */
		/*in order to match density and screen size*/
		params->physical_width = 65;
	    params->physical_height = 115 ;
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
	//	params->dsi.cont_clock=1;
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
		params->dsi.vertical_sync_active				= 20; //modified by hebiao at 20161221 for Tp noise
		params->dsi.vertical_backporch				    = 50; //modified by hebiao at 20161221 for Tp noise
		params->dsi.vertical_frontporch				    = 30; //modified by hebiao at 20161221 for Tp noise
		params->dsi.vertical_active_line				= FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 60; //modified by hebiao at 20161221 for Tp noise
		params->dsi.horizontal_frontporch				= 60; //modified by hebiao at 20161221 for Tp noise
	//	params->dsi.horizontal_blanking_pixel			= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  

		params->dsi.HS_TRAIL=15;
        params->dsi.PLL_CLOCK = 212;
	    params->dsi.cont_clock=0;   
	    params->dsi.esd_check_enable = 1;
	    params->dsi.customization_esd_check_enable      = 1;
	    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	    params->dsi.lcm_esd_check_table[0].count        = 1;
	    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init(void)
{
#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif
	pinctrl_select_state(lcmbiasctrl, lcmbias_enable); 

	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1); 

	printk("lcm ili9881_hd720_dsi_vdo_yassy %s \n",__func__);	  
}

static void lcm_suspend(void) 
{
	// when phone sleep , config output low, disable backlight drv chip  
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef GPIO_LCD_BIAS_ENP_PIN
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
#endif
	pinctrl_select_state(lcmbiasctrl, lcmbias_disable); 
	MDELAY(10);

	printk("lcm ili9881_hd720_dsi_vdo_yassy %s \n",__func__);
	MDELAY(10);
}

static void lcm_resume(void)
{
	lcm_init();

}

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0,id1=0,id2=0;
	unsigned char buffer[3];
	unsigned int data_array[16];  
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50); 

	data_array[0]=0x00053902;
	data_array[1]=0x8198FFFF;
	data_array[2]=0x00000001;
	
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10); 

	data_array[1]=0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x00, buffer, 1);
	id1= buffer[0];
	read_reg_v2(0x01, buffer, 1);
	id2= buffer[0];

	id=(id1 << 8) | id2;

#if defined(BUILD_LK)||defined(BUILD_UBOOT)
	printf(" LK ili9881 hlt debug: %s id=%x id1=%x id2=%x \n",__func__,id, id1, id2);
#else
	printk(" kernel ili9881 hlt horse debug: %s id=%x  \n",__func__,id);
#endif	

	if(LCM_ID_ILI9881==id) 
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
    }else{

#if 0
        if(buffer[3] != 0x02) //error data type is 0x02
		{
		//  is not 02, 
             err_count = 0;
        }else{
		// is 02, so ,
             if((buffer[4] == 0x40) || (buffer[5] == 0x80))
             {
			 // buffer[4] is not 0, || (huo),buffer[5] is 0x80.
			   err_count = 0;
             }else{
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
	unsigned int id=0,id1=0,id2=0;	
	unsigned char buffer[3];
	unsigned int data_array[16];  		

	data_array[0]=0x00053902;
	data_array[1]=0x8198FFFF;
	data_array[2]=0x00000001;
	
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10); 

	data_array[1]=0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	
	
	atomic_set(&ESDCheck_byCPU,1);	
	read_reg_v2(0x00, buffer, 1);
	atomic_set(&ESDCheck_byCPU,0);
	id1= buffer[0]; //should be 0x01
	atomic_set(&ESDCheck_byCPU,1);
	read_reg_v2(0x01, buffer, 1);
	atomic_set(&ESDCheck_byCPU,0);
	id2= buffer[0]; //should be 0x01

	id=(id1 << 8) | id2;

	if(LCM_ID_ILI9881==id){
		return 1;	
	}
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

LCM_DRIVER ili9881_hd720_dsi_vdo_hlt_lcm_drv =
{
	.name           	= "ili9881_hd720_dsi_vdo_hlt",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           	= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,
	.compare_id     	= lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
	.ata_check    =   lcm_ata_check,
	
#ifdef WIND_LCD_POWER_SUPPLY_SUPPORT
	.init_power		= lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
#endif
};

//add by hebiao@wind-mobi.com 20160822 end