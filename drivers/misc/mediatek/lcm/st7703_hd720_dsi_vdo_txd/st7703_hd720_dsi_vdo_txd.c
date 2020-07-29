// zhangkaiyuan@wind-mobi.com 20170926 begin
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
#define LCM_DSI_CMD_MODE        0
#define FRAME_WIDTH             (720)
#define FRAME_HEIGHT            (1440)
#define LCM_ID_ST7703_TXD           (0x7703)

#define REGFLAG_DELAY           0xFC
#define REGFLAG_END_OF_TABLE    0xFD            // END OF REGISTERS MARKER

#ifndef TRUE
        #define TRUE 1
#endif

#ifndef FALSE
        #define FALSE 0
#endif

extern int esd_check_alarm;
static unsigned int lcm_esd_test        = FALSE;                        // only for ESD test
static char* lcm_name                   = "st7703_txd";                 // define the lcm'name

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util;

#define __SAME_IC_COMPATIBLE__

#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))
#define MDELAY(n)               (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)           lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                      lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                                lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

extern struct pinctrl *lcmbiasctrl;
extern struct pinctrl_state *lcmbias_enable;
extern struct pinctrl_state *lcmbias_disable;

struct LCM_setting_table
{
        unsigned cmd;
        unsigned char count;
        unsigned char para_list[64];
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{
        {0x28, 1, {0x00}},
        {REGFLAG_DELAY, 20, {}},
        {0x10, 1, {0x00}},
        {REGFLAG_DELAY, 150, {}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
        unsigned int i;
        for(i = 0; i < count; i++)
        {
                unsigned cmd;
                cmd = table[i].cmd;
                switch (cmd)
                {
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
        params->type                                    = LCM_TYPE_DSI;
        params->physical_width                          = 65;
        params->physical_height                         = 130;
        params->width                                   = FRAME_WIDTH;
        params->height                                  = FRAME_HEIGHT;
        params->dbi.te_mode                             = LCM_DBI_TE_MODE_VSYNC_ONLY;
        params->dbi.te_edge_polarity                    = LCM_POLARITY_RISING;
        params->dsi.mode                                = SYNC_PULSE_VDO_MODE;
        params->dsi.LANE_NUM                            = LCM_FOUR_LANE;
        params->dsi.data_format.color_order             = LCM_COLOR_ORDER_RGB;
        params->dsi.data_format.trans_seq               = LCM_DSI_TRANS_SEQ_MSB_FIRST;
        params->dsi.data_format.padding                 = LCM_DSI_PADDING_ON_LSB;
        params->dsi.data_format.format                  = LCM_DSI_FORMAT_RGB888;
        params->dsi.intermediat_buffer_num              = 2;
        params->dsi.PS                                  = LCM_PACKED_PS_24BIT_RGB888;
        params->dsi.packet_size                         = 256;
        params->dsi.vertical_sync_active                = 4;
        params->dsi.vertical_backporch                  = 21;
        params->dsi.vertical_frontporch                 = 17;
        params->dsi.vertical_active_line                = FRAME_HEIGHT;
// wangbing@wind-mobi.com 20171018 begin
        params->dsi.horizontal_sync_active              = 30;           // harvey 170902 10->35->4
        params->dsi.horizontal_backporch                = 45;           // harvey 170902 50->80->30
        params->dsi.horizontal_frontporch               = 45;           // harvey 170902 50->80->30
        params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
        params->dsi.PLL_CLOCK                           = 243;
        params->dsi.ssc_disable                         = 1; //add by qiangang
        params->dsi.cont_clock                          = 0;
        params->dsi.esd_check_enable                    = 1;
        params->dsi.customization_esd_check_enable      = 1;
        
        params->dsi.lcm_esd_check_table[0].cmd          = 0x68;
        params->dsi.lcm_esd_check_table[0].count        = 1;
        params->dsi.lcm_esd_check_table[0].para_list[0] = 0xC0;
        
        params->dsi.lcm_esd_check_table[1].cmd          = 0x09;
        params->dsi.lcm_esd_check_table[1].count        = 4;
        params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
        params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73;
        params->dsi.lcm_esd_check_table[1].para_list[2] = 0x04;
        params->dsi.lcm_esd_check_table[1].para_list[3] = 0x1C;
        
        params->dsi.lcm_esd_check_table[2].cmd          = 0xAF;
        params->dsi.lcm_esd_check_table[2].count        = 1;
        params->dsi.lcm_esd_check_table[2].para_list[0] = 0xFD;
        
        params->dsi.lcm_esd_check_table[3].cmd          = 0xB0;
        params->dsi.lcm_esd_check_table[3].count        = 4;
        params->dsi.lcm_esd_check_table[3].para_list[0] = 0xA1;
        params->dsi.lcm_esd_check_table[3].para_list[1] = 0x03;
        params->dsi.lcm_esd_check_table[3].para_list[2] = 0xB5;
        params->dsi.lcm_esd_check_table[3].para_list[3] = 0x3A;

        params->dsi.lcm_esd_check_table[4].cmd          = 0xBA;
        params->dsi.lcm_esd_check_table[4].count        = 10;
        params->dsi.lcm_esd_check_table[4].para_list[0] = 0x33;
        params->dsi.lcm_esd_check_table[4].para_list[1] = 0x81;
        params->dsi.lcm_esd_check_table[4].para_list[2] = 0x05;
        params->dsi.lcm_esd_check_table[4].para_list[3] = 0xF9;
        params->dsi.lcm_esd_check_table[4].para_list[4] = 0x0E;
        params->dsi.lcm_esd_check_table[4].para_list[5] = 0x0E;
        params->dsi.lcm_esd_check_table[4].para_list[6] = 0x02;
        params->dsi.lcm_esd_check_table[4].para_list[7] = 0x00;
        params->dsi.lcm_esd_check_table[4].para_list[8] = 0x00;
        params->dsi.lcm_esd_check_table[4].para_list[9] = 0x00;
        
// wangbing@wind-mobi.com 20171018 end
        // wangjun@wind-mobi.com 20170828 begin 
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 40;
#endif
// wangjun@wind-mobi.com 20170828 end 
}

// wangbing@wind-mobi.com 20171018 begin
static struct LCM_setting_table lcm_initialization_setting[] =
{
        {0xB9, 0x03, {0xF1, 0x12, 0x83}},                                               // Set EXTC
        {0xBA, 0x1B, {0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x02, 0x00, 0x00, 0x00,       // Set DSI
                      0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x91, 0x0a, 0x00,
                      0x00, 0x02, 0x4F, 0x01, 0x03, 0x02, 0x37}},//harvey 171019 24&25&26th 11,00,00->01,03,02
        {0xB8, 0x04, {0x75, 0x22, 0x20, 0x03}},                                         // ECP
        {0xB3, 0x0A, {0x10, 0x10, 0x05, 0x05, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00}},     // SET RGB
        {0xC0, 0x09, {0x73, 0x73, 0x50, 0x50, 0x00, 0xC0, 0x08, 0x70, 0x00}},           // SCR
        {0xBC, 0x01, {0x4E}},                                                           // Set VDC
        {0xCC, 0x01, {0x0B}},                                                           // Set Panel
        {0xB4, 0x01, {0x80}},                                                           // Set Panel inversion
        {0xB2, 0x03, {0xF0, 0x22, 0xF0}},//2nd 0x12->0x22 for bug 1258652 //harvey 170901 3th 30->f0                    // Set RSO
        {0xE3, 0x0E, {0x07, 0x07, 0x0B, 0x0B, 0x03, 0x0b, 0x00, 0x00, 0x00, 0x00,       // Set EQ
                      0xFF, 0x00, 0xC0, 0x10}},//harvey 171026 00 00 0b 0b 10 10 -> 07 07 0b 0b 03 0b
        {0xC6, 0x05, {0x00, 0x00, 0xCF, 0xFF, 0x00}},                                   // SCR
        {0xC1, 0x0C, {0x54, 0x00, 0x1E, 0x1E, 0x77, 0xF1, 0xFF, 0xFF, 0xCC, 0xCC,       // Set POWER
                      0x77, 0x77}},
        {0xB5, 0x02, {0x07, 0x07}},                                                     // Set POWER
        {0xB6, 0x02, {0x30, 0x30}},                                                     // Set VCOM
        {0xBF, 0x03, {0x02, 0x11, 0x00}},                                               // Set PCR
        {0xE9, 0x3F, {0X82, 0x10, 0x06, 0x05, 0xA2, 0x0A, 0xA5, 0x12, 0x31, 0x23,       // GIP
                      0x37, 0x83, 0x04, 0xBC, 0x27, 0x38, 0x0C, 0x00, 0x03, 0x00,
                      0x00, 0x00, 0x0C, 0x00, 0x03, 0x00, 0x00, 0x00, 0x75, 0x75,
                      0x31, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x13, 0x88, 0x64,
                      0x64, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x02, 0x88,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00}},
        {0xEA, 0x3D, {0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       // GIP
                      0x00, 0x00, 0x02, 0x46, 0x02, 0x88, 0x88, 0x88, 0x88, 0x88,
                      0x88, 0x64, 0x88, 0x13, 0x57, 0x13, 0x88, 0x88, 0x88, 0x88,
                      0x88, 0x88, 0x75, 0x88, 0x23, 0x14, 0x00, 0x00, 0x02, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x03, 0x0A, 0xA5, 0x00, 0x00, 0x00,
                      0x00}},
        {0xE0, 0x22, {0x00, 0x07, 0x0F, 0x26, 0x30, 0x3C, 0x44, 0x36, 0x07, 0x0D, 0x0D, 0x11, 0x12, 0x10, 0x12, 0x13, 0x17,
                      0x00, 0x07, 0x0F, 0x26, 0x30, 0x3C, 0x44, 0x36, 0x07, 0x0D, 0x0D, 0x11, 0x12, 0x10, 0x12, 0x13, 0x17}},     //gamma2.2
        {0x11, 0x00, {0x00}},                                                           // Sleep Out
        {REGFLAG_DELAY, 120, {}},
        {0x29, 0x00, {0x00}},                                                           // Dispaly On
        {REGFLAG_DELAY, 20, {}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};
// wangbing@wind-mobi.com 20171018 end

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
        MDELAY(20);
        if(esd_check_alarm)
        {
                MDELAY(100);
        }
        push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
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
//add by qiangang 20171026 begin
        pinctrl_select_state(lcmbiasctrl, lcmbias_disable);
        MDELAY(10);
        SET_RESET_PIN(0);
        MDELAY(10);
//add by qiangang 20171026 end
}

static void lcm_resume(void)
{
        printk("[wind_lcm][k][%s][%d] lcm_name = %s\n", __func__, __LINE__, lcm_name);
        lcm_init();
}

static unsigned int lcm_compare_id(void)
{
        unsigned int id = 0, id1 = 0, id2 = 0, id3 = 0;
        unsigned char buffer[3];
        unsigned int data_array[16];
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
        MDELAY(120);

        data_array[0] = 0x00013700;             // return byte number
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(10);

        read_reg_v2(0xDA, buffer, 1); //0x1D txd
        id1 = buffer[0];
        read_reg_v2(0xDB, buffer, 1); //0x77
        id2 = buffer[0];
		read_reg_v2(0xDC, buffer, 1); //0x03
        id3 = buffer[0];
		id = (id2 << 8) | id3;
        printk("[wind_lcm_st7703_txd][lk][%s][%d] ID1(0XDA)=0x%02x, ID2(0XDB)=0x%02x, ID3(0XDC)=0x%02x\n", __func__, __LINE__, id1, id2, id3);
		if(id1 == 0x1D) //txd
		{
		     if(LCM_ID_ST7703_TXD == id) //st7703			 
			     return 1;
			 else
			     return 0;
		}
        else
           return 0;
		
}

static unsigned int lcm_esd_check(void)
{
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
        printk("st7703 lcm_esd_check: buffer[0] = %d,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
        if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/
        {
                printk("[wind_lcm][%s][k][%s][%d] buffer[0] = %x\n", lcm_name, __func__, __LINE__, buffer[0]);
                return TRUE;
        }
        return FALSE;
}

static unsigned int lcm_esd_recover(void)
{
        lcm_init();
        return TRUE;
}

extern atomic_t ESDCheck_byCPU;

static unsigned int lcm_ata_check(unsigned char *buf)
{
#ifndef BUILD_LK
        unsigned int id = 0,id1 = 0;
        unsigned char buffer[3];
        unsigned int data_array[16];

        data_array[0] = 0x00023700;
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(10);

        atomic_set(&ESDCheck_byCPU,1);
        read_reg_v2(0xDC, buffer, 1);           // 0x03
        atomic_set(&ESDCheck_byCPU,0);
        id1 = buffer[0];

        id = id1;
        if(0x03 == id)
                return 1;
        else
                return 0;
#else
        return 0;
#endif
}

#ifdef WIND_LCD_POWER_SUPPLY_SUPPORT
extern void lcm_init_power(void);
extern void lcm_resume_power(void);
extern void lcm_suspend_power(void);
#endif

LCM_DRIVER st7703_hd720_dsi_vdo_txd_lcm_drv =
{
        .name                   = "st7703_hd720_dsi_vdo_txd",
        .set_util_funcs         = lcm_set_util_funcs,
        .get_params             = lcm_get_params,
        .init                   = lcm_init,
        .suspend                = lcm_suspend,
        .resume                 = lcm_resume,
        .compare_id             = lcm_compare_id,
        .esd_check              = lcm_esd_check,
        .ata_check              = lcm_ata_check,
        .esd_recover            = lcm_esd_recover,
#ifdef WIND_LCD_POWER_SUPPLY_SUPPORT
        .init_power             = lcm_init_power,
        .resume_power           = lcm_resume_power,
        .suspend_power          = lcm_suspend_power,
#endif
};
// zhangkaiyuan@wind-mobi.com 20170926 end
