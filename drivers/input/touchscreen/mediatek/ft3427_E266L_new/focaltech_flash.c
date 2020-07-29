/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2010-2016, Focaltech Ltd. All rights reserved.
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

/*****************************************************************************
*
* File Name: focaltech_flash.c
*
* Author:    fupeipei
*
* Created:    2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include "focaltech_flash.h"

//hebiao@wind-mobi.com 20160101 begin
//hebiao@wind-mobi.com 20170419 begin
#ifdef CONFIG_WIND_DEVICE_INFO
#include "wind_device_info.h"
extern wind_device_info_t wind_device_info;
#endif
//hebiao@wind-mobi.com 20170419 end

/*****************************************************************************
* Static variables
*****************************************************************************/
struct ft_chip_t chip_types_onsell = FTS_CHIP_TYPE_MAPPING;

struct fts_Upgrade_Info fts_updateinfo[] = {
    {0x54, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 2, 2, 0x54, 0x2c, 20, 2000}, //,"FT5x46"
    {0x55, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 10, 2000},  //,"FT5x06"
    {0x08, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 10, 0x79, 0x06, 100, 2000}, //,"FT5606"
    {0x0a, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x07, 10, 1500},  //,"FT5x16"
    {0x06, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 100, 30, 0x79, 0x08, 10, 2000},   //,"FT6x06"
    {0x36, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x18, 10, 2000},    //,"FT6x36"
    {0x64, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x1c, 10, 2000},    //,"FT6336GU"
    {0x55, FTS_MAX_POINTS_5, AUTO_CLB_NEED, 50, 30, 0x79, 0x03, 10, 2000},  //,"FT5x06i"
    {0x14, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},    //,"FT5336"
    {0x13, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},    //,"FT3316"
    {0x12, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},    //,"FT5436i"
    {0x11, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 30, 30, 0x79, 0x11, 10, 2000},    //,"FT5336i"
    {0x58, FTS_MAX_POINTS_5, AUTO_CLB_NONEED, 2, 2, 0x58, 0x2c, 20, 2000},  //"FT5822"
    {0x59, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 30, 50, 0x79, 0x10, 1, 2000},    //"FT5x26"
    {0x86, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 2, 2, 0x86, 0xA7, 20, 2000}, //"FT8607"
    {0x87, FTS_MAX_POINTS_10, AUTO_CLB_NONEED, 2, 2, 0x87, 0xA6, 20, 2000}, //"FT8716"
    {0x0E, FTS_MAX_POINTS_2, AUTO_CLB_NONEED, 10, 10, 0x79, 0x18, 10, 2000},    //,"FT3x07"
};

//hebiao@wind-mobi.com begin at 20170101 for guoxian and txd compatible
unsigned char CTPM_FW_GX[] = {
#include FTS_UPGRADE_FW_APP_GX
};
//FTS_DEBUG("[UPGRADE]:get gx fw size\n");
unsigned char CTPM_FW_TXD[] = {
#include FTS_UPGRADE_FW_APP_TXD
};

//qiangang@wind-mobi.com begin at 20170326 begin for hlt update
unsigned char CTPM_FW_HLT[] = {  
#include FTS_UPGRADE_FW_APP_HLT
};
//qiangang@wind-mobi.com begin at 20170326 end for hlt update


unsigned char CTPM_FW_HS[] =                // wangbing@wind-mobi.com 20170502 begin
{  
#include FTS_UPGRADE_FW_APP_HS
};                                          // wangbing@wind-mobi.com 20170502 end

unsigned char CTPM_FW_YKL[] =                // wangjun@wind-mobi.com 20170522 begin
{  
#include FTS_UPGRADE_FW_APP_YKL
};                                          // wangjun@wind-mobi.com 20170522 end

static int force_update = 0;
//hebiao@wind-mobi.com end at 20170101 for guoxian and txd compatible

unsigned char aucFW_PRAM_BOOT_onsell[] = {
#ifdef FTS_UPGRADE_PRAMBOOT
#include FTS_UPGRADE_PRAMBOOT
#endif
};

unsigned char CTPM_LCD_CFG_ONSELL[] = {
#ifdef FTS_UPGRADE_LCD_CFG
#include FTS_UPGRADE_LCD_CFG
#endif
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_Upgrade_Info fts_updateinfo_curr;
struct fts_Upgrade_Fun fts_updatefun_curr_onsell;

struct workqueue_struct *touch_wq_onsell;
struct work_struct fw_update_work_onsell;

int compatible_flag = 0;  //hebiao@wind-mobi.com begin at 20170101 for guoxian and txd compatible
/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ctpm_workqueue_init(void);

/************************************************************************
* Name: fts_ctpm_upgrade_delay_onsell
* Brief: 0
* Input: 0
* Output: 0
* Return: 0
***********************************************************************/
void fts_ctpm_upgrade_delay_onsell(u32 i)
{
    do {
        i--;
    }
    while (i > 0);
}

/************************************************************************
* Name: fts_ctpm_i2c_hid2std_onsell
* Brief:  HID to I2C
* Input: i2c info
* Output: no
* Return: fail =0
***********************************************************************/
int fts_ctpm_i2c_hid2std_onsell(struct i2c_client *client)
{
#if (FTS_CHIP_IDC)
    return 0;
#else
    u8 auc_i2c_write_buf[5] = { 0 };
    int bRet = 0;

    auc_i2c_write_buf[0] = 0xeb;
    auc_i2c_write_buf[1] = 0xaa;
    auc_i2c_write_buf[2] = 0x09;
    bRet = fts_i2c_write_onsell(client, auc_i2c_write_buf, 3);
    msleep(10);
    auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;
    fts_i2c_read_onsell(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

    if (0xeb == auc_i2c_write_buf[0] && 0xaa == auc_i2c_write_buf[1]
        && 0x08 == auc_i2c_write_buf[2]) {
        FTS_DEBUG("hidi2c change to stdi2c successful!!");
        bRet = 1;
    }
    else {
        FTS_ERROR("hidi2c change to stdi2c error!!");
        bRet = 0;
    }

    return bRet;
#endif
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadVendorID
* Brief:  read vendor ID
* Input: i2c info, vendor ID
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadVendorID(struct i2c_client *client,
                                     u8 * ucPVendorID)
{
    u8 reg_val[4] = { 0 };
    u32 i = 0;
    u8 auc_i2c_write_buf[10];
    int i_ret;

    *ucPVendorID = 0;
    fts_ctpm_i2c_hid2std_onsell(client);

    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg_onsell(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg_onsell(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);
        /*********Step 2:Enter upgrade mode *****/
        fts_ctpm_i2c_hid2std_onsell(client);
        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write_onsell(client, auc_i2c_write_buf, 2);
        if (i_ret < 0) {
            FTS_DEBUG("failed writing  0x55 and 0xaa!!");
            continue;
        }
        /*********Step 3:check READ-ID***********************/
        msleep(10);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
            0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read_onsell(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (reg_val[0] == chip_types_onsell.rom_idh
            && reg_val[1] == chip_types_onsell.rom_idl) {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x!!",
                      reg_val[0], reg_val[1]);
            break;
        }
        else {
            FTS_DEBUG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x!!",
                      reg_val[0], reg_val[1]);
            continue;
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;
    /*********Step 4: read vendor id from app param area***********************/
    msleep(10);
    auc_i2c_write_buf[0] = 0x03;
    auc_i2c_write_buf[1] = 0x00;
    auc_i2c_write_buf[2] = 0xd7;
    auc_i2c_write_buf[3] = 0x84;
    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        fts_i2c_write_onsell(client, auc_i2c_write_buf, 4);
        msleep(5);
        reg_val[0] = reg_val[1] = 0x00;
        i_ret = fts_i2c_read_onsell(client, auc_i2c_write_buf, 0, reg_val, 2);
        if (0 != reg_val[0]) {
            *ucPVendorID = 0;
            FTS_DEBUG
                ("In upgrade Vendor ID Mismatch, REG1 = 0x%x, REG2 = 0x%x, Definition:0x%x, i_ret=%d!!",
                 reg_val[0], reg_val[1], 0, i_ret);
        }
        else {
            *ucPVendorID = reg_val[0];
            FTS_DEBUG("In upgrade Vendor ID, REG1 = 0x%x, REG2 = 0x%x!!",
                      reg_val[0], reg_val[1]);
            break;
        }
    }
    msleep(50);
    /*********Step 5: reset the new FW***********************/
    FTS_DEBUG("Step 5: reset the new FW!!");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write_onsell(client, auc_i2c_write_buf, 1);
    msleep(200);
    fts_ctpm_i2c_hid2std_onsell(client);
    msleep(10);

    return 0;
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadProjectCode
* Brief:  read project code
* Input: i2c info, project code
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadProjectCode(struct i2c_client *client,
                                        char *pProjectCode)
{
    u8 reg_val[4] = { 0 };
    u32 i = 0;
    u8 j = 0;
    u8 auc_i2c_write_buf[10];
    int i_ret;
    u32 temp;
    fts_ctpm_i2c_hid2std_onsell(client);

    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg_onsell(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg_onsell(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);
        /*********Step 2:Enter upgrade mode *****/
        fts_ctpm_i2c_hid2std_onsell(client);

        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write_onsell(client, auc_i2c_write_buf, 2);
        if (i_ret < 0) {
            FTS_DEBUG("failed writing  0x55 and 0xaa!!");
            continue;
        }
        /*********Step 3:check READ-ID***********************/
        msleep(10);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
            0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read_onsell(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (reg_val[0] == chip_types_onsell.pramboot_idh
            && reg_val[1] == chip_types_onsell.pramboot_idl) {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x!!",
                      reg_val[0], reg_val[1]);
            break;
        }
        else {
            FTS_DEBUG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x!!",
                      reg_val[0], reg_val[1]);
            continue;
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;
    /*********Step 4: read vendor id from app param area***********************/
    msleep(10);
    /*read project code */
    auc_i2c_write_buf[0] = 0x03;
    auc_i2c_write_buf[1] = 0x00;
    for (j = 0; j < 33; j++) {
        temp = 0x20 + j;
        auc_i2c_write_buf[2] = (u8) (temp >> 8);
        auc_i2c_write_buf[3] = (u8) temp;
        fts_i2c_read_onsell(client, auc_i2c_write_buf, 0x20, pProjectCode + j, 1);
        if (*(pProjectCode + j) == '\0')
            break;
    }
    pr_info("project code = %s!!", pProjectCode);
    msleep(50);
    /*********Step 5: reset the new FW***********************/
    FTS_DEBUG("Step 5: reset the new FW!!");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write_onsell(client, auc_i2c_write_buf, 1);
    msleep(200);
    fts_ctpm_i2c_hid2std_onsell(client);

    msleep(10);
    return 0;
}

/************************************************************************
* Name: fts_ctpm_get_upgrade_array_onsell
* Brief: decide which ic
* Input: no
* Output: get ic info in fts_updateinfo_curr
* Return: no
***********************************************************************/
void fts_ctpm_get_upgrade_array_onsell(void)
{
    u8 chip_id;
    u32 i;
    int ret = 0;
    unsigned char auc_i2c_write_buf[10];
    unsigned char reg_val[4] = { 0 };
    fts_ctpm_i2c_hid2std_onsell(fts_i2c_client_onsell);

    for (i = 0; i < 5; i++) {
        ret = fts_i2c_read_reg_onsell(fts_i2c_client_onsell, FTS_REG_CHIP_ID, &chip_id);
        if (ret < 0) {
            FTS_DEBUG("[UPGRADE]: read value fail!!");
        }
        else {
            break;
        }
    }
    FTS_DEBUG("[UPGRADE]: chip_id = %x!!", chip_id);
    if (ret < 0 || chip_id == 0xEF) {
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        fts_i2c_write_onsell(fts_i2c_client_onsell, auc_i2c_write_buf, 1);
        msleep(fts_updateinfo_curr.delay_readid);

        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
            0x00;
        fts_i2c_read_onsell(fts_i2c_client_onsell, auc_i2c_write_buf, 4, reg_val, 2);

        if (reg_val[0] == chip_types_onsell.chip_idh) {
            chip_id = chip_types_onsell.chip_idh;
            FTS_DEBUG
                ("[UPGRADE]: read id for success : id is: ID1 = 0x%x,ID2 = 0x%x!!",
                 reg_val[0], reg_val[1]);
        }
        else {
            FTS_DEBUG
                ("[UPGRADE]: read id for test error: id is: ID1 = 0x%x,ID2 = 0x%x!!",
                 reg_val[0], reg_val[1]);
        }
    }
    for (i = 0; i < sizeof(fts_updateinfo) / sizeof(struct fts_Upgrade_Info);
         i++) {
        if (chip_id == fts_updateinfo[i].CHIP_ID
            || chip_types_onsell.chip_idh == fts_updateinfo[i].CHIP_ID) {
            memcpy(&fts_updateinfo_curr, &fts_updateinfo[i],
                   sizeof(struct fts_Upgrade_Info));
            break;
        }
    }

    if (i >= sizeof(fts_updateinfo) / sizeof(struct fts_Upgrade_Info)) {
        memcpy(&fts_updateinfo_curr, &fts_updateinfo[0],
               sizeof(struct fts_Upgrade_Info));
    }

    if ((chip_types_onsell.chip_idh == fts_updatefun_onsell.chip_idh)
        && (chip_types_onsell.chip_idl == fts_updatefun_onsell.chip_idl)) {
        memcpy(&fts_updatefun_curr_onsell, &fts_updatefun_onsell,
               sizeof(struct fts_Upgrade_Fun));
    }
}

/************************************************************************
* Name: fts_8716_ctpm_fw_write_pram
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
void fts_ctpm_rom_or_pram_reset_onsell(struct i2c_client *client, bool needupgrade)
{
    u8 auc_i2c_write_buf[10];
#if (FTS_CHIP_IDC)
    u8 i = 0;
    unsigned char inRomBoot = 0x00; //0x01 : run in rom boot;  0x02 : run in pram boot
#endif

    FTS_FUNC_ENTER();

    FTS_DEBUG("[UPGRADE]: reset the new FW!!");

    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write_onsell(client, auc_i2c_write_buf, 1);

#if (FTS_CHIP_IDC)
    if (needupgrade) {
        for (i = 0; i < 30; i++) {
            msleep(10);

            inRomBoot = fts_ctpm_get_pram_or_rom_id_onsell(client);
            FTS_DEBUG("[UPGRADE]: inRomBoot = %d!!", inRomBoot);
            if (inRomBoot == FTS_RUN_IN_ROM) {
                FTS_DEBUG("[UPGRADE]: reset ok!!");
                return;
            }
            else {
                FTS_ERROR("[UPGRADE]: reset fail!!");
                continue;
            }
        }
    }
    else
#endif
    {
        msleep(300);
    }

    FTS_FUNC_EXIT();
}

/************************************************************************
* Name: fts_ctpm_auto_clb_onsell
* Brief:  auto calibration
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_clb_onsell(struct i2c_client *client)
{
    unsigned char uc_temp = 0x00;
    unsigned char i = 0;

    /*start auto CLB */
    msleep(200);

    fts_i2c_write_reg_onsell(client, 0, FTS_REG_WORKMODE_FACTORY_VALUE);
    /*make sure already enter factory mode */
    msleep(100);
    /*write command to start calibration */
    fts_i2c_write_reg_onsell(client, 2, 0x4);
    msleep(300);
    if ((chip_types_onsell.chip_idh == 0x11) || (chip_types_onsell.chip_idh == 0x12) || (chip_types_onsell.chip_idh == 0x13) || (chip_types_onsell.chip_idh == 0x14))   //5x36,5x36i
    {
        for (i = 0; i < 100; i++) {
            fts_i2c_read_reg_onsell(client, 0x02, &uc_temp);
            if (0x02 == uc_temp || 0xFF == uc_temp) {
                break;
            }
            msleep(20);
        }
    }
    else {
        for (i = 0; i < 100; i++) {
            fts_i2c_read_reg_onsell(client, 0, &uc_temp);
            if (0x0 == ((uc_temp & 0x70) >> 4)) {
                break;
            }
            msleep(20);
        }
    }
    fts_i2c_write_reg_onsell(client, 0, 0x40);
    msleep(200);
    fts_i2c_write_reg_onsell(client, 2, 0x5);
    msleep(300);
    fts_i2c_write_reg_onsell(client, 0, FTS_REG_WORKMODE_WORK_VALUE);
    msleep(300);
    return 0;
}

/************************************************************************
* Name: fts_GetFirmwareSize_onsell
* Brief:  get file size
* Input: file name
* Output: no
* Return: file size
***********************************************************************/
int fts_GetFirmwareSize_onsell(char *firmware_name)
{
    struct file *pfile = NULL;
    struct inode *inode;
    unsigned long magic;
    off_t fsize = 0;
    char filepath[FILE_NAME_LENGTH];

    memset(filepath, 0, sizeof(filepath));
    sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
    if (NULL == pfile) {
        pfile = filp_open(filepath, O_RDONLY, 0);
    }
    if (IS_ERR(pfile)) {
        pr_err("error occured while opening file %s.\n", filepath);
        return -EIO;
    }
    inode = pfile->f_dentry->d_inode;
    magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    filp_close(pfile, NULL);
    return fsize;
}

/************************************************************************
* Name: fts_ReadFirmware_onsell
* Brief:  read firmware buf for .bin file.
* Input: file name, data buf
* Output: data buf
* Return: 0
***********************************************************************/
int fts_ReadFirmware_onsell(char *firmware_name, unsigned char *firmware_buf)
{
    struct file *pfile = NULL;
    struct inode *inode;
    unsigned long magic;
    off_t fsize;
    char filepath[FILE_NAME_LENGTH];
    loff_t pos;
    mm_segment_t old_fs;

    memset(filepath, 0, sizeof(filepath));
    sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
    if (NULL == pfile) {
        pfile = filp_open(filepath, O_RDONLY, 0);
    }
    if (IS_ERR(pfile)) {
        pr_err("error occured while opening file %s.\n", filepath);
        return -EIO;
    }
    inode = pfile->f_dentry->d_inode;
    magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_read(pfile, firmware_buf, fsize, &pos);
    filp_close(pfile, NULL);
    set_fs(old_fs);
    return 0;
}

/************************************************************************
* Name: fts_getsize_onsell
* Brief: 0
* Input: 0
* Output: 0
* Return: 0
***********************************************************************/
int fts_getsize_onsell(u8 fw_type)
{
    int fw_len = 0;

#if ((FTS_CHIP_TYPE == _FT8716) || (FTS_CHIP_TYPE == _FT8607) || (FTS_CHIP_TYPE == _FT3427)) 
    if (fw_type == FW_SIZE) {
//hebiao@wind-mobi.com begin at 20170101 for guoxian and txd compatible
    if (compatible_flag == 1) {
        FTS_DEBUG("[UPGRADE]:get gx fw size\n");
            fw_len = sizeof(CTPM_FW_GX);
    }else if (compatible_flag == 2) { 
        FTS_DEBUG("[UPGRADE]:get txd fw size\n");
        fw_len = sizeof(CTPM_FW_TXD);
//add by qiangang@wind-mobi.com 20170329 begin
    }else if (compatible_flag == 3) { 
        printk("[UPGRADE]:qiangang get hlt fw size\n");
        fw_len = sizeof(CTPM_FW_HLT);
//add by qiangang@wind-mobi.com 20170329 end

// wangbing@wind-mobi.com 20170502 begin
    }else if (compatible_flag == 4) { 
        printk("[UPGRADE]: get hs fw size\n");
        fw_len = sizeof(CTPM_FW_HS);
// wangbing@wind-mobi.com 20170502 end

// wangjun@wind-mobi.com 20170522 begin
    }else if (compatible_flag == 5) { 
        printk("[UPGRADE]: get ykl fw size\n");
        fw_len = sizeof(CTPM_FW_YKL);
// wangjun@wind-mobi.com 20170522 end

    }else {
        FTS_DEBUG("[UPGRADE]:unknown type size\n");
    }
//hebiao@wind-mobi.com end at 20170101 for guoxian and txd compatible
    }
    else if (fw_type == PRAMBOOT_SIZE) {
        fw_len = sizeof(aucFW_PRAM_BOOT_onsell);
    }
    else if (fw_type == LCD_CFG_SIZE) {
        fw_len = sizeof(CTPM_LCD_CFG_ONSELL);
    }
#else
    if (fw_type == FW_SIZE) {
        fw_len = sizeof(CTPM_FW);
    }
    else if (fw_type == LCD_CFG_SIZE) {
        fw_len = sizeof(CTPM_LCD_CFG_ONSELL);
    }
#endif
    return fw_len;
}

/************************************************************************
* Name: fts_ctpm_update_project_setting
* Brief:  update project setting, only update these settings for COB project, or for some special case
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_update_project_setting(struct i2c_client *client)
{
    u8 uc_i2c_addr;
    u8 uc_io_voltage;
    u8 uc_panel_factory_id;
    u8 buf[FTS_SETTING_BUF_LEN];
    u8 reg_val[2] = { 0 };
    u8 auc_i2c_write_buf[10] = { 0 };
    u8 packet_buf[FTS_SETTING_BUF_LEN + 6];
    u32 i = 0;
    int i_ret;

    uc_i2c_addr = client->addr;
    uc_io_voltage = 0x0;
    uc_panel_factory_id = 0x5a;

    /*Step 1:Reset  CTPM */
    if (chip_types_onsell.chip_idh == 0x06 || chip_types_onsell.chip_idh == 0x36) {
        fts_i2c_write_reg_onsell(client, 0xbc, 0xaa);
    }
    else {
        fts_i2c_write_reg_onsell(client, 0xfc, 0xaa);
    }
    msleep(50);

    /*write 0x55 to register 0xfc */
    if (chip_types_onsell.chip_idh == 0x06 || chip_types_onsell.chip_idh == 0x36) {
        fts_i2c_write_reg_onsell(client, 0xbc, 0x55);
    }
    else {
        fts_i2c_write_reg_onsell(client, 0xfc, 0x55);
    }
    msleep(30);

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do {
        i++;
        i_ret = fts_i2c_write_onsell(client, auc_i2c_write_buf, 2);
        msleep(5);
    }
    while (i_ret <= 0 && i < 5);

    /*********Step 3:check READ-ID***********************/
    auc_i2c_write_buf[0] = 0x90;
    auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

    fts_i2c_read_onsell(client, auc_i2c_write_buf, 4, reg_val, 2);

    if (reg_val[0] == chip_types_onsell.pramboot_idh
        && reg_val[1] == chip_types_onsell.pramboot_idl)
        dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
                reg_val[0], reg_val[1]);
    else
        return -EIO;

    auc_i2c_write_buf[0] = 0xcd;
    fts_i2c_read_onsell(client, auc_i2c_write_buf, 1, reg_val, 1);
    dev_dbg(&client->dev, "bootloader version = 0x%x\n", reg_val[0]);

    /*--------- read current project setting  ---------- */
    /*set read start address */
    buf[0] = 0x3;
    buf[1] = 0x0;
    buf[2] = 0x78;
    buf[3] = 0x0;

    fts_i2c_read_onsell(client, buf, 4, buf, FTS_SETTING_BUF_LEN);
    dev_dbg(&client->dev, "[FTS] old setting: uc_i2c_addr = 0x%x,\
            uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n", buf[0], buf[2], buf[4]);

    /*--------- Step 4:erase project setting --------------*/
    auc_i2c_write_buf[0] = 0x63;
    fts_i2c_write_onsell(client, auc_i2c_write_buf, 1);
    msleep(100);

    /*----------  Set new settings ---------------*/
    buf[0] = uc_i2c_addr;
    buf[1] = ~uc_i2c_addr;
    buf[2] = uc_io_voltage;
    buf[3] = ~uc_io_voltage;
    buf[4] = uc_panel_factory_id;
    buf[5] = ~uc_panel_factory_id;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    packet_buf[2] = 0x78;
    packet_buf[3] = 0x0;
    packet_buf[4] = 0;
    packet_buf[5] = FTS_SETTING_BUF_LEN;

    for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
        packet_buf[6 + i] = buf[i];

    fts_i2c_write_onsell(client, packet_buf, FTS_SETTING_BUF_LEN + 6);
    msleep(100);

    /********* reset the new FW***********************/
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write_onsell(client, auc_i2c_write_buf, 1);

    msleep(200);
    return 0;
}

/************************************************************************
* Name: fts_ctpm_get_pram_or_rom_id_onsell
* Brief: 0
* Input: 0
* Output: 0
* Return: 0
***********************************************************************/
unsigned char fts_ctpm_get_pram_or_rom_id_onsell(struct i2c_client *client)
{
    unsigned char auc_i2c_write_buf[4];
    unsigned char reg_val[2] = { 0 };
    unsigned char inRomBoot = 0x00; //0x01 : run in rom boot;  0x02 : run in pram boot
    int i_ret;

    fts_ctpm_i2c_hid2std_onsell(client);

    /*Enter upgrade mode */
    /*send 0x55 in time windows */
    auc_i2c_write_buf[0] = FTS_UPGRADE_55;
    auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
    i_ret = fts_i2c_write_onsell(client, auc_i2c_write_buf, 2);
    if (i_ret < 0) {
        FTS_ERROR("[UPGRADE]: write 0x55, 0xaa fail!!");
    }
    msleep(fts_updateinfo_curr.delay_readid);

    auc_i2c_write_buf[0] = 0x90;
    auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
    fts_i2c_read_onsell(client, auc_i2c_write_buf, 4, reg_val, 2);

    if (reg_val[0] == chip_types_onsell.pramboot_idh
        && reg_val[1] == chip_types_onsell.pramboot_idl) {
        inRomBoot = FTS_RUN_IN_PRAM;
        FTS_DEBUG
            ("[UPGRADE]: read pram boot id for success , pram boot id is: ID1 = 0x%x,ID2 = 0x%x!!",
             reg_val[0], reg_val[1]);
    }
    else if (reg_val[0] == chip_types_onsell.rom_idh
             && reg_val[1] == chip_types_onsell.rom_idl) {
        inRomBoot = FTS_RUN_IN_ROM;
        FTS_DEBUG
            ("[UPGRADE]: read rom boot id for success , rom boot id is: ID1 = 0x%x,ID2 = 0x%x!!",
             reg_val[0], reg_val[1]);
    }
    else if (reg_val[0] == chip_types_onsell.bootloader_idh
             && reg_val[1] == chip_types_onsell.bootloader_idl) {
        inRomBoot = FTS_RUN_IN_BOOTLOADER;
        FTS_DEBUG
            ("[UPGRADE]: read bootloader id for success , bootloader id is: ID1 = 0x%x,ID2 = 0x%x!!",
             reg_val[0], reg_val[1]);
    }
    else {
        FTS_DEBUG
            ("[UPGRADE]: read pram boot id for test error: pram boot id is: ID1 = 0x%x,ID2 = 0x%x!!",
             reg_val[0], reg_val[1]);
    }

    return inRomBoot;
}

/************************************************************************
* Name: fts_ctpm_check_fw_status
* Brief: 0
* Input: 0
* Output: 0
* Return: 0
***********************************************************************/
u8 fts_ctpm_check_fw_status(struct i2c_client * client)
{
    u8 uc_chip_id = 0;
    u8 uc_tp_vendor_id;
    u8 fw_status = 0;
    u8 i = 0;
    unsigned char inRomBoot = 0x00; //0x01 : run in rom boot;  0x02 : run in pram boot
    extern char mtkfb_lcm_name[256];

    FTS_FUNC_ENTER();

    for (i = 0; i < 5; i++) {
        fts_i2c_read_reg_onsell(client, FTS_REG_VENDOR_ID, &uc_tp_vendor_id);
        fts_i2c_read_reg_onsell(client, FTS_REG_CHIP_ID, &uc_chip_id);
//modify by qiangang@wind-mobi.com 20170329 begin
        if ((uc_chip_id == chip_types_onsell.chip_idh) 
             && ((uc_tp_vendor_id == FTS_VENDOR_1_ID) 
             || (uc_tp_vendor_id == FTS_VENDOR_2_ID)
             || (uc_tp_vendor_id == FTS_VENDOR_3_ID) 
             || (uc_tp_vendor_id == FTS_VENDOR_4_ID)
             || (uc_tp_vendor_id == FTS_VENDOR_5_ID)))
        {
//modify by qiangang@wind-mobi.com 20170329 end
            printk("[wjwind] Vender Id = %d\n",uc_tp_vendor_id); 
            break;
        }
        else {
            continue;
        }
    }
    
    /*
    printk("[UPGRADE]: uc_tp_vendor_id = %x\n", uc_tp_vendor_id);
    printk("[UPGRADE]: uc_chip_id = %x\n", uc_chip_id);
    printk("[UPGRADE]: chip_types_onsell.chip_idh = %x\n", chip_types_onsell.chip_idh);
    
    printk("[UPGRADE]: FTS_VENDOR_1_ID = %x\n", FTS_VENDOR_1_ID);
    printk("[UPGRADE]: FTS_VENDOR_2_ID = %x\n", FTS_VENDOR_2_ID);
    printk("[UPGRADE]: FTS_VENDOR_3_ID = %x\n", FTS_VENDOR_3_ID);
    printk("[UPGRADE]: FTS_VENDOR_4_ID = %x\n", FTS_VENDOR_4_ID);
    */

    // printk("ft3427 lcm_name=%s\n", mtkfb_lcm_name);
    if (uc_tp_vendor_id == FTS_VENDOR_1_ID) 
    {
        compatible_flag = 1;
        printk("ft3427 guoxian update\n");
    }
    else if (uc_tp_vendor_id == FTS_VENDOR_2_ID) 
    {
        compatible_flag = 2;
        printk("ft3427 txd update\n");
    }
    else if (uc_tp_vendor_id == FTS_VENDOR_3_ID)    //add by qiangang@wind-mobi.com 20170329 begin
    {
        compatible_flag = 3;
        printk("ft3427 hlt update\n");
    }                                               //add by qiangang@wind-mobi.com 20170329 end
    else if (uc_tp_vendor_id == FTS_VENDOR_4_ID)    // wangbing@wind-mobi.com 20170502 begin
    {
        compatible_flag = 4;
        printk("ft3427 hs update\n");
    }                                               // wangbing@wind-mobi.com 20170502 end
      else if (uc_tp_vendor_id == FTS_VENDOR_5_ID)    // wangjun@wind-mobi.com 20170522 begin
    {
        compatible_flag = 5;
        printk("ft3427 ykl update\n");
    }                                               // wangjun@wind-mobi.com 20170522 end
    else if (strcmp(mtkfb_lcm_name, "otm1289a_hsd_dsi_vdo_gx") == 0) 
    {
        compatible_flag = 1;
        force_update = 1;
        printk("ft3427 can't read vendor, lcd is otm1289a_hsd_dsi_vdo_gx, force update\n");
    }
    else if (strcmp(mtkfb_lcm_name, "otm1289a_hsd_dsi_vdo_txd") == 0) 
    {
        compatible_flag = 2;
        force_update = 1;
        printk("ft3427 can't read vendor, lcd is otm1289a_hsd_dsi_vdo_txd, force update\n");
    }
    else if (strcmp(mtkfb_lcm_name, "hx8394f_hd720_dsi_vdo_hlt") == 0)      //add by qiangang@wind-mobi.com 20170329 begin
    {
        compatible_flag = 3;
        force_update = 1;
        printk("ft3427 can't read vendor, lcd is hx8394f_hd720_dsi_vdo_hlt, force update\n");
    }                                                                       //add by qiangang@wind-mobi.com 20170329 end
    else if (strcmp(mtkfb_lcm_name, "s6d7aa6x01_hd720_dsi_vdo_hs") == 0)    // wangbing@wind-mobi.com 20170502 begin
    {
        compatible_flag = 4;
        force_update = 1;
        printk("ft3427 can't read vendor, lcd is s6d7aa6x01_hd720_dsi_vdo_hs, force update\n");
    }                                                                       // wangbing@wind-mobi.com 20170502 end
    else if (strcmp(mtkfb_lcm_name, "ili9881c_hd720_dsi_vdo_boe_ykl") == 0)    // wangjun@wind-mobi.com 20170522 begin
    {
        compatible_flag = 5;
        force_update = 1;
        printk("ft3427 can't read vendor, lcd is s6d7aa6x01_hd720_dsi_vdo_hs, force update\n");
    }                                                                            // wangjun@wind-mobi.com 20170522 end
    else 
    {
        printk("ft3427 unknown type not update\n");
    }

    if ((uc_chip_id == chip_types_onsell.chip_idh) 
            && ((uc_tp_vendor_id == FTS_VENDOR_1_ID) 
            || (uc_tp_vendor_id == FTS_VENDOR_2_ID) 
            || (uc_tp_vendor_id == FTS_VENDOR_3_ID) 
            || (uc_tp_vendor_id == FTS_VENDOR_4_ID)
            || (uc_tp_vendor_id == FTS_VENDOR_5_ID)))  // call fts_flash_get_upgrade_info in probe function firstly.
    {
        fw_status = FTS_RUN_IN_APP;
        printk("[UPGRADE]: APP OK!!\n");
    }
    else if ((uc_tp_vendor_id == FTS_VENDOR_4_ID) || (uc_tp_vendor_id == FTS_VENDOR_5_ID)) 
    {
        inRomBoot = fts_ctpm_get_pram_or_rom_id_onsell(client);
        FTS_DEBUG("[UPGRADE]: inRomBoot = %d!!", inRomBoot);
        if (inRomBoot == FTS_RUN_IN_ROM) 
        {
            fw_status = FTS_RUN_IN_ROM;
            FTS_INFO("[UPGRADE]: run in rom!!");
        }
        else 
        {
            FTS_INFO("[UPGRADE]: not run in rom!!");
        }

        FTS_INFO("[UPGRADE]: APP invalid!!");
    }
    else 
    {
        inRomBoot = fts_ctpm_get_pram_or_rom_id_onsell(client);
        FTS_DEBUG("[UPGRADE]: inRomBoot = %d!!", inRomBoot);
        if (inRomBoot == FTS_RUN_IN_PRAM) {
            fw_status = FTS_RUN_IN_PRAM;
            FTS_INFO("[UPGRADE]: run in pram!!");
        }
        if (inRomBoot == FTS_RUN_IN_BOOTLOADER) {
            fw_status = FTS_RUN_IN_BOOTLOADER;
            FTS_INFO("[UPGRADE]: run in bootloader!!");
        }
        else {
            FTS_INFO("[UPGRADE]: not run in pram!!");
        }

        FTS_INFO("[UPGRADE]: APP invalid!!");
    }
    
    printk("[UPGRADE]: fw_status = %d\n", fw_status);
    FTS_FUNC_EXIT();

    return fw_status;
}

/************************************************************************
* Name: fts_ctpm_check_need_upgrade
* Brief: 0
* Input: 0
* Output: 0
* Return: 0
***********************************************************************/
bool fts_ctpm_check_need_upgrade(struct i2c_client * client)
{
    int i = 0;
    u8 fw_status = 0;
    bool bUpgradeFlag = false;
    u8 uc_tp_fm_ver;
    u8 uc_host_fm_ver = 0;

    FTS_FUNC_ENTER();

    for (i = 0; i < 5; i++) 
    {
        fw_status = fts_ctpm_check_fw_status(client);
        if (!fw_status) 
        {
            msleep(5);
        }
        else 
        {
            break;
        }
    }

    if (fw_status == FTS_RUN_IN_APP)    //call fts_flash_get_upgrade_info in probe function firstly.
    {
        fts_i2c_read_reg_onsell(client, FTS_REG_FW_VER, &uc_tp_fm_ver);

        uc_host_fm_ver = fts_ctpm_get_app_ver_onsell();

        printk("[UPGRADE]: ic fw version = 0x%x, *.i file fw version = 0x%x \n", uc_tp_fm_ver, uc_host_fm_ver);
        if (uc_tp_fm_ver < uc_host_fm_ver) 
        {
            bUpgradeFlag = true;
        }
    }
    else if (force_update == 1) 
    {
        bUpgradeFlag = true;
    }
    else if (fw_status == FTS_RUN_IN_ROM) 
    {
        FTS_DEBUG("[UPGRADE]: run in rom!!");
        bUpgradeFlag = true;
    }
    else if (fw_status == FTS_RUN_IN_BOOTLOADER) 
    {
        FTS_DEBUG("[UPGRADE]: run in bootloader!!");
        bUpgradeFlag = true;
    }
    else if (fw_status == FTS_RUN_IN_PRAM) 
    {
        FTS_DEBUG("[UPGRADE]: run in pram, reset and upgrade!!");
        fts_ctpm_rom_or_pram_reset_onsell(client, true);
        bUpgradeFlag = true;
    } 

    FTS_FUNC_EXIT();
    // printk("qiangang bUpgradeFlag1 = %d\n", bUpgradeFlag);

    return bUpgradeFlag;
}

#if (FTS_UPGRADE_WITH_APP_BIN_EN)
/************************************************************************
* Name: fts_ctpm_get_app_i_file_ver
* Brief:  get .i file version
* Input: no
* Output: no
* Return: fw version
***********************************************************************/
int fts_ctpm_get_app_ver_onsell(void)
{
    return fts_updatefun_curr_onsell.get_app_bin_file_ver(FTS_UPGRADE_FW_APP_BIN);
}

int fts_ctpm_fw_upgrade_onsell(struct i2c_client *client)
{
    int i_ret = 0;
    i_ret =
        fts_updatefun_curr_onsell.upgrade_with_app_bin_file(client,
                                                     FTS_UPGRADE_FW_APP_BIN);
    if (i_ret < 0) {
        FTS_ERROR("[UPGRADE]: app.bin upgrade failed!!");
        return i_ret;
    }
    return 0;
}

#else
/************************************************************************
* Name: fts_ctpm_get_app_i_file_ver
* Brief:  get .i file version
* Input: no
* Output: no
* Return: fw version
***********************************************************************/
int fts_ctpm_get_app_ver_onsell(void)
{
    return fts_updatefun_curr_onsell.get_app_i_file_ver();
}

int fts_ctpm_fw_upgrade_onsell(struct i2c_client *client)
{
    int i_ret = 0;
    i_ret = fts_updatefun_curr_onsell.upgrade_with_app_i_file(client);
    if (i_ret < 0) {
        FTS_ERROR("[UPGRADE]: app.i upgrade failed!!");
        return i_ret;
    }
    return 0;
}

#endif

#if FTS_UPGRADE_LCD_CFG_BIN_EN
int fts_ctpm_lcd_cfg_upgrade_onsell(struct i2c_client *client)
{
    int i_ret = 0;
    i_ret =
        fts_updatefun_curr_onsell.upgrade_with_lcd_cfg_bin_file(client,
                                                         FTS_UPGRADE_LCD_CFG_BIN);
    if (i_ret < 0) {
        FTS_ERROR("[UPGRADE]: lcd_cfg.bin upgrade failed!!");
        return i_ret;
    }
    return 0;
}

#else
int fts_ctpm_lcd_cfg_upgrade_onsell(struct i2c_client *client)
{
    int i_ret = 0;
    i_ret = fts_updatefun_curr_onsell.upgrade_with_lcd_cfg_i_file(client);
    if (i_ret < 0) {
        FTS_ERROR("[UPGRADE]: lcd_cfg.i upgrade failed!!");
        return i_ret;
    }
    return 0;
}

#endif
/************************************************************************
* Name: fts_ctpm_auto_upgrade_onsell
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
#if (!(FTS_UPGRADE_PINGPONG_TEST))
int fts_ctpm_auto_upgrade_onsell(struct i2c_client *client)
{
    u8 uc_tp_fm_ver;
    int i_ret = 0;
    bool bUpgradeFlag = false;
    u8 uc_upgrade_times = 0;

    FTS_FUNC_ENTER();

    /* check need upgrade or not */
    bUpgradeFlag = fts_ctpm_check_need_upgrade(client);
    printk("[UPGRADE]: qiangangbUpgradeFlag2 = 0x%x\n", bUpgradeFlag);
//modify by qiangang@wind-mobi.com 20170329 begin
    // bUpgradeFlag = 0;
//modify by qiangang@wind-mobi.com 20170329 end
    /* will upgrade */
    if (bUpgradeFlag) {
        printk("[UPGRADE]: qiangangneed upgrade!!\n");
        do {
            uc_upgrade_times++;

            /* esd check */
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(DISABLE);
#endif

            /* fw upgrade */
            i_ret = fts_ctpm_fw_upgrade_onsell(client);

#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(ENABLE);
#endif
            if (i_ret == 0)                 /* upgrade success */
            {
                fts_i2c_read_reg_onsell(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
                //printk("[UPGRADE]:qiangang upgrade to new version 0x%x", uc_tp_fm_ver);
#ifdef CONFIG_WIND_DEVICE_INFO
                // msleep(5);
                {
                    u8 ver;
                    fts_i2c_read_reg_onsell(client, 0xA8, &ver);
                    sprintf(wind_device_info.ctp_module_info.ic_name, "%s", "ft3427");
                    wind_device_info.ctp_module_info.fwvr = uc_tp_fm_ver;
                    wind_device_info.ctp_module_info.vendor = ver;
                }
#endif
            }
            else {              /* upgrade fail */

                /* if upgrade fail, reset to run ROM. if app in flash is ok. TP will work success */
                //printk("[UPGRADE]: qiangangupgrade fail, reset now!!\n");
                fts_ctpm_rom_or_pram_reset_onsell(client, (!(uc_upgrade_times - 1)));
            }
        }
        while ((i_ret != 0) && (uc_upgrade_times < 2)); /* if upgrade fail, upgrade again. then return */
    }

    FTS_FUNC_EXIT();
    return i_ret;
}
#endif

static void fts_ctpm_update_work_func(struct work_struct *work)
{
#if FTS_AUTO_UPGRADE_FOR_LCD_CFG_EN
    int i_ret = 0;
#endif

    FTS_DEBUG("[UPGRADE]:FTS enter upgrade!!");
    disable_irq(ft_touch_irq);

    fts_ctpm_auto_upgrade_onsell(fts_i2c_client_onsell);

#if FTS_AUTO_UPGRADE_FOR_LCD_CFG_EN
    msleep(2000);

    /* lcd_cfg upgrade */
    i_ret = fts_ctpm_lcd_cfg_upgrade_onsell(fts_i2c_client_onsell);
    if (i_ret < 0) {            /* lcd cfg upgrade fail */
        FTS_DEBUG("[UPGRADE]: upgrade lcd cfg fail!!");
    }
#endif

    enable_irq(ft_touch_irq);
}


static int fts_ctpm_workqueue_init(void)
{
    touch_wq_onsell = create_singlethread_workqueue("touch_wq_onsell");
    if (touch_wq_onsell) {
        INIT_WORK(&fw_update_work_onsell, fts_ctpm_update_work_func);
    }
    else {
        goto err_workqueue_init;
    }
    return 0;

err_workqueue_init:
    FTS_ERROR("create_singlethread_workqueue failed\n");
    return -1;
}



#if FTS_AUTO_UPGRADE_EN
void fts_ctpm_upgrade_init(void)
{
    int retval = 0;

    FTS_FUNC_ENTER();

    retval = fts_ctpm_workqueue_init();
    if (retval != 0) {
        FTS_ERROR("[UPGRADE]fts_ctpm_workqueue_init failed!");
    }
    else
        queue_work(touch_wq_onsell, &fw_update_work_onsell);

    FTS_FUNC_EXIT();
}

void fts_ctpm_upgrade_exit(void)
{
    FTS_FUNC_ENTER();

    cancel_work_sync(&fw_update_work_onsell);

    FTS_FUNC_EXIT();
}
#endif
