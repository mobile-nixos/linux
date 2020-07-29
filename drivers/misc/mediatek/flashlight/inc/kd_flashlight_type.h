/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _KD_FLASHLIGHT_TYPE_H
#define _KD_FLASHLIGHT_TYPE_H


/* ------------------------*/
/* Basic Type Definitions */
/* -----------------------*/

typedef long LONG;
typedef unsigned char UBYTE;
typedef short SHORT;

typedef signed char kal_int8;
typedef signed short kal_int16;
typedef signed int kal_int32;
typedef long long kal_int64;
typedef unsigned char kal_uint8;
typedef unsigned short kal_uint16;
typedef unsigned int kal_uint32;
typedef unsigned long long kal_uint64;
typedef char kal_char;

typedef unsigned int *UINT32P;
typedef volatile unsigned short *UINT16P;
typedef volatile unsigned char *UINT8P;
typedef unsigned char *U8P;


typedef unsigned char U8;
typedef signed char S8;
typedef unsigned short U16;
typedef signed short S16;
typedef unsigned int U32;
typedef signed int S32;
typedef unsigned long long U64;
typedef signed long long S64;
/* typedef unsigned char       bool; */

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef unsigned short USHORT;
typedef signed char INT8;
typedef signed short INT16;
typedef signed int INT32;
typedef unsigned int DWORD;
typedef void VOID;
typedef unsigned char BYTE;
typedef float FLOAT;

typedef char *LPCSTR;
typedef short *LPWSTR;


/* -----------*/
/* Constants */
/* ----------*/
#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
#define TRUE  (1)
#endif

#ifndef NULL
#define NULL  (0)
#endif


#ifndef BOOL
typedef unsigned char BOOL;
#endif

/* temp define for 3643 dual functions */
int flashEnable_lm3643_2(void);
int flashDisable_lm3643_2(void);
int setDuty_lm3643_2(int duty);

/* Function declaration to avoid extern usage in .c files, for kernel standardization */
/* New init functions */
int strobe_getPartId(int sensorDev, int strobeId);
MUINT32 strobeInit_dummy(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 constantFlashlightInit(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 strobeInit_main_sid1_part2(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 strobeInit_main_sid2_part1(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 strobeInit_main_sid2_part2(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 subStrobeInit(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 strobeInit_sub_sid1_part2(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 strobeInit_sub_sid2_part1(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);
MUINT32 strobeInit_sub_sid2_part2(FLASHLIGHT_FUNCTION_STRUCT **pfFunc);

/* Old init function */
MUINT32 dummyFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 defaultFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 peakFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 torchFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 strobeInit_sub_sid1_part2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 strobeInit_main_sid1_part2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);

/* Tablet version */
MUINT32 subStrobeInit_2ndPart_2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);
MUINT32 mainStrobeInit_2ndPart_2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc);

/* Externs from imgsensor*/
int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

/* Externs from xhci*/
bool mtk_is_host_mode(void);

/* Boost4Flash */
int mt6332_OpenBoost4Flash(void);
int mt6332_CloseBoost4Flash(void);

/* GPIO pinctrl */
int flashlight_gpio_init(struct platform_device *pdev);
int flashlight_gpio_set(int pin, int state);
int flashlight_gpio_hwen_high(void);
int flashlight_gpio_hwen_low(void);
#endif				/* _KD_FLASHLIGHT_TYPE_H */
