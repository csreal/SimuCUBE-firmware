/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CUSTOM_HID 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
//simucube axel defines

#define NB_AXIS			8
#define NB_FF_AXIS		1

#define X_AXIS_NB_BITS	16
#define Y_AXIS_NB_BITS	16
#define Z_AXIS_NB_BITS	16
#define RX_AXIS_NB_BITS	16
#define RY_AXIS_NB_BITS	16
#define RZ_AXIS_NB_BITS	16
#define SX_AXIS_NB_BITS	16
#define SY_AXIS_NB_BITS	16
#define NB_BUTTONS		32

#define X_AXIS_LOG_MAX		((1L<<(X_AXIS_NB_BITS))-1)
#define X_AXIS_LOG_MIN		0//(-X_AXIS_LOG_MAX)
#define X_AXIS_PHYS_MAX		((1L<<X_AXIS_NB_BITS)-1)

#define Y_AXIS_LOG_MAX		((1L<<(Y_AXIS_NB_BITS))-1)
#define Y_AXIS_LOG_MIN		0//(-Y_AXIS_LOG_MAX)
#define Y_AXIS_PHYS_MAX		((1L<<Y_AXIS_NB_BITS)-1)

#define Z_AXIS_LOG_MAX		((1L<<(Z_AXIS_NB_BITS))-1)
#define Z_AXIS_LOG_MIN		0//(-Z_AXIS_LOG_MAX)
#define Z_AXIS_PHYS_MAX		((1L<<Z_AXIS_NB_BITS)-1)

#define RX_AXIS_LOG_MAX		((1L<<(RX_AXIS_NB_BITS))-1)
#define RX_AXIS_LOG_MIN		0//(-RX_AXIS_LOG_MAX)
#define RX_AXIS_PHYS_MAX	((1L<<RX_AXIS_NB_BITS)-1)

#define RY_AXIS_LOG_MAX		((1L<<(RY_AXIS_NB_BITS))-1)
#define RY_AXIS_LOG_MIN		0//(-RY_AXIS_LOG_MAX)
#define RY_AXIS_PHYS_MAX	((1L<<RY_AXIS_NB_BITS)-1)

#define RZ_AXIS_LOG_MAX		((1L<<(RZ_AXIS_NB_BITS))-1)
#define RZ_AXIS_LOG_MIN		0//(-RZ_AXIS_LOG_MAX)
#define RZ_AXIS_PHYS_MAX	((1L<<RZ_AXIS_NB_BITS)-1)

#define SX_AXIS_LOG_MAX		((1L<<(SX_AXIS_NB_BITS))-1)
#define SX_AXIS_LOG_MIN		0//(-SX_AXIS_LOG_MAX)
#define SX_AXIS_PHYS_MAX	((1L<<SX_AXIS_NB_BITS)-1)

#define SY_AXIS_LOG_MAX		((1L<<(SY_AXIS_NB_BITS))-1)
#define SY_AXIS_LOG_MIN		0//(-SY_AXIS_LOG_MAX)
#define SY_AXIS_PHYS_MAX	((1L<<SY_AXIS_NB_BITS)-1)

#define TRANSFER_PGM		0x80
#define TRANSFER_RELEASE	0x40
#define TRANSFER_ZERO		0x20

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros
  * @{
  */
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables
 * @{
 */



#if default_usb_customhid
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{

		  /* USER CODE BEGIN 0 */
		  0x06, 0x00, 0xFF,//USAGE_PAGE (Vendor Defined Page 1)       // 3 B
		  0x09, 0x00,      //USAGE (Undefined)                        // 5 B
		  0xA1, 0x01,      //COLLECTION (Application)                 // 7 B
		  0x75, 0x08,      //  REPORT_SIZE (8)                        // 9 B
		  0x95, 0x01,      //  REPORT_COUNT (1)                       // 11B
		  0x92, 0xA3, 0x01,//  OUTPUT (Cnst, Var, Abs, NPrf, Vol, Buf)// 14B
		  /* USER CODE END 0 */
		  0xC0    /*     END_COLLECTION              */
}; 
#else

__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE]  __ALIGN_END =
{
		0x05, 0x01,                     // Usage Page (Generic Desktop)
		0x09, 0x04,                     // Usage (Joystick)
		0xA1, 0x01,                     // Collection (Application)
		0x85, 4,	// REPORT_ID (REPORT_ID_JOYSTICK)

			0x09, 0x01,                     // Usage (Pointer)
	        0xA1, 0x00,                     // Collection ()
				0x09, 0x30,					// USAGE (x)
				0x16,X_AXIS_LOG_MIN & 0xFF,(X_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,X_AXIS_LOG_MAX & 0xFF,(X_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,X_AXIS_PHYS_MAX & 0xFF,(X_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(X_AXIS_PHYS_MAX >> 16) & 0xFF,(X_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,X_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)
				0x09,0x31,					// USAGE (y)
				0x16,Y_AXIS_LOG_MIN & 0xFF,(Y_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,Y_AXIS_LOG_MAX & 0xFF,(Y_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,Y_AXIS_PHYS_MAX & 0xFF,(Y_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(Y_AXIS_PHYS_MAX >> 16) & 0xFF,(Y_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,Y_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)

				0x09,0x32,					// USAGE (z)
				0x16,Z_AXIS_LOG_MIN & 0xFF,(Z_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,Z_AXIS_LOG_MAX & 0xFF,(Z_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,Z_AXIS_PHYS_MAX & 0xFF,(Z_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(Z_AXIS_PHYS_MAX >> 16) & 0xFF,(Z_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,Z_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)

				0x09,0x33, // USAGE (Pointer)
				0x16,RX_AXIS_LOG_MIN & 0xFF,(RX_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,RX_AXIS_LOG_MAX & 0xFF,(RX_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,RX_AXIS_PHYS_MAX & 0xFF,(RX_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(RX_AXIS_PHYS_MAX >> 16) & 0xFF,(RX_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,RX_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)
				0x09,0x34,					// USAGE (y)
				0x16,RY_AXIS_LOG_MIN & 0xFF,(RY_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,RY_AXIS_LOG_MAX & 0xFF,(RY_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,RY_AXIS_PHYS_MAX & 0xFF,(RY_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(RY_AXIS_PHYS_MAX >> 16) & 0xFF,(RY_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,RY_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)

				0x09,0x35,					// USAGE (z)
				0x16,RZ_AXIS_LOG_MIN & 0xFF,(RZ_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,RZ_AXIS_LOG_MAX & 0xFF,(RZ_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,RZ_AXIS_PHYS_MAX & 0xFF,(RZ_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(RZ_AXIS_PHYS_MAX >> 16) & 0xFF,(RZ_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,RZ_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)
				0x09,0x36,					// USAGE (z)
				0x16,SX_AXIS_LOG_MIN & 0xFF,(SX_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,SX_AXIS_LOG_MAX & 0xFF,(SX_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,SX_AXIS_PHYS_MAX & 0xFF,(SX_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(SX_AXIS_PHYS_MAX >> 16) & 0xFF,(SX_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,SX_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)
				0x09,0x37,					// USAGE (z)
				0x16,SY_AXIS_LOG_MIN & 0xFF,(SY_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
				0x27,SY_AXIS_LOG_MAX & 0xFF,(SY_AXIS_LOG_MAX >> 8) & 0xFF,0,0, // LOGICAL_MAXIMUM
				0x35,0x00,					// PHYSICAL_MINIMUM (00)
				0x47,SY_AXIS_PHYS_MAX & 0xFF,(SY_AXIS_PHYS_MAX >> 8) & 0xFF,0,0,//(SY_AXIS_PHYS_MAX >> 16) & 0xFF,(SY_AXIS_PHYS_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM (0xffff)
				0x75,SY_AXIS_NB_BITS,		// REPORT_SIZE (AXIS_NB_BITS)
				0x95,1,						// REPORT_COUNT (1)
				0x81,0x02,					// INPUT (Data,Var,Abs)
			0xC0,                           // End Collection

			0x15, 0x00,                     // Logical Minimum (0)
			0x25, 0x01,                     // Logical Maximum (1)
			0x75, 0x01,                     // Report Size (1)
			0x95, 0x20,                     // Report Count (32)
			0x05, 0x09,                     // Usage Page (Button)
			0x19, 0x01,                     // Usage Minimum (Button #1)
			0x29, 0x20,                     // Usage Maximum (Button #32)
			0x81, 0x02,                     // Input (variable,absolute)

			0x05,0x0F,	// USAGE_PAGE (Physical Interface)
			0x09,0x92,	// USAGE (PID State Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x02,	// REPORT_ID (02)
				0x09,0x9F,	// USAGE (Device Paused)
				0x09,0xA0,	// USAGE (Actuators Enabled)
				0x09,0xA4,	// USAGE (Safety Switch)
				0x09,0xA5,	// USAGE (Actuator Override Switch)
				0x09,0xA6,	// USAGE (Actuator Power)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x25,0x01,	// LOGICAL_MINIMUM (01)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x45,0x01,	// PHYSICAL_MAXIMUM (01)
				0x75,0x01,	// REPORT_SIZE (01)
				0x95,0x05,	// REPORT_COUNT (05)
				0x81,0x02,	// INPUT (Data,Var,Abs)
				0x95,0x03,	// REPORT_COUNT (03)
				0x81,0x03,	// INPUT (Constant,Var,Abs)
				0x09,0x94,	// USAGE (Effect Playing)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x25,0x01,	// LOGICAL_MAXIMUM (01)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x45,0x01,	// PHYSICAL_MAXIMUM (01)
				0x75,0x01,	// REPORT_SIZE (01)
				0x95,0x01,	// REPORT_COUNT (01)
				0x81,0x02,	// INPUT (Data,Var,Abs)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x07,	// REPORT_SIZE (07)
				0x95,0x01,	// REPORT_COUNT (01)
				0x81,0x02,	// INPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x09,0x21,	// USAGE (Set Effect Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x01,	// REPORT_ID (01)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x25,	// USAGE (25)
				0xA1,0x02,	// COLLECTION (Logical)
					0x09,0x26,	// USAGE (26)
					0x09,0x27,	// USAGE (27)
					0x09,0x30,	// USAGE (30)
					0x09,0x31,	// USAGE (31)
					0x09,0x32,	// USAGE (32)
					0x09,0x33,	// USAGE (33)
					0x09,0x34,	// USAGE (34)
					0x09,0x40,	// USAGE (40)
					0x09,0x41,	// USAGE (41)
					0x09,0x42,	// USAGE (42)
					0x09,0x43,	// USAGE (43)
					0x09,0x28,	// USAGE (28)
					0x25,0x0C,	// LOGICAL_MAXIMUM (0C)
					0x15,0x01,	// LOGICAL_MINIMUM (01)
					0x35,0x01,	// PHYSICAL_MINIMUM (01)
					0x45,0x0C,	// PHYSICAL_MAXIMUM (0C)
					0x75,0x08,	// REPORT_SIZE (08)
					0x95,0x01,	// REPORT_COUNT (01)
					0x91,0x00,	// OUTPUT (Data)
				0xC0,	// END COLLECTION ()
				0x09,0x50,	// USAGE (Duration)
				0x09,0x54,	// USAGE (Trigger Repeat Interval)
				0x09,0x51,	// USAGE (Sample Period)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x7F,	// LOGICAL_MAXIMUM (7F FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x7F,	// PHYSICAL_MAXIMUM (7F FF)
				0x66,0x03,0x10,	// UNIT (Eng Lin:Time)
				0x55,0xFD,	// UNIT_EXPONENT (-3)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x03,	// REPORT_COUNT (03)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x55,0x00,	// UNIT_EXPONENT (00)
				0x66,0x00,0x00,	// UNIT (None)
				0x09,0x52,	// USAGE (Gain)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x53,	// USAGE (Trigger Button)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x08,	// LOGICAL_MAXIMUM (08)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x08,	// PHYSICAL_MAXIMUM (08)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x55,	// USAGE (Axes Enable)
				0xA1,0x02,	// COLLECTION (Logical)
					0x05,0x01,	// USAGE_PAGE (Generic Desktop)
					0x09,0x30,	// USAGE (X)

					0x15,0x00,	// LOGICAL_MINIMUM (00)
	 				0x25,0x01,	// LOGICAL_MAXIMUM (01)
					0x75,0x01,	// REPORT_SIZE (01)
					0x95,NB_FF_AXIS,	// REPORT_COUNT (NB_FF_AXIS)
					0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0xC0,	// END COLLECTION ()
				0x05,0x0F,	// USAGE_PAGE (Physical Interface)
				0x09,0x56,	// USAGE (Direction Enable)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x95,0x07 - NB_FF_AXIS,	// REPORT_COUNT (05 (2 axes) or 06 (1 axes)) seems to be for padding
				0x91,0x03,	// OUTPUT (Constant,Var,Abs)
				0x09,0x57,	// USAGE (Direction)
				0xA1,0x02,	// COLLECTION (Logical)
					0x0B,0x01,0x00,0x0A,0x00,
					0x0B,0x02,0x00,0x0A,0x00,
					0x66,0x14,0x00,	// UNIT (Eng Rot:Angular Pos)
					0x55,0xFE,	// UNIT_EXPONENT (FE)
					0x15,0x00,	// LOGICAL_MINIMUM (00)
					0x26,0xB4,0x00,	// LOGICAL_MAXIMUM (00 B4)
					0x35,0x00,	// PHYSICAL_MINIMUM (00)
					0x47,0xA0,0x8C,0x00,0x00,	// PHYSICAL_MAXIMUM (00 00 8C A0)
					0x66,0x00,0x00,	// UNIT (None)
					0x75,0x08,	// REPORT_SIZE (08)
					0x95,0x01,	// REPORT_COUNT (01)		//0x02,	// REPORT_COUNT (02)
					0x91,0x02,	// OUTPUT (Data,Var,Abs)
					0x55,0x00,	// UNIT_EXPONENT (00)
					0x66,0x00,0x00,	// UNIT (None)
				0xC0,	// END COLLECTION ()
				0x05,0x0F,	// USAGE_PAGE (Physical Interface)
	//			0x09,0xA7,	// USAGE (Start Delay)
				0x66,0x03,0x10,	// UNIT (Eng Lin:Time)
				0x55,0xFD,	// UNIT_EXPONENT (-3)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x7F,	// LOGICAL_MAXIMUM (7F FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x7F,	// PHYSICAL_MAXIMUM (7F FF)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x01,	// REPORT_COUNT (01)
	//			0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x66,0x00,0x00,	// UNIT (None)
				0x55,0x00,	// UNIT_EXPONENT (00)
			0xC0,	// END COLLECTION ()

			0x05,0x0F,	// USAGE_PAGE (Physical Interface)
			0x09,0x5A,	// USAGE (Set Envelope Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x02,	// REPORT_ID (02)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x5B,	// USAGE (Attack Level)
				0x09,0x5D,	// USAGE (Fade Level)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x95,0x02,	// REPORT_COUNT (02)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x5C,	// USAGE (5C)
				0x09,0x5E,	// USAGE (5E)
				0x66,0x03,0x10,	// UNIT (Eng Lin:Time)
				0x55,0xFD,	// UNIT_EXPONENT (-3)
				0x26,0xFF,0x7F,	// LOGICAL_MAXIMUM (7F FF)
				0x46,0xFF,0x7F,	// PHYSICAL_MAXIMUM (7F FF)
				0x75,0x10,	// REPORT_SIZE (10)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x45,0x00,	// PHYSICAL_MAXIMUM (00)
				0x66,0x00,0x00,	// UNIT (None)
				0x55,0x00,	// UNIT_EXPONENT (00)
			0xC0,	// END COLLECTION ()

			0x09,0x5F,	// USAGE (Set Condition Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x03,	// REPORT_ID (03)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x23,	// USAGE (Parameter Block Offset)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x25,0x01,	// LOGICAL_MAXIMUM (01)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x45,0x01,	// PHYSICAL_MAXIMUM (01)
				0x75,0x04,	// REPORT_SIZE (04)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x58,	// USAGE (Type Specific Block Offset)
				0xA1,0x02,	// COLLECTION (Logical)
					0x0B,0x01,0x00,0x0A,0x00,	// USAGE (Instance 1)
					0x0B,0x02,0x00,0x0A,0x00,	// USAGE (Instance 2)
					0x75,0x02,	// REPORT_SIZE (02)
					0x95,0x02,	// REPORT_COUNT (02)
					0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0xC0,	// END COLLECTION ()
				0x15,0x80,	// LOGICAL_MINIMUM (80)
				0x25,0x7F,	// LOGICAL_MAXIMUM (7F)
				0x36,0xF0,0xD8,	// PHYSICAL_MINIMUM (-10000)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x09,0x60,	// USAGE (CP Offset)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x36,0xF0,0xD8,	// PHYSICAL_MINIMUM (-10000)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x09,0x61,	// USAGE (Positive Coefficient)
	//			0x09,0x62,	// USAGE (Negative Coefficient)
				0x95,0x01,	// REPORT_COUNT (01)	// ???? WAS 2 with "negative coeff"
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x09,0x63,	// USAGE (Positive Saturation)
				0x09,0x64,	// USAGE (Negative Saturation)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x02,	// REPORT_COUNT (02)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
	//			0x09,0x65,	// USAGE (Dead Band )
	//			0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
	//			0x95,0x01,	// REPORT_COUNT (01)
	//			0x91,0x02,	// OUTPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x09,0x6E,	// USAGE (Set Periodic Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x04,	// REPORT_ID (04)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x70,	// USAGE (Magnitude)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x6F,	// USAGE (Offset)
				0x15,0x80,	// LOGICAL_MINIMUM (80)
				0x25,0x7F,	// LOGICAL_MAXIMUM (7F)
				0x36,0xF0,0xD8,	// PHYSICAL_MINIMUM (-10000)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x71,	// USAGE (Phase)
				0x66,0x14,0x00,	// UNIT (Eng Rot:Angular Pos)
				0x55,0xFE,	// UNIT_EXPONENT (FE)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x47,0xA0,0x8C,0x00,0x00,	// PHYSICAL_MAXIMUM (00 00 8C A0)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x72,	// USAGE (Period)
				0x26,0xFF,0x7F,	// LOGICAL_MAXIMUM (7F FF)
				0x46,0xFF,0x7F,	// PHYSICAL_MAXIMUM (7F FF)
				0x66,0x03,0x10,	// UNIT (Eng Lin:Time)
				0x55,0xFD,	// UNIT_EXPONENT (-3)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x66,0x00,0x00,	// UNIT (None)
				0x55,0x00,	// UNIT_EXPONENT (00)
			0xC0,	// END COLLECTION ()

			0x09,0x73,	// USAGE (Set Constant Force Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x05,	// REPORT_ID (05)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x70,	// USAGE (Magnitude)
				0x16,0x01,0xFF,	// LOGICAL_MINIMUM (-255)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (255)
				0x36,0xF0,0xD8,	// PHYSICAL_MINIMUM (-10000)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x09,0x74,	// USAGE (Set Ramp Force Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x06,	// REPORT_ID (06)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x75,	// USAGE (Ramp Start)
				0x09,0x76,	// USAGE (Ramp End)
				0x15,0x80,	// LOGICAL_MINIMUM (-128)
				0x25,0x7F,	// LOGICAL_MAXIMUM (127)
				0x36,0xF0,0xD8,	// PHYSICAL_MINIMUM (-10000)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x02,	// REPORT_COUNT (02)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x09,0x68,	// USAGE (Custom Force Data Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x07,	// REPORT_ID (07)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x6C,	// USAGE (Custom Force Data Offset)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0x10,0x27,	// LOGICAL_MAXIMUM (10000)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x69,	// USAGE (Custom Force Data)
				0x15,0x81,	// LOGICAL_MINIMUM (-127)
				0x25,0x7F,	// LOGICAL_MAXIMUM (127)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x00,	// PHYSICAL_MAXIMUM (255)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x0C,	// REPORT_COUNT (0C)
				0x92,0x02,0x01,	// OUTPUT ( Data,Var,Abs,Buf)
			0xC0,	// END COLLECTION ()

			0x09,0x66,	// USAGE (Download Force Sample)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x08,	// REPORT_ID (08)
				0x05,0x01,	// USAGE_PAGE (Generic Desktop)
				0x09,0x30,	// USAGE (X)
				0x09,0x31,	// USAGE (Y)
				0x15,0x81,	// LOGICAL_MINIMUM (-127)
				0x25,0x7F,	// LOGICAL_MAXIMUM (127)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x00,	// PHYSICAL_MAXIMUM (255)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x02,	// REPORT_COUNT (02)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x05,0x0F,	// USAGE_PAGE (Physical Interface)
			0x09,0x77,	// USAGE (Effect Operation Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x0A,	// REPORT_ID (0A)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x78,	// USAGE (78)
				0xA1,0x02,	// COLLECTION (Logical)
					0x09,0x79,	// USAGE (Op Effect Start)
					0x09,0x7A,	// USAGE (Op Effect Start Solo)
					0x09,0x7B,	// USAGE (Op Effect Stop)
					0x15,0x01,	// LOGICAL_MINIMUM (01)
					0x25,0x03,	// LOGICAL_MAXIMUM (03)
					0x75,0x08,	// REPORT_SIZE (08)
					0x95,0x01,	// REPORT_COUNT (01)
					0x91,0x00,	// OUTPUT (Data,Ary,Abs)
				0xC0,	// END COLLECTION ()
				0x09,0x7C,	// USAGE (Loop Count)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x00,	// PHYSICAL_MAXIMUM (00 FF)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x09,0x90,	// USAGE (PID Block Free Report)
				0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x0B,	// REPORT_ID (0B)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x09,0x96,	// USAGE (PID Device Control)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x0C,	// REPORT_ID (0C)
				0x09,0x97,	// USAGE (DC Enable Actuators)
				0x09,0x98,	// USAGE (DC Disable Actuators)
				0x09,0x99,	// USAGE (DC Stop All Effects)
				0x09,0x9A,	// USAGE (DC Device Reset)
				0x09,0x9B,	// USAGE (DC Device Pause)
				0x09,0x9C,	// USAGE (DC Device Continue)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x06,	// LOGICAL_MAXIMUM (06)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x00,	// OUTPUT (Data)
			0xC0,	// END COLLECTION ()

			0x09,0x7D,	// USAGE (Device Gain Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x0D,	// REPORT_ID (0D)
				0x09,0x7E,	// USAGE (Device Gain)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0x10,0x27,	// PHYSICAL_MAXIMUM (10000)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
			0xC0,	// END COLLECTION ()

			0x09,0x6B,	// USAGE (Set Custom Force Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x0E,	// REPORT_ID (0E)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x6D,	// USAGE (Sample Count)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x00,	// PHYSICAL_MAXIMUM (00 FF)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x09,0x51,	// USAGE (Sample Period)
				0x66,0x03,0x10,	// UNIT (Eng Lin:Time)
				0x55,0xFD,	// UNIT_EXPONENT (-3)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x7F,	// LOGICAL_MAXIMUM (32767)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x7F,	// PHYSICAL_MAXIMUM (32767)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x01,	// REPORT_COUNT (01)
				0x91,0x02,	// OUTPUT (Data,Var,Abs)
				0x55,0x00,	// UNIT_EXPONENT (00)
				0x66,0x00,0x00,	// UNIT (None)
			0xC0,	// END COLLECTION ()

			0x09,0xAB,	// USAGE (Create New Effect Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x05,	// REPORT_ID (05)
				0x09,0x25,	// USAGE (Effect Type)
				0xA1,0x02,	// COLLECTION (Logical)
					0x09,0x26,	// USAGE (26)
					0x09,0x27,	// USAGE (27)
					0x09,0x30,	// USAGE (30)
					0x09,0x31,	// USAGE (31)
					0x09,0x32,	// USAGE (32)
					0x09,0x33,	// USAGE (33)
					0x09,0x34,	// USAGE (34)
					0x09,0x40,	// USAGE (40)
					0x09,0x41,	// USAGE (41)
					0x09,0x42,	// USAGE (42)
					0x09,0x43,	// USAGE (43)
					0x09,0x28,	// USAGE (28)
					0x25,0x0C,	// LOGICAL_MAXIMUM (0C)
					0x15,0x01,	// LOGICAL_MINIMUM (01)
					0x35,0x01,	// PHYSICAL_MINIMUM (01)
					0x45,0x0C,	// PHYSICAL_MAXIMUM (0C)
					0x75,0x08,	// REPORT_SIZE (08)
					0x95,0x01,	// REPORT_COUNT (01)
					0xB1,0x00,	// FEATURE (Data)
				0xC0,	// END COLLECTION ()
				0x05,0x01,	// USAGE_PAGE (Generic Desktop)
				0x09,0x3B,	// USAGE (Byte Count)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x26,0xFF,0x01,	// LOGICAL_MAXIMUM (511)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x46,0xFF,0x01,	// PHYSICAL_MAXIMUM (511)
				0x75,0x0A,	// REPORT_SIZE (0A)
				0x95,0x01,	// REPORT_COUNT (01)
				0xB1,0x02,	// FEATURE (Data,Var,Abs)
				0x75,0x06,	// REPORT_SIZE (06)
				0xB1,0x01,	// FEATURE (Constant,Ary,Abs)
			0xC0,	// END COLLECTION ()

			0x05,0x0F,	// USAGE_PAGE (Physical Interface)
			0x09,0x89,	// USAGE (PID Block Load Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x06,	// REPORT_ID (06)
				0x09,0x22,	// USAGE (Effect Block Index)
				0x25,0x28,	// LOGICAL_MAXIMUM (28)
				0x15,0x01,	// LOGICAL_MINIMUM (01)
				0x35,0x01,	// PHYSICAL_MINIMUM (01)
				0x45,0x28,	// PHYSICAL_MAXIMUM (28)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0xB1,0x02,	// FEATURE (Data,Var,Abs)
				0x09,0x8B,	// USAGE (Block Load Status)
				0xA1,0x02,	// COLLECTION (Logical)
					0x09,0x8C,	// USAGE (Block Load Success)
					0x09,0x8D,	// USAGE (Block Load Full)
					0x09,0x8E,	// USAGE (Block Load Error)
					0x25,0x03,	// LOGICAL_MAXIMUM (03)
					0x15,0x01,	// LOGICAL_MINIMUM (01)
					0x35,0x01,	// PHYSICAL_MINIMUM (01)
					0x45,0x03,	// PHYSICAL_MAXIMUM (03)
					0x75,0x08,	// REPORT_SIZE (08)
					0x95,0x01,	// REPORT_COUNT (01)
					0xB1,0x00,	// FEATURE (Data)
				0xC0,	// END COLLECTION ()
				0x09,0xAC,	// USAGE (RAM Pool Available)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x27,0xFF,0xFF,0x00,0x00,	// LOGICAL_MAXIMUM (00 00 FF FF)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x47,0xFF,0xFF,0x00,0x00,	// PHYSICAL_MAXIMUM (00 00 FF FF)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x01,	// REPORT_COUNT (01)
				0xB1,0x00,	// FEATURE (Data)
			0xC0,	// END COLLECTION ()

			0x09,0x7F,	// USAGE (PID Pool Report)
			0xA1,0x02,	// COLLECTION (Logical)
				0x85,0x07,	// REPORT_ID (07)
				0x09,0x80,	// USAGE (RAM Pool Size)
				0x75,0x10,	// REPORT_SIZE (10)
				0x95,0x01,	// REPORT_COUNT (01)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x27,0xFF,0xFF,0x00,0x00,	// LOGICAL_MAXIMUM (00 00 FF FF)
				0x47,0xFF,0xFF,0x00,0x00,	// PHYSICAL_MAXIMUM (00 00 FF FF)
				0xB1,0x02,	// FEATURE (Data,Var,Abs)
				0x09,0x83,	// USAGE (Simultaneous Effects Max)
				0x26,0xFF,0x00,	// LOGICAL_MAXIMUM (00 FF)
				0x46,0xFF,0x00,	// PHYSICAL_MAXIMUM (00 FF)
				0x75,0x08,	// REPORT_SIZE (08)
				0x95,0x01,	// REPORT_COUNT (01)
				0xB1,0x02,	// FEATURE (Data,Var,Abs)
				0x09,0xA9,	// USAGE (Device Managed Pool)
				0x09,0xAA,	// USAGE (Shared Parameter Blocks)
				0x75,0x01,	// REPORT_SIZE (01)
				0x95,0x02,	// REPORT_COUNT (02)
				0x15,0x00,	// LOGICAL_MINIMUM (00)
				0x25,0x01,	// LOGICAL_MAXIMUM (01)
				0x35,0x00,	// PHYSICAL_MINIMUM (00)
				0x45,0x01,	// PHYSICAL_MAXIMUM (01)
				0xB1,0x02,	// FEATURE (Data,Var,Abs)
				0x75,0x06,	// REPORT_SIZE (06)
				0x95,0x01,	// REPORT_COUNT (01)
				0xB1,0x03,	// FEATURE ( Cnst,Var,Abs)
			0xC0,	// END COLLECTION ()
		0xc0, // END_COLLECTION
};
#endif
/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */
/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @{
  */
static int8_t CUSTOM_HID_Init_FS     (void);
static int8_t CUSTOM_HID_DeInit_FS   (void);
static int8_t CUSTOM_HID_OutEvent_FS (uint8_t* eventpointer);
 

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS = 
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CUSTOM_HID_Init_FS
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{ 
  /* USER CODE BEGIN 4 */ 
  return (0);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CUSTOM_HID_DeInit_FS
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */ 
  return (0);
  /* USER CODE END 5 */ 
}

/**
  * @brief  CUSTOM_HID_OutEvent_FS
  *         Manage the CUSTOM HID class events       
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS  (uint8_t* eventpointer)
{ 
  /* USER CODE BEGIN 6 */
	joystick.EPINT_OUT_callback(eventpointer);
  return (0);
  /* USER CODE END 6 */ 
}

/* USER CODE BEGIN 7 */ 
/**
  * @brief  USBD_CUSTOM_HID_SendReport_FS
  *         Send the report to the Host       
  * @param  report: the report to be sent
  * @param  len: the report length
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CUSTOM_HID_SendReport_FS ( uint8_t *report,uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len); 
}
/* USER CODE END 7 */ 

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
