/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define X12_LOWER_1_Pin GPIO_PIN_2
#define X12_LOWER_1_GPIO_Port GPIOE
#define X12_LOWER_2_Pin GPIO_PIN_3
#define X12_LOWER_2_GPIO_Port GPIOE
#define X12_LOWER_4_Pin GPIO_PIN_4
#define X12_LOWER_4_GPIO_Port GPIOE
#define X12_UPPER_6_Pin GPIO_PIN_5
#define X12_UPPER_6_GPIO_Port GPIOE
#define STM_PE6_Pin GPIO_PIN_6
#define STM_PE6_GPIO_Port GPIOE
#define X12_LOWER_6_Pin GPIO_PIN_13
#define X12_LOWER_6_GPIO_Port GPIOC
#define X12_LOWER_7_Pin GPIO_PIN_14
#define X12_LOWER_7_GPIO_Port GPIOC
#define X11_LOWER_1_Pin GPIO_PIN_15
#define X11_LOWER_1_GPIO_Port GPIOC
#define EXT_POT_IN_3_Pin GPIO_PIN_0
#define EXT_POT_IN_3_GPIO_Port GPIOC
#define EXT_POT_IN_2_Pin GPIO_PIN_1
#define EXT_POT_IN_2_GPIO_Port GPIOC
#define EXT_POT_IN_1_Pin GPIO_PIN_2
#define EXT_POT_IN_1_GPIO_Port GPIOC
#define X11_LOWER_2_Pin GPIO_PIN_3
#define X11_LOWER_2_GPIO_Port GPIOC
#define STM_ENC_IN_A_Pin GPIO_PIN_0
#define STM_ENC_IN_A_GPIO_Port GPIOA
#define STM_ENC_IN_B_Pin GPIO_PIN_1
#define STM_ENC_IN_B_GPIO_Port GPIOA
#define STM_ENC_IN_C_Pin GPIO_PIN_2
#define STM_ENC_IN_C_GPIO_Port GPIOA
#define X11_LOWER_3_Pin GPIO_PIN_3
#define X11_LOWER_3_GPIO_Port GPIOA
#define X11_LOWER_5_Pin GPIO_PIN_4
#define X11_LOWER_5_GPIO_Port GPIOA
#define X11_LOWER_6_Pin GPIO_PIN_5
#define X11_LOWER_6_GPIO_Port GPIOA
#define X11_LOWER_7_Pin GPIO_PIN_6
#define X11_LOWER_7_GPIO_Port GPIOA
#define X11_UPPER_6_Pin GPIO_PIN_7
#define X11_UPPER_6_GPIO_Port GPIOA
#define X11_UPPER_5_Pin GPIO_PIN_4
#define X11_UPPER_5_GPIO_Port GPIOC
#define X11_UPPER_3_Pin GPIO_PIN_5
#define X11_UPPER_3_GPIO_Port GPIOC
#define X11_UPPER_2_Pin GPIO_PIN_0
#define X11_UPPER_2_GPIO_Port GPIOB
#define X11_UPPER_1_Pin GPIO_PIN_1
#define X11_UPPER_1_GPIO_Port GPIOB
#define GP01_STM_Pin GPIO_PIN_2
#define GP01_STM_GPIO_Port GPIOB
#define STM_PE7_Pin GPIO_PIN_7
#define STM_PE7_GPIO_Port GPIOE
#define STM_PE8_Pin GPIO_PIN_8
#define STM_PE8_GPIO_Port GPIOE
#define STM_IONI_PWM_OUT_HSIN2_Pin GPIO_PIN_9
#define STM_IONI_PWM_OUT_HSIN2_GPIO_Port GPIOE
#define STM_PE10_Pin GPIO_PIN_10
#define STM_PE10_GPIO_Port GPIOE
#define STM_IONI_DIR_OUT_HSIN1_Pin GPIO_PIN_11
#define STM_IONI_DIR_OUT_HSIN1_GPIO_Port GPIOE
#define HX711_CLKOUT_Pin GPIO_PIN_12
#define HX711_CLKOUT_GPIO_Port GPIOE
#define STM_PE13_Pin GPIO_PIN_13
#define STM_PE13_GPIO_Port GPIOE
#define STM_PE14_Pin GPIO_PIN_14
#define STM_PE14_GPIO_Port GPIOE
#define STM_PE15_Pin GPIO_PIN_15
#define STM_PE15_GPIO_Port GPIOE
#define RS485_TXEN_STM_Pin GPIO_PIN_8
#define RS485_TXEN_STM_GPIO_Port GPIOD
#define STM_PD9_Pin GPIO_PIN_9
#define STM_PD9_GPIO_Port GPIOD
#define STM_PD10_Pin GPIO_PIN_10
#define STM_PD10_GPIO_Port GPIOD
#define STM_PD11_Pin GPIO_PIN_11
#define STM_PD11_GPIO_Port GPIOD
#define LED4_OUT_Pin GPIO_PIN_12
#define LED4_OUT_GPIO_Port GPIOD
#define LED3_OUT_Pin GPIO_PIN_13
#define LED3_OUT_GPIO_Port GPIOD
#define LED2_OUT_Pin GPIO_PIN_14
#define LED2_OUT_GPIO_Port GPIOD
#define LED1_CLIPPING_OUT_Pin GPIO_PIN_15
#define LED1_CLIPPING_OUT_GPIO_Port GPIOD
#define X12_UPPER_2_Pin GPIO_PIN_6
#define X12_UPPER_2_GPIO_Port GPIOC
#define X12_UPPER_3_Pin GPIO_PIN_7
#define X12_UPPER_3_GPIO_Port GPIOC
#define X12_UPPER_4_Pin GPIO_PIN_8
#define X12_UPPER_4_GPIO_Port GPIOC
#define X12_UPPER_5_Pin GPIO_PIN_9
#define X12_UPPER_5_GPIO_Port GPIOC
#define X12_UPPER_7_Pin GPIO_PIN_10
#define X12_UPPER_7_GPIO_Port GPIOC
#define X12_UPPER_1_Pin GPIO_PIN_11
#define X12_UPPER_1_GPIO_Port GPIOC
#define X12_LOWER_5_Pin GPIO_PIN_12
#define X12_LOWER_5_GPIO_Port GPIOC
#define IDSEL_0_Pin GPIO_PIN_0
#define IDSEL_0_GPIO_Port GPIOD
#define IDSEL_1_Pin GPIO_PIN_1
#define IDSEL_1_GPIO_Port GPIOD
#define IDSEL_2_Pin GPIO_PIN_2
#define IDSEL_2_GPIO_Port GPIOD
#define FTDI_USB_SLEEP_IN_Pin GPIO_PIN_3
#define FTDI_USB_SLEEP_IN_GPIO_Port GPIOD
#define USB_GRANITY_VCC_SENSE_Pin GPIO_PIN_4
#define USB_GRANITY_VCC_SENSE_GPIO_Port GPIOD
#define USB_HID_VCC_SENSE_Pin GPIO_PIN_5
#define USB_HID_VCC_SENSE_GPIO_Port GPIOD
#define STM_PD6_Pin GPIO_PIN_6
#define STM_PD6_GPIO_Port GPIOD
#define STM_PD7_Pin GPIO_PIN_7
#define STM_PD7_GPIO_Port GPIOD
#define GPI4_CLEAR_FAULTS_Pin GPIO_PIN_4
#define GPI4_CLEAR_FAULTS_GPIO_Port GPIOB
#define DIPSW_2_Pin GPIO_PIN_8
#define DIPSW_2_GPIO_Port GPIOB
#define STM_PB9_Pin GPIO_PIN_9
#define STM_PB9_GPIO_Port GPIOB
#define X15_3_Pin GPIO_PIN_0
#define X15_3_GPIO_Port GPIOE
#define X12_LOWER_3_Pin GPIO_PIN_1
#define X12_LOWER_3_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
