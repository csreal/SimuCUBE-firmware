/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include <main.h>
#include "stm32f4xx_hal.h"


/* USER CODE BEGIN Includes */
//#include "stdio.h"
bool debugMode = true;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void delay_us(uint32_t delay_us) {
	uint32_t counter = 0;
	counter = (delay_us * (SystemCoreClock / 1000000U));
	while(counter != 0U)
	{
		counter--;
	}
}


/* USER CODE END PV */


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
#ifdef __cplusplus
extern "C" void Error_Handler(void);
#endif
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);






/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


#include "usb_device.h"
#include "usbgamecontroller.h"

#include "cFFBDevice.h"

#include "Command.h"

cFFBDevice gFFBDevice;
USBGameController joystick;

enum SystemStatus {BeforeInit, DriveInit, DriveConnectionError, DriveWaitClearfaults, DriveWaitReady, DriveFWUnsupported, Operational };
volatile SystemStatus currentSystemStatus=BeforeInit;


extern "C" {
uint8_t gamecontroller_callback_wrapper(uint8_t *report) {
	return joystick.EPINT_OUT_callback(report);
}
}
/* USER CODE END 0 */


bool SMSerialMasterIsMe=true;



void SMPortSetMaster(bool me)
{
	GPIO_InitTypeDef pin;

	//TX
	pin.Pin = GPIO_PIN_10;
	pin.Speed=GPIO_SPEED_FREQ_LOW;
	pin.Pull=GPIO_NOPULL;
	pin.Alternate=7;//USART3
	if(me)
		pin.Mode=GPIO_MODE_AF_PP;
	else
		pin.Mode=GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB,&pin);

	//TXEN
	pin.Pin=GPIO_PIN_8;
	if(me)
		pin.Mode=GPIO_MODE_OUTPUT_PP;
	else
		pin.Mode=GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOD,&pin);

	SMSerialMasterIsMe=me;
}




#ifdef __cplusplus
extern "C" {
#endif


int SMPortWrite(const char *data, int len)
{
	//int i;

	//if we are not in control of SM bus, return
	if(SMSerialMasterIsMe==false)
		return 0;

	//write
	//SMSerialTXEN=1;

	//#define RS485_TXEN_STM_Pin GPIO_PIN_8
	//#define RS485_TXEN_STM_GPIO_Port GPIOD

	HAL_GPIO_WritePin(RS485_TXEN_STM_GPIO_Port, RS485_TXEN_STM_Pin, GPIO_PIN_SET);  // sets pin high
	//HAL_GPIO_WritePin(RS485_TXEN_STM_GPIO_Port,RS485_TXEN_STM_Pin,GPIO_PIN_RESET) // sets pin low

	//for(i=0;i<len;i++)
	//	SMSerial.putc(data[i]);
	HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 0xFFFF);
	//wait(43e-6);//keep TXEN up for last 2 bytes because putc returns before data is physically out
	// not sure if HAL_transmit does it but lets wait anyway...
	delay_us(43);

	//SMSerialTXEN=0;
	HAL_GPIO_WritePin(RS485_TXEN_STM_GPIO_Port, RS485_TXEN_STM_Pin, GPIO_PIN_RESET); // sets pin low

	return len;
}

//extern smuint16 readTimeoutMs;


int SMPortReadByte( char *byte )
{
	// HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	/*
	Timer timeout;
	timeout.start();
	timeout.reset();
	*/

	//if we are not in control of SM bus, return
	if(SMSerialMasterIsMe==false) {
		return 0;
	}

	// try to read one byte with 200ms timout
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, (uint8_t *)byte, 1, 200);
	if(status != HAL_OK ) {
		return 0;	// something went wrong, or timeout.
	}
	return 1;

	//try reading a byte
//	bool done=false;

	/*
	do
	{
		if(SMSerial.readable())
		{
			*byte=SMSerial.getc();
			return 1;
		}
	} while(timeout.read()<0.2);//loop until timeout or data received
*/
	//timeouted
	//return 0;
}
#ifdef __cplusplus
}
#endif

//call this when system status changes. TODO implement some way of telling user about the status
void broadcastSystemStatus(SystemStatus status, bool stopHere=false)
{
	currentSystemStatus=status;

	//now on errors, just do dummy infinte loop with led blinking
	//FIXME replace with this some way to tell user thru USB too
	if(stopHere)
	{
		SMPortSetMaster(false);//release control of SM bus to allow Granity connection

		while(1)
		{
			HAL_Delay(100);//100ms delay
		}
	}
}


// search index pulse. Encoder is at 0 on startup.
bool WaitForIndexPulse( int &indexPos )
{
	smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_SYSTEM_CONTROL, 2048);

	bool found=false;
	do
	{
		smint32 enc, status;
		smRead2Parameters(gFFBDevice.mSMBusHandle, 1, SMP_DEBUGPARAM1, &enc, SMP_DEBUGPARAM2, &status);
		if(status==200)
		{
			found=true;
			indexPos=enc;
			return true;
		}
	} while(found==false);

	return false;
}

bool InitializeDrive()
{
	printf("1\r\n");
	gFFBDevice.mSMBusHandle = smOpenBus("MBEDSERIAL");
	//SMSerial.baud(460800);
	MX_USART3_UART_Init(); // inits to default 460800


	smSetTimeout(200);

	broadcastSystemStatus(DriveInit);

	//clear sm bus error from previous session. hope not need it
	smint32 dummy;
	smRead1Parameter(gFFBDevice.mSMBusHandle, 1, SMP_FIRMWARE_VERSION, &dummy);
	resetCumulativeStatus(gFFBDevice.mSMBusHandle);


	smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_FAULT_BEHAVIOR, 0);
	//read some ioni drive parameters
	smint32 driveStatus=-1, initialPosition=-1, homingConfigurationBits, driveFWversion=-1, encoderResolution=-1;
	smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_FIRMWARE_VERSION, &driveFWversion);
	smRead1Parameter(gFFBDevice.mSMBusHandle, 1, SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);

	//todo change these when cFFBDevice port is complete
	smRead1Parameter(gFFBDevice.mSMBusHandle, 1, SMP_ENCODER_PPR, &gFFBDevice.mConfig.hardwareConfig.mEncoderCPR);//read encoder resolution from ioni.
	gFFBDevice.mConfig.hardwareConfig.mEncoderCPR*=4;//PPR to CPR

	//switch leds off
	//defines work like this: #define LED4_OUT_Pin GPIO_PIN_12 #define LED4_OUT_GPIO_Port GPIOD
	HAL_GPIO_WritePin(LED1_CLIPPING_OUT_GPIO_Port, LED1_CLIPPING_OUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_OUT_GPIO_Port, LED2_OUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_OUT_GPIO_Port, LED3_OUT_Pin, GPIO_PIN_RESET);

	//led4=led5=led6=0;

	//comm error
	if(getCumulativeStatus(gFFBDevice.mSMBusHandle)!=SM_OK)
		broadcastSystemStatus(DriveConnectionError,true);
	if(driveFWversion<1100)//V1092 would be enough, but it has bug in SMP_FAULT_BEHAVIOR which is fixed in the next version
		broadcastSystemStatus(DriveFWUnsupported, true);

	smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_FAULTS,0);//reset prev communication fault & others

	//disable enable drive watchdog: if communication is lost for over 1sec or has error, drive will go fault state
	if(debugMode)
		smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_FAULT_BEHAVIOR, 1|(100<<8));

	//if drive is in fault state, wait that user resets the faults with STO input
	while(driveStatus&STAT_FAULTSTOP )
	{
		broadcastSystemStatus(DriveWaitClearfaults);
		smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);
		HAL_Delay(500);//wait(0.5);
	}

	smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);
	//wait drive to initialize (wait for phasing & homing if configured)
	while(!(driveStatus&STAT_SERVO_READY) )
	{
			broadcastSystemStatus(DriveWaitReady);
			smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);
			HAL_Delay(500);//wait(0.5);
	}


	// this is the code for centering after phasing.
	// for now, start the system when wheel is at center!
	/*
	int indexPos=0;
	if(!WaitForIndexPulse(indexPos)) {
		broadcastSystemStatus(IndexPointNotFound, false);
	}
	*/

	//read position counter
	volatile int p1,p2;
	smint32 positionFB=0;
	p1=SetTorque(0);//call this twice to have 16 bit differential encoder unwrapper initialized
	p2=SetTorque(0);
	smRead1Parameter(gFFBDevice.mSMBusHandle, 1, SMP_ACTUAL_POSITION_FB, &positionFB);//read 32 bit position

	// TODO at this time, the steering wheel should be driven too center
	// defined by encoderOffsetValue variable
	// and only after that resetPositionCountAt should be called.
	resetPositionCountAt(positionFB);

	//enable drive watchdog: if communication is lost for over 0.3sec or has error, drive will go fault state
	smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_FAULT_BEHAVIOR, 1 | (30<<8));

	//smSetTimeout(50);
	//smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_BUS_SPEED,1000000);
	//smSetTimeout(200);
	//SMSerial.baud(1000000);


	broadcastSystemStatus(Operational, false);


	return true;
}



int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  if(!InitializeDrive()) {
	  //handle error here.
  }
  broadcastSystemStatus(Operational, false);



  // unsigned 16-bit for these, as joystick API needs it.
  // calculated internally with more accuracy when reading/scaling.
  uint16_t throttle = 0;
  uint16_t rudder = 0;
  uint16_t clutch = 0;
  uint16_t brake = 0;
  int8_t hat = 0;
  uint16_t y = 0;
  uint32_t buttons = 0;

  float steeringAngle = 0.0;

  // init min and max steering angles based on user profile.
  float minSteeringAngle = (-1)*gFFBDevice.mConfig.profileConfig.mMaxAngle/2;
  float maxSteeringAngle = gFFBDevice.mConfig.profileConfig.mMaxAngle/2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
      s32 encoderCounter=0;
      gFFBDevice.CalcTorqueCommand(&encoderCounter); //reads encoder counter too
	    steeringAngle = (float)encoderCounter/(float)gFFBDevice.mConfig.hardwareConfig.mEncoderCPR*360.0;
	    if(steeringAngle < minSteeringAngle) {
	    	steeringAngle = minSteeringAngle;
	    	// todo: endstop effect here!
	    } else if (steeringAngle > maxSteeringAngle ) {
	    	steeringAngle = maxSteeringAngle;
	    	// todo: endstop effect here!
	    }


      //directly to int
      int16_t steering = (int16_t)(steeringAngle/maxSteeringAngle*32768.0);// + 0.5);
      if(debugMode)printf("raw encoder: %d ", encoderCounter);
      if(debugMode)printf("direct scaled angle: %d", steering);
      uint16_t steeringScaled = 0x7fff - steering;
      if(debugMode)printf("raw joyval with offset: %u ", steeringScaled);


      joystick.update(brake, clutch,throttle,rudder, steeringScaled, y, buttons, hat);

      if(debugMode)printf("angle %f \r\n", steeringAngle);

      if(joystick.getPendingReceivedReportCount())
      {
      	HID_REPORT recv_report=joystick.getReceivedReport();
	       	joystick.handleReceivedHIDReport(recv_report);
      }
      // HAL_Delay = milliseconds
      HAL_Delay(1);//wait(0.001*CONTROL_PERIOD_MS);

      //wait(0.001);


  /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */

}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;//115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800; // SimpleMotion bus default
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : X12_LOWER_1_Pin X12_LOWER_2_Pin X12_LOWER_4_Pin X12_UPPER_6_Pin 
                           STM_PE6_Pin STM_PE7_Pin STM_PE8_Pin STM_PE10_Pin 
                           STM_PE13_Pin STM_PE14_Pin STM_PE15_Pin X15_3_Pin 
                           X12_LOWER_3_Pin */
  GPIO_InitStruct.Pin = X12_LOWER_1_Pin|X12_LOWER_2_Pin|X12_LOWER_4_Pin|X12_UPPER_6_Pin 
                          |STM_PE6_Pin|STM_PE7_Pin|STM_PE8_Pin|STM_PE10_Pin 
                          |STM_PE13_Pin|STM_PE14_Pin|STM_PE15_Pin|X15_3_Pin 
                          |X12_LOWER_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : X12_LOWER_6_Pin X12_LOWER_7_Pin X11_LOWER_1_Pin X12_UPPER_2_Pin 
                           X12_UPPER_3_Pin X12_UPPER_4_Pin X12_UPPER_5_Pin X12_UPPER_7_Pin 
                           X12_UPPER_1_Pin X12_LOWER_5_Pin */
  GPIO_InitStruct.Pin = X12_LOWER_6_Pin|X12_LOWER_7_Pin|X11_LOWER_1_Pin|X12_UPPER_2_Pin 
                          |X12_UPPER_3_Pin|X12_UPPER_4_Pin|X12_UPPER_5_Pin|X12_UPPER_7_Pin 
                          |X12_UPPER_1_Pin|X12_LOWER_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_ENC_IN_A_Pin STM_ENC_IN_B_Pin STM_ENC_IN_C_Pin PA8 */
  GPIO_InitStruct.Pin = STM_ENC_IN_A_Pin|STM_ENC_IN_B_Pin|STM_ENC_IN_C_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GP01_STM_Pin PB12 PB13 PB14 
                           PB15 DIPSW_2_Pin STM_PB9_Pin */
  GPIO_InitStruct.Pin = GP01_STM_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|DIPSW_2_Pin|STM_PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_IONI_PWM_OUT_HSIN2_Pin STM_IONI_DIR_OUT_HSIN1_Pin HX711_CLKOUT_Pin */
  GPIO_InitStruct.Pin = STM_IONI_PWM_OUT_HSIN2_Pin|STM_IONI_DIR_OUT_HSIN1_Pin|HX711_CLKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_TXEN_STM_Pin LED4_OUT_Pin LED3_OUT_Pin LED2_OUT_Pin 
                           LED1_CLIPPING_OUT_Pin */
  GPIO_InitStruct.Pin = RS485_TXEN_STM_Pin|LED4_OUT_Pin|LED3_OUT_Pin|LED2_OUT_Pin 
                          |LED1_CLIPPING_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STM_PD9_Pin STM_PD10_Pin STM_PD11_Pin IDSEL_0_Pin 
                           IDSEL_1_Pin IDSEL_2_Pin FTDI_USB_SLEEP_IN_Pin USB_GRANITY_VCC_SENSE_Pin 
                           USB_HID_VCC_SENSE_Pin STM_PD6_Pin STM_PD7_Pin */
  GPIO_InitStruct.Pin = STM_PD9_Pin|STM_PD10_Pin|STM_PD11_Pin|IDSEL_0_Pin 
                          |IDSEL_1_Pin|IDSEL_2_Pin|FTDI_USB_SLEEP_IN_Pin|USB_GRANITY_VCC_SENSE_Pin 
                          |USB_HID_VCC_SENSE_Pin|STM_PD6_Pin|STM_PD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GPI4_CLEAR_FAULTS_Pin PB5 */
  GPIO_InitStruct.Pin = GPI4_CLEAR_FAULTS_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STM_IONI_PWM_OUT_HSIN2_Pin|STM_IONI_DIR_OUT_HSIN1_Pin|HX711_CLKOUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RS485_TXEN_STM_Pin|LED4_OUT_Pin|LED3_OUT_Pin|LED2_OUT_Pin 
                          |LED1_CLIPPING_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPI4_CLEAR_FAULTS_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}


#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
