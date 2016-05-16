#include "mbed.h"
#include "USBHID.h"
#include "cFFBDevice.h"

DigitalOut led4(PD_15);
DigitalOut led5(PD_14);
DigitalOut led6(PD_13);
 
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx.h"

Serial SMSerial(PB_10, PB_11); // tx, rx
DigitalOut SMSerialTXEN(PD_8);
bool SMSerialMasterIsMe=true;

cFFBDevice gFFBDevice;

#ifdef __cplusplus
extern "C" {
#endif

//Init encoder input
//credit to David Lowe from https://developer.mbed.org/forum/platform-34-ST-Nucleo-F401RE-community/topic/4963/?page=1#comment-26870
void EncoderInitialize()
{
    // configure GPIO PA0 & PA1 aka A0 & A1 as inputs for Encoder
    // Enable clock for GPIOA
    __GPIOA_CLK_ENABLE(); //equivalent from hal_rcc.h

    //stm32f4xx.h
    GPIOA->MODER   |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;           //PA0 & PA1 as Alternate Function   /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOA->OTYPER  |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1 ;                 //PA0 & PA1 as Inputs               /*!< GPIO port output type register,        Address offset: 0x04      */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 ;     //Low speed                         /*!< GPIO port output speed register,       Address offset: 0x08      */
    GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 ;           //Pull Down                         /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOA->AFR[0]  |= 0x00000011 ;                                          //AF01 for PA0 & PA1                /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOA->AFR[1]  |= 0x00000000 ;                                          //nibbles here refer to gpio8..15   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */

    // configure TIM2 as Encoder input
    // Enable clock for TIM2
    __TIM2_CLK_ENABLE();

    TIM2->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    TIM2->SMCR  = TIM_ENCODERMODE_TI12;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM2->CCMR1 = 0x8181;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1. 0x_1_1 blanks are input filter strength 0-F
    TIM2->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM2->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM2->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM2->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register

    TIM2->CNT = 0x0000;  //reset the counter before we use it
}

int EncoderRead()
{
	return TIM2->CNT;
}

int SMPortWrite(const char *data, int len)
{
	int i;

	//if we are not in control of SM bus, return
	if(SMSerialMasterIsMe==false)
		return 0;

	//write
	SMSerialTXEN=1;
	for(i=0;i<len;i++)
		SMSerial.putc(data[i]);
	wait(43e-6);//keep TXEN up for last 2 bytes because putc returns before data is physically out
	SMSerialTXEN=0;

	return len;
}

int SMPortReadByte( char *byte )
{
	Timer timeout;
	timeout.start();
	timeout.reset();

	//if we are not in control of SM bus, return
	if(SMSerialMasterIsMe==false)
		return 0;

	//try reading a byte
//	bool done=false;
	do
	{
		if(SMSerial.readable())
		{
			*byte=SMSerial.getc();
			return 1;
		}
	} while(timeout.read()<0.2);//loop until timeout or data received

	//timeouted
	return 0;
}

#ifdef __cplusplus
}
#endif

//takes control of SM bus for IONI if parameter is true, if false, lets second USB port (dedicated Granity port) to control it
//in practice it turns TX & TXEN to high impedance state (inputs) when me=false and outputs when true
void SMPortSetMaster(bool me)
{
	GPIO_InitTypeDef pin;

	//TX
	pin.Pin=GPIO_PIN_10;
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


#include "USBGameController.h"
USBGameController joystick;

AnalogIn ADCUpperPin3(PC_5);
AnalogIn ADCUpperPin2(PB_0);
AnalogIn ADCUpperPin1(PB_1);

Serial pc(PA_9, PA_10); // tx, rx
#include "simplemotion.h"

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

//	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

//	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
//	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

enum SystemStatus {BeforeInit, DriveInit, DriveConnectionError, DriveWaitClearfaults, DriveWaitReady, DriveFWUnsupported, Operational };
volatile SystemStatus currentSystemStatus=BeforeInit;

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
			wait(0.1);
		}
	}
}

Ticker ledBlinker;
void controlLeds()//timer, called at every 0.5secs
{
	switch(currentSystemStatus)//blink some leds
	{
		case DriveConnectionError:
			led4=!led4;
			break;
		case DriveWaitClearfaults:
			led5=!led5;
			break;
		case DriveWaitReady:
			led6=!led6;
			break;
		case DriveFWUnsupported:
			led4=!led4;
			led5=!led5;
			break;
		case BeforeInit:
			led4=!led4;
			led5=!led5;
			led6=!led6;
			break;
		case Operational:
			led4=led5=0;
			led6=1;
			break;
		default:
			led4=led5=led6=1;
			break;
	}
}


bool InitializeDrive()
{
	gFFBDevice.mSMBusHandle = smOpenBus("MBEDSERIAL");
	SMSerial.baud(460800);

	broadcastSystemStatus(DriveInit);

	//read some ioni drive parameters
	smint32 driveStatus, initialPosition, homingConfigurationBits, driveFWversion;
	smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);
	smRead1Parameter(gFFBDevice.mSMBusHandle, 1, SMP_FIRMWARE_VERSION, &driveFWversion);

	led4=led5=led6=0;

	//comm error
	if(getCumulativeStatus(gFFBDevice.mSMBusHandle)!=SM_OK)
		broadcastSystemStatus(DriveConnectionError,true);

	if(driveFWversion<1093)//V1092 would be enough, but it has bug in SMP_FAULT_BEHAVIOR which is fixed in the next version
		broadcastSystemStatus(DriveFWUnsupported, true);

	//disable enable drive watchdog: if communication is lost for over 1sec or has error, drive will go fault state
	smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_FAULT_BEHAVIOR, 1|(100<<8));

	//if drive is in fault state, wait that user resets the faults with STO input
	while(driveStatus&STAT_FAULTSTOP )
	{
		broadcastSystemStatus(DriveWaitClearfaults);
		smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);
		wait(0.5);
	}

	smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);
	//wait drive to initialize (wait for phasing & homing if configured)
	while(!(driveStatus&STAT_SERVO_READY) )
	{
			broadcastSystemStatus(DriveWaitReady);
			smRead3Parameters(gFFBDevice.mSMBusHandle, 1, SMP_STATUS, &driveStatus, SMP_ACTUAL_POSITION_FB,&initialPosition,SMP_TRAJ_PLANNER_HOMING_BITS,&homingConfigurationBits);
			wait(0.5);
	}

	//read position counter
	volatile int p1,p2;
	smint32 positionFB=0;
	p1=SetTorque(0);//call this twice to have 16 bit differential encoder unwrapper initialized
	p2=SetTorque(0);
	smRead1Parameter(gFFBDevice.mSMBusHandle, 1, SMP_ACTUAL_POSITION_FB, &positionFB);//read 32 bit position
	resetPositionCountAt(positionFB);

	//enable drive watchdog: if communication is lost for over 0.3sec or has error, drive will go fault state
	smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_FAULT_BEHAVIOR, 1 | (30<<8));

	broadcastSystemStatus(Operational);


	return true;
}

int main()
{
	HAL_Init();
	SystemClock_Config();

	ledBlinker.attach(&controlLeds,0.5);

	InitializeDrive();

	int32_t i = 0;

    pc.baud(230400);
    pc.printf("Hello World!\n\r");

    EncoderInitialize();
	InitializeTorqueCommand();
    while (1) 
    {
	    //s32 x = EncoderRead();//direct read of quadrature encoder
    	s32 encoderCounter;
	    gFFBDevice.CalcTorqueCommand(&encoderCounter);//reads encoder counter too
	    
	    i += 32;
		uint16_t throttle = i & 0xffff;
	    uint16_t rudder = (i + 10000) & 0xffff;
	    uint16_t clutch = (i + 20000) & 0xffff;
	    uint16_t brake = (i + 30000) & 0xffff;
	    uint32_t button = i;
	    int8_t hat    = (i >> 8) & 0x07;
	    uint16_t y = (i + 40000) & 0xffff;
	    encoderCounter = constrain(encoderCounter + 0x7fff, X_AXIS_LOG_MIN, X_AXIS_LOG_MAX);
        joystick.update(brake, clutch,throttle,rudder, encoderCounter, y, button, hat);

        if(joystick.getPendingReceivedReportCount())
        {
        	HID_REPORT recv_report=joystick.getReceivedReport();
	       	joystick.handleReceivedHIDReport(recv_report);
        }
	    wait(0.001*CONTROL_PERIOD_MS);
    }
}
