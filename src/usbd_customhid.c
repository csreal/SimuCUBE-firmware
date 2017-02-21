/**
  ******************************************************************************
  * @file    usbd_customhid.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the CUSTOM_HID core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                CUSTOM_HID Class  Description
  *          =================================================================== 
  *           This module manages the CUSTOM_HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (CUSTOM_HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - Usage Page : Generic Desktop
  *             - Usage : Vendor
  *             - Collection : Application 
  *      
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
//#include <ffbengine.h>
#include "usbd_customhid.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "answer_filetypes.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CUSTOM_HID 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

#if 0
void FfbOnCreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData)
{
	outData->reportId = 6;
	outData->effectBlockIndex = 1;// GetNextFreeEffect();

	if (outData->effectBlockIndex == 0)
	{
		outData->loadStatus = 2;	// 1=Success,2=Full,3=Error
		//LogText("Could not create effect");
		printf("effectBlockIndex->full\r\n");
	}
	else
	{
		outData->loadStatus = 1;	// 1=Success,2=Full,3=Error
		printf("going to create new effect\r\n");
		CreateNewEffect(inData, outData->effectBlockIndex);

//		LogText("Created effect ");
//		LogBinary(&outData->effectBlockIndex,1);
//		LogText(", type ");
//		LogBinaryLf(&inData->effectType,1);
	}
	outData->ramPoolAvailable = 0xFFFF;	// =0 or 0xFFFF - don't really know what this is used for?
//	WaitMs(5);
}
#endif


/** @defgroup USBD_CUSTOM_HID_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_CUSTOM_HID_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 
/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_CUSTOM_HID_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_CUSTOM_HID_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_CUSTOM_HID_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_CUSTOM_HID_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_CUSTOM_HID_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_CUSTOM_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_CUSTOM_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_CUSTOM_HID_EP0_RxReady (USBD_HandleTypeDef  *pdev);
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Variables
  * @{
  */ 

USBD_ClassTypeDef  USBD_CUSTOM_HID = 
{
  USBD_CUSTOM_HID_Init,
  USBD_CUSTOM_HID_DeInit,
  USBD_CUSTOM_HID_Setup,
  NULL, /*EP0_TxSent*/  
  USBD_CUSTOM_HID_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
  USBD_CUSTOM_HID_DataIn, /*DataIn*/
  USBD_CUSTOM_HID_DataOut,
  NULL, /*SOF */
  NULL,
  NULL,      
  USBD_CUSTOM_HID_GetCfgDesc,
  USBD_CUSTOM_HID_GetCfgDesc, 
  USBD_CUSTOM_HID_GetCfgDesc,
  USBD_CUSTOM_HID_GetDeviceQualifierDesc,
};

#if 0
/* USB CUSTOM_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_CfgDesc[USB_CUSTOM_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_CUSTOM_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  
  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: CUSTOM_HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
  
  CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  0x20,          /*bInterval: Polling Interval (20 ms)*/
  /* 34 */
  
  0x07,	         /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,	/* bDescriptorType: */
  CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03,	/* bmAttributes: Interrupt endpoint */
  CUSTOM_HID_EPOUT_SIZE,	/* wMaxPacketSize: 2 Bytes max  */
  0x00,
  0x20,	/* bInterval: Polling Interval (20 ms) */
  /* 41 */
} ;
#else
/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_CfgDesc[USB_CUSTOM_HID_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  //USB_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  ((((1 * (0x09))  + (1 * (0x09)) + (1 * (0x09)) + (2 * (0x07))))&0xff),
  (((((1 * (0x09)) + (1 * (0x09)) + (1 * (0x09)) + (2 * (0x07))))&0xff00)>>8),
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  (1U<<7)|(1U<<6),
  //0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of Joystick interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of SIMUCUBE HID ********************/
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  LOBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE),
  HIBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE),/*wItemLength: Total length of Report descriptor*/

  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  //(((((3)))>>1) | (((((3))) & 1) ? 0x80:0)),     /*bEndpointAddress: Endpoint Address (IN)*/
  0x81,
  0x03,          /*bmAttributes: Interrupt endpoint*/
  ((((64)))&0xff), /*wMaxPacketSize: 4 Byte max */
  (((((64)))&0xff00)>>8),
  1,//HID_FS_BINTERVAL,          /*bInterval: Polling Interval (1 ms)*/

  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  //(((((2)))>>1) | (((((2))) & 1) ? 0x80:0)),     /*bEndpointAddress: Endpoint Address (IN)*/
  0x01,
  0x03,          /*bmAttributes: Interrupt endpoint*/
  ((((64)))&0xff), /*wMaxPacketSize: 4 Byte max */
  (((((64)))&0xff00)>>8),
  1         /*bInterval: Polling Interval (1 ms)*/
  /* 34 */
} ;
#endif

/* USB CUSTOM_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_Desc[USB_CUSTOM_HID_DESC_SIZ] __ALIGN_END =
{
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_CUSTOM_HID_Init
  *         Initialize the CUSTOM_HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CUSTOM_HID_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_CUSTOM_HID_HandleTypeDef     *hhid;
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                 CUSTOM_HID_EPIN_ADDR,
                 USBD_EP_TYPE_INTR,
                 CUSTOM_HID_EPIN_SIZE);  
  
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
                 CUSTOM_HID_EPOUT_ADDR,
                 USBD_EP_TYPE_INTR,
                 CUSTOM_HID_EPOUT_SIZE);
  
  pdev->pClassData = USBD_malloc(sizeof (USBD_CUSTOM_HID_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    hhid = (USBD_CUSTOM_HID_HandleTypeDef*) pdev->pClassData;
      
    hhid->state = CUSTOM_HID_IDLE;
    ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->Init();
          /* Prepare Out endpoint to receive 1st packet */ 
    USBD_LL_PrepareReceive(pdev, CUSTOM_HID_EPOUT_ADDR, hhid->Report_buf, 
                           USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
  }
    
  return ret;
}

/**
  * @brief  USBD_CUSTOM_HID_Init
  *         DeInitialize the CUSTOM_HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_CUSTOM_HID_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  /* Close CUSTOM_HID EP IN */
  USBD_LL_CloseEP(pdev,
                  CUSTOM_HID_EPIN_ADDR);
  
  /* Close CUSTOM_HID EP OUT */
  USBD_LL_CloseEP(pdev,
                  CUSTOM_HID_EPOUT_ADDR);
  
  /* FRee allocated memory */
  if(pdev->pClassData != NULL)
  {
    ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_Setup
  *         Handle the CUSTOM_HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */

volatile int temp_debug;
volatile int temp_debug2;

static uint8_t  USBD_CUSTOM_HID_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
	printf("customhid_setup\r\n");
  uint16_t len = 0;
  uint8_t  *pbuf = NULL;
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;
  int r = 0;
  uint8_t report_id;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :  
	  printf("req_type_class\r\n");
    switch (req->bRequest)
    {
      
      
    case CUSTOM_HID_REQ_SET_PROTOCOL:
      hhid->Protocol = (uint8_t)(req->wValue);
      printf("hid_req_set_protocol\r\n");
      break;
      
    case CUSTOM_HID_REQ_GET_PROTOCOL:
    	printf("hid_req_get_protocol\r\n");
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hhid->Protocol,
                        1);    
      break;
      
    case CUSTOM_HID_REQ_SET_IDLE:
    	printf("hid_req_set_idle\r\n");
      hhid->IdleState = (uint8_t)(req->wValue >> 8);
      break;
      
    case CUSTOM_HID_REQ_GET_IDLE:
    	printf("hid_req_get_idle\r\n");
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hhid->IdleState,
                        1);        
      break;      
    
    case CUSTOM_HID_REQ_SET_REPORT:
    	printf("hid_req_set_report\r\n");
      hhid->IsReportAvailable = 1;
      USBD_CtlPrepareRx (pdev, hhid->Report_buf, (uint8_t)(req->wLength));


      printf("going to createnewffect from hid_req_set_report!\r\n");

      USB_FFBReport_PIDBlockLoad_Feature_Data_t outdata;
	  //FfbOnCreateNewEffect((USB_FFBReport_CreateNewEffect_Feature_Data_t *)hhid->Report_buf, &mSetReportAnwser);
      FfbOnCreateNewEffect_wrapper();
#if 0
      //bruteforce
      mSetReportAnswer.reportId=6;
      mSetReportAnswer.effectBlockIndex=1;
      mSetReportAnswer.loadStatus=1;
      mSetReportAnswer.ramPoolAvailable=0xFFFF;
#endif
      
      break;

    // new, added case for CUSTOM_HID_REQ_GET_REPORT
    case CUSTOM_HID_REQ_GET_REPORT:
    	printf("hid_req_get_report\r\n");
    	r = req->bmRequest;
    	report_id = (req->wValue) & 0xff;
//#if 0
		if (report_id == 6)// && (gNewEffectBlockLoad.reportId==6))
		{
			/*
			here, the original code went to usbcallback_requestcompleted
			BEFORE triggering this.

			todo: do the same things here as the original. Call to
			ffb functions to set right mSetReportAnswer.
			 */

			printf("report_id=6\r\n");


			send_mSetReportAnswer_wrapper();
#if 0
			USBD_CtlSendData(pdev,(uint8_t *)&mSetReportAnswer,sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
			mSetReportAnswer.reportId = 0;
#endif
			return USBD_OK;
			/*
			USB_SendControl(0,(uint8_t *)&mSetReportAnswer,sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
			mSetReportAnswer.reportId = 0;
			return (true);
			*/
		}
		if (report_id == 7)
		{
			printf("report_id=7\r\n");
			send_mGetReportAnswer_wrapper(report_id);
#if 0
			mGetReportAnswer.reportId = report_id;
			mGetReportAnswer.ramPoolSize = 0xffff;
			mGetReportAnswer.maxSimultaneousEffects = MAX_EFFECTS;
			mGetReportAnswer.memoryManagement = 3;
			USBD_CtlSendData(pdev,(uint8_t *)&mGetReportAnswer,sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
#endif
			return USBD_OK;
			/*
			mSetReportAnswer.reportId = report_id;
			mSetReportAnswer.ramPoolSize = 0xffff;
			mSetReportAnswer.maxSimultaneousEffects = MAX_EFFECTS;
			mSetReportAnswer.memoryManagement = 3;
			USB_SendControl(0,(uint8_t *)&mSetReportAnswer,sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
			return (true);
			*/
		}
//#endif

    default:
    	printf("default_case\r\n");
    	temp_debug = req->bRequest;
      // this got triggered in the iR loading dialog before fullscreen. bRequest = 0x01;
      // in the middle of fullscreen loading iR crashes. bRequest  = 0x01;
      // in the old usb stack that was REQUEST_INTERFACE->HID_GET_REPORT.
      // in this new stack it is CUSTOM_HID_REQ_GET_REPORT
      //
      asm("nop");

      USBD_CtlError (pdev, req);
      return USBD_FAIL; 
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
	  printf("req_type_standard\r\n");
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
    	printf(" req_type_getdescriptor\r\n");
      if( req->wValue >> 8 == CUSTOM_HID_REPORT_DESC)
      {
    	  printf("custom_hid_report_desc!\r\n");
        len = MIN(USBD_CUSTOM_HID_REPORT_DESC_SIZE , req->wLength);
        pbuf =  ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->pReport;
      }
      else if( req->wValue >> 8 == CUSTOM_HID_DESCRIPTOR_TYPE)
      {
    	  printf("other report?\r\n");
        pbuf = USBD_CUSTOM_HID_Desc;   
        len = MIN(USB_CUSTOM_HID_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      
      break;
      
    case USB_REQ_GET_INTERFACE :
    	printf("req_get_interface?\r\n");
      USBD_CtlSendData (pdev,
                        (uint8_t *)&hhid->AltSetting,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
    	printf("req_set_interface?\r\n");
      hhid->AltSetting = (uint8_t)(req->wValue);
      break;
    }
    default:
    	printf("default case\r\n");
    	temp_debug = req->bRequest;
    	temp_debug2 = req->bmRequest;

    	// on startup req->bRequest shows 0x06 and req->bmRequest is 129  and wValue is 8704 which is STANDARD->GET_DESCRIPTOR in the old firmware


    	// and then it shows 0x01 which is..?



    	asm("nop\r\n");
    	break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_SendReport 
  *         Send CUSTOM_HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_CUSTOM_HID_SendReport     (USBD_HandleTypeDef  *pdev, 
                                 uint8_t *report,
                                 uint16_t len)
{
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;
  
  if (pdev->dev_state == USBD_STATE_CONFIGURED )
  {
    if(hhid->state == CUSTOM_HID_IDLE)
    {
      hhid->state = CUSTOM_HID_BUSY;
      USBD_LL_Transmit (pdev, 
                        CUSTOM_HID_EPIN_ADDR,                                      
                        report,
                        len);
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_CUSTOM_HID_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_CUSTOM_HID_CfgDesc);
  return USBD_CUSTOM_HID_CfgDesc;
}

/**
  * @brief  USBD_CUSTOM_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_CUSTOM_HID_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  //printf("datainstage");
  /* Ensure that the FIFO is empty before a new transfer, this condition could 
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassData)->state = CUSTOM_HID_IDLE;

  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_CUSTOM_HID_DataOut (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  //printf("dataoutstage\r\n");
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;  
  
  ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(&hhid->Report_buf[0]);
    
  USBD_LL_PrepareReceive(pdev, CUSTOM_HID_EPOUT_ADDR , hhid->Report_buf, 
                         USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

  return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CUSTOM_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;  
  printf("ep0_rxready\r\n");
  if (hhid->IsReportAvailable == 1)
  {
    ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf);
    hhid->IsReportAvailable = 0;      
  }

  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_CUSTOM_HID_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_CUSTOM_HID_DeviceQualifierDesc);
  return USBD_CUSTOM_HID_DeviceQualifierDesc;
}

/**
* @brief  USBD_CUSTOM_HID_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CUSTOMHID Interface callback
  * @retval status
  */
uint8_t  USBD_CUSTOM_HID_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                             USBD_CUSTOM_HID_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData= fops;
    ret = USBD_OK;    
  }
  
  return ret;
}
/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
