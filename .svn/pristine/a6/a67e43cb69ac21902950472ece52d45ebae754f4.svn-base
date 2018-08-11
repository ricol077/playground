/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the USBD descriptors and string formating method.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#include "usbd_conf.h"
#include "usb_regs.h"


 
//for temp
#define USBD_VID                     0x0683//0x0486		 // VID
#define USBD_PID                     0x5850//0x0002		 // PID

//for image
//#define USBD_VID                     0x0483		 // VID
//#define USBD_PID                     0x5750    // PID

#define USBD_PVN                     0x0200		 // 设备版本2.00

#define USBD_LANGID_STRING            0x409		 // 语言ID
#define USBD_MANUFACTURER_STRING      "STM32 USB Test"	   // 厂商字符串

#define USBD_PRODUCT_HS_STRING        "Joystick in HS mode"
#define USBD_SERIALNUMBER_HS_STRING   "00000000011B"

#define USBD_PRODUCT_FS_STRING        "My HID"		     // 产品字符串
#define USBD_SERIALNUMBER_FS_STRING   "00000000012C"		// 产品序列号字符串

#define USBD_CONFIGURATION_HS_STRING  "HID Config"
#define USBD_INTERFACE_HS_STRING      "HID Interface"

#define USBD_CONFIGURATION_FS_STRING  "HID Config"		  
#define USBD_INTERFACE_FS_STRING      "HID Interface"


// 描述符
USBD_DEVICE USR_desc =
{
  USBD_USR_DeviceDescriptor,
  USBD_USR_LangIDStrDescriptor, 
  USBD_USR_ManufacturerStrDescriptor,
  USBD_USR_ProductStrDescriptor,
  USBD_USR_SerialStrDescriptor,
  USBD_USR_ConfigStrDescriptor,
  USBD_USR_InterfaceStrDescriptor,
  
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* USB Standard Device Descriptor 设备描述符 */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
  {
    0x12,                       /*bLength 改描述符长度 */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType   描述符类型 */
    0x00,                       /*bcdUSB  USB版本 2bytes低字节在前*/
    0x02,
    0x00,                       /*bDeviceClass 类代码 */
    0x00,                       /*bDeviceSubClass 子类代码 */
    0x00,                       /*bDeviceProtocol 设备所使用的协议 */
    USB_OTG_MAX_EP0_SIZE,      /*bMaxPacketSize   端点0包最大字节数 */
    LOBYTE(USBD_VID),           /*idVendor VID 厂商ID*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID),           /*idVendor PID 产品ID*/
    HIBYTE(USBD_PID),           /*idVendor*/
    LOBYTE(USBD_PVN),           /*bcdDevice rel. 2.00   设备版本号 2bytes */
    HIBYTE(USBD_PVN),
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string 描述厂商的字符串的索引 */
    USBD_IDX_PRODUCT_STR,       /*Index of product string 描述产品的字符串的索引 */
    USBD_IDX_SERIAL_STR,        /*Index of serial number string 产品序列号字符串的索引 */
    USBD_CFG_MAX_NUM            /*bNumConfigurations 配置数量 */
  } ; /* USB_DeviceDescriptor */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

//#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
//  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
//    #pragma data_alignment=4   
//  #endif
//#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* USB Standard Device Descriptor 定义语言ID */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
{
     USB_SIZ_STRING_LANGID,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING), 
};



/**
* @brief  USBD_USR_DeviceDescriptor 	    获取设备描述
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length)
{
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

/**
* @brief  USBD_USR_LangIDStrDescriptor 			获取语言ID描述
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length)
{
  *length =  sizeof(USBD_LangIDDesc);  
  return USBD_LangIDDesc;
}


/**
* @brief  USBD_USR_ProductStrDescriptor 			  获取产品描述
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ProductStrDescriptor( uint8_t speed , uint16_t *length)
{
 
  
  if(speed == 0)
  {   
    USBD_GetString (USBD_PRODUCT_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ManufacturerStrDescriptor 		  获取厂商描述
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ManufacturerStrDescriptor( uint8_t speed , uint16_t *length)
{
  USBD_GetString (USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_SerialStrDescriptor 				获取产品序列号描述
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed  == USB_OTG_SPEED_HIGH)
  {    
    USBD_GetString (USBD_SERIALNUMBER_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ConfigStrDescriptor 				 获取配置描述符
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed  == USB_OTG_SPEED_HIGH)
  {  
    USBD_GetString (USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length); 
  }
  return USBD_StrDesc;  
}


/**
* @brief  USBD_USR_InterfaceStrDescriptor 			获取接口描述符
*         return the interface string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString (USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;  
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

