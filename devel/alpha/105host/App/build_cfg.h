#ifndef __BUILD_CFG_H
#define __BUILD_CFG_H

////////////////////////////////////////////////////////////
// version info 
// /////////////////////////////////////////////////////////
//#define FUNC_TEMP_CTRL   		0x01 // 
//#define FUNC_G1_CTRL		 	0x03 // system function 0x03 -- 1x PCR image BULK control
#define FUNC_CODE   				0x10 //host function
#define VERSION_INFO_MSB    0x01  
//#define VERSION_INFO_LSB    0x02
//#define VERSION_INFO_LSB    0x03  // add debug on LED, 																	
//#define VERSION_INFO_LSB    0x04  // add delay after USB send
//#define VERSION_INFO_LSB    0x05    // fix addtional USB reply by a new DONE_ACCEPT, image send already. 
//#define VERSION_INFO_LSB    0x06    // first for alpha H/W. change IC to 105RC from 105R8
//#define VERSION_INFO_LSB    0x07 			// @20170430: remove USBD_Usr_cb_TypeDef USBD_USR_Init()																			
//#define VERSION_INFO_LSB      0x08	    // @20170511: to send mailbox after whole image received only
//#define VERSION_INFO_LSB      0x09      // @20170512:  all CAN1 command length longer than one packet will be sent by 
																				// interrupt check pending flag
#define VERSION_INFO_LSB      0x0A      // @20170517:  clear image_pending flag to fix the case image read
#define YEAR_INFO    17 // year 2015
#define MONTH_INFO   5  // month 9
#define DATE_INFO    17	// date  5th	// change I2C to OD mode	
///////////////////////////////////////////////////////////////
// HW config
///////////////////////////////////////////////////////////////
#define __QPCR_HW

#endif