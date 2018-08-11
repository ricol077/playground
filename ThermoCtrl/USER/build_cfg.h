#ifndef __BUILD_CFG_H
#define __BUILD_CFG_H

#define 	__USE_CAN_BUS  
//#define	__USE_USB_BUS
#define __QPCR_HW

#ifdef __QPCR_HW
		#define MAX_PWM_RATION	0.97
		#define DUTY_LIMIT				484
		
		#define	__USE_IC_DRIVE 
		//#define __USE_NMOS_DRIVE
#endif
#define __FAN_ALWAYS_ON
///////// version 1.7:  FAN valid until peltier < 35 at the end of cycle
#define __FAN_EXT_AFTER_CYCLE

	#ifdef  __FAN_EXT_AFTER_CYCLE
			#define FAN_EXT_COOL_THRES 	35
	#endif
// @2017-10-14 version 1.9 to support independent overshot for colding
#define __USE_TWO_OVERSHOT
////////////// branch beta on RPI
//#define __PROTOCOL_RPI_V1
//#define __UI_MODE
// add @2018-06-23 support lysis experiment

// @2018-06-23 version 1.14 for lysis feature support
#define __USE_LYSIS   
#define __HAS_CONFIGURABLE_SNAP



#endif




