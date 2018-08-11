#ifndef __BUILD_CFG_H
#define __BUILD_CFG_H

#define 	__USE_CAN_BUS  
#define __QPCR_HW

#ifdef __QPCR_HW
		#define MAX_PWM_RATION	0.97
		#define DUTY_LIMIT				484
		#define	__USE_IC_DRIVE 
#endif
#define __FAN_ALWAYS_ON
#define __FAN_EXT_AFTER_CYCLE

#ifdef  __FAN_EXT_AFTER_CYCLE
	#define FAN_EXT_COOL_THRES 	35
#endif

#endif