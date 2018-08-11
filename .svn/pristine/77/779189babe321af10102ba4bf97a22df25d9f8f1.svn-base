#ifndef __USER_H
#define __USER_H

#include "build_cfg.h"




///////////MAX PCR number support
#define MAX_PCR_CH	4

#define SYS_CLK_72M
#define SAMPPLE_INTERVAL_1_66	 // sample interval of, scale is 100ms

#define MAX_TEMPERATURE  100 // max temperature to limit the range
#define MIN_TEMPERATURE  50	 // min temperature to limit the range

#define SENSOR_RESOLUION_12 
#define __SENSOR_TMP112

#ifndef _DMA
	#define _DMA
#endif
#define SPI_BUF_BYTE_LEN	40

#define BETA_VER

#if defined(BETA_VER)
	  #define PIX_TOTAL_ROW 12	
	  #define ALL_ROW_MASK	0xFFF
	  #define YG_FIX_LEN	64
#else
	  #define PIX_TOTAL_ROW 16
	  #define ALL_ROW_MASK  0xFFFF
	  #define YG_FIX_LEN	64
#endif

#define PIX_TOTAL_COL	(PIX_TOTAL_ROW+1)
#define VIDEO_TOTAL_ROW	0xff
#define REG_MAX_NUM	6

#define SPI_BUSY	0x1
#define SPI_CMPLT	0x2
#define SPI_IDLE	0x0	

/////register address map
/*
#define RAMP_TRIM_ADDR	0x1
#define RANG_TRIM_ADDR	0x2
#define V24_TRIM_ADDR	0x3
#define V20_TRIM_ADDR	0x4
#define V15_TRIM_ADDR	0x4
#define IPIX_TRIM_ADDR	0x3
#define SWITCH_ADDR		0x5
#define TX_PATTERN_ADDR	0x5
#define AMUX_CONTROL_ADDR	0x6
#define TEST_ADC_SEL_ADDR	0x6
*/
#define RAMP_TRIM_ADDR	0x5
#define RANG_TRIM_ADDR	0x4
#define V24_TRIM_ADDR	0x3
#define V20_TRIM_ADDR	0x2
#define V15_TRIM_ADDR	0x2
#define IPIX_TRIM_ADDR	0x3
#define SWITCH_ADDR		0x1
#define TX_PATTERN_ADDR	0x1
#define AMUX_CONTROL_ADDR	0x6
#define TEST_ADC_SEL_ADDR	0x6



//// PCR chip comand
#define PXL_DRIVE		0x3
#define REG_WRITE		0x2
#define ADC_START		0x4
#define PCR_ADC_READ	0x5

#define HEADER	0xAA
#define TAIL    0x17
#define ACK		0x0

#define IDLE	0x0
#define BUSY	0x1
#define END		0x2
#define CMPLT	0x3
#define MAX_LIMIT	64
 
#define CMD_BYTE_NUM	1
#define LEN_BYTE_NUM	2
#define TYP_BYTE_NUM	3
#define ROW_BYTE_NUM	4	

//// checking the packet length
#define MIN_PACKET_LENGTH	1


extern u8 RxBuffer[MAX_LIMIT];
extern u8 RxIdx;
extern u8 RxStage;

extern u16 TMR_Int_Flag; 
extern u16 TMR_Int_Flag_1; 
extern u16 TMR_Int_Flag_2; 
extern u16 TMR_Int_Flag_3; 
extern  u8 PixReadmMode;
extern u16 InteCnt;
extern u8 PixReadmMode;


#define CMD_SET	0x1
#define CMD_GET	0x2
#define CMD_READ	0x4
#define CMD_MEASURE	0x8

#define CMD_TEMP_SET 0x10  // peltier control
#define TEMP_SET_ON	 0x01   // peltier ON
#define TEMP_SET_OFF 0x00 // peltier off
#define TEMP_READ_SENSOR	0x02	 // read i2c sensor
#define TYP_FAN_CTRL		0x03	//  Fan control
#define TYP_FAN_READ        0x0A    //  Fan status read
				 
#define CMD_PID_CFG  0x11
#define CMD_PID_READ 0x12

/////////////CYCLE control
#define CMD_CYCLE_CTRL	0x13
#define TYP_CYCLE_SENSOR_1	0x1
#define TYP_CYCLE_SENSOR_2	0x2

#define TYP_CYCLE_LOAD   0x3
#define TYP_PE_LOAD		 0x4
#define TYP_BOOT_LOAD    0x5

#define CMD_CYCLE_CTRL_READ  0x14
#define TYP_CYCLE_READ	0x1
#define TYP_PWM_READ	0x2
//#define CMD_PWM_READ  0x15


////////////// trigger status to let PC know
#define CMD_HOST_NOTIFY	0x15
#define TRG_VALID		0x1
/////////////////////////////

	/////////////PE_LOAD packet
#define CYCLE_SET_BYTE	0x4
#define CYCLE_NUM_BYTE  0x5
#define CYCLE_RSV_BYTE  0x6    
//////////////////////

#define TYP_PID_KP	0x1
#define TYP_PID_KI	0x2
#define TYP_PID_KD	0x4
#define TYP_PID_KP_KI	0x3
#define TYP_PID_KP_KI_KD	0x7
#define TYP_PID_KL   0x8

#define TYP_PID_KTM  0x9 // add @2014-12-03, PID segment point

/// row read mode								
#define TYP_ROW		0x1
#define TYP_IMAGE	0x2
#define TYP_VIDEO	0x3
#define TYP_DEBUG_1	0x4  // XinChuang debug, only SPI read,  required @ 131022
#define TYP_DEBUG_2	0x5  // new feature @ 13-10-23, only ADC conversion & SPI read
#define TYP_24PIXROW 0x7
#define TYP_24PIXIMAG 0x8
#define ENABLE		0x1
#define DISABLE		0x0


#define IMAGE_24	0xFE
#define VIDEO_24	0xFF
//////// command type definition
#define RAMP_TRIM 0x1
#define RANG_TRIM 0x2
#define V24_TRIM 0x3
#define V20_TRIM 0x4
#define V15_TRIM 0x5
#define IPIX_TRIM 0x6
#define SWITCH_BIT 0x7
#define TX_PATTERN 0x8
#define AMUX_CONTROL 0x9
#define TEST_ADC 0xA

#define OSC_CONTROL	0xE
#define PCR_RESET	0xF
#define SPI_PULSE 0x10
#define INTE_TIME 0x20
#define LED_PRO_TIME 	0x21
#define LED_HOLD_TIME 	0x22
#define LED_SWITCH      0x23
#define TYP_PCR_TRG_MASK	0x24

#define TYE_TRG_PIX_MODE 0x25   // multi PCR will capture image before PC command GET, 
//#define TYE_12PIX_MODE 0x26   // so must set pixel mode, by this quick set 
															// or set in pattern type previous defined
#define TYP_PCR_SEL		0x26    // PCR channel selection, for multi CH 
#define TYP_VER_INFO		0x27
        //#define FUNC_G4_CTRL   0x02 // sytem function 0x02 -- 4x PCR image control
				//#define FUNC_G1_CTRL		 0x03 // system function 0x03 -- 1x PCR image BULK control
#ifdef   __INTEGRATION_INDIV

				#ifndef __USE_CAN_BUS
								#define FUNC_G4_OVERLAP   0x06 // sytem function 0x05 -- 4x PCR image overap --- temp FW code
				#else
								#define FUNC_G4_OVERLAP_ON_CAN   0x16
								#define FUNC_G4_OVERLAP					FUNC_G4_OVERLAP_ON_CAN
				#endif
#else
				#ifndef __USE_CAN_BUS
								#define FUNC_G4_OVERLAP   0x05 // sytem function 0x05 -- 4x PCR image overap --- temp FW code
				#else
								#define FUNC_G4_OVERLAP_ON_CAN   0x15
								#define FUNC_G4_OVERLAP					FUNC_G4_OVERLAP_ON_CAN
				#endif	
#endif

        #define VERSION_INFO_MSB    0x01
				//#define VERSION_INFO_LSB    0x07  //sync with shutter enable command, but default is enable to comply with 
				                                  // current Zhimin's software as it dosen't incude shutter feature yet.
				//#define VERSION_INFO_LSB    0x08 // run on qPCR
				//#define VERSION_INFO_LSB    0x09 		// enable autosnap 
				//#define VERSION_INFO_LSB    10        // for alpha H/W, aand change MCU to 103RE from 103ZE @ 170406
				//#define VERSION_INFO_LSB    11          // disable multi-cahnnel overlap by request
				//#define VERSION_INFO_LSB    12  					// fix read GPIO_C
				//#define VERSION_INFO_LSB    13 		// @20170424  to control OSC on each channel per swiss project
				//#define VERSION_INFO_LSB    14 			// @20170428  to add button
				//#define VERSION_INFO_LSB    15			// @20170518: just a version change
				//#define VERSION_INFO_LSB    16      //@20170603: fix bug on Lucentix, ungate power before PCR ADC read  
				//#define VERSION_INFO_LSB    17 			//@20170604: change LED-Channel mapping per SX request
				//#define VERSION_INFO_LSB    18			//@20170605: increase delay between ungate the power & ADC read
																							// per ZM comment there's big capcity on AMP
				#define VERSION_INFO_LSB    19
				#define YEAR_INFO    17 // year 2015
				#define MONTH_INFO  6  // month 9
        #define DATE_INFO   8	


#define TYP_IMG_EN	0x28

#define AMUX_OUT	0x40

#define PCR_TEMP	0x11  // add @ 2014-01-02 by XinChuang
#define K_PCR_DIFF	12.3  // (529.43+577.61)/(100-10)
#define A_PCR_PNT_10	(-577.61)

       // add @14-10-07	 for new temperature read thru OP
	#define CH_INDEX_0  0
	#define CH_INDEX_1	1
   
	   // 
//// response code

#define TOP_REG_ADDR		
#define MAX_LENGTH_VALID	16
#define MAX_ROW_NUM			16


#define ERR_LENGTH			1
#define ERR_CMD				2
#define ERR_ROW				3
#define ERR_CHKSUM			4
#define ERR_TYP				5
#define NO_ERR				0
#define BAD_DATA			6
#define OUT_RANGE			7
#define ERR_UNDEF			8   
/* LPG: local register map record PCR configuration */
typedef struct
{
  u8 RampTrim;
  u8 RangTrim;
  u8 Ipix_V24Trim;		/* ipix & V24 int the same byte, */
  u8 V20_V15Trim; 		/* V20 & V15 it the same byte */
  u8 SW_TxCtrl; 		/* switch & TX pattern control */ 
  u8 TsTADC_AmuxCtrl;   /* test ADC & Amux control     */

  u8 SpiInsertion;		/* SPI insertion*/
  u16 InteTime;			/* Integration time in mS*/
  u16 InteCount;		/* INtegration count for timer*/
  u16 InteDelayCount;	

} PCR_Regs_type;
extern PCR_Regs_type PCR_Regs[MAX_PCR_CH];

#define RAMP_MASK	0xff
#define RAMP_SHIFT	0x00

#define RANG_MASK	0x0f
#define RANG_SHIFT	0x00

#define V24_MASK	0x0f
#define IPIX_MASK	0xf0
#define V24_SHIFT	0x00
#define IPIX_SHIFT	0x04

#define V15_MASK	0x0f
#define V20_MASK	0xf0
#define V15_SHIFT	0x00
#define V20_SHIFT	0x04

#define SW_MASK		0x10
#define TX_MASK		0x0f
#define SW_SHIFT	0x04
#define TX_SHIFT	0x00


#define AMUX_MASK	0x07
#define TSTADC_MASK	0x10
#define AMUX_SHIFT	0x00
#define TSTADC_SHIFT 0x04

#define RAMP_ALGN(x) ((u8)x)
#define RANG_ALGN(x) ((u8)((x & RANG_MASK)>>RANG_SHIFT))
#define V24_ALGN(x)	 ((u8)((x & V24_MASK)>>V24_SHIFT))
#define IPIX_ALGN(x) ((u8)((x & IPIX_MASK)>>IPIX_SHIFT))
#define V20_ALGN(x) ((u8)((x & V20_MASK)>>V20_SHIFT))
#define V15_ALGN(x) ((u8)((x & V15_MASK)>>V15_SHIFT))
#define SW_ALGN(x)	((u8)((x & SW_MASK)>>SW_SHIFT))
#define TX_ALGN(x)	((u8)((x & TX_MASK)>>TX_SHIFT))
#define AMUX_ALGN(x)	((u8)((x & AMUX_MASK)>>AMUX_SHIFT))
#define TSTADC_ALGN(x)	((u8)((x & TSTADC_MASK)>>TSTADC_SHIFT))



#define RAMP_PACK(x,y) ((u8)x)
#define RANG_PACK(x,y) ((u8)(x & RANG_MASK))
#define V24_PACK(x,y)  (((x & 0x0f)<<V24_SHIFT)|(y & ~(V24_MASK)))
#define IPIX_PACK(x,y) (((x & 0x0f)<<IPIX_SHIFT)|(y & ~(IPIX_MASK)))
#define V20_PACK(x,y)  (((x & 0x0f)<<V20_SHIFT)|(y & ~(V20_MASK)))
#define V15_PACK(x,y)  (((x & 0x0f)<<V15_SHIFT)|(y & ~(V15_MASK)))
#define SW_PACK(x,y)  (((x & 0x01)<<SW_SHIFT)|(y & ~(SW_MASK)))
#define TX_PACK(x,y)  (((x & 0x0f)<<TX_SHIFT)|(y & ~(TX_MASK)))
#define AMUX_PACK(x,y)  (((x & 0x07)<<AMUX_SHIFT)|(y & ~(AMUX_MASK)))
#define TSTADC_PACK(x,y)  (((x & 0x01)<<TSTADC_SHIFT)|(y & ~(TSTADC_MASK)))


///////////////////////////////
#define INTE_CYCLE_COUNT	(100u)   /*LPG: adjust for first integration cycle, 1 count= 10us*/
									 /*LPG: the base scale is 1ms, which is 100 count*/
#define TIME_US_PER_CNT		(10)
#define DELAY_ALIGNMENT		(50u)	 /*LPG: SPI & firmware delay in us, this is measured value*/
/////////////////////////////////
///////////////////////////////
#define TIME_US_PER_CNT		(10)
#define INTE_DELAY_COUNT	(22/TIME_US_PER_CNT)//(37u)	  /*LPG: this is record the integration time between first and next during overlap mode*/
									  /*LPG: this is measured by OSC, API Send_Command sending 2 bytes will cost ~22us*/
#define INTERVAL_DELAY_US	400 // this number is based on a measurement of SPI process after integration times up
#define INTERVAL_DELAY_COUNT (INTERVAL_DELAY_US/TIME_US_PER_CNT-1+8)//-(INTE_DELAY_COUNT))	

#define INTERVAL_DELAY_MS	5
/////////////////////////////////

#define DEFAULT_INTE 1
#define OSC_ON	1
#define OSC_OFF	0
#define OSC_AUTO	2

#ifdef __QPCR_HW
	#define PCR_RST	PAout(9)
	#define OSC_Ctrl PAout(4)// DS1	
	#define OSC_Ctrl_1 PBout(10)// DS2	
	#define OSC_Ctrl_2 PBout(11)// DS3	
	#define OSC_Ctrl_3 PCout(6)// DS4	
#else
	#define PCR_RST	PGout(8)
	#define OSC_Ctrl PGout(3)// DS1	
	#define OSC_Ctrl_1 PGout(4)// DS2	
	#define OSC_Ctrl_2 PGout(5)// DS3	
	#define OSC_Ctrl_3 PGout(6)// DS4	
	
	#ifdef __USE_MOS_GATE
				#define MOS_Gate PDout(2)
	#endif 
	
	#ifdef __SWISS_LUCENTIX_BUTTON
				
				#define LED_EX	PFout(14)
				
				#ifdef 	__LUCENTIX_DEBUG_ON_WARSHIP
					#define LED_KEY	PCout(11)
					#define KEY1_IN	PCin(10)
				#else
					#define LED_KEY	PCout(15)
					#define KEY1_IN	PCin(14)
				#endif
				
				#define BUTTON_ACTIVE_MASK  0x1
				#define BUTTON_PROCESSED_MASK	0x2
				#define BUTTON_DONE_MASK		0x4
				
				#define BLINK_HZ						5
				#define BLINK_SEC						10
				#define LED_BLINK_TIMEOUT		50  // 10 second * 5Hz
				#define LUCENTIX_KEY_ACTIVE	0x0
				#define LUCENTIX_CH_BIT					0x3    //lucentix has 2 channel totally, ch0, ch1
				u8 Lucentix_Button_Check(void);
				void Lucentix_Button_Process(u8 sts);
				void Lucentix_Button_msg_clr(void);
				u8 Lucentix_Button_sts_read(void);
				void Lucentix_Button_sts_Bit_Set(u8 ctrl);
				extern u8 blink_count;

	#endif 
	
	
#endif

#define OSC_ENABLE	 {OSC_Ctrl=OSC_ON;}
#define OSC_DISABLE	 {OSC_Ctrl=OSC_OFF;}
void OSC_Ctrl_Init(void);//³õÊ¼»¯	
extern u8 OSC_Status;
extern u8 OSC_mode;
extern u8 OSC_Busy;
extern u16 UserCountMS;
extern u16 BaseCounter;
extern u16 BaseCounter_1;
extern u16 BaseCounter_2;
extern u16 BaseCounter_3;
//status	of cycle: 
                       //   INIT, INIT_DONE, PUMP_RUN, PUMP_DONE, CYCLE_RUN, CYCLE_DONE,EXT_RUN, EXT_DONE
typedef enum
{	
    READY,
	WAIT,
	ACTIVE
}Cycle_STS_Type;

extern Cycle_STS_Type CycleSTS;

#define SENSOR_1  0x1
#define SENSOR_2  0x2
#define SENSOR_PUMP 0xf	  // actually is sensor 1, which works as pump sensor


#if defined(SYS_CLK_72M)
	#define SYS_CLK_M		72
  	#define TIM1_PSC_VAL	71	
	#define TIM3_PSC_VAL	719
	#define TIM4_PSC_VAL	71
	#define TIM6_PSC_VAL	7199
	#define TIM8_PSC_VAL	71
#else	// 64Mhz sys_clock
	#define SYS_CLK_M		64
  	#define TIM1_PSC_VAL	63	
	#define TIM3_PSC_VAL	639
	#define TIM4_PSC_VAL	63
	#define TIM6_PSC_VAL	6399 
	#define TIM8_PSC_VAL	63
#endif

#ifdef SAMPPLE_INTERVAL_1
	#define TICK_PER_SEC 0x0A
	#define SAMPLE_CLOCK 999
#else 
	#ifdef SAMPPLE_INTERVAL_2
		#define TICK_PER_SEC 0x5
		#define SAMPLE_CLOCK 1999
	#else
		#ifdef SAMPPLE_INTERVAL_1_66
			#define TICK_PER_SEC 0x6
			#define SAMPLE_CLOCK 1665
		#else
			#define TICK_PER_SEC 0x2
			#define SAMPLE_CLOCK 4999
		#endif	
	#endif
#endif



#define TEMP_GAP_SWAP	8

//#define DEBUG_MSG

typedef union 
{
	struct
	{
		u8 byte0;
		u8 byte1;
		u8 byte2;
		u8 byte3;
	}tempd;
	float float_num;
}KL_union;


typedef struct
{
	u8 CycleValid;
	u8 TotalCycle;
	u8 TotalSect;
	u8 TotalStage;
}CycleControlType;

typedef struct
{
 	KL_union SetPoint;
	union
	{
		struct
		{
			u8 byte0;
			u8 byte1;
		}tempw;
		u16 SetTime;
	}SetTime_Union;	
}CycleTempType;
#define MAX_STAGE  10
#define EMPTY		0

#define SS1_TMOUT	0x1
#define SS2_TMOUT	0x2
#define SS1_ACT3	0x4		
#define SS2_CY_CMP  0x10	

#define UART_PORT	0x0
#define USB_PORT    0x1

//#define PIX24_DEBUG_UART_PRINT

#ifdef PIX24_DEBUG_UART_PRINT
#undef PIX24_DEBUG_UART_PRINT
#endif


typedef u8 MSG_TYP;

//#define USB_WAIT_MODE
#if defined(USB_WAIT_MODE)
	void EpMsgStk(MSG_TYP u8Q);
	MSG_TYP EpMsgPop(void);
	void EpMsgClr(void);
	#define USB_WAIT_TIM	5
	#define EP1_READ		0x1
#endif

///////////PID use multi slops @2014-12-03, now set 2 slop
#define PID_SLOP_SEG_MAX	2
//#define SPI_DEBUG

//#define INTE_DEBUG

#ifdef INTE_DEBUG
extern u8 inte_toggle;
#endif 
#define FAN PFout(4) //#define FAN PCout(5)// DS0

extern u8 multi_overlap_ctrl;

//#define _OVERLAP_TEST
//#define _OVERLAP_ISSUE_DEBUG



#endif
