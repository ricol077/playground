#ifndef __USER_H
#define __USER_H

#include "build_cfg.h"



///////////MAX PCR number support
#define MAX_PCR_CH	4

#define SYS_CLK_72M
#define SAMPPLE_INTERVAL_1_66	 // sample interval of, scale is 100ms

#define MAX_TEMPERATURE  120 // max temperature to limit the range
#define MIN_TEMPERATURE  9	 // min temperature to limit the range

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
extern  u8 PixReadmMode;
extern u16 InteCnt;
extern u8 PixReadmMode;


#define CMD_SET	0x1
#define CMD_GET	0x2
#define CMD_READ	0x4
#define CMD_MEASURE	0x8

////////////////////////////debug usage
#define CMD_DEBUG			0x09
#define TYP_RAM_DUMP	0x01
////////////////////////////

#define CMD_TEMP_SET 0x10  // peltier control
#define TEMP_SET_ON	 0x01   // peltier ON
#define TEMP_SET_OFF 0x00 // peltier off
#define TEMP_READ_SENSOR	0x02	 // read i2c sensor
#define TYP_FAN_CTRL		0x03	//  Fan control
#define TYP_FAN_READ        0x0A    //  Fan status read
#define TYP_FAN_GAP_SET			0x4
#define TYP_FAN_GAP_READ		0x5




#define CMD_PID_CFG  0x11
#define CMD_PID_READ 0x12

/////////////CYCLE control
#define CMD_CYCLE_CTRL	0x13
#define TYP_CYCLE_SENSOR_1	0x1
#define TYP_CYCLE_SENSOR_2	0x2

#define TYP_CYCLE_LOAD   0x3
#define TYP_PE_LOAD		 0x4
//#define TYP_BOOT_LOAD    0x5

#define TYP_OVERSHOT_TIM	0x5
#define TYP_OVERSHOT_TEM	0x6

#define TYP_PWM_LIMIT     0x7

#define CMD_CYCLE_CTRL_READ  0x14
#define CYCLE_STATE_POLL		 0x15
#define TYP_CYCLE_READ	0x1
#define TYP_PWM_READ	0x2
//#define CMD_PWM_READ  0x15


////////////// trigger status to let PC know
#define CMD_HOST_NOTIFY	0x15
#define TRG_VALID		0x1
///////////////////////////// new comand for auto triger


//////////////////////////////
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
#define TYP_PCR_TRG_MASK 0x24
// 0x25
// 0x26 reserve for PCR_Gen4, PCR_select
#define TYP_VER_INFO		0x27
#ifdef __USE_USB_BUS

        #define FUNC_TEMP_CTRL   0x01 // sytem function 0x01 -- temperature control
				//#define FUNC_G1_CTRL		 0x03 // system function 0x03 -- 1x PCR image BULK control
        #define VERSION_INFO_MSB    0x01
				#define VERSION_INFO_LSB    0x07 // remove overshot limitation // skip 1.5 / 1.6 which is RAM dump
				#define YEAR_INFO    17 // year 2015
				#define MONTH_INFO   2  // month 9
        #define DATE_INFO    21 	// date  5th	// change I2C to OD mode		
#else

        #define FUNC_TEMP_CTRL   0x11 // sytem function 0x11 -- CAN BUS, temperature control
				//#define FUNC_G1_CTRL		 0x03 // system function 0x03 -- 1x PCR image BULK control
        #define VERSION_INFO_MSB    0x01
				//#define VERSION_INFO_LSB    0x02 			// fix unexpected TxBuffer[0] overwrite
				//#define VERSION_INFO_LSB    0x03   		// for alpha H/W
				//#define VERSION_INFO_LSB    0x04      // change to use NMOS, N1G, N2G output complementary
				//#define VERSION_INFO_LSB    0x05			// support IC drive / NMOS drive H/W  by definiation
				//#define VERSION_INFO_LSB    0x06   		// fix PWM ouput in active high, information is wrong from SCH.\
																								// and PWM 97% = 485 limitation when in active high state
				#define VERSION_INFO_LSB    0x07 		  	// keep FAN on untill < 35 after cycle done.																	
				#define YEAR_INFO    17 // year 2015
				#define MONTH_INFO   6  // month 9
        #define DATE_INFO    3 //13 @20170413//4 @0412 //6: version 1.3 	// date  5th	// change I2C to OD mode		
#endif
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
extern u8 SPI_Sel;
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
#define INTERVAL_DELAY_US	600 // this number is based on a measurement of SPI process after integration times up
#define INTERVAL_DELAY_COUNT (INTERVAL_DELAY_US/TIME_US_PER_CNT)//-(INTE_DELAY_COUNT))	
/////////////////////////////////




#define PCR_RST	PGout(8)

/////////////////////////////
/*LPG: PC0 is external OSC enable*/
#define DEFAULT_INTE 1
#define OSC_ON	1
#define OSC_OFF	0
#define OSC_AUTO	2
#define OSC_Ctrl PGout(6)// DS1	
#define OSC_ENABLE	 {OSC_Ctrl=OSC_ON;}
#define OSC_DISABLE	 {OSC_Ctrl=OSC_OFF;}
void OSC_Ctrl_Init(void);//³õÊ¼»¯	
extern u8 OSC_Status;
extern u8 OSC_mode;
extern u8 OSC_Busy;
extern u16 UserCountMS;
extern u16 BaseCounter;

//status	of cycle: 
                       //   INIT, INIT_DONE, PUMP_RUN, PUMP_DONE, CYCLE_RUN, CYCLE_DONE,EXT_RUN, EXT_DONE
typedef enum
{	
  READY,
	WAIT,
	ACTIVE,
	FAN_EXT
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
			#define SAMPLE_CLOCK 1666
		#else
			#define TICK_PER_SEC 0x2
			#define SAMPLE_CLOCK 4999
		#endif	
	#endif
#endif



#define TEMP_GAP_SWAP	50//8

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

#define USB_WAIT_MODE
#if defined(USB_WAIT_MODE)
	void EpMsgStk(MSG_TYP u8Q);
	MSG_TYP EpMsgPop(void);
	void EpMsgClr(void);
	#define USB_WAIT_TIM	5
	#define EP1_READ		0x1
#endif

///////////PID use multi slops @2014-12-03, now set 2 slop
#define PID_SLOP_SEG_MAX	2

#define LED_CTRL	PFout(5)

//#define __USE_PI
#define __USE_PID

#define DELTA_THRES  20
#define NEG_DELTA_THRES  5
extern float delta_threshold;

extern u16 TickLength_Reg_2;
extern MSG_TYP PcrMskReg;

///// original definition is PF6--PF9, below is to workaround on Rev02 H/W
#ifdef __QPCR_HW       // qPCR HW, which use 105 host by CAN
		#define TRG_0 PCout(10)
		#define TRG_1 PCout(11)
		#define TRG_2 PCout(12)
		#define TRG_3 PDout(2)	
#else
		#define TRG_0 PFout(5)
		#define TRG_1 PFout(6)
		#define TRG_2 PFout(7)
		#define TRG_3 PAout(2)
#endif 

extern u16 SetTm_LED_Delay;

//////////Ram debug @2017-01-08
//#define __USE_RAM_DEBUG
#ifdef __USE_USB_BUS
	#ifdef __USE_RAM_DEBUG
		#define DUMPSIZE				6000
		#define SIZE_PER_PACKET	(MAX_LIMIT-10)   //head,resp,cmd,len,type,total,seq,....checksum,tail,tail 
		#define TRG_OUT	PDout(8)
	#endif
#endif
/////////////////////////////
#endif
