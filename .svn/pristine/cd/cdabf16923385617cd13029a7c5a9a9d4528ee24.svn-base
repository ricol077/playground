#ifndef __PCR_CYCLE_H
#define __PCR_CYCLE_H

 ////define for cycle conrol
#define INVALID	0
#define VALID	1
#define	INIT	2 
#define INIT_DONE	3
#define PUMP_RUN	4
#define PUMP_DONE	5 
#define CYCLE_RUN 	6 
#define	CYCLE_DONE	7
#define	EXT_RUN		8
#define	EXT_DONE  	9
 
  ///byte format in cycle control packet
#define CYCLE_BYTE	0x4
#define SECT_ALL_BYTE	0x4
#define SECT_BYTE	0x5
#define STAGE_BYTE	0x6

#define SET_START_BYTE	0x7
#define BYTE_PER_STAGE  0x6 // 4 byte temperature, 2 byte time

#define FAN PFout(4) //#define FAN PCout(5)// DS0
#define LOW		0x0
#define HIGH	0X1
void FAN_Init(void);//≥ı ºªØ	

extern CycleControlType  PCR_Cycle_Control,Buffer_Cycle_Control;
extern CycleTempType	 PCR_Cycle_SetPoint[MAX_STAGE],Buffer_Cycle_SetPoint[MAX_STAGE], *pCycleArray;
extern CycleTempType     PE_Cycle_SetPoint[2];

// FanCSR:  status register /* xxxxxx mode, status*/
#define FAN_MODE_MSK	0x2
#define FAN_CTRL_MSK	0x1
void FanCtrl_Force(u8 ctrl);   // manual control 
void FanCtrl_Auto(u8 ctrl);	   // auto control
void FanMode_Clear(void);      // clear mode bit, to be auto

void PCR_Cycle_Init(void);
u8 SetPoint_Check(u8 Stage_Num,CycleTempType *pCycleArray);
void SetPoint_Copy(u8 Stage_Num,CycleTempType *pSource, CycleTempType *pDest);
void SetPoint_Cycle_Start(CycleTempType *p);
u8 Cycle_Check(void);
u8 Stage_Check(void);
void Cycle_Set(u8 cycle);
void Stage_Set(u8 stage);
void Cycle_INC(void);
void Stage_INC(void);
void TempCtrl_Reload_InCycle(void);
void FanCtrl_Init(void);
u8 FanCSR_Read(void);
void SetPoint_PrePump_Start(void);
void SetPoint_Extension_Start(void);
void Cycle_Control_Start(void);
void Fan_Echo(u8 dat);

extern u16   Tick_OverShot;
extern float TempSet_2_Reg;
extern float Temper_OverShot;
extern u8 WaitOverShot;

//////////////RamDbug
#ifdef __USE_RAM_DEBUG
extern u8 RamDumpBlock[6000]; // input temper: 4byte,  PID output, 2 byte 
extern u8 RamDumpFlag;
extern u16 RAM_Dump_index;
extern u8 TickDump;
#endif
////////////////////
void trigger_time_Cal(void);
void trigger_out(void);
void trigger_reset(void);
void trigger_done(void);
#endif
