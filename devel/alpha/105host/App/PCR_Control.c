
#include <includes.h>
#include "PCR_Cycle.h"

u8 PIX_Buf[16]={0};
u8 REG_Buf[16]={0};
u8 OSC_Status=0;
u8 OSC_mode=0;
u8 OSC_Busy=0;
u16 UserCountMS=1; // LPG: this is user command data, 1ms/count
u16 BaseCounter=0; // LPG: this is to count the base tick --1ms

static u8 Current_Stage=0;
static u8 Current_Cycle=0;  // record cycle & stage, be protected
static u8 Current_Sect=0;
static u8 FanCSR=0;         // FanCSR:  status register /* xxxxxx mode, status*/

//////////////////////////
float TempSet_2_Reg=0;
u16   TickLength_Reg_2=0;

u8 SetPoint_Check(u8 Stage_Num,CycleTempType *pCycleArray)  /* check whether new data is out of range*/
{
	u8 i;
	for(i=0;i<Stage_Num;i++)
	{
		if((pCycleArray[i].SetPoint.float_num < MIN_TEMPERATURE) || (pCycleArray[i].SetPoint.float_num>MAX_TEMPERATURE))
			return INVALID;
		if(pCycleArray[i].SetTime_Union.SetTime<1)
			return INVALID;
	}
	return VALID;	
}

/* copy the new setpoint into set register*/
/* parameter : stage number , source address, destination address*/
void SetPoint_Copy(u8 Stage_Num,CycleTempType *pSource, CycleTempType *pDest)
{
	u8 i;
	if(Stage_Num>0)
	{
		for(i=0;i<Stage_Num;i++)
		{
			pDest[i].SetPoint.float_num=pSource[i].SetPoint.float_num;
			pDest[i].SetTime_Union.SetTime=	pSource[i].SetTime_Union.SetTime;
		}		
	}
}


