#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H


#define STLM75_ADDR_RD 0x91
#define STLM75_ADDR_WR  0x90

#define TEMP_SENSOR_2_ADDR_RD	0x93
#define TEMP_SENSOR_2_ADDR_WR	0x92	

#define SENSOR_TOTAL	2

#define I2C_IDLE        0
#define I2C_START       1
#define I2C_ADDR_RD     2
#define I2C_ADDR_WR     8
#define I2C_RESTART     3
#define I2C_OFFSET      4
#define I2C_DATA_RD     5
#define I2C_PRESTOP     6
#define  I2C_STOP        7
#define  I2C_CMPLT       8

//extern u8 I2C_State_Machine;
//extern u8 TermpValue[2];
//extern u8 I2C_Rx_Cnt;

#define NULL_DIR		0x0
#define POS_DIR			0x1
#define NEG_DIR			0x2
#define BI_DIR			0x3


#define SHUT		0x0
#define OPEN		0x1

typedef enum {TEMP1=1,TEMP2} TempControlNum;

//void TempSensor_Initial(TempControlNum TempNum);
void TempControl_Initial(float TempSet);
void TempControl_2_Initial(float TempSet);
float  TempSensorRead(u8 IIC_Addr);

#ifdef __SENSOR_TMP100
bool Sensor_Cfg_Write(u8 idx, u8 dt);
u8 Sensor_Cfg_Read(u8 idx);
#endif

#ifdef __SENSOR_TMP112
bool Sensor_Cfg_Write(u8 idx, u16 dt);
u16 Sensor_Cfg_Read(u8 idx);
#endif

void Sensor_Res_Init(void);
//float  TempSensorRead(void);
void TempControl_stop(void);
void TempControl_2_stop(void);
void PWM1_Init(void);
void PWM2_Init(void);
void PWM1_FREEZE(void);
void PWM2_FREEZE(void);
void PWM1_RELEASE(void);
void PWM2_RELEASE(void);
void TempContorl_2_Init(void);
void Channel_Swap(u8 sensor_num);
void PID_Clear(u8 INDEX);

extern float TempCurrent;
extern float TempCurrent_2;
extern float TempCurrent_reg;
extern float TempCurrent_2_reg;
extern u8 sensor_1_error_cnt;
extern u8 sensor_2_error_cnt;

/*
extern u16 TempCtrlTime_Sec;   
extern u16 TempTickLength;
extern u16 TempCtrlTime_Sec_2;   
extern u16 TempTickLength_2;
*/
extern u8 TempCtrl_Active;
extern u8 TempCtrl_Active_2;
extern u8 TempValid;
extern u8 Control_2_Dir;
extern float TempSet;
extern float TempSet_2;
extern float TempPumpSet;
extern u8 TickLock;
extern u8 Peltier_Swap_Msg;

extern float Kp[PID_SLOP_SEG_MAX];
extern float Ki[PID_SLOP_SEG_MAX];
extern float Kd[PID_SLOP_SEG_MAX];
extern float Kl[PID_SLOP_SEG_MAX];
extern float Ktm[PID_SLOP_SEG_MAX];
typedef struct PID_Err
{
	float Current_Err;
	float Previous_Err;
	float Current_Derr;
	float Previous_Derr;
	float Inte_Err;
}PID_Err_Rec_type;
#define TOTAL_SENSOR_NUM 2
#define INDEX0  0
#define INDEX1  1

#define SENSOR1_INDEX	INDEX0
#define SENSOR2_INDEX	INDEX1
extern PID_Err_Rec_type PID_Err_Rec[TOTAL_SENSOR_NUM];

#define SENSOR_CFG_MSK  	0x60
#if defined(SENSOR_RESOLUION_11)
	#define SENSOR_RES	0.125
	#define SENSOR_R1_R0	(0x2<<5)
	#define SENSOR_LSB_SHIFT	5		
#else
	#if defined(SENSOR_RESOLUION_12)
		#define SENSOR_RES	0.0625
		#define SENSOR_R1_R0	(0x3<<5)
		#define SENSOR_LSB_SHIFT	4
	#else
		#define SENSOR_RES	0.25
		#define SENSOR_R1_R0	(0x1<<5)
		#define SENSOR_LSB_SHIFT	6
				
	#endif	
#endif

#ifdef __SENSOR_TMP112
#define SENSOR_CR1_CR0 0x11
#endif
	
typedef u8 MsgQ;
#ifdef DEBUG_MSG
extern u8 msg_debug;
#endif
////////msg for Cycle control
//extern MsgQ MsgStk;
#define AssertMsg(x) (MsgStk & x)
MsgQ PollMsg(void);
void MsgHandler(void);
void PopMsg(u8 msk);
void PushMsg(u8 msk);
void ClearMsg(void);
extern float Fan_Gap_temp;

extern u16   Tick_OverShot;
extern float TempSet_2_Reg;
extern float Temper_OverShot;
#ifdef __USE_TWO_OVERSHOT
		extern float Temper_colding_OverShot;
		extern u16   Tick_colding_OverShot;
#endif
extern u8 trg_assert;
extern float integrationDur;
#define IS_IDLE 0
#define IS_ACTIVE 1
#define IS_ASSERT 2
#define IS_DONE  3
#endif
