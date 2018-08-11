#include "math.h"
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h" 
#include "beep.h"	 	 
#include "key.h"	 	 
#include "exti.h"	 	 
#include "wdg.h" 	 
#include "timer.h"		 	 
#include "tpad.h"
#include "oled.h"		 	 
#include "lcd.h"
#include "usmart.h"	
#include "rtc.h"	 	 
#include "wkup.h"	
#include "adc.h" 	 
#include "dac.h" 	 
#include "dma.h" 	 
#include "24cxx.h" 	 
#include "flash.h" 
#include "spi.h"
#include "user_def.h"
#include "temperature.h"
//#include "PID_LIB.lib"
#include "PCR_Cycle.h"	
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h" 
#include "usb_conf.h"
#include "can.h" 
extern u8 Receive_Buffer [nReportCnt];
extern u8 Transi_Buffer [nReportCnt];
extern u8 USB_ReceiveFlg;
extern u16 USB_RxIdx;

#ifdef  __USE_CAN_BUS
			extern u8 CAN_rxbuf[8];
			extern u32 CAN_id;
			extern u8 CAN_ide,CAN_rtr,CAN_rx_len; 
			extern u8 CAN_ReceiveFlg;
#endif
//ALIENTEK战舰STM32开发板实验23
//SPI 实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司  
 				 	
//要写入到W25Q64的字符串数组
const u8 TEXT_Buffer[]={"WarShipSTM32 SPI TEST"};
#define SIZE sizeof(TEXT_Buffer)	

u8 u8buff[SPI_BUF_BYTE_LEN]={0};
u8 PCRChip_Command_Send[SPI_BUF_BYTE_LEN]={0};
u16 TMR_Int_Flag=0;
u8 PCR_ADC_Done_Flag=0;
void PIX_Drive_Sequence_Sim(void);
u8 Send_Command(u8 cmd, u8 row_num, u8 length, u8 * pbuf);

u8 Comand_Buf[MAX_LIMIT]={0};
u8 Command_Len=0;
u8 TxBuffer[MAX_LIMIT]={HEADER};
u8 PacketChkSum(u8 *p, u8 length);
void UARTRes(u8 response, u8 command,u8 * buffer, u8 length);
void USBRes(u8 response, u8 command,u8 * buffer, u8 length);
void Read_Row(u8 row);
void ReadUpdate_Image(void);
//void Data_Cov(u8 txset);

void ReadUpdate_Image24_XC(u8 mode,u8 row);
void ReadUpdate_Image24_slow(u8 mode, u8 row);
u8 SPI_Sel=0;
PCR_Regs_type PCR_Regs[MAX_PCR_CH];

u16 InteCnt=0;	/*LPG:  this is record the integration time */
u8 PixReadmMode;

//u8 ImageBuf[(PIX_TOTAL_ROW<<1)][(PIX_TOTAL_COL<<1)+2]={0};
u8 ImageBuf[PIX_TOTAL_ROW][(PIX_TOTAL_COL<<1)+2]={0};
u8 ImageBufPIX[64][64]={0};
/*
/////// count conversion for integration time
u16 InteCycleCount=INTE_CYCLE_COUNT;
u16 InteDelayCount=INTE_DELAY_COUNT;
u16 UsPerCount=TIME_US_PER_CNT;
*/
void Print_packet(u8 ResCode, u8 command, u8 *TxBuffer, u8 TransLen);
//#define VEDIO_DEBUG
#define PCR_RESET_PROCESS	{PCR_RST=0;delay_us(10);PCR_RST=1;}

u8 msg_debug=0;

void Trim_Reset(void);
void Read_Row_Debug_2(void);
KL_union KL_temp,KP_temp,KI_temp,KD_temp,TempSet_1_temp,TempSet_2_temp,TempCurr_1_temp,TempCurr_2_temp, UnionTemp;

CycleControlType  PCR_Cycle_Control,Buffer_Cycle_Control;
CycleTempType	  PCR_Cycle_SetPoint[MAX_STAGE],Buffer_Cycle_SetPoint[MAX_STAGE];


CycleTempType     *pCycleArray; 

CycleTempType     PE_Cycle_SetPoint[2]; // pre_pump & extension setpoint

Cycle_STS_Type CycleSTS=READY; 

u8 USB_Reply_Tail=FALSE;
u8 const PIXEL_24READ[4]={0x1,0x2,0x4,0x8};
/////for USB

#if defined(USB_WAIT_MODE)
void EpMsgEnable(void);
void EpMsgDisable(void);
MSG_TYP UsbReadDone(void);
#endif


//////////////external trigger .2014-12-20
//#define LED_CTRL	PFout(5)//#define LED_CTRL	PGout(13)	   // led control out
#define TRG_IMG		PFin(6)//#define TRG_IMG		PEin(14)       // trigger input


#define BRD_IN_IDLE	0
#define BRD_IN_TRG 	1
#define BRD_IN_PROCESS	2

u8 mst_flag=BRD_IN_IDLE;  // flag record
u16 Time_LED_Delay=0; 	// timer
u16 SetTm_LED_Delay=10;	// setup time
u16 HoldTm_LED_Delay=10; // hold time 
float Fan_Gap_temp=2;
u8 led_mode=0; 
//////////////
float Temper_OverShot=OVERSHOT_DEFULT;
float	Time_OverShot=OVERSHOT_TIME_DEFAULT;
u16   Tick_OverShot=OVERSHOT_TIME_DEFAULT * TICK_PER_SEC;


// @2017-10-14 version 1.9 to support independent overshot for colding
#ifdef __USE_TWO_OVERSHOT
		float Temper_colding_OverShot=OVERSHOT_DEFULT;
		float	Time_colding_OverShot=OVERSHOT_TIME_DEFAULT;
		u16   Tick_colding_OverShot=OVERSHOT_TIME_DEFAULT * TICK_PER_SEC;
#endif
//////////////////////////////////
float delta_threshold=20;
///////////////////////////////////
MSG_TYP PcrMskReg=0xf; //default enable all channel
u8 TrgSMS=IDLE;
///////////////////////////////////////
void PcrTrgDisable(void);
void PcrMskMapping(u8 mask);

void usb_port_set(u8 enable)
{
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
	if(enable)_SetCNTR(_GetCNTR()&(~(1<<1)));//退出断电模式
	else
	{	  
		_SetCNTR(_GetCNTR()|(1<<1));  // 断电模式
		GPIOA->CRH&=0XFFF00FFF;
		GPIOA->CRH|=0X00033000;
		PAout(12)=0;	    		  
	}
}


#ifdef __USE_CAN_BUS
#define SCALER 8
#define CAN_BUF_START_INDEX	1
#endif


#ifdef __RPI_THERMO

		 const char version_info_string[]="H/W = qPCR Beta. F/W = Thermo v1.10  @2018-02-28";
		 
		 /////////////////////////////////
		 // a flag to support infinite cycle
		 
		 u8 Flag_Current_forever = 0;
		 u8 SnapTrigger_En = 0;
		u8 RpiReplyBuf[2200];
        //u8 RpiRecvBuf[2200];
		u8 RpiReplyLen=0;
		
		CycleSetType RpiSetting;//, RpiSetting_Buffed;
	  CycleChkType RpiChkCmd;
	  RpiCmdType   RpiCmdBuf;
		TimType SnapTime_count_2=0;
		
		u8 SPI1_Flag_wait=0;

		 float TemperatureReg=0;
		 float ThermoSlopRate=0;   // record the ramp rate  C/sec, should be save into flash
		 u32 RampTimeCnt=0;
		 u8  RampFlag=READY;
		 
		#define ACKED           0
		#define PIPE_FULL       1 

		u8 Set_Idx=0;
		u8 Run_Idx=0;
/*
        u8 Stage_Pipe_full=0;
        u8 Stage_Pipe_empty=0;
*/


    /*******************************
    ** RPI version
    *******************************/
/*
    u8                     SPI1_RxBuf[RPI_CMD_PACKET_LEN];
    IMG_CMD_TYPE           RPI_cmd_received;
    ImgBuf_TYPE            Beta_image_buffer;

    BETA_IMG_SET_CFG_TYPE  Beta_Img_Setting_Cfg;
    RPI_REPLY_TYPE         RPI_Reply_buffer;


   void MemoryCopy(u8 * src, u8 * des, u16 length)
   {
       u16 i;
       for(i=0;j<length;i++)
           *(des+i) = *(src+i);
   }


   void ReInitPCRByCH(u8 ch, BETA_IMG_SET_CFG_TYPE * p)
   {
       u8 u8temp;
       SPI_Sel = (*p).ChSetting[ch].chip;
       PCR_Regs[SPI_Sel].InteTime = (*p).ChSetting[ch].IntegrationTime_10us / 100;
       u8temp=(*p).ChSetting[ch].ImgBinPattern;
       
       PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].SW_TxCtrl=TX_PACK(u8temp,PCR_Regs[SPI_Sel].SW_TxCtrl);
       Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[0]);
   }
*/

   u16 SPI_Tick_Cnt=0; 
   u32 tick=0;
   void TIM2_IRQHandler(void)
   {                                   
       if(TIM2->SR & 1)
        {
						tick++;
					  IRQ_PIN= tick & 0x1;
             if(SPI_Tick_Cnt < 65535) 
                    SPI_Tick_Cnt ++;    
        }        
       TIM2->SR&= ~1;
   }

   void Timer2_Init(u16 arr,u16 psc)
   {
       RCC->APB1ENR|=1<<0;//TIM2时钟使能    
       
       TIM2->PSC=psc; 
			 TIM2->CNT = arr;
		   TIM2->ARR=arr;  //设定计数器自动重装值  
       //这两个东东要同时设置才可以使用中断
       TIM2->DIER|=1<<0;   //允许更新中断              
       TIM2->CR1|=(0x01<<4);                                   
       TIM2->CR1|= (1<<3);    //OPM
       MY_NVIC_Init(3,3,TIM2_IRQChannel,2);                                 
   }

   void Timer2_update(u16 arr)
	 {
			TIM2->CR1&= ~1; 
		  TIM2->CNT = arr;
		  TIM2->SR &= ~1;
		  SPI_Tick_Cnt = 0;
		  TIM2->CR1|=1;
	 }
   
   void SPI1_timer_command(FunctionalState cmd)
   {
       
       if(cmd == ENABLE)
       {
           //TIM2->CNT  = 0;
           TIM2->CR1|=0x01;
           
       }
       else
       {
           TIM2->CR1 &= ~0x01;
       }

       SPI_Tick_Cnt=0; 
   }

   
   
   u8 SPI1_TimeOut_handler(void)
   {
       //static u8 SPI1_Timer = SPI_IDLE;
       u8 val=0;
       
       switch(SPI_1_Rx_sts)
       {
           case SPI_BUSY:

                      if(SPI_Tick_Cnt > 0)   //SPI1_INTERVAL_TIMEOUT_CNT)  
												   SPI_1_Rx_sts = SPI_IDLE;

               break;
            
           // add @2018-02-27 to support RPI length header
            case SPI_CMPLT:
            case SPI_OVERFLOW:
                                // in this case, means packet length is enough
                    //SPI1_timer_command(DISABLE);
						        val=1;
                break;

           ////////////////////////////////////////////////

           default:
                       //SPI1_timer_command(DISABLE);
               break;
       }
       return val;

   }

/*     
   u8 SPI1_cmd_packet_handler(u8 * RxIdx, RpiCmdType *p) 
   {
      u8 i,j,k,u8temp;
      U32_PACK_TYPE u32temp;
      F32_PACK_TYPE ftemp;
      
      u8 val=NO_ERR;
      
      switch(SPI1_RxBuf[RPI_CMD_BYTE_NUM])
      {
           case  RSP_CMD__SET_STEP:
                       if(*RxIdx < LENGTH_PACKET__SET_STEP)
                           val=ERR_LENGTH;                         
                       else
                       {
                            p->Cmd = RSP_CMD__SET_STEP;
                            for(i=0;i<4;i++)
                                 ftemp.b[i] = SPI1_RxBuf[1+i];
                            
                            p->SetCmd.ThermoSetting[0].fTemperature = ftemp.f32data;

                            for(i=0;i<4;i++)
                                 ftemp.b[i] = SPI1_RxBuf[5+i];
                            
                            p->SetCmd.ThermoSetting[0].fRampRate = ftemp.f32data;

                            for(i=0;i<4;i++)
                                 u32temp.b[i] = SPI1_RxBuf[9+i];
                            p->SetCmd.ThermoSetting[0].u32HoldingTime = u32temp;


                            for(i=0;i<4;i++)
                                 u32temp.b[i] = SPI1_RxBuf[13+i];
                            p->SetCmd.ThermoSetting[0].u32SnapTime = u32temp;
                       }
               break;
               
           case  RSP_CMD__CHK_STEP:

                       if(*RxIdx < LENGTH_PACKET__CHK_STEP)
                           val=ERR_LENGTH;
                       else
                       {
                           p->Cmd = RSP_CMD__CHK_STEP;
                           for(i=0;i<4;i++)
                                p->ChkCmd.u32StepNum.b[i]= SPI1_RxBuf[1+i];
                       }
               break;
               
           case  RSP_CMD__GET_TMP:
                        p->Cmd = RSP_CMD__GET_TMP;
               break;
               
               
           default:
                   val=ERR_CMD;
               break;
      }
      
      *RxIdx = 0;
      return val;
   }
 */   
   
#else
        #ifdef __RAMP_TEST
         u8    loop=0;
         u8    FirstSlop=0;
         u16   SlopParameter=0;
         float TemperatureReg=0;
         float ThermoSlopRate[10]={0};   // record the ramp rate  C/sec, should be save into flash
         float SlopRateReg=0;
         float TempStart=0;
				 float TempPerStep=0;
				 float TempStep=0;
         u32 RampTimeCnt=0;
         u32 RampFullStep=0;
         u8  RampFlag=READY;


         float TempErr[2]={0,0};        // error k, k-1
         float TempErrDiff[2]={0,0};    // error diff k,k-1

         ///////// 2nd version test
         ///////// Fuzzy PID

         
         
        

         ///////////////////////////
         
        #endif
#endif

		#ifdef __RPI_THERMO
					 
				u8 Pending=0;
				Cycle_run_Type Cycle_Lock=RUN_IDLE;	  // add @2018-0401 to check whether set step is too late
																					//               Per Leon, if too late, only reset can recover
				 
		    void Set_Cycle_Run_Status(Cycle_run_Type sts) 
			  {
						switch(sts)
						{
							case RUN_IDLE:
							case RUN_BUSY:
							case RUN_REJECT:
									Cycle_Lock=sts;
							break;
							
							default:
									Cycle_Lock=RUN_IDLE;
							break;
						}
				}
				
				Cycle_run_Type Get_Cycle_Run_status(void)
				{
						return Cycle_Lock;
				}
				
				u8 ASSERT_CYCLE_REJECT(void)
				{
						u8 val=0;
						val=(Cycle_Lock == RUN_REJECT)? 1:0;
						return val;
				}
		#endif

			 
		u8 direction =0;	
         RampCtrlType RampCtrl;

#ifdef __USE_LYSIS
    //Lysis_STS_type      Lysis_request=READY;
	  Cycle_STS_Type Lysis_request=READY;
    float   Lysis_Start_Thresh=0;
    float   Lysis_Stop_Thresh=0;
    float   Lysis_Slop=0;
    u32     Lysis_Tick=0;
    float   Lysis_temp_reg=0;
    float32_union fTempNumber;
#endif 


int main(void)
{
	u8 tick;
//	u16 adcx;
	u8 u8temp;	
//	u8 CurrentRowNum;

	u8 u8temp1,u8temp2,u8temp3;
	u16 u16temp,u16temp1;
	float f32temp1; //f32temp2;
	u8 ResCode;
	u8 TransLen;

//	u8 VideoRow;

	
		
	u8 ldecoun=0;
// u8 key,
//	u8 Key_count=0;
//	u8 test_lenth=0;
	u16 j,i=0;
	#ifdef VEDIO_DEBUG
	u8 DatChange=0;				
	#endif

#ifdef __RPI_THERMO
   
	u32_union_type u32Number;
    float32_union fTempNumber;
     u8 checkstatus=NO_ERR;
	RpiSetting.RpiSetTotalCnt=RpiSetting.RpiSetCurrentIdx=RpiSetting.RpiSetIdx=0;
	
//	RpiSetting.RpiSetFifoTop=RpiSetting.RpiSetFifoBottom=0;
//	RpiSetting.ThermoBusy=RpiSetting.ThermoNextPending=0;
	
#endif
	
#ifdef __USE_CAN_BUS
//		u8 res;
		u8 CAN_cnt_total_wait=1;	// to track how many total packet expected
		u8 CAN_idx_next=1;  // to check packet whether match expected index
		u8 CAN_Flag_wait=0;
		u8 CAN_Timer_wait=0;
		u8 mode=0; //CAN工作模式;0,普通模式;1,环回模式
		u8 CAN_cnt_index=CAN_BUF_START_INDEX;  // always start from 1, as reserve  0 for header
		
//		u8 CAN_command_byte=0;
//		u8 CAN_type_byte=0;
		u8 CAN_txbuf[8]={0};
#endif

	// add @2018-04-21 brake on accident		
	u8 senror1_too_high_cnt=0;
	u8 senror2_too_high_cnt=0;
		
  PCR_Cycle_Init();
	KL_temp.float_num=Kl[0];
	KP_temp.float_num=Kp[0];
	KI_temp.float_num=Ki[0];
	KD_temp.float_num=Kd[0];
	tick=0;
 	Stm32_Clock_Init(9);	//系统时钟设置 --72M
//	uart_init(36,9600);	 	//串口初始化为9600
	delay_init(72);	   	 	//延时初始化 
	LED_Init();		  		//初始化与LED连接的硬件接口
	LED1=1;

	#ifdef __RPI_THERMO
		IRQ_PIN =1;	
	#endif
		
  LED_CTRL=0;

	FanCtrl_Init();
/*
	OSC_Ctrl=OSC_OFF;
	OSC_mode=OSC_AUTO;
	OSC_Status=0;
	PCR_RESET_PROCESS;
*/																 
	Trim_Reset();	
    //SPI_2_Rx_sts=SPI_IDLE;
    //InteCnt= 4999;
    /*
    	KEY_Init();				//按键初始化	
    	EXTIX_Init(); 		  
    	SPI2_Initializaion();
    */
    //Adc_Init();
    IIC_Init();
    TempContorl_2_Init();
    //TempSensor_Initial();
    //TempControl_Initial(50);
    TempCtrl_Active=0;
    TempValid=0;
    //Sensor_Res_Init();

    #ifdef __RPI_THERMO  // version 1.8 : new feature, SPI slave interface
					SPI1_Slave_Initializaion();
					Timer2_Init(4,719);   // 1ms tick for SPI1 interval cnt
					checkstatus=NO_ERR;
					Timer2_update(4);
					while(SPI_Tick_Cnt == 0);
    #else
			 #ifndef __USE_CAN_BUS
					#if defined(USB_WAIT_MODE)
							EpMsgDisable();
					#endif

					usb_port_set(0); 	//USB先断开
					delay_ms(300);
						usb_port_set(1);	//USB再次连接
					//USB配置
					USB_Interrupts_Config();    
					Set_USBClock();   
					USB_Init();		

			#else
					CAN_Mode_Init(1,4,4,SCALER,mode);	
			#endif

    #endif // end of #ifndef RPI
    
		//TRG_0=TRG_1=TRG_2=TRG_3=1;
		Tick_OverShot=Time_OverShot * TICK_PER_SEC;
		// @2017-10-14 version 1.9 to support independent overshot for colding
		#ifdef __USE_TWO_OVERSHOT
			Tick_colding_OverShot=Time_colding_OverShot * TICK_PER_SEC;
		#endif

		#ifdef __RPI_THERMO
		  
			RpiSetting.ThermoSetting[0].fTemperature   = 60;
			RpiSetting.ThermoSetting[0].fRampRate      = 0;
			RpiSetting.ThermoSetting[0].u32HoldingTime = 0;
			RpiSetting.ThermoSetting[0].u32SnapTime    = 0;

			RpiSetting.RpiSetIdx =0;
			RpiSetting.RpiSetCurrentIdx=0;
			RpiSetting.RpiSetTotalCnt=0; 
			for(u16temp=0;u16temp<10000;u16temp++);
			IRQ_PIN =0;	
		#endif		

        //add @2018-04-20 to support ramp parameter in alpha
        RampCtrl.HasValidRamp=0;
        RampCtrl.RampRate=0;

	while(1)
	{
	    /*
		if(mst_flag==BRD_IN_TRG)
		{
			ResCode=0;
			TransLen=1;
			TxBuffer[0]=(0| TRG_VALID) ;
		//txc//	USBRes(ResCode, CMD_HOST_NOTIFY,TxBuffer,TransLen);
		}
         */
         
		////2 brd branch
		
		if(TempValid & 0x1)	// Temp1 Control valid
		{
			if(TempCtrl_Active==4)
			{
					TempCtrl_Active=0;
					TempValid &=~ 0x1;
					TempControl_stop();
			}else if(TempCtrl_Active==1)
						{
							TempCtrl_Active=2;
							//TempSensor_Initial(TEMP1);
							TempControl_Initial(TempSet);
						}
						else if(TempCtrl_Active!=0)
				   		{
									TempCurrent=TempSensorRead(STLM75_ADDR_WR);
									if (TempCurrent > 0)
									{
											TempCurrent_reg = TempCurrent;
											sensor_1_error_cnt=0;
										 
										  // add @2018-04-21 brake on accident
										  if(TempCurrent >= BRAKE_TEMPERATURE)
													senror1_too_high_cnt++;
											else
													senror1_too_high_cnt=0;
									}
									else
									{
											TempCurrent = TempCurrent_reg;
											TempCurrent_reg += 0.5;
											sensor_1_error_cnt += 1;							
									}
									
									// add @2018-04-21 brake on accident
									if((sensor_1_error_cnt>MAX_LIMIT) || (senror1_too_high_cnt > 10))
								  {
											Brake_assert();
									}
							}
		}
			



		if(TempValid & 0x2)	// Temp2 Control valid
		{

			if((Peltier_Swap_Msg & SENSOR_2) !=0) // channel 2 swap msg
			{
				Channel_Swap(SENSOR_2);
				PID_Clear(SENSOR2_INDEX);
			}
				
			if(TempCtrl_Active_2==4)
			{
					if(CycleSTS==READY)	  // @0607, if not in cycle mode, just stop it
																//        else dot not stop, MsgHandler will handle it 
					{
						TempCtrl_Active_2=0;
						TempValid &=~ 0x2;
						TempControl_2_stop();
					}
					#ifdef DEBUG_MSG
						msg_debug=0;
					#endif
			}else if(TempCtrl_Active_2==1)
					{
						TempCtrl_Active_2=2;
						TempControl_2_Initial(TempSet_2);

					 // new feature to cal ramp  time 
					 #ifdef __RPI_THERMO
							 /*
										if(RampFlag == WAIT)
												{
														RampFlag = ACTIVE;
														RampTimeCnt=0;
		 
												}
							 */
					 #else

								#ifdef __RAMP_TEST

                                            /*
										 RampTimeCnt = 0;
										 TemperatureReg=(TempSet_2>TempCurrent_2)? (TempSet_2-TempCurrent_2) : (TempCurrent_2 - TempSet_2);
										 direction=(TempSet_2>TempCurrent_2)? 0:1;
						
										 TempStart=TempCurrent_2;
										 if(RampFlag != ACTIVE)
										 {
												 RampFlag = ACTIVE;
												 TempPerStep = TemperatureReg / 60;                     // fixed 1min ramp time
												 //TempPerStep = TempPerStep /	TICK_PER_SEC;    
												 RampFullStep = (u32)(60 * TICK_PER_SEC);   
										 }
										 */
										 
										 
										// add @2018-04-20 to support ramp paramter setting in alpha
										 if(RampCtrl.HasValidRamp==1)
										 {
												 RampTimeCnt = 0;
												 TemperatureReg=(TempSet_2>TempCurrent_2)? (TempSet_2-TempCurrent_2) : (TempCurrent_2 - TempSet_2);
												 direction=(TempSet_2>TempCurrent_2)? 0:1;
								
												 TempStart=TempCurrent_2;
												 if(RampCtrl.RampRate !=0)
												 {
														 RampFlag = ACTIVE;
														 TempPerStep = RampCtrl.RampRate;
														 RampFullStep = (u32)(TemperatureReg/TempPerStep) * TICK_PER_SEC;   
												 }
												 else
														 RampFlag = WAIT;
                     }
                     else
													RampFlag = WAIT;
										 RampCtrl.HasValidRamp=0;

								#endif
					 #endif
					}
			
				   else if(TempCtrl_Active_2!=0)
				   		{
								TempCurrent_2=TempSensorRead(TEMP_SENSOR_2_ADDR_WR);  // new sensor
								
								if (TempCurrent_2 > 0)
								{
										TempCurrent_2_reg = TempCurrent_2;
										sensor_2_error_cnt=0;
										
									  // add @2018-04-21 brake on accident
										if(TempCurrent >= BRAKE_TEMPERATURE)
												senror2_too_high_cnt++;
										else
												senror2_too_high_cnt=0;
								}
								else
								{
										TempCurrent_2 = TempCurrent_2_reg;
										TempCurrent_2_reg += 0.5;
										sensor_2_error_cnt += 1;
								}
								
								// add @2018-04-21 brake on accident
								if((sensor_2_error_cnt>MAX_LIMIT) || (senror2_too_high_cnt > 10))
								{
										Brake_assert();
								}
						}
 
		#ifdef DEBUG_MSG
				if(msg_debug>4)
				{
					printf("%f \n",TempCurrent_2);
				}
		#endif
		}
		//}while(1);

		
#ifdef __RPI_THERMO   /// for beta system

         // handle comm time out.
		if(SPI_1_Rx_sts != SPI_IDLE)
    {   
             
		    if(SPI1_TimeOut_handler() == 1)
        {
						 SPI1_SetStatus(SET_BUSY);
						 
						 ResCode=RPI_STS_NO_ERR;
						 RpiReplyLen=0;
						 checkstatus = SPI1_cmd_packet_handler((RpiCmdType *)&RpiCmdBuf);
						 SPI_1_Rx_sts = SPI_IDLE;
						 Clear_RPI_flag();
             Pending	 =1;
					
						if(checkstatus == NO_ERR)  // copy packet into command buffer, save byte number, return packet status 
						{
								// correct command packet is received
								
							
						    switch(RpiCmdBuf.cmd)
            		{
            				case RSP_CMD__SET_STEP:
            	
                         Flag_Current_forever = 0;
                                        
													if(!ASSERT_CYCLE_REJECT())
            							{
															u8temp=0;
															if((RpiSetting.RpiSetIdx & RPI_SET_AVAILABLE_MASK) < RPI_SET_AVAILABLE_MASK)   // at least one slot is available
															{
																	u8temp=0;
                                                 
																	RpiSetting.ThermoSetting[u8temp].fTemperature   = RpiCmdBuf.SetCmd.ThermoSetting[0].fTemperature;
																	RpiSetting.ThermoSetting[u8temp].fRampRate      = RpiCmdBuf.SetCmd.ThermoSetting[0].fRampRate;
																	RpiSetting.ThermoSetting[u8temp].u32HoldingTime = RpiCmdBuf.SetCmd.ThermoSetting[0].u32HoldingTime;
																	RpiSetting.ThermoSetting[u8temp].u32SnapTime    = RpiCmdBuf.SetCmd.ThermoSetting[0].u32SnapTime;

																	RpiSetting.RpiSetIdx |= (1<<u8temp);
																	RpiSetting.RpiSetTotalCnt++;

                                  if(PCR_Cycle_Control.CycleValid == INVALID)
                                        PCR_Cycle_Control.CycleValid=VALID;

																	if((CycleSTS == READY) || (CycleSTS == FAN_EXT))
																	{
																			if(CycleSTS == FAN_EXT)
																			{
																					PCR_Cycle_Control.CycleValid=VALID;
																					CycleSTS=READY;
																					FanCtrl_Auto(0);
																			}
																			PCR_Cycle_Control.TotalCycle=1; 
																			PCR_Cycle_Control.TotalStage=1;

																			//pending ***??? here must set peltier 1 for initial heating
																			#ifdef __DEBUG_BETA_ERROR
																					TempPumpSet = TempSensorRead(STLM75_ADDR_WR);
																			#endif
																			
																			Cycle_Control_Start();
																			Set_Cycle_Run_Status(RUN_BUSY);
																	}
															}
															else
																	ResCode = RPI_STS_ERR_REJECT;
            							}
                          
            							else
													 {            
																ResCode=RPI_STS_ERR_BUSY;
													 }
													
            							for(i=0;i<4;i++)
																RpiReplyBuf[i]=(u8)(RpiSetting.RpiSetTotalCnt>>(8*i));

														RpiReplyLen=4;

                          SPI1_Res(ResCode,RpiReplyLen,RpiReplyBuf);
            						break;
            						
										case RSP_CMD__CHK_STEP:
                                    
														u32Number.u32Num = RpiCmdBuf.ChkCmd.u32StepNum.u32data;
														u8temp =0;
														if(CycleSTS == READY) 
																	ResCode=RPI_STS_ERR_REJECT;                         // exp not start
														else  if(u32Number.u32Num > RpiSetting.RpiSetTotalCnt)
																			ResCode=RPI_STS_ERR_BUSY;                    // beyond step number
																	else
																	{
																	        
                                                                                    // workaround for PI per Jun request @2018-04-03
                                                                                    // when initial heating @first setstep , always reply runing current step
                                                                                    if(PCR_Cycle_Control.CycleValid == INIT)  
                                                                                    {
                                                                                        if(u32Number.u32Num == RpiSetting.RpiSetCurrentIdx + 1)
                                                                                            {
                                                                                                RpiReplyBuf[0]=0;  
                                                                                            }
                                                                                        
                                                                                    }
                                                                                    else
                                                                                        {
            																			if(u32Number.u32Num == RpiSetting.RpiSetCurrentIdx)
            																			{  
            																					if(RpiSetting.ThermoStatus)
            																								RpiReplyBuf[0]=0;          // current runing
            																					else
            																								RpiReplyBuf[0]=1;          // current done. it's last step already
            																			}                                     // think the case RPI send later than expect
            																			else if(u32Number.u32Num == (RpiSetting.RpiSetCurrentIdx+1))
            																						{   
            																								RpiReplyBuf[0]=3;        // next to run
            																						}
            																						else if(u32Number.u32Num == (RpiSetting.RpiSetCurrentIdx-1))
            																								{   
            																										RpiReplyBuf[0]=1;  // last step
            																								}  
            																								 else 
            																								 {   
            																										 RpiReplyBuf[0]=2;  // before last
            																								 }
                                                                                        }
																	} 

																	
														//fTempNumber.float_num = RpiSetting.ThermoSetting[u8temp].fRampRate;
														for(i=0;i<4;i++)
																RpiReplyBuf[i+1]= 0;//fTempNumber.b.byte[i];
																
														u32Number.u32Num = RpiSetting.ThermoSetting[u8temp].u32HoldingTime;
														for(i=0;i<4;i++)
																RpiReplyBuf[i+5]= u32Number.b.byte[i];

														RpiReplyLen=9;					
            					
            						break;

										case RSP_CMD__GET_TMP:
														TempCurrent=TempSensorRead(STLM75_ADDR_WR);
														TempCurrent_2=TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
														fTempNumber.float_num = TempCurrent;          //TempSensorRead(STLM75_ADDR_WR);
														for(i=0;i<4;i++)
																RpiReplyBuf[i]= fTempNumber.b.byte[i];
														fTempNumber.float_num = TempCurrent_2;        //TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
														for(i=0;i<4;i++)
																RpiReplyBuf[i+4]= fTempNumber.b.byte[i];  
														RpiReplyLen=8;
//////fix Jun error, need a float for "Fan tempertature"
														for(i=0;i<4;i++)
																RpiReplyBuf[i+8]= fTempNumber.b.byte[i]; 
														RpiReplyLen=12;
 ////////////////////////////////////////////////////////////////////										
														
													
											 break;

															/// new add defined by Jun @2018-02-28
										case  RSP_CMD__Get_VERSION:   

														 /*
														 RpiReplyBuf[0]= SIZE_OF_VERSION_INFO;  
														 RpiReplyBuf[1]= VERSION_INFO_MSB;  
														 RpiReplyBuf[2]= VERSION_INFO_LSB;  
														 RpiReplyBuf[3]= YEAR_INFO;  
														 RpiReplyBuf[4]= MONTH_INFO;  
														 RpiReplyBuf[5]= DATE_INFO;  
														 */
														 
														 u8temp = sizeof(version_info_string);
														 RpiReplyBuf[0] = u8temp;
														 RpiReplyLen = u8temp+1;
														 for(i=0; i<u8temp;i++)
																	RpiReplyBuf[i+1]=version_info_string[i];
													break;
										case  RSP_CMD__INIT:

													break;
            						
										default:
													ResCode=RPI_STS_ERR_UNKNOWN;						
											break;				
            		}
					}
					else 
				 {  
						// invalid cmd received
						ResCode=RPI_STS_ERR_UNKNOWN;		    
				 }

			 //// only reply if host intend to write, instead of read only with null write
			 //// if handler reply ERR_UNDEF, means it's reading with dummy writing, no nead to reply
			 
			 if(checkstatus != ERR_UNDEF)
				{
					SPI1_SetStatus(SET_READY);
					SPI1_Res(ResCode,RpiReplyLen,RpiReplyBuf);
				}
			 else
					SPI1_SetStatus(SET_IDLE);
			    Pending	 =0;
		    }  // end of if(SPI1_TimeOut_handler() == 1)
    }// end of if(SPI_1_Rx_sts != SPI_IDLE)
			
		
		// beta step implement
		if(CycleSTS != READY)
					MsgHandler_Beta();   
		
#else  
////////////////////////////////////////////////////
// below is for alpha version / two board platform//
////////////////////////////////////////////////////
		if(CycleSTS != READY)
			MsgHandler();


         if(trg_assert==IS_ASSERT)
			trigger_idle();
		
		#ifndef __USE_CAN_BUS
		if(USB_ReceiveFlg==TRUE)
		{
			RxStage= CMPLT;
			RxIdx=USB_RxIdx;
			USB_ReceiveFlg=FALSE;
		//	USBRes(0, 1,RxBuffer, 0);
		}

		#else
		/*
				u8 CAN_rxbuf[8];
				u32 CAN_id;
				u8 CAN_ide,CAN_rtr,CAN_rx_len; 
				u8 CAN_ReceiveFlg=0;		
		*/
		if(CAN_Flag_wait)	
		{
			if(CAN_Timer_wait > 10)
			{
					CAN_Timer_wait=0;
				  CAN_Flag_wait=0;
				  CAN_cnt_index=1;
					CAN_idx_next=CAN_BUF_START_INDEX;
			}
			else
					CAN_Timer_wait++;
		}
		
	  if(CAN_ReceiveFlg)
		{
			CAN_ReceiveFlg=0;
			
			//Can_Send_Msg_by_index(CAN_rxbuf,CAN_id,8);
			
			CAN_cnt_total_wait= ((CAN_id & 0xf0)>>4);
			
			if(CAN_idx_next == (CAN_id & 0xf))  // whether match expectation
			{
					for(i=0;i<CAN_rx_len;i++)
						Comand_Buf[CAN_cnt_index++]=CAN_rxbuf[i];
				  Comand_Buf[0]=HEADER;
					if(CAN_cnt_total_wait <= (CAN_id & 0xF))   // total packet is done
					{	
							CAN_Flag_wait=0;
							CAN_Timer_wait=0;
							CAN_idx_next=1;

							RxStage= CMPLT;
							RxIdx=CAN_cnt_index;
							CAN_cnt_index=CAN_BUF_START_INDEX;
					}
					else
					{
							CAN_Flag_wait=1;	
							CAN_idx_next+=1;
						
						  CAN_txbuf[0]=NO_ERR;
						  CAN_txbuf[1]=CAN_rx_len;
						  Can_Send_Msg_by_index(CAN_txbuf,CAN_id,2); // send ACK no error
					}
			}
			else
			{
							CAN_txbuf[0]=ERR_ROW;
				      CAN_txbuf[1]=CAN_idx_next;
						  Can_Send_Msg_by_index(CAN_txbuf,CAN_id,2); // send ACK no error
			}
		}		
		
		#endif
		
		
			/*
				#if defined(PIX24_DEBUG_UART_PRINT)
						 RxStage=CMPLT;
						 RxBuffer[0]=0xaa;
						 RxBuffer[1]=CMD_GET;
						 RxBuffer[2]=0x2;
						 RxBuffer[3]=TYP_24PIXROW;
						 RxBuffer[4]=0x0;
						 RxBuffer[5]=0xb;
						 RxBuffer[6]=0x17;
						 RxBuffer[7]=0x17;
						 RxIdx=0x7;
				#endif
				*/
	  if(RxStage==CMPLT)
		{
			Command_Len=RxIdx;
			
			#ifndef __USE_CAN_BUS		
			for(i=0;i<Command_Len;i++)
				Comand_Buf[i]=RxBuffer[i];
			#endif
			
			RxStage=IDLE;
			RxIdx=0;

		#ifndef __USE_CAN_BUS		
			if(PacketChkSum(Comand_Buf, Command_Len)==TRUE)
		#endif
			{
					
				switch(Comand_Buf[CMD_BYTE_NUM])
				{
						case CMD_GET:
						
							break;
	

						case  CMD_SET:
						
										ResCode=NO_ERR;
										TxBuffer[0]= Comand_Buf[TYP_BYTE_NUM];
										TransLen=1;
					
										if(Comand_Buf[LEN_BYTE_NUM]<MIN_PACKET_LENGTH)
												ResCode=ERR_LENGTH;

										else
										{
												switch (Comand_Buf[TYP_BYTE_NUM])
												{
												
														case INTE_TIME:		//0x20
																	ResCode=NO_ERR;
																	u16temp=0;
																	u16temp = Comand_Buf[TYP_BYTE_NUM+2];
																	u16temp= ((u16temp <<8) |Comand_Buf[TYP_BYTE_NUM+1]);  // LSB byte before MSB

																	UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
																	UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
																	UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
																	UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];
														
																	if(UnionTemp.float_num!=0)
																	{
																		for(i=0;i<MAX_PCR_CH;i++)
																		PCR_Regs[i].InteTime=UnionTemp.float_num;
																		
																		trigger_time_Cal();
																	}
																	else
																		ResCode=BAD_DATA;	
															break;
					

															case LED_PRO_TIME:
																
																			ResCode=NO_ERR;
																			u16temp=0;
																			u16temp = Comand_Buf[TYP_BYTE_NUM+2];
																			u16temp= ((u16temp <<8) |Comand_Buf[TYP_BYTE_NUM+1]);  // LSB byte before MSB
																			
																			UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
																			UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
																			UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
																			UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];

																			if((UnionTemp.float_num>0)&&(UnionTemp.float_num<2000))
																				SetTm_LED_Delay=UnionTemp.float_num;
																			else
																				ResCode=OUT_RANGE;

																break;


															case LED_HOLD_TIME:
																
																				ResCode=NO_ERR;
																				u16temp=0;
																				u16temp = Comand_Buf[TYP_BYTE_NUM+2];
																				u16temp= ((u16temp <<8) |Comand_Buf[TYP_BYTE_NUM+1]);  // LSB byte before MSB
																			
																				UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
																				UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
																				UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
																				UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];

																				if((UnionTemp.float_num>0)&&(UnionTemp.float_num<2000))
																					HoldTm_LED_Delay=UnionTemp.float_num;
																				else
																					ResCode=OUT_RANGE;
																break;

														 case LED_SWITCH:
																			ResCode=NO_ERR;
																			u8temp3=Comand_Buf[TYP_BYTE_NUM+1];
																			LED_CTRL=led_mode=(u8temp3 & 0x1);
																break;	
											 
														 case TYP_PCR_TRG_MASK:    // trigger IRQ mask
																			ResCode=NO_ERR;

																			TxBuffer[0]= TYP_PCR_TRG_MASK;
																			//PcrMskReg=(Comand_Buf[TYP_BYTE_NUM+1] & 0xF);		// mask byte	
																			trigger_time_Cal();
														 break;
											 
														case TYP_VER_INFO:   // F/W information is read only
															break; 
														
														default:  			// invalid type 
																ResCode=ERR_TYP;
															break;
										 }
									 }
									#ifndef __USE_CAN_BUS	
													USBRes(ResCode, CMD_SET, TxBuffer, TransLen);
									#else
													CANRes(ResCode, CMD_SET, TxBuffer, TransLen); // send ACK no error						 
									#endif
								break;


						case CMD_READ:

										ResCode=NO_ERR;
										TxBuffer[0]= Comand_Buf[TYP_BYTE_NUM];
										TransLen=1;
										
										if(Comand_Buf[LEN_BYTE_NUM]<MIN_PACKET_LENGTH)
														ResCode=ERR_LENGTH;
										else
										{
												switch (Comand_Buf[TYP_BYTE_NUM])
												{
					
														case INTE_TIME:		//0x20
																	ResCode=NO_ERR;
																	TxBuffer[0]=INTE_TIME;

																	UnionTemp.float_num= PCR_Regs[0].InteTime;
																	TxBuffer[1]= UnionTemp.tempd.byte0;
																	TxBuffer[2]= UnionTemp.tempd.byte1;
																	TxBuffer[3]= UnionTemp.tempd.byte2;
																	TxBuffer[4]= UnionTemp.tempd.byte3;
																	TransLen=5;
														break;

														case LED_PRO_TIME:

																	ResCode=NO_ERR;
																	TxBuffer[0]=LED_PRO_TIME;
														
																	UnionTemp.float_num= SetTm_LED_Delay;
																	TxBuffer[1]= UnionTemp.tempd.byte0;
																	TxBuffer[2]= UnionTemp.tempd.byte1;
																	TxBuffer[3]= UnionTemp.tempd.byte2;
																	TxBuffer[4]= UnionTemp.tempd.byte3;
																	TransLen=5;
														break;

													case LED_HOLD_TIME:

																	ResCode=NO_ERR;
																	TxBuffer[0]=LED_HOLD_TIME;
														
																	UnionTemp.float_num= HoldTm_LED_Delay;
																	TxBuffer[1]= UnionTemp.tempd.byte0;
																	TxBuffer[2]= UnionTemp.tempd.byte1;
																	TxBuffer[3]= UnionTemp.tempd.byte2;
																	TxBuffer[4]= UnionTemp.tempd.byte3;
																	TransLen=5;
														break;
													
													 case TYP_PCR_TRG_MASK:    // trigger IRQ mask
														 
																	ResCode=NO_ERR;
																	TransLen=2;
																	TxBuffer[0]=TYP_PCR_TRG_MASK;
																	TxBuffer[1]=PcrMskReg;		// mask byte	
														break;
													
													 case TYP_VER_INFO:   // get F/W information
														 
																	ResCode=NO_ERR;
																	TransLen=7;
																	TxBuffer[0]=TYP_VER_INFO;
																	TxBuffer[1]=FUNC_TEMP_CTRL;		//  temperature control
																	TxBuffer[2]=VERSION_INFO_MSB;	
																	TxBuffer[3]=VERSION_INFO_LSB;	
																	TxBuffer[4]=YEAR_INFO;		//  
																	TxBuffer[5]=MONTH_INFO;		//  
																	TxBuffer[6]=DATE_INFO;		//  
													 
														break;
													
													default:  			// invalid type 
																ResCode=ERR_TYP;
														break;
												 }
									 }
									#ifndef __USE_CAN_BUS	
													 USBRes(ResCode, CMD_READ, TxBuffer, TransLen);
									#else
													 CANRes(ResCode, CMD_READ, TxBuffer, TransLen); // send ACK no error	
									#endif
								break;

					case CMD_MEASURE:

						break;
  					
					case CMD_TEMP_SET: // control temperature

									ResCode=NO_ERR;
									TxBuffer[0]= Comand_Buf[TYP_BYTE_NUM];
									TransLen=1;
					
								 {
									 switch (Comand_Buf[TYP_BYTE_NUM])  // modify @ 140326, add sensor number at byte 1
									 {
											case TEMP_SET_ON:	// float	

															u8temp3=Comand_Buf[TYP_BYTE_NUM+1];	 // #sensor 
															UnionTemp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1+1];
															UnionTemp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2+1];
															UnionTemp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3+1];
															UnionTemp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4+1];

															f32temp1=UnionTemp.float_num;
															if((f32temp1>0)&& (f32temp1<150))
															{
					
																	u8temp1 = Comand_Buf[TYP_BYTE_NUM+5+1];
																	u8temp2 = Comand_Buf[TYP_BYTE_NUM+6+1];
																	switch (u8temp3)
																	{
																			/*
																			case SENSOR_1:
																					 TempCtrlTime_Sec= u8temp1;  // xxxx second @ 4,5 byte
																					TempCtrlTime_Sec<<=8;
																					TempCtrlTime_Sec|=u8temp2;
																					TempTickLength=  TempCtrlTime_Sec * TICK_PER_SEC;
																					TempSet= f32temp1;
																					TempCtrl_Active=1;
																					TempValid |= 1;
																				 TxBuffer[0]= TEMP_SET_ON;//txc
																			 
																				break;
																				*/
																			case SENSOR_2:
																					TempCtrlTime_Sec_2= u8temp1;  // xxxx second @ 4,5 byte
																					TempCtrlTime_Sec_2<<=8;
																					TempCtrlTime_Sec_2|=u8temp2;
																					TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC;
																					TempSet_2= f32temp1;
																					TempCtrl_Active_2=1;
																					TempValid |= 2;									      
																				break;

																			case SENSOR_1:        //SENSOR_PUMP:  // set the pump temperature for cycle control//txc
																					TempPumpSet= f32temp1;	
																				break;

																			default:
																					ResCode=BAD_DATA;
																				break;
																	}

															}
															else
															{
																	ResCode=BAD_DATA;
															}

												break;

											case TEMP_SET_OFF:	 		// modify @ 140326, add sensor number at byte 1
													u8temp3=Comand_Buf[TYP_BYTE_NUM+1];	 // #sensor 
													ResCode=NO_ERR;
													switch (u8temp3)
													{
															case SENSOR_1:
																		TempCtrl_Active=0;
																		TempValid &= ~0x1;
																		TempControl_stop();
																
																		TempCtrl_Active_2=0;
																		TempValid &= ~0x2;
																		TempControl_2_stop();
																		TempCurrent_2_reg=TempCurrent_reg=0;
																		
																break;
															case SENSOR_2:
														
																		TempCtrl_Active=0;
																		TempValid &= ~0x1;
																		TempControl_stop();
																
																		TempCtrl_Active_2=0;
																		TempValid &= ~0x2;
																		TempControl_2_stop();
																		trigger_reset();
																		TempCurrent_2_reg=TempCurrent_reg=0;
																break;
															
															default:
																		ResCode=BAD_DATA;
																break;
													}
													break;

												case TEMP_READ_SENSOR:   // modify @ 140326, add sensor number at byte 1
																u8temp3=Comand_Buf[TYP_BYTE_NUM+1];	 // #sensor 
																ResCode=NO_ERR;
																TransLen=5;
										
																switch (u8temp3)
																{
																		case SENSOR_1:
																				UnionTemp.float_num= TempSensorRead(STLM75_ADDR_WR);
																			break;
																		case SENSOR_2:
																				UnionTemp.float_num= TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
																			break;
																		default:
																				ResCode=BAD_DATA;
																				TxBuffer[0]= TEMP_READ_SENSOR;
																				TransLen=1;
																			break;
																}
																if(ResCode==NO_ERR)
																{
												
																	TxBuffer[0]= TEMP_READ_SENSOR;//txc//u8temp3;
																	TxBuffer[1]= UnionTemp.tempd.byte0;
																	TxBuffer[2]= UnionTemp.tempd.byte1;
																	TxBuffer[3]= UnionTemp.tempd.byte2;
																	TxBuffer[4]= UnionTemp.tempd.byte3;
																	
																	
																	TxBuffer[5]= u8temp3;
																	TransLen++;
																}											
															break;

												case TYP_FAN_CTRL:   // fan control, feature @0512, output on PC5

																u8temp3=Comand_Buf[TYP_BYTE_NUM+1];
																if(u8temp3 & FAN_MODE_MSK)
																		FanMode_Clear();
																else
																		FanCtrl_Force((u8temp3 & FAN_CTRL_MSK));									
													break;

											 case TYP_FAN_READ:
																TxBuffer[0]= TYP_FAN_READ;
																TxBuffer[1]= FanCSR_Read();
																TransLen=2;
													break;
										
											 case TYP_FAN_GAP_SET:
																ResCode=NO_ERR;
																UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
																UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
																UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
																UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];		


																if((UnionTemp.float_num>=0)&&(UnionTemp.float_num<TEMP_GAP_SWAP))
																	Fan_Gap_temp=UnionTemp.float_num;
																else
																	ResCode=OUT_RANGE;

																TxBuffer[0]= TYP_FAN_GAP_SET;
																TransLen=1;									 
												break;
										 
												case TYP_FAN_GAP_READ:
																ResCode=NO_ERR;
																TransLen=5;
																UnionTemp.float_num=Fan_Gap_temp;
																TxBuffer[0]= TYP_FAN_GAP_READ;//txc//u8temp3;
																TxBuffer[1]= UnionTemp.tempd.byte0;
																TxBuffer[2]= UnionTemp.tempd.byte1;
																TxBuffer[3]= UnionTemp.tempd.byte2;
																TxBuffer[4]= UnionTemp.tempd.byte3;
																											
													break;

											
												default:
																ResCode=ERR_TYP;
													break;
										}
									}
									#ifndef __USE_CAN_BUS	
													USBRes(ResCode, CMD_TEMP_SET, TxBuffer, TransLen);
									#else
													CANRes(ResCode, CMD_TEMP_SET, TxBuffer, TransLen); // send ACK no error	
									#endif
									break;					

					case  CMD_PID_CFG: // modify @2014-12-03
									   // Comand_Buf[TYP_BYTE_NUM]	will insert segment index at bit7..bit4
									   // bit3..bit0 still be type
										ResCode=NO_ERR;
										TransLen=1;
										TxBuffer[0]= Comand_Buf[TYP_BYTE_NUM];
					
										u8temp3= (Comand_Buf[TYP_BYTE_NUM]>>4);
										if (u8temp3>=PID_SLOP_SEG_MAX)
												ResCode=OUT_RANGE;	
										else
										{
											u8temp2=(Comand_Buf[TYP_BYTE_NUM] & 0xf);
											u8temp1=Comand_Buf[LEN_BYTE_NUM];
											
											switch(u8temp2)
											{
												case TYP_PID_KTM:
																if(u8temp3 !=0)
																{
																	ResCode=OUT_RANGE; // just only support one point now.
																}
																else
																{
																	if(u8temp1 < 4)
																		ResCode=ERR_LENGTH;
																	else
																	{
																		KP_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1];
																		KP_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2];
																		KP_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3];
																		KP_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4];
																		//Ktm[u8temp3] = KP_temp.float_num;
																		Ktm[0] =Ktm[1] = KP_temp.float_num;
																	}
																}
													break;

												case TYP_PID_KP:
														if(u8temp1 < 4)
															ResCode=ERR_LENGTH;
														else
														{
															KP_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1];
															KP_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2];
															KP_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3];
															KP_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4];
															Kp[u8temp3] = KP_temp.float_num;
														}
														break;
				
												case TYP_PID_KI:
														if(u8temp1 < 4)
															ResCode=ERR_LENGTH;
														else
														{
															KI_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1];
															KI_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2];
															KI_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3];
															KI_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4];
															Ki[u8temp3] = KI_temp.float_num;
														}
														break;
				
				
												case TYP_PID_KD:
														if(u8temp1 < 4)
															ResCode=ERR_LENGTH;
														else
														{
															KD_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1];
															KD_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2];
															KD_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3];
															KD_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4];
															Kd[u8temp3] = KD_temp.float_num;
														}
														break;
				
				
												case TYP_PID_KP_KI:
														if(u8temp1 < 8)
															ResCode=ERR_LENGTH;
														else
														{
															KP_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1];
															KP_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2];
															KP_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3];
															KP_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4];
															Kp[u8temp3] = KP_temp.float_num;
				
															KI_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+5];
															KI_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+6];
															KI_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+7];
															KI_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+8];
															Ki[u8temp3] = KI_temp.float_num;
				
														}
														break;
				
				
												case TYP_PID_KP_KI_KD:
														if(u8temp1 < 12)
															ResCode=ERR_LENGTH;
														else
														{
															KP_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1];
															KP_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2];
															KP_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3];
															KP_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4];
															Kp[u8temp3] = KP_temp.float_num;
				
															KI_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+5];
															KI_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+6];
															KI_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+7];
															KI_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+8];
															Ki[u8temp3] = KI_temp.float_num;
				
															KD_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+9];
															KD_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+10];
															KD_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+11];
															KD_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+12];
															Kd[u8temp3] = KD_temp.float_num;
				
														}
														break;
				
												case TYP_PID_KL:
															if(u8temp1 < 4)
																ResCode=ERR_LENGTH;
															else
															{
																KL_temp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1];
																KL_temp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2];
																KL_temp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3];
																KL_temp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4];
																Kl[u8temp3] = KL_temp.float_num;
															}
														break;
												default:
														ResCode=ERR_TYP;
													break;
											}
										}
										/*
																	if(USB_ReceiveFlg==TRUE)
																	{
																			TxBuffer[0]= CMD_PID_CFG;									
																			TransLen=1;
																			USB_ReceiveFlg=FALSE;
																		
										#ifndef __USE_CAN_BUS	
																			USBRes(ResCode, CMD_PID_CFG, TxBuffer, TransLen);
										#endif
																	}
																	else
																	{

																		TxBuffer[0]= CMD_PID_CFG;									
																		TransLen=1;
										*/							
										#ifndef __USE_CAN_BUS	
													USBRes(ResCode, CMD_PID_CFG, TxBuffer, TransLen);
										#else
													CANRes(ResCode, CMD_PID_CFG, TxBuffer, TransLen); // send ACK no error	
										#endif
						break; 
					
					case  CMD_PID_READ:
					/*
						  	TransLen=12;
							ResCode=NO_ERR;
							TxBuffer[0]=KP_temp.tempd.byte0;
							TxBuffer[1]=KP_temp.tempd.byte1;
							TxBuffer[2]=KP_temp.tempd.byte2;
							TxBuffer[3]=KP_temp.tempd.byte3;

							TxBuffer[4]=KI_temp.tempd.byte0;
							TxBuffer[5]=KI_temp.tempd.byte1;
							TxBuffer[6]=KI_temp.tempd.byte2;
							TxBuffer[7]=KI_temp.tempd.byte3;

							TxBuffer[8]=KL_temp.tempd.byte0;
							TxBuffer[9]=KL_temp.tempd.byte1;
							TxBuffer[10]=KL_temp.tempd.byte2;
							TxBuffer[11]=KL_temp.tempd.byte3;
							USBRes(ResCode, CMD_PID_READ, TxBuffer, TransLen);
					*/
							ResCode=NO_ERR;
              TxBuffer[0] = Comand_Buf[TYP_BYTE_NUM];
							TxBuffer[1]=PID_SLOP_SEG_MAX;
							KP_temp.float_num=Ktm[0];
 							TxBuffer[2]=KP_temp.tempd.byte0;
							TxBuffer[3]=KP_temp.tempd.byte1;
							TxBuffer[4]=KP_temp.tempd.byte2;
							TxBuffer[5]=KP_temp.tempd.byte3;

							j=6;
							for(i=0;i<PID_SLOP_SEG_MAX;i++)
							{
							
							KP_temp.float_num=Kp[i];
							TxBuffer[j++]=KP_temp.tempd.byte0;
							TxBuffer[j++]=KP_temp.tempd.byte1;
							TxBuffer[j++]=KP_temp.tempd.byte2;
							TxBuffer[j++]=KP_temp.tempd.byte3;

 							KP_temp.float_num=Ki[i];
							TxBuffer[j++]=KP_temp.tempd.byte0;
							TxBuffer[j++]=KP_temp.tempd.byte1;
							TxBuffer[j++]=KP_temp.tempd.byte2;
							TxBuffer[j++]=KP_temp.tempd.byte3;

 							KP_temp.float_num=Kd[i];
							TxBuffer[j++]=KP_temp.tempd.byte0;
							TxBuffer[j++]=KP_temp.tempd.byte1;
							TxBuffer[j++]=KP_temp.tempd.byte2;
							TxBuffer[j++]=KP_temp.tempd.byte3;

							KP_temp.float_num=Kl[i];
							TxBuffer[j++]=KP_temp.tempd.byte0;
							TxBuffer[j++]=KP_temp.tempd.byte1;
							TxBuffer[j++]=KP_temp.tempd.byte2;
							TxBuffer[j++]=KP_temp.tempd.byte3;
							}
							TransLen=j+1;
							
							#ifndef __USE_CAN_BUS	
											USBRes(ResCode, CMD_PID_READ, TxBuffer, TransLen);
							#else
											CANRes(ResCode, CMD_PID_READ, TxBuffer, TransLen); // send ACK no error					
							#endif

						break;


					////////////////// 2014-05-20 //////////////
											   
					case CMD_CYCLE_CTRL:
						
									u8temp2=Comand_Buf[TYP_BYTE_NUM];
									u8temp1=Comand_Buf[LEN_BYTE_NUM];

									ResCode=NO_ERR;
									TxBuffer[0]= u8temp2;
									TransLen=1;
							
									switch(u8temp2)
									{
												case TYP_CYCLE_LOAD:
									
													if((Comand_Buf[SECT_BYTE]<1) || (Comand_Buf[STAGE_BYTE]<1))
														ResCode=OUT_RANGE;
													else
													{
														//Buffer_Cycle_Control.TotalSect=Comand_Buf[SECT_BYTE]-1;

                                                             
                                                        
														u8temp=Comand_Buf[STAGE_BYTE] & 0xF;

                                                         #ifdef __HAS_CONFIGURABLE_SNAP    // if flexible configue
                                                              u8temp3=Comand_Buf[STAGE_BYTE] >>4;

                                                              if(u8temp3 > 0)   // has setting
                                                              {
                                                                 if(u8temp3 > u8temp)  
                                                                    u8temp3 =0;                                                                    
                                                              }
                                                              
                                                         #endif

                                                        
														Buffer_Cycle_Control.TotalStage= u8temp-1;
													
														for(i=0;i<u8temp;i++)	
														{
																u8temp1=i*BYTE_PER_STAGE;
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[SET_START_BYTE+u8temp1];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[SET_START_BYTE+u8temp1+1];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[SET_START_BYTE+u8temp1+2];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[SET_START_BYTE+u8temp1+3];
								
																Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[SET_START_BYTE+u8temp1+4];
																Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[SET_START_BYTE+u8temp1+5];
																															
																// add @2018-04-19 support ramp data in structure
																// clear ramp flag in case previouse load valid data
																Buffer_Cycle_SetPoint[i].HasValidRampdata=0;
							
														}
														if(SetPoint_Check(u8temp,Buffer_Cycle_SetPoint)!= VALID)
																ResCode=OUT_RANGE;	
														else
														{							
																PCR_Cycle_Control.CycleValid=INVALID;
																TempControl_2_stop();		
																PID_Clear(SENSOR2_INDEX);
																SetPoint_Copy(u8temp,Buffer_Cycle_SetPoint, PCR_Cycle_SetPoint);
																//PCR_Cycle_Control.TotalSect= Buffer_Cycle_Control.TatalSect;
																PCR_Cycle_Control.TotalStage= Buffer_Cycle_Control.TotalStage;
																PCR_Cycle_Control.CycleValid=VALID;
																//SetPoint_Cycle_Start(PCR_Cycle_SetPoint);

                                                                      
                                                                    #ifdef __HAS_CONFIGURABLE_SNAP    // if flexible configue
                                                                          
                                                                          if(u8temp3 > 0)   // has setting
                                                                          {
                                                                             PCR_Cycle_Control.HasSnapCfg = 1;
                                                                             PCR_Cycle_SetPoint[u8temp3-1].SnapValid = 1;
                                                                          }
                                                                          else
                                                                          {
                                                                             PCR_Cycle_Control.HasSnapCfg = 0;
                                                                             for(i=0;i<u8temp;i++)
                                                                                PCR_Cycle_SetPoint[i].SnapValid = 0;
                                                                          } 
                                                                          
                                                                    #endif
                                                                
														}
														TxBuffer[0]= Comand_Buf[TYP_BYTE_NUM]; // 2017-03-23 fix Fan echo overwrite TxBuf[0]
													}
												break;


												case TYP_PE_LOAD: // pump, ext, start inforamtion

														ResCode=NO_ERR;
														for(i=0;i<2;i++)
														{
																u8temp1=i*BYTE_PER_STAGE;  
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[SET_START_BYTE+u8temp1];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[SET_START_BYTE+u8temp1+1];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[SET_START_BYTE+u8temp1+2];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[SET_START_BYTE+u8temp1+3];
								
																Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[SET_START_BYTE+u8temp1+4];
																Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[SET_START_BYTE+u8temp1+5];
																// add @2018-04-19 support ramp data in structure
																// clear ramp flag in case previouse load valid data
																Buffer_Cycle_SetPoint[i].HasValidRampdata=0;
														}

														if(SetPoint_Check(2,Buffer_Cycle_SetPoint)!= VALID)
																ResCode=OUT_RANGE;	
														else
														{
																SetPoint_Copy(2,Buffer_Cycle_SetPoint, PE_Cycle_SetPoint);
																		 
																if(Comand_Buf[CYCLE_SET_BYTE]==1) // comand to start cycle
																{
																		
																		if(PCR_Cycle_Control.CycleValid==VALID)
																		{
																			if(CycleSTS != READY)
																			{
																					TempCtrl_Active=TempCtrl_Active_2=0;
																					TempValid =0;
																					TempControl_stop();
																					TempControl_2_stop();
																					CycleSTS=READY;	
																					TempCurrent_2_reg=TempCurrent_reg=0;
																			}
																			PCR_Cycle_Control.TotalCycle=Comand_Buf[CYCLE_NUM_BYTE]-1;  // @0607, new cmd
																			Cycle_Control_Start();
																		}
																		else
																			ResCode=ERR_UNDEF;	// @0607 user did not define data				
																}
																//// v1.8 @20170710 bug fix to stop cycle immediately 
																else
																{
																		TempCtrl_Active=TempCtrl_Active_2=0;
																		TempValid =0;
																		TempControl_stop();
																		TempControl_2_stop();
																		CycleSTS=READY;		
																}
														}
															
													break;
												 
												
												case TYP_OVERSHOT_TIM:
													
															ResCode=NO_ERR;
															UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
															UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
															UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
															UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];	
													
															if(UnionTemp.float_num>=0)
															{
																Time_OverShot=UnionTemp.float_num;
																Tick_OverShot=Time_OverShot * TICK_PER_SEC;
															}
															else
																ResCode=OUT_RANGE;										

													break;
													
												case TYP_OVERSHOT_TEM:
													
															ResCode=NO_ERR;
															UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
															UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
															UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
															UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];	
													
															if((UnionTemp.float_num>=0)&&(UnionTemp.float_num<TEMP_GAP_SWAP))
																Temper_OverShot=UnionTemp.float_num;
															else
																ResCode=OUT_RANGE;										
										
													break;


								// @2017-10-14 version 1.9 to support independent overshot for colding
								#ifdef __USE_TWO_OVERSHOT
															
												case TYP_OVERSHOT_TWO:    // |byte number | float time | float temp | float time | float temp |
													    i=Comand_Buf[TYP_BYTE_NUM+1]; // reserve for multi-overshot
															ResCode=NO_ERR;
												
															UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+2];
															UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+3];
															UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+4];
															UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+5];	

															if(UnionTemp.float_num>0)
															{
																Time_OverShot=UnionTemp.float_num;
																Tick_OverShot=Time_OverShot * TICK_PER_SEC;
															}
															else
															{
																ResCode=OUT_RANGE;
																break;
															}
															
															UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+6];
															UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+7];
															UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+8];
															UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+9];	
															if((UnionTemp.float_num>=0)&&(UnionTemp.float_num<TEMP_GAP_SWAP))
																Temper_OverShot=UnionTemp.float_num;
															else
															{
																ResCode=OUT_RANGE;
																break;
															}

															UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+10];
															UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+11];
															UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+12];
															UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+13];	

															if(UnionTemp.float_num>0)
															{
																Time_colding_OverShot=UnionTemp.float_num;
																Tick_colding_OverShot=Time_colding_OverShot * TICK_PER_SEC;
															}
															else
															{
																ResCode=OUT_RANGE;
																break;
															}	
															
															UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+14];
															UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+15];
															UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+16];
															UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+17];	
															if((UnionTemp.float_num>=0)&&(UnionTemp.float_num<TEMP_GAP_SWAP))
																Temper_colding_OverShot=UnionTemp.float_num;
															else
																ResCode=OUT_RANGE;
														
													break;

								#endif

															
											case TYP_PWM_LIMIT:
														
													break;


											case TYP_CYCLE_RAMP_LOAD:  //add @2018-04-19
													
														u8temp2=Comand_Buf[TYP_BYTE_NUM];
														u8temp1=Comand_Buf[LEN_BYTE_NUM];
														
														ResCode=NO_ERR;
														TxBuffer[0]= u8temp2;
														TransLen=1;

														if((Comand_Buf[SECT_BYTE]<1) || (Comand_Buf[STAGE_BYTE]<1))
																ResCode=OUT_RANGE;
														else
														{
																//Buffer_Cycle_Control.TotalSect=Comand_Buf[SECT_BYTE]-1;
																u8temp=Comand_Buf[STAGE_BYTE];
																Buffer_Cycle_Control.TotalStage= u8temp-1;
														
																for(i=0;i<u8temp;i++)   
																{
																		u8temp1=i*BYTE_PER_STAGE_WITH_RAMP;
																		Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[SET_START_BYTE+u8temp1];
																		Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[SET_START_BYTE+u8temp1+1];
																		Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[SET_START_BYTE+u8temp1+2];
																		Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[SET_START_BYTE+u8temp1+3];
						
																		Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[SET_START_BYTE+u8temp1+4];
																		Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[SET_START_BYTE+u8temp1+5];

																		for(j=0;j<4;j++)
																				Buffer_Cycle_SetPoint[i].SetRamp_Union.RampTempPerSec.byte[j]=Comand_Buf[SET_START_BYTE+u8temp1+6+j];
																												
																}
																if(SetPoint_Check(u8temp,Buffer_Cycle_SetPoint)!= VALID)
																{
																		ResCode=OUT_RANGE; 

																		for(i=0;i<u8temp;i++) 
																				Buffer_Cycle_SetPoint[i].HasValidRampdata=0;
																}
																else
																{
																		for(i=0;i<u8temp;i++) 
																				Buffer_Cycle_SetPoint[i].HasValidRampdata=1;
																		PCR_Cycle_Control.CycleValid=INVALID;
																		TempControl_2_stop();       
																		PID_Clear(SENSOR2_INDEX);
																		SetPoint_Copy(u8temp,Buffer_Cycle_SetPoint, PCR_Cycle_SetPoint);
																		//PCR_Cycle_Control.TotalSect= Buffer_Cycle_Control.TatalSect;
																		PCR_Cycle_Control.TotalStage= Buffer_Cycle_Control.TotalStage;
																		PCR_Cycle_Control.CycleValid=VALID;
																		//SetPoint_Cycle_Start(PCR_Cycle_SetPoint);
																}
																TxBuffer[0]= Comand_Buf[TYP_BYTE_NUM]; // 2017-03-23 fix Fan echo overwrite TxBuf[0]
														}

													break;


											case TYP_PE_RAMP_LOAD:
											
														ResCode=NO_ERR;
														for(i=0;i<2;i++)
														{
																u8temp1=i*BYTE_PER_STAGE_WITH_RAMP;  
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[SET_START_BYTE+u8temp1];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[SET_START_BYTE+u8temp1+1];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[SET_START_BYTE+u8temp1+2];
																Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[SET_START_BYTE+u8temp1+3];
														
																Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[SET_START_BYTE+u8temp1+4];
																Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[SET_START_BYTE+u8temp1+5];

																for(j=0;j<4;j++)
																				Buffer_Cycle_SetPoint[i].SetRamp_Union.RampTempPerSec.byte[j]=Comand_Buf[SET_START_BYTE+u8temp1+6+j];
														}
																			
														if(SetPoint_Check(2,Buffer_Cycle_SetPoint)!= VALID)
														{
																ResCode=OUT_RANGE; 
																for(i=0;i<2;i++) 
																			Buffer_Cycle_SetPoint[i].HasValidRampdata=0;
														}
														else
														{
																for(i=0;i<2;i++) 
																			Buffer_Cycle_SetPoint[i].HasValidRampdata=1;
																SetPoint_Copy(2,Buffer_Cycle_SetPoint, PE_Cycle_SetPoint);
																						 
																if(Comand_Buf[CYCLE_SET_BYTE]==1) // comand to start cycle
																{
																				
																		if(PCR_Cycle_Control.CycleValid==VALID)
																		{
																				if(CycleSTS != READY)
																				{
																						TempCtrl_Active=TempCtrl_Active_2=0;
																						TempValid =0;
																						TempControl_stop();
																						TempControl_2_stop();
																						CycleSTS=READY; 
																						TempCurrent_2_reg=TempCurrent_reg=0;
																				}
																				PCR_Cycle_Control.TotalCycle=Comand_Buf[CYCLE_NUM_BYTE]-1;  // @0607, new cmd
																				Cycle_Control_Start();
																		}
																		else
																				ResCode=ERR_UNDEF;  // @0607 user did not define data               
																}
																//// v1.8 @20170710 bug fix to stop cycle immediately 
																else
																{
																		TempCtrl_Active=TempCtrl_Active_2=0;
																		TempValid =0;
																		TempControl_stop();
																		TempControl_2_stop();
																		CycleSTS=READY;     
																}
														}
																					

												break;

                                               #ifdef __USE_LYSIS
                                               
                                                    case TYP_LYSIS_LOAD:    // format :  |start (u8) |T0 (float)| T1 (float) | T/sec (float)|

                                                            ResCode=NO_ERR;
                                                            u8temp1=Comand_Buf[TYP_BYTE_NUM+1];   // start byte

                                                            if(u8temp1 & 0xF)
                                                            {
                                                            
                                                                for(i=0;i<4;i++)
    														    fTempNumber.b.byte[i] = Comand_Buf[TYP_BYTE_NUM+2+i];
                                                                Lysis_Start_Thresh = fTempNumber.float_num;

                                                                for(i=0;i<4;i++)
    														    fTempNumber.b.byte[i] = Comand_Buf[TYP_BYTE_NUM+6+i];
                                                                Lysis_Stop_Thresh = fTempNumber.float_num;

                                                                for(i=0;i<4;i++)
    														    fTempNumber.b.byte[i] = Comand_Buf[TYP_BYTE_NUM+10+i];
                                                                Lysis_Slop = fTempNumber.float_num;


                                                                if((Lysis_Start_Thresh >= LYSIS_FLOOR) && (Lysis_Stop_Thresh <= LYSIS_CEILING))
                                                                {
                                                                    Lysis_request = WAIT;
                                                                      // if cycle control is not valid, means only run Lysis
                                                                    if((CycleSTS == READY) || (CycleSTS == FAN_EXT))
                                                                    {
                                                                        Cycle_Control_Start();
                                                                        CycleSTS = LYSIS_READY;
                                                                    
                                                                    }
                                                                }
                                                                else
                                                                    ResCode=OUT_RANGE;
                                                                /// note: currently ignore the slop, control by system automatically
                                                            }
                                                            else
                                                            {
                                                                Lysis_request = READY;

                                                            }
                                                            


                                                        break;

                                               #endif
                                                                        
											default:
															ResCode=ERR_TYP;	
												break;
										}	

										#ifndef __USE_CAN_BUS								
														USBRes(ResCode, CMD_CYCLE_CTRL, TxBuffer, TransLen);
										#else
														CANRes(ResCode, CMD_CYCLE_CTRL, TxBuffer, TransLen); // send ACK no error					
										#endif								 
								break;

						/////////////////////////////////////////
						case CMD_CYCLE_CTRL_READ:   // add @ 2014-11-30, read out cycle number

									ResCode=NO_ERR;
									TxBuffer[0]=u8temp2=Comand_Buf[TYP_BYTE_NUM];										
									TransLen=1;
						
									switch(u8temp2)
									{
											case TYP_CYCLE_READ:
														TxBuffer[0]=TYP_CYCLE_READ;
														TxBuffer[1]=Cycle_Check();  //current cycle
														TxBuffer[2]=Stage_Check();  // current stage
														TransLen=3;
												break;

											case TYP_PWM_READ:
														TxBuffer[0]=TYP_PWM_READ;
														TxBuffer[1]=TempValid;  //sensor 1,2 valid
														if(Control_2_Dir==NEG_DIR)
														{
															u8temp=1;
															u16temp1=TIM8->CCR1;
														}
														else
														{
															u8temp=0;
																u16temp1=TIM1->CCR1;
														}
							
														TxBuffer[2]=((TempCtrl_Active<<4) | (u8temp<<3)|(TempCtrl_Active_2)); // active status
														u16temp= TIM4->CCR3;
														TxBuffer[3]= (u8)(u16temp>>8);	  //PWM
														TxBuffer[4]= (u8)u16temp;
														TxBuffer[5]= (u8)(u16temp1>>8);
														TxBuffer[6]= (u8)u16temp1;
														TransLen=7;
												break;
												
											case TYP_OVERSHOT_TIM:
														ResCode=NO_ERR;
														UnionTemp.float_num=Time_OverShot;
														TxBuffer[0]=TYP_OVERSHOT_TIM;
														TxBuffer[1]=UnionTemp.tempd.byte0;
														TxBuffer[2]=UnionTemp.tempd.byte1;
														TxBuffer[3]=UnionTemp.tempd.byte2;
														TxBuffer[4]=UnionTemp.tempd.byte3;
														TransLen=5;
												break;
												
											case TYP_OVERSHOT_TEM:
														ResCode=NO_ERR;
														UnionTemp.float_num=Temper_OverShot;
														TxBuffer[0]=TYP_OVERSHOT_TEM;
														TxBuffer[1]=UnionTemp.tempd.byte0;
														TxBuffer[2]=UnionTemp.tempd.byte1;
														TxBuffer[3]=UnionTemp.tempd.byte2;
														TxBuffer[4]=UnionTemp.tempd.byte3;
														TransLen=5;									
												break;
												
											case CYCLE_STATE_POLL:
														ResCode=NO_ERR;
														TxBuffer[0]=CYCLE_STATE_POLL;
														//TxBuffer[1]= (CycleSTS==READY)? 0:1; //(TempCtrl_Active_2==0)? 0:1;
													   //// v1.8 @20170710 send state, instead of idle ot busy
										        TxBuffer[1]= CycleSTS;
														TransLen=2;										
												break;										
			
											case TYP_PWM_LIMIT:
													
												break;

                                                     			#ifdef __USE_LYSIS
																				   
											case TYP_LYSIS_LOAD:	// format :  |start (u8) |T0 (float)| T1 (float) | T/sec (float)|

														ResCode=NO_ERR;
														
														TxBuffer[0]=TYP_LYSIS_LOAD;
														TxBuffer[1]=Lysis_request;

														UnionTemp.float_num=Lysis_Start_Thresh;
														TxBuffer[2]=UnionTemp.tempd.byte0;
														TxBuffer[3]=UnionTemp.tempd.byte1;
														TxBuffer[4]=UnionTemp.tempd.byte2;
														TxBuffer[5]=UnionTemp.tempd.byte3;

														UnionTemp.float_num=Lysis_Stop_Thresh;
														TxBuffer[6]=UnionTemp.tempd.byte0;
														TxBuffer[7]=UnionTemp.tempd.byte1;
														TxBuffer[8]=UnionTemp.tempd.byte2;
														TxBuffer[9]=UnionTemp.tempd.byte3;

														UnionTemp.float_num=Lysis_Slop;
														TxBuffer[10]=UnionTemp.tempd.byte0;
														TxBuffer[11]=UnionTemp.tempd.byte1;
														TxBuffer[12]=UnionTemp.tempd.byte2;
														TxBuffer[13]=UnionTemp.tempd.byte3;
														TransLen=14;		
												
												break;

										#endif
											
									// @2017-10-14 version 1.9 to support independent overshot for colding
									#ifdef __USE_TWO_OVERSHOT
											case TYP_OVERSHOT_TWO: 
														ResCode=NO_ERR;
											
														TxBuffer[0]=TYP_OVERSHOT_TEM;
											      TxBuffer[1]= 2;
											
											      UnionTemp.float_num=Time_OverShot;
														TxBuffer[2]=UnionTemp.tempd.byte0;
														TxBuffer[3]=UnionTemp.tempd.byte1;
														TxBuffer[4]=UnionTemp.tempd.byte2;
														TxBuffer[5]=UnionTemp.tempd.byte3;
											
											      UnionTemp.float_num=Temper_OverShot;
														TxBuffer[6]=UnionTemp.tempd.byte0;
														TxBuffer[7]=UnionTemp.tempd.byte1;
														TxBuffer[8]=UnionTemp.tempd.byte2;
														TxBuffer[9]=UnionTemp.tempd.byte3;

														UnionTemp.float_num=Time_colding_OverShot;
														TxBuffer[10]=UnionTemp.tempd.byte0;
														TxBuffer[11]=UnionTemp.tempd.byte1;
														TxBuffer[12]=UnionTemp.tempd.byte2;
														TxBuffer[13]=UnionTemp.tempd.byte3;
											
											      UnionTemp.float_num=Temper_colding_OverShot;
														TxBuffer[14]=UnionTemp.tempd.byte0;
														TxBuffer[15]=UnionTemp.tempd.byte1;
														TxBuffer[16]=UnionTemp.tempd.byte2;
														TxBuffer[17]=UnionTemp.tempd.byte3;
											
														TransLen=18;														
											
											break;
									#endif												
										default:
														ResCode=ERR_TYP;
											break;
									}
									#ifndef __USE_CAN_BUS								
																USBRes(ResCode, CMD_CYCLE_CTRL_READ, TxBuffer, TransLen);
									#else
																CANRes(ResCode, CMD_CYCLE_CTRL_READ, TxBuffer, TransLen); // send ACK no error					
									#endif	
						break;
 /*
					case CMD_PWM_READ:  //add @2014-11-30 read PWM, peltier status, direction
 						  	TransLen=6;

							ResCode=NO_ERR;
							TxBuffer[0]=TempValid;  //sensor 1,2 valid
							if(Control_2_Dir==NEG_DIR)
							{
								u8temp=1;
								u16temp1=TIM8->CCR1;
							}
							else
							{
								u8temp=0;
							    u16temp1=TIM1->CCR1;
							}

							TxBuffer[1]=((TempCtrl_Active<<4) | (u8temp<<3)|(TempCtrl_Active_2)); // active status
							u16temp= TIM4->CCR3;
							TxBuffer[2]= (u8)(u16temp>>8);	  //PWM
							TxBuffer[3]= (u8)u16temp;
							TxBuffer[4]= (u8)(u16temp1>>8);
							TxBuffer[5]= (u8)u16temp1;
							USBRes(ResCode, CMD_PID_READ, TxBuffer, TransLen);
						break;
 */
#ifdef __USE_USB_BUS
#ifdef __USE_RAM_DEBUG
 
					case CMD_DEBUG:
						
							ResCode=NO_ERR;
							TxBuffer[0]=u8temp2=Comand_Buf[TYP_BYTE_NUM];
					
							switch(u8temp2)
							{
								case TYP_RAM_DUMP:  // dump the RAM

												if(RamDumpFlag==2)
												{
														u8temp= DUMPSIZE/SIZE_PER_PACKET;
														u8temp=(DUMPSIZE%SIZE_PER_PACKET)? (u8temp+1):u8temp;
														TransLen=SIZE_PER_PACKET;
														TxBuffer[0]=TYP_RAM_DUMP;
														TxBuffer[1]=RAM_Dump_index/SIZE_PER_PACKET;
														TxBuffer[1]=(RAM_Dump_index%SIZE_PER_PACKET)? (TxBuffer[1]+1):TxBuffer[1];
												
														u16temp=0;
														for(i=0;i<u8temp;i++)
														{
																TxBuffer[2]=i+1;
																for(j=0;j<SIZE_PER_PACKET;j++)
																		TxBuffer[j+3]=RamDumpBlock[u16temp++];

																USBRes(ResCode, CMD_DEBUG, TxBuffer, TransLen+3);
																delay_ms(32);
														}
														RamDumpFlag=TickDump=0;  // unlock ram to dump again;
														TRG_OUT=0;
												}
												else
												{
														ResCode=BAD_DATA;
													  TransLen=1;
													  USBRes(ResCode, CMD_DEBUG, TxBuffer, TransLen);
												}
									break;					
								default:
									break;
							}
						break;
#endif
#endif							
					default:
						break;
				
				}
			}		
		}


		tick++;
		delay_ms(10);
/*
		if(SPI_2_Rx_sts==SPI_CMPLT)
		{
			for(i=0;i<test_lenth;i++)
				u8buff[i]=SPI_2_RcvBuf[i];
			SPI_2_Rx_sts=SPI_IDLE;
		}
*/		
	if(tick==5)
		{
       ldecoun++;
			//LED0=!LED0;//提示系统正在运行	
		  	if(ldecoun==0)
					LED0=!LED0;
        if(ldecoun==1)
					LED1=!LED1;
        if(ldecoun==2)
					LED2=!LED2;
       if(ldecoun==3)
			  LED3=!LED3;
       if(ldecoun==4)
			  LED4=!LED4;
       if(ldecoun==5)
			 { 
			   LED5=!LED5;
				 ldecoun=0;
			 }
 		 
			tick=0;
		}
		
#endif   // end of #ifdef __RPI_THERMO  
			   	   
	}
}

void Print_packet(u8 ResCode, u8 command, u8 *TxBuffer, u8 TransLen)
{
   if(USB_ReceiveFlg!=TRUE)
	   UARTRes(ResCode, CMD_READ, TxBuffer, TransLen);
   else 
   {
   	   USBRes(ResCode, CMD_PID_CFG, TxBuffer, TransLen);
	   USB_ReceiveFlg = FALSE;
	}
}


void EXTI0_IRQHandler(void)

{
		    		    				     		    
	EXTI->PR=1;  //清除中断标志位	  
	PCR_ADC_Done_Flag=1;
} 


u8 PacketChkSum(u8 *p, u8 length)
{
	u8 i;  
	u8 temp=0;
	for	(i=1;i<(length-2);i++)
		temp+=*(p+i);
	if(temp==TAIL)
		temp++;
	if(temp==*(p+length-2))
		return TRUE;
	else
		return FALSE;
	
}

u8 buf[64]={0};	  
/* LPG: response to PC through UART*/
/* 		parameter: respose, command, data string, data length*/
/*		function format the packet & cal checksum, send out  */
void UARTRes(u8 response, u8 command,u8 * buffer, u8 length)
{
	u8 i, u8temp, len;
	buf[0]=HEADER;
	buf[1]=response;
	buf[2]=command;
	buf[3]=length;
	len=4;
	for(i=0;i<length;i++,len++)
			buf[len]=*(buffer+i);
	u8temp=0;
	for(i=1;i<(len-1);i++)
		u8temp+=buf[i];
	if(u8temp==TAIL)
		u8temp++;
   	buf[len]=u8temp;
	buf[len+1]=buf[len+2]=TAIL;
	buf[len+3]='\0';
	{
		for(i=0;i<YG_FIX_LEN;i++)
			putchar(buf[i]);		
	}
	
}

void USBRes(u8 response, u8 command,u8 * buffer, u8 length)
{
	u16 i, u8temp, len;
	Transi_Buffer[0]=HEADER;
	Transi_Buffer[1]=response;
	Transi_Buffer[2]=command;
	Transi_Buffer[3]=length;
	len=4;
	for(i=0;i<length;i++,len++)
			Transi_Buffer[len]=*(buffer+i);
	u8temp=0;
	for(i=1;i<(len-1);i++)
		u8temp+=Transi_Buffer[i];
	if(u8temp==TAIL)
		u8temp++;
   	Transi_Buffer[len]=u8temp;
	Transi_Buffer[len+1]=Transi_Buffer[len+2]=TAIL;
	//while((GetEPTxStatus(ENDP2) & 0x30)!=EP_TX_VALID);
    UserToPMABufferCopy(Transi_Buffer, ENDP2_TXADDR, nReportCnt);//len+3);
	SetEPTxCount(ENDP2, nReportCnt); 
    SetEPTxValid(ENDP2);
}
	

void Fan_Echo(u8 dat)
{
	TxBuffer[0]= dat;
//txc//	USBRes(NO_ERR, CMD_TEMP_SET, TxBuffer, 1); 	
}


#if defined(USB_WAIT_MODE)
static MSG_TYP UsbFlg=0;
static u8 EpMsgRdy=0;
void EpMsgEnable(void)
{	
	EpMsgClr();	
	EpMsgRdy=1;				
}
void EpMsgDisable(void)
{
	EpMsgRdy=0;
	EpMsgClr();					
}

void EpMsgStk(MSG_TYP u8Q)
{
	if(EpMsgRdy)
		UsbFlg=	u8Q;	
}

MSG_TYP EpMsgPop(void)
{
	MSG_TYP msg=UsbFlg;
	EpMsgClr();
	return msg;
}

void EpMsgClr(void)
{
   UsbFlg=0;
}

MSG_TYP UsbReadDone(void)
{
//	u8 i;
	MSG_TYP msg=EpMsgPop();
	if(msg==0)
	{
	/*
		for(i=0;i<USB_WAIT_TIM;i++)
		{
			delay_ms(35);
			msg=EpMsgPop();
			if(msg!=0)
				break;
			else
				msg=0xff;	
		}
		if(msg==0xff)
			msg=0xfe;
	*/
	do{			delay_ms(64);
			msg=EpMsgPop();}while(msg==0);
	}
	return msg;
}
#endif



void EXTI9_5_IRQHandler(void)
{
	if(EXTI->PR & (0x1<<6)) //trg happen
	{
		EXTI->PR |= (0x1<<6);
		if(mst_flag == BRD_IN_IDLE)
		{
			//trg_count=0; // count the trg number
			mst_flag = BRD_IN_TRG;  // flag record
		}
	}
	else/*
		 if(EXTI->PR & (0x1<<15)) // rst happen
		{
			EXTI->PR |= (0x1<<15);
			trg_count=0; 				
		}
		*/
		EXTI->PR |= 0xffff;
}
