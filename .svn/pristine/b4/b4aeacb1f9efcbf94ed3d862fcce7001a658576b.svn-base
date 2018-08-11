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
	//#include "temperature.h"
	#include "PCR_Cycle.h"	
	#include "usb_lib.h"
	#include "hw_config.h"
	#include "usb_pwr.h" 
	#include "usb_conf.h"
	#include "user_def.h"
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
	u16 TMR_Int_Flag_1=0;
	u16 TMR_Int_Flag_2=0;
	u16 TMR_Int_Flag_3=0;

	u8 PCR_ADC_Done_Flag=0;
	u8 PCR_ADC_Done_Flag_1=0;
	u8 PCR_ADC_Done_Flag_2=0;
	u8 PCR_ADC_Done_Flag_3=0;
	void PIX_Drive_Sequence_Sim(void);
	u8 Send_Command(u8 cmd, u8 row_num, u8 length, u8 * pbuf);

	u8 Comand_Buf[MAX_LIMIT]={0};
	u8 Command_Len=0;
	u8 TxBuffer[MAX_LIMIT]={HEADER};
	u8 PacketChkSum(u8 *p, u8 length);
	void UARTRes(u8 response, u8 command,u8 * buffer, u8 length);
	void USBRes(u8 response, u8 command,u8 * buffer, u8 length);
	//void Read_Row(u8 row);
	//void ReadUpdate_Image(void);
	//void Data_Cov(u8 txset);

	//void ReadUpdate_Image24_XC(u8 mode,u8 row);
	//void ReadUpdate_Image24_slow(u8 mode, u8 row);



	PCR_Regs_type PCR_Regs[MAX_PCR_CH];

	u16 InteCnt=0;	/*LPG:  this is record the integration time */
	u8 	PixReadmMode=DISABLE;

	//u8 ImageBuf[(PIX_TOTAL_ROW<<1)][(PIX_TOTAL_COL<<1)+2]={0};
	u8 ImageBuf[PIX_TOTAL_ROW][(PIX_TOTAL_COL<<1)+2]={0};
	u8 ImageBufPIX[64][64]={0};
	u8 pipeLine_ImageBuf[MAX_PCR_CH][PIX_TOTAL_ROW][(PIX_TOTAL_COL<<1)+2]={0};

	const u8 PCRChip_Pattern[11]={0xC0,0xE0,0x60,0x00,0xA1,0xB5,0x21,0x61,0x29,0x21,0x00}; 
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

	Cycle_STS_Type CycleSTS; 

	u8 USB_Reply_Tail=FALSE;
	u8 const PIXEL_24READ[4]={0x1,0x2,0x4,0x8};
	/////for USB

	#if defined(USB_WAIT_MODE)
	void EpMsgEnable(void);
	void EpMsgDisable(void);
	MSG_TYP UsbReadDone(void);
	#endif

	#ifdef __QPCR_HW
/*
		#define LED_CTRL		PAout(0)	 
		#define LED_CTRL_1 	PAout(1)
		#define LED_CTRL_2 	PAout(2)
		#define LED_CTRL_3 	PAout(3)
*/
	/////// version 1.17 
		#define LED_CTRL		PAout(3)	 
		#define LED_CTRL_1 	PAout(2)
		#define LED_CTRL_2 	PAout(1)
		#define LED_CTRL_3 	PAout(0)	
	#else
		//////////////external trigger .2014-12-20
		#define LED_CTRL	PFout(5)//#define LED_CTRL	PGout(13)	   // led control out
		#define LED_CTRL_1 PDout(7)
		#define LED_CTRL_2 PDout(14)
		#define LED_CTRL_3 PDout(15)
	#endif
	
	#define TRG_IMG		PFin(6)//#define TRG_IMG		PEin(14)       // trigger input


	#define BRD_IN_IDLE	0
	#define BRD_IN_TRG 	1
	#define BRD_IN_PROCESS	2
	#define BRD_IN_READ			3

	u8 mst_flag=BRD_IN_IDLE;  // flag record
	u16 Time_LED_Delay=0; 	// timer
	u16 SetTm_LED_Delay=10;	// setup time
	u16 HoldTm_LED_Delay=10; // hold time 

	u8 led_mode=0; 
	u8 led_indiv=0;
	//////////////
	///////////////////multi PCR
	u8 PcrSTS=0;      // host notify status register LSB 4 bits valid, 1: trigger capture valid
	u8 PCR_Trg_Reg=0; // shadow register from Exti trigger input
	u8 Exti=0;        // 1:  Exti event happen
	u8 LedTimOut=0;   // 1: led setup/hold time tine out event
	u8 PixMode=0;     // 0: 12 pixel mode, 1: 24 pixel mode
	const u8 PcrChannelTable[16]={0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};
	
#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))  //version 1.14: check button	
	MSG_TYP PcrMskReg=0x3; //default enable all channel 
#else
	MSG_TYP PcrMskReg=0xF; //default enable all channel 
#endif
	
	u8 ImageArrayBuf[MAX_PCR_CH][64][64]={0};     // image buffer array for multi PCR

	u8 SPI_Sel=0;
	u8 SPI_Sel_Reg=0; // shadow 
	u8 PCR_Data_Valid_mode_0=0; // pending of 12PIX 
	u8 PCR_Data_Valid_mode_1=0; // pending of 24PIX
											////////////read multi PCR
	void ReadUpdate_Image_multiPCR_PipeLine_mode_0(u8 cnt);
	void ReadUpdate_Image_multiPCR_oneRead_mode_0(u8 sel, u8 ContinueMode);
	void ReadUpdate_Image_multiPCR_SlowRead_mode_0(u8 sel, u8 ContinueMode);

	void ReadUpdate_Image_multiPCR_PipeLine_mode_1(u8 cnt);
	void ReadUpdate_Image_multiPCR_oneRead_mode_1(u8 sel, u8 ContinueMode);
	void ReadUpdate_Image_multiPCR_SlowRead_mode_1(u8 sel, u8 ContinueMode);

	void Read_Row_multiPCR_mode_0(u8 row, u8 sel, u8 Is_continue);
	void Read_Row_multiPCR_mode_1(u8 row, u8 sel);
	void PcrTrgDisable(void);
	void PcrMskMapping(u8 mask);

	void Read_Image_to_buffer(u8 sel, u8 mode);

	//////2015-09-20, add time out
	#define MAX_FAIL_TIMOUT_CNT	6
	extern u8 u8TimOut_flag;
	extern u8 u8TimOut_check;
	extern u8 u8Timout_Cnt;
	void timeout_initial(u16 mS);
	u8 timeout_check(void);
	void timeout_stop(u8 channel);
	u8 PCR_Fail_cnt[MAX_PCR_CH]={0};  // channel fail counter
	void PCR_initialize(void);
	////////////////////
	u8 TrgChQue[MAX_PCR_CH]={0}; // buffer to save channel# for overlap
	//////////////////////
	#ifdef INTE_DEBUG
		u8 inte_toggle=0;
	#endif 
	u16 tempReg=0; 
	u8 debugReg[4]={0,0,0,0};

	u8 multi_overlap_ctrl=0;
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
	u8 test_flag=0;
	void Image_capture_enable()
	{
#ifdef __QPCR_HW
		
		Ex_NVIC_Config(GPIO_C,13,RTIR);
		Ex_NVIC_Config(GPIO_C,12,RTIR);
		Ex_NVIC_Config(GPIO_C,11,RTIR);
		Ex_NVIC_Config(GPIO_C,10,RTIR);
		EXTI->PR |= 0xffff;
		MY_NVIC_Init(2,3,EXTI15_10_IRQChannel,2);	
		
#else
		Ex_NVIC_Config(GPIO_F,6,RTIR); 
		Ex_NVIC_Config(GPIO_F,7,RTIR);
		Ex_NVIC_Config(GPIO_F,8,RTIR); 
		Ex_NVIC_Config(GPIO_F,9,RTIR);	
		MY_NVIC_Init(2,3,EXTI9_5_IRQChannel,2);  
#endif
	}

	void Image_capture_disable()
	{
#ifdef __QPCR_HW
		//NVIC->ISER[0]&=~((vu32)1<<EXTI15_10_IRQChannel);
		EXTI->IMR&=~(0xf<<10);		
#else
		NVIC->ISER[0]&=~(1<<EXTI9_5_IRQChannel);
		EXTI->IMR&=~(0xf<<6);
#endif
	}

	float PCR_temp[4]={0};
	void tick_for_ADC_initial(void);

	#ifdef __USE_CAN_BUS
	#define SCALER 8
	#define CAN_BUF_START_INDEX	1
	#endif

	
#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))
		#ifdef __USE_PCR_ADC_DELAY_DEBUG	
				float f_MOS_delay=300;
		#endif
#endif	
	
	int main(void)
	{
			u8 tick,txpattern,sel,j;
			u16 adcx;
			u8 u8temp, ledcount;	
			u8 CurrentRowNum;
			MSG_TYP msgLog=0;
			u8 u8temp1,u8temp3;
			u16 u16temp;

			u8 ResCode;
			u8 TransLen;

			float fTemp=0;
		  /////////20170608
		  float fPcrTemp[4]={0};
		
			u8 Img_Enable=0;
		
	//	u8 VideoRow;

		
			
	//	u8 key,ldecoun=0;
	//	u8 Key_count=0;
			u8 test_lenth;
			u16 i;
			#ifdef VEDIO_DEBUG
			u8 DatChange=0;				
			#endif

			#ifdef INTE_DEBUG
				FAN=inte_toggle=0;
			#endif 
      
	#ifdef __USE_CAN_BUS
			u8 res;
			u8 CAN_cnt_total_wait=1;	// to track how many total packet expected
			u8 CAN_idx_next=1;  // to check packet whether match expected index
			u8 CAN_Flag_wait=0;
			u8 CAN_Timer_wait=0;
			u8 mode=0; //CAN工作模式;0,普通模式;1,环回模式
			u8 CAN_cnt_index=CAN_BUF_START_INDEX;  // always start from 1, as reserve  0 for header
			
			u8 CAN_command_byte=0;
			u8 CAN_type_byte=0;
			u8 CAN_txbuf[8]={0};
	#endif
	
			
		
		tick=ledcount=0;

		Stm32_Clock_Init(9);	//系统时钟设置 --72M
//		uart_init(36,9600);	 	//串口初始化为9600
		delay_init(72);	   	 	//延时初始化 


#ifdef __USE_SPI_1_SLAVE  // version 1.8
		SPI1_Slave_Initializaion();
#endif			
		SPI2_Initializaion();
			
	#ifndef __USE_CAN_BUS
		#if defined(USB_WAIT_MODE)
		EpMsgDisable();
		#endif
	#endif			
			

		//PCR_Regs.InteDelayCount=INTE_DELAY_COUNT;
		for(i=0;i<MAX_PCR_CH;i++)
		{
				PCR_Regs[i].InteDelayCount=INTERVAL_DELAY_COUNT;
				PCR_Regs[i].InteTime=DEFAULT_INTE;//(u16)INTE_CYCLE_COUNT*(u16)TIME_US_PER_CNT;
				PCR_Regs[i].InteCount=INTE_CYCLE_COUNT;
		}
		SPI_2_Rx_sts=SPI_IDLE;
		//InteCnt= 4999;
//		FAN=0;
		TIM3_Int_Init(PCR_Regs[0].InteCount,TIM3_PSC_VAL);
		TIM1_Int_Init(PCR_Regs[0].InteCount,TIM3_PSC_VAL);
		TIM4_Int_Init(PCR_Regs[0].InteCount,TIM3_PSC_VAL);
		TIM8_Int_Init(PCR_Regs[0].InteCount,TIM3_PSC_VAL);
		
		TMR_Int_Flag=0;
		TIM3->CR1|=0x01;
		while(TMR_Int_Flag==0);
		TMR_Int_Flag=0;

		timeout_initial(1);
		while(timeout_check()==0);
		timeout_stop(1);
		
	////////////////////////////////////////////////////////////

		Adc_Init();

		Trim_Reset();	

	#ifndef __USE_CAN_BUS

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
					

	/////////2015-09-20 PCR detection

	////////////////////////////////
		LED_Init();		  		// GPIO. LED, Trigger initialization
		OSC_Ctrl_Init();
		
		LED_CTRL=LED_CTRL_2=LED_CTRL_3=LED_CTRL_1=led_mode=led_indiv=0;   // 2016-04-23, LED control separated

		
		
#ifdef __USE_OSC_GATE   //version 1.13 
		OSC_mode=OSC_AUTO;
		OSC_Status=0;
		OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_OFF;
#else
		OSC_mode=OSC_ON;
		OSC_Status=0xF;
		OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_ON;
#endif
		PCR_RESET_PROCESS;

#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))  //version 1.14
		Lucentix_Button_Process(BUTTON_DONE_MASK);
		Lucentix_Button_msg_clr();		
#endif

//		EXTIX_Init();

		//  PCR_initialize();
		//	PCR_initialize();
			
			////////////////////debug only

		//Exti=1;
		//MsgTrgPush(0xf);

		///////////////////debug only
		//////////////////	while(1);
		
#if (!defined(__USE_MOS_GATE))	
		tick_for_ADC_initial();
		PCR_temp[0]=PCR_temp[1]=PCR_temp[2]=PCR_temp[3]=0;
		for(i=0;i<4;i++)
		{
				fTemp=0;
				u16temp=Get_Adc(ADC_CH10);
				PCR_temp[0] += (float)u16temp * 3.3/4096;
				
				u16temp=Get_Adc(ADC_CH11);
				PCR_temp[1] += (float)u16temp * 3.3/4096;
				
				u16temp=Get_Adc(ADC_CH12);
				PCR_temp[2] += (float)u16temp * 3.3/4096;
				
				u16temp=Get_Adc(ADC_CH13);
				PCR_temp[3] += (float)u16temp * 3.3/4096;
		}
		for(i=0;i<4;i++)
		{
			PCR_temp[i] = PCR_temp[i] /4;
		}	
#else

		#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)) && (defined(__USE_POWER_FOR_EVER)))
						MOS_Gate_Ctrl(OSC_ON);
						//version 1.8: 																
		#endif			
		
#endif
		
		while(1)
		{		

	////////////////////debug only
	/*
	if(Exti==0) 
	{
	Exti=1;
		PcrMskReg=0x4;
	MsgTrgPush(0x4);
		PCR_Regs[0].InteTime=PCR_Regs[1].InteTime=PCR_Regs[2].InteTime=PCR_Regs[3].InteTime=16;
	}
	*/

	///////////////////debug only
	#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))  //version 1.14
		u8temp = Lucentix_Button_sts_read();
		if((u8temp & BUTTON_ACTIVE_MASK)==0)
		{
			    if(Lucentix_Button_Check())
					{
							Lucentix_Button_sts_Bit_Set(BUTTON_ACTIVE_MASK);		 
							MsgTrgForce(LUCENTIX_CH_BIT, LUCENTIX_CH_BIT);  
							Lucentix_Button_Process(BUTTON_ACTIVE_MASK);
					}
		}
		else
		{
				if (u8temp & BUTTON_PROCESSED_MASK)
				{
						if(u8temp & BUTTON_DONE_MASK)
						{
								Lucentix_Button_msg_clr();
								Lucentix_Button_Process(BUTTON_DONE_MASK);
						}
				}
		}
	#endif

			
			if(mst_flag !=BRD_IN_READ)  
			{
				
				
					 if(Exti !=0)
					 {
							Exti=0;
						 
	#ifdef _OVERLAP_ISSUE_DEBUG	
								//if(msgLog==0x3)
										LED3=1;
	#endif		
						 msgLog = MsgTrgPop();
	#ifdef _OVERLAP_ISSUE_DEBUG	
								if(msgLog==0x3)
										LED4=1;
	#endif					 
						 
							 PCR_Trg_Reg |= msgLog;
							 
							
								
							 if(mst_flag == BRD_IN_IDLE)
									mst_flag = BRD_IN_TRG;
					 }	

					if(mst_flag==BRD_IN_TRG)    
					{
						 mst_flag = BRD_IN_PROCESS; // handle this trg, & in LED ctrl
						/*
						 Time_LED_Delay=0;
						 TIM7_Init();
						 /////////////////////////////2016-04-23 LED control separated
						 //LED_CTRL=1;
						 u8temp=PCR_Trg_Reg;
						
						 if(led_indiv)  // independence mode
						 {
							 if(u8temp & 0x1)	LED_CTRL=1;
							 if(u8temp & 0x2)	LED_CTRL_1=1;
							 if(u8temp & 0x4)	LED_CTRL_2=1;
							 if(u8temp & 0x8)	LED_CTRL_3=1;
						 }
						 else LED_CTRL=1;
						*/
					}
					else if(mst_flag == BRD_IN_PROCESS)	
								{
										//if(Time_LED_Delay>=SetTm_LED_Delay)
										{
												//TIM7_Stop();
												mst_flag = BRD_IN_READ;
												
												// PCR_Trg_Reg  is locked now,  later trigger processed as next capture
												//u8temp=PcrChannelTable[(PCR_Trg_Reg & 0xF)];  // check how many channel is triggered
												u8temp=PCR_Trg_Reg;
												
												u8temp1=0;
												for(i=0; i<MAX_PCR_CH; i++)
												{
														if(u8temp & (1<<i))
														{
															TrgChQue[u8temp1]=i;
															u8temp1+=1;									
														}
														
												}
												
											
												
												if(u8temp1>0)   // more than one channel is to be sampled check whether can integrate overlap						
												{
	/*
	#ifdef _OVERLAP_ISSUE_DEBUG	
								
										LED4=1;
	#endif	
	*/			

#ifdef __USE_MULTI_OVERLAP      // version 1.11
													
 													if(u8temp1>1)
														{
														multi_overlap_ctrl =0x80; // bit7 : overlap_enable
														for(i=0; i<u8temp1; i++)
														{
																if((PCR_Regs[TrgChQue[i]].InteTime < (u8temp1 * 20))) //&& (PixMode == 0))  // >10ms can do pipeline, assume pipeline of 12x12 is 20ms
																{
																		multi_overlap_ctrl=0;   // can not overlap
	/*
																	#ifdef _OVERLAP_ISSUE_DEBUG	
								
										LED3=0;
	#endif	
	*/									
																	break;
																}
														}
										#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
														//version 1.13
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_ON);
													#endif
										#endif
														
										#ifdef __USE_OSC_GATE   //version 1.13
														if(multi_overlap_ctrl)
														{
																for(i=0; i<u8temp1; i++)
															  {
																		Multi_OSC_Access(TrgChQue[i],OSC_ON);
																}
														}
										#endif	
													}
														else 
#endif															
																multi_overlap_ctrl=0;
														 

														
															
														if(multi_overlap_ctrl !=0)   // can do integrate overlap
														{
															
															if(led_indiv)  // independence mode
															 {
																 if(u8temp & 0x1)	LED_CTRL=1;
																 if(u8temp & 0x2)	LED_CTRL_1=1;
																 if(u8temp & 0x4)	LED_CTRL_2=1;
																 if(u8temp & 0x8)	LED_CTRL_3=1;
															 }
															 else LED_CTRL=1;
															 delay_ms(8);
															 
																if(PixMode == 0)   // 12 pixel mode
																{
																		PixReadmMode=TYP_IMAGE;	 
																		ReadUpdate_Image_multiPCR_PipeLine_mode_0(u8temp1);
																		
																		for(i=0; i<u8temp1; i++)												
																			PCR_Data_Valid_mode_0 |= (1<<TrgChQue[i]);  // flag for 4 PCR data to be read
																}
																else 	// 24x24 pixel mode
																{
																		PixReadmMode=TYP_24PIXIMAG;
																	
																		ReadUpdate_Image_multiPCR_PipeLine_mode_1(u8temp1); /* access a image*/
																		
																		for(i=0; i<u8temp1; i++)												
																			PCR_Data_Valid_mode_1 |= (1<<TrgChQue[i]);  // flag for 4 PCR data to be read
																		
																}
																
																LED_CTRL=(led_mode & 0x1);
																if(led_indiv)  // independence mode
															 {

																 LED_CTRL_1=(led_mode & 0x2);
																 LED_CTRL_2=(led_mode & 0x4);
																 LED_CTRL_3=(led_mode & 0x8);
															 }
												
										#ifdef __USE_OSC_GATE   //version 1.13
														  if(OSC_mode != OSC_ON)
															{
																	for(i=0; i<u8temp1; i++)
																	{
																			Multi_OSC_Access(TrgChQue[i],OSC_OFF);
																	}
															}
										#endif

										#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
														//version 1.13	
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_OFF);
													#endif
										#endif

														}
														else
														{
																if(PixMode == 0)   // 12 pixel mode
																{
																		for(u8temp1=0; u8temp1<MAX_PCR_CH; u8temp1++)
																		{
																				if(u8temp & (1<<u8temp1))
																				{	
																		#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																						//version 1.13											
																						#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																									MOS_Gate_Ctrl(OSC_ON);
																						#endif
																		#endif																					

																					
																		#ifdef __USE_OSC_GATE   //version 1.13
																					Multi_OSC_Access(u8temp1,OSC_ON);
																		#endif		
																					 if(led_indiv)  // independence mode
																					 {
																						 switch(u8temp1)
																						 {
																								default:
																								case 0: LED_CTRL=1;
																												if(!(led_mode & 0x1))
																														delay_ms(8);
																										break;
																								case 1: LED_CTRL_1=1; if(!(led_mode & 0x2)) delay_ms(8);break;
																								case 2: LED_CTRL_2=1; if(!(led_mode & 0x4)) delay_ms(8);break;
																								case 3: LED_CTRL_3=1; if(!(led_mode & 0x8)) delay_ms(8);break;
																						 }
																					 }
																					 else 
																					 {
																							LED_CTRL=1; 
																							delay_ms(8);
																					 }																			
																						
																						if(PCR_Fail_cnt[u8temp1] > MAX_FAIL_TIMOUT_CNT)  // check MAX timeout or not?  @2015-09-20
																						{
																								for(j=0;j<(PIX_TOTAL_ROW<<1);j++)
																								{
																										ImageArrayBuf[u8temp1][j][0]=((TYP_IMAGE) | (u8temp1<<4));    
																										ImageArrayBuf[u8temp1][j][1]=0xF1;
																																											
																										for(i=0;i<(PIX_TOTAL_COL<<1);i++)
																											ImageArrayBuf[u8temp1][j][i+2]=0;
																								}
																						}
																						else
																						{
				
																								PixReadmMode=TYP_IMAGE;
																								if(PCR_Regs[u8temp1].InteTime>10)
																								{												 // must in serial method															  
																											u8temp3=PCR_Fail_cnt[u8temp1];
																											ReadUpdate_Image_multiPCR_oneRead_mode_0(u8temp1,(u8temp>>(u8temp1+1)));
																											if(u8temp3 < PCR_Fail_cnt[u8temp1])   // fail happened in this cycle, then overwrite packet
																											{
																													for(j=0;j<(PIX_TOTAL_ROW<<1);j++)
																													{
																															ImageArrayBuf[u8temp1][j][0]=((TYP_IMAGE) | (u8temp1<<4));    
																															ImageArrayBuf[u8temp1][j][1]=0xF1;
																																															
																															for(i=0;i<(PIX_TOTAL_COL<<1);i++)
																																ImageArrayBuf[u8temp1][j][i+2]=0;
																													}
																											}
																											PCR_Data_Valid_mode_0 |= (1<<u8temp1);  // flag for 4 PCR data to be read
																								}
																								else   // integration T<10, have to row by row
																								{
																										u8temp3=PCR_Fail_cnt[u8temp1];
																										ReadUpdate_Image_multiPCR_SlowRead_mode_0(u8temp1,(u8temp>>(u8temp1+1)));
																										PCR_Data_Valid_mode_0 |= (1<<u8temp1);  // flag for 4 PCR data to be read
																										if(u8temp3 < PCR_Fail_cnt[u8temp1])   // fail happened in this cycle, then overwrite packet
																										{
																												for(j=0;j<(PIX_TOTAL_ROW<<1);j++)
																												{
																														ImageArrayBuf[u8temp1][j][0]=((TYP_IMAGE) | (u8temp1<<4));    
																														ImageArrayBuf[u8temp1][j][1]=0xF1;
																																														
																														for(i=0;i<(PIX_TOTAL_COL<<1);i++)
																															ImageArrayBuf[u8temp1][j][i+2]=0;
																												}
																										}
																								}																									
																								
																						}
																						
															#ifdef __USE_OSC_GATE   //version 1.13
																				if(OSC_mode != OSC_ON)
																						Multi_OSC_Access(u8temp1,OSC_OFF);
															#endif

															#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																			//version 1.13	
																		#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																					MOS_Gate_Ctrl(OSC_OFF);
																		#endif
															#endif						
							
																				}					
																				
																				LED_CTRL=(led_mode & 0x1);
																				if(led_indiv)  // independence mode
																				{
																					 LED_CTRL_1=(led_mode & 0x2);
																					 LED_CTRL_2=(led_mode & 0x4);
																					 LED_CTRL_3=(led_mode & 0x8);
																				}
												
																		}
																}
																else               // 24 pixel mode
																{
																		PixReadmMode=TYP_24PIXIMAG;
					//											 	if(PCR_Regs.InteTime>(float)(10 * u8temp)) 	// can use piepline method
					//											   	 ReadUpdate_Image_multiPCR_PipeLine_mode_1(PCR_Trg_Reg);
					//											 	else											
																		{  // must in serial method
																				for(u8temp1=0; u8temp1<MAX_PCR_CH; u8temp1++)
																				{

																						if(u8temp & (1<<u8temp1))
																						{
																					#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																									//version 1.13
																								#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																											MOS_Gate_Ctrl(OSC_ON);
																								#endif
																					#endif
																				
																							
																					#ifdef __USE_OSC_GATE   //version 1.13 
																								Multi_OSC_Access(u8temp1,OSC_ON);
																					#endif					
																							
																							
																								if(led_indiv)  // independence mode
																								{
																											switch(u8temp1)
																											{
																												default:
																												case 0: LED_CTRL=1;
																																if(!(led_mode & 0x1))
																																		delay_ms(8);
																														break;
																												case 1: LED_CTRL_1=1; if(!(led_mode & 0x2)) delay_ms(8);break;
																												case 2: LED_CTRL_2=1; if(!(led_mode & 0x4)) delay_ms(8);break;
																												case 3: LED_CTRL_3=1; if(!(led_mode & 0x8)) delay_ms(8);break;
																											}
																								 }
																								 else 
																								 {
																										LED_CTRL=1; 
																										delay_ms(8);
																								 }
																							
																							
																							
																								if(PCR_Regs[u8temp1].InteTime>10)
																									ReadUpdate_Image_multiPCR_oneRead_mode_1(u8temp1,(u8temp>>(u8temp1+1))); /* access a image*/
																								else
																									ReadUpdate_Image_multiPCR_SlowRead_mode_1(u8temp1,(u8temp>>(u8temp1+1)));
																								PCR_Data_Valid_mode_1 |= (1<<u8temp1);
																								
																			#ifdef __USE_OSC_GATE   //version 1.13 
																								if(OSC_mode != OSC_ON)
																										Multi_OSC_Access(u8temp1,OSC_OFF);
																			#endif																									
																								
																								
																								LED_CTRL=(led_mode & 0x1);
																								if(led_indiv)  // independence mode
																								{
																								 LED_CTRL_1=(led_mode & 0x2);
																								 LED_CTRL_2=(led_mode & 0x4);
																								 LED_CTRL_3=(led_mode & 0x8);
																								}
																								
																						}
																				}																				
																		}
																		
																				
																		#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																						//version 1.13
																					#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																								MOS_Gate_Ctrl(OSC_OFF);
																					#endif
																		#endif																			
																		
																}
														}
														//LED_CTRL=LED_CTRL_1=LED_CTRL_2=LED_CTRL_3=0;
														///////////2016-04-23 LED control separated, including the LED control mode, maybe forced 
																											 
														mst_flag=BRD_IN_IDLE;
														PcrSTS |= (PCR_Trg_Reg & 0xF);
														PCR_Trg_Reg=0;
														#ifndef __QPCR_HW
																#ifdef  __DEBUG_ON_OLD_HW													
																		LED_D1=!LED_D1;
																#endif
														#endif
												}							
										}
									#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))  //version 1.14
											u8temp=Lucentix_Button_sts_read();
											if ((u8temp & BUTTON_ACTIVE_MASK) && (!(u8temp & BUTTON_PROCESSED_MASK))) 
											{
													Lucentix_Button_sts_Bit_Set(BUTTON_PROCESSED_MASK);
													Lucentix_Button_Process(BUTTON_PROCESSED_MASK);
											}
									#endif									
								}
			}
		
				
			
			
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


	#ifdef SPI_DEBUG
			RxStage=CMPLT;
	#endif		
			
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

			#ifndef SPI_DEBUG			
						if(PacketChkSum(Comand_Buf, Command_Len)==TRUE)
			#else
						Comand_Buf[CMD_BYTE_NUM]=CMD_GET;
						Comand_Buf[TYP_BYTE_NUM]=TYP_IMAGE;
			#endif
						
	#endif 
				{
						
					switch(Comand_Buf[CMD_BYTE_NUM])
					{
						case CMD_GET:
		
								u8temp1=(Comand_Buf[TYP_BYTE_NUM]>>4);  // dispatch PCR selection
								u8temp1 &= 0xF;
								if(u8temp1)
										u8temp1 &= 0xF;
								TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];
								if((u8temp1>=MAX_PCR_CH) && (u8temp1 != 0xF))
								{			// invalid PCR selection
											ResCode=OUT_RANGE;
											//TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];
											
											#ifndef __USE_CAN_BUS	
																		USBRes(OUT_RANGE, CMD_GET,TxBuffer,1);
											#else
																		CANRes(ResCode, CMD_GET, TxBuffer, 1); // send ACK no error						 
											#endif								
									
											break;
								}
								
								SPI_Sel=SPI_Sel_Reg=u8temp1;               // lock SPI_SS[x] pin

								switch(Comand_Buf[TYP_BYTE_NUM] & 0xF)
								{

										case TYP_ROW:

											 /////////////////////////////2016-04-23 LED control separated
											 //LED_CTRL=1;
												if(led_indiv)  // not share mode
												{
													 switch(u8temp1)
													 {
														 case 0: LED_CTRL=1;
															break;
														 case 1: LED_CTRL_1=1;
															break;
														 case 2: LED_CTRL_2=1;
															break;
														 case 3: LED_CTRL_3=1;
															break;
														 default:
															break;											 
													 }
												}
												else 
													LED_CTRL=1;
												
								#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
												//version 1.13												
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_ON);
													#endif
								#endif
												
								#ifdef __USE_OSC_GATE   //version 1.13 
											Multi_OSC_Access(SPI_Sel,OSC_ON);
								#endif	
												
											delay_ms(6);
											PixReadmMode=TYP_ROW;	  
											CurrentRowNum=Comand_Buf[TYP_BYTE_NUM+1];
											if(CurrentRowNum > PIX_TOTAL_ROW)  
											{	//LPG:  invalid row number
												ResCode=ERR_ROW;
												//TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];
												TransLen=1;
											}	
											else
											{
												Read_Row_multiPCR_mode_0(CurrentRowNum, SPI_Sel,0); // access a row
												ResCode=NO_ERR;
													
												///////////2016-04-23 LED control separated, including the LED control mode, maybe forced
												LED_CTRL = (led_mode & 0x1)? 1:0;
												if(led_indiv)		
												{																					
													LED_CTRL_1 = (led_mode & 0x2)? 1:0;
													LED_CTRL_2 = (led_mode & 0x4)? 1:0;
													LED_CTRL_3 = (led_mode & 0x8)? 1:0;
												}	
												//  TxBuffer[0]=TYP_ROW;
												TxBuffer[1]=CurrentRowNum;
												for(i=0;i<(PIX_TOTAL_COL<<1);i++)
													TxBuffer[i+2]= SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015

												TransLen=(PIX_TOTAL_COL<<1)+2;
											}
											
								#ifdef __USE_OSC_GATE   //version 1.13 
											if(OSC_mode != OSC_ON)
													Multi_OSC_Access(SPI_Sel,OSC_OFF);
								#endif

								#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
												//version 1.13	
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_OFF);
													#endif
								#endif
								
											
											#ifndef __USE_CAN_BUS	
													USBRes(ResCode, CMD_GET,TxBuffer,TransLen);
													//PixReadmMode=DISABLE;
													#if !defined(USB_WAIT_MODE)
													delay_ms(35);
													#else
													EpMsgEnable();
													UsbReadDone();
													EpMsgDisable();
													#endif	
											#else
													CANRes(ResCode, CMD_GET,TxBuffer,TransLen);		
													delay_ms(6);
											#endif
											
									break;

									case TYP_IMAGE:
											 
										 //add new item "pre-read multi-channels"
										 
										 if(u8temp1==0x0f)  // means pre-read for multi-channels
										 {
												u8temp=Comand_Buf[TYP_BYTE_NUM+1] & 0xF; // data[0] is multi-channels bit mask
												PixMode = ((Comand_Buf[TYP_BYTE_NUM+1] & 0xF0)>>4); // data[0] MSB represent pixel mode, 0 means 12x12
																																					// 1 means 24x24
												ResCode=NO_ERR;
												MsgTrgPush(u8temp);	
											 /*
	#ifdef _OVERLAP_ISSUE_DEBUG	 // pass on this 
						if(u8temp==0x3)	
										LED4=1;
						if(Exti ==1) 
						{
								u8temp=MsgTrgPop();
								if(u8temp==0x3)
											LED3=1;
						}
	#endif					
	*/										 
												TransLen=2;
												TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];
												TxBuffer[1]=Comand_Buf[TYP_BYTE_NUM+1];
												#ifndef __USE_CAN_BUS											  
													USBRes(ResCode, CMD_GET,TxBuffer,TransLen);
												#else
													CANRes(ResCode, CMD_GET,TxBuffer,TransLen);
												#endif
												break;					  
										 }
	/*									 
										 {
												MsgTrgPush(0x7);											
												TransLen=2;
												TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];
												TxBuffer[1]=Comand_Buf[TYP_BYTE_NUM+1];
												USBRes(ResCode, CMD_GET,TxBuffer,TransLen);
												break;					  
										 }
	*/
											// think about maybe trigger data is still to be read
										
											if((PCR_Data_Valid_mode_0>>u8temp1) & 0x1)     // is this PCR triggered pending for read?
											{
													PCR_Data_Valid_mode_0 &= ~(1<<u8temp1);     // clear this pending
													PcrSTS &= ~(1<<u8temp1);
												
													u8temp=(PIX_TOTAL_COL<<1)+2;
													for(i=0;i<(PIX_TOTAL_ROW);i++)
													{
														
											#ifndef __USE_CAN_BUS								
														
														USBRes(NO_ERR, CMD_GET,&ImageArrayBuf[u8temp1][i][0],u8temp);
															#if !defined(USB_WAIT_MODE)
															delay_ms(35);
															#else
															EpMsgEnable();
															UsbReadDone();
															EpMsgDisable();
															#endif
											#else
														CANRes(NO_ERR, CMD_GET,&ImageArrayBuf[u8temp1][i][0],u8temp);
														delay_ms(6);
											#endif												
													 }
													 break;
											}
											
											if(PCR_Fail_cnt[u8temp1] > MAX_FAIL_TIMOUT_CNT)  // check MAX timeout or not?  @2015-09-20
											{													
														for(i=0;i<PIX_TOTAL_ROW;i++)
														{
															ImageBuf[i][0]=(TYP_IMAGE | (u8temp1<<4));
															ImageBuf[i][1]=0xF1;                       // pcr fail code
												
															for(j=0;j<(PIX_TOTAL_COL<<1);j++)
																ImageBuf[i][j+2]= 0;//RowData[i]; 

															TransLen=(PIX_TOTAL_COL<<1)+2;
															
											#ifndef __USE_CAN_BUS				
															USBRes(ResCode, CMD_GET,&ImageBuf[i][0],TransLen);
															#if !defined(USB_WAIT_MODE)
															delay_ms(35);
															#else
															EpMsgEnable();
															UsbReadDone();
															EpMsgDisable();
															#endif
											#else
															CANRes(ResCode, CMD_GET,&ImageBuf[i][0],TransLen);
															delay_ms(6);
											#endif															
														}
													break;
											}
											
													//modify @ 140301:
													//required by Anitoa @140218, if inteTime ,10ms, use row mode	
											 /////////////////////////////2016-04-23 LED control separated
											 //LED_CTRL=1;
												if(led_indiv)  // not share mode
												{
													 switch(u8temp1)
													 {
														 case 0: LED_CTRL=1;
															break;
														 case 1: LED_CTRL_1=1;
															break;
														 case 2: LED_CTRL_2=1;
															break;
														 case 3: LED_CTRL_3=1;
															break;
														 default:
															break;											 
													 }
												}
												else 
													LED_CTRL=1;
												
								#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
											//version 1.13												
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_ON);
													#endif
								#endif
												
								#ifdef __USE_OSC_GATE   //version 1.13 
											Multi_OSC_Access(SPI_Sel,OSC_ON);
								#endif	
												
											delay_ms(6);
											if(PCR_Regs[SPI_Sel].InteTime>10)
											{		
												PixReadmMode=TYP_IMAGE;
												u8temp3= PCR_Fail_cnt[SPI_Sel];
												ReadUpdate_Image_multiPCR_oneRead_mode_0(u8temp1,0);
												//LED_CTRL=LED_CTRL_1=LED_CTRL_2=LED_CTRL_3=0;
												///////////2016-04-23 LED control separated, including the LED control mode, maybe forced 
												LED_CTRL = (led_mode & 0x1)? 1:0;
												if(led_indiv)		
												{																					
													LED_CTRL_1 = (led_mode & 0x2)? 1:0;
													LED_CTRL_2 = (led_mode & 0x4)? 1:0;
													LED_CTRL_3 = (led_mode & 0x8)? 1:0;
												}	
												
												if(u8temp3 < PCR_Fail_cnt[SPI_Sel])   // fail happened in this cycle, then overwrite packet
												{
														for(j=0;j<(PIX_TOTAL_ROW);j++)
														{
															ImageArrayBuf[SPI_Sel][j][0]=((TYP_IMAGE) | (SPI_Sel<<4));    
															ImageArrayBuf[SPI_Sel][j][1]= 0xF1;
															for(i=0;i<(PIX_TOTAL_COL<<1);i++)
																	ImageArrayBuf[SPI_Sel][j][i+2]=0;

														}
												}
												u8temp=(PIX_TOTAL_COL<<1)+2;
												for(i=0;i<(PIX_TOTAL_ROW);i++)
												{
													
											#ifndef __USE_CAN_BUS	
														USBRes(NO_ERR, CMD_GET,&ImageArrayBuf[u8temp1][i][0],u8temp);
														#if !defined(USB_WAIT_MODE)
														delay_ms(35);
														#else
														EpMsgEnable();
														UsbReadDone();
														EpMsgDisable();
														#endif
											#else
														CANRes(NO_ERR, CMD_GET,&ImageArrayBuf[u8temp1][i][0],u8temp);		
														delay_ms(6);
											#endif												
												 }
											}
											else
											{
													PixReadmMode=TYP_ROW;
													ResCode=NO_ERR;
													
													u8temp3= PCR_Fail_cnt[SPI_Sel];
													for(CurrentRowNum=0;CurrentRowNum<PIX_TOTAL_ROW;CurrentRowNum++)
													{
														
														Read_Row_multiPCR_mode_0(CurrentRowNum,SPI_Sel,(PIX_TOTAL_ROW-1-CurrentRowNum));
														/*
															TxBuffer[0]=(TYP_IMAGE | (SPI_Sel<<4));
														TxBuffer[1]=CurrentRowNum;
														for(i=0;i<(PIX_TOTAL_COL<<1);i++)
															TxBuffer[i+2]= SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015
														*/
														if(u8temp3 < PCR_Fail_cnt[SPI_Sel])
																break;
														else
														{
															for(i=0;i<(PIX_TOTAL_COL<<1);i++)
																ImageBuf[CurrentRowNum][i+2]= SPI_2_RcvBuf[i];//RowData[i]; 
														}
													}
													//LED_CTRL=LED_CTRL_1=LED_CTRL_2=LED_CTRL_3=0;
														///////////2016-04-23 LED control separated, including the LED control mode, maybe forced 
														LED_CTRL = (led_mode & 0x1)? 1:0;
														if(led_indiv)		
														{																					
															LED_CTRL_1 = (led_mode & 0x2)? 1:0;
															LED_CTRL_2 = (led_mode & 0x4)? 1:0;
															LED_CTRL_3 = (led_mode & 0x8)? 1:0;
														}	
													
													if(u8temp3 < PCR_Fail_cnt[SPI_Sel])
													{
														for(i=0;i<(PIX_TOTAL_ROW);i++)
														{
																ImageBuf[i][0]=(TYP_IMAGE | (SPI_Sel<<4));
																ImageBuf[i][1]=0xF1;
																for(j=0;j<(PIX_TOTAL_COL<<1);j++)
																	ImageBuf[i][j+2]=0;

														}
													}
													else
													{
														for(i=0;i<PIX_TOTAL_ROW;i++)
														{
															ImageBuf[i][0]=(TYP_IMAGE | (SPI_Sel<<4));
															ImageBuf[i][1]=i;
														}
													}
												u8temp=(PIX_TOTAL_COL<<1)+2;
												for(i=0;i<(PIX_TOTAL_ROW);i++)
												{
												#ifndef __USE_CAN_BUS	
														USBRes(NO_ERR, CMD_GET,&ImageBuf[i][0],u8temp);
														#if !defined(USB_WAIT_MODE)
														delay_ms(35);
														#else
														EpMsgEnable();
														UsbReadDone();
														EpMsgDisable();
														#endif	
												#else
														CANRes(NO_ERR, CMD_GET,&ImageBuf[i][0],u8temp);
														delay_ms(6);
												#endif 
												 }	
											}
											
									#ifdef __USE_OSC_GATE   //version 1.13
											if(OSC_mode != OSC_ON)
													Multi_OSC_Access(SPI_Sel,OSC_OFF);
									#endif	

									#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
													//version 1.13:
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_OFF);
													#endif
									#endif
									
											
											break;
	/*
										 case TYP_VIDEO:
											if(Comand_Buf[TYP_BYTE_NUM+1]==ENABLE)
											{
												PixReadmMode=TYP_VIDEO;
												VideoRow=0;
											}
											else
											{
												PixReadmMode=DISABLE; // stop reading image
												VideoRow=0;
											}
											break;
	*/
	/*
										case TYP_DEBUG_1:	  // XinChuang required to debug @ 13-10-22

											Send_Command(PCR_ADC_READ, 0, 0, &PCRChip_Command_Send[0]);	
											ResCode=NO_ERR;
												TxBuffer[0]=TYP_ROW;
											TxBuffer[1]=CurrentRowNum;
											for(i=0;i<(PIX_TOTAL_COL<<1);i++)
												TxBuffer[i+2]= SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015

											TransLen=(PIX_TOTAL_COL<<1)+2;
											USBRes(ResCode, CMD_GET,TxBuffer,TransLen);
											
											break;
	*/
	/*
										case TYP_DEBUG_2:
											PixReadmMode=TYP_ROW;
											Read_Row_Debug_2();	
											ResCode=NO_ERR;
												TxBuffer[0]=TYP_ROW;
											TxBuffer[1]=CurrentRowNum;
											for(i=0;i<(PIX_TOTAL_COL<<1);i++)
												TxBuffer[i+2]= SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015

											TransLen=(PIX_TOTAL_COL<<1)+2;
											USBRes(ResCode, CMD_GET,TxBuffer,TransLen);
											break;
	*/
	/*
										case TYP_24PIXROW:  //row

											//PixReadmMode=TYP_24PIXROW;
											PixReadmMode=TYP_24PIXIMAG;		  
											CurrentRowNum=Comand_Buf[TYP_BYTE_NUM+1];
											if(CurrentRowNum > 24)  
											{	//LPG:  invalid row number
												ResCode=ERR_ROW;
												TransLen=0;
												USBRes(ResCode, CMD_GET,TxBuffer,TransLen);
											}	
											else
											{
												
												if(PCR_Regs.InteTime>10)
													ReadUpdate_Image24_XC(TYP_24PIXROW,CurrentRowNum);
												else
													ReadUpdate_Image24_slow(TYP_24PIXROW,CurrentRowNum);
												
											}
											break;
	*/										 
										case TYP_24PIXIMAG:  //frame

													ResCode=NO_ERR;
													PixReadmMode=TYP_24PIXIMAG;
											
													if((PCR_Data_Valid_mode_1>>u8temp1) & 0x1)     // is this PCR triggered pending for read?
													{
															PCR_Data_Valid_mode_1 &= ~(1<<u8temp1);     // clear this pending
															PcrSTS &= ~(1<<u8temp1);
														
															for(i=0;i<24;i++)
															{
																	#ifndef __USE_CAN_BUS	
																			USBRes(NO_ERR, CMD_GET, &ImageArrayBuf[u8temp1][i][0], 52);
																			#if !defined(USB_WAIT_MODE)
																			delay_ms(35);
																			#else
																			EpMsgEnable();
																			UsbReadDone();
																			EpMsgDisable();
																			#endif	
																	#else
																			CANRes(NO_ERR, CMD_GET, &ImageArrayBuf[u8temp1][i][0], 52);
																			delay_ms(6);
																	#endif 
															 }
															 break;
													}		
											
													if(PCR_Fail_cnt[SPI_Sel] > MAX_FAIL_TIMOUT_CNT)  // check MAX timeout or not?  @2015-09-20
													{
																for(i=0;i<((PIX_TOTAL_ROW)<<1);i++)
																{
																			ImageBuf[i][0]=(TYP_24PIXIMAG|(SPI_Sel<<4));
																			ImageBuf[i][1]=0xF1;                       // pcr fail code
																			for(j=0;j<((PIX_TOTAL_COL)<<2);j++)
																				ImageBuf[i][j+2]= 0;//RowData[i]; 
																		
																			#ifndef __USE_CAN_BUS	
																					USBRes(ResCode, CMD_GET,&ImageBuf[i][0],52);
																					#if !defined(USB_WAIT_MODE)
																					delay_ms(35);
																					#else
																					EpMsgEnable();
																					UsbReadDone();
																					EpMsgDisable();
																					#endif	
																			#else
																					CANRes(ResCode, CMD_GET,&ImageBuf[i][0],52);
																					delay_ms(6);
																			#endif
																}
															break;
													}
										
													 /////////////////////////////2016-04-23 LED control separated
													 //LED_CTRL=1;
														if(led_indiv)  // not share mode
														{
															 switch(u8temp1)
															 {
																 case 0: LED_CTRL=1;
																	break;
																 case 1: LED_CTRL_1=1;
																	break;
																 case 2: LED_CTRL_2=1;
																	break;
																 case 3: LED_CTRL_3=1;
																	break;
																 default:
																	break;											 
															 }
														}
														else 
															LED_CTRL=1;
														
										#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
										//#ifdef __USE_MOS_GATE  //version 1.13
				
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_ON);
													#endif
										#endif

														
											#ifdef __USE_OSC_GATE   //version 1.13 
														Multi_OSC_Access(SPI_Sel,OSC_ON);
											#endif
														
														delay_ms(6);
														u8temp3 = PCR_Fail_cnt[SPI_Sel];
														if(PCR_Regs[SPI_Sel].InteTime>10)
														{
															
																//ReadUpdate_Image_multiPCR_oneRead_mode_1(SPI_Sel,0); /* access a image*/
																TrgChQue[0]= SPI_Sel;
																ReadUpdate_Image_multiPCR_PipeLine_mode_1(1); /* access a image*/
																//LED_CTRL=LED_CTRL_1=LED_CTRL_2=LED_CTRL_3=0;
																///////////2016-04-23 LED control separated, including the LED control mode, maybe forced 
																		LED_CTRL = (led_mode & 0x1)? 1:0;
																		if(led_indiv)		
																		{																					
																			LED_CTRL_1 = (led_mode & 0x2)? 1:0;
																			LED_CTRL_2 = (led_mode & 0x4)? 1:0;
																			LED_CTRL_3 = (led_mode & 0x8)? 1:0;
																		}	
																	
																
																if(u8temp3 < PCR_Fail_cnt[SPI_Sel])   // fail happened in this cycle, then overwrite packet
																{
																		for(j=0;j<((PIX_TOTAL_ROW)<<1);j++)
																		{
																			ImageArrayBuf[SPI_Sel][j][0]=(TYP_24PIXIMAG|(SPI_Sel<<4));   
																			ImageArrayBuf[SPI_Sel][j][1]=0xF1;
																			for(i=0;i<((PIX_TOTAL_COL)<<2);i++)
																					ImageArrayBuf[SPI_Sel][j][i+2]=0;
																		}
																}
														}
													else
													{
															PixReadmMode=TYP_ROW;
															ResCode=NO_ERR;	

															for(u8temp=0;u8temp<4;u8temp++)
															{																
																txpattern =  PIXEL_24READ[u8temp];
																PCRChip_Command_Send[20]=PCR_Regs[SPI_Sel].SW_TxCtrl=TX_PACK(txpattern,PCR_Regs[SPI_Sel].SW_TxCtrl);
																Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[20]);

																for(j=0;j<PIX_TOTAL_ROW;j++)
																{										
																	Read_Row_multiPCR_mode_0(j,SPI_Sel,(PIX_TOTAL_ROW-1-j));
																	
																	if(u8temp3 < PCR_Fail_cnt[SPI_Sel])
																		break;
																	
																for(i=0;i<(PIX_TOTAL_COL<<1);i++)
																		ImageBuf[j][i+2]= SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015
																	
																	sel=SPI_Sel;
																	switch(u8temp)
																	{
																		case 0:
																			//for(j=0;j<PIX_TOTAL_ROW;j++)
																			{
																				for(i=0;i<(PIX_TOTAL_COL);i++)
																				{
																					ImageArrayBuf[sel][(j<<1)][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
																					ImageArrayBuf[sel][(j<<1)][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
																				}
																			}	
																		break;

																		case 1:
																			//for(j=0;j<PIX_TOTAL_ROW;j++)
																			{

																				for(i=0;i<(PIX_TOTAL_COL);i++)
																				{
																					ImageArrayBuf[sel][(j<<1)][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
																					ImageArrayBuf[sel][(j<<1)][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
																				}	

																			}	
																		break;

																		case 2:
																			//for(j=0;j<PIX_TOTAL_ROW;j++)
																			{
																				for(i=0;i<(PIX_TOTAL_COL);i++)
																				{
																					ImageArrayBuf[sel][(j<<1)+1][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
																					ImageArrayBuf[sel][(j<<1)+1][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
																				}
																			}	
																		break;

																		case 3:
																			//for(j=0;j<PIX_TOTAL_ROW;j++)
																			{
																				for(i=0;i<(PIX_TOTAL_COL);i++)
																				{
																					ImageArrayBuf[sel][(j<<1)+1][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
																					ImageArrayBuf[sel][(j<<1)+1][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
																				}
																			}	
																		break;
																	
																		default:
																		break;
																	}	
																}	
																if(u8temp3 < PCR_Fail_cnt[SPI_Sel])
																{
																	for(j=0;j<((PIX_TOTAL_ROW)<<1);j++)
																	{
																		ImageArrayBuf[SPI_Sel][j][0]=(TYP_24PIXIMAG|(SPI_Sel<<4));   
																		ImageArrayBuf[SPI_Sel][j][1]=0xF1;
																		for(i=0;i<((PIX_TOTAL_COL)<<2);i++)
																				ImageArrayBuf[SPI_Sel][j][i+2]=0;
																	}
																	break;
																}
														}	
														//LED_CTRL=LED_CTRL_1=LED_CTRL_2=LED_CTRL_3=0;
														///////////2016-04-23 LED control separated, including the LED control mode, maybe forced 
																LED_CTRL = (led_mode & 0x1)? 1:0;
																if(led_indiv)		
																{																					
																	LED_CTRL_1 = (led_mode & 0x2)? 1:0;
																	LED_CTRL_2 = (led_mode & 0x4)? 1:0;
																	LED_CTRL_3 = (led_mode & 0x8)? 1:0;
																}																
													}
									
								#ifdef __USE_OSC_GATE   //version 1.13
											if(OSC_mode != OSC_ON)
													Multi_OSC_Access(SPI_Sel,OSC_OFF);
								#endif
											
								#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
												//version 1.13
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_OFF);
													#endif
								#endif													
											for(i=0;i<24;i++)
											{	 
												 ImageArrayBuf[SPI_Sel][i][0]=(TYP_24PIXIMAG|(SPI_Sel<<4));
												 if(u8temp3 < PCR_Fail_cnt[SPI_Sel])
														ImageArrayBuf[SPI_Sel][i][1] = 0xF1;
												 else
														ImageArrayBuf[SPI_Sel][i][1] = i;
											}
											for(i=0;i<24;i++)
													{
													#ifndef __USE_CAN_BUS	
															USBRes(NO_ERR, CMD_GET, &ImageArrayBuf[SPI_Sel][i][0], 52);
															#if !defined(USB_WAIT_MODE)
															delay_ms(35);
															#else
															EpMsgEnable();
															UsbReadDone();
															EpMsgDisable();
															#endif	
													#else
															CANRes(NO_ERR, CMD_GET, &ImageArrayBuf[SPI_Sel][i][0], 52);
															delay_ms(6);
													#endif
													 }
											PixReadmMode=DISABLE;
											break;


										default:
											break;
									
									}
								
							break;
		

						case  CMD_SET:
											ResCode=NO_ERR;
											TransLen=1;
											TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];

								#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
												//version 1.18												
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_ON);
													#endif
								#endif

						
											if(Comand_Buf[LEN_BYTE_NUM]<MIN_PACKET_LENGTH)
													ResCode=ERR_LENGTH;
											else
											{
													switch (Comand_Buf[TYP_BYTE_NUM])
													{
									 
																case RAMP_TRIM:	// 0x1	
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].RampTrim=RAMP_PACK(u8temp,PCR_Regs[SPI_Sel].RampTrim);
																					Send_Command(REG_WRITE, RAMP_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
																					//TransLen=0;
																break;
		
		
																case RANG_TRIM: // 0x2
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].RangTrim=RANG_PACK(u8temp,PCR_Regs[SPI_Sel].RangTrim);
																					Send_Command(REG_WRITE, RANG_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
																					//TransLen=0;
																break;
		
																case V24_TRIM:	//0x3
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].Ipix_V24Trim=V24_PACK(u8temp,PCR_Regs[SPI_Sel].Ipix_V24Trim);
																					Send_Command(REG_WRITE, V24_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);											
																					//TransLen=0;							
																break;
											
																case V20_TRIM:	//0x4
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].V20_V15Trim=V20_PACK(u8temp,PCR_Regs[SPI_Sel].V20_V15Trim);
																					Send_Command(REG_WRITE, V20_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
																					//TransLen=0;							
																	break;
											
																case V15_TRIM:	//0x5
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].V20_V15Trim=V15_PACK(u8temp,PCR_Regs[SPI_Sel].V20_V15Trim);
																					Send_Command(REG_WRITE, V15_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
																					//TransLen=0;	
																	break;
		
																case IPIX_TRIM: //0x6
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].Ipix_V24Trim=IPIX_PACK(u8temp,PCR_Regs[SPI_Sel].Ipix_V24Trim);
																					Send_Command(REG_WRITE, IPIX_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
																					//TransLen=0;									
																		break;
		
																case SWITCH_BIT:	//0x7
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					/*
																							PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].SW_TxCtrl=SW_PACK(u8temp,PCR_Regs[SPI_Sel].SW_TxCtrl);
																							Send_Command(REG_WRITE, SWITCH_ADDR, 1, &PCRChip_Command_Send[0]);
																					*/
																					///////to fix YiFei bug, which sent 1 gain only , not by a selection preceded
																					SPI_Sel_Reg=SPI_Sel;
												#ifdef  __INTEGRATION_INDIV
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].SW_TxCtrl=SW_PACK(u8temp,PCR_Regs[SPI_Sel].SW_TxCtrl);
																					Send_Command(REG_WRITE, SWITCH_ADDR, 1, &PCRChip_Command_Send[0]);
												#else
																					for(i=0;i<4;i++)
																					{
																							
																							SPI_Sel=i;
																							PCRChip_Command_Send[0]=PCR_Regs[i].SW_TxCtrl=SW_PACK(u8temp,PCR_Regs[i].SW_TxCtrl);
																							Send_Command(REG_WRITE, SWITCH_ADDR, 1, &PCRChip_Command_Send[0]);	
																					}
												#endif
																					SPI_Sel=SPI_Sel_Reg;
																					//TransLen=0;	
																	break;
		
																case TX_PATTERN:	//0x8
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].SW_TxCtrl=TX_PACK(u8temp,PCR_Regs[SPI_Sel].SW_TxCtrl);
																					Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[0]);
																					//TransLen=0;
																break;
		
																case AMUX_CONTROL:	//0x9
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCRChip_Command_Send[0]=PCR_Regs[SPI_Sel].TsTADC_AmuxCtrl=AMUX_PACK(u8temp,PCR_Regs[SPI_Sel].TsTADC_AmuxCtrl);
																					Send_Command(REG_WRITE, AMUX_CONTROL_ADDR, 1, &PCRChip_Command_Send[0]);
																					//TransLen=0;
																	break;
		
																case TEST_ADC:		//0xA
																		#if !defined(BETA_VER)
																					ResCode=NO_ERR;
																					u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																					PCR_Regs[SPI_Sel].TsTADC_AmuxCtrl=TSTADC_PACK(u8temp,PCR_Regs[SPI_Sel].TsTADC_AmuxCtrl);
																					//TransLen=0;
																		#else
																					ResCode=ERR_CMD;   /*LPG: no test adc in beta version, return error*/
																				//TransLen=0;
																		#endif
																	break;
		
																case SPI_PULSE:		//0x10
																					ResCode=NO_ERR;
																					PCR_Regs[SPI_Sel].SpiInsertion=Comand_Buf[TYP_BYTE_NUM+1];
																					//TransLen=0;
																	break;
		
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
											#ifdef  __INTEGRATION_INDIV
											////// integration independend
																							PCR_Regs[SPI_Sel].InteTime=UnionTemp.float_num;
											#else
											///// integration shared
																							for(i=0;i<MAX_PCR_CH;i++)
																							PCR_Regs[i].InteTime=UnionTemp.float_num;
											///// integration shared
											#endif
											
											//version 1.8: 		debug PCR_ADC																	
											#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))
													#ifdef __USE_PCR_ADC_DELAY_DEBUG	
																						
															if(f_MOS_delay< UnionTemp.float_num)
																		f_MOS_delay=UnionTemp.float_num;
															else if(UnionTemp.float_num==1)
																				f_MOS_delay=300;
													#endif
											#endif												
																					}
																					else
	
																							ResCode=BAD_DATA;	

																					//TransLen=0;
																		break;
		
																case PCR_RESET:	  // 0xF
																					ResCode=NO_ERR;
																					PCR_RESET_PROCESS;
																					Trim_Reset();
																					//TransLen=0;
																	break;

																case OSC_CONTROL:   // 0xE
																		ResCode=NO_ERR;
																		u8temp=Comand_Buf[TYP_BYTE_NUM+1];
																		u8temp &= 0x3;
																		OSC_mode= (u8temp != 0x3)? u8temp: OSC_AUTO;  
																									//version 1.13
																									// ON, OFF, AUTO
																		if(u8temp==OSC_ON)	 // ON
																		{
																			OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_ON;
																			OSC_Status=0xF;
																		}
																		else if (u8temp==OSC_OFF)	// Off																					 
																					{
																						//if(OSC_Busy != 1)	 // customer insure not to disable OSC during ADC
																						OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_OFF;
																						OSC_Status=0;
																					}
																					else // 0x2: auto
																							 // 0x3: invalid mode, force to auto				  
																							OSC_mode=OSC_AUTO; 	
																						
																		//TransLen=0;
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

																					//TransLen=0;
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
																					
																					//TransLen=0;
																	break;

																 case LED_SWITCH:
																						ResCode=NO_ERR;
																						//TransLen=0;
																						u8temp3=Comand_Buf[TYP_BYTE_NUM+1];
																				 
																						/////////////2016-04-23 LED control separated
																						//LED_CTRL=led_mode=(u8temp3 & 0x1);
																						led_mode=u8temp3;  // so here change the defintion to extend 4 channle bits
																						led_indiv=(u8temp3 & 0x80);
																						LED_CTRL=LED_CTRL_1=LED_CTRL_2=LED_CTRL_3=0;
																						if(led_indiv)  // mode 2, independent LED mode
																						{
																							LED_CTRL = (led_mode & 0x1)? 1:0;
																							LED_CTRL_1 = (led_mode & 0x2)? 1:0;
																							LED_CTRL_2 = (led_mode & 0x4)? 1:0;
																							LED_CTRL_3 = (led_mode & 0x8)? 1:0;
																						}
																						else
																						{
																							LED_CTRL=(led_mode & 0x01); 
																						}
																	break;									 	

																 case TYP_PCR_TRG_MASK:    // trigger IRQ mask
																						ResCode=NO_ERR;
																						//TransLen=1;
																						//TxBuffer[0]=TYP_PCR_TRG_MASK;	
																						PcrMskReg=(Comand_Buf[TYP_BYTE_NUM+1] & 0xF);		// mask byte	

																						if(PcrMskReg==0)      // disable all Channel
																								PcrTrgDisable();
																						else
																						{
																							Image_capture_enable();
																							PcrMskMapping(PcrMskReg);
																							EXTI->PR |= 0xffff;
																						}
																	break;
										 
																 case TYE_TRG_PIX_MODE:  // set into 24 pixel mode 
																						ResCode=NO_ERR;
																						u8temp3=Comand_Buf[TYP_BYTE_NUM+1];
																				 if(u8temp3>1)             // 0: 12PIX, 1: 24PIX
																							ResCode=OUT_RANGE;
																					else
																							PixMode=u8temp3;
																					//TransLen=1;
																					//TxBuffer[0]=TYE_TRG_PIX_MODE;										
																	break;	

																 case TYP_PCR_SEL:
																					ResCode=NO_ERR;
																					//TransLen=1;
																					//TxBuffer[0]=TYP_PCR_SEL;
																				if(Comand_Buf[TYP_BYTE_NUM+1]<MAX_PCR_CH)
																					SPI_Sel=SPI_Sel_Reg=Comand_Buf[TYP_BYTE_NUM+1];	
																				else
																					ResCode=OUT_RANGE;
																	break;
	////////////////////////////////// test version @2016-03-26
																case TYP_IMG_EN:
																					ResCode=NO_ERR;
																					//TransLen=1;
																					//TxBuffer[0]=TYP_IMG_EN;
																					Img_Enable= Comand_Buf[TYP_BYTE_NUM+1]& 0x1;
																					if(Img_Enable)
																							Image_capture_enable();
																					else
																					{
																							Image_capture_disable();
																							//PCR_Data_Valid_mode_0=PCR_Data_Valid_mode_1=PcrSTS=0;
																					}
																					PCR_Data_Valid_mode_0=PCR_Data_Valid_mode_1=PcrSTS=0;
																				//else
																				//	ResCode=OUT_RANGE;
																	break;
	////////////////////////////////////////////////////////////////
			
																	default:  			// invalid type 
																					ResCode=ERR_TYP;
																					//TransLen=0;
																	break;
														}
											}
						#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
										//version 1.18												
													#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																MOS_Gate_Ctrl(OSC_OFF);
													#endif
						#endif											
											
											
						#ifndef __USE_CAN_BUS	
											USBRes(ResCode, CMD_SET, TxBuffer, TransLen);
						#else
											CANRes(ResCode, CMD_SET, TxBuffer, TransLen); // send ACK no error						 
						#endif
									break;


						case CMD_READ:	
										ResCode=NO_ERR;
										TransLen=1;
										TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];
										if(Comand_Buf[LEN_BYTE_NUM]<MIN_PACKET_LENGTH)
												ResCode=ERR_LENGTH;
										else
										{
												switch (Comand_Buf[TYP_BYTE_NUM])
												{
														case RAMP_TRIM:	// 0x1
																		ResCode=NO_ERR;
																		TxBuffer[0]=RAMP_TRIM;
																		TxBuffer[1]=RAMP_ALGN(PCR_Regs[SPI_Sel].RampTrim);
																		TransLen=2;
														break;

														case RANG_TRIM: // 0x2
																		ResCode=NO_ERR;
																		TxBuffer[0]=RANG_TRIM;
																		TxBuffer[1]=RANG_ALGN(PCR_Regs[SPI_Sel].RangTrim);
																		TransLen=2;
															break;
		
														case V24_TRIM:	//0x3
																		ResCode=NO_ERR;
																		TxBuffer[0]=V24_TRIM;
																		TxBuffer[1]=V24_ALGN(PCR_Regs[SPI_Sel].Ipix_V24Trim);
																		TransLen=2;								
															break;
											
														case V20_TRIM:	//0x4
																		ResCode=NO_ERR;
																		TxBuffer[0]=V20_TRIM;
																		TxBuffer[1]=V20_ALGN(PCR_Regs[SPI_Sel].V20_V15Trim);
																		TransLen=2;								
															break;
											
														case V15_TRIM:	//0x5
																		ResCode=NO_ERR;
																		TxBuffer[0]=V15_TRIM;
																		TxBuffer[1]=V15_ALGN(PCR_Regs[SPI_Sel].V20_V15Trim);
																		TransLen=2;	
															break;
		
														case IPIX_TRIM: //0x6
																		ResCode=NO_ERR;
																		TxBuffer[0]=IPIX_TRIM;
																		TxBuffer[1]=IPIX_ALGN(PCR_Regs[SPI_Sel].Ipix_V24Trim);
																		TransLen=2;									
														break;
		
														case SWITCH_BIT:	//0x7
																		ResCode=NO_ERR;
																		TxBuffer[0]=SWITCH_BIT;
																		TxBuffer[1]=SW_ALGN(PCR_Regs[SPI_Sel].SW_TxCtrl);
																		TransLen=2;	
															break;
		
													case TX_PATTERN:	//0x8
																		ResCode=NO_ERR;
																		TxBuffer[0]=TX_PATTERN;
																		TxBuffer[1]=TX_ALGN(PCR_Regs[SPI_Sel].SW_TxCtrl);
																		TransLen=2;
														break;
		
													case AMUX_CONTROL:	//0x9
																		ResCode=NO_ERR;
																		TxBuffer[0]=AMUX_CONTROL;
																		TxBuffer[1]=AMUX_ALGN(PCR_Regs[SPI_Sel].TsTADC_AmuxCtrl);
																		TransLen=2;
														break;
		
													case TEST_ADC:		//0xA
																		#if !defined(BETA_VER) 
																			ResCode=NO_ERR;
																			TxBuffer[0]=TEST_ADC;
																			TxBuffer[1]=TSTADC_ALGN(PCR_Regs[SPI_Sel].TsTADC_AmuxCtrl);
																			TransLen=2;
																		#else
																			ResCode=ERR_CMD;   /*LPG: no test adc in beta version, return error*/
																		#endif
														break;
		

													case OSC_CONTROL:
																		ResCode=NO_ERR;
																		TxBuffer[0]=OSC_CONTROL;
																		TxBuffer[1]=OSC_Status;
																		TransLen=2;										
															break;

												case SPI_PULSE:		//0x10
																		ResCode=NO_ERR;
																		TxBuffer[0]=SPI_PULSE;
																		TxBuffer[1]=PCR_Regs[SPI_Sel].SpiInsertion;
																		TransLen=2;
													break;
		
												case INTE_TIME:		//0x20
																		ResCode=NO_ERR;
																		TxBuffer[0]=INTE_TIME;

																		UnionTemp.float_num= PCR_Regs[SPI_Sel].InteTime;
																		TxBuffer[1]= UnionTemp.tempd.byte0;
																		TxBuffer[2]= UnionTemp.tempd.byte1;
																		TxBuffer[3]= UnionTemp.tempd.byte2;
																		TxBuffer[4]= UnionTemp.tempd.byte3;
																		TransLen=5;
													break;
		

												case  AMUX_OUT:
													
												// version 1.18
													#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																	//version 1.13
																	#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																				MOS_Gate_Ctrl(OSC_ON);
																	#endif
													#endif													
																	switch(SPI_Sel)
																	 {
																		 case 0:
																			 adcx=Get_Adc_Average(ADC_CH1,14);
																			break;
																		 case 1:
																			 adcx=Get_Adc_Average(ADC_CH1,15);
																			break;
																		 case 2:
																			 adcx=Get_Adc_Average(ADC_CH1,8);
																			break;
																		 case 3:
																			 adcx=Get_Adc_Average(ADC_CH1,9);
																			break;
																		 
																		 default:
																			 adcx=0;
																			break;												 
																	 }
																 //	adcx=Get_Adc_Average(ADC_CH1,10);
																//temp=(float)adcx*(3.3/4096);
																//adcx=temp;
																		ResCode=NO_ERR;
																		TxBuffer[0]=AMUX_OUT;
																		TxBuffer[1]= (u8)adcx;
																		TxBuffer[2]= (u8)(adcx>>8);
																		TransLen=3;	
												#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																//version 1.18												
															#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																		MOS_Gate_Ctrl(OSC_OFF);
															#endif
												#endif

																	 
													break;

												case PCR_TEMP:  
																	ResCode=NO_ERR;
												// version 1.16
													#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																	//version 1.13
																#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																			MOS_Gate_Ctrl(OSC_ON);
												
																		#ifdef __USE_PCR_ADC_DELAY_DEBUG																							
																					delay_ms((u16) f_MOS_delay);
																		#else
																					delay_ms(300);  // version 1.18
																		#endif
												
																#endif
												
																	//version 1.8: 		debug PCR_ADC																	
												
												
												
												
																	fTemp=0;
																	///////version 1.19 
																	switch(SPI_Sel)
																	 {
																		 case 0:
																			 //adcx=Get_Adc_Average(ADC_CH10,10);
																				
																		    if(PCR_temp[0]==0) 
																				{
																					adcx=Get_Adc(ADC_CH10);
																					PCR_temp[0]= adcx * 3.3/4096;
																				}
																				
																		    for(i=0;i<50;i++)
																		    {
																					adcx=Get_Adc(ADC_CH10);
																					fTemp+= adcx * 3.3/4096;
																					delay_ms(5);  ///////version 1.19 
																				}
																				fTemp=fTemp/50;
																				
																			break;
																		 case 1:
																			 //adcx=Get_Adc_Average(ADC_CH11,10);
																		 																			
																		    if(PCR_temp[1]==0) 
																				{
																					adcx=Get_Adc(ADC_CH11);
																					PCR_temp[1]= adcx * 3.3/4096;
																				}
																				
																		    for(i=0;i<50;i++)
																		    {
																					adcx=Get_Adc(ADC_CH11);
																					fTemp+= adcx * 3.3/4096;
																				}
																				fTemp=fTemp/50;																		 
																		 
																			break;
																		 case 2:
																		    if(PCR_temp[2]==0) 
																				{
																					adcx=Get_Adc(ADC_CH12);
																					PCR_temp[2]= adcx * 3.3/4096;
																				}
																				
																		    for(i=0;i<50;i++)
																		    {
																					adcx=Get_Adc(ADC_CH12);
																					fTemp+= adcx * 3.3/4096;
																				}
																				fTemp=fTemp/50;	
																			break;
																		 case 3:
																			 	if(PCR_temp[3]==0) 
																				{
																					adcx=Get_Adc(ADC_CH13);
																					PCR_temp[3]= adcx * 3.3/4096;
																				}
																				
																		    for(i=0;i<50;i++)
																		    {
																					adcx=Get_Adc(ADC_CH13);
																					fTemp+= adcx * 3.3/4096;
																				}
																				fTemp=fTemp/50;	
																			break;
																		 
																		 default:
																			 ResCode=OUT_RANGE;
																			break;												 
																	 }
																	 
																	if(SPI_Sel<4)
																			//PCR_temp[SPI_Sel] = PCR_temp[SPI_Sel] + 0.1*(fTemp - PCR_temp[SPI_Sel]);
																			PCR_temp[SPI_Sel]=fTemp;
												#endif	
																	//if(Comand_Buf[TYP_BYTE_NUM+2]== CH_INDEX_0)
																	 //		adcx=Get_Adc(ADC_CH14);
																	//else if(Comand_Buf[TYP_BYTE_NUM+2]== CH_INDEX_1)
																	//	adcx=Get_Adc(ADC_CH15);
												// version 1.16
													#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
																	//version 1.13
																	#if (!defined(__USE_POWER_FOR_EVER))  // version 1.18
																			MOS_Gate_Ctrl(OSC_OFF);
																	#endif
													#endif	

																	UnionTemp.float_num=PCR_temp[SPI_Sel];//(float)adcx*(3.3/4096);
																	TxBuffer[0]= PCR_TEMP;
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
										
												 case LED_SWITCH:
																ResCode=NO_ERR;
																TransLen=2;
																TxBuffer[0]=LED_SWITCH;
																TxBuffer[1]=led_mode; 
														 
													break;		
										
												 case TYP_PCR_TRG_MASK:    // trigger IRQ mask
																ResCode=NO_ERR;
																TransLen=2;
																TxBuffer[0]=	TYP_PCR_TRG_MASK;
																TxBuffer[1]=	PcrMskReg;
													break;
								 
												 case TYE_TRG_PIX_MODE:  // just to comply with set command, so twi type are shared to read  PixMode
																ResCode=NO_ERR;
																TransLen=2;
																TxBuffer[0]=	Comand_Buf[TYP_BYTE_NUM+1];
																TxBuffer[1]=	PixMode;
													break;	
							 
											 case TYP_PCR_SEL:
															ResCode=NO_ERR;
															TransLen=2;
															TxBuffer[0]=TYP_PCR_SEL;
															TxBuffer[1]=	SPI_Sel;
												break;									
											 case TYP_VER_INFO:   // get F/W information
															ResCode=NO_ERR;
															TransLen=7;
															TxBuffer[0]=TYP_VER_INFO;
															TxBuffer[1]=FUNC_G4_OVERLAP;		//  4xchip image control
															TxBuffer[2]=VERSION_INFO_MSB;	
															TxBuffer[3]=VERSION_INFO_LSB;	
															TxBuffer[4]=YEAR_INFO;		//  
															TxBuffer[5]=MONTH_INFO;		//  
															TxBuffer[6]=DATE_INFO;		// 
													break;
										 
												case TYP_IMG_EN:
																ResCode=NO_ERR;
																TransLen=2;
																TxBuffer[0]=	TYP_IMG_EN;
																TxBuffer[1]=	Img_Enable;
														break;
										 
												default:  			// invalid type 
													ResCode=ERR_TYP;
													//TransLen=0;
													break;
										}
								 }
				#ifndef __USE_CAN_BUS	
								 USBRes(ResCode, CMD_READ, TxBuffer, TransLen);
				#else
								 CANRes(ResCode, CMD_READ, TxBuffer, TransLen); // send ACK no error	
				#endif
							break;

						case CMD_HOST_NOTIFY:
									
										ResCode=NO_ERR;
										TransLen=1;
										TxBuffer[0]=Comand_Buf[TYP_BYTE_NUM];
										switch (Comand_Buf[TYP_BYTE_NUM])
										{
											
												case TRG_VALID:	// trigger host notify
																TxBuffer[0]=TRG_VALID;
																TxBuffer[1]=PcrSTS;
																TransLen=2;
												#ifndef __QPACR_HW
														#ifdef  __DEBUG_ON_OLD_HW
																		if(PcrSTS) LED_D2=!LED_D2;
														#endif
												#endif
												break;
											
												default:
															ResCode=ERR_TYP;
													//TransLen=1;
												break;
											
										}
									
	#ifndef __USE_CAN_BUS	
								 USBRes(ResCode, CMD_HOST_NOTIFY, TxBuffer, TransLen);
	#else
								 CANRes(ResCode, CMD_HOST_NOTIFY, TxBuffer, TransLen); // send ACK no error	
	#endif									
										
							break;

						default:
							break;
					
					}
				}		
			}


			tick++;
			//delay_ms(10);

			if(SPI_2_Rx_sts==SPI_CMPLT)
			{
				for(i=0;i<test_lenth;i++)
					u8buff[i]=SPI_2_RcvBuf[i];
				SPI_2_Rx_sts=SPI_IDLE;
			}
	/*		
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
	*/		
#ifndef __QPCR_HW
/*
			if(tick==200)
			{
				 ledcount++;
				 if(ledcount==100)
					LED_D1=!LED_D1;
				 if(ledcount==200)
				 { 
					 LED_D2=!LED_D2;
					 ledcount=0;
				 }
			 
				tick=0;
			}
*/
#endif
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




#ifdef __QPCR_HW
	void EXTI9_5_IRQHandler(void)
	{
		u32 temp=EXTI->PR;
		if(temp & (0xf<<5))	
		{		
			PCR_ADC_Done_Flag |=((temp>>5)& 0xf);
		}
		EXTI->PR |= (0xFFFF); 
	}
	
	void EXTI15_10_IRQHandler(void)
	{
		u32 temp;
		u8 i;
		u32 temp2;
		temp = ((EXTI->PR 	& (0xf<<10))>>10);
		if(temp) // valid trg happen
		{
			for(i=0; i<100; i++){};
			temp = ((EXTI->PR 	& (0xf<<10))>>10);	
			temp2= ((GPIOC->IDR & (0xf<<10))>>10);
			temp &= temp2;
			if(temp)
			{
				MsgTrgPush((MSG_TYP)temp);
				Exti=1;
			}
		}
		EXTI->PR |= 0xffff;
	} 	
	
#else
	void EXTI9_5_IRQHandler(void)
	{
		u32 temp;
		u8 i;
	/*
		if(EXTI->PR & (0xf<<6)) // valid trg happen
		{
			for(i=0; i<40; i++)
			{}
			temp=((EXTI->PR & (0xf<<6))>>6);
			MsgTrgPush((MSG_TYP)temp);
			Exti=1;
		}
	*/
		u32 temp2;
		temp = ((EXTI->PR 	& (0xf<<6))>>6);
		//if(temp & (0xf<<6)) // valid trg happen
		if(temp) // valid trg happen
		{
			for(i=0; i<240; i++){};
			temp = ((EXTI->PR 	& (0xf<<6))>>6);	
			temp2= ((GPIOF->IDR & (0xf<<6))>>6);
			temp &= temp2;
			if(temp)
			{
				MsgTrgPush((MSG_TYP)temp);
				Exti=1;
			}
		}
		EXTI->PR |= 0xffff;
	}

	
	void EXTI0_IRQHandler(void)
	{
																			
		EXTI->PR=1;  //清除中断标志位	  
		PCR_ADC_Done_Flag |=1;
	} 
	
	void EXTI15_10_IRQHandler(void)

	{
		u32 temp=EXTI->PR;
		if(temp & (7<<13))	
		{		
			PCR_ADC_Done_Flag |=((temp>>12)& 0xe);
		}
		EXTI->PR |= (7<<13); 
	} 
#endif
	
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

	/*LPG:  read a row through SPI */
	/*      return after SPI is done*/

	#define PXL_PHASE	11



	void Trim_Reset(void)
	{
		u8 i;
		SPI_Sel_Reg=SPI_Sel;
		for(i=0;i<MAX_PCR_CH;i++)
		{
		PCR_Regs[i].RampTrim=0x88;
		PCR_Regs[i].RangTrim=0x08;
		PCR_Regs[i].Ipix_V24Trim=0x88;
		PCR_Regs[i].V20_V15Trim=0x88;
		PCR_Regs[i].SW_TxCtrl=0x08;
		PCR_Regs[i].TsTADC_AmuxCtrl=0x00;
		BaseCounter=PCR_Regs[i].InteTime=DEFAULT_INTE ;
		/*
		SPI_Sel=i;
		PCRChip_Command_Send[0]=PCR_Regs[i].SW_TxCtrl;
		Send_Command(REG_WRITE, SWITCH_ADDR, 1, &PCRChip_Command_Send[0]);	
			
		PCRChip_Command_Send[0]=PCR_Regs[i].RampTrim;
		Send_Command(REG_WRITE, RAMP_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);		
			
		PCRChip_Command_Send[0]=PCR_Regs[i].RangTrim;
		Send_Command(REG_WRITE, RANG_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
			
		PCRChip_Command_Send[0]=PCR_Regs[i].Ipix_V24Trim;
		Send_Command(REG_WRITE, V24_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);	
			
		PCRChip_Command_Send[0]=PCR_Regs[i].V20_V15Trim;
		Send_Command(REG_WRITE, V20_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
		
		PCRChip_Command_Send[0]=PCR_Regs[i].V20_V15Trim;
		Send_Command(REG_WRITE, V15_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
		
		PCRChip_Command_Send[0]=PCR_Regs[i].Ipix_V24Trim;
		Send_Command(REG_WRITE, IPIX_TRIM_ADDR, 1, &PCRChip_Command_Send[0]);
		
		PCRChip_Command_Send[0]=PCR_Regs[i].SW_TxCtrl;
		Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[0]);	
		*/
		}
		SPI_Sel=SPI_Sel_Reg;
	}	

	/*	LPG: new feature @ 13-10-23: */
	/* 	ADC convert & SPI  read, without pixel driving*/
	void Read_Row_Debug_2(void)
	{


		OSC_Ctrl=OSC_ON;
		OSC_Status=1;

		delay_ms(10);
		Send_Command(ADC_START, 0, 0, &PCRChip_Command_Send[0]);
		PCR_ADC_Done_Flag=0;
		while(PCR_ADC_Done_Flag==0);
		if(OSC_mode != OSC_ON)
		{
			OSC_Ctrl=OSC_OFF;
			OSC_Status=0;
		}	 
		Send_Command(PCR_ADC_READ, 0, 0, &PCRChip_Command_Send[0]);
	}	

	void Fan_Echo(u8 dat)
	{
		TxBuffer[0]= dat;
		#ifndef __USE_CAN_BUS
			USBRes(NO_ERR, CMD_TEMP_SET, TxBuffer, 1);
		#else
			CANRes(NO_ERR, CMD_TEMP_SET, TxBuffer, 1);
		#endif
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
		do{			delay_ms(32);
				msg=EpMsgPop();}while(msg==0);
		}
		return msg;
	}
	#endif


	//////////////////////////////////////////////////////////////////
	//  12 pixel read function for multi PCR
	//  read row:  
	//             Read_Row_multiPCR_mode_0(u8 row, u8 sel, u8 Is_continue)
	//
	//  chip pipeline read:  integration T > n*10
	//             ReadUpdate_Image_multiPCR_PipeLine_mode_0(u8 PcrReg,u8 cnt)
	//
	//  single chip pipeline:  10ms < integration T  < n*10ms
	//            ReadUpdate_Image_multiPCR_oneRead_mode_0(u8 sel, u8 ContinueMode) 
	//
	//  slow read: integration T <10ms , actually read row by row
	//            ReadUpdate_Image_multiPCR_SlowRead_mode_0(u8 sel, u8 ContinueMode)
	///////////////////////////////////////////////////////////////////

	void Read_Row_multiPCR_mode_0(u8 row, u8 sel,u8 Is_continue)
	{
		u8 i, u8temp;
		SPI_Sel=sel;

	//		Multi_OSC_Access(sel,OSC_ON);
			for(i=0;i<11;i++)
				PCRChip_Command_Send[i]=PCRChip_Pattern[i];
	 
			for(i=0;i<=10;i++)
			{
				if(i==4)
				{
					while(TMR_Int_Flag==0);
	//				OSC_Ctrl=OSC_ON;
	//				OSC_Status=1;
				}
				Send_Command(PXL_DRIVE, row, 1, &PCRChip_Command_Send[i]);
				
				if(i==3)
				{
					TIM3_ARR_Update(PCR_Regs[SPI_Sel].InteCount);
					tempReg=BaseCounter= PCR_Regs[SPI_Sel].InteTime-1;
					TMR_Int_Flag=0;
					TIM3->CR1|=0x01; 
				}
				
			}
	//		delay_ms(10);
			Send_Command(ADC_START, 0, 0, &PCRChip_Command_Send[0]);
			PCR_ADC_Done_Flag=0;
	//FAN=1;
			timeout_initial(3);
			u8temp=0;
			while(((PCR_ADC_Done_Flag & (1<<SPI_Sel))==0) && (u8temp ==0)) {u8temp=timeout_check();};
			timeout_stop(1);
	//FAN=0;
			if(PCR_ADC_Done_Flag & (1<<SPI_Sel))  // ADC ready
			{			
				PCR_ADC_Done_Flag=0;
				Send_Command(PCR_ADC_READ, 0, 0, &PCRChip_Command_Send[0]);
				PCR_Fail_cnt[SPI_Sel]=0;
			}
			else if(u8temp !=0)                   // time out
						{  
								PCR_Fail_cnt[SPI_Sel] ++;	   						
						}
			 
		if(!Is_continue)
		{
			/*
			if(OSC_mode != OSC_ON)
			{
				LED_CTRL=(0x1 & led_mode);
				Multi_OSC_Access(sel,OSC_OFF);
			}	
			*/
			//SPI_Sel=SPI_Sel_Reg;
		}	
		
	}


	void ReadUpdate_Image_multiPCR_PipeLine_mode_0(u8 cnt)
	{
		 u8 i,j,k,u8temp,sel;
		 multi_overlap_ctrl=0x80;
		
		 for(i=0;i<11;i++)
				PCRChip_Command_Send[i]=PCRChip_Pattern[i]; 
		 
		 for(k=0;k<cnt;k++)
		 {
				sel=SPI_Sel=TrgChQue[k];
				if(k!=0)
					delay_ms(INTERVAL_DELAY_MS);
				//TIM3_ARR_Update(100);   // interrupt @ 1ms
				//tempReg = BaseCounter = PCR_Regs[SPI_Sel].InteTime-1;
			/*
	#ifdef _OVERLAP_TEST
				switch(k)
				{
					case 0: LED3=1; break;
					case 1: LED4=1; break;
					case 2: LED5=1; break;
					default: break;
				}				
	#endif
			 */
				for(j=0;j<PIX_TOTAL_ROW;j++)
				{

						for(i=0;i<4;i++)
							Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);  // all row reset & start integration

						if(j==0)
						{
								switch(sel)
								{
										case 0:
											TIM3_ARR_Update(PCR_Regs[sel].InteCount);
											BaseCounter = PCR_Regs[sel].InteTime-1;
											TIM3->CR1|=0x01; 
											TMR_Int_Flag=0;	

	#ifdef _OVERLAP_TEST
				LED3=1;
	#endif
										
										break;
										
										case 1:
											TIM1_ARR_Update(PCR_Regs[sel].InteCount);
											BaseCounter_1 = PCR_Regs[sel].InteTime-1;
											TIM1->CR1|=0x01; 
											TMR_Int_Flag_1=0;	

	#ifdef _OVERLAP_TEST
				LED4=1;
	#endif
										
										break;
										
										case 2:
											TIM4_ARR_Update(PCR_Regs[sel].InteCount); 
											BaseCounter_2 = PCR_Regs[sel].InteTime-1;
											TIM4->CR1|=0x01; 
											TMR_Int_Flag_2=0;	

	#ifdef _OVERLAP_TEST
				LED5=1;
	#endif	
										
										break;
										
										case 3:
											TIM8_ARR_Update(PCR_Regs[sel].InteCount); 
											BaseCounter_3 = PCR_Regs[sel].InteTime-1;
											TIM8->CR1|=0x01; 
											TMR_Int_Flag_3=0;		
										break;
										
										default:
										break;
								}
						
						}	
						delay_us(INTERVAL_DELAY_US);
				}
			}	
				
		 for(k=0;k<cnt;k++)
		 {
				sel=SPI_Sel=TrgChQue[k];	
			 
				for(j=0;j<PIX_TOTAL_ROW;j++)
				{
						switch(sel)
						{
								case 0:
									while((TMR_Int_Flag & (1<<j))==0);
								
	#ifdef _OVERLAP_TEST
				LED3=0;
	#endif	
								break;
								
								case 1:
									while((TMR_Int_Flag_1 & (1<<j))==0);
	#ifdef _OVERLAP_TEST
				LED4=0;
	#endif	
								break;
								
								case 2:
									while((TMR_Int_Flag_2 & (1<<j))==0);
	#ifdef _OVERLAP_TEST
				LED5=0;
	#endif	
								break;
								
								case 3:
									while((TMR_Int_Flag_3 & (1<<j))==0);
								break;
								
								default:
									break;
						}
					
						for(i=4;i<=10;i++)
						{
								Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);
						}
						
						PCR_ADC_Done_Flag=0;

						Send_Command(ADC_START, j, 0, &PCRChip_Command_Send[0]); // ADC process
						timeout_initial(1);
						//			FAN=1;
						u8temp=0;
						while(((PCR_ADC_Done_Flag & (1<<sel))==0) && (u8temp ==0)) {u8temp=timeout_check();};
						timeout_stop(1);	
						//			FAN=0;
						if(PCR_ADC_Done_Flag & (1<<sel))  // ADC ready
						{
							
								PCR_ADC_Done_Flag=0;
								Send_Command(PCR_ADC_READ, 0, 0, &PCRChip_Command_Send[0]);
								PCR_Fail_cnt[sel]=0;
							
								for(i=0;i<(PIX_TOTAL_COL<<1);i++)
									ImageArrayBuf[sel][j][i+2]=SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015				
								ImageArrayBuf[sel][j][0]=((TYP_IMAGE) | (sel<<4));    // new type definition with PCR ch number
								ImageArrayBuf[sel][j][1]=j;

						}
						else if(u8temp !=0)                   // time out
									{  
											PCR_Fail_cnt[sel] ++;	

											for(j=0;j<(PIX_TOTAL_ROW<<1);j++)
											{
													ImageArrayBuf[sel][j][0]=((TYP_IMAGE) | (sel<<4));    
													ImageArrayBuf[sel][j][1]=0xF1;
																													
													for(i=0;i<(PIX_TOTAL_COL<<1);i++)
														ImageArrayBuf[sel][j][i+2]=0;
											}							
											break;						
									}
				}

	#ifdef _OVERLAP_TEST
				switch(k)
				{
					case 0: LED3=1; break;
					case 1: LED4=1; break;
					case 2: LED5=1; break;
					default: break;
				}				
	#endif

		 }

	}

	void ReadUpdate_Image_multiPCR_oneRead_mode_0(u8 sel, u8 ContinueMode)
	{
		u8 i,j,u8temp;
		SPI_Sel=sel;
		//if(PCR_Fail_cnt[SPI_Sel] < MAX_FAIL_TIMOUT_CNT)   // reach at max fail number 
		{
	//		Multi_OSC_Access(sel,OSC_ON);
				 // modify the prototype to support multi PCR chip with different SPI_SS pin
			for(i=0;i<11;i++)
				PCRChip_Command_Send[i]=PCRChip_Pattern[i];
			TIM3_ARR_Update(PCR_Regs[SPI_Sel].InteCount);
			tempReg = BaseCounter = PCR_Regs[SPI_Sel].InteTime-1;
		
			for(j=0;j<PIX_TOTAL_ROW;j++)
			{

				for(i=0;i<4;i++)
				{		
					Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);  // all row reset & start integration
				//	if(i==1)
				//	delay_us(10);	
				}
				if(j==0)
				{
					TIM3->CR1|=0x01; 
					TMR_Int_Flag=0;
		/*			
					FAN=test_flag=1;
					test_flag=~test_flag;
		*/			
					
				}	
				/*
				else
				{
				FAN=test_flag;
				test_flag=~test_flag;
				}
				*/
				//delay_us(INTERVAL_DELAY_US);
			}
			for(j=0;j<PIX_TOTAL_ROW;j++)
			{
				while((TMR_Int_Flag & (1<<j))==0);

				for(i=4;i<=10;i++)
				{
					Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);
					//		delay_us(5);	
					//		if(i==7)
					//			delay_us(10);	
				}
				//PCR_ADC_Done_Flag &= ~(1<<sel);
		
				PCR_ADC_Done_Flag=0;
				//delay_ms(10);////////////////////////
				
			
				Send_Command(ADC_START, j, 0, &PCRChip_Command_Send[0]); // ADC process
				timeout_initial(1);
	//			FAN=1;
				u8temp=0;
				while(((PCR_ADC_Done_Flag & (1<<SPI_Sel))==0) && (u8temp ==0)) {u8temp=timeout_check();};
				timeout_stop(1);	
	//			FAN=0;
				if(PCR_ADC_Done_Flag & (1<<SPI_Sel))  // ADC ready
				{
					
					PCR_ADC_Done_Flag=0;
					Send_Command(PCR_ADC_READ, 0, 0, &PCRChip_Command_Send[0]);
					PCR_Fail_cnt[SPI_Sel]=0;
					
	//				Send_Command(PCR_ADC_READ, j, 0, &PCRChip_Command_Send[0]);
					
					for(i=0;i<(PIX_TOTAL_COL<<1);i++)
						ImageArrayBuf[sel][j][i+2]=SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015
					
							ImageArrayBuf[sel][j][0]=((TYP_IMAGE) | (sel<<4));    // new type definition with PCR ch number
							ImageArrayBuf[sel][j][1]=j;

				}
				else if(u8temp !=0)                   // time out
							{  
									PCR_Fail_cnt[SPI_Sel] ++;	
	//                Multi_OSC_Access(sel,OSC_OFF);  // fail on this channel, LED off							
									break;						
							}
			}
		}
		
		///////	add LED control
		if( !ContinueMode)     // this is last operation, can stop LED & OSC
		{
	//		LED_CTRL=(0x1 & led_mode);
	/*
			if(OSC_mode != OSC_ON)
			{
				OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_OFF;
				OSC_Status=0;
			}
	*/
	//		Multi_OSC_Access(sel,OSC_OFF);
			PixReadmMode=DISABLE;
		}
	//		
	//  SPI_Sel=SPI_Sel_Reg;

	}

	void Read_Image_to_buffer(u8 sel, u8 mode)
	{

		u8 i,j,u8temp;
		//SPI_Sel=sel;
			 // modify the prototype to support multi PCR chip with different SPI_SS pin
			for(i=0;i<11;i++)
				PCRChip_Command_Send[i]=PCRChip_Pattern[i];

		TIM3_ARR_Update(PCR_Regs[SPI_Sel].InteCount);
		tempReg = BaseCounter = PCR_Regs[SPI_Sel].InteTime-1;
		
		for(j=0;j<PIX_TOTAL_ROW;j++)
		{
			for(i=0;i<4;i++)	
				Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);  // all row reset & start integration
			if(j==0)
			{
				TIM3->CR1|=0x01; 
				TMR_Int_Flag=0;
			}	
			delay_us(INTERVAL_DELAY_US);
		}

		for(j=0;j<PIX_TOTAL_ROW;j++)
		{
			while((TMR_Int_Flag & (1<<j))==0);

			for(i=4;i<=10;i++)
				Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);
			

			PCR_ADC_Done_Flag &= ~(1<<SPI_Sel);
			Send_Command(ADC_START, j, 0, &PCRChip_Command_Send[0]); // ADC process
			timeout_initial(2);
			u8temp=0;
			while(((PCR_ADC_Done_Flag & (1<<SPI_Sel))==0) && (u8temp ==0)) {u8temp=timeout_check();};
			timeout_stop(1);
			if(PCR_ADC_Done_Flag & (1<<SPI_Sel))  // ADC ready
			{
				timeout_stop(1);
				PCR_ADC_Done_Flag=0;
				Send_Command(PCR_ADC_READ, j, 0, &PCRChip_Command_Send[0]);
				for(i=0;i<(PIX_TOTAL_COL<<1);i++)
					ImageBuf[j][i+2]=SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015
			}
			else if(u8temp !=0)                   // time out 
						{
								PCR_Fail_cnt[SPI_Sel] ++;	
								for(i=0;i<(PIX_TOTAL_COL<<1);i++)
										ImageBuf[j][i+2]=0;
								break;
						}
		}
	/*
		if( !mode)     // this is last operation, can stop LED & OSC
		{
			LED_CTRL=(0x1 & led_mode);
			Multi_OSC_Access(sel,OSC_OFF);
		}
	*/
	}

	u8 Sel_reg=0;
	void ReadUpdate_Image_multiPCR_SlowRead_mode_0(u8 sel, u8 ContinueMode)
	{
		u8 i,j,k;
		SPI_Sel=sel;

	//	Multi_OSC_Access(sel,OSC_ON);
		PixReadmMode=TYP_ROW;	
		for(i=0;i<PIX_TOTAL_ROW;i++)
		{
			k=PCR_Fail_cnt[SPI_Sel];
			//Read_Row_multiPCR_mode_0(i,sel,(PIX_TOTAL_ROW-i-1));
			Read_Row_multiPCR_mode_0(i,sel,ContinueMode);
			if(k<PCR_Fail_cnt[SPI_Sel])  // 2015-09-20: PCR fail happened, break this channel
					break; 
			ImageArrayBuf[sel][i][0]=(TYP_IMAGE | (sel<<4));
			ImageArrayBuf[sel][i][1]=i;
			for(j=0;j<(PIX_TOTAL_COL<<1);j++)
				ImageArrayBuf[sel][i][j+2]= SPI_2_RcvBuf[j];//RowData[i];  // LPG: read SPI buffer directly, 131015
		}	
		
		if( !ContinueMode)     // this is last operation, can stop LED & OSC
		{
	//		LED_CTRL=(0x1 & led_mode);
	/*
			if(OSC_mode != OSC_ON)
			{
				OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_OFF;
				OSC_Status=0;
			}
	*/
	//		Multi_OSC_Access(sel,OSC_OFF);
		}
		//for(j=0;j<(PIX_TOTAL_ROW);j++)
		//ImageArrayBuf[sel][j][0]=((TYP_IMAGE) | (sel<<4));    // new type definition with PCR ch number
		//SPI_Sel=SPI_Sel_Reg;
	}

	void Read_Image_to_buffer_pipeline(u8 cnt)
	{
		 u8 i,j,k,u8temp,sel;
		 multi_overlap_ctrl=0x80;
		
		 for(i=0;i<11;i++)
				PCRChip_Command_Send[i]=PCRChip_Pattern[i]; 
		
	#ifdef _OVERLAP_TEST
					 LED3=0; 
					 LED4=0; 
					 LED5=0; 
	#endif
		
		 for(k=0;k<cnt;k++)
		 {
				sel=SPI_Sel=TrgChQue[k];
				if(k!=0)
					delay_ms(INTERVAL_DELAY_MS);
				//TIM3_ARR_Update(100);   // interrupt @ 1ms
				//tempReg = BaseCounter = PCR_Regs[SPI_Sel].InteTime-1;
			/*
	#ifdef _OVERLAP_TEST
				switch(k)
				{
					case 0: LED3=1; break;
					case 1: LED4=1; break;
					case 2: LED5=1; break;
					default: break;
				}				
	#endif
			 */
				for(j=0;j<PIX_TOTAL_ROW;j++)
				{

						for(i=0;i<4;i++)
							Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);  // all row reset & start integration

						if(j==0)
						{
								switch(sel)
								{
										case 0:
											TIM3_ARR_Update(PCR_Regs[sel].InteCount);
											BaseCounter = PCR_Regs[sel].InteTime-1;
											TIM3->CR1|=0x01; 
											TMR_Int_Flag=0;	

	#ifdef _OVERLAP_TEST
				LED3=1;
	#endif
										
										break;
										
										case 1:
											TIM1_ARR_Update(PCR_Regs[sel].InteCount);
											BaseCounter_1 = PCR_Regs[sel].InteTime-1;
											TIM1->CR1|=0x01; 
											TMR_Int_Flag_1=0;	

	#ifdef _OVERLAP_TEST
				LED4=1;
	#endif
										
										break;
										
										case 2:
											TIM4_ARR_Update(PCR_Regs[sel].InteCount); 
											BaseCounter_2 = PCR_Regs[sel].InteTime-1;
											TIM4->CR1|=0x01; 
											TMR_Int_Flag_2=0;	

	#ifdef _OVERLAP_TEST
				LED5=1;
	#endif	
										
										break;
										
										case 3:
											TIM8_ARR_Update(PCR_Regs[sel].InteCount); 
											BaseCounter_3 = PCR_Regs[sel].InteTime-1;
											TIM8->CR1|=0x01; 
											TMR_Int_Flag_3=0;		
										break;
										
										default:
										break;
								}
						
						}	
						delay_us(INTERVAL_DELAY_US);
				}
			}	
				
		 for(k=0;k<cnt;k++)
		 {
				sel=SPI_Sel=TrgChQue[k];	
			 
				for(j=0;j<PIX_TOTAL_ROW;j++)
				{
						switch(sel)
						{
								case 0:
									while((TMR_Int_Flag & (1<<j))==0);
								
	#ifdef _OVERLAP_TEST
				LED3=0;
	#endif	
								break;
								
								case 1:
									while((TMR_Int_Flag_1 & (1<<j))==0);
	#ifdef _OVERLAP_TEST
				LED4=0;
	#endif	
								break;
								
								case 2:
									while((TMR_Int_Flag_2 & (1<<j))==0);
	#ifdef _OVERLAP_TEST
				LED5=0;
	#endif	
								break;
								
								case 3:
									while((TMR_Int_Flag_3 & (1<<j))==0);
								break;
								
								default:
									break;
						}
					
						for(i=4;i<=10;i++)
						{
								Send_Command(PXL_DRIVE, j, 1, &PCRChip_Command_Send[i]);
						}
						
						PCR_ADC_Done_Flag=0;

						Send_Command(ADC_START, j, 0, &PCRChip_Command_Send[0]); // ADC process
						timeout_initial(1);
						//			FAN=1;
						u8temp=0;
						while(((PCR_ADC_Done_Flag & (1<<sel))==0) && (u8temp ==0)) {u8temp=timeout_check();};
						timeout_stop(1);	
						//			FAN=0;
						if(PCR_ADC_Done_Flag & (1<<sel))  // ADC ready
						{
							
								PCR_ADC_Done_Flag=0;
								Send_Command(PCR_ADC_READ, 0, 0, &PCRChip_Command_Send[0]);
								PCR_Fail_cnt[sel]=0;
							
								for(i=0;i<(PIX_TOTAL_COL<<1);i++)
									pipeLine_ImageBuf[sel][j][i+2]=SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015				
								pipeLine_ImageBuf[sel][j][0]=((TYP_IMAGE) | (sel<<4));    // new type definition with PCR ch number
								pipeLine_ImageBuf[sel][j][1]=j;

						}
						else if(u8temp !=0)                   // time out
									{  
											PCR_Fail_cnt[sel] ++;	

											for(j=0;j<(PIX_TOTAL_ROW<<1);j++)
											{
													pipeLine_ImageBuf[sel][j][0]=((TYP_IMAGE) | (sel<<4));    
													pipeLine_ImageBuf[sel][j][1]=0xF1;
																													
													for(i=0;i<(PIX_TOTAL_COL<<1);i++)
														pipeLine_ImageBuf[sel][j][i+2]=0;
											}							
											break;						
									}
				}

	#ifdef _OVERLAP_TEST
				switch(k)
				{
					case 0: LED3=1; break;
					case 1: LED4=1; break;
					case 2: LED5=1; break;
					default: break;
				}				
	#endif

		 }
	}

	//////////////////////////////////////////////////////////////////
	//  24 pixel read function for multi PCR
	//  read row:  
	//             Read_Row_multiPCR_mode_1(u8 row, u8 sel)
	//
	//  chip pipeline read:  integration T > n*40
	//             ReadUpdate_Image_multiPCR_PipeLine_mode_1(u8 PcrReg)
	//
	//  single chip pipeline:  40ms < integration T  < n*40ms
	//            ReadUpdate_Image_multiPCR_oneRead_mode_0(u8 sel, u8 ContinueMode) 
	//
	//  slow read: integration T <10ms , actually read row by row
	//            ReadUpdate_Image_multiPCR_SlowRead_mode_0(u8 sel, u8 ContinueMode)
	///////////////////////////////////////////////////////////////////

	void ReadUpdate_Image_multiPCR_PipeLine_mode_1(u8 cnt)
	{
		u8 i,j,txpattern,txset,temp3;
		u8 k,sel;
		u8 fail[MAX_PCR_CH]={0};
		u8 fail_assert[MAX_PCR_CH]={0};
		for(i=0;i<11;i++)
			PCRChip_Command_Send[i]=PCRChip_Pattern[i];
		

		for(txset=0;txset<4;txset++)
		{
			txpattern =  PIXEL_24READ[txset];
			
			for(k=0;k<cnt;k++)
			{
				sel=SPI_Sel=TrgChQue[k];
				PCRChip_Command_Send[20]=PCR_Regs[SPI_Sel].SW_TxCtrl=TX_PACK(txpattern,PCR_Regs[SPI_Sel].SW_TxCtrl);
				Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[20]);
				
				fail[sel]= PCR_Fail_cnt[sel]=0;
			}
		
			
			Read_Image_to_buffer_pipeline(cnt);
			for(k=0;k<cnt;k++)
			{
				if(fail[TrgChQue[k]]< PCR_Fail_cnt[TrgChQue[k]])
					fail_assert[TrgChQue[k]]=1;
			}
			switch(txset)
			{
				case 0:
							for(k=0;k<cnt;k++)
							{
								sel=TrgChQue[k];
								for(j=0;j<PIX_TOTAL_ROW;j++)
								{
									for(i=0;i<(PIX_TOTAL_COL);i++)
									{
										ImageArrayBuf[sel][(j<<1)][(i<<2)+4] =pipeLine_ImageBuf[sel][j][(i<<1)+2];
										ImageArrayBuf[sel][(j<<1)][(i<<2)+5] =pipeLine_ImageBuf[sel][j][(i<<1)+3];
									}
								}	
							}
				break;

				case 1:
							for(k=0;k<cnt;k++)
							{
									sel=TrgChQue[k];
									for(j=0;j<PIX_TOTAL_ROW;j++)
									{
										for(i=0;i<(PIX_TOTAL_COL);i++)
										{
											ImageArrayBuf[sel][(j<<1)][(i<<2)+2] =pipeLine_ImageBuf[sel][j][(i<<1)+2];
											ImageArrayBuf[sel][(j<<1)][(i<<2)+3] =pipeLine_ImageBuf[sel][j][(i<<1)+3];
										}	

									}	
							}
				break;

				case 2:
							for(k=0;k<cnt;k++)
							{
									sel=TrgChQue[k];
									for(j=0;j<PIX_TOTAL_ROW;j++)
									{
										for(i=0;i<(PIX_TOTAL_COL);i++)
										{
											ImageArrayBuf[sel][(j<<1)+1][(i<<2)+4] =pipeLine_ImageBuf[sel][j][(i<<1)+2];
											ImageArrayBuf[sel][(j<<1)+1][(i<<2)+5] =pipeLine_ImageBuf[sel][j][(i<<1)+3];
										}
									}	
							}
				break;

				case 3:
							for(k=0;k<cnt;k++)
							{
									sel=TrgChQue[k];
									for(j=0;j<PIX_TOTAL_ROW;j++)
									{
										for(i=0;i<(PIX_TOTAL_COL);i++)
										{
											ImageArrayBuf[sel][(j<<1)+1][(i<<2)+2] =pipeLine_ImageBuf[sel][j][(i<<1)+2];
											ImageArrayBuf[sel][(j<<1)+1][(i<<2)+3] =pipeLine_ImageBuf[sel][j][(i<<1)+3];
										}
									}
							}
				break;
			
				default:
				break;
			}
		}
	/*
		PCR_Regs.SW_TxCtrl=	PCRChip_Command_Send[10];
		Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[10]);	
	*/

		//if( !ContinueMode)     // this is last operation, can stop LED & OSC
		{
	//		LED_CTRL=(0x1 & led_mode);
	/*
			if(OSC_mode != OSC_ON)
			{
				OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_OFF;
				OSC_Status=0;
			}
	*/
	//		Multi_OSC_Access(SPI_Sel,OSC_OFF);
		}
			for(k=0;k<cnt;k++)
			{
		
					for(i=0;i<24;i++)
					{	 
							ImageArrayBuf[TrgChQue[k]][i][0]=(TYP_24PIXIMAG|(TrgChQue[k]<<4));
							ImageArrayBuf[TrgChQue[k]][i][1] = i;
					}
					if(fail_assert[TrgChQue[k]] !=0 )
					{
								for(i=0;i<24;i++)
										ImageArrayBuf[TrgChQue[k]][i][1] = 0xF1;
					}
			}
	}






	void ReadUpdate_Image_multiPCR_oneRead_mode_1(u8 sel, u8 ContinueMode)
	{
		u8 i,j,txpattern,txset,temp3;
	// 	u8 count;
		SPI_Sel=sel;
			 // modify the prototype to support multi PCR chip with different SPI_SS pin
	//	Multi_OSC_Access(sel,OSC_ON);
		
		for(txset=0;txset<4;txset++)
		{
			txpattern =  PIXEL_24READ[txset];
			PCRChip_Command_Send[20]=PCR_Regs[SPI_Sel].SW_TxCtrl=TX_PACK(txpattern,PCR_Regs[SPI_Sel].SW_TxCtrl);
			Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[20]);
			
			temp3= PCR_Fail_cnt[SPI_Sel];
			Read_Image_to_buffer(SPI_Sel,(4-txset-1));
			if(temp3 < PCR_Fail_cnt[SPI_Sel]) // PCR error happened
				 break;
			
			//ReadUpdate_Image_multiPCR_oneRead_mode_0(SPI_Sel,(4-txset-1));
			switch(txset)
			{
				case 0:
					for(j=0;j<PIX_TOTAL_ROW;j++)
					{
						for(i=0;i<(PIX_TOTAL_COL);i++)
						{
							ImageArrayBuf[SPI_Sel][(j<<1)][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
							ImageArrayBuf[SPI_Sel][(j<<1)][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
						}
					}	
				break;

				case 1:
					for(j=0;j<PIX_TOTAL_ROW;j++)
					{
						for(i=0;i<(PIX_TOTAL_COL);i++)
						{
							ImageArrayBuf[SPI_Sel][(j<<1)][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
							ImageArrayBuf[SPI_Sel][(j<<1)][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
						}	

					}	
				break;

				case 2:
					for(j=0;j<PIX_TOTAL_ROW;j++)
					{
						for(i=0;i<(PIX_TOTAL_COL);i++)
						{
							ImageArrayBuf[SPI_Sel][(j<<1)+1][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
							ImageArrayBuf[SPI_Sel][(j<<1)+1][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
						}
					}	
				break;

				case 3:
					for(j=0;j<PIX_TOTAL_ROW;j++)
					{
						for(i=0;i<(PIX_TOTAL_COL);i++)
						{
							ImageArrayBuf[SPI_Sel][(j<<1)+1][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
							ImageArrayBuf[SPI_Sel][(j<<1)+1][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
						}
					}	
				break;
			
				default:
				break;
			}
		}
	/*
		PCR_Regs.SW_TxCtrl=	PCRChip_Command_Send[10];
		Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[10]);	
	*/

		if( !ContinueMode)     // this is last operation, can stop LED & OSC
		{
	//		LED_CTRL=(0x1 & led_mode);
	/*
			if(OSC_mode != OSC_ON)
			{
				OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_OFF;
				OSC_Status=0;
			}
	*/
	//		Multi_OSC_Access(SPI_Sel,OSC_OFF);
		}

			for(i=0;i<24;i++)
			{	 ImageArrayBuf[SPI_Sel][i][0]=(TYP_24PIXIMAG|(SPI_Sel<<4));
				 ImageArrayBuf[SPI_Sel][i][1] = i;
			}
	}



	void ReadUpdate_Image_multiPCR_SlowRead_mode_1(u8 sel, u8 ContinueMode)
	{
		u8 i,j,txpattern,u8temp;
		SPI_Sel=sel;
		PixReadmMode=TYP_ROW;
		for(u8temp=0;u8temp<4;u8temp++)
		{
						
	//		Multi_OSC_Access(SPI_Sel,OSC_ON);
			txpattern =  PIXEL_24READ[u8temp];
			PCRChip_Command_Send[20]=PCR_Regs[SPI_Sel].SW_TxCtrl=TX_PACK(txpattern,PCR_Regs[SPI_Sel].SW_TxCtrl);
			Send_Command(REG_WRITE, TX_PATTERN_ADDR, 1, &PCRChip_Command_Send[20]);
				
			for(j=0;j<PIX_TOTAL_ROW;j++)
			{
				
				Read_Row_multiPCR_mode_0(j,SPI_Sel,(PIX_TOTAL_ROW-1-j));
				for(i=0;i<(PIX_TOTAL_COL<<1);i++)
					ImageBuf[j][i+2]= SPI_2_RcvBuf[i];//RowData[i];  // LPG: read SPI buffer directly, 131015
				
				sel=SPI_Sel;
				switch(u8temp)
				{
					case 0:
						//for(j=0;j<PIX_TOTAL_ROW;j++)
						{
							for(i=0;i<(PIX_TOTAL_COL);i++)
							{
								ImageArrayBuf[sel][(j<<1)][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
								ImageArrayBuf[sel][(j<<1)][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
							}
						}	
					break;

					case 1:
						//for(j=0;j<PIX_TOTAL_ROW;j++)
						{

							for(i=0;i<(PIX_TOTAL_COL);i++)
							{
								ImageArrayBuf[sel][(j<<1)][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
								ImageArrayBuf[sel][(j<<1)][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
							}	

						}	
					break;

					case 2:
						//for(j=0;j<PIX_TOTAL_ROW;j++)
						{
							for(i=0;i<(PIX_TOTAL_COL);i++)
							{
								ImageArrayBuf[sel][(j<<1)+1][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
								ImageArrayBuf[sel][(j<<1)+1][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
							}
						}	
					break;

					case 3:
						//for(j=0;j<PIX_TOTAL_ROW;j++)
						{
							for(i=0;i<(PIX_TOTAL_COL);i++)
							{
								ImageArrayBuf[sel][(j<<1)+1][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
								ImageArrayBuf[sel][(j<<1)+1][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
							}
						}	
					break;
				
					default:
					break;
				}

				
			}	
		}

		if( !ContinueMode)     // this is last operation, can stop LED & OSC
		{
	//		LED_CTRL=(0x1 & led_mode);
	/*
			if(OSC_mode != OSC_ON)
			{
				OSC_Ctrl=OSC_Ctrl_1=OSC_Ctrl_2=OSC_Ctrl_3=OSC_OFF;
				OSC_Status=0;
			}
	*/
			
	//		Multi_OSC_Access(sel,OSC_OFF);
		}

	//	if(PixReadmMode == TYP_24PIXIMAG)
		{			
			for(i=0;i<24;i++)
			{	 ImageArrayBuf[sel][i][0]=(TYP_24PIXIMAG | (sel<<4));
				 ImageArrayBuf[sel][i][1] = i;
			}
	 }
	/*
		 else if(mode == TYP_24PIXROW)
		 {
			 ImageBufPIX[row][0] =  TYP_24PIXROW;
			 ImageBufPIX[row][1] =  row ;
	//#ifndef PIX24_DEBUG_UART_PRINT
			 USBRes(NO_ERR, CMD_GET, &ImageBufPIX[row][0], 52);	
	//#else
			// UARTRes(NO_ERR, CMD_GET, &ImageBufPIX[row][0], 52);
	//#endif
				#if !defined(USB_WAIT_MODE)
				delay_ms(35);
				#else
				EpMsgEnable();
				UsbReadDone();
				EpMsgDisable();
				#endif	 		  	
		 }
	*/
	 //SPI_Sel=SPI_Sel_Reg;
	}




	void PcrTrgDisable(void)
	{
#ifdef __QPCR_HW
		EXTI->IMR &= ~(0xf<<10);
#else
		EXTI->IMR &= ~(0xf<<6);
#endif
	}

	void PcrMskMapping(u8 mask)
	{
			u8 i;
			for(i=0;i<MAX_PCR_CH;i++)
			{
				//if(mask>>i)
				if(mask & (1<<i))
				{
#ifdef __QPCR_HW
						EXTI->IMR |= (1<<(10+i));
#else
						EXTI->IMR |= (1<<(6+i));
#endif
				}
				else
				{
#ifdef __QPCR_HW
						EXTI->IMR &= ~(1<<(10+i));
#else
						EXTI->IMR &= ~(1<<(6+i));
#endif
				}
						
			}
	}

	u8 u8TimOut_flag=0;
	u8 u8TimOut_check=0;
	u8 u8Timout_Cnt=0;
	void timeout_initial(u16 mS)
	{
		u8Timout_Cnt= 0;
		TIM6_Int_Init(100,7199);
		TIM6->CR1|=0x01;
	}

	u8 timeout_check(void)
	{
		u8 temp=0;
		if(u8Timout_Cnt>4)
		{
			//TIM6->CR1 &=~(u16)0x1;
			temp=1;
		}
		return temp; 	
	}	

	void timeout_stop(u8 channel)
	{
		//u8TimOut_check &=  ~channel;
		//if(u8TimOut_check==0)               // if no channle to be monitored, stop timer.
			TIM6->CR1 &=~(u16)0x1;
			u8Timout_Cnt=0;
	}

	void PCR_initialize(void)
	{
		 u8 i,u8temp;
		 PixReadmMode=TYP_IMAGE;
		PcrMskReg=0xf;
		for(i=0;i<MAX_PCR_CH;i++)
		{
				u8temp=PCR_Fail_cnt[i]=0;
				PCR_Regs[i].InteTime=25;
				{												 // must in serial method															  
						ReadUpdate_Image_multiPCR_oneRead_mode_0(i,(MAX_PCR_CH-i-1));
						if(u8temp < PCR_Fail_cnt[i])   // fail happened in this cycle, then overwrite packet
							PcrMskReg &= ~(1<<i);
				}
				PCR_Regs[i].InteTime=1;
		}
		PixReadmMode=DISABLE;
	}


	void TIM5_IRQHandler(void)
	{

		u16 i;
		u16 adcx[4];
		float fTemp[4]={0};

		TIM5->SR&=~(1<<0);

		for(i=0;i<50;i++)
		{
			adcx[0]=Get_Adc(ADC_CH10);
			fTemp[0] += adcx[0] * 3.3/4096;
			
			adcx[1]=Get_Adc(ADC_CH11);
			fTemp[1] += adcx[1] * 3.3/4096;
			
			adcx[2]=Get_Adc(ADC_CH12);
			fTemp[2] += adcx[2] * 3.3/4096;
			
			adcx[3]=Get_Adc(ADC_CH13);
			fTemp[3] += adcx[3] * 3.3/4096;
		}
		for(i=0;i<4;i++)
		{
				fTemp[i] = fTemp[i]/50;
				PCR_temp[i] = PCR_temp[i] + 0.1*(fTemp[i] - PCR_temp[i]);
		}
	}