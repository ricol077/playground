#include "sys.h"	
#include "delay.h"	 	 
#include "rtc.h"	 	 
#include "adc.h" 	 	 
#include "dma.h" 	 
#include "spi.h"
#include "user_def.h"
#include "PCR_Cycle.h"
#include "timer.h"
//#include "Temperature.h"

u8 PIX_Buf[16]={0};
u8 REG_Buf[16]={0};
u8 OSC_Status=0;
u8 OSC_mode=0;
u8 OSC_Busy=0;
u16 UserCountMS=1; // LPG: this is user command data, 1ms/count
u16 BaseCounter=0; // LPG: this is to count the base tick --1ms
u16 BaseCounter_1=0;
u16 BaseCounter_2=0;
u16 BaseCounter_3=0;
//static u8 Current_Stage=0;
//static u8 Current_Cycle=0;  // record cycle & stage, be protected
//static u8 Current_Sect=0;
//static u8 FanCSR=0;         // FanCSR:  status register /* xxxxxx mode, status*/

//////for multi-PCR///////
static MSG_TYP MsgExtiTrg=0;

///////           ///////

MSG_TYP MsgTrgPop(void)
{
	MSG_TYP Msg=MsgExtiTrg;
//	INTX_DISABLE();
	MsgExtiTrg=0;
	Exti=0;
//	INTX_ENABLE();
	return Msg;
}

#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))  // version 1.14: 
void MsgTrgForce(MSG_TYP msg, MSG_TYP msk_force)  
		//  Lucentix butto, press to force capture
{
	 msg &= msk_force; 
	 if(MsgExtiTrg != msg)
	 {
		 MsgExtiTrg |=msg;
		 Exti=1;
	 }
}
#endif 


void MsgTrgPush(MSG_TYP msg) // mask first, then report only any new update, 
														 //  as msg did not pop out, it's under process now.
{
	 msg &= PcrMskReg; 
	 if(MsgExtiTrg != msg)
	 {
		 MsgExtiTrg |=msg;
		 Exti=1;
	 }
}

//////////////////////////
// transfer the command packet
// input:
//	   Command, targt_row, how many byte data, data buffer pointer
///////////////////////////
															   
u8 Send_Command(u8 cmd, u8 row_num, u8 length, u8 * pbuf)
{
	u8 i;

	if(length>MAX_LENGTH_VALID)
		return ERR_LENGTH;
/*
	if(row_num>MAX_ROW_NUM)
		return ERR_CMD;
*/
 	switch(cmd)
	{
		case PXL_DRIVE:
				if(row_num < PIX_TOTAL_ROW)
				{
			  		SPI_2_SendBuf[0]= (PXL_DRIVE<<5)| row_num;
					for(i=0;i<length;i++)
						SPI_2_SendBuf[1+i]= *(pbuf+i);	
					SPI_2_Trans(length+1);
				}else
					return ERR_ROW;
					
			break;	
			
		case REG_WRITE:
				if(row_num <= REG_MAX_NUM)
				{
			  		SPI_2_SendBuf[0]= (REG_WRITE<<5)| row_num;
					for(i=0;i<length;i++)
						SPI_2_SendBuf[1+i]= *(pbuf+i);	
					SPI_2_Trans(length+1);
				}else
					return ERR_ROW;
			break;	

		case ADC_START:
					//OSC_Ctrl=OSC_ON;
					//OSC_Status=1;
					//OSC_Busy=1;

			  		SPI_2_SendBuf[0]= (ADC_START<<5);	
					SPI_2_Trans(1);
					
			break;
		
		case PCR_ADC_READ:
			 		SPI_2_SendBuf[0]= 0xff;	  /*set to a invalid command, in case of misfunction*/
					SPI_2_Trans((PIX_TOTAL_COL<<1));
					SPI_Rev_Data_Copy((PIX_TOTAL_COL<<1));
			break;
									   
		default:
				return ERR_CMD;
//			break;
	}
	return 	NO_ERR;
}
/////////////////
//  format the ADC data received from SPI
//  SWAP byte order, remove the 4 additional bits
//  input:
//			target u16 buffer pointer, how many bytes to be formated in SPI buffer
////////////////

void Format_ADC_Data(u8 *pu16Buf, u8 Byte_Num)
{
 	u8 i;
	u16 u16temp;
	for (i=0; i<Byte_Num;i+=2)
	{
		u16temp = SPI_2_RcvBuf[i+1];
		u16temp <<= 4;
		u16temp |= 	SPI_2_RcvBuf[i];
		*(pu16Buf++)=u16temp;
	}
}

////////////////
//	external OSC control initialize
//
//////////////
void OSC_Ctrl_Init(void)
{
#ifdef __QPCR_HW  //PA4, PB10,PB11,PC6
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟	
	RCC->APB2ENR|=1<<4;    //使能PORTC时钟 
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟   
	
  GPIOA->CRL&=0XFFF0FFFF;	
	GPIOA->CRL|=0X00030000;
	GPIOA->ODR &=~(u32)(0x1<<4); 
	
  GPIOB->CRH&=0XFFFF00FF;	
	GPIOB->CRH|=0X00003300;
	GPIOB->ODR &=~(u32)(0x3<<10); 	
	
  GPIOC->CRL&=0XF0FFFFFF;	
	GPIOC->CRL|=0X03000000;
	GPIOC->ODR &=~(u32)(0x1<<6); 		
#else
	RCC->APB2ENR|=1<<8;     //使能PORTG时钟
  GPIOG->CRL&=0XF0000FFF;	
	GPIOG->CRL|=0X0003333000;
	GPIOG->ODR &=~(u32)(0xf<<3); 
#endif
}

void Multi_OSC_Access(u8 sel, u8 op) // select a OSC, operation: on, off, auto
{
	switch(op)
	{
		case OSC_ON:
				if((OSC_Status & (1<<sel))==0)
				{
						switch(sel)
						{
							case 0:
								OSC_Ctrl=OSC_ON;
							break;
							case 1:
								OSC_Ctrl_1=OSC_ON;
							break;		
							case 2:
								OSC_Ctrl_2=OSC_ON;
							break;
							case 3:
								OSC_Ctrl_3=OSC_ON;
							break;
							default:
									break;
						}
						delay_ms(10);
						OSC_Status |= (1<<sel);
				}	
				
		break;
				
		case OSC_OFF:
				if((OSC_Status & (1<<sel))==1)
				{
						switch(sel)
						{
							case 0:
								OSC_Ctrl=OSC_OFF;
							break;
							case 1:
								OSC_Ctrl_1=OSC_OFF;
							break;		
							case 2:
								OSC_Ctrl_2=OSC_OFF;
							break;
							case 3:
								OSC_Ctrl_3=OSC_OFF;
							break;
							default:
									break;
						}
						OSC_Status &= ~(1<<sel);
				}				
		break;

		default:
			break;
	}
}

#if ((!defined(__QPCR_HW)) && (defined(__USE_MOS_GATE)))
				//version 1.13
		static u8 MOS_Gate_sts=0;

		void MOS_Gate_Ctrl(u8 ctrl)   // note: TPS76933 is enable by /EN=0, and disable b /EN=1
																	// to align with OSC flag, just use OSC_ON or OSC_OFF, here use ~ctrl
		{
				if ((ctrl & 0x1) != MOS_Gate_sts)
				{
						MOS_Gate_sts = (ctrl & 0x1);
					  MOS_Gate	= ~(ctrl & 0x1);  // /EN=0,enable MOS,  /EN=1, disable MOS
						if(MOS_Gate_sts) 
								delay_ms(10);
				}
		}
#endif

	#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))  //version 1.14
		u8 blink_count=0;
		static u8 Lucentix_Button_sts=0;
		void Lucentix_Button_msg_clr(void)
		{
			Lucentix_Button_sts=0;
		}	
		u8 Lucentix_Button_sts_read(void)
		{
			return Lucentix_Button_sts;
		}
		
		void Lucentix_Button_sts_Bit_Set(u8 ctrl)
		{
				switch(ctrl)
				{
					case BUTTON_ACTIVE_MASK:
							Lucentix_Button_sts |= BUTTON_ACTIVE_MASK;
						break;
					
					case BUTTON_PROCESSED_MASK:
							Lucentix_Button_sts |= BUTTON_PROCESSED_MASK;
						break;
					
					case BUTTON_DONE_MASK:
							Lucentix_Button_sts |= BUTTON_DONE_MASK;
						break;
					
					default:
						
						break;
				}
		}
		
		void LED_KEY_blink(void)
		{
				
			  blink_count=LED_BLINK_TIMEOUT;
				TIM7_Init(2000,6399);
		}
		void LED_KEY_idle(void)
		{
				TIM7_Stop();
			  blink_count=0;
		}		
		
		
		u8 Lucentix_Button_Check(void)
		{
				u8 temp=0;
			  temp=(KEY1_IN);
				if(temp==LUCENTIX_KEY_ACTIVE)
				{
					delay_ms(10);
					temp |= (KEY1_IN);
				
					if(temp==LUCENTIX_KEY_ACTIVE) 
							return 1;
				}
				
				return 0;   
		}

		void	Lucentix_Button_Process(u8 sts)
		{
			switch(sts)
			{
				case BUTTON_ACTIVE_MASK:
							LED_EX=1;
							LED_KEY=0;					
					break;
				
				case BUTTON_PROCESSED_MASK:
							LED_EX=0;
							LED_KEY_blink();					
					break;
				
				default:
							LED_KEY_idle();	
							LED_EX=0;
							LED_KEY=0;
					break;
						
			}
		
		}
		

		
	#endif
