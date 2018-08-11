#ifndef  __BSP_H 
#define  __BSP_H 
#define  GPIO_PORT_LED         GPIOC  //根据自己板子情况修改
#define  RCC_GPIO_PORT_LED    RCC_APB2Periph_GPIOC  //根据自己板子情况修改


#ifdef __QPCR_HW
			#define  GPIO_Pin_led1          GPIO_Pin_9   //根据自己板子情况修改
			#define  GPIO_Pin_led2          GPIO_Pin_8   //根据自己板子情况修改
			
			#define  GPIO_PORT_RST         	GPIOC  
			#define  RCC_GPIO_PORT_RST    	RCC_APB2Periph_GPIOC  //根据自己板子情况修改
			#define  GPIO_Pin_RST_IM        GPIO_Pin_1   //根据自己板子情况修改
			#define  GPIO_Pin_RST_TM        GPIO_Pin_0   //根据自己板子情况修改

			/////////////////////////////////////////// boot control pin
			#define  GPIO_Pin_TM_Boot				GPIO_Pin_7
			#define  GPIO_Pin_IM_Boot				GPIO_Pin_6
			
			///////////////////////////////////////// KeyPad
			#define  GPIO_PAD_KEYPAD_LED		GPIOB
			#define  GPIO_Pin_KEYPAD_LED1		GPIO_Pin_15
			#define  GPIO_Pin_KEYPAD_LED2	  GPIO_Pin_14
			
			#define  GPIO_PAD_KEYPAD_KEY		GPIOC
			#define  GPIO_Pin_KEYPAD_KEY1		GPIO_Pin_3
			#define  GPIO_Pin_KEYPAD_KEY2	  GPIO_Pin_2			
			
			
#else
			#define  GPIO_Pin_led1          GPIO_Pin_0   //D7
			#define  GPIO_Pin_led2          GPIO_Pin_1   //D9
			#define  GPIO_Pin_led3          GPIO_Pin_14  //D12
			#define  GPIO_Pin_led4          GPIO_Pin_15  //D13
#endif

#define  LED1     0
#define  LED2     1
#define  LED3     2
#define  LED4     3

void     BSP_Init(void);
void     led_on(uint32_t n); 
void     led_off(uint32_t n);
 
#endif
