72Mhz sys clock
 use SPI2 @ 1Mhz SCLK

 MISO PB14
 MOSI PB15
 SCLK PB13
 SS   PB12

 PCR_AMUX: PA1
 PCR rst  PG8
 PG6 -- OSC enable
 ADC_Done input @ PA0

 i2C1 to read temperature sensor
 SCL PB10
 SDA PB11

 ADC1 is used. 3 channels are used:
AMUX input for ADC, PA1
//Tempetrature sensor +- input to ADC, PA2(P),PA3(N)
  PCR temperature read from PC4,PC5 over OP
Temp control PWM 0
TIM4_CH3  PB8

Temp control PWM 1
TIM1_CH1  PA8
Temp control PWM 2
TIM8_CH1  PC6

PWM1N PC0
PWM2N PC1

//////////////////////
Image:
timer1,3,4,8 for integation
timer 6 for ADC timeout 
timer7 for LED setup/holdon   --- in Lucentix use it as touch LED blink of 5Hz tick
timer5 for CMOS temp read tick
//////////////////////
timer2 for UART EX scan



FAN control: PF4// changee from PC5 
LED_control: PF5// change from PG13
trgger_input: PF6 --EXTI_9_5// change from PE14 EXTI_15_10	 

////////TIM1 is used as real (500ms) Systick for temperature reading 
TIM6 is used as real (500ms) Systick for temperature reading   @ 130302
integration timer: TIM3
SysTick is used for delay.

PB0,--ADC8
PB1 --ADC9
PC0 --ADC10
PC1 --ADC11
PC2 --ADC12
PC3 --ADC14
PC5 --ADC15


trigger1,2,3,4==PF6, 7,8,9

////////////////////////////////////
timer 6 is for ADC time out
timer 3 is for integration tick
timer 7 is for LED time out
///////////////////////////////////
snap LED0 - PF5
     LED1 - PD7
	 LED2 - PD14
	 LED3 -	PD15
///////////////////////////////////////
use __qPCR_HW
	output:
	LED1~4 use PA0~3
	OSC_EN1-PA4
	OSC_EN2-PB10
	OSC_EN3-PB11
	OSC_EN4-PC6
	snap LED1-4 use PA0-PA3
	SS1-PB12
	SS2-PB6
	SS3-PB7
	SS4-PB8
	POR_1-PA9

	input:
	analog1-PC0-ADC10
	analog2-PC1-ACD11
	analog3-PC2-ADC12
	analog4-PC3-ADC13

	ADCRDY1-4 	use PA5-PA8
	trigger1-4 	use PC10-PC13
