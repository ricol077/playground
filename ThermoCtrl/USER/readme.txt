72Mhz sys clock
 use SPI2 @ 1Mhz SCLK

 MISO PB14
 MOSI PB15
 SCLK PB13
 SS   PB2

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

FAN control: PF4// changee from PC5 
LED_control: PF5// change from PG13
trgger_input: PF6 --EXTI_9_5// change from PE14 EXTI_15_10	 

////////TIM1 is used as real (500ms) Systick for temperature reading 
TIM6 is used as real (500ms) Systick for temperature reading   @ 130302
integration timer: TIM3
SysTick is used for delay.
////////////////
qPCR:
PWM output change to Low
trigger output on PC10,PC11,PC12,PD2
FAN is on PB0