#line 1 "test.c"
#line 1 "C:\\Keil\\ARM\\RV31\\INC\\math.h"




 





 












 







 






#line 47 "C:\\Keil\\ARM\\RV31\\INC\\math.h"

#line 61 "C:\\Keil\\ARM\\RV31\\INC\\math.h"

   




 















 
#line 92 "C:\\Keil\\ARM\\RV31\\INC\\math.h"











 





extern __softfp unsigned __ARM_dcmp4(double  , double  );
extern __softfp unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __softfp int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __softfp int __ARM_fpclassify(double  );
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
__inline __declspec(__nothrow) __softfp int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
__inline __declspec(__nothrow) __softfp int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
__inline __declspec(__nothrow) __softfp int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

__inline __declspec(__nothrow) __softfp int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
__inline __declspec(__nothrow) __softfp int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 210 "C:\\Keil\\ARM\\RV31\\INC\\math.h"



   
  typedef float float_t;
  typedef double double_t;











extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __pure double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __pure double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    
#line 340 "C:\\Keil\\ARM\\RV31\\INC\\math.h"
    __inline double _sqrt(double __x) { return sqrt(__x); }
    __inline float _sqrtf(float __x) { return (float)sqrt(__x); }


extern __declspec(__nothrow) __pure double ceil(double  );
    
    
extern __declspec(__nothrow) __pure double fabs(double  );
    
    

extern __declspec(__nothrow) __pure double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
__inline __declspec(__nothrow) __pure double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
__inline __declspec(__nothrow) __pure float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 440 "C:\\Keil\\ARM\\RV31\\INC\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) __pure int ilogb(double  );
    

 
extern __declspec(__nothrow) __pure int ilogbf(float  );
    

 
extern __declspec(__nothrow) __pure int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) __pure double logb(double  );
    

 
extern __declspec(__nothrow) __pure float logbf(float  );
    

 
extern __declspec(__nothrow) __pure long double logbl(long double  );
    

 
extern __declspec(__nothrow) __pure double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) __pure float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) __pure long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) __pure double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) __pure float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) __pure long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) __pure double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __pure double rint(double  );
    

 
extern __declspec(__nothrow) __pure double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) __pure float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) __pure long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) __pure double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) __pure float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) __pure long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __pure float _fabsf(float);  
__inline __declspec(__nothrow) __pure float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __pure float ceilf(float  );
extern __declspec(__nothrow) __pure float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );

#line 812 "C:\\Keil\\ARM\\RV31\\INC\\math.h"





#line 905 "C:\\Keil\\ARM\\RV31\\INC\\math.h"











#line 1107 "C:\\Keil\\ARM\\RV31\\INC\\math.h"



 
#line 2 "test.c"
#line 1 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\SYSTEM\\sys\\stm32f10x_map.h"














 

 







 
#line 1 "..\\SYSTEM\\sys\\stm32f10x_conf.h"













 

 



 
#line 1 "..\\SYSTEM\\sys\\stm32f10x_type.h"














 

 



 
 
typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;   
typedef signed short const sc16;   
typedef signed char  const sc8;    

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;   
typedef volatile signed short const vsc16;   
typedef volatile signed char  const vsc8;    

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;   
typedef unsigned short const uc16;   
typedef unsigned char  const uc8;    

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;   
typedef volatile unsigned short const vuc16;   
typedef volatile unsigned char  const vuc8;    

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#line 73 "..\\SYSTEM\\sys\\stm32f10x_type.h"

 
 
 



 
#line 22 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 
 


 
 

 
 





 


 


 


 


 


 
#line 66 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 


 



 


 


 
#line 90 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 




 


 


 


 


 


 


 





 


 
#line 133 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 
#line 141 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 



 


 
#line 167 "..\\SYSTEM\\sys\\stm32f10x_conf.h"



 
#line 27 "..\\SYSTEM\\sys\\stm32f10x_map.h"
#line 28 "..\\SYSTEM\\sys\\stm32f10x_map.h"
#line 1 "..\\SYSTEM\\sys\\cortexm3_macro.h"













 

 



 
#line 22 "..\\SYSTEM\\sys\\cortexm3_macro.h"

 
 
 
 
void __WFI(void);
void __WFE(void);
void __SEV(void);
void __ISB(void);
void __DSB(void);
void __DMB(void);
void __SVC(void);
u32 __MRS_CONTROL(void);
void __MSR_CONTROL(u32 Control);
u32 __MRS_PSP(void);
void __MSR_PSP(u32 TopOfProcessStack);
u32 __MRS_MSP(void);
void __MSR_MSP(u32 TopOfMainStack);
void __RESETPRIMASK(void);
void __SETPRIMASK(void);
u32 __READ_PRIMASK(void);
void __RESETFAULTMASK(void);
void __SETFAULTMASK(void);
u32 __READ_FAULTMASK(void);
void __BASEPRICONFIG(u32 NewPriority);
u32 __GetBASEPRI(void);
u16 __REV_HalfWord(u16 Data);
u32 __REV_Word(u32 Data);



 
#line 29 "..\\SYSTEM\\sys\\stm32f10x_map.h"

 
 
 
 

 
typedef struct
{
  vu32 SR;
  vu32 CR1;
  vu32 CR2;
  vu32 SMPR1;
  vu32 SMPR2;
  vu32 JOFR1;
  vu32 JOFR2;
  vu32 JOFR3;
  vu32 JOFR4;
  vu32 HTR;
  vu32 LTR;
  vu32 SQR1;
  vu32 SQR2;
  vu32 SQR3;
  vu32 JSQR;
  vu32 JDR1;
  vu32 JDR2;
  vu32 JDR3;
  vu32 JDR4;
  vu32 DR;
} ADC_TypeDef;

 
typedef struct
{
  u32  RESERVED0;
  vu16 DR1;
  u16  RESERVED1;
  vu16 DR2;
  u16  RESERVED2;
  vu16 DR3;
  u16  RESERVED3;
  vu16 DR4;
  u16  RESERVED4;
  vu16 DR5;
  u16  RESERVED5;
  vu16 DR6;
  u16  RESERVED6;
  vu16 DR7;
  u16  RESERVED7;
  vu16 DR8;
  u16  RESERVED8;
  vu16 DR9;
  u16  RESERVED9;
  vu16 DR10;
  u16  RESERVED10; 
  vu16 RTCCR;
  u16  RESERVED11;
  vu16 CR;
  u16  RESERVED12;
  vu16 CSR;
  u16  RESERVED13[5];
  vu16 DR11;
  u16  RESERVED14;
  vu16 DR12;
  u16  RESERVED15;
  vu16 DR13;
  u16  RESERVED16;
  vu16 DR14;
  u16  RESERVED17;
  vu16 DR15;
  u16  RESERVED18;
  vu16 DR16;
  u16  RESERVED19;
  vu16 DR17;
  u16  RESERVED20;
  vu16 DR18;
  u16  RESERVED21;
  vu16 DR19;
  u16  RESERVED22;
  vu16 DR20;
  u16  RESERVED23;
  vu16 DR21;
  u16  RESERVED24;
  vu16 DR22;
  u16  RESERVED25;
  vu16 DR23;
  u16  RESERVED26;
  vu16 DR24;
  u16  RESERVED27;
  vu16 DR25;
  u16  RESERVED28;
  vu16 DR26;
  u16  RESERVED29;
  vu16 DR27;
  u16  RESERVED30;
  vu16 DR28;
  u16  RESERVED31;
  vu16 DR29;
  u16  RESERVED32;
  vu16 DR30;
  u16  RESERVED33; 
  vu16 DR31;
  u16  RESERVED34;
  vu16 DR32;
  u16  RESERVED35;
  vu16 DR33;
  u16  RESERVED36;
  vu16 DR34;
  u16  RESERVED37;
  vu16 DR35;
  u16  RESERVED38;
  vu16 DR36;
  u16  RESERVED39;
  vu16 DR37;
  u16  RESERVED40;
  vu16 DR38;
  u16  RESERVED41;
  vu16 DR39;
  u16  RESERVED42;
  vu16 DR40;
  u16  RESERVED43;
  vu16 DR41;
  u16  RESERVED44;
  vu16 DR42;
  u16  RESERVED45;    
} BKP_TypeDef;

 
typedef struct
{
  vu32 TIR;
  vu32 TDTR;
  vu32 TDLR;
  vu32 TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct
{
  vu32 RIR;
  vu32 RDTR;
  vu32 RDLR;
  vu32 RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  vu32 FR1;
  vu32 FR2;
} CAN_FilterRegister_TypeDef;

typedef struct
{
  vu32 MCR;
  vu32 MSR;
  vu32 TSR;
  vu32 RF0R;
  vu32 RF1R;
  vu32 IER;
  vu32 ESR;
  vu32 BTR;
  u32  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  u32  RESERVED1[12];
  vu32 FMR;
  vu32 FM1R;
  u32  RESERVED2;
  vu32 FS1R;
  u32  RESERVED3;
  vu32 FFA1R;
  u32  RESERVED4;
  vu32 FA1R;
  u32  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

 
typedef struct
{
  vu32 DR;
  vu8  IDR;
  u8   RESERVED0;
  u16  RESERVED1;
  vu32 CR;
} CRC_TypeDef;


 
typedef struct
{
  vu32 CR;
  vu32 SWTRIGR;
  vu32 DHR12R1;
  vu32 DHR12L1;
  vu32 DHR8R1;
  vu32 DHR12R2;
  vu32 DHR12L2;
  vu32 DHR8R2;
  vu32 DHR12RD;
  vu32 DHR12LD;
  vu32 DHR8RD;
  vu32 DOR1;
  vu32 DOR2;
} DAC_TypeDef;

 
typedef struct
{
  vu32 IDCODE;
  vu32 CR;	
}DBGMCU_TypeDef;

 
typedef struct
{
  vu32 CCR;
  vu32 CNDTR;
  vu32 CPAR;
  vu32 CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  vu32 ISR;
  vu32 IFCR;
} DMA_TypeDef;

 
typedef struct
{
  vu32 IMR;
  vu32 EMR;
  vu32 RTSR;
  vu32 FTSR;
  vu32 SWIER;
  vu32 PR;
} EXTI_TypeDef;

 
typedef struct
{
  vu32 ACR;
  vu32 KEYR;
  vu32 OPTKEYR;
  vu32 SR;
  vu32 CR;
  vu32 AR;
  vu32 RESERVED;
  vu32 OBR;
  vu32 WRPR;
} FLASH_TypeDef;

typedef struct
{
  vu16 RDP;
  vu16 USER;
  vu16 Data0;
  vu16 Data1;
  vu16 WRP0;
  vu16 WRP1;
  vu16 WRP2;
  vu16 WRP3;
} OB_TypeDef;

 
typedef struct
{
  vu32 BTCR[8];   
} FSMC_Bank1_TypeDef; 

typedef struct
{
  vu32 BWTR[7];
} FSMC_Bank1E_TypeDef;

typedef struct
{
  vu32 PCR2;
  vu32 SR2;
  vu32 PMEM2;
  vu32 PATT2;
  u32  RESERVED0;   
  vu32 ECCR2; 
} FSMC_Bank2_TypeDef;  

typedef struct
{
  vu32 PCR3;
  vu32 SR3;
  vu32 PMEM3;
  vu32 PATT3;
  u32  RESERVED0;   
  vu32 ECCR3; 
} FSMC_Bank3_TypeDef; 

typedef struct
{
  vu32 PCR4;
  vu32 SR4;
  vu32 PMEM4;
  vu32 PATT4;
  vu32 PIO4; 
} FSMC_Bank4_TypeDef; 

 
typedef struct
{
  vu32 CRL;
  vu32 CRH;
  vu32 IDR;
  vu32 ODR;
  vu32 BSRR;
  vu32 BRR;
  vu32 LCKR;
} GPIO_TypeDef;

typedef struct
{
  vu32 EVCR;
  vu32 MAPR;
  vu32 EXTICR[4];
} AFIO_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 OAR1;
  u16  RESERVED2;
  vu16 OAR2;
  u16  RESERVED3;
  vu16 DR;
  u16  RESERVED4;
  vu16 SR1;
  u16  RESERVED5;
  vu16 SR2;
  u16  RESERVED6;
  vu16 CCR;
  u16  RESERVED7;
  vu16 TRISE;
  u16  RESERVED8;
} I2C_TypeDef;

 
typedef struct
{
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

 
typedef struct
{
  vu32 ISER[2];
  u32  RESERVED0[30];
  vu32 ICER[2];
  u32  RSERVED1[30];
  vu32 ISPR[2];
  u32  RESERVED2[30];
  vu32 ICPR[2];
  u32  RESERVED3[30];
  vu32 IABR[2];
  u32  RESERVED4[62];
  vu32 IPR[15];
} NVIC_TypeDef;

typedef struct
{
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CSR;
} PWR_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_TypeDef;

 
typedef struct
{
  vu16 CRH;
  u16  RESERVED0;
  vu16 CRL;
  u16  RESERVED1;
  vu16 PRLH;
  u16  RESERVED2;
  vu16 PRLL;
  u16  RESERVED3;
  vu16 DIVH;
  u16  RESERVED4;
  vu16 DIVL;
  u16  RESERVED5;
  vu16 CNTH;
  u16  RESERVED6;
  vu16 CNTL;
  u16  RESERVED7;
  vu16 ALRH;
  u16  RESERVED8;
  vu16 ALRL;
  u16  RESERVED9;
} RTC_TypeDef;

 
typedef struct
{
  vu32 POWER;
  vu32 CLKCR;
  vu32 ARG;
  vu32 CMD;
  vuc32 RESPCMD;
  vuc32 RESP1;
  vuc32 RESP2;
  vuc32 RESP3;
  vuc32 RESP4;
  vu32 DTIMER;
  vu32 DLEN;
  vu32 DCTRL;
  vuc32 DCOUNT;
  vuc32 STA;
  vu32 ICR;
  vu32 MASK;
  u32  RESERVED0[2];
  vuc32 FIFOCNT;
  u32  RESERVED1[13];
  vu32 FIFO;
} SDIO_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SR;
  u16  RESERVED2;
  vu16 DR;
  u16  RESERVED3;
  vu16 CRCPR;
  u16  RESERVED4;
  vu16 RXCRCR;
  u16  RESERVED5;
  vu16 TXCRCR;
  u16  RESERVED6;
  vu16 I2SCFGR;
  u16  RESERVED7;
  vu16 I2SPR;
  u16  RESERVED8;  
} SPI_TypeDef;

 
typedef struct
{
  vu32 CTRL;
  vu32 LOAD;
  vu32 VAL;
  vuc32 CALIB;
} SysTick_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SMCR;
  u16  RESERVED2;
  vu16 DIER;
  u16  RESERVED3;
  vu16 SR;
  u16  RESERVED4;
  vu16 EGR;
  u16  RESERVED5;
  vu16 CCMR1;
  u16  RESERVED6;
  vu16 CCMR2;
  u16  RESERVED7;
  vu16 CCER;
  u16  RESERVED8;
  vu16 CNT;
  u16  RESERVED9;
  vu16 PSC;
  u16  RESERVED10;
  vu16 ARR;
  u16  RESERVED11;
  vu16 RCR;
  u16  RESERVED12;
  vu16 CCR1;
  u16  RESERVED13;
  vu16 CCR2;
  u16  RESERVED14;
  vu16 CCR3;
  u16  RESERVED15;
  vu16 CCR4;
  u16  RESERVED16;
  vu16 BDTR;
  u16  RESERVED17;
  vu16 DCR;
  u16  RESERVED18;
  vu16 DMAR;
  u16  RESERVED19;
} TIM_TypeDef;

 
typedef struct
{
  vu16 SR;
  u16  RESERVED0;
  vu16 DR;
  u16  RESERVED1;
  vu16 BRR;
  u16  RESERVED2;
  vu16 CR1;
  u16  RESERVED3;
  vu16 CR2;
  u16  RESERVED4;
  vu16 CR3;
  u16  RESERVED5;
  vu16 GTPR;
  u16  RESERVED6;
} USART_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CFR;
  vu32 SR;
} WWDG_TypeDef;

 
 
 
 



 



 


 




#line 634 "..\\SYSTEM\\sys\\stm32f10x_map.h"

#line 651 "..\\SYSTEM\\sys\\stm32f10x_map.h"



#line 670 "..\\SYSTEM\\sys\\stm32f10x_map.h"

 

 


 






 


 






 
 
 

 



























































































































































































































#line 924 "..\\SYSTEM\\sys\\stm32f10x_map.h"














 
#line 1180 "..\\SYSTEM\\sys\\stm32f10x_map.h"

 
 
 



 
#line 4 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"














 

 



 
#line 23 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"

 
 
typedef struct
{
  u8 NVIC_IRQChannel;
  u8 NVIC_IRQChannelPreemptionPriority;
  u8 NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

 
 
#line 96 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"


#line 158 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"


 
#line 170 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"





#line 182 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"












#line 201 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"











 






 








 
#line 239 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"












 
 
void NVIC_DeInit(void);
void NVIC_SCBDeInit(void);
void NVIC_PriorityGroupConfig(u32 NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_StructInit(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SETPRIMASK(void);
void NVIC_RESETPRIMASK(void);
void NVIC_SETFAULTMASK(void);
void NVIC_RESETFAULTMASK(void);
void NVIC_BASEPRICONFIG(u32 NewPriority);
u32 NVIC_GetBASEPRI(void);
u16 NVIC_GetCurrentPendingIRQChannel(void);
ITStatus NVIC_GetIRQChannelPendingBitStatus(u8 NVIC_IRQChannel);
void NVIC_SetIRQChannelPendingBit(u8 NVIC_IRQChannel);
void NVIC_ClearIRQChannelPendingBit(u8 NVIC_IRQChannel);
u16 NVIC_GetCurrentActiveHandler(void);
ITStatus NVIC_GetIRQChannelActiveBitStatus(u8 NVIC_IRQChannel);
u32 NVIC_GetCPUID(void);
void NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void NVIC_GenerateSystemReset(void);
void NVIC_GenerateCoreReset(void);
void NVIC_SystemLPConfig(u8 LowPowerMode, FunctionalState NewState);
void NVIC_SystemHandlerConfig(u32 SystemHandler, FunctionalState NewState);
void NVIC_SystemHandlerPriorityConfig(u32 SystemHandler, u8 SystemHandlerPreemptionPriority,
                                      u8 SystemHandlerSubPriority);
ITStatus NVIC_GetSystemHandlerPendingBitStatus(u32 SystemHandler);
void NVIC_SetSystemHandlerPendingBit(u32 SystemHandler);
void NVIC_ClearSystemHandlerPendingBit(u32 SystemHandler);
ITStatus NVIC_GetSystemHandlerActiveBitStatus(u32 SystemHandler);
u32 NVIC_GetFaultHandlerSources(u32 SystemHandler);
u32 NVIC_GetFaultAddress(u32 SystemHandler);



 
#line 5 "..\\SYSTEM\\sys\\sys.h"





























																	    
	 







#line 50 "..\\SYSTEM\\sys\\sys.h"

#line 58 "..\\SYSTEM\\sys\\sys.h"
 
























#line 92 "..\\SYSTEM\\sys\\sys.h"
								   







void Stm32_Clock_Init(u8 PLL);  
void Sys_Soft_Reset(void);      
void Sys_Standby(void);         
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);
void JTAG_Set(u8 mode);


void WFI_SET(void);		
void INTX_DISABLE(void);
void INTX_ENABLE(void);	
void MSR_MSP(u32 addr);	















#line 3 "test.c"
#line 1 "..\\SYSTEM\\usart\\usart.h"
#line 4 "..\\SYSTEM\\usart\\usart.h"
#line 5 "..\\SYSTEM\\usart\\usart.h"
#line 1 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"
 
 
 





 






 









#line 34 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 125 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 944 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"



 
#line 6 "..\\SYSTEM\\usart\\usart.h"



























	  	
extern u8  USART_RX_BUF[200]; 
extern u16 USART_RX_STA;         		

void uart_init(u32 pclk2,u32 bound);


















#line 4 "test.c"
#line 1 "..\\SYSTEM\\delay\\delay.h"
#line 4 "..\\SYSTEM\\delay\\delay.h"






























void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);































#line 5 "test.c"
#line 1 "..\\HARDWARE\\LED\\led.h"
#line 4 "..\\HARDWARE\\LED\\led.h"














#line 24 "..\\HARDWARE\\LED\\led.h"





void LED_Init(void);


















#line 6 "test.c"
#line 1 "..\\HARDWARE\\BEEP\\beep.h"
#line 4 "..\\HARDWARE\\BEEP\\beep.h"















void BEEP_Init(void);	


















#line 7 "test.c"
#line 1 "..\\HARDWARE\\KEY\\key.h"
#line 4 "..\\HARDWARE\\KEY\\key.h"
























void KEY_Init(void);
u8 KEY_Scan(u8);  	
#line 8 "test.c"
#line 1 "..\\HARDWARE\\EXTI\\exti.h"
#line 4 "..\\HARDWARE\\EXTI\\exti.h"












void EXTIX_Init(void);


























#line 9 "test.c"
#line 1 "..\\HARDWARE\\WDG\\wdg.h"
#line 4 "..\\HARDWARE\\WDG\\wdg.h"
















void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);
void WWDG_Init(u8 tr,u8 wr,u8 fprer);
void WWDG_Set_Counter(u8 cnt);





























#line 10 "test.c"
#line 1 "..\\HARDWARE\\TIMER\\timer.h"
#line 4 "..\\HARDWARE\\TIMER\\timer.h"




























void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM5_Cap_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM3_ARR_Update(u16 arr);

void TIM1_Int_Init(u16 arr,u16 psc);
void TIM1_ARR_Update(u16 arr);

void TIM6_Int_Init(u16 arr,u16 psc);
void TIM6_ARR_Update(u16 arr);

void TIM8_Int_Init(u16 arr,u16 psc);
void TIM8_ARR_Update(u16 arr);


void TIM7_Int_Init(u16 arr,u16 psc);
void TIM7_ARR_Update(u16 arr);
void TIM7_Init(void);
void TIM7_Stop(void);
void TIM7_IRQHandler(void);
























#line 11 "test.c"
#line 1 "..\\HARDWARE\\TPAD\\tpad.h"
#line 4 "..\\HARDWARE\\TPAD\\tpad.h"
#line 5 "..\\HARDWARE\\TPAD\\tpad.h"












	   


extern vu16 tpad_default_val;
							   	    
void TPAD_Reset(void);
u16  TPAD_Get_Val(void);
u16 TPAD_Get_MaxVal(u8 n);
u8   TPAD_Init(u8 systick);
u8   TPAD_Scan(u8 mode);
void TIM5_CH2_Cap_Init(u16 arr,u16 psc);    
























#line 12 "test.c"
#line 1 "..\\HARDWARE\\OLED\\oled.h"
#line 4 "..\\HARDWARE\\OLED\\oled.h"
#line 1 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"
 
 
 




 
 



 












  


 








#line 45 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"


  
  typedef unsigned int size_t;










    



    typedef unsigned short wchar_t;  
#line 74 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { __int64 quot, rem; } lldiv_t;
    


#line 95 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"
   



 

   




 
#line 114 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) __int64 atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) __int64 strtoll(const char * __restrict  ,
                               char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned __int64 strtoull(const char * __restrict  ,
                                         char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 
typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 389 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 477 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 506 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"

extern __declspec(__nothrow) __pure int abs(int  );
   



 

extern __declspec(__nothrow) __pure div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __pure long int labs(long int  );
   



 




extern __declspec(__nothrow) __pure ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __pure __int64 llabs(__int64  );
   



 




extern __declspec(__nothrow) __pure lldiv_t lldiv(__int64  , __int64  );
   











 
#line 587 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"



 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __pure __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 



 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 832 "C:\\Keil\\ARM\\RV31\\INC\\stdlib.h"


 
#line 5 "..\\HARDWARE\\OLED\\oled.h"

















		    						  








  



		     



void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	 

	 



#line 13 "test.c"
#line 1 "..\\HARDWARE\\LCD\\lcd.h"
#line 4 "..\\HARDWARE\\LCD\\lcd.h"
#line 5 "..\\HARDWARE\\LCD\\lcd.h"

















































 
  

typedef struct  
{										    
	u16 width;			
	u16 height;			
	u16 id;				
	u8  dir;			
	u8	wramcmd;		
	u8  setxcmd;		
	u8  setycmd;		
}_lcd_dev; 	  


extern _lcd_dev lcddev;	

extern u16  POINT_COLOR;
extern u16  BACK_COLOR; 






typedef struct
{
	u16 LCD_REG;
	u16 LCD_RAM;
} LCD_TypeDef;





	 














#line 118 "..\\HARDWARE\\LCD\\lcd.h"






 






	    															  
void LCD_Init(void);													   	
void LCD_DisplayOn(void);													
void LCD_DisplayOff(void);													
void LCD_Clear(u16 Color);	 												
void LCD_SetCursor(u16 Xpos, u16 Ypos);										
void LCD_DrawPoint(u16 x,u16 y);											
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color);								
u16  LCD_ReadPoint(u16 x,u16 y); 											
void Draw_Circle(u16 x0,u16 y0,u8 r);										
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);							
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);		   				
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);		   				
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color);				
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode);						
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size);  						
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode);				
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p);		

void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);		  
void LCD_Scan_Dir(u8 dir);							
void LCD_Display_Dir(u8 dir);						
 					   																			 

#line 267 "..\\HARDWARE\\LCD\\lcd.h"
	 
	 



#line 14 "test.c"
#line 1 "..\\USMART\\usmart.h"
#line 1 "..\\USMART\\usmart_str.h"





















































typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;
						  
u8 usmart_get_parmpos(u8 num);						
u8 usmart_strcmp(u8*str1,u8 *str2);					
u32 usmart_pow(u8 m,u8 n);							
u8 usmart_str2num(u8*str,u32 *res);					
u8 usmart_get_cmdname(u8*str,u8*cmdname,u8 *nlen,u8 maxlen);
u8 usmart_get_fname(u8*str,u8*fname,u8 *pnum,u8 *rval);		
u8 usmart_get_aparm(u8 *str,u8 *fparm,u8 *ptype); 	
u8 usmart_get_fparam(u8*str,u8 *parn);  			












#line 4 "..\\USMART\\usmart.h"












































































 
struct _m_usmart_nametab
{
	void* func;			
	const u8* name;		
};

struct _m_usmart_dev
{
	struct _m_usmart_nametab *funs;	

	void (*init)(u8);				
	u8 (*cmd_rec)(u8*str);			
	void (*exe)(void); 				
	void (*scan)(void);             
	u8 fnum; 				  		
	u8 pnum;                        
	u8 id;							
	u8 sptype;						
	u16 parmtype;					
	u8  plentbl[10];  		
	u8  parm[200];  			
};
extern struct _m_usmart_nametab usmart_nametab[];	
extern struct _m_usmart_dev usmart_dev;				


void usmart_init(u8 sysclk);
u8 usmart_cmd_rec(u8*str);	
void usmart_exe(void);		
void usmart_scan(void);     
u32 read_addr(u32 addr);	
void write_addr(u32 addr,u32 val);































#line 15 "test.c"
#line 1 "..\\HARDWARE\\RTC\\rtc.h"
















typedef struct 
{
	vu8 hour;
	vu8 min;
	vu8 sec;			
	
	vu16 w_year;
	vu8  w_month;
	vu8  w_date;
	vu8  week;		 
}_calendar_obj;					 
extern _calendar_obj calendar;	
												    
void Disp_Time(u8 x,u8 y,u8 size);			
void Disp_Week(u8 x,u8 y,u8 size,u8 lang);	
u8 RTC_Init(void);        
u8 Is_Leap_Year(u16 year);
u8 RTC_Get(void);         
u8 RTC_Get_Week(u16 year,u8 month,u8 day);
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);





























 
#line 16 "test.c"
#line 1 "..\\HARDWARE\\WKUP\\wkup.h"
#line 4 "..\\HARDWARE\\WKUP\\wkup.h"












					    

	 
u8 Check_WKUP(void);  			
void WKUP_Init(void); 			
void Sys_Enter_Standby(void);	



#line 17 "test.c"
#line 1 "..\\HARDWARE\\ADC\\adc.h"
#line 4 "..\\HARDWARE\\ADC\\adc.h"
















							  







	   									   
void Adc_Init(void); 				
u16  Get_Adc(u8 ch); 				
u16 Get_Adc_Average(u8 ch,u8 times);
















#line 18 "test.c"
#line 1 "..\\HARDWARE\\DAC\\dac.h"
#line 4 "..\\HARDWARE\\DAC\\dac.h"












								    

void Dac1_Init(void);		
void Dac1_Set_Vol(u16 vol);	


















#line 19 "test.c"
#line 1 "..\\HARDWARE\\DMA\\dma.h"
#line 4 "..\\HARDWARE\\DMA\\dma.h"












							    					    

void MYDMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);































#line 20 "test.c"
#line 1 "..\\HARDWARE\\24CXX\\24cxx.h"
#line 1 "..\\HARDWARE\\24CXX\\myiic.h"
#line 4 "..\\HARDWARE\\24CXX\\myiic.h"













   	   		   










void IIC_Init(void);                
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

















#line 4 "..\\HARDWARE\\24CXX\\24cxx.h"













#line 26 "..\\HARDWARE\\24CXX\\24cxx.h"


					  
u8 AT24CXX_ReadOneByte(u16 ReadAddr);							
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	

u8 AT24CXX_Check(void);  
void AT24CXX_Init(void); 

















#line 21 "test.c"
#line 1 "..\\HARDWARE\\FLASH\\flash.h"
#line 4 "..\\HARDWARE\\FLASH\\flash.h"























extern u16 SPI_FLASH_TYPE;		

				 

 

#line 49 "..\\HARDWARE\\FLASH\\flash.h"

void SPI_Flash_Init(void);
u16  SPI_Flash_ReadID(void);  	    
u8	 SPI_Flash_ReadSR(void);        
void SPI_FLASH_Write_SR(u8 sr);  	
void SPI_FLASH_Write_Enable(void);  
void SPI_FLASH_Write_Disable(void);	
void SPI_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void SPI_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   
void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void SPI_Flash_Erase_Chip(void);    	  
void SPI_Flash_Erase_Sector(u32 Dst_Addr);
void SPI_Flash_Wait_Busy(void);           
void SPI_Flash_PowerDown(void);           
void SPI_Flash_WAKEUP(void);			  

















#line 22 "test.c"
#line 1 "..\\HARDWARE\\SPI\\spi.h"
#line 4 "..\\HARDWARE\\SPI\\spi.h"













#line 1 "..\\USER\\user_def.h"



















#line 29 "..\\USER\\user_def.h"





















 
#line 61 "..\\USER\\user_def.h"


















 









extern u8 RxBuffer[64];
extern u8 RxIdx;
extern u8 RxStage;

extern u16 TMR_Int_Flag; 
extern  u8 PixReadmMode;
extern u16 InteCnt;
extern u8 PixReadmMode;







#line 110 "..\\USER\\user_def.h"
				 























	





#line 146 "..\\USER\\user_def.h"




#line 159 "..\\USER\\user_def.h"





#line 174 "..\\USER\\user_def.h"

#line 183 "..\\USER\\user_def.h"





       


   
	   







#line 209 "..\\USER\\user_def.h"
 
typedef struct
{
  u8 RampTrim;
  u8 RangTrim;
  u8 Ipix_V24Trim;		 
  u8 V20_V15Trim; 		 
  u8 SW_TxCtrl; 		  
  u8 TsTADC_AmuxCtrl;    

  u8 SpiInsertion;		 
  u16 InteTime;			 
  u16 InteCount;		 
  u16 InteDelayCount;	

} PCR_Regs_type;
extern PCR_Regs_type PCR_Regs;




























#line 264 "..\\USER\\user_def.h"



#line 277 "..\\USER\\user_def.h"




									  






									   










 
#line 307 "..\\USER\\user_def.h"
void OSC_Ctrl_Init(void);
extern u8 OSC_Status;
extern u8 OSC_mode;
extern u8 OSC_Busy;
extern u16 UserCountMS;
extern u16 BaseCounter;


                       
typedef enum
{	
    READY,
	WAIT,
	ACTIVE
}Cycle_STS_Type;

extern Cycle_STS_Type CycleSTS;






#line 345 "..\\USER\\user_def.h"

#line 363 "..\\USER\\user_def.h"







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


















typedef u8 MSG_TYP;



	void EpMsgStk(MSG_TYP u8Q);
	MSG_TYP EpMsgPop(void);
	void EpMsgClr(void);








	
#line 18 "..\\HARDWARE\\SPI\\spi.h"

#line 27 "..\\HARDWARE\\SPI\\spi.h"


extern u8 SPI_2_SendBuf[40];
extern u8 SPI_2_RcvBuf[40];
extern u8 SPI_2_CommLen;
extern u8 SPI_2_Rx_sts;

void SPI_2_Trans(u8 TransLen);						  	    													  
void SPI2_Init(void);			 
void SPI2_SetSpeed(u8 SpeedSet); 
u8 SPI2_ReadWriteByte(u8 TxData);


void SPI2_Initializaion(void);
void SPI_Rev_Data_Copy(u8 length);
extern u8 RowData[40]; 

		 


#line 23 "test.c"
#line 24 "test.c"
#line 1 "temperature.h"












#line 23 "temperature.h"














typedef enum {TEMP1=1,TEMP2} TempControlNum;


void TempControl_Initial(float TempSet);
void TempControl_2_Initial(float TempSet);
float  TempSensorRead(u8 IIC_Addr);







bool Sensor_Cfg_Write(u8 idx, u16 dt);
u16 Sensor_Cfg_Read(u8 idx);


void Sensor_Res_Init(void);

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
extern u16 TempCtrlTime_Sec;   
extern u16 TempTickLength;
extern u16 TempCtrlTime_Sec_2;   
extern u16 TempTickLength_2;
extern u8 TempCtrl_Active;
extern u8 TempCtrl_Active_2;
extern u8 TempValid;
extern u8 Control_2_Dir;
extern float TempSet;
extern float TempSet_2;
extern float TempPumpSet;
extern u8 TickLock;
extern u8 Peltier_Swap_Msg;

extern float Kp[2];
extern float Ki[2];
extern float Kd[2];
extern float Kl[2];
extern float Ktm[2];
typedef struct PID_Err
{
	float Current_Err;
	float Previous_Err;
	float Current_Derr;
	float Previous_Derr;
	float Inte_Err;
}PID_Err_Rec_type;






extern PID_Err_Rec_type PID_Err_Rec[2];

#line 122 "temperature.h"




	
typedef u8 MsgQ;




extern MsgQ MsgStk;

MsgQ PollMsg(void);
void MsgHandler(void);
void PopMsg(u8 msk);
void PushMsg(u8 msk);
void ClearMsg(void);
#line 25 "test.c"
#line 1 "PCR_Cycle.h"



 
#line 15 "PCR_Cycle.h"
 
  











void FAN_Init(void);

extern CycleControlType  PCR_Cycle_Control,Buffer_Cycle_Control;
extern CycleTempType	 PCR_Cycle_SetPoint[10],Buffer_Cycle_SetPoint[10], *pCycleArray;
extern CycleTempType     PE_Cycle_SetPoint[2];




void FanCtrl_Force(u8 ctrl);   
void FanCtrl_Auto(u8 ctrl);	   
void FanMode_Clear(void);      

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

#line 26 "test.c"
#line 1 "usb_lib.h"













 

 



 
#line 1 "..\\USB\\LIB\\usb_type.h"













 

 



 
#line 1 "..\\USB\\CONFIG\\usb_conf.h"













 

 



 
 
 
 
 
 
 
 
 
 


 
 
 
 
 


 
 



 
 







 
 
 
 
 
 



 
 









#line 79 "..\\USB\\CONFIG\\usb_conf.h"



 

#line 22 "..\\USB\\LIB\\usb_type.h"

 
 




#line 68 "..\\USB\\LIB\\usb_type.h"

 
 
 



 
#line 22 "usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_regs.h"













 

 



 
 
typedef enum _EP_DBUF_DIR
{
   
  EP_DBUF_ERR,
  EP_DBUF_OUT,
  EP_DBUF_IN
}EP_DBUF_DIR;

 
enum EP_BUF_NUM
{
  EP_NOBUF,
  EP_BUF0,
  EP_BUF1
};

 



 
 
 

 

 

 

 

 

 
 
 


 
#line 70 "..\\USB\\LIB\\usb_regs.h"
 
 
 
#line 81 "..\\USB\\LIB\\usb_regs.h"





#line 94 "..\\USB\\LIB\\usb_regs.h"

 
 
 
#line 106 "..\\USB\\LIB\\usb_regs.h"








 
 
 





 
 
 


 
 
 
 
#line 141 "..\\USB\\LIB\\usb_regs.h"

 


 
#line 152 "..\\USB\\LIB\\usb_regs.h"


 


 
#line 165 "..\\USB\\LIB\\usb_regs.h"

 
#line 174 "..\\USB\\LIB\\usb_regs.h"
 
 


 


 


 


 


 


 


 


 


 



 









 









 









 
#line 248 "..\\USB\\LIB\\usb_regs.h"








 
#line 269 "..\\USB\\LIB\\usb_regs.h"







 










 










 











 











 









 









 











 











 











 









 














 









 










 
#line 431 "..\\USB\\LIB\\usb_regs.h"

#line 438 "..\\USB\\LIB\\usb_regs.h"




















 











 










 











 











 












 
#line 527 "..\\USB\\LIB\\usb_regs.h"

#line 536 "..\\USB\\LIB\\usb_regs.h"












 




 
extern volatile u16 wIstr;   

 
void SetCNTR(u16  );
void SetISTR(u16  );
void SetDADDR(u16  );
void SetBTABLE(u16  );
void SetBTABLE(u16  );
u16 GetCNTR(void);
u16 GetISTR(void);
u16 GetFNR(void);
u16 GetDADDR(void);
u16 GetBTABLE(void);
void SetENDPOINT(u8  , u16  );
u16 GetENDPOINT(u8  );
void SetEPType(u8  , u16  );
u16 GetEPType(u8  );
void SetEPTxStatus(u8  , u16  );
void SetEPRxStatus(u8  , u16  );
void SetDouBleBuffEPStall(u8  , u8 bDir);
u16 GetEPTxStatus(u8  );
u16 GetEPRxStatus(u8  );
void SetEPTxValid(u8  );
void SetEPRxValid(u8  );
u16 GetTxStallStatus(u8  );
u16 GetRxStallStatus(u8  );
void SetEP_KIND(u8  );
void ClearEP_KIND(u8  );
void Set_Status_Out(u8  );
void Clear_Status_Out(u8  );
void SetEPDoubleBuff(u8  );
void ClearEPDoubleBuff(u8  );
void ClearEP_CTR_RX(u8  );
void ClearEP_CTR_TX(u8  );
void ToggleDTOG_RX(u8  );
void ToggleDTOG_TX(u8  );
void ClearDTOG_RX(u8  );
void ClearDTOG_TX(u8  );
void SetEPAddress(u8  , u8  );
u8 GetEPAddress(u8  );
void SetEPTxAddr(u8  , u16  );
void SetEPRxAddr(u8  , u16  );
u16 GetEPTxAddr(u8  );
u16 GetEPRxAddr(u8  );
void SetEPCountRxReg(u32 *  , u16  );
void SetEPTxCount(u8  , u16  );
void SetEPRxCount(u8  , u16  );
u16 GetEPTxCount(u8  );
u16 GetEPRxCount(u8  );
void SetEPDblBuf0Addr(u8  , u16  );
void SetEPDblBuf1Addr(u8  , u16  );
void SetEPDblBuffAddr(u8  , u16  , u16  );
u16 GetEPDblBuf0Addr(u8  );
u16 GetEPDblBuf1Addr(u8  );
void SetEPDblBuffCount(u8  , u8  , u16  );
void SetEPDblBuf0Count(u8  , u8  , u16  );
void SetEPDblBuf1Count(u8  , u8  , u16  );
u16 GetEPDblBuf0Count(u8  );
u16 GetEPDblBuf1Count(u8  );
EP_DBUF_DIR GetEPDblBufDir(u8  );
void FreeUserBuffer(u8 bEpNum , u8 bDir);
u16 ToWord(u8, u8);
u16 ByteSwap(u16);



 
#line 23 "usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_def.h"













 

 



 
 
typedef enum _RECIPIENT_TYPE
{
  DEVICE_RECIPIENT,      
  INTERFACE_RECIPIENT,   
  ENDPOINT_RECIPIENT,    
  OTHER_RECIPIENT
} RECIPIENT_TYPE;


typedef enum _STANDARD_REQUESTS
{
  GET_STATUS = 0,
  CLEAR_FEATURE,
  RESERVED1,
  SET_FEATURE,
  RESERVED2,
  SET_ADDRESS,
  GET_DESCRIPTOR,
  SET_DESCRIPTOR,
  GET_CONFIGURATION,
  SET_CONFIGURATION,
  GET_INTERFACE,
  SET_INTERFACE,
  TOTAL_sREQUEST,   
  SYNCH_FRAME = 12
} STANDARD_REQUESTS;

 
typedef enum _DESCRIPTOR_TYPE
{
  DEVICE_DESCRIPTOR = 1,
  CONFIG_DESCRIPTOR,
  STRING_DESCRIPTOR,
  INTERFACE_DESCRIPTOR,
  ENDPOINT_DESCRIPTOR
} DESCRIPTOR_TYPE;

 
typedef enum _FEATURE_SELECTOR
{
  ENDPOINT_STALL,
  DEVICE_REMOTE_WAKEUP
} FEATURE_SELECTOR;

 
 







 
 



 
#line 24 "usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_core.h"













 

 



 
 
typedef enum _CONTROL_STATE
{
  WAIT_SETUP,        
  SETTING_UP,        
  IN_DATA,           
  OUT_DATA,          
  LAST_IN_DATA,      
  LAST_OUT_DATA,     
  WAIT_STATUS_IN,    
  WAIT_STATUS_OUT,   
  STALLED,           
  PAUSE              
} CONTROL_STATE;     

typedef struct OneDescriptor
{
  u8 *Descriptor;
  u16 Descriptor_Size;
}
ONE_DESCRIPTOR, *PONE_DESCRIPTOR;


 
typedef enum _RESULT
{
  USB_SUCCESS = 0,     
  USB_ERROR,
  USB_UNSUPPORT,
  USB_NOT_READY       
 
} RESULT;


 
typedef struct _ENDPOINT_INFO
{
  




















 
  u16  Usb_wLength;
  u16  Usb_wOffset;
  u16  PacketSize;
  u8   *(*CopyData)(u16 Length);
}ENDPOINT_INFO;

 

typedef struct _DEVICE
{
  u8 Total_Endpoint;      
  u8 Total_Configuration; 
}
DEVICE;

typedef union
{
  u16 w;
  struct BW
  {
    u8 bb1;
    u8 bb0;
  }
  bw;
} u16_u8;

typedef struct _DEVICE_INFO
{
  u8 USBbmRequestType;        
  u8 USBbRequest;             
  u16_u8 USBwValues;          
  u16_u8 USBwIndexs;          
  u16_u8 USBwLengths;         

  u8 ControlState;            
  u8 Current_Feature;
  u8 Current_Configuration;    
  u8 Current_Interface;        
  u8 Current_AlternateSetting;
 

  ENDPOINT_INFO Ctrl_Info;
}DEVICE_INFO;

typedef struct _DEVICE_PROP
{
  void (*Init)(void);         
  void (*Reset)(void);        

   
  void (*Process_Status_IN)(void);
  void (*Process_Status_OUT)(void);

   
  













 
  RESULT (*Class_Data_Setup)(u8 RequestNo);

   
  






 
  RESULT (*Class_NoData_Setup)(u8 RequestNo);

  





 

  RESULT  (*Class_Get_Interface_Setting)(u8 Interface, u8 AlternateSetting);

  u8* (*GetDeviceDescriptor)(u16 Length);
  u8* (*GetConfigDescriptor)(u16 Length);
  u8* (*GetStringDescriptor)(u16 Length);

  u8* RxEP_buffer;
  u8 MaxPacketSize;

}DEVICE_PROP;

typedef struct _USER_STANDARD_REQUESTS
{
  void (*User_GetConfiguration)(void);        
  void (*User_SetConfiguration)(void);        
  void (*User_GetInterface)(void);            
  void (*User_SetInterface)(void);            
  void (*User_GetStatus)(void);               
  void (*User_ClearFeature)(void);            
  void (*User_SetEndPointFeature)(void);      
  void (*User_SetDeviceFeature)(void);        
  void (*User_SetDeviceAddress)(void);        
}
USER_STANDARD_REQUESTS;

 





#line 210 "..\\USB\\LIB\\usb_core.h"

 
 
u8 Setup0_Process(void);
u8 Post0_Process(void);
u8 Out0_Process(void);
u8 In0_Process(void);

RESULT Standard_SetEndPointFeature(void);
RESULT Standard_SetDeviceFeature(void);

u8 *Standard_GetConfiguration(u16 Length);
RESULT Standard_SetConfiguration(void);
u8 *Standard_GetInterface(u16 Length);
RESULT Standard_SetInterface(void);
u8 *Standard_GetDescriptorData(u16 Length, PONE_DESCRIPTOR pDesc);

u8 *Standard_GetStatus(u16 Length);
RESULT Standard_ClearFeature(void);
void SetDeviceAddress(u8);
void NOP_Process(void);

extern DEVICE_PROP Device_Property;
extern  USER_STANDARD_REQUESTS User_Standard_Requests;
extern  DEVICE  Device_Table;
extern DEVICE_INFO Device_Info;

 
extern u16 SaveRState;
extern u16 SaveTState;



 
#line 25 "usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_init.h"













 

 



 
 
 
 
 
void USB_Init(void);

 
 
extern u8	EPindex;
 
 
 
 
extern DEVICE_INFO*	pInformation;
 
 
extern DEVICE_PROP*	pProperty;
 
 
 
 
extern USER_STANDARD_REQUESTS *pUser_Standard_Requests;

extern u16	SaveState ;
extern u16 wInterrupt_Mask;



 
#line 26 "usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_mem.h"













 

 



 
 
 
 
 
void UserToPMABufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);
void PMAToUserBufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);

 



 
#line 27 "usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_int.h"














 

 



 
 
 
 
 
void CTR_LP(void);
void CTR_HP(void);

 



 
#line 28 "usb_lib.h"

 
 
 
 
 



 
#line 27 "test.c"
#line 1 "..\\USB\\CONFIG\\hw_config.h"













 

 



 
#line 22 "..\\USB\\CONFIG\\hw_config.h"

 
 
 
 






 
void Set_System(void);
void Set_USBClock(void);
void GPIO_AINConfig(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void Joystick_Send(u8 buf0,u8 buf1,u8 buf2,u8 buf3);
u8 JoyState(void);
void Get_SerialNum(void);



 
#line 28 "test.c"
#line 1 "..\\USB\\CONFIG\\usb_pwr.h"













 

 



 
 
typedef enum _RESUME_STATE
{
  RESUME_EXTERNAL,
  RESUME_INTERNAL,
  RESUME_LATER,
  RESUME_WAIT,
  RESUME_START,
  RESUME_ON,
  RESUME_OFF,
  RESUME_ESOF
} RESUME_STATE;

typedef enum _DEVICE_STATE
{
  UNCONNECTED,
  ATTACHED,
  POWERED,
  SUSPENDED,
  ADDRESSED,
  CONFIGURED
} DEVICE_STATE;

 
 
 
void Suspend(void);
void Resume_Init(void);
void Resume(RESUME_STATE eResumeSetVal);
RESULT PowerOn(void);
RESULT PowerOff(void);
 
extern  vu32 bDeviceState;  
extern volatile bool fSuspendEnabled;   



 
#line 29 "test.c"
#line 30 "test.c"
extern u8 Receive_Buffer [64];
extern u8 Transi_Buffer [64];
extern u8 USB_ReceiveFlg;
extern u16 USB_RxIdx;





 				 	

const u8 TEXT_Buffer[]={"WarShipSTM32 SPI TEST"};


u8 u8buff[40]={0};
u8 PCRChip_Command_Send[40]={0};
u16 TMR_Int_Flag=0;
u8 PCR_ADC_Done_Flag=0;
void PIX_Drive_Sequence_Sim(void);
u8 Send_Command(u8 cmd, u8 row_num, u8 length, u8 * pbuf);

u8 Comand_Buf[64]={0};
u8 Command_Len=0;
u8 TxBuffer[64]={0xAA};
u8 PacketChkSum(u8 *p, u8 length);
void UARTRes(u8 response, u8 command,u8 * buffer, u8 length);
void USBRes(u8 response, u8 command,u8 * buffer, u8 length);
void Read_Row(u8 row);
void ReadUpdate_Image(void);


void ReadUpdate_Image24_XC(u8 mode,u8 row);
void ReadUpdate_Image24_slow(u8 mode, u8 row);
PCR_Regs_type PCR_Regs;

u16 InteCnt=0;	 
u8 PixReadmMode;


u8 ImageBuf[12][((12+1)<<1)+2]={0};
u8 ImageBufPIX[64][64]={0};





 
void Print_packet(u8 ResCode, u8 command, u8 *TxBuffer, u8 TransLen);



u8 msg_debug=0;

void Trim_Reset(void);
void Read_Row_Debug_2(void);
KL_union KL_temp,KP_temp,KI_temp,KD_temp,TempSet_1_temp,TempSet_2_temp,TempCurr_1_temp,TempCurr_2_temp, UnionTemp;

CycleControlType  PCR_Cycle_Control,Buffer_Cycle_Control;
CycleTempType	  PCR_Cycle_SetPoint[10],Buffer_Cycle_SetPoint[10];
CycleTempType     *pCycleArray; 

CycleTempType     PE_Cycle_SetPoint[2]; 

Cycle_STS_Type CycleSTS; 

u8 USB_Reply_Tail=FALSE;
u8 const PIXEL_24READ[4]={0x1,0x2,0x4,0x8};



void EpMsgEnable(void);
void EpMsgDisable(void);
MSG_TYP UsbReadDone(void);











u8 mst_flag=0;  
u16 Time_LED_Delay=0; 	
u16 SetTm_LED_Delay=10;	
u16 HoldTm_LED_Delay=10; 

u8 led_mode=0; 


void usb_port_set(u8 enable)
{
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<2;    
	if(enable)(*((volatile unsigned *)((0x40005C00L) + 0x40)) = (u16)((u16) *((volatile unsigned *)((0x40005C00L) + 0x40)))&(~(1<<1)));
	else
	{	  
		(*((volatile unsigned *)((0x40005C00L) + 0x40)) = (u16)((u16) *((volatile unsigned *)((0x40005C00L) + 0x40)))|(1<<1));  
		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH&=0XFFF00FFF;
		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH|=0X00033000;
		*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x0800)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x0800)+12) &0xFFFFF)<<5)+(12<<2))))=0;	    		  
	}
} 
int main(void)
{
   	u8 tick;
	u16 adcx;
	u8 u8temp;	u8 CurrentRowNum;

	u8 u8temp1,u8temp2,u8temp3;
	u16 u16temp,u16temp1;
	float f32temp1,f32temp2;
	u8 ResCode;
	u8 TransLen;

	u8 VideoRow;

	
		
	u8 key,ldecoun=0;

	u8 test_lenth=0;
	u16 j,i=0;




    PCR_Cycle_Init();
	KL_temp.float_num=Kl[0];
	KP_temp.float_num=Kp[0];
	KI_temp.float_num=Ki[0];
	KD_temp.float_num=Kd[0];
	tick=0;
 	Stm32_Clock_Init(9);	
	uart_init(36,9600);	 	
	delay_init(72);	   	 	
	LED_Init();		  		
	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(11<<2))))=1;

    *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=0;

	FanCtrl_Init();
	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=0;
	OSC_mode=2;
	OSC_Status=0;

	{*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(8<<2))))=0;delay_us(10);*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(8<<2))))=1;};
																 
	Trim_Reset();	
	KEY_Init();				
	EXTIX_Init();
		 		  
	SPI2_Initializaion();

	EpMsgDisable();

	
	PCR_Regs.InteDelayCount=(600/(10));
	PCR_Regs.InteTime=1;
	PCR_Regs.InteCount=(100u);

	SPI_2_Rx_sts=0x0;
	
	TIM3_Int_Init(PCR_Regs.InteCount,719);
	TMR_Int_Flag=0;
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1|=0x01;
	while(TMR_Int_Flag==0);
	TMR_Int_Flag=0;

	Adc_Init();
	IIC_Init();
	TempContorl_2_Init();
	
    
	TempCtrl_Active=0;
	TempValid=0;
	Sensor_Res_Init();

 	usb_port_set(0); 	
	delay_ms(300);
   	usb_port_set(1);	
	
 	USB_Interrupts_Config();    
 	Set_USBClock();   
 	USB_Init();					

	while(1)
	{











 	   
		
 























 		















































 
		
		
        
		if(mst_flag==1)
		{
			ResCode=0;
			TransLen=1;
			TxBuffer[0]=(0| 0x1) ;
		
		}


















































 
		
		
		if(TempValid & 0x1)	
		{
			if(TempCtrl_Active==4)
			{
				TempCtrl_Active=0;
				TempValid &=~ 0x1;
				TempControl_stop();
			}else if(TempCtrl_Active==1)
					{
						TempCtrl_Active=2;
						
						TempControl_Initial(TempSet);
						TempCurrent=TempSensorRead(0x90);
					}
			
				   else if(TempCtrl_Active!=0)
				   		{
							

							if(TempCurrent<TempSet)
							     TempCurrent+=0.5;
						}
		}
			

		if(TempValid & 0x2)	
		{

			if((Peltier_Swap_Msg & 0x2) !=0) 
				Channel_Swap(0x2);
				
			if(TempCtrl_Active_2==4)
			{
				if(CycleSTS==READY)	  
				                      
				{
					TempCtrl_Active_2=0;
					TempValid &=~ 0x2;
					TempControl_2_stop();
				}



			}else if(TempCtrl_Active_2==1)
					{
						TempCtrl_Active_2=2;
						TempControl_2_Initial(TempSet_2);
						TempCurrent_2=TempSensorRead(0x92);
					}
			
				   else if(TempCtrl_Active_2!=0)
				   		{
							

							if(TempCurrent_2<TempSet_2)
							     TempCurrent_2+=0.5;
						}
 
#line 432 "test.c"
		}
		
		if(CycleSTS != READY)
			MsgHandler();


		if(PixReadmMode==0x3)
		{

			Read_Row(VideoRow);  
			ResCode=0;
		    TxBuffer[0]=0x3;
			TxBuffer[1]=VideoRow;
			for(i=0;i<((12+1)<<1);i++)
				TxBuffer[i+2]= SPI_2_RcvBuf[i];
#line 453 "test.c"
			TransLen=(12<<1)+2;
			USBRes(ResCode, 0x2,TxBuffer,TransLen);



			EpMsgEnable();
			UsbReadDone();
			EpMsgDisable();

			if((++VideoRow)==12)
				VideoRow=0;		
		}

		if(USB_ReceiveFlg==TRUE)
		{
			RxStage= 0x3;
			RxIdx=USB_RxIdx;
			USB_ReceiveFlg=FALSE;
		
		}
		












 
	   	if(RxStage==0x3)
		{
			Command_Len=RxIdx;
			for(i=0;i<Command_Len;i++)
				Comand_Buf[i]=RxBuffer[i];
			
			RxStage=0x0;
			RxIdx=0;
			
			if(PacketChkSum(Comand_Buf, Command_Len)==TRUE)
			{
					
				switch(Comand_Buf[1])
				{
					case 0x2:
							






















































































































































 
							
						break;
	

					case  0x1:
							if(Comand_Buf[2]<1)
							 {
							 	ResCode=1;
								TransLen=0;
							 }
							 else
							 {
								 switch (Comand_Buf[3])
								 {
								  	







































































































































































         

									case 0x21:
	      									ResCode=0;
											u16temp=0;
											u16temp = Comand_Buf[3+2];
											u16temp= ((u16temp <<8) |Comand_Buf[3+1]);  
											
 											UnionTemp.tempd.byte0= Comand_Buf[3+1];
											UnionTemp.tempd.byte1= Comand_Buf[3+2];
											UnionTemp.tempd.byte2= Comand_Buf[3+3];
											UnionTemp.tempd.byte3= Comand_Buf[3+4];

											if((UnionTemp.float_num>0)&&(UnionTemp.float_num<2000))
												SetTm_LED_Delay=UnionTemp.float_num;
											else
												ResCode=7;

											TxBuffer[0]= 0x21;
											TransLen=1;
										break;


 									case 0x22:
	      									ResCode=0;
											u16temp=0;
											u16temp = Comand_Buf[3+2];
											u16temp= ((u16temp <<8) |Comand_Buf[3+1]);  
										
 											UnionTemp.tempd.byte0= Comand_Buf[3+1];
											UnionTemp.tempd.byte1= Comand_Buf[3+2];
											UnionTemp.tempd.byte2= Comand_Buf[3+3];
											UnionTemp.tempd.byte3= Comand_Buf[3+4];

											if((UnionTemp.float_num>0)&&(UnionTemp.float_num<2000))
												HoldTm_LED_Delay=UnionTemp.float_num;
											else
												ResCode=7;

											TxBuffer[0]= 0x22;
											TransLen=1;

										break;

									 case 0x23:
											ResCode=0;
											TransLen=0;
									 		u8temp3=Comand_Buf[3+1];
									     	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=led_mode=(u8temp3 & 0x1);

										break;									 	

									default:  			
										ResCode=5;
											TxBuffer[0]= 0x22;
											TransLen=1;
										break;
								 }
							 }
							 USBRes(ResCode, 0x1, TxBuffer, TransLen);
						break;


					case 0x4:	
							if(Comand_Buf[2]<1)
							 {
							 	ResCode=1;
											TxBuffer[0]= 0x4;
											TransLen=1;
							 }
							 else
							 {
								 switch (Comand_Buf[3])
								 {
								  	





































































































































 
 									case 0x21:

 	      									ResCode=0;
										    TxBuffer[0]=0x21;
										
											UnionTemp.float_num= SetTm_LED_Delay;
											TxBuffer[1]= UnionTemp.tempd.byte0;
											TxBuffer[2]= UnionTemp.tempd.byte1;
											TxBuffer[3]= UnionTemp.tempd.byte2;
											TxBuffer[4]= UnionTemp.tempd.byte3;
											TransLen=5;
										break;

 									case 0x22:

 	      									ResCode=0;
										    TxBuffer[0]=0x22;
										
											UnionTemp.float_num= HoldTm_LED_Delay;
											TxBuffer[1]= UnionTemp.tempd.byte0;
											TxBuffer[2]= UnionTemp.tempd.byte1;
											TxBuffer[3]= UnionTemp.tempd.byte2;
											TxBuffer[4]= UnionTemp.tempd.byte3;
											TransLen=5;
										break;

									default:  			
										ResCode=5;
											TxBuffer[0]= 0x4;
											TransLen=1;
										break;
								 }
							 }
							 USBRes(ResCode, 0x4, TxBuffer, TransLen);

						break;

					case 0x8:

						break;
  					
					case 0x10: 
							






 
							 {
								 switch (Comand_Buf[3])  
								 {
								  	case 0x01:	
	      									ResCode=0;
											u8temp3=Comand_Buf[3+1];	 
											UnionTemp.tempd.byte0 = Comand_Buf[3+1+1];
											UnionTemp.tempd.byte1 = Comand_Buf[3+2+1];
											UnionTemp.tempd.byte2 = Comand_Buf[3+3+1];
											UnionTemp.tempd.byte3 = Comand_Buf[3+4+1];

											f32temp1=UnionTemp.float_num;
											if((f32temp1>0)&& (f32temp1<150))
											{
	
												u8temp1 = Comand_Buf[3+5+1];
											    u8temp2 = Comand_Buf[3+6+1];
												switch (u8temp3)
												{
												









 
														break;
													case 0x2:
														  TempCtrlTime_Sec_2= u8temp1;  
															TempCtrlTime_Sec_2<<=8;
															TempCtrlTime_Sec_2|=u8temp2;
															TempTickLength_2=  TempCtrlTime_Sec_2 * 0x6;
															TempSet_2= f32temp1;
															TempCtrl_Active_2=1;
															TempValid |= 2;
													    TxBuffer[0]= 0x01;
									      
														break;

													case 0x1:        
															TempPumpSet= f32temp1;	
													    TxBuffer[0]= 0x01;
														break;

													default:
															ResCode=6;
                       		    TxBuffer[0]= 0x01;
														break;
												}

											}
											else
											{
												ResCode=6;
                		    TxBuffer[0]= 0x01;
											}
											TransLen=1;
										break;

									case 0x00:	 		
											u8temp3=Comand_Buf[3+1];	 
											ResCode=0;
 											switch (u8temp3)
											{
											 	case 0x1:
														TempCtrl_Active=0;
														TempValid &= ~0x1;
														TempControl_stop();
													break;
												case 0x2:
														TempCtrl_Active_2=0;
														TempValid &= ~0x2;
														TempControl_2_stop();
													break;
												default:
														ResCode=6;
													break;
											}
               		    TxBuffer[0]= 0x00;
											TransLen=1;
										break;

									case 0x02:   
											u8temp3=Comand_Buf[3+1];	 
 	  										ResCode=0;
											TransLen=5;
 											switch (u8temp3)
											{
											 	case 0x1:
														UnionTemp.float_num= TempSensorRead(0x90);
													break;
												case 0x2:
														UnionTemp.float_num= TempSensorRead(0x92);
													break;
												default:
														ResCode=6;
												    TxBuffer[0]= 0x02;
														TransLen=1;
													break;
											}
											if(ResCode==0)
											{
							
												TxBuffer[0]= 0x02;
												TxBuffer[1]= UnionTemp.tempd.byte0;
												TxBuffer[2]= UnionTemp.tempd.byte1;
												TxBuffer[3]= UnionTemp.tempd.byte2;
												TxBuffer[4]= UnionTemp.tempd.byte3;
		
											}											
										break;

									case 0x03:   

											u8temp3=Comand_Buf[3+1];
										    if(u8temp3 & 0x2)
												FanMode_Clear();
											else
												FanCtrl_Force((u8temp3 & 0x1));
																								  
										break;

								   case 0x0A:
										  TxBuffer[0]= 0x0A;
									    TxBuffer[1]= FanCSR_Read();
											TransLen=2;
								   		break;

									default:
											ResCode=5;
								      TxBuffer[0]= 0x03;										 
											TransLen=1;
										break;
								}
							}
							USBRes(ResCode, 0x10, TxBuffer, TransLen);
						break;					

					case  0x11: 
									   
									   
						    ResCode=0;
							TransLen=0;

							u8temp3= (Comand_Buf[3]>>4);
							if (u8temp3>=2)
								ResCode=7;	
							else
							{
								u8temp2=(Comand_Buf[3] & 0xf);
								u8temp1=Comand_Buf[2];
								
								switch(u8temp2)
								{
									case 0x9:
									        if(u8temp3 !=0)
											{
											  ResCode=7; 
											}
											else
											{
	 											if(u8temp1 < 4)
													ResCode=1;
												else
												{
													KP_temp.tempd.byte0 = Comand_Buf[3+1];
													KP_temp.tempd.byte1 = Comand_Buf[3+2];
													KP_temp.tempd.byte2 = Comand_Buf[3+3];
													KP_temp.tempd.byte3 = Comand_Buf[3+4];
													Ktm[u8temp3] = KP_temp.float_num;
												}
											}
										break;

									case 0x1:
											if(u8temp1 < 4)
												ResCode=1;
											else
											{
												KP_temp.tempd.byte0 = Comand_Buf[3+1];
												KP_temp.tempd.byte1 = Comand_Buf[3+2];
												KP_temp.tempd.byte2 = Comand_Buf[3+3];
												KP_temp.tempd.byte3 = Comand_Buf[3+4];
												Kp[u8temp3] = KP_temp.float_num;
											}
											break;
	
									case 0x2:
											if(u8temp1 < 4)
												ResCode=1;
											else
											{
												KI_temp.tempd.byte0 = Comand_Buf[3+1];
												KI_temp.tempd.byte1 = Comand_Buf[3+2];
												KI_temp.tempd.byte2 = Comand_Buf[3+3];
												KI_temp.tempd.byte3 = Comand_Buf[3+4];
												Ki[u8temp3] = KI_temp.float_num;
											}
											break;
	
	
									case 0x4:
											if(u8temp1 < 4)
												ResCode=1;
											else
											{
												KD_temp.tempd.byte0 = Comand_Buf[3+1];
												KD_temp.tempd.byte1 = Comand_Buf[3+2];
												KD_temp.tempd.byte2 = Comand_Buf[3+3];
												KD_temp.tempd.byte3 = Comand_Buf[3+4];
												Kd[u8temp3] = KD_temp.float_num;
											}
											break;
	
	
									case 0x3:
											if(u8temp1 < 8)
												ResCode=1;
											else
											{
												KP_temp.tempd.byte0 = Comand_Buf[3+1];
												KP_temp.tempd.byte1 = Comand_Buf[3+2];
												KP_temp.tempd.byte2 = Comand_Buf[3+3];
												KP_temp.tempd.byte3 = Comand_Buf[3+4];
												Kp[u8temp3] = KP_temp.float_num;
	
	 											KI_temp.tempd.byte0 = Comand_Buf[3+5];
												KI_temp.tempd.byte1 = Comand_Buf[3+6];
												KI_temp.tempd.byte2 = Comand_Buf[3+7];
												KI_temp.tempd.byte3 = Comand_Buf[3+8];
												Ki[u8temp3] = KI_temp.float_num;
	
											}
											break;
	
	
									case 0x7:
											if(u8temp1 < 12)
												ResCode=1;
											else
											{
												KP_temp.tempd.byte0 = Comand_Buf[3+1];
												KP_temp.tempd.byte1 = Comand_Buf[3+2];
												KP_temp.tempd.byte2 = Comand_Buf[3+3];
												KP_temp.tempd.byte3 = Comand_Buf[3+4];
												Kp[u8temp3] = KP_temp.float_num;
	
	 											KI_temp.tempd.byte0 = Comand_Buf[3+5];
												KI_temp.tempd.byte1 = Comand_Buf[3+6];
												KI_temp.tempd.byte2 = Comand_Buf[3+7];
												KI_temp.tempd.byte3 = Comand_Buf[3+8];
												Ki[u8temp3] = KI_temp.float_num;
	
												KD_temp.tempd.byte0 = Comand_Buf[3+9];
												KD_temp.tempd.byte1 = Comand_Buf[3+10];
												KD_temp.tempd.byte2 = Comand_Buf[3+11];
												KD_temp.tempd.byte3 = Comand_Buf[3+12];
												Kd[u8temp3] = KD_temp.float_num;
	
											}
											break;
	
									case 0x8:
												if(u8temp1 < 4)
													ResCode=1;
												else
												{
													KL_temp.tempd.byte0 = Comand_Buf[3+1];
													KL_temp.tempd.byte1 = Comand_Buf[3+2];
													KL_temp.tempd.byte2 = Comand_Buf[3+3];
													KL_temp.tempd.byte3 = Comand_Buf[3+4];
													Kl[u8temp3] = KL_temp.float_num;
												}
											break;
									default:
											ResCode=5;
										break;
								}
							}
							if(USB_ReceiveFlg==TRUE)
							{
							    TxBuffer[0]= 0x11;									
                 	TransLen=1;
							    USB_ReceiveFlg=FALSE;
							    USBRes(ResCode, 0x11, TxBuffer, TransLen);
							}
							else
							  TxBuffer[0]= 0x11;									
               	TransLen=1;
								USBRes(ResCode, 0x11, TxBuffer, TransLen);
						break; 
					
					case  0x12:
					

















 
							ResCode=0;
              TxBuffer[0] = 0x1;
							TxBuffer[1]=2;
							KP_temp.float_num=Ktm[0];
 							TxBuffer[2]=KP_temp.tempd.byte0;
							TxBuffer[3]=KP_temp.tempd.byte1;
							TxBuffer[4]=KP_temp.tempd.byte2;
							TxBuffer[5]=KP_temp.tempd.byte3;

							j=6;
							for(i=0;i<2;i++)
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
							USBRes(ResCode, 0x12, TxBuffer, TransLen);


						break;


					
											   
					case 0x13:
							TxBuffer[0]= 0x13;	
					    TransLen=1;
							u8temp2=Comand_Buf[3];
							u8temp1=Comand_Buf[2];
							ResCode=0;
							switch(u8temp2)
							{
								case 0x3:
					
									if((Comand_Buf[0x5]<1) || (Comand_Buf[0x6]<1))
									 	ResCode=7;
									else
									{
										
										u8temp=Comand_Buf[0x6];
										Buffer_Cycle_Control.TotalStage= u8temp-1;
									
										for(i=0;i<u8temp;i++)	
										{
											u8temp1=i*0x6;
											Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[0x7+u8temp1];
											Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[0x7+u8temp1+1];
											Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[0x7+u8temp1+2];
											Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[0x7+u8temp1+3];
			
											Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[0x7+u8temp1+4];
											Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[0x7+u8temp1+5];
			
										}
										if(SetPoint_Check(u8temp,Buffer_Cycle_SetPoint)!= 1)
											ResCode=7;	
										else
										{
			
											PCR_Cycle_Control.CycleValid=0;
											TempControl_2_stop();		
											PID_Clear(1);
											SetPoint_Copy(u8temp,Buffer_Cycle_SetPoint, PCR_Cycle_SetPoint);
											
											PCR_Cycle_Control.TotalStage= Buffer_Cycle_Control.TotalStage;
											PCR_Cycle_Control.CycleValid=1;
											
										}
									}
									USBRes(ResCode, 0x13, TxBuffer, TransLen);

									break;


								case 0x4: 

									ResCode=0;
							    TxBuffer[0]= 0x4;	
									TransLen=1;
									for(i=0;i<2;i++)
									{
										u8temp1=i*0x6;  
										Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[0x7+u8temp1];
										Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[0x7+u8temp1+1];
										Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[0x7+u8temp1+2];
										Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[0x7+u8temp1+3];
		
										Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[0x7+u8temp1+4];
										Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[0x7+u8temp1+5];
									}

									if(SetPoint_Check(2,Buffer_Cycle_SetPoint)!= 1)
											ResCode=7;	
									else
									{
										SetPoint_Copy(2,Buffer_Cycle_SetPoint, PE_Cycle_SetPoint);
	                       
										if(Comand_Buf[0x4]==1) 
										{
												
												if(PCR_Cycle_Control.CycleValid==1)
												{
													if(CycleSTS != READY)
													{
										 				TempCtrl_Active=TempCtrl_Active_2=0;
														TempValid =0;
														TempControl_stop();
														TempControl_2_stop();
														CycleSTS=READY;												
													}
													PCR_Cycle_Control.TotalCycle=Comand_Buf[0x5]-1;  
													Cycle_Control_Start();
												}
												else
													ResCode=8;	
															
										}
									}
									if(USB_ReceiveFlg==TRUE)
									{
										USB_ReceiveFlg=FALSE;
									 	USBRes(ResCode, 0x13, TxBuffer, TransLen);
									}
									else
										USBRes(ResCode, 0x13, TxBuffer, TransLen);
										
									break;

								default:
									break;
							 }
							
						break;

					
				   	case 0x14:   
 						  	TransLen=2;
							ResCode=0;

                            u8temp2=Comand_Buf[3];
							switch(u8temp2)
							{
								case 0x1:
									TxBuffer[0]=0x1;
		 							TxBuffer[1]=Cycle_Check();  
								    TxBuffer[2]=Stage_Check();  
									TransLen=2;
									break;

								case 0x2:
									TxBuffer[0]=0x2;
										TxBuffer[1]=TempValid;  
										if(Control_2_Dir==0x2)
										{
											u8temp=1;
											u16temp1=((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CCR1;
										}
										else
										{
											u8temp=0;
										    u16temp1=((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x2C00))->CCR1;
										}
			
										TxBuffer[2]=((TempCtrl_Active<<4) | (u8temp<<3)|(TempCtrl_Active_2)); 
										u16temp= ((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCR3;
										TxBuffer[3]= (u8)(u16temp>>8);	  
										TxBuffer[4]= (u8)u16temp;
										TxBuffer[5]= (u8)(u16temp1>>8);
										TxBuffer[6]= (u8)u16temp1;
										TransLen=7;
									break;

								default:
										ResCode=5;
										TransLen=0;
									break;
							}
						   USBRes(ResCode, 0x14, TxBuffer, TransLen);
						break;
 
























 
					default:
						break;
				
				}
			}		
		}


		tick++;
		delay_ms(10);

		if(SPI_2_Rx_sts==0x2)
		{
			for(i=0;i<test_lenth;i++)
				u8buff[i]=SPI_2_RcvBuf[i];
			SPI_2_Rx_sts=0x0;
		}
		
	if(tick==5)
		{
       ldecoun++;
			
		  	if(ldecoun==0)
					*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(10<<2))))=! *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(10<<2))));
        if(ldecoun==1)
					*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(11<<2))))=! *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(11<<2))));
        if(ldecoun==2)
					*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(12<<2))))=! *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(12<<2))));
       if(ldecoun==3)
			  *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(13<<2))))=! *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(13<<2))));
       if(ldecoun==4)
			  *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(14<<2))))=! *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(14<<2))));
       if(ldecoun==5)
			 { 
			   *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(15<<2))))=! *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(15<<2))));
				 ldecoun=0;
			 }
 		 
			tick=0;
		}
			   	   
	}
}

void Print_packet(u8 ResCode, u8 command, u8 *TxBuffer, u8 TransLen)
{
   if(USB_ReceiveFlg!=TRUE)
	   UARTRes(ResCode, 0x4, TxBuffer, TransLen);
   else 
   {
   	   USBRes(ResCode, 0x11, TxBuffer, TransLen);
	   USB_ReceiveFlg = FALSE;
	}
}


void EXTI0_IRQHandler(void)

{
		    		    				     		    
	((EXTI_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0400))->PR=1;  
	PCR_ADC_Done_Flag=1;
} 


u8 PacketChkSum(u8 *p, u8 length)
{
	u8 i;  
	u8 temp=0;
	for	(i=1;i<(length-2);i++)
		temp+=*(p+i);
	if(temp==0x17)
		temp++;
	if(temp==*(p+length-2))
		return TRUE;
	else
		return FALSE;
	
}

u8 buf[64]={0};	  
 
 
 
void UARTRes(u8 response, u8 command,u8 * buffer, u8 length)
{
	u8 i, u8temp, len;
	buf[0]=0xAA;
	buf[1]=response;
	buf[2]=command;
	buf[3]=length;
	len=4;
	for(i=0;i<length;i++,len++)
			buf[len]=*(buffer+i);
	u8temp=0;
	for(i=1;i<(len-1);i++)
		u8temp+=buf[i];
	if(u8temp==0x17)
		u8temp++;
   	buf[len]=u8temp;
	buf[len+1]=buf[len+2]=0x17;
	buf[len+3]='\0';
	{
		for(i=0;i<64;i++)
			putc(buf[i], (& __stdout));		
	}
	
}

void USBRes(u8 response, u8 command,u8 * buffer, u8 length)
{
	u16 i, u8temp, len;
	Transi_Buffer[0]=0xAA;
	Transi_Buffer[1]=response;
	Transi_Buffer[2]=command;
	Transi_Buffer[3]=length;
	len=4;
	for(i=0;i<length;i++,len++)
			Transi_Buffer[len]=*(buffer+i);
	u8temp=0;
	for(i=1;i<(len-1);i++)
		u8temp+=Transi_Buffer[i];
	if(u8temp==0x17)
		u8temp++;
   	Transi_Buffer[len]=u8temp;
	Transi_Buffer[len+1]=Transi_Buffer[len+2]=0x17;
	
    UserToPMABufferCopy(Transi_Buffer, (0x118), 64);
	SetEPTxCount(((u8)2), 64); 
    SetEPTxValid(((u8)2));
}

 
 


void Read_Row(u8 row)
{
	u8 i;

 	PCRChip_Command_Send[0]=0xC3;
	PCRChip_Command_Send[1]=0x00;
	PCRChip_Command_Send[2]=0xA1;
	PCRChip_Command_Send[3]=0xB5;
	PCRChip_Command_Send[4]=0x21;
	PCRChip_Command_Send[5]=0x61;
	


 
     
	PCRChip_Command_Send[6]=0x29; 
	PCRChip_Command_Send[7]=0x21; 
	PCRChip_Command_Send[8]=0x00; 




 

  	for(i=0;i<9;i++)
	{
		if(i==2)
		{
			while(TMR_Int_Flag==0);
			*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=1;
			OSC_Status=1;

		}
		Send_Command(0x3, row, 1, &PCRChip_Command_Send[i]);
		
		if(i==1)
		{
			TIM3_ARR_Update(PCR_Regs.InteCount);
			BaseCounter= PCR_Regs.InteTime;
			TMR_Int_Flag=0;
			((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1|=0x01; 

		}
		
	}
	delay_ms(10);
	Send_Command(0x4, 0, 0, &PCRChip_Command_Send[0]);
	PCR_ADC_Done_Flag=0;
	while(PCR_ADC_Done_Flag==0);
	if(OSC_mode != 1)
	{
		*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=0;
		OSC_Status=0;
	}	 
	Send_Command(0x5, 0, 0, &PCRChip_Command_Send[0]);
}


void ReadUpdate_Image24_slow(u8 mode, u8 row)
{
 	u8 i,j,k,u8temp,txpattern,txset;

	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=1;
	OSC_Status=1;
	delay_ms(10);

	
	           Time_LED_Delay=0;
			   TIM7_Init();
			   *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=1;
			   while(Time_LED_Delay<SetTm_LED_Delay);
			   TIM7_Stop();
	


	for(txset=0;txset<4;txset++)
	{
		txpattern =  PIXEL_24READ[txset];
		PCRChip_Command_Send[9]=PCR_Regs.SW_TxCtrl=(((txpattern & 0x0f)<<0x00)|(PCR_Regs . SW_TxCtrl & ~(0x0f)));
		Send_Command(0x2, 0x1, 1, &PCRChip_Command_Send[9]);

		for(j=0;j<12;j++)
		{
			Read_Row(j);
			ImageBuf[j][1]=j;
			for(i=0;i<((12+1)<<1);i++)
				ImageBuf[j][i+2]=SPI_2_RcvBuf[i];
		}
	
		switch(txset)
		{
			case 0:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
					}
#line 1881 "test.c"
				}	
			break;

			case 1:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
					}	
#line 1900 "test.c"
				}	
			break;

			case 2:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)+1][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)+1][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
					}
#line 1919 "test.c"
				}	
			break;

			case 3:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)+1][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)+1][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
					}
#line 1938 "test.c"
				}	
			break;
		
			default:
			break;
		}
	}



 

	
			   *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=(0x1 & led_mode);
	


	if(OSC_mode != 1)
	{
		*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=0;
		OSC_Status=0;
	}


	if(mode == 0x8)
	{			
		for(i=0;i<24;i++)
		{	 ImageBufPIX[i][0]=0x8;
			 ImageBufPIX[i][1] = i;
		}

		for(i=0;i<24;i++)
		{

			 USBRes(0, 0x2, &ImageBufPIX[i][0], 52);






			EpMsgEnable();
			UsbReadDone();
			EpMsgDisable();

		}
	 }
	 else if(mode == 0x7)
	 {
		 ImageBufPIX[row][0] =  0x7;
		 ImageBufPIX[row][1] =  row ;

		 USBRes(0, 0x2, &ImageBufPIX[row][0], 52);	

		




			EpMsgEnable();
			UsbReadDone();
			EpMsgDisable();

	 }
}



void ReadUpdate_Image24_XC(u8 mode, u8 row)	  
{
 	u8 i,j,k,u8temp,txpattern,txset;

	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=1;
	OSC_Status=1;
	delay_ms(10);

 	PCRChip_Command_Send[0]=0xC3;
	PCRChip_Command_Send[1]=0x00;
	PCRChip_Command_Send[2]=0xA1;
	PCRChip_Command_Send[3]=0xB5;
	PCRChip_Command_Send[4]=0x21;
	PCRChip_Command_Send[5]=0x61;

	PCRChip_Command_Send[6]=0x29; 
	PCRChip_Command_Send[7]=0x21; 
	PCRChip_Command_Send[8]=0x00; 
	PCRChip_Command_Send[9]=0x00; 


 	
	
			   Time_LED_Delay=0;
			   TIM7_Init();
			   *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=1;
			   
			   while(Time_LED_Delay<SetTm_LED_Delay);
			   TIM7_Stop();
	
			
	for(txset=0;txset<4;txset++)
	{
		txpattern =  PIXEL_24READ[txset];
		PCRChip_Command_Send[9]=PCR_Regs.SW_TxCtrl=(((txpattern & 0x0f)<<0x00)|(PCR_Regs . SW_TxCtrl & ~(0x0f)));
		Send_Command(0x2, 0x1, 1, &PCRChip_Command_Send[9]);


		for(j=0;j<12;j++)
	        Send_Command(0x3, j, 1, &PCRChip_Command_Send[0]);  
		for(j=0;j<12;j++)
		{
			Send_Command(0x3, j, 1, &PCRChip_Command_Send[1]);  
	        if(j==0)
			{
				TIM3_ARR_Update(PCR_Regs.InteCount);
				BaseCounter= PCR_Regs.InteTime;
				((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1|=0x01; 
				TMR_Int_Flag=0;

			}
			delay_us(600) ;	
		}
		for(j=0;j<12;j++)
		{
				 	
			while((TMR_Int_Flag & (1<<j))==0);

			for(i=2;i<9;i++)
				Send_Command(0x3, j, 1, &PCRChip_Command_Send[i]);

			Send_Command(0x4, j, 0, &PCRChip_Command_Send[0]); 
			PCR_ADC_Done_Flag=0;

			while(PCR_ADC_Done_Flag==0); 

			Send_Command(0x5, j, 0, &PCRChip_Command_Send[0]);	

			ImageBuf[j][1]=j;
			for(i=0;i<((12+1)<<1);i++)
			{
				ImageBuf[j][i+2]=SPI_2_RcvBuf[i];
			}
		
		}	
		switch(txset)
		{
			case 0:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
					}
#line 2099 "test.c"
				}	
			break;

			case 1:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
					}	
#line 2118 "test.c"
				}	
			break;

			case 2:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)+1][(i<<2)+4] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)+1][(i<<2)+5] =ImageBuf[j][(i<<1)+3];
					}
#line 2137 "test.c"
				}	
			break;

			case 3:
				for(j=0;j<12;j++)
				{

					for(i=0;i<((12+1));i++)
					{
						ImageBufPIX[(j<<1)+1][(i<<2)+2] =ImageBuf[j][(i<<1)+2];
						ImageBufPIX[(j<<1)+1][(i<<2)+3] =ImageBuf[j][(i<<1)+3];
					}
#line 2156 "test.c"
				}	
			break;
		
			default:
			break;
		}
	}



 

	
			   *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=(0x1 & led_mode);
	

	if(OSC_mode != 1)
	{
		*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=0;
		OSC_Status=0;
	}

	if(mode == 0x8)
	{			
		for(i=0;i<24;i++)
		{	 ImageBufPIX[i][0]=0x8;
			 ImageBufPIX[i][1] = i;
		}

		for(i=0;i<24;i++)
		{

			 USBRes(0, 0x2, &ImageBufPIX[i][0], 52);






			EpMsgEnable();
			UsbReadDone();
			EpMsgDisable();

		}
	 }
	 else if(mode == 0x7)
	 {
		 ImageBufPIX[row][0] =  0x7;
		 ImageBufPIX[row][1] =  row ;

		 USBRes(0, 0x2, &ImageBufPIX[row][0], 52);	

		




			EpMsgEnable();
			UsbReadDone();
			EpMsgDisable();

	 }













 

}

void ReadUpdate_Image(void)	
{
 	u8 i,j,u8temp;

	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=1;
	OSC_Status=1;
	delay_ms(10);
 	PCRChip_Command_Send[0]=0xC3;
	PCRChip_Command_Send[1]=0x00;
	PCRChip_Command_Send[2]=0xA1;
	PCRChip_Command_Send[3]=0xB5;
	PCRChip_Command_Send[4]=0x21;
	PCRChip_Command_Send[5]=0x61;
	


 
     
	PCRChip_Command_Send[6]=0x29; 
	PCRChip_Command_Send[7]=0x21; 
	PCRChip_Command_Send[8]=0x00; 




 

	
	           Time_LED_Delay=0;
			   TIM7_Init();
			   *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=1;
			   while(Time_LED_Delay<SetTm_LED_Delay);
			   TIM7_Stop();
	


	for(j=0;j<12;j++)
		Send_Command(0x3, j, 1, &PCRChip_Command_Send[0]);  
	 
	for(j=0;j<12;j++)
	{
		Send_Command(0x3, j, 1, &PCRChip_Command_Send[1]); 


		if(j==0)
		{
			TIM3_ARR_Update(PCR_Regs.InteCount);
			BaseCounter= PCR_Regs.InteTime;
			((TIM_TypeDef *) (((u32)0x40000000) + 0x0400))->CR1|=0x01; 
			TMR_Int_Flag=0;

			
		}
		delay_us(600);
	}

	for(j=0;j<12;j++)
	{
		while((TMR_Int_Flag & (1<<j))==0);








 

		for(i=2;i<9;i++)
			Send_Command(0x3, j, 1, &PCRChip_Command_Send[i]);

		Send_Command(0x4, j, 0, &PCRChip_Command_Send[0]); 
		PCR_ADC_Done_Flag=0;

		while(PCR_ADC_Done_Flag==0); 

		Send_Command(0x5, j, 0, &PCRChip_Command_Send[0]);


		ImageBuf[j][1]=j;
		for(i=0;i<((12+1)<<1);i++)
		{
			ImageBuf[j][i+2]=SPI_2_RcvBuf[i];
		}
			
	}

	
			   *((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=(0x1 & led_mode);
	

	if(OSC_mode != 1)
	{
		*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=0;
		OSC_Status=0;
	}


	if(PixReadmMode==0x2)
	{
		for(j=0;j<(12);j++)
		ImageBuf[j][0]=0x2;  
   		PixReadmMode=0x0;
	}
	else if(PixReadmMode==0x3)
	{
		for(j=0;j<(12);j++)
			ImageBuf[j][0]=0xff;
	}
	u8temp=((12+1)<<1)+2;








 
	for(i=0;i<(12);i++)
	{
		USBRes(0, 0x2,&ImageBuf[i][0],u8temp);



			EpMsgEnable();
			UsbReadDone();
			EpMsgDisable();

	 }
}

void Trim_Reset(void)
{
	PCR_Regs.RampTrim=0x88;
	PCR_Regs.RangTrim=0x08;
	PCR_Regs.Ipix_V24Trim=0x88;
	PCR_Regs.V20_V15Trim=0x88;
	PCR_Regs.SW_TxCtrl=0x08;
	PCR_Regs.TsTADC_AmuxCtrl=0x00;
	BaseCounter=PCR_Regs.InteTime=1 ;

}	

 
 
void Read_Row_Debug_2(void)
{


	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=1;
	OSC_Status=1;

	delay_ms(10);
	Send_Command(0x4, 0, 0, &PCRChip_Command_Send[0]);
	PCR_ADC_Done_Flag=0;
	while(PCR_ADC_Done_Flag==0);
	if(OSC_mode != 1)
	{
		*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(6<<2))))=0;
		OSC_Status=0;
	}	 
	Send_Command(0x5, 0, 0, &PCRChip_Command_Send[0]);
}	

void Fan_Echo(u8 dat)
{
	TxBuffer[0]= dat;

}



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
	u8 i;
	MSG_TYP msg=EpMsgPop();
	if(msg==0)
	{
	











 
	do{			delay_ms(64);
			msg=EpMsgPop();}while(msg==0);
	}
	return msg;
}




void EXTI9_5_IRQHandler(void)
{
	if(((EXTI_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0400))->PR & (0x1<<6)) 
	{
		((EXTI_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0400))->PR |= (0x1<<6);
		if(mst_flag == 0)
		{
			
			mst_flag = 1;  
		}
	}
	else





 
		((EXTI_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0400))->PR |= 0xffff;
}
