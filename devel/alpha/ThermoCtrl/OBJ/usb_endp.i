#line 1 "..\\USB\\CONFIG\\usb_endp.c"













 

 
#line 1 "..\\USER\\usb_lib.h"













 

 



 
#line 1 "..\\USB\\LIB\\usb_type.h"













 

 



 
#line 1 "..\\USB\\CONFIG\\usb_conf.h"













 

 



 
 
 
 
 
 
 
 
 
 


 
 
 
 
 


 
 



 
 







 
 
 
 
 
 



 
 









#line 79 "..\\USB\\CONFIG\\usb_conf.h"



 

#line 22 "..\\USB\\LIB\\usb_type.h"

 
 






typedef signed long      s32;
typedef signed short     s16;
typedef signed char      s8;

typedef volatile signed long      vs32;
typedef volatile signed short     vs16;
typedef volatile signed char      vs8;

typedef unsigned long       u32;
typedef unsigned short      u16;
typedef unsigned char       u8;

typedef unsigned long  const    uc32;   
typedef unsigned short const    uc16;   
typedef unsigned char  const    uc8;    

typedef volatile unsigned long      vu32;
typedef volatile unsigned short     vu16;
typedef volatile unsigned char      vu8;

typedef volatile unsigned long  const    vuc32;   
typedef volatile unsigned short const    vuc16;   
typedef volatile unsigned char  const    vuc8;    


typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;

typedef enum { RESET = 0, SET   = !RESET } FlagStatus, ITStatus;

typedef enum { DISABLE = 0, ENABLE  = !DISABLE} FunctionalState;

typedef enum { ERROR = 0, SUCCESS  = !ERROR} ErrorStatus;


 
 
 



 
#line 22 "..\\USER\\usb_lib.h"
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



 
#line 23 "..\\USER\\usb_lib.h"
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

 
 







 
 



 
#line 24 "..\\USER\\usb_lib.h"
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



 
#line 25 "..\\USER\\usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_init.h"













 

 



 
 
 
 
 
void USB_Init(void);

 
 
extern u8	EPindex;
 
 
 
 
extern DEVICE_INFO*	pInformation;
 
 
extern DEVICE_PROP*	pProperty;
 
 
 
 
extern USER_STANDARD_REQUESTS *pUser_Standard_Requests;

extern u16	SaveState ;
extern u16 wInterrupt_Mask;



 
#line 26 "..\\USER\\usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_mem.h"













 

 



 
 
 
 
 
void UserToPMABufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);
void PMAToUserBufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);

 



 
#line 27 "..\\USER\\usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_int.h"














 

 



 
 
 
 
 
void CTR_LP(void);
void CTR_HP(void);

 



 
#line 28 "..\\USER\\usb_lib.h"

 
 
 
 
 



 
#line 18 "..\\USB\\CONFIG\\usb_endp.c"
#line 1 "..\\USB\\CONFIG\\usb_istr.h"














 

 


 
#line 22 "..\\USB\\CONFIG\\usb_istr.h"
 
 
 
 
void USB_Istr(void);
 
#line 52 "..\\USB\\CONFIG\\usb_istr.h"

void NOP_Process(void);
void EP2_IN_Callback(void);
void NOP_Process(void);
void NOP_Process(void);
void NOP_Process(void);
void NOP_Process(void);
void NOP_Process(void);

void EP1_OUT_Callback(void);
void NOP_Process(void);
void NOP_Process(void);
void NOP_Process(void);
void NOP_Process(void);
void NOP_Process(void);
void NOP_Process(void);



 
#line 19 "..\\USB\\CONFIG\\usb_endp.c"
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








	
#line 20 "..\\USB\\CONFIG\\usb_endp.c"
 
 
 
 
u8 Receive_Buffer[64];
u8 Transi_Buffer[64];
u8 USB_ReceiveFlg = FALSE;
u16 USB_RxIdx=0;
 
 






 
void EP1_OUT_Callback(void)
{
	u16 i,j;
    
	i=((u16)(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+((u8)1)*8+6)*2 + (0x40006000L)))) & 0x3ff);
	i &= 0x1ff;
	PMAToUserBufferCopy(Receive_Buffer, (0x98),i);

	USB_ReceiveFlg = FALSE;
	USB_RxIdx=0;

	if(Receive_Buffer[0]==0xAA)
	{

		{
			
			 for(j=0;j<i;j++)
			 {
			 	if((Receive_Buffer[j]==0x17) && (Receive_Buffer[j+1]==0x17))
					break;
			 }
			 for(i=0;i<=(j+1);i++)
			 	RxBuffer[i]=Receive_Buffer[i];
			
			
		 	USB_ReceiveFlg=TRUE;
			USB_RxIdx=i-1;
		}	
	}

	SetEPRxStatus(((u8)1), (0x3000));
}

void EP2_IN_Callback(void)
{

	EpMsgStk(1);

}

 

