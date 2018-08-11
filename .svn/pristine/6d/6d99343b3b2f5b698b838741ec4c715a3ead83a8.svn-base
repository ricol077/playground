#line 1 "..\\USB\\LIB\\usb_core.c"













 

 
#line 1 "..\\USB\\LIB\\usb_lib.h"













 

 



 
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


 
 
 



 
#line 22 "..\\USB\\LIB\\usb_lib.h"
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



 
#line 23 "..\\USB\\LIB\\usb_lib.h"
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

 
 







 
 



 
#line 24 "..\\USB\\LIB\\usb_lib.h"
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



 
#line 25 "..\\USB\\LIB\\usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_init.h"













 

 



 
 
 
 
 
void USB_Init(void);

 
 
extern u8	EPindex;
 
 
 
 
extern DEVICE_INFO*	pInformation;
 
 
extern DEVICE_PROP*	pProperty;
 
 
 
 
extern USER_STANDARD_REQUESTS *pUser_Standard_Requests;

extern u16	SaveState ;
extern u16 wInterrupt_Mask;



 
#line 26 "..\\USB\\LIB\\usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_mem.h"













 

 



 
 
 
 
 
void UserToPMABufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);
void PMAToUserBufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);

 



 
#line 27 "..\\USB\\LIB\\usb_lib.h"
#line 1 "..\\USB\\LIB\\usb_int.h"














 

 



 
 
 
 
 
void CTR_LP(void);
void CTR_HP(void);

 



 
#line 28 "..\\USB\\LIB\\usb_lib.h"

 
 
 
 
 



 
#line 18 "..\\USB\\LIB\\usb_core.c"
 
 

















 
 
u16_u8 StatusInfo;
bool Data_Mul_MaxPacketSize = FALSE;
 
static void DataStageOut(void);
static void DataStageIn(void);
static void NoData_Setup0(void);
static void Data_Setup0(void);
 








 
u8 *Standard_GetConfiguration(u16 Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength =
      sizeof(pInformation->Current_Configuration);
    return 0;
  }
  pUser_Standard_Requests->User_GetConfiguration();
  return (u8 *)&pInformation->Current_Configuration;
}









 
RESULT Standard_SetConfiguration(void)
{

  if ((pInformation->USBwValues . bw . bb0 <=
      Device_Table.Total_Configuration) && (pInformation->USBwValues . bw . bb1 == 0)
      && (pInformation->USBwIndexs . w == 0))  
  {
    pInformation->Current_Configuration = pInformation->USBwValues . bw . bb0;
    pUser_Standard_Requests->User_SetConfiguration();
    return USB_SUCCESS;
  }
  else
  {
    return USB_UNSUPPORT;
  }
}








 
u8 *Standard_GetInterface(u16 Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength =
      sizeof(pInformation->Current_AlternateSetting);
    return 0;
  }
  pUser_Standard_Requests->User_GetInterface();
  return (u8 *)&pInformation->Current_AlternateSetting;
}









 
RESULT Standard_SetInterface(void)
{
  RESULT Re;
  
 
  Re = (*pProperty->Class_Get_Interface_Setting)(pInformation->USBwIndexs . bw . bb0, pInformation->USBwValues . bw . bb0);

  if (pInformation->Current_Configuration != 0)
  {
    if ((Re != USB_SUCCESS) || (pInformation->USBwIndexs . bw . bb1 != 0)
        || (pInformation->USBwValues . bw . bb1 != 0))
    {
      return  USB_UNSUPPORT;
    }
    else if (Re == USB_SUCCESS)
    {
      pUser_Standard_Requests->User_SetInterface();
      pInformation->Current_Interface = pInformation->USBwIndexs . bw . bb0;
      pInformation->Current_AlternateSetting = pInformation->USBwValues . bw . bb0;
      return USB_SUCCESS;
    }

  }

  return USB_UNSUPPORT;
}








 
u8 *Standard_GetStatus(u16 Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = 2;
    return 0;
  }

  StatusInfo.w = 0;
   

  if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | DEVICE_RECIPIENT))
  {
     
    u8 Feature = pInformation->Current_Feature;

     
    if ((Feature & (1 << 5)))
    {
      (StatusInfo . bw . bb1 |= (1 << 1));
    }

     
    if ((Feature & (1 << 6)))
    {
      (StatusInfo . bw . bb1 &= ((1 << 0) ^ 255));
    }
    else  
    {
      (StatusInfo . bw . bb1 |= (1 << 0));
    }
  }
   
  else if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | INTERFACE_RECIPIENT))
  {
    return (u8 *)&StatusInfo;
  }
   
  else if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | ENDPOINT_RECIPIENT))
  {
    u8 Related_Endpoint;
    u8 wIndex0 = pInformation->USBwIndexs . bw . bb0;

    Related_Endpoint = (wIndex0 & 0x0f);
    if ((wIndex0 & (1 << 7)))
    {
       
      if ((((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x0030)) == (0x0010)))
      {
        (StatusInfo . bw . bb1 |= (1 << 0));  
      }
    }
    else
    {
       
      if ((((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x3000)) == (0x1000)))
      {
        (StatusInfo . bw . bb1 |= (1 << 0));  
      }
    }

  }
  else
  {
    return ((void *)0);
  }
  pUser_Standard_Requests->User_GetStatus();
  return (u8 *)&StatusInfo;
}








 
RESULT Standard_ClearFeature(void)
{
  u32     Type_Rec = (pInformation->USBbmRequestType & (0x60 | 0x1F));
  u32     Status;


  if (Type_Rec == (0x00 | DEVICE_RECIPIENT))
  { 
    (pInformation->Current_Feature &= ((1 << 5) ^ 255));
    return USB_SUCCESS;
  }
  else if (Type_Rec == (0x00 | ENDPOINT_RECIPIENT))
  { 
    DEVICE* pDev;
    u32 Related_Endpoint;
    u32 wIndex0;
    u32 rEP;

    if ((pInformation->USBwValues . w != ENDPOINT_STALL)
        || (pInformation->USBwIndexs . bw . bb1 != 0))
    {
      return USB_UNSUPPORT;
    }

    pDev = &Device_Table;
    wIndex0 = pInformation->USBwIndexs . bw . bb0;
    rEP = wIndex0 & ~0x80;
    Related_Endpoint = ((u8)0) + rEP;

    if ((pInformation->USBwIndexs . bw . bb0 & (1 << 7)))
    {
      
 
      Status = ((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x0030));
    }
    else
    {
      Status = ((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x3000));
    }

    if ((rEP >= pDev->Total_Endpoint) || (Status == 0)
        || (pInformation->Current_Configuration == 0))
    {
      return USB_UNSUPPORT;
    }


    if (wIndex0 & 0x80)
    {
       
      if ((((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x0030)) == (0x0010)))
      {
        ClearDTOG_TX(Related_Endpoint);
        SetEPTxStatus(Related_Endpoint, (0x0030));
      }
    }
    else
    {
       
      if ((((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x3000)) == (0x1000)))
      {
        if (Related_Endpoint == ((u8)0))
        {
           
          SetEPRxCount(Related_Endpoint, Device_Property.MaxPacketSize);
          { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & ((0x3000)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x1000) & (0x3000))!= 0) _wRegVal ^= (0x1000); if(((0x2000) & (0x3000))!= 0) _wRegVal ^= (0x2000); (*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint)= (u16)_wRegVal); };
        }
        else
        {
          ClearDTOG_RX(Related_Endpoint);
          { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & ((0x3000)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x1000) & (0x3000))!= 0) _wRegVal ^= (0x1000); if(((0x2000) & (0x3000))!= 0) _wRegVal ^= (0x2000); (*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint)= (u16)_wRegVal); };
        }
      }
    }
    pUser_Standard_Requests->User_ClearFeature();
    return USB_SUCCESS;
  }

  return USB_UNSUPPORT;
}








 
RESULT Standard_SetEndPointFeature(void)
{
  u32    wIndex0;
  u32    Related_Endpoint;
  u32    rEP;
  u32   Status;

  wIndex0 = pInformation->USBwIndexs . bw . bb0;
  rEP = wIndex0 & ~0x80;
  Related_Endpoint = ((u8)0) + rEP;

  if ((pInformation->USBwIndexs . bw . bb0 & (1 << 7)))
  {
    
 
    Status = ((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x0030));
  }
  else
  {
    Status = ((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x3000));
  }

  if (Related_Endpoint >= Device_Table.Total_Endpoint
      || pInformation->USBwValues . w != 0 || Status == 0
      || pInformation->Current_Configuration == 0)
  {
    return USB_UNSUPPORT;
  }
  else
  {
    if (wIndex0 & 0x80)
    {
       
      { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & ((0x0030)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x0010) & (0x0010))!= 0) _wRegVal ^= (0x0010); if(((0x0020) & (0x0010))!= 0) _wRegVal ^= (0x0020); (*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint)= (u16)_wRegVal); };
    }

    else
    {
       
      { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & ((0x3000)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x1000) & (0x1000))!= 0) _wRegVal ^= (0x1000); if(((0x2000) & (0x1000))!= 0) _wRegVal ^= (0x2000); (*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint)= (u16)_wRegVal); };
    }
  }
  pUser_Standard_Requests->User_SetEndPointFeature();
  return USB_SUCCESS;
}








 
RESULT Standard_SetDeviceFeature(void)
{
  (pInformation->Current_Feature |= (1 << 5));
  pUser_Standard_Requests->User_SetDeviceFeature();
  return USB_SUCCESS;
}



















 
u8 *Standard_GetDescriptorData(u16 Length, ONE_DESCRIPTOR *pDesc)
{
  u32  wOffset;

  wOffset = pInformation->Ctrl_Info.Usb_wOffset;
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = pDesc->Descriptor_Size - wOffset;
    return 0;
  }

  return pDesc->Descriptor + wOffset;
}







 
void DataStageOut(void)
{
  ENDPOINT_INFO *pEPinfo = &pInformation->Ctrl_Info;
  u32 save_rLength;

  save_rLength = pEPinfo->Usb_wLength;

  if (pEPinfo->CopyData && save_rLength)
  {
    u8 *Buffer;
    u32 Length;

    Length = pEPinfo->PacketSize;
    if (Length > save_rLength)
    {
      Length = save_rLength;
    }

    Buffer = (*pEPinfo->CopyData)(Length);
    pEPinfo->Usb_wLength -= Length;
    pEPinfo->Usb_wOffset += Length;

    PMAToUserBufferCopy(Buffer, GetEPRxAddr(((u8)0)), Length);
  }

  if (pEPinfo->Usb_wLength != 0)
  {
    (SaveRState = (0x3000)); 
    SetEPTxCount(((u8)0), 0);
    (SaveTState = (0x0030)); 
  }
   
  if (pEPinfo->Usb_wLength >= pEPinfo->PacketSize)
  {
    pInformation->ControlState = OUT_DATA;
  }
  else
  {
    if (pEPinfo->Usb_wLength > 0)
    {
      pInformation->ControlState = LAST_OUT_DATA;
    }
    else if (pEPinfo->Usb_wLength == 0)
    {
      pInformation->ControlState = WAIT_STATUS_IN;
      { (*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+((u8)0)*8+2)*2 + (0x40006000L))) = 0); (SaveTState = (0x0030)); };
    }
  }
}







 
void DataStageIn(void)
{
  ENDPOINT_INFO *pEPinfo = &pInformation->Ctrl_Info;
  u32 save_wLength = pEPinfo->Usb_wLength;
  u32 ControlState = pInformation->ControlState;

  u8 *DataBuffer;
  u32 Length;

  if ((save_wLength == 0) && (ControlState == LAST_IN_DATA))
  {
    if(Data_Mul_MaxPacketSize == TRUE)
    {
       
      { (*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+((u8)0)*8+2)*2 + (0x40006000L))) = 0); (SaveTState = (0x0030)); };
      ControlState = LAST_IN_DATA;
      Data_Mul_MaxPacketSize = FALSE;
    }
    else 
    {
       
      ControlState = WAIT_STATUS_OUT;
      (SaveTState = (0x0010));
    }
    
    goto Expect_Status_Out;
  }

  Length = pEPinfo->PacketSize;
  ControlState = (save_wLength <= Length) ? LAST_IN_DATA : IN_DATA;

  if (Length > save_wLength)
  {
    Length = save_wLength;
  }

  DataBuffer = (*pEPinfo->CopyData)(Length);

  UserToPMABufferCopy(DataBuffer, GetEPTxAddr(((u8)0)), Length);

  SetEPTxCount(((u8)0), Length);

  pEPinfo->Usb_wLength -= Length;
  pEPinfo->Usb_wOffset += Length;
  (SaveTState = (0x0030));

  (SaveRState = (0x3000)); 

Expect_Status_Out:
  pInformation->ControlState = ControlState;
}







 
void NoData_Setup0(void)
{
  RESULT Result = USB_UNSUPPORT;
  u32 RequestNo = pInformation->USBbRequest;
  u32 ControlState;

  if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | DEVICE_RECIPIENT))
  {
     
     
    if (RequestNo == SET_CONFIGURATION)
    {
      Result = Standard_SetConfiguration();
    }

     
    else if (RequestNo == SET_ADDRESS)
    {
      if ((pInformation->USBwValues . bw . bb0 > 127) || (pInformation->USBwValues . bw . bb1 != 0)
          || (pInformation->USBwIndexs . w != 0)
          || (pInformation->Current_Configuration != 0))
         
      {
        ControlState = STALLED;
        goto exit_NoData_Setup0;
      }
      else
      {
        Result = USB_SUCCESS;
      }
    }
     
    else if (RequestNo == SET_FEATURE)
    {
      if ((pInformation->USBwValues . bw . bb0 == DEVICE_REMOTE_WAKEUP)
          && (pInformation->USBwIndexs . w == 0)
          && ((pInformation->Current_Feature & (1 << 5))))
      {
        Result = Standard_SetDeviceFeature();
      }
      else
      {
        Result = USB_UNSUPPORT;
      }
    }
     
    else if (RequestNo == CLEAR_FEATURE)
    {
      if (pInformation->USBwValues . bw . bb0 == DEVICE_REMOTE_WAKEUP
          && pInformation->USBwIndexs . w == 0
          && (pInformation->Current_Feature & (1 << 5)))
      {
        Result = Standard_ClearFeature();
      }
      else
      {
        Result = USB_UNSUPPORT;
      }
    }

  }

   
  else if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | INTERFACE_RECIPIENT))
  {
     
    if (RequestNo == SET_INTERFACE)
    {
      Result = Standard_SetInterface();
    }
  }

   
  else if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | ENDPOINT_RECIPIENT))
  {
     
    if (RequestNo == CLEAR_FEATURE)
    {
      Result = Standard_ClearFeature();
    }
     
    else if (RequestNo == SET_FEATURE)
    {
      Result = Standard_SetEndPointFeature();
    }
  }
  else
  {
    Result = USB_UNSUPPORT;
  }


  if (Result != USB_SUCCESS)
  {
    Result = (*pProperty->Class_NoData_Setup)(RequestNo);
    if (Result == USB_NOT_READY)
    {
      ControlState = PAUSE;
      goto exit_NoData_Setup0;
    }
  }

  if (Result != USB_SUCCESS)
  {
    ControlState = STALLED;
    goto exit_NoData_Setup0;
  }

  ControlState = WAIT_STATUS_IN; 

  { (*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+((u8)0)*8+2)*2 + (0x40006000L))) = 0); (SaveTState = (0x0030)); };

exit_NoData_Setup0:
  pInformation->ControlState = ControlState;
  return;
}







 
void Data_Setup0(void)
{
  u8 *(*CopyRoutine)(u16);
  RESULT Result;
  u32 Request_No = pInformation->USBbRequest;

  u32 Related_Endpoint, Reserved;
  u32 wOffset, Status;



  CopyRoutine = ((void *)0);
  wOffset = 0;

  if (Request_No == GET_DESCRIPTOR)
  {
    if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | DEVICE_RECIPIENT))
    {
      u8 wValue1 = pInformation->USBwValues . bw . bb1;
      if (wValue1 == DEVICE_DESCRIPTOR)
      {
        CopyRoutine = pProperty->GetDeviceDescriptor;
      }
      else if (wValue1 == CONFIG_DESCRIPTOR)
      {
        CopyRoutine = pProperty->GetConfigDescriptor;
      }
      else if (wValue1 == STRING_DESCRIPTOR)
      {
        CopyRoutine = pProperty->GetStringDescriptor;
      }   
    }
  }

   
  else if ((Request_No == GET_STATUS) && (pInformation->USBwValues . w == 0)
           && (pInformation->USBwLengths . w == 0x0002)
           && (pInformation->USBwIndexs . bw . bb1 == 0))
  {
     
    if (((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | DEVICE_RECIPIENT))
        && (pInformation->USBwIndexs . w == 0))
    {
      CopyRoutine = Standard_GetStatus;
    }

     
    else if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | INTERFACE_RECIPIENT))
    {
      if (((*pProperty->Class_Get_Interface_Setting)(pInformation->USBwIndexs . bw . bb0, 0) == USB_SUCCESS)
          && (pInformation->Current_Configuration != 0))
      {
        CopyRoutine = Standard_GetStatus;
      }
    }

     
    else if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | ENDPOINT_RECIPIENT))
    {
      Related_Endpoint = (pInformation->USBwIndexs . bw . bb0 & 0x0f);
      Reserved = pInformation->USBwIndexs . bw . bb0 & 0x70;

      if ((pInformation->USBwIndexs . bw . bb0 & (1 << 7)))
      {
        
 
        Status = ((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x0030));
      }
      else
      {
        Status = ((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + Related_Endpoint))) & (0x3000));
      }

      if ((Related_Endpoint < Device_Table.Total_Endpoint) && (Reserved == 0)
          && (Status != 0))
      {
        CopyRoutine = Standard_GetStatus;
      }
    }

  }

   
  else if (Request_No == GET_CONFIGURATION)
  {
    if ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | DEVICE_RECIPIENT))
    {
      CopyRoutine = Standard_GetConfiguration;
    }
  }
   
  else if (Request_No == GET_INTERFACE)
  {
    if (((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | INTERFACE_RECIPIENT))
        && (pInformation->Current_Configuration != 0) && (pInformation->USBwValues . w == 0)
        && (pInformation->USBwIndexs . bw . bb1 == 0) && (pInformation->USBwLengths . w == 0x0001)
        && ((*pProperty->Class_Get_Interface_Setting)(pInformation->USBwIndexs . bw . bb0, 0) == USB_SUCCESS))
    {
      CopyRoutine = Standard_GetInterface;
    }

  }
  
  if (CopyRoutine)
  {
    pInformation->Ctrl_Info.Usb_wOffset = wOffset;
    pInformation->Ctrl_Info.CopyData = CopyRoutine;
     
     
    (*CopyRoutine)(0);
    Result = USB_SUCCESS;
  }
  else
  {
    Result = (*pProperty->Class_Data_Setup)(pInformation->USBbRequest);
    if (Result == USB_NOT_READY)
    {
      pInformation->ControlState = PAUSE;
      return;
    }
  }

  if (pInformation->Ctrl_Info.Usb_wLength == 0xFFFF)
  {
     
    pInformation->ControlState = PAUSE;
    return;
  }
  if ((Result == USB_UNSUPPORT) || (pInformation->Ctrl_Info.Usb_wLength == 0))
  {
     
    pInformation->ControlState = STALLED;
    return;
  }


  if ((pInformation->USBbmRequestType & (1 << 7)))
  {
     
    vu32 wLength = pInformation->USBwLengths . w;
     
     
    if (pInformation->Ctrl_Info.Usb_wLength > wLength)
    {
      pInformation->Ctrl_Info.Usb_wLength = wLength;
    }
    
    else if (pInformation->Ctrl_Info.Usb_wLength < pInformation->USBwLengths . w)
    {
      if (pInformation->Ctrl_Info.Usb_wLength < pProperty->MaxPacketSize)
      {
        Data_Mul_MaxPacketSize = FALSE;
      }
      else if ((pInformation->Ctrl_Info.Usb_wLength % pProperty->MaxPacketSize) == 0)
      {
        Data_Mul_MaxPacketSize = TRUE;
      }
    }   

    pInformation->Ctrl_Info.PacketSize = pProperty->MaxPacketSize;
    DataStageIn();
  }
  else
  {
    pInformation->ControlState = OUT_DATA;
    (SaveRState = (0x3000));  
  }

  return;
}







 
u8 Setup0_Process(void)
{

  union
  {
    u8* b;
    u16* w;
  } pBuf;

  pBuf.b = (0x40006000L) + (u8 *)(((u16)*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+((u8)0)*8+4)*2 + (0x40006000L)))) * 2);  

  if (pInformation->ControlState != PAUSE)
  {
    pInformation->USBbmRequestType = *pBuf.b++;  
    pInformation->USBbRequest = *pBuf.b++;  
    pBuf.w++;   
    pInformation->USBwValues . w = ByteSwap(*pBuf.w++);  
    pBuf.w++;   
    pInformation->USBwIndexs . w  = ByteSwap(*pBuf.w++);  
    pBuf.w++;   
    pInformation->USBwLengths . w = *pBuf.w;  
  }

  pInformation->ControlState = SETTING_UP;
  if (pInformation->USBwLengths . w == 0)
  {
     
    NoData_Setup0();
  }
  else
  {
     
    Data_Setup0();
  }
  return Post0_Process();
}







 
u8 In0_Process(void)
{
  u32 ControlState = pInformation->ControlState;

  if ((ControlState == IN_DATA) || (ControlState == LAST_IN_DATA))
  {
    DataStageIn();
     
    ControlState = pInformation->ControlState;
  }

  else if (ControlState == WAIT_STATUS_IN)
  {
    if ((pInformation->USBbRequest == SET_ADDRESS) &&
        ((pInformation->USBbmRequestType & (0x60 | 0x1F)) == (0x00 | DEVICE_RECIPIENT)))
    {
      SetDeviceAddress(pInformation->USBwValues . bw . bb0);
      pUser_Standard_Requests->User_SetDeviceAddress();
    }
    (*pProperty->Process_Status_IN)();
    ControlState = STALLED;
  }

  else
  {
    ControlState = STALLED;
  }

  pInformation->ControlState = ControlState;

  return Post0_Process();
}







 
u8 Out0_Process(void)
{
  u32 ControlState = pInformation->ControlState;

  if ((ControlState == OUT_DATA) || (ControlState == LAST_OUT_DATA))
  {
    DataStageOut();
    ControlState = pInformation->ControlState;  
  }

  else if (ControlState == WAIT_STATUS_OUT)
  {
    (*pProperty->Process_Status_OUT)();
    ControlState = STALLED;
  }

  else if ((ControlState == IN_DATA) || (ControlState == LAST_IN_DATA))
  {
     
    ControlState = STALLED;
  }

   
  else
  {
    ControlState = STALLED;
  }

  pInformation->ControlState = ControlState;

  return Post0_Process();
}








 
u8 Post0_Process(void)
{
  SetEPRxCount(((u8)0), Device_Property.MaxPacketSize);

  if (pInformation->ControlState == STALLED)
  {
    (SaveRState = (0x1000));
    (SaveTState = (0x0010));
  }

  return (pInformation->ControlState == PAUSE);
}







 
void SetDeviceAddress(u8 Val)
{
  u32 i;
  u32 nEP = Device_Table.Total_Endpoint;

   
  for (i = 0; i < nEP; i++)
  {
    (*(((volatile unsigned *)((0x40005C00L))) + (u8)i)= (u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + (u8)i))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F)) | (u8)i);
  }  
  (*((volatile unsigned *)((0x40005C00L) + 0x4C)) = (u16)Val | (0x80));  
}







 
void NOP_Process(void)
{
}

 
