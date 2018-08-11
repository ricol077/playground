/* task priority */ 
#define STARTUP_TASK_PRIO                     	9
#define LED1_TASK_PRIO                        	6
#define CAN1_TASK_PRIO                         	5
#define CMD_TASK_PRIO                         	7
#define USB_TASK_PRIO                         	8
#define CAN2_TASK_PRIO                         	4

/* task stack size */ 
#define STARTUP_TASK_STK_SIZE                  80  
#define LED1_TASK_STK_SIZE                     80
#define CAN1_TASK_STK_SIZE                     512
#define CAN2_TASK_STK_SIZE                     512
#define CMD_TASK_STK_SIZE                      512
#define USB_TASK_STK_SIZE                      512


#define CAN_MAX_LEN														64
#define MAX_CAN_DAT_PER_PAK												8
//extern u8 Buf_CAN1_DAT[CAN_MAX_LEN];
//extern TxMsg1
