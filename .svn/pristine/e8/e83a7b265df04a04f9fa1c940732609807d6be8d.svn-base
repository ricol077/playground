function code 06, integration independent on every channel.
============================================
v1.6 @2016-04-23
1, use independent LED for every channle, PF5, PD7,PD14,PD15
2, extend LED_SWITCH command, 0b'y000 xxxx, y=0 is share mode , comply with current design to  LSB as PF5 for all channle
   y=1, is independent mode, will control every channle by LSB 4bit for 4 channles
============================================
v1.5 @2016-04-21
1,add LED control when read image manually
============
V1.4 @2016-04-10
1, fix 24x24 normal read, the root cause is TMR_Int_Flag is not cleared after 24x24 Read_Image_to_buffer() called by ReadUpdate_Image_multiPCR_oneRead_mode_1()

======================================
V1.1 @2016-04-03
1, modify the to if((u8temp1>=MAX_PCR_CH) && (u8temp1 != 0xF)), fix to let pre-read type 0xFx be accepted correctly
2, change function code to 0x5, to make it diffrent to with currect 4xPCR FW which has been released with shutter enable.
3, change FW version to v1.1 as new fucntion code
4, sync shutter enable command, but default is enabled to comply with current Zhimin's software as it dosen't incude shutter feature yet.
======================================
v1.3 @2016-02-27
12x12 image multi-channel overlap implementation.
add new command GET / multi-channel read type. 


this version fix the image mode packet lenght to 35 from 35
this version is to debug the buffer, SPI buffer is not copied, UART copy data from SPI_RevBuf[] instead of RowData[]