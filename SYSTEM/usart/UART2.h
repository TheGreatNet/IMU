#ifndef __UART2_H
#define __UART2_H
//#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART2_RX      1
void Initial_UART2(unsigned long baudrate);
void UART2_Put_Char(unsigned char DataToSend);
void UART2_Put_String(unsigned char *Str);

extern u8  USART2_RX_BUF[USART_REC_LEN];
extern u16 USART2_RX_STA;
#endif

//------------------End of File----------------------------

