#ifndef __USART6_H
#define __USART6_H 
#include "sys.h"
#include "stdio.h"	  

void uart6_init(u32 pclk2,u32 bound);
void usart6_Send_byte(u8 data);
void usart6_Send_string(u8 *str);

#endif	   
















