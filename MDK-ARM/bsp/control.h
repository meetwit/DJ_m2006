#ifndef __CONTROL_H
#define __CONTROL_H 
#include "sys.h"
#include "stdio.h"	  

u16 myabs(s16 num);
void run(int32_t run_spd);
void run_b(int32_t power,u16 distance);
void run_func_a(u8 direct,u8 loop,int32_t power,u16 distance,u16 wait_time);
void CopeSerialData(u8 data);
#endif	   


