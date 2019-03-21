#include "control.h"
#include "usart6.h"	
#include "pid.h"	
#include "bsp_can.h"
#include "xnumx.h"

char p[6]={'z','x','c','v','b','n'};
char rx[100],rLen,mLen=6;
double m[6];

u16 myabs(s16 num){
	if(num){
		return num;
	}else{
		return -num;
	}
}

void run(int32_t run_spd){
	for(int i=0; i<4; i++)
			{	
				motor_pid[i].target = run_spd; 																							
				motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //根据设定值进行PID计算。
			}
			set_moto_current(&hcan1, motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
													motor_pid[1].output,
													motor_pid[2].output,
													motor_pid[3].output);
			HAL_Delay(10);      //PID控制频率100HZ
}

void run_b(int32_t power,u16 distance){
	int32_t cnt_temp = moto_chassis[0].round_cnt;
	if(power){
		while(abs(moto_chassis[0].round_cnt-cnt_temp)<distance){
			run(power);
		}
	}else{
		while(abs(moto_chassis[0].round_cnt-cnt_temp)<distance){
			run(power);
		}
	}
}


void run_func_a(u8 direct,u8 loop,int32_t power,u16 distance,u16 wait_time){
	u16 wait_time2;
	int32_t power2;
	wait_time2 = wait_time;
	
	for(u8 i=0;i<loop;i++){
		if(1==direct) {
			power2 = power;
		}else{
			power2 = -power;
		}
		run_b(power2,distance);
		while(wait_time2--){
			run(0);
		}
		wait_time2 = wait_time;
		if(1==direct) {
			power2 = -power;
		}else{
			power2 = power;
		}
		run_b(power2,distance);
		while(wait_time2--){
			run(0);
		}
		wait_time2 = wait_time;
	}
	
}


void CopeSerialData(u8 data)
{
	static u8 index = 0,rx_flag = 0;
	if(rx_flag){
		rx_flag = 0;
	}
	switch(data){
		case 'a':
			index = 0;
			break;
		case 's':
			xnumx(rx,index,p,m,mLen);
			rx_flag = 1;
			break;
		case 'q':
			usart6_Send_string((u8*)"\r\nfunction 1\r\n [z]power(-4000~4000)\r\n [x]distance(only +)\r\n");
			run_b(m[0],m[1]);
			usart6_Send_string((u8*)"function 1 done\r\n\r\n");
			break;
		case 'w':usart6_Send_string((u8*)"\r\nfunction 2\r\n [z]direct(1&2)\r\n [x]loop(only +)\r\n [c]power(0~4000)\r\n [v]distance(only +)\r\n [b]wait_time(only +,per 10ms)\r\n");
			run_func_a(m[0],m[1],m[2],m[3],m[4]);
			usart6_Send_string((u8*)"function 2 done\r\n\r\n");
			break;
		case 'e':
			
			break;
		case 'r':
			
			break;
	}
		rx[index++]=data;
	
}





