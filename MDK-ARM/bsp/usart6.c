#include "sys.h"
#include "usart6.h"	
#include "control.h"	  

//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart6_init(u32 pclk2,u32 bound)		//uart_init(84,921600);		//初始化串口波特率为115200  
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV@OVER8=0
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<6;   	//使能PORTG口时钟  
	RCC->APB2ENR|=1<<5;  	//使能串口6时钟 
	GPIO_Set(GPIOG,PIN9|PIN14,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PG9,PG14,复用功能,上拉输出
 	GPIO_AF_Set(GPIOG,9,8);	//PG9,AF8
	GPIO_AF_Set(GPIOG,14,8);//PG14,AF8  	   
	//波特率设置
 	USART6->BRR=mantissa; 	//波特率设置	 
	USART6->CR1&=~(1<<15); 	//设置OVER8=0 
	USART6->CR1|=1<<3;  	//串口发送使能 
#if 1		  	//如果使能了接收
	//使能接收中断 
	USART6->CR1|=1<<2;  	//串口接收使能
	USART6->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART6_IRQn,2);//组2，最低优先级 
#endif
	USART6->CR1|=1<<13;  	//串口使能
}


void usart6_Send_byte(u8 data)
{
		while((USART6->SR&0X40)==0);//等待发送结束		  
		USART6->DR=data;
}


void usart6_Send_string(u8 *str)
{
		while(*str !='\0'){
			usart6_Send_byte(*(str++));
		}
}


void USART6_IRQHandler(void)
{
	u8 tempdata;
	if(USART6->SR&(0x1<<5))	//接收到数据
	{ 
		tempdata=USART6->DR;
		CopeSerialData(tempdata);
	}		
}









