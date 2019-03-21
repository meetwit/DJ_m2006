#include "sys.h"
#include "usart6.h"	
#include "control.h"	  

//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void uart6_init(u32 pclk2,u32 bound)		//uart_init(84,921600);		//��ʼ�����ڲ�����Ϊ115200  
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<6;   	//ʹ��PORTG��ʱ��  
	RCC->APB2ENR|=1<<5;  	//ʹ�ܴ���6ʱ�� 
	GPIO_Set(GPIOG,PIN9|PIN14,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PG9,PG14,���ù���,�������
 	GPIO_AF_Set(GPIOG,9,8);	//PG9,AF8
	GPIO_AF_Set(GPIOG,14,8);//PG14,AF8  	   
	//����������
 	USART6->BRR=mantissa; 	//����������	 
	USART6->CR1&=~(1<<15); 	//����OVER8=0 
	USART6->CR1|=1<<3;  	//���ڷ���ʹ�� 
#if 1		  	//���ʹ���˽���
	//ʹ�ܽ����ж� 
	USART6->CR1|=1<<2;  	//���ڽ���ʹ��
	USART6->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(3,3,USART6_IRQn,2);//��2��������ȼ� 
#endif
	USART6->CR1|=1<<13;  	//����ʹ��
}


void usart6_Send_byte(u8 data)
{
		while((USART6->SR&0X40)==0);//�ȴ����ͽ���		  
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
	if(USART6->SR&(0x1<<5))	//���յ�����
	{ 
		tempdata=USART6->DR;
		CopeSerialData(tempdata);
	}		
}









