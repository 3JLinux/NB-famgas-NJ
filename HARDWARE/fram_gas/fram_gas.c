#include "fram_gas.h"


void fram_gas_init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA,PD�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				 //LED0-->PA.8 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //�������
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
	//GPIO_SetBits(GPIOA,GPIO_Pin_11);						 //PA.8 �����
}




