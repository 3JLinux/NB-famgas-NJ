#include "fram_gas.h"
#include "string.h"
#include "crc16.h"


void fram_gas_init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA,PD端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				 //LED0-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //推挽输出
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8
	//GPIO_SetBits(GPIOA,GPIO_Pin_11);						 //PA.8 输出高
}


#define GAS_AGREEMENT_CRC_LEN	6
#define GAS_AGREEMENT_LEN		8
u8 fram_gas_read_packet(u8 *fram_gas_485_packet,u8 slave_addr,u8 cmd,u8 *star_addr,u8 *register_num)
{
	u16 crc16_num; 
	GAS_AGREEMENT *pFrame = NULL;
	if (NULL == fram_gas_485_packet)
	{
		return 0;
	}
	pFrame = (GAS_AGREEMENT*)fram_gas_485_packet;
	pFrame->GASslave_addr = slave_addr;
	pFrame->GASfunction_code = cmd;
	memcpy(pFrame->GASregister_star_addr,star_addr,2);
	memcpy(pFrame->GASregister_num,register_num,2);
	crc16_num = crc16((u8*)pFrame,GAS_AGREEMENT_CRC_LEN);
	pFrame->GAScrc[1] = crc16_num>>8 & 0xFF;
	pFrame->GAScrc[0] = crc16_num & 0xFF;
	//if(strlen((const char*)fram_gas_485_packet) >= GAS_AGREEMENT_LEN)
	{
		memcpy(fram_gas_485_packet,pFrame,GAS_AGREEMENT_LEN);
		return GAS_AGREEMENT_LEN;
	}
	return 0;
}

