#include "enc28j60.h"

#define ENC28J60_CS		PAout(11)
#define ENC28J60_INT	PAin(12)
#define ENC28J60_WOL	PAin(13)


#define EIE			0x1B
#define EIR			0x1C
#define ESTAT		0x1D
#define ECON2		0x1E
#define ECON1		0x1F

//bank 0
#define ERDPTL		0x00
#define ERDPTH		0x01
#define EWRPTL		0x02
#define EWRPTH		0x03
#define ETXSTL		0x04
#define ETXSTH		0x05
#define ETXNDL		0x06
#define ETXNDH		0x07
#define ERXSTL		0x08
#define ERXSTH		0x09
#define ERXNDL		0x0A
#define ERXNDH		0x0B
#define ERXRDPTL	0x0C
#define ERXPDPTH	0x0D
#define ERXWRPTL	0x0E
#define ERXWRPTH	0x0F
#define EDMASTL		0x10
#define EDMASTH		0x11
#define EDMANDL		0x12
#define EDMANDH		0x13
#define EDMANDSTL	0x14
#define EDMANDSTH	0x15
#define EDMACSL		0x16
#define EDMACSH		0x17


//bank 1
#define EHT0		0x00
#define EHT1		0x01
#define EHT2		0x02
#define EHT3		0x03
#define	EHT4		0x04
#define EHT5		0x05
#define EHT6		0x06
#define EHT7		0x07
#define EPMM0		0x08
#define EPMM1		0x09
#define EPMM2		0x0A
#define EPMM3		0x0B
#define EPMM4		0x0C
#define EPMM5		0x0D
#define EPMM6		0x0E
#define EPMM7		0x0F
#define EPMCSL		0x10
#define EPMCSH		0x11


#define EPMOL		0x14
#define EPMOH		0x15
#define EWOLIE		0x16
#define EWOLIR		0x17
#define ERXFCON		0x18
#define	ERKTCNT		0x19

//bank 2
#define MACON1		0x00
#define MACON2		0x01
#define MACON3		0x02
#define MACON4		0x03
#define MABBIPG		0x04

#define MAIPGL		0x06
#define MAIPGH		0x07
#define MACLCON1	0x08
#define MACLCON2	0x09
#define MAMXFLL		0x0A
#define MAMXFLH		0x0B

#define MAPHSUP		0x0D



#define MICON		0x11
#define MICMD		0x12

#define MIREGADR	0x14

#define MIWRL		0x16
#define MIWRH		0x17
#define MIRDL		0x18
#define MIRDH		0x19

//bank 3
#define MAADR1		0x00
#define MAADR0		0x01
#define MAADR3		0x02
#define MAADR2		0x03
#define MAADR5		0x04
#define MAADR4		0x05
#define EBSTSD		0x06
#define EBSTCON		0x07
#define EBSTCSL		0x08
#define EBSTCSH		0x09
#define MISTAT		0x0A







#define EREVID		0x12


#define ECOCON		0x15

#define EFLOCON		0x17
#define EPAUSL		0x18
#define EPAUSH		0x19

//SPI指令集
#define RCR			(0x00 << 5)		//读控制寄存器
#define RBM			(0x01 << 5)		//读缓冲器
#define	WCR			(0x02 << 5)		//写控制寄存器
#define WBM			(0x03 << 5)		//写缓冲器
#define BFG			(0x04 << 5)		//位域置1
#define BFC			(0x05 << 5)		//位域清零
#define SC			(0x07 << 5)		//系统命令（软件复位）


#define BANK_0		0
#define BANK_1		1
#define BANK_2		2
#define BANK_3		3

static u8 bank = BANK_0;

void enc28j60_io_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				 //PA.11 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.11
	GPIO_SetBits(GPIOA,GPIO_Pin_11);						 //PA.11 输出高
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PA.11 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 	//推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.11
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //PA.11 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 	//推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.11
}






