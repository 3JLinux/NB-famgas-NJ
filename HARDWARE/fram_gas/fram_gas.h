#ifndef FRAM_GAS_H
#define FRAM_GAS_H

#include "sys.h"

#define FRAM_GAS_485_READ_CMD		0x03
#define FRAM_GAS_485_SET_CMD		0x06

#define FRAM_GAS_SOFTWARE_NUM_ADDR	0x01		//����汾��
#define FRAM_GAS_MODEL_ADDR			0x07		//̽�����ͺ�
#define FRAM_GAS_SENSOR_TYPE_ADDR	0x08		//̽��������
#define FRAM_GAS_UNIT_ADDR			0x09		//��λ
#define FRAM_GAS_RANGE_ADDR			0x0a		//����
#define FRAM_GAS_A1_WARN_POINT		0x0b		//A1������
#define FRAM_GAS_A2_WARN_POINT		0x0c		//A2������
#define FRAM_GAS_SENSOR_STATUS		0x10		//̽����״̬
#define FRAM_GAS_RELAY1_STATUS		0x11		//�̵���1״̬
#define FRAM_GAS_CONCENTRATION		0x13		//����Ũ��
#define FRAM_GAS_SENSOR_TEMP		0x14		//�������¶�
#define FRAM_GAS_AD0				0x15		//
#define FRAM_GAS_AD1				0x16		//
#define FRAM_GAS_AD2				0x17		//


#define FRAM_GAS_ALARM	PAin(11)
void fram_gas_init(void);

typedef struct gas_agreement
{
	u8 GASslave_addr;
	u8 GASfunction_code;
	u8 GASregister_star_addr[2];
	u8 GASregister_num[2];
	u8 GAScrc[2];
}GAS_AGREEMENT;

u8 fram_gas_read_packet(u8 *fram_gas_485_packet,u8 slave_addr,u8 cmd,u8 *star_addr,u8 *register_num);
#endif
