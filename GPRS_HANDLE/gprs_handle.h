#ifndef GPRS_HANDLE_H
#define GPRS_HANDLE_H

#include "sys.h"
#include "usart.h"

#ifdef USE_SIM800A
#include "sim800a.h"
#define GPRS_REC_CHECK	sim800a_rec_check
#endif


#ifdef USE_BC95		
#include "bc95.h"
#define GPRS_REC_CHECK	bc95_rec_check
#endif

//与服务器通信协议中的特殊字符
#ifdef USE_SIM800A
#define GPRS_HEAD						0x7e
#define	GPRS_SYN_CMD					0x0e
#define GPRS_TAIL						0x7f

#define GPRS_CMD_HEART					0x01
#define GPRS_CMD_ALARM					0x02
#define GPRS_CMD_HEART_ACK				0x99
#define GPRS_CMD_SERVER_SYNC			0x03
#define GPRS_CMD_PASS_THTOUGH			0x04
#define GPRS_CMD_PASS_THTOUGH_ACK		0x07
#define GPRS_CMD_ALARM_ACK				0x05
#define GPRS_CMD_HOST_SYNC				0x06
#define GPRS_CMD_HOST_SYNC_ACK			0x99
#define GPRS_CMD_SMOKE_CHECK			0x08
#define GPRS_CMD_SMOKE_CHECK_ACK		0x09
#define GPRS_CMD_ELECTRICAL_CONTROL 	0x0a
#define GPRS_CMD_ELECTRICAL_CONTROL_ACK 0x0b
#define GPRS_CMD_ELECTRICAL_STATE		0x0c
#endif

#ifdef USE_BC95
#define GPRS_HEAD						0x7e
#define	GPRS_SYN_CMD					0x1e
#define GPRS_TAIL						0x7f

#define GPRS_CMD_HEART					0x01
#define GPRS_CMD_HEART_ACK				0x99
#define GPRS_CMD_ALARM					0x02
#define GPRS_CMD_ALARM_ACK				0x03
#define GPRS_CMD_DATA					0x04
#define GPRS_CMD_DATA_ACK				0x05
#endif


//extern u8* usart2_rx_buffer_get(u8* len);
//extern void usart2_rx_buffer_clean(void);
#define GPRS_RX_BUFFER_GET		usart2_rx_buffer_get
#define GPRS_RX_BUFFER_CLEAN	usart2_rx_buffer_clean

#ifndef USE_COAP
#define MAC_LENTH		4
typedef struct gprs_agreement
{
	u8 gprsHead;
	u8 gprsSynCmd;
	u8 gprsCmd;
	u8 gprsSeqL;
	u8 gprsSeqH;
	u8 gprsLenL;
	u8 gprsLenH;
	u8 grrsHostMac[MAC_LENTH];
	u8 gprsData[]; //data + CRC_L + CRC_H + gprsTrail
}GPRS_AGREEMENT;
#else

typedef struct gprs_coap_agreement
{
	u8 messageld[2];
	u8 coapHead[4];
	u8 coapSynCmd[4];
	u8 coapCmd[4];
	u8 coaptype[4];
	u8 coapIMEI[32];
	u8 coapIMSI[32];
	u8 coapPowerValue[4];
	u8 coapRSSI[4];
	u8 coapEARFCN[4];
	u8 coapSNR[4];
	u8 coapECL[4]; 
	u8 coapRSRP[4];
	u8 coapPCI[4];
	u8 coapLength[4];
	u8 coapData[]; //data + CRC_L + CRC_H + gprsTrail
}GPRS_COAP_AGREEMENT;
#endif


void gprs_handle(void);
#ifndef USE_COAP
s32 gprs_send_data_packet(u8* pioBuf, u8 ubCmd, u16 uwSeq, const u8 *pcMAC, const u8 *pcData, u16 dataLen);
#else
s32 gprs_coap_send_data_packet(u8* pioBuf, u8* type, const u8 *pcIMEI, const u8 *pcIMSI, u8* power_value, 
	u8 *pcRSSI, u8* pcEARFCN,u8* pcECL,u8* pcSNR,u8* pcRSRP,u8* pcPCI,const u8 *pcData, u16 dataLen);
#endif
void stringtoint(u8* dst,char* scr,u8 length,u8 negative);
extern u8 COAP_RSSI[4];
extern u8 COAP_EARFCN[4];
extern u8 COAP_RSRP[4];
extern u8 COAP_PCI[4];
extern u8 COAP_IMEI[16];
extern u8 COAP_IMSI[16];
extern u8 COAP_TYPE[4];
extern u8 COAP_POWERVALUE[4];
extern u8 COAP_ECL[4]; 
extern u8 COAP_SNR[4];
#endif
