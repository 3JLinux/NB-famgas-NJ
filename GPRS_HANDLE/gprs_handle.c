#include "gprs_handle.h"
#include <string.h>
#include "crc16.h"

//GPRS接收打印等级
#define GPRS_PRINTF_LEVEL 				12


#define GPRS_CMD_PLACE					2

const u8 COAP_MESSAGELD[2] = "00";
const u8 COAP_HEAD[4] = "3745";
const u8 COAP_SYNCMD[4] = "3145";
const u8 COAP_TAIL[4] = "3746";
const u8 COAP_CMD[4] = "3331";
u8 COAP_TYPE[4] = "3030";
u8 COAP_POWERVALUE[4] = "0064";
u8 COAP_IMEI[16] = "0";
u8 COAP_IMSI[16] = "0";
u8 COAP_RSSI[4] = "0";
u8 COAP_EARFCN[4] = "0";
u8 COAP_ECL[4] = "0";
u8 COAP_SNR[4] = "0";
u8 COAP_RSRP[4] = "0";
u8 COAP_PCI[4] = "0";

static u32 format_conversion(u8* dst,u8* src,u32 len);
static u32 hex2char(u8* scr_data,u8* obj_data,u32 len);

void gprs_handle(void)
{
	u32 len;
	u32 i;
	u8* gprs_rx_buffer = NULL;
	u8* gprs_rx_curr_buffer = NULL;
	u32 gprs_rx_curr_len = 0;
	u8 cout_time = 0;
	gprs_rx_buffer = GPRS_RX_BUFFER_GET(&len);
	for(i = 0;i < len;i++)
	{
		if(gprs_rx_buffer[i] == GPRS_HEAD && gprs_rx_buffer[i+1] == GPRS_SYN_CMD)
		{
			cout_time++;
			if(cout_time >= 2)
			{
				gprs_rx_curr_len = len - i;
				gprs_rx_curr_buffer = &gprs_rx_buffer[i];
				XPRINT(level_printf_hex,GPRS_PRINTF_LEVEL,"GPRS receive<<",gprs_rx_curr_buffer,gprs_rx_curr_len);
				cout_time = 0;
			}
		}
	}
	if(gprs_rx_curr_buffer[GPRS_CMD_PLACE] == GPRS_CMD_HEART)
	{
		
	}
	GPRS_RX_BUFFER_CLEAN();
}


#ifndef USE_COAP
/*****************************************************
函数原型： 		u32 gprs_send_data_packet(GPRS_AGREEMENT *pioBuf, u8 ubCmd, u16 uwSeq, const u8 *pcMAC, const u8 *pcData, u16 dataLen)
功能：			根据与服务器通信的协议进行数据打包
输入：			u8* pioBuf,				协议包指针
				u8 ubCmd, 				命令
				u16 uwSeq,	 			seq
				const u8 *pcMAC, 		主机mca
				const u8 *pcData, 		需要发送的数据指针
				u16 dataLen				发送数据的长度
返回：			整包数据长度
*****************************************************/
#define LEN_EXCEPT_DATA		10 //除了数据之外的包的长度
s32 gprs_send_data_packet(u8* pioBuf, u8 ubCmd, u16 uwSeq, const u8 *pcMAC, const u8 *pcData, u16 dataLen)
{
	static u16 nFrameL;
	u16 crc16_num;
	GPRS_AGREEMENT *pFrame = NULL;
	if (NULL == pioBuf)
	{
		return -1;
	}
	pFrame = (GPRS_AGREEMENT *)pioBuf;
	pFrame->gprsHead = GPRS_HEAD;
	pFrame->gprsSynCmd = GPRS_SYN_CMD;
	pFrame->gprsCmd = ubCmd;
	pFrame->gprsSeqL = uwSeq & 0xFF;
	pFrame->gprsSeqH = (uwSeq >> 8) & 0xFF;
	pFrame->gprsLenL = (dataLen + MAC_LENTH) & 0xFF;
	pFrame->gprsLenH = ((dataLen + MAC_LENTH) >> 8) & 0xFF;
	memcpy(pFrame->grrsHostMac,pcMAC,MAC_LENTH);
//	if(pcData == NULL || dataLen == 0)
//	{
//		pFrame->gprsLenL = MAC_LENTH;
//		pFrame->gprsLenH = 0;
//	}
//	else
//	{
	if(pcData != NULL && dataLen != 0)
	{
		memcpy(pFrame->gprsData,pcData,dataLen);
	}
//	}
	crc16_num = crc16((u8*)(&(pioBuf[1])),dataLen + 6 + MAC_LENTH); 	//crc从除包头外的第二位同步命令开始计算到数据结尾
	pFrame->gprsData[dataLen] = crc16_num & 0xFF;
	pFrame->gprsData[dataLen + 1] = (crc16_num >> 8) & 0xFF;
	pFrame->gprsData[dataLen + 2] = GPRS_TAIL;
	
	nFrameL = dataLen + LEN_EXCEPT_DATA + MAC_LENTH;
	return nFrameL;
}


#else
/*****************************************************
函数原型： 		u32 gprs_send_data_packet(GPRS_AGREEMENT *pioBuf, u8 ubCmd, u16 uwSeq, const u8 *pcMAC, const u8 *pcData, u16 dataLen)
功能：			根据与服务器通信的协议进行数据打包
输入：			u8* pioBuf,				协议包指针
				u8 ubCmd, 				命令
				u16 uwSeq,	 			seq
				const u8 *pcMAC, 		主机mca
				const u8 *pcData, 		需要发送的数据指针
				u16 dataLen				发送数据的长度
返回：			整包数据长度
*****************************************************/
s32 gprs_coap_send_data_packet(u8* pioBuf, u8* type, const u8 *pcIMEI, const u8 *pcIMSI, u8* power_value, 
	u8 *pcRSSI, u8* pcEARFCN,u8* pcECL,u8* pcSNR,u8* pcRSRP,u8* pcPCI,const u8 *pcData, u16 dataLen)
{
	static u32 nFrameL = 0;
	u16 crc16_num = 0;
	//u32 i;
	u8 crc16_numL,crc16_numH;
	u32 buf_len=0;
	GPRS_COAP_AGREEMENT *pFrame = NULL;
	if (NULL == pioBuf)
	{
		return -1;
	}
	nFrameL = dataLen*4 +18+32+32+32+8+4;
	
	pFrame = (GPRS_COAP_AGREEMENT *)pioBuf;
	memcpy(pFrame->messageld,COAP_MESSAGELD,sizeof(COAP_MESSAGELD)/sizeof(u8));
	memcpy(pFrame->coapHead,COAP_HEAD,sizeof(COAP_HEAD)/sizeof(u8));
	memcpy(pFrame->coapSynCmd,COAP_SYNCMD,sizeof(COAP_SYNCMD)/sizeof(u8));
	memcpy(pFrame->coapCmd,COAP_CMD,sizeof(COAP_CMD)/sizeof(u8));
	//format_conversion(COAP_TYPE,&type,strlen((const char *)&type));
	memcpy(pFrame->coaptype,COAP_TYPE,4);
	hex2char((u8*)pcIMEI,pFrame->coapIMEI,16);
	hex2char((u8*)pcIMSI,pFrame->coapIMSI,16);
	//memcpy(pFrame->coapIMEI,pcIMEI,16);
	//memcpy(pFrame->coapIMSI,pcIMSI,16);
	//format_conversion(COAP_POWERVALUE,&power_value,strlen((const char *)&power_value));
	memcpy(pFrame->coapPowerValue,COAP_POWERVALUE,4);
	memcpy(pFrame->coapRSSI,pcRSSI,4);
	memcpy(pFrame->coapEARFCN,pcEARFCN,4);
	memcpy(pFrame->coapECL,pcECL,4);
	memcpy(pFrame->coapSNR,pcSNR,4);
	memcpy(pFrame->coapRSRP,pcRSRP,4);
	memcpy(pFrame->coapPCI,pcPCI,4);
	
	
	pFrame->coapLength[0] = '0';
	pFrame->coapLength[1] = '0';
	buf_len = dataLen*2;
	hex2char((u8*)(&buf_len),pFrame->coapLength+2,1);
	//pFrame->coapLength[2] = '1';
	//pFrame->coapLength[3] = '4';
	
	crc16_num = crc16((u8*)pcData,dataLen);
	crc16_numL = crc16_num & 0xFF;
	crc16_numH = (crc16_num>>8) & 0xFF;
	//hex2char((u8*)pcData,pFrame->coapData,dataLen);
	//hex2char((u8*)&(dataLen),pFrame->coapLength+2,1);
	//stringtoint(pFrame->coapLength+4,,4);
	format_conversion(pFrame->coapData,(u8*)pcData,dataLen);
	format_conversion(pFrame->coapData+(dataLen*4),&crc16_numL,1);
	format_conversion(pFrame->coapData+(dataLen*4+4),&crc16_numH,1);
	//hex2char(&crc16_numL,pFrame->coapData+(dataLen*2),1);
	//hex2char(&crc16_numH,pFrame->coapData+(dataLen*2+2),1);
	memcpy(pFrame->coapData+dataLen*4+8,COAP_TAIL,4);
	
//	pFrame->gprsSynCmd = GPRS_SYN_CMD;
//	pFrame->gprsCmd = ubCmd;
//	pFrame->gprsSeqL = uwSeq & 0xFF;
//	pFrame->gprsSeqH = (uwSeq >> 8) & 0xFF;
//	pFrame->gprsLenL = (dataLen + MAC_LENTH) & 0xFF;
//	pFrame->gprsLenH = ((dataLen + MAC_LENTH) >> 8) & 0xFF;
//	memcpy(pFrame->grrsHostMac,pcMAC,MAC_LENTH);
////	if(pcData == NULL || dataLen == 0)
////	{
////		pFrame->gprsLenL = MAC_LENTH;
////		pFrame->gprsLenH = 0;
////	}
////	else
////	{
//	if(pcData != NULL && dataLen != 0)
//	{
//		memcpy(pFrame->gprsData,pcData,dataLen);
//	}
////	}
//	crc16_num = crc16((u8*)(&(pioBuf[1])),dataLen + 6 + MAC_LENTH); 	//crc从除包头外的第二位同步命令开始计算到数据结尾
//	pFrame->gprsData[dataLen] = crc16_num & 0xFF;
//	pFrame->gprsData[dataLen + 1] = (crc16_num >> 8) & 0xFF;
//	pFrame->gprsData[dataLen + 2] = GPRS_TAIL;
//	
//	nFrameL = dataLen + LEN_EXCEPT_DATA + MAC_LENTH;
	return nFrameL;
}


static u32 format_conversion(u8* dst,u8* src,u32 len)
{
	//u32 dst_len = 0;
	u32 i;
	u32 j=0;
	u8 buf[300] = {0};
	for(i=0;i<len;i++)
	{
		buf[j] = (src[i] >> 4) &0x0F;
		if(buf[j] <= 9)
		{
			buf[j] += 0x30;
		}
		else
		{
			buf[j] = buf[j] - 10 + 'A';
		}
		j++;
		buf[j] = src[i] &0x0F;
		if(buf[j] <= 9)
		{
			buf[j] += 0x30;
		}
		else
		{
			buf[j] = buf[j] - 10 + 'A';
		}
		j++;
	}
	for(i=0;i<j;i++)
	{
		if((buf[i]>='0') && (buf[i]<='9'))
		{
			dst[2*i] = '3';
			dst[2*i+1] = buf[i];
		}
		else
		{
			dst[2*i] = '4';
			dst[2*i+1] = buf[i] - 'A' + '1';
		}
	}
	return 2*j;
}

static u32 hex2char(u8* scr_data,u8* obj_data,u32 len)
{
	u32 i;
	u32 j = 0;
	for(i = 0;i < len;i++)
	{
		obj_data[j] = scr_data[i]/16;
		if(obj_data[j] >= 10)
		{
			obj_data[j] =obj_data[j] - 10 + 'A';
		}
		else
		{
			obj_data[j] += '0';
		}
		j++;
		obj_data[j] = scr_data[i]%16;
		if(obj_data[j] >= 10)
		{
			obj_data[j] =obj_data[j] - 10 + 'A';
		}
		else
		{
			obj_data[j] += '0';
		}
		j++;
	}
	obj_data[j] = 0;
	return j;
}



void stringtoint(u8* dst,char* scr,u8 length,u8 negative)
{
	u16 buf = 0;
	u8 buffer[4] = "0";
	u8 i;
	for(i=0;i<4;i++)
	{
		dst[i] = '0';
	}
	for(i=0;i<length;i++)
	{
		buf *= 10;
		buf += scr[i] - '0';
	}
	if(negative)
	{
		buf =0xFFFF - buf + 1;
	}
	i=0;
	while(buf>0)
	{
		if(buf/16==0)
		{
			buf%=16;
			break;
		}
		if(buf%16<10)
		{
			buffer[i] = buf%16 + '0';
		}
		else
		{
			//buf -= 10;
			buffer[i] = buf%16 - 10 + 'A';
		}
		i++;
		buf/=16;
	}
	if(buf>=0)
	{
		if(buf<10)
		{
			buffer[i] = buf + '0';
		}
		else
		{
			//buf -= 10;
			buffer[i] = buf - 10 + 'A';
		}
	}
	else
	{
		//i--;
	}
	
	//memcpy(dst+3-i,buffer,i+1);
	for(i=0;i<4;i++)
	{
		if(buffer[3-i] == 0)
		{
			buffer[3-i] += '0'; 
		}
		dst[i] = buffer[3-i];
	}
}


#endif










