#include "bc95.h"
#include "string.h"

#ifdef USE_BC95

#define SUCCEED			1
#define ID_NUM_ERROR 	0
#define DATA_ERROR 		0


//const char UDP_local_port[] = "4569";
//const char UDP_ipv4_addr[] = "119.29.224.28";
static const char double_quote[] = "\"";
static const char comma[] = ",";

/*****************************************************
函数原型： 		void bc95_send_cmd(const char *cmd)
功能：			bc95发送命令
输入：			const u8 *cmd 命令
返回：			0
*****************************************************/
static void bc95_send_cmd(const char *cmd)
{
	static char bc95_send_buff[512] = {0};
	u32 bc95_send_num = 0;
	u32 i;
	if(cmd != NULL && (strlen((char*)cmd)) < 512)
	{
		bc95_send_num = strlen((char *)cmd);
		memcpy(bc95_send_buff,cmd,bc95_send_num);
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 send>>",bc95_send_buff,bc95_send_num);
		bc95_Send_Data(bc95_send_buff,bc95_send_num);
		bc95_Send_Data("\r\n",strlen("\r\n"));
		for(i=0;i<bc95_send_num;i++)
		{
			bc95_send_buff[i] = 0;
		}
	}
	else
	{
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 error>>","cmd input error",strlen("cmd input error"));
	}
}

/*****************************************************
函数原型： 		void bc95_rec_check(void)
功能：			bc95接收检测
输入：			
返回：			0
*****************************************************/
u8* bc95_rec_check(const char *pcTarget, const char* pcFindStr)
{
	char *strx = NULL;
	strx = (char *)strstr(pcTarget,pcFindStr);
	return (u8*)strx;
}




/*****************************************************
函数原型： 		u8 bc95_setID(u8* id_num)
功能：			写模块IMEI
输入：			u8 *id_num IMEI号
返回：			ID_NUM_ERROR 号码大于15位出错
				SUCCEED	发送成功
*****************************************************/
u8 bc95_setID(u8* id_num)
{
	char send_cmd_buffer[30];
	const char set_id_cmd[] = "AT+NTSETID=1,";
	memcpy(send_cmd_buffer,set_id_cmd,strlen(set_id_cmd));
	if(strlen((char*)id_num) <= 15)
	{
		strcat(send_cmd_buffer,double_quote);
		strcat(send_cmd_buffer,(char*)id_num);
		//strcat(send_cmd_buffer,double_quote);
		bc95_send_cmd(send_cmd_buffer);
	}
	else
	{
		return ID_NUM_ERROR;
	}
	return SUCCEED;
}


/*****************************************************
函数原型： 		void bc95_request_sofware(void)
功能：			Request Manufacturer Revision
输入：			无
返回：			无
*****************************************************/
void bc95_request_sofware(void)
{
	bc95_send_cmd("AT+CGSN=1");
}


/*****************************************************
函数原型： 		void bc95_request_IMEI(void)
功能：			Request Product Seril Number
输入：			无
返回：			无
*****************************************************/
void bc95_request_IMEI(void)
{
	bc95_send_cmd("AT+CGSN=1");
}

/*****************************************************
函数原型： 		void bc95_restar(void)
功能：			Reboot
输入：			无
返回：			无
*****************************************************/
void bc95_restar(void)
{
	bc95_send_cmd("AT+NRB");
}


/*****************************************************
函数原型： 		void bc95_get_singal(void)
功能：			Get Singal Strength Indicator
输入：			无
返回：			无
*****************************************************/
void bc95_get_singal(void)
{
	bc95_send_cmd("AT+CSQ");
}


/*****************************************************
函数原型： 		void bc95_query_UE_statistics(void)
功能：			Query UE Statistics
输入：			无
返回：			无
*****************************************************/
void bc95_query_UE_statistics(void)
{
	bc95_send_cmd("AT+NUESTATS");
}

/*****************************************************
函数原型： 		void bc95_query_UE_statistics_CELL(void)
功能：			Query UE Statistics
输入：			无
返回：			无
*****************************************************/
void bc95_query_UE_statistics_CELL(void)
{
	bc95_send_cmd("AT+NUESTATS=CELL");
}



/*****************************************************
函数原型： 		void bc95_set_band_5(void)
功能：			Set Support
输入：			无
返回：			无
*****************************************************/
void bc95_set_band_5(void)
{
	bc95_send_cmd("AT+NBAND=5");
}

/*****************************************************
函数原型： 		void bc95_set_band_8(void)
功能：			Set Support
输入：			无
返回：			无
*****************************************************/
void bc95_set_band_8(void)
{
	bc95_send_cmd("AT+NBAND=8");
}

/*****************************************************
函数原型： 		void bc95_set_band_20(void)
功能：			Set Support
输入：			无
返回：			无
*****************************************************/
void bc95_set_band_20(void)
{
	bc95_send_cmd("AT+NBAND=20");
}
/*************************************************Open scrambing code****************************************************/

/*****************************************************
函数原型： 		void bc95_set_min_fun(void)
功能：			Set Minimum functionality
输入：			无
返回：			无
*****************************************************/
void bc95_set_min_fun(void)
{
	bc95_send_cmd("AT+CFUN=0");
}

/*****************************************************
函数原型： 		void bc95_request_UE_behaviour(void)
功能：			Request UE Behaviour
输入：			无
返回：			无
*****************************************************/
void bc95_request_UE_behaviour(void)
{
	bc95_send_cmd("AT+NCONFIG?");
}

/*****************************************************
函数原型： 		void bc95_open_UE_behaviour(void)
功能：			Open UE Behaviour
输入：			无
返回：			无
*****************************************************/
void bc95_open_UE_behaviour(void)
{
	bc95_send_cmd("AT+NCONFIG=CR_0354_0338_SCRAMBLING,TRUE");
	bc95_send_cmd("AT+NCONFIG=CR_0859_SI_AVOID,TRUE");
}

/*****************************************************
函数原型： 		void bc95_close_UE_behaviour(void)
功能：			Close UE Behaviour
输入：			无
返回：			无
*****************************************************/
void bc95_close_UE_behaviour(void)
{
	bc95_send_cmd("AT+NCONFIG=CR_0354_0338_SCRAMBLING,FALSE");
	bc95_send_cmd("AT+NCONFIG=CR_0859_SI_AVOID,FALSE");
}

/************************************************************************************************************************/

/*************************************************Attach Network*********************************************************/

/*****************************************************
函数原型： 		void bc95_request_phone_fun(void)
功能：			Request Set Phone Functionality
输入：			无
返回：			无
*****************************************************/
void bc95_request_phone_fun(void)
{
	bc95_send_cmd("AT+CFUN?");
}


/*****************************************************
函数原型： 		void bc95_configure_server_settings(char* ip_addr,char* port)
功能：			Configure and Query CDP Server Settings
输入：			char* ip_addr ip地址
				char* port 端口
返回：			无
*****************************************************/
void bc95_configure_server_settings(char* ip_addr,char* port)
{
	char send_cmd_buffer[30];
	const char configure_server[] = "AT+NCDP=";
	memcpy(send_cmd_buffer,configure_server,strlen(configure_server));
	strcat(send_cmd_buffer,ip_addr);
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,port);
	//strcat(send_cmd_buffer,double_quote);
	bc95_send_cmd(send_cmd_buffer);
}



/*****************************************************
函数原型： 		void bc95_set_phone_fun(void)
功能：			Set Phone Functionality
输入：			无
返回：			无
*****************************************************/
void bc95_set_phone_fun(void)
{
	bc95_send_cmd("AT+CFUN=1");
}


/*****************************************************
函数原型： 		void bc95_query_IMSI(void)
功能：			Request International Mobile Subscriber Identity
输入：			无
返回：			无
*****************************************************/
/*延时3秒后查询，确保模块识别该卡*/
void bc95_query_IMSI(void)
{
	bc95_send_cmd("AT+CIMI");
}

/*****************************************************
函数原型： 		void bc95_Configuration_PDP(void)
功能：			Define a PDP Context
输入：			无
返回：			无
*****************************************************/
void bc95_Configuration_PDP(void)
{
	bc95_send_cmd("AT+CGDCONT=1,\"IP\",\"ctnet\"");
} 

/*****************************************************
函数原型： 		void bc95_request_bands(void)
功能：			Request Supported Bands
输入：			无
返回：			无
*****************************************************/
void bc95_request_bands(void)
{
	bc95_send_cmd("AT+NBAND?");
}


/*****************************************************
函数原型： 		void bc95_set_network_reg_status(void)
功能：			Set EPS Network Registration Status
输入：			无
返回：			无
*****************************************************/
void bc95_set_network_reg_status(void)
{
	bc95_send_cmd("AT+CEREG=1");
}

/*****************************************************
函数原型： 		void bc95_set_connetion_status(void)
功能：			Set Signalling Connection Status
输入：			无
返回：			无
*****************************************************/
void bc95_set_connetion_status(void)
{
	bc95_send_cmd("AT+CSCON=1");
}

/*****************************************************
函数原型： 		void bc95_select_PLMN(void)
功能：			PLMN Selection
输入：			无
返回：			无
*****************************************************/
void bc95_select_PLMN(void)
{
	bc95_send_cmd("AT+COPS=1,2,\"46011\"");
}


/*****************************************************
函数原型： 		void bc95_activate_network(void)
功能：			Activate the network
输入：			无
返回：			无
*****************************************************/
void bc95_activate_network(void)
{
	bc95_send_cmd("AT+CGATT=1");
}


/*****************************************************
函数原型： 		void bc95_request_activate_network(void)
功能：			Query whether network activation
输入：			无
返回：			无
*****************************************************/
/*查询模块注网情况，30秒内查询直到为1*/
void bc95_request_activate_network(void)
{
	bc95_send_cmd("AT+CGATT?");
}


/*****************************************************
函数原型： 		void bc95_network_reg_status(void)
功能：			Request EPS Network Registration Status
输入：			无
返回：			无
*****************************************************/
void bc95_request_network_reg_status(void)
{
	bc95_send_cmd("AT+CEREG?");
}

/*****************************************************
函数原型： 		void bc95_network_reg_status(void)
功能：			Request Signalling Connection Status
输入：			无
返回：			无
*****************************************************/
void bc95_request_connetion_status(void)
{
	bc95_send_cmd("AT+CSCON?");
}





/**************************************************************************************************************************/


/*****************************************************
函数原型： 		void bc95_creat_socket(void)
功能：			Create a Socket
输入：			无
返回：			无
*****************************************************/
void bc95_creat_UDP_socket(char* port)
{
	char send_cmd_buffer[30] = {0};
	const char set_UDP_socket[] = "AT+NSOCR=DGRAM,17,";
	memcpy(send_cmd_buffer,set_UDP_socket,strlen(set_UDP_socket));
	strcat(send_cmd_buffer,port);
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,"1");
	bc95_send_cmd(send_cmd_buffer);
}

char* u32_to_hex(const u32 num)
{
	static char source_num[10] = {0};
	char change_buff[10] = {0};
	u32 change_num = num;
	u8 i = 0;
	u8 j = 0;
	while(change_num != 0)
	{
		change_buff[i++]=change_num%10 + '0';
		change_num/=10;
	}
	for(j = 0;j < i;j++)
	{
		source_num[j] = change_buff[(i-1) - j];
	}
	//source_num[i] = '\n';
	return source_num;
}


/*****************************************************
函数原型： 		void bc95_UDP_send_messages()
功能：			UDP Send messages
输入：			无
返回：			无
*****************************************************/
#define UDP_SEND_MAX 512
u8 bc95_UDP_send_messages(char socket,char* ip_addr,char* port,u32 len,u8* data)
{
	char send_cmd_buffer[530] = {0};
	const char udp_send_messages[] = "AT+NSOST=";
	if((len >= UDP_SEND_MAX) ||(data == NULL) || (len == 0) || (len == NULL))
	{
		return DATA_ERROR;
	}
	socket = socket + '0';
	memcpy(send_cmd_buffer,udp_send_messages,strlen(udp_send_messages));
	strcat(send_cmd_buffer,&socket);
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,ip_addr);
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,port);
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,u32_to_hex(len));
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,(char*)data);
	bc95_send_cmd(send_cmd_buffer);
	//bc95_send_cmd("AT+NSOST=0,119.29.155.148,4568,4,22061740");
	return SUCCEED;
}

void bc95_close_socket(void)
{
	bc95_send_cmd("AT+NSOCL=0");
}

/*****************************************************
函数原型： 		void bc95_UDP_receive_commend(u32 len)
功能：			Receive Commend (UDP only)
输入：			无
返回：			无
*****************************************************/
u8 bc95_UDP_receive_commend(u32 len)
{
	const char udp_rec_messages[] = "AT+NSORF=0,";
	char send_cmd_buffer[20] = {0};
	if(len > 0 && len < 256)
	{
		memcpy(send_cmd_buffer,udp_rec_messages,strlen(udp_rec_messages));
		strcat(send_cmd_buffer,u32_to_hex(len));
		bc95_send_cmd(send_cmd_buffer);
	}
	else
	{
		return DATA_ERROR;
	}
	return SUCCEED;
}


/*****************************************************
函数原型： 		void bc95_creat_COAP_socket(void)
功能：			Create a Socket
输入：			无
返回：			无
*****************************************************/
void bc95_creat_COAP_socket(char* ip_addr,char* port)
{
	char send_cmd_buffer[100] = {0};
	const char set_UDP_socket[] = "AT+NCDP=";
	memcpy(send_cmd_buffer,set_UDP_socket,strlen(set_UDP_socket));
	strcat(send_cmd_buffer,ip_addr);
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,port);
	bc95_send_cmd(send_cmd_buffer);
}


/*****************************************************
函数原型： 		void bc95_request_CDP_server(void)
功能：			Query CDP server
输入：			无
返回：			无
*****************************************************/
void bc95_request_CDP_server(void)
{
	bc95_send_cmd("AT+NCDP?");
}

/*****************************************************
函数原型： 		void bc95_NSMI(void)
功能：			Sent message indications is enabled
输入：			无
返回：			无
*****************************************************/
void bc95_NSMI(void)
{
	bc95_send_cmd("AT+NSMI=1");
}

/*****************************************************
函数原型： 		void bc95_NNMI(void)
功能：			Sent message indications is enabled
输入：			无
返回：			无
*****************************************************/
void bc95_NNMI(void)
{
	bc95_send_cmd("AT+NNMI=2");
}


/*****************************************************
函数原型： 		void bc95_UDP_send_messages()
功能：			UDP Send messages
输入：			无
返回：			无
*****************************************************/
#define COAP_SEND_MAX 512
u8 bc95_COAP_send_messages(u32 len,u8* data)
{
	char send_cmd_buffer[512] = {0};
	const char coap_send_messages[] = "AT+NMGS=";
	if((len >= COAP_SEND_MAX) ||(data == NULL) || (len == 0) || (len == NULL))
	{
		return DATA_ERROR;
	}
	memcpy(send_cmd_buffer,coap_send_messages,strlen(coap_send_messages));
	strcat(send_cmd_buffer,u32_to_hex(len));
	strcat(send_cmd_buffer,comma);
	strcat(send_cmd_buffer,(char*)data);
	bc95_send_cmd(send_cmd_buffer);
	//bc95_send_cmd("AT+NSOST=0,119.29.155.148,4568,4,22061740");
	return SUCCEED;
}


#endif










