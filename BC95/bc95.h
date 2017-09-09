#ifndef	BC95_H
#define BC95_H

#include "sys.h"
#include "usart.h"
#include "shell.h"

#define BC95_PRINTF_LEVEL		8

extern void USART3_send(char *data,u8 num);
#define	bc95_Send_Data USART3_send

u8 bc95_setID(u8* id_num);
void bc95_restar(void);
void bc95_set_band_5(void);
void bc95_set_band_8(void);
void bc95_set_band_20(void);

void bc95_set_min_fun(void);
void bc95_request_UE_behaviour(void);
void bc95_open_UE_behaviour(void);
void bc95_close_UE_behaviour(void);

void bc95_request_IMEI(void);
void bc95_get_singal(void);
void bc95_query_UE_statistics(void);

u8* bc95_rec_check(const char *pcTarget, const char* pcFindStr);

void bc95_request_phone_fun(void);
void bc95_configure_server_settings(char* ip_addr,char* port);
void bc95_set_phone_fun(void);
void bc95_query_IMSI(void);
void bc95_Configuration_PDP(void);
void bc95_request_bands(void);
void bc95_activate_network(void);
void bc95_request_activate_network(void);
void bc95_request_network_reg_status(void);
void bc95_request_connetion_status(void);

void bc95_creat_UDP_socket(char* port);
u8 bc95_UDP_send_messages(char socket,char* ip_addr,char* port,u32 len,u8* data);
void bc95_close_socket(void);

#endif


