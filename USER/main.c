#include "delay.h"
#include "sys.h"
#include "includes.h" 
#include "led.h"
#include "shell.h"
#include "usart.h"
#include "gprs_handle.h"
#include <string.h>
#include "fram_gas.h"
#include "ucos_ii.h"
#include "get_mac.h"
#include "iwdg.h"


#ifdef	USE_BC95
#include "bc95.h"
#endif
#ifdef USE_SIM800A
#include "sim800a.h"
#endif

#define BC95_AUTO_CONNECT	1

#define ATCMD_MAX_REPEAT_NUMS			3
#define NET_CONNRTION_ERR				0
#define NET_CONNRTION_SUCCEED			1
#define UDP_SEND_ERR					0
#define UDP_SEND_SUCCEED				2
#define CONFIGURE_ERR					0
#define CONFIGURE_SUCCEED				1
#define FALSE							0
#define TRUE							1

//模块需要转换的特殊字符
#define GPRS_TRAN	0xef
#define GPRS_DSC	0xea
#define GPRS_SRC	0x1a
#define GPRS_EB		0xeb
#define GPRS_DSC1B	0x1b

static int osqGetOut0x00(u8* pbuf,const u8* pdata, u32 len);
static int osqGetOut0xef(u8* pbuf,const u8* pdata, u32 len);

#ifdef USE_SIM800A
#define sim800a_rx_buffer_clean	usart2_rx_buffer_clean
#define sim800a_rx_buffer_get	usart2_rx_buffer_get
#endif

//const char UDP_local_port[] = "4569";			//端口
const char UDP_local_port[] = "4568";			//端口 
//const char UDP_ipv4_addr[] = "119.29.224.28";	//测试服务器
const char UDP_ipv4_addr[] = "119.29.155.148";	//正式服务器

#ifdef USE_BC95
static u8 bc95_creat_network_connetion(void);
static u8 bc95_UDP_send(char* ip_addr,char* port,u32 length,u8* data);
static void bc95_band_set(u8 band);
static u8 bc95_open_scrambing_code(void);
static u32 hex2char(u8* scr_data,u8* obj_data,u32 len);
#define bc95_rx_buffer_clean	usart3_rx_buffer_clean
#define bc95_rx_buffer_get	usart3_rx_buffer_get
#define BAND5	5
#define BAND8	8
#define BAND20	20

#define DEFAULT_PRINTF_LEVEL 2
const char bc95_connetion_error[] = "NB-IoT CONNERTION ERR";
const char bc95_connetion_connet[] = "NB-IoT CONNERTING";
const char bc95_connetion_succeed[] = "NB-IoT CONNERTION SUCCEED";
const char bc95_UDP_send_err[] = "NB-IoT UDP SEND ERROR";

static u8 bc95_send_ok = TRUE;
static u8 rec_len = 0;
#endif

static void QR_printf(void);

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	


//SHELL任务
//设置任务优先级
#define SHELL_TASK_PRIO       			9 
//设置任务堆栈大小
#define SHELL_STK_SIZE  		    		64
//任务堆栈	
OS_STK SHELL_TASK_STK[SHELL_STK_SIZE];
//任务函数
void shell_task(void *pdata);

 			   
////LED0任务
////设置任务优先级
//#define LED0_TASK_PRIO       			7 
////设置任务堆栈大小
//#define LED0_STK_SIZE  		    		64
////任务堆栈	
//OS_STK LED0_TASK_STK[LED0_STK_SIZE];
////任务函数
//void led0_task(void *pdata);


////LED1任务
////设置任务优先级
//#define LED1_TASK_PRIO       			6 
////设置任务堆栈大小
//#define LED1_STK_SIZE  					64
////任务堆栈
//OS_STK LED1_TASK_STK[LED1_STK_SIZE];
////任务函数
//void led1_task(void *pdata);

//燃气报警任务
//设置任务优先级
#define FRAM_GSA_TASK_PRIO       			5 
//设置任务堆栈大小
#define FRAM_GSA_STK_SIZE  					64
//任务堆栈
OS_STK FRAM_GSA_TASK_STK[FRAM_GSA_STK_SIZE];
//任务函数
void fram_gas_task(void *pdata);

////心跳任务
////设置任务优先级
//#define HEARTBEAT_TASK_PRIO       			4 
////设置任务堆栈大小
//#define HEARTBEAT_STK_SIZE  				64
////任务堆栈
//OS_STK HEARTBEAT_TASK_STK[HEARTBEAT_STK_SIZE];
////任务函数
//void heartbeat_task(void *pdata);


////main任务
////设置任务优先级
//#define MAIN_TASK_PRIO       			3 
////设置任务堆栈大小
//#define MAIN_STK_SIZE  					64
////任务堆栈
//OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
////任务函数
//void main_task(void *pdata);

#ifdef	USE_BC95
//BC95任务
//设置任务优先级
#define BC95_TASK_PRIO       			2 
//设置任务堆栈大小
#define BC95_STK_SIZE  					1024
//任务堆栈
OS_STK BC95_TASK_STK[BC95_STK_SIZE];
//任务函数
void bc95_task(void *pdata);

OS_EVENT * q_bc95_send_msg;
void * MsgGrp[256];			//消息队列存储地址,最大支持256个消息
#endif




#ifdef	USE_SIM800A
//BC95任务
//设置任务优先级
#define SIM800A_TASK_PRIO       			2
//设置任务堆栈大小
#define SIM800A_STK_SIZE  					(8*1024)
//任务堆栈
OS_STK SIM800A_TASK_STK[SIM800A_STK_SIZE];
//任务函数
void sim800a_task(void *pdata);

OS_EVENT * q_sim800a_send_msg;
void * MsgGrp[256];			//消息队列存储地址,最大支持256个消息
#endif

OS_TMR	*heartbeat_tmr;
OS_TMR  *feeddog_tmr;
static void heartbeat_tmr_callback(void);
static void feeddog_tmr_callback(void);

static u8 connetion_state = NET_CONNRTION_ERR;  //网络连接状态
static u16 seq = 0;
static u8 mac[4] = {0x01,0x02,0x03,0x04};

 int main(void)
 {	
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	//LED_Init();		  	//初始化与LED连接的硬件接口
	fram_gas_init();
	uart_init(9600);
	//Get_Device_MAC(mac);
	uart2_init(9600);
	uart3_init(9600);
	delay_ms(2000);
	QR_printf();
	delay_ms(2000);
	uart_init(921600);
//	IWDG_Init(4,625*10);
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	
 }

	  
//开始任务
void start_task(void *pdata)
{
	u8 err;
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
#ifdef	USE_BC95
	q_bc95_send_msg = OSQCreate(&MsgGrp[0],256);
#endif
#ifdef USE_SIM800A
	q_sim800a_send_msg = OSQCreate(&MsgGrp[0],256);
#endif
  	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
 	//OSTaskCreate(led0_task,(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);
 	//OSTaskCreate(led1_task,(void *)0,(OS_STK*)&LED1_TASK_STK[LED1_STK_SIZE-1],LED1_TASK_PRIO);
	//OSTaskCreate(main_task,(void *)0,(OS_STK*)&MAIN_TASK_STK[MAIN_STK_SIZE-1],MAIN_TASK_PRIO);
	OSTaskCreate(shell_task,(void *)0,(OS_STK*)&SHELL_TASK_STK[SHELL_STK_SIZE-1],SHELL_TASK_PRIO);
	//OSTaskCreate(heartbeat_task,(void *)0,(OS_STK*)&HEARTBEAT_TASK_STK[HEARTBEAT_STK_SIZE-1],HEARTBEAT_TASK_PRIO);
#ifdef	USE_BC95
	OSTaskCreate(bc95_task,(void *)0,(OS_STK*)&BC95_TASK_STK[BC95_STK_SIZE-1],BC95_TASK_PRIO);
	OSTaskCreate(fram_gas_task,(void *)0,(OS_STK*)&FRAM_GSA_TASK_STK[FRAM_GSA_STK_SIZE-1],FRAM_GSA_TASK_PRIO);
#endif
#ifdef	USE_SIM800A
	OSTaskCreate(sim800a_task,(void *)0,(OS_STK*)&SIM800A_TASK_STK[SIM800A_STK_SIZE-1],SIM800A_TASK_PRIO);
#endif
	heartbeat_tmr = OSTmrCreate((3*60*100),(3*60*100),OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)heartbeat_tmr_callback,0,(INT8U*)"heartbeat_tmr",&err);
	//feeddog_tmr = OSTmrCreate((100),(100),OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)feeddog_tmr_callback,0,(INT8U*)"feeddog_tmr",&err);
	OSTmrStart(heartbeat_tmr,&err);
	//OSTmrStart(feeddog_tmr,&err);
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}

////LED0任务
//void led0_task(void *pdata)
//{	 	
//	while(1)
//	{
//		LED0=0;
//		delay_ms(80);
//		LED0=1;
//		delay_ms(920);
//	}
//}

////LED1任务
//void led1_task(void *pdata)
//{	  
//	while(1)
//	{
//		LED1=0;
//		delay_ms(300);
//		LED1=1;
//		delay_ms(300);
//	}
//}

//报警任务
void fram_gas_task(void *pdata)
{
	u8 nFrameL = 0;
	static u8 fram_gas_alarm_packet[14] = {0};
	static u8 fram_gas_alarm_packet_buffer[28] = {0};
	u8 i;
	while(1)
	{
		if(connetion_state != NET_CONNRTION_ERR && FRAM_GAS_ALARM)
		{
			if(bc95_send_ok)
			{
				for(i=0;i<28;i++)
				{
					fram_gas_alarm_packet_buffer[i] = 0;
				}
				nFrameL = gprs_send_data_packet(fram_gas_alarm_packet,GPRS_CMD_ALARM,seq,mac,NULL,0);
				osqGetOut0x00(fram_gas_alarm_packet_buffer,fram_gas_alarm_packet,nFrameL);
				OSQPost(q_bc95_send_msg,(void *)fram_gas_alarm_packet_buffer);
			}
		}
		delay_ms(3000);
	}
}

////心跳任务
//void heartbeat_tmr_callback(OS_TMR *ptmr,void *p_arg)
//{
//	
//}


//喂狗回调函数
void feeddog_tmr_callback(void)
{
	IWDG_Feed();
	delay_ms(100);
}

//心跳回调函数
void heartbeat_tmr_callback(void)
{
	static u8 heartbeat_packet[14] = {0};
	static u8 heartbeat_packet_send[30] = {0};
	u8 nFrameL = 0;
	//u32 len = 0;
	u8 i;
	if(connetion_state != NET_CONNRTION_ERR)
	{
		if(bc95_send_ok)
		{
			for(i=0;i<30;i++)
			{
				heartbeat_packet_send[i] = 0;
			}
			nFrameL = gprs_send_data_packet(heartbeat_packet,GPRS_CMD_HEART,seq,mac,NULL,0); //数据按协议打包
			osqGetOut0x00(heartbeat_packet_send,heartbeat_packet,nFrameL);					//将0转化0xef 0xe0，使消息队列能正常发送
			OSQPost(q_bc95_send_msg,(void *)heartbeat_packet_send);
		}
	}
	delay_ms(100);
}

//static u8 close_flag = FALSE;
//shell任务
void shell_task(void *pdata)
{	  
	static u8 heartbeat_packet[14] = {0};
	u8 nFrameL = 0;
	//u8 test[50] = {0};
	static u8 test_change[80] = {0};
	u8 i;
	//const static u8 hostMac[4] = {0x22,0x06,0x17,0x40};
	//const static u8 testData[4] = {0};
	//const static u8 testData[42] = {0x0A,0x00,0x01,0x02,0x03,0x04,0x16,0x2F,0xA5,0xD4,0x17,0x8A,0x10,0x02,0x76,0x30,0x00,0x7C,0x76,0x81,0x04,\
									 0x25,0x76,0x85,0x07,0xAB,0x76,0x85,0x07,0xE0,0x76,0x85,0x09,0x96,0x76,0x85,0x09,0xD2,0x76,0x85,0x0A,0x75};
	//u8 test_len;
	u32 len = 0;
	while(1)
	{
		shell_handle();				//shell指令，供调试使用
		delay_ms(300);
		if(singal_flag)
		{
			bc95_get_singal();
			delay_ms(300);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
			bc95_rx_buffer_clean();
			singal_flag = FALSE;
		}
#ifdef USE_BC95
		if(reboot_flag)
		{
			bc95_restar();
			reboot_flag = FALSE;
		}
		else if(nb_state_flag)
		{
			bc95_request_connetion_status();
			delay_ms(500);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
			bc95_rx_buffer_clean();
			nb_state_flag = FALSE;
		}
		else if(band5_flag)
		{
			bc95_band_set(BAND5);
			band5_flag = FALSE;
		}
		else if(band8_flag)
		{
			bc95_band_set(BAND8);
			band8_flag = FALSE;
		}
		else if(band20_flag)
		{
			bc95_band_set(BAND20);
			band20_flag = FALSE;
		}
//		if(test_send_flag)
//		{
//			close_flag = TRUE;
//			osqGetOut0x00(test_change,testData,42);
//			OSQPost(q_bc95_send_msg,(void *)test_change);
//			test_send_flag = FALSE;
//		}
//		if(udp_close_flag)
//		{
//			bc95_close_socket();
//			delay_ms(2000);
//			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
//			bc95_rx_buffer_clean();
//			udp_close_flag = FALSE;
//		}
#endif
		if(test_Flag == TRUE && connetion_state != NET_CONNRTION_ERR)
		{
#ifdef USE_BC95
			//OSQPost(q_bc95_send_msg,"bc95 send test");
			if(bc95_send_ok)
			{
				for(i=0;i<80;i++)
				{
					test_change[i] = 0;
				}
				nFrameL = gprs_send_data_packet(heartbeat_packet,GPRS_CMD_HEART,seq,mac,NULL,0);
				osqGetOut0x00(test_change,heartbeat_packet,nFrameL);
				OSQPost(q_bc95_send_msg,(void *)test_change);
			}
#endif
#ifdef USE_SIM800A
			for(i=0;i<20;i++)
			{
				test[i] = 0;
			}
			test_len = gprs_send_data_packet(test,GPRS_CMD_HEART,0x3E,hostMac,testData,0);
			XPRINT(level_printf_hex,SIM800A_PRINTF_LEVEL,"sim800a test>>",test,test_len);
			osqGetOut0x00(test_change,test,test_len);
			XPRINT(level_printf_int,SIM800A_PRINTF_LEVEL,"sim800a test>>",&test_len,1);
			OSQPost(q_sim800a_send_msg,test_change);
			//OSQPost(q_sim800a_send_msg,"sim800a send test");
#endif
			test_Flag = FALSE;
		}
	}
}


//void main_task(void *pdata)
//{
//	while(1)
//	{
//		
//	}
//	
//}


#ifdef USE_BC95
#define AT_ONE_DIRECTIVE_DELAY_TIME		500			//发送后等待接收完成
#define QUERY_NETWORK_TIME				30			//等待30秒连网时间
#define SECOND_DELAY					1000
//bc95任务
void bc95_task(void *pdata)
{
	static u8 first_star = TRUE;
	u8 err;
	u8 *p = NULL;
	static u8 p_buffer[200] = {0};
	static u8 test_buffer[80] = {0};
	u8 i;
//	u8* presp = NULL;
	u32 len = 0;
//	u8 restaar_count = 0;
	while(1)
	{
		if(connetion_state == NET_CONNRTION_ERR)
		{
			do
			{
				if(first_star == TRUE)
				{
					delay_ms(1000);
					while(bc95_open_scrambing_code() == CONFIGURE_ERR)
					{	
						delay_ms(500);
					}
					first_star = FALSE;
				}
				bc95_restar();
				delay_ms(10000);
				bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
				bc95_rx_buffer_clean();
				delay_ms(5000);
			}
			while(bc95_creat_network_connetion() == NET_CONNRTION_ERR);
			connetion_state = NET_CONNRTION_SUCCEED;
			XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_connetion_succeed,strlen(bc95_connetion_succeed));
		}
		else
		{
			p = OSQPend(q_bc95_send_msg,0,&err);		//调试的消息队列
			bc95_send_ok = FALSE;
			len = osqGetOut0xef(test_buffer,p,strlen((char*)p));
			hex2char(test_buffer,p_buffer,len);
			for(i=0;i<strlen((char*)test_buffer);i++)
			{
				test_buffer[i] = 0;
			}
			for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
			{
				if(bc95_UDP_send((char*)UDP_ipv4_addr,(char*)UDP_local_port,len,p_buffer) == UDP_SEND_SUCCEED)
				{
					connetion_state = UDP_SEND_SUCCEED;
					XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 connetion:","UDP_SEND_SUCCEED",strlen("UDP_SEND_SUCCEED"));
					XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 UDP send",p_buffer,strlen((char*)p_buffer));
					for(i=0;i<strlen((char*)p_buffer);i++)
					{
						p_buffer[i] = 0;
					}
					if(rec_len > 0)
					{
						bc95_UDP_receive_commend(rec_len);
						delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
						XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
//						if()
//						{
//							bc95_rx_buffer_clean();
//							break;
//						}
//						else
//						{
//							bc95_rx_buffer_clean();
//						}
						bc95_rx_buffer_clean();
						break;
					}
					else
					{
						break;
					}
				}
				else
				{
					connetion_state = NET_CONNRTION_ERR;
					XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 connetion:","NET_CONNRTION_ERR",strlen("NET_CONNRTION_ERR"));
					break;
				}
			}
			bc95_send_ok = TRUE;
			delay_ms(500);
//			bc95_close_socket();
//			delay_ms(2000);
//			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
//			bc95_rx_buffer_clean();
		}
	}
	
}


//入网AT指令发送

void bc95_band_set(u8 band)
{
	u32 len;
	u8* presp = NULL;
	u8 i;
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NBAND=5
		if(band == BAND5)
		{
			bc95_set_band_5();
		}
		else if(band == BAND8)
		{
			bc95_set_band_8();
		}
		else if(band == BAND20)
		{
			bc95_set_band_20();
		}
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
	}
}


u8 bc95_open_scrambing_code(void)
{
	u32 len;
	u8* presp = NULL;
	u8 i;
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CFUN=0 
		bc95_set_min_fun();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return CONFIGURE_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NCONFIG?
		bc95_request_UE_behaviour();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return CONFIGURE_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NCONFIG=CR_0354_0338_SCRAMBLING,TRUE
		//AT+NCONFIG=CR_0859_SI_AVOID,TRUE
		bc95_open_UE_behaviour();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return CONFIGURE_ERR;
		}
	}
	return CONFIGURE_SUCCEED;
}

#ifndef BC95_AUTO_CONNECT
u8 bc95_creat_network_connetion(void)
{
	u32 len;
	u8* presp = NULL;
	u8* key_word = NULL;
	u8 i;
	XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_connetion_connet,strlen(bc95_connetion_connet));

	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CFUN=1 
		bc95_set_phone_fun();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		//key_word = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"1"); //+CFUN:1
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if((presp != NULL))
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CIMI   
		bc95_query_IMSI();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NBAND? 
		bc95_request_bands();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CGDCONT=1,"IP","ctnet"" 
		bc95_Configuration_PDP();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_connetion_connet,strlen(bc95_connetion_connet));
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CEREG=1 
		bc95_set_network_reg_status();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CSCON=1
		bc95_set_connetion_status();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+COPS=1,2,"46011"
		bc95_select_PLMN();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
//	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
//	{
//		//AT+CGATT=1  
//		bc95_activate_network();
//		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
//		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
//		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
//		bc95_rx_buffer_clean();
//		if(presp != NULL)
//		{
//			break;
//		}
//		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
//		{
//			return NET_CONNRTION_ERR;
//		}
//	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CSQ  
		bc95_get_singal();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NUESTATS 
		bc95_query_UE_statistics();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
//	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
//	{
//		//AT+CGDCONT=1,”IP”,”HUAWEI.COM” 
//		bc95_Configuration_PDP();
//		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
//		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
//		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
//		bc95_rx_buffer_clean();
//		if(presp != NULL )
//		{
//			break;
//		}
//		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
//		{
//			return NET_CONNRTION_ERR;
//		}
//	}
//	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
//	{
//		//AT+CGATT=1 
//		bc95_activate_network();
//		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
//		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
//		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
//		bc95_rx_buffer_clean();
//		if(presp != NULL)
//		{
//			break;
//		}
//		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
//		{
//			return NET_CONNRTION_ERR;
//		}
//	}
	XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_connetion_connet,strlen(bc95_connetion_connet));
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CGATT? 
		for(i = 0;i < QUERY_NETWORK_TIME;i++)	//延时30秒
		{
			delay_ms(SECOND_DELAY*2);
		}
		bc95_request_activate_network();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		key_word = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"0"); //+CGATT:1
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if((presp != NULL) && (key_word == NULL))
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS*3; i++)
	{
		//AT+CEREG?
		bc95_request_network_reg_status();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME );
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		key_word = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),",1"); //+CEREG:0,1
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if((presp != NULL) && (key_word != NULL))
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CSCON?
		bc95_request_connetion_status();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME );
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		//key_word = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"1"); //+CSCON:0,1
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL) //&& key_word !=NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	return NET_CONNRTION_SUCCEED;
}
#else
u8 bc95_creat_network_connetion(void)
{
	u32 len;
	u8* presp = NULL;
	u8* key_word = NULL;
	u8 i;
	XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_connetion_connet,strlen(bc95_connetion_connet));
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NBAND? 
		bc95_request_bands();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CFUN?
		bc95_request_phone_fun();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CIMI
		bc95_query_IMSI();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CSQ
		bc95_get_singal();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NUESTATS
		bc95_query_UE_statistics();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
		XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_connetion_connet,strlen(bc95_connetion_connet));
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CGATT? 
		for(i = 0;i < QUERY_NETWORK_TIME;i++)	//延时30秒
		{
			delay_ms(SECOND_DELAY);
		}
		bc95_request_activate_network();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		key_word = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"0"); //+CGATT:1
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if((presp != NULL) && (key_word == NULL))
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CEREG?
		bc95_request_network_reg_status();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME * 5);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		key_word = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),",1"); //+CEREG:0,1
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if((presp != NULL) && (key_word != NULL))
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CSCON?
		bc95_request_connetion_status();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME * 5);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		//key_word = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"1"); //+CSCON:0,1
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp != NULL) //&& key_word !=NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	return NET_CONNRTION_SUCCEED;
}
#endif




u8 bc95_UDP_send(char* ip_addr,char* port,u32 length,u8* data)
{
	u32 len;
	u8* presp = NULL;
	u8 i;
	char* bc95_rec_buffer = NULL;
	u8 rec_len_buffer;
	u8 socket = 0;
	if(connetion_state != UDP_SEND_SUCCEED)
	{
		for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
		{
			bc95_creat_UDP_socket(port);
			delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
			presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"ERROR");
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
			bc95_rx_buffer_clean();
			if(presp == NULL)
			{
				presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"1");
				if(presp != NULL)
				{
					socket = 1;
				}
				else
				{
					socket = 0;
				}
				break;
			}
			else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
			{
				XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_UDP_send_err,strlen(bc95_UDP_send_err));
				return UDP_SEND_ERR;
			}
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		seq++;
		bc95_UDP_send_messages(socket,ip_addr,port,length,data);
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME*10);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"ERROR");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rec_buffer = strstr((char*)(bc95_rx_buffer_get(&len)),":0,");
		if(bc95_rec_buffer != NULL)
		{
			rec_len_buffer = strlen(bc95_rec_buffer);
			rec_len_buffer -= strlen(":0,")+2;
			switch(rec_len_buffer)
			{
				case 1: rec_len = bc95_rec_buffer[3] - '0';break;
				case 2: rec_len = (bc95_rec_buffer[3] - '0') * 10 + bc95_rec_buffer[4] - '0';break;
				case 3: rec_len = (bc95_rec_buffer[3] - '0') * 100 + (bc95_rec_buffer[4] - '0') *10 + bc95_rec_buffer[5] - '0';break;
				default: break;
			}
			//XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rec_buffer,rec_len_buffer);
			XPRINT(level_printf_int,BC95_PRINTF_LEVEL,"bc95 receive<<",&rec_len,1);
			for(i = 0;i < 10;i++)
			{
				bc95_rec_buffer[i] = 0;
			}
		}
		else 
		{
			rec_len = 0;
		}
		bc95_rx_buffer_clean();
		if(presp == NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return UDP_SEND_ERR;
		}
	}
	return UDP_SEND_SUCCEED;
	
}
#endif


#ifdef USE_SIM800A
#define AT_ONE_DIRECTIVE_DELAY_TIME		500
#define QUERY_NETWORK_TIME				30
#define SECOND_DELAY					1000


u8 sim800a_creat_network_connetion(void)
{
	u32 len = 0;
	u8* presp = NULL;
	u8* key_word = NULL;
	u8 i;
	//u8 rx_buffer[200];
	//for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	//{
		//AT+CIPCLOSE 
		sim800a_UDP_close();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		key_word = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"1");
		XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
		sim800a_rx_buffer_clean();
//		if(presp != NULL && key_word != NULL)
//		{
//			break;
//		}
//		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
//		{
//			return NET_CONNRTION_ERR;
//		}
	//}
		sim800a_net_close();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		key_word = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"1");
		XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
		sim800a_rx_buffer_clean();
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CGATT? 
		sim800a_check_GPRS_state();
		delay_ms(5*1000);
		presp = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		key_word = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"1");
		XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
		sim800a_rx_buffer_clean();
		if(presp != NULL && key_word != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CSTT=”CMNET”
		sim800a_start_task();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
		sim800a_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CIICR 
		sim800a_bring_up_connection();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
		sim800a_rx_buffer_clean();
		if(presp != NULL )
		{
			break;
		}
		//else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		//{
			//return NET_CONNRTION_ERR;
		//}
	}
	//for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	//{
		//AT+CIFSR
		sim800a_get_ip();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		//presp = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
		sim800a_rx_buffer_clean();
		//if(presp != NULL)
		//{
			//break;
		//}
		//else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		//{
			//return NET_CONNRTION_ERR;
		//}
	//}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+CIPSTART=”UDP,
		sim800a_configure_server_settings((char*)UDP_ipv4_addr,(char*)UDP_local_port);
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
		sim800a_rx_buffer_clean();
		if(presp != NULL)
		{
			break;
		}
		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
		{
			return NET_CONNRTION_ERR;
		}
	}
	return NET_CONNRTION_SUCCEED;	
}


u8 sim800a_UDP_send(u8* data,u8 data_len)
{
//	u32 len = 0;
//	u8* presp = NULL;
//	u8 i;
	//for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	//{
		sim800a_send_data((char*)data,data_len);
		XPRINT(level_printf_hex,SIM800A_PRINTF_LEVEL,"sim800a send data>>",data,data_len);
		//delay_ms(5*1000);
		//sim800a_rec_check((char*)(sim800a_rx_buffer_get(&len)),"OK");
		//XPRINT(level_printf_int,SIM800A_PRINTF_LEVEL,"sim800a receive<<",&len,1);
		//XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<",sim800a_rx_buffer_get(&len),len);
//		if(presp != NULL)
//		{
//			break;
//		}
//		else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
//		{
//			sim800a_rx_buffer_clean();
//			return NET_CONNRTION_ERR;
//		}
//		else
//		{
//			XPRINT(level_printf_char,SIM800A_PRINTF_LEVEL,"sim800a receive<<","error",strlen("error"));
//			sim800a_rx_buffer_clean();
//		}
	//}
	return UDP_SEND_SUCCEED;
}


//GSM模块sim800a任务
void sim800a_task(void *pdata)
{
	static u8 connetion_state = NET_CONNRTION_ERR;
	static u8 first_star = TRUE;
	u8 err;
	u8 *p = NULL;
	u8 i;
	u8 send_data_buf[256] = {0};
	u8 send_data_len = 0;
	while(1)
	{
		if(connetion_state == NET_CONNRTION_ERR)
		{
			do
			{
				if(first_star == TRUE)
				{
					delay_ms(3000);
					first_star = FALSE;
				}
				else
				{
					for(i = 0;i < 30;i++)
					{
						delay_ms(1000);
					}
				}
			}
			while(sim800a_creat_network_connetion() == NET_CONNRTION_ERR);
			connetion_state = NET_CONNRTION_SUCCEED;
		}
		else
		{
			p = OSQPend(q_sim800a_send_msg,0,&err);
			send_data_len = osqGetOut0xef(send_data_buf,p,strlen((char*)p));
			if(sim800a_UDP_send(send_data_buf,send_data_len) == UDP_SEND_SUCCEED)
			{
				connetion_state = UDP_SEND_SUCCEED;
				delay_ms(5000);
				gprs_handle();
			}
			else
			{
				connetion_state = NET_CONNRTION_ERR;
			}
		}
		delay_ms(300);
	}
	
}

#endif

#define MAC_LEN		4	
void QR_printf(void)
{
	static u8 mac_char[MAC_LEN] = {0}; 
	USART1_send("OFFSET 0MM\r\n",strlen("OFFSET 0MM\r\n"));
	USART1_send("SET PEEL OFF\r\n",strlen("SET PEEL OFF\r\n"));
	USART1_send("SET TEAR ON\r\n",strlen("SET TEAR ON\r\n"));
	USART1_send("SIZE 40MM,30MM\n",strlen("SIZE 40MM,30MM\n"));
	USART1_send("CLS\r\n",strlen("CLS\r\n"));
	USART1_send("QRCODE 75,15,L,5,M,2,M1,S2,",strlen("QRCODE 75,15,L,5,M,2,M1,S2,"));
	hex2char(mac,mac_char,MAC_LEN);
	USART1_send((char*)mac_char,MAC_LEN);
	USART1_send("TEXT 85,130,\"TSS24.BF2\",0,1,1,",strlen("TEXT 85,130,\"TSS24.BF2\",0,1,1,"));
	USART1_send((char*)mac_char,MAC_LEN);
	USART1_send("PRINT 1\r\n",strlen("PRINT 1\r\n"));
}




//#define GPRS_TRAN	0xef
//#define GPRS_DSC	0xea
//#define GPRS_SRC	0x1a
//#define GPRS_EB		0xeb
//#define GPRS_DSC1B	0x1b
int gprsCodeGetOut0xla(u8 *pbuf, const u8 *pdata, u16 len)
{
	u32 i = 0;
	u32 j = 0;

	for (i = 0; i < len; i++)
	{
		if (pdata[i] == GPRS_SRC)
		{
			pbuf[j++] = GPRS_TRAN;
			pbuf[j++] = GPRS_DSC;
		}
		else if (pdata[i] == GPRS_TRAN)
		{
			pbuf[j++] = GPRS_TRAN;
			pbuf[j++] = GPRS_TRAN;			
		}
		
		else if (pdata[i] == GPRS_DSC1B)
		{
			pbuf[j++] = GPRS_TRAN;
			pbuf[j++] = GPRS_EB;				
		}

		else
		{
			pbuf[j++] = pdata[i];
		}
	}
	//XPRINTF((6, "CODE 0X1A\n"));
	return j;
}


#define OSQ_SRC		0x00
#define	OSQ_TRAN	0xef
#define OSQ_DSC		0xe0


int osqGetOut0x00(u8* pbuf,const u8* pdata, u32 len)
{
	u32 i = 0;
	u32	j = 0;
	for(i = 0;i < len;i++)
	{
		if(pdata[i] == OSQ_SRC)
		{
			pbuf[j++] = OSQ_TRAN;
			pbuf[j++] = OSQ_DSC;
		}
		else if(pdata[i] == OSQ_TRAN)
		{
			pbuf[j++] = OSQ_TRAN;
			pbuf[j++] = OSQ_TRAN;
		}
		else 
		{
			pbuf[j++] = pdata[i];
		}
	}
	return j;
}




int osqGetOut0xef(u8* pbuf,const u8* pdata, u32 len)
{
	u32 i = 0;
	u32 j = 0;
	for(i = 0;i < len;i++)
	{
		if(pdata[i] == OSQ_TRAN)
		{
			if(pdata[i + 1] == OSQ_DSC)
			{
				pbuf[j++]  = OSQ_SRC;
			}
			else if(pdata[i + 1] == OSQ_TRAN)
			{
				pbuf[j++] = OSQ_TRAN;
			}
			i++;
		}
		else
		{
			pbuf[j++] = pdata[i];
		}
	}
	return j;
}




u32 hex2char(u8* scr_data,u8* obj_data,u32 len)
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



