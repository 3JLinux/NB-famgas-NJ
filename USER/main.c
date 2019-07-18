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

#ifndef USE_COAP
//const char UDP_local_port[] = "4569";			//端口
const char UDP_local_port[] = "4568";			//端口 
//const char UDP_ipv4_addr[] = "119.29.224.28";	//测试服务器
const char UDP_ipv4_addr[] = "119.29.155.148";	//正式服务器
#else
//const char UDP_local_port[] = "5683";
//const char UDP_ipv4_addr[] = "180.101.147.115";//测试

const char UDP_local_port[] = "5683";
const char UDP_ipv4_addr[] = "117.60.157.137";//测试
#endif

#define fram_gas_485_rx_buffer_clean	usart2_rx_buffer_clean
#define fram_gas_485_rx_buffer_get		usart2_rx_buffer_get
#define fram_gas_485_send 				USART2_send

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
static u8 change2tvl(u8* src,u8* dst);

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


//485任务
//设置任务优先级
#define FRAM_GSA_RS_485_TASK_PRIO       			6 
//设置任务堆栈大小
#define FRAM_GSA_RS485_STK_SIZE  					512
//任务堆栈
OS_STK FRAM_GSA_RS485_TASK_STK[FRAM_GSA_RS485_STK_SIZE];
//任务函数
void fram_gas_rs485_task(void *pdata);

//485任务
//设置任务优先级
#define FRAM_GSA_RS_485_REC_TASK_PRIO       			7 
//设置任务堆栈大小
#define FRAM_GSA_RS485_REC_STK_SIZE  					512
//任务堆栈
OS_STK FRAM_GSA_RS485_REC_TASK_STK[FRAM_GSA_RS485_REC_STK_SIZE];
//任务函数
void fram_gas_rs485_rec_tack(void *pdata);


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
void * MsgGrp[350];			//消息队列存储地址,最大支持256个消息
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
void * MsgGrp[350];			//消息队列存储地址,最大支持256个消息
#endif

OS_TMR	*heartbeat_tmr;
OS_TMR  *feeddog_tmr;
//static void heartbeat_tmr_callback(void);
static void feeddog_tmr_callback(void);

static u8 connetion_state = NET_CONNRTION_ERR;  //网络连接状态
static u16 seq = 0;
static u8 mac[4] = {0x79,0xC1,0x11,0x11};
//static u8 mac[4] = {0};

 int main(void)
 {
	u8* presp = NULL;
	char* str = NULL;
	u32 len = 0;
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	LED_Init();		  	//初始化与LED连接的硬件接口
	fram_gas_init();
	uart_init(9600);
	//Get_Device_MAC(mac);
	uart2_init(4800);
	uart3_init(9600);
	delay_ms(2000);
	while(1)
	{		
		bc95_request_IMEI();
		delay_ms(500);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
	
		str = strstr((char*)bc95_rx_buffer_get(&len),"CGSN");
		if(str!=NULL)
		{
			memcpy(COAP_IMEI,str+5,15);
			COAP_IMEI[15] = '0';
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_IMEI:",COAP_IMEI,16);
			bc95_rx_buffer_clean();
			break;
		}
		bc95_rx_buffer_clean();
	}
		
	QR_printf();
	delay_ms(2000);
	uart_init(921600);
	//IWDG_Init(4,625*10);
	POWER_LED = 1;
	SINGAL_LED = 0;
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
	//OSTaskCreate(fram_gas_task,(void *)0,(OS_STK*)&FRAM_GSA_TASK_STK[FRAM_GSA_STK_SIZE-1],FRAM_GSA_TASK_PRIO);
#endif
#ifdef	USE_SIM800A
	OSTaskCreate(sim800a_task,(void *)0,(OS_STK*)&SIM800A_TASK_STK[SIM800A_STK_SIZE-1],SIM800A_TASK_PRIO);
#endif
	OSTaskCreate(fram_gas_rs485_task,(void *)0,(OS_STK*)&FRAM_GSA_RS485_TASK_STK[FRAM_GSA_RS485_STK_SIZE-1],FRAM_GSA_RS_485_TASK_PRIO);
	OSTaskCreate(fram_gas_rs485_rec_tack,(void *)0,(OS_STK*)&FRAM_GSA_RS485_REC_TASK_STK[FRAM_GSA_RS485_REC_STK_SIZE-1],FRAM_GSA_RS_485_REC_TASK_PRIO);
	
	//heartbeat_tmr = OSTmrCreate((10*60*100),(10*60*100),OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)heartbeat_tmr_callback,0,(INT8U*)"heartbeat_tmr",&err); //心跳时间软件定时器
	//feeddog_tmr = OSTmrCreate((100),(100),OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)feeddog_tmr_callback,0,(INT8U*)"feeddog_tmr",&err); //喂狗软件定时器
	//OSTmrStart(heartbeat_tmr,&err);
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

/*
//报警任务
void fram_gas_task(void *pdata)
{
	u8 nFrameL = 0;
	static u8 fram_gas_alarm_packet[14] = {0};
	static u8 fram_gas_alarm_packet_buffer[28] = {0};
	static u8 fram_gas_alarm_count = 0;
	u8 i;
	while(1)
	{
		if(connetion_state != NET_CONNRTION_ERR && FRAM_GAS_ALARM)
		{
			if(bc95_send_ok && (fram_gas_alarm_count < 3))
			{
				for(i=0;i<28;i++)
				{
					fram_gas_alarm_packet_buffer[i] = 0;
				}
				nFrameL = gprs_send_data_packet(fram_gas_alarm_packet,GPRS_CMD_ALARM,seq,mac,NULL,0);
				osqGetOut0x00(fram_gas_alarm_packet_buffer,fram_gas_alarm_packet,nFrameL);
				OSQPost(q_bc95_send_msg,(void *)fram_gas_alarm_packet_buffer);
				fram_gas_alarm_count++;
			}
		}
		if(!FRAM_GAS_ALARM)
		{
			fram_gas_alarm_count = 0;
		}
		delay_ms(500);
	}
}
*/

#define SLAVE_ADDR		0x01
#define RS485_READ_CMD	0x03
static u8 star_addr[2] = {0x00,0x08};
static u8 star_num[2] = {0x00,0x05};
static u8 sec_addr[2] = {0x00,0x10};
static u8 sec_num[2] = {0x00,0x02};
static u8 third_addr[2] = {0x00,0x13};
static u8 third_num[2] = {0x00,0x02};

static u8 fram_gas_485_send_flag = 0;
void fram_gas_rs485_task(void *pdata)
{
	static u8 fram_gas_485_packet[30] = {0};
	u8 len = 0;
	static u8 first_star = 1;
	u8 i;
	while(1)
	{
		if(first_star)
		{
			for(i=0;i<100;i++)
			{
				delay_ms(1000);
			}
		}
		first_star = 0;
		len = fram_gas_read_packet(fram_gas_485_packet,SLAVE_ADDR,RS485_READ_CMD,star_addr,star_num);
		fram_gas_485_send((char*)fram_gas_485_packet,len);
		fram_gas_485_send_flag = 1;
		delay_ms(2000);
		len = fram_gas_read_packet(fram_gas_485_packet,SLAVE_ADDR,RS485_READ_CMD,sec_addr,sec_num);
		fram_gas_485_send((char*)fram_gas_485_packet,len);
		fram_gas_485_send_flag = 2;
		delay_ms(2000);
		len = fram_gas_read_packet(fram_gas_485_packet,SLAVE_ADDR,RS485_READ_CMD,third_addr,third_num);
		fram_gas_485_send((char*)fram_gas_485_packet,len);
		fram_gas_485_send_flag = 3;
		delay_ms(2000);
	}
}

static u8 bc95_send_lock = 0;

#define RS_485_REC_MIN_NUM		7
#define RS_485_REC_MAX_NUM		255
static u8 rs485_rec_data[20] = {0};
void fram_gas_rs485_rec_tack(void *pdata)
{
	static u32 len = 0;
	static u8 pren[50] = {0};
	u8 i;
	u32 nFrameL = 0;
	static u8 rs485_packet[350] = {0};
	static u8 rs485_packet_send[350] = {0};
	static u8 net_sent_flag = 0;
	static u32 net_send_count = 10 * 10;
	static u8 net_alarm_count = 10 *10;
	static u8 net_alarm_flag = 0;
	static u8 tlv_packet[50]={0};
	u8 tlv_len = 0;
	while(1)
	{
		memcpy(pren,fram_gas_485_rx_buffer_get(&len),len);
		if(len >= RS_485_REC_MIN_NUM && len <= RS_485_REC_MAX_NUM)
		{
			for(i=0;i<len;i++)
			{
				if(pren[i] == SLAVE_ADDR && pren[i + 1] == RS485_READ_CMD)
				{
					if(pren[i + 2] <= (len - 2))
					{
						SINGAL_LED = ~SINGAL_LED;
						rs485_rec_data[0] = 1;
						rs485_rec_data[1] = 18;
						switch(fram_gas_485_send_flag)
						{
							case 1:memcpy((rs485_rec_data + 2),(pren + i + 3),10);break;
							case 2:memcpy((rs485_rec_data + 12),(pren + i + 3),4);break;
							case 3:memcpy((rs485_rec_data + 16),(pren + i + 3),4);net_sent_flag = 1;break;
							default:break;
						}
						for(i=0;i<len;i++)
						{
							pren[i] = 0;
						}
						fram_gas_485_rx_buffer_clean();
						fram_gas_485_send_flag = 0;
					}
					//XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"RS485 REC:",rs485_rec_data,20);
				}
			}
		}
		else if(len > RS_485_REC_MAX_NUM)
		{
			fram_gas_485_rx_buffer_clean();
		}
		if(net_sent_flag)
		{
			net_sent_flag = 0;
			net_send_count++;
			/*
			if(net_send_count>=10)
			{
				net_send_count = 0;
				bc95_send_lock = 0;
			}
			*/
			if(net_alarm_flag)
			{
				net_alarm_count++;
			}
			if((connetion_state != NET_CONNRTION_ERR) && (rs485_rec_data[13] != 0) && (net_alarm_count>=10*10))
			{
				net_alarm_flag = 1;
				net_alarm_count = 0;
				for(i=0;i<50;i++)
				{
					rs485_packet[i] = 0;
					rs485_packet_send[i] = 0;
				}
#ifndef USE_COAP
				nFrameL = gprs_send_data_packet(rs485_packet,GPRS_CMD_DATA,seq,mac,rs485_rec_data,20); //数据按协议打包
				osqGetOut0x00(rs485_packet_send,rs485_packet,nFrameL);					//将0转化0xef 0xe0，使消息队列能正常发送
				XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"RS485 SEND:",rs485_packet_send,strlen((char*)rs485_packet_send));
				OSQPost(q_bc95_send_msg,(void *)rs485_packet_send);
#else
				tlv_len = change2tvl(rs485_rec_data+2,tlv_packet);
				nFrameL = gprs_coap_send_data_packet(rs485_packet,COAP_TYPE,COAP_IMEI,COAP_IMSI,COAP_POWERVALUE,COAP_RSSI,\
				COAP_EARFCN,COAP_ECL,COAP_SNR,COAP_RSRP,COAP_PCI,tlv_packet,tlv_len);
				osqGetOut0x00(rs485_packet_send,rs485_packet,nFrameL);
				XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"RS485 SEND:",rs485_packet_send,strlen((char*)rs485_packet_send));
				OSQPost(q_bc95_send_msg,(void *)rs485_packet_send);
#endif
			}
			if(rs485_rec_data[13] == 0)
			{
				net_alarm_count = 10*10;
				net_alarm_flag = 0;
			}
			if((connetion_state != NET_CONNRTION_ERR)&&(net_send_count>=10*10) && (bc95_send_lock == 0))
			{
				net_send_count = 0;
				for(i=0;i<50;i++)
				{
					rs485_packet[i] = 0;
					rs485_packet_send[i] = 0;
				}
#ifndef USE_COAP
				nFrameL = gprs_send_data_packet(rs485_packet,GPRS_CMD_DATA,seq,mac,rs485_rec_data,20); //数据按协议打包
				osqGetOut0x00(rs485_packet_send,rs485_packet,nFrameL);					//将0转化0xef 0xe0，使消息队列能正常发送
				XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"RS485 SEND:",rs485_packet_send,strlen((char*)rs485_packet_send));
				OSQPost(q_bc95_send_msg,(void *)rs485_packet_send);
#else
				tlv_len = change2tvl(rs485_rec_data+2,tlv_packet);
				nFrameL = gprs_coap_send_data_packet(rs485_packet,COAP_TYPE,COAP_IMEI,COAP_IMSI,COAP_POWERVALUE,COAP_RSSI,\
				COAP_EARFCN,COAP_ECL,COAP_SNR,COAP_RSRP,COAP_PCI,tlv_packet,tlv_len);
				osqGetOut0x00(rs485_packet_send,rs485_packet,nFrameL);
				XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"RS485 SEND:",rs485_packet_send,strlen((char*)rs485_packet_send));
				OSQPost(q_bc95_send_msg,(void *)rs485_packet_send);
#endif
			}
		}
		delay_ms(600);
	}
}



//喂狗回调函数
void feeddog_tmr_callback(void)
{
	IWDG_Feed();
	delay_ms(100);
}

/*
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
*/

//static u8 close_flag = FALSE;
//shell任务
void shell_task(void *pdata)
{	  
	static u8 heartbeat_packet[100] = {0};
	u8 nFrameL = 0;
	//u8 test[50] = {0};
	static u8 test_change[200] = {0};
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
#ifndef USE_COAP
				nFrameL = gprs_send_data_packet(heartbeat_packet,GPRS_CMD_HEART,seq,mac,NULL,0);
				osqGetOut0x00(test_change,heartbeat_packet,nFrameL);
				OSQPost(q_bc95_send_msg,(void *)test_change);
#else
				nFrameL = gprs_coap_send_data_packet(heartbeat_packet,COAP_TYPE,COAP_IMEI,COAP_IMSI,COAP_POWERVALUE,COAP_RSSI,\
				COAP_EARFCN,COAP_ECL,COAP_SNR,COAP_RSRP,COAP_PCI,rs485_rec_data,20);
				osqGetOut0x00(test_change,heartbeat_packet,nFrameL);
				XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"RS485 SEND:",heartbeat_packet,strlen((char*)heartbeat_packet));
				OSQPost(q_bc95_send_msg,(void *)test_change);
#endif
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
	static u8 p_buffer[350] = {0};
	static u8 test_buffer[350] = {0};
	u32 i;
//	u8* presp = NULL;
	u32 len = 0;
//	u8 restaar_count = 0;
	while(1)
	{
		if(connetion_state == NET_CONNRTION_ERR)
		{
			POWER_LED = 1;
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
			XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"q_bc95_send_msg",p,strlen((char*)p));
			POWER_LED = 0;
			bc95_send_lock = 1;
			bc95_send_ok = FALSE;
#ifndef USE_COAP
			len = osqGetOut0xef(test_buffer,p,strlen((char*)p));
			hex2char(test_buffer,p_buffer,len);
#else
			len = osqGetOut0xef(test_buffer,p,strlen((char*)p));
			//hex2char(test_buffer,p_buffer,len);
			memcpy(p_buffer,test_buffer,len);
			len/=2;
#endif
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
						bc95_send_lock = 0;
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
					bc95_send_lock = 0;
					connetion_state = NET_CONNRTION_ERR;
					XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 connetion:","NET_CONNRTION_ERR",strlen("NET_CONNRTION_ERR"));
					XPRINT(level_printf_hex,BC95_PRINTF_LEVEL,"bc95 UDP send",p_buffer,strlen((char*)p_buffer));
					break;
				}
			}
			bc95_send_lock = 0;
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
	//for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		// 
		bc95_request_IMEI();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
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
	char* str = NULL;
	u8 i,j;
	u8 str_len = 0;
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
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME*2);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		
		memcpy(COAP_IMSI,bc95_rx_buffer_get(&len) + 2,15);
		COAP_IMSI[15] = '0';
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_IMSI:",COAP_IMSI,16);
		
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
		/*
		str = strstr((char*)bc95_rx_buffer_get(&len),"CSQ");
		if(str!=NULL)
		{
			str_len = 0;
			while(str[4+str_len] != ',')
			{
				str_len++;
			}
			stringtoint(COAP_RSSI,str+4,str_len,0);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSSI:",str+4,str_len);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSSI:",COAP_RSSI,4);
		}
*/		
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
		bc95_request_IMEI();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
	
		str = strstr((char*)bc95_rx_buffer_get(&len),"CGSN");
		if(str!=NULL)
		{
			memcpy(COAP_IMEI,str+5,15);
			COAP_IMEI[15] = '0';
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_IMEI:",COAP_IMEI,16);
		}
	
		bc95_rx_buffer_clean();
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		//AT+NUESTATS
		bc95_query_UE_statistics();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		
		str = strstr((char*)bc95_rx_buffer_get(&len),"EARFCN");
		if(str!=NULL)
		{
			str_len = 0;
			while(str[7+str_len] != '\r')
			{
				str_len++;
			}
			stringtoint(COAP_EARFCN,str+7,str_len,0);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_EARFCN:",str+7,str_len);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_EARFCN:",COAP_EARFCN,4);
		}
		
		str = strstr((char*)bc95_rx_buffer_get(&len),"ECL");
		if(str!=NULL)
		{
			str_len = 0;
			while(str[4+str_len] != '\r')
			{
				str_len++;
			}
			stringtoint(COAP_ECL,str+4,str_len,0);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_ECL:",str+4,str_len);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_ECL:",COAP_ECL,4);
		}
		
		str = strstr((char*)bc95_rx_buffer_get(&len),"SNR");
		if(str!=NULL)
		{
			str_len = 0;
			while(str[4+str_len] != '\r')
			{
				str_len++;
			}
			stringtoint(COAP_SNR,str+4,str_len,0);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_SNR:",str+4,str_len);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_SNR:",COAP_SNR,4);
		}
		
		str = strstr((char*)bc95_rx_buffer_get(&len),"RSRQ");
		if(str!=NULL)
		{
			str_len = 0;
			while(str[5+str_len] != '\r')
			{
				str_len++;
			}
			stringtoint(COAP_RSRP,str+6,str_len-1,1);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRQ:",str+5,str_len);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRQ:",COAP_RSRP,4);
		}
		
		
		str = strstr((char*)bc95_rx_buffer_get(&len),"PCI");
		if(str!=NULL)
		{
			str_len = 0;
			while(str[4+str_len] != '\r')
			{
				str_len++;
			}
			stringtoint(COAP_PCI,str+4,str_len,0);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_PCI:",str+4,str_len);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_PCI:",COAP_PCI,4);
		}
		
		
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
		//AT+NUESTATS=CELL
		bc95_query_UE_statistics_CELL();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		
		str = strstr((char*)bc95_rx_buffer_get(&len),"CELL,");
		//str+=5;
		if(str!=NULL)
		{
			str+=5;
			for(j=0;j<4;j++)
			{
				str_len = 0;
				while(str[str_len] != ',')
				{
					str_len++;
				}
				str+=str_len+1;
			}
			stringtoint(COAP_RSRP,str-str_len,str_len-1,1);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRP:",str-str_len,str_len-1);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRP:",COAP_RSRP,4);
			
			for(j=0;j<2;j++)
			{
				str_len = 0;
				while(str[str_len] != ',')
				{
					str_len++;
				}
				str+=str_len+1;
			}
			stringtoint(COAP_RSSI,str-str_len,str_len-2,1);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSSI:",str-str_len,str_len-2);
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSSI:",COAP_RSSI,4);
		}
		
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
	u8 i,j;
	char* bc95_rec_buffer = NULL;
	u32 rec_len_buffer;
#ifndef USE_COAP
	u8 socket = 0;
#else
	char* str = NULL;
	u8 str_len = 0;
#endif
	if(connetion_state != UDP_SEND_SUCCEED)
	{
		
#ifdef USE_COAP
		for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
		{
			//AT+NUESTATS
			bc95_query_UE_statistics();
			delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
			presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
			
			str = strstr((char*)bc95_rx_buffer_get(&len),"EARFCN");
			if(str!=NULL)
			{
				str_len = 0;
				while(str[7+str_len] != '\r')
				{
					str_len++;
				}
				stringtoint(COAP_EARFCN,str+7,str_len,0);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_EARFCN:",str+7,str_len);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_EARFCN:",COAP_EARFCN,4);
			}
			
			str = strstr((char*)bc95_rx_buffer_get(&len),"ECL");
			if(str!=NULL)
			{
				str_len = 0;
				while(str[4+str_len] != '\r')
				{
					str_len++;
				}
				stringtoint(COAP_ECL,str+4,str_len,0);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_ECL:",str+4,str_len);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_ECL:",COAP_ECL,4);
			}
			
			str = strstr((char*)bc95_rx_buffer_get(&len),"SNR");
			if(str!=NULL)
			{
				str_len = 0;
				while(str[4+str_len] != '\r')
				{
					str_len++;
				}
				stringtoint(COAP_SNR,str+4,str_len,0);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_SNR:",str+4,str_len);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_SNR:",COAP_SNR,4);
			}
			
			str = strstr((char*)bc95_rx_buffer_get(&len),"RSRQ");
			if(str!=NULL)
			{
				str_len = 0;
				while(str[5+str_len] != '\r')
				{
					str_len++;
				}
				stringtoint(COAP_RSRP,str+6,str_len-1,1);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRQ:",str+5,str_len);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRQ:",COAP_RSRP,4);
			}
			
			
			str = strstr((char*)bc95_rx_buffer_get(&len),"PCI");
			if(str!=NULL)
			{
				str_len = 0;
				while(str[4+str_len] != '\r')
				{
					str_len++;
				}
				stringtoint(COAP_PCI,str+4,str_len,0);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_PCI:",str+4,str_len);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_PCI:",COAP_PCI,4);
			}
			
			
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
			//AT+NUESTATS=CELL
			bc95_query_UE_statistics_CELL();
			delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
			presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
			
			str = strstr((char*)bc95_rx_buffer_get(&len),"CELL,");
			//str+=5;
			if(str!=NULL)
			{
				str+=5;
				for(j=0;j<4;j++)
				{
					str_len = 0;
					while(str[str_len] != ',')
					{
						str_len++;
					}
					str+=str_len+1;
				}
				stringtoint(COAP_RSRP,str-str_len,str_len-1,1);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRP:",str-str_len,str_len-1);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSRP:",COAP_RSRP,4);
				
				for(j=0;j<2;j++)
				{
					str_len = 0;
					while(str[str_len] != ',')
					{
						str_len++;
					}
					str+=str_len+1;
				}
				stringtoint(COAP_RSSI,str-str_len,str_len-2,1);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSSI:",str-str_len,str_len-2);
				XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"COAP_RSSI:",COAP_RSSI,4);
			}
			
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
#endif
		for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
		{
#ifndef USE_COAP
			bc95_creat_UDP_socket(port);
#else
			bc95_creat_COAP_socket(ip_addr,port);
#endif
			delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
			presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"ERROR");
			XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
			bc95_rx_buffer_clean();
			if(presp == NULL)
			{
#ifndef USE_COAP
				presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"1");
				if(presp != NULL)
				{
					socket = 1;
				}
				else
				{
					socket = 0;
				}
#endif
				break;
			}
			else if(i == ATCMD_MAX_REPEAT_NUMS - 1)
			{
				XPRINT(level_printf_char,DEFAULT_PRINTF_LEVEL,"bc95 connetion:",(void*)bc95_UDP_send_err,strlen(bc95_UDP_send_err));
				return UDP_SEND_ERR;
			}
		}
	}
#ifdef USE_COAP
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		bc95_request_CDP_server();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"+NCDP");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp!=NULL)
		{
			break;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		bc95_NSMI();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp!=NULL)
		{
			break;
		}
	}
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		bc95_NNMI();
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME);
		presp = bc95_rec_check((char*)(bc95_rx_buffer_get(&len)),"OK");
		XPRINT(level_printf_char,BC95_PRINTF_LEVEL,"bc95 receive<<",bc95_rx_buffer_get(&len),len);
		bc95_rx_buffer_clean();
		if(presp!=NULL)
		{
			break;
		}
	}
#endif
	for (i = 0; i < ATCMD_MAX_REPEAT_NUMS; i++)
	{
		seq++;
#ifndef USE_COAP
		bc95_UDP_send_messages(socket,ip_addr,port,length,data);
#else
		bc95_COAP_send_messages(length,data);
#endif
		delay_ms(AT_ONE_DIRECTIVE_DELAY_TIME*6);
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
	USART1_send("OFFSET 3MM\r\n",strlen("OFFSET 3MM\r\n"));
	USART1_send("SET PEEL OFF\r\n",strlen("SET PEEL OFF\r\n"));
	USART1_send("SET TEAR ON\r\n",strlen("SET TEAR ON\r\n"));
	USART1_send("SIZE 30MM,20MM\n",strlen("SIZE 30MM,20MM\n"));
	USART1_send("CLS\r\n",strlen("CLS\r\n"));
	USART1_send("QRCODE 75,15,L,5,M,2,M1,S2,",strlen("QRCODE 75,15,L,5,M,2,M1,S2,"));
	//memcpy(COAP_IMEI)
	//hex2char(mac,mac_char,MAC_LEN);
	USART1_send("\"",strlen("\""));
	USART1_send("R",strlen("R"));
	USART1_send((char*)COAP_IMEI,15);
	USART1_send("P",strlen("P"));
	USART1_send("\"",strlen("\""));
	USART1_send("\r\n",strlen("\r\n"));
	USART1_send("TEXT 85,130,\"TSS24.BF2\",0,1,1,",strlen("TEXT 85,130,\"TSS24.BF2\",0,1,1,"));
	USART1_send("\"",strlen("\""));
	USART1_send("R",strlen("R"));
	USART1_send((char*)mac_char,8);
	USART1_send("G",strlen("G"));
	USART1_send("\"",strlen("\""));
	USART1_send("\r\n",strlen("\r\n"));
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


const static u8 REGISTER1[2] = {0xA0,0x00};
const static u8 REGISTER2[2] = {0xA0,0x01};
const static u8 REGISTER3[2] = {0xA0,0x02};
const static u8 REGISTER4[2] = {0xA0,0x03};
const static u8 REGISTER5[2] = {0xA0,0x04};
const static u8 REGISTER6[2] = {0xA0,0x05};
const static u8 REGISTER7[2] = {0xA0,0x06};
const static u8 REGISTER8[2] = {0xA0,0x07};
const static u8 REGISTER9[2] = {0xA0,0x08};
const static u8 TLV_LLEN[2] = {0x00,0x02};
static u8 change2tvl(u8* src,u8* dst)
{
	
	u8 i;
	u8 len =0;
	for(i=0;i<9;i++)
	{
		switch(i)
		{
			case 0:memcpy(dst+i*6,REGISTER1,2);len+=2;break;
			case 1:memcpy(dst+i*6,REGISTER2,2);len+=2;break;
			case 2:memcpy(dst+i*6,REGISTER3,2);len+=2;break;
			case 3:memcpy(dst+i*6,REGISTER4,2);len+=2;break;
			case 4:memcpy(dst+i*6,REGISTER5,2);len+=2;break;
			case 5:memcpy(dst+i*6,REGISTER6,2);len+=2;break;
			case 6:memcpy(dst+i*6,REGISTER7,2);len+=2;break;
			case 7:memcpy(dst+i*6,REGISTER8,2);len+=2;break;
			case 8:memcpy(dst+i*6,REGISTER9,2);len+=2;break;
			default:break;
		}
		memcpy(dst+i*6+2,TLV_LLEN,2);
		len+=2;
		memcpy(dst+i*6+4,src+i*2,2);
		len+=2;
	}
//	memcpy(dst,REGISTER1,2);
//	memcpy(dst+2,TLV_LLEN,2);
//	memcpy(dst+4,src+2,18);
	return len;
}

