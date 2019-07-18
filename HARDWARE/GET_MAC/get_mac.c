#include "get_mac.h"
#include <stdlib.h>

void Get_Device_MAC(u8* mac)
{
	
  u8 i = 0;
	/*
  u16 srand_data;

  srand_data = crc16((unsigned char *)0x08007800, 6); 
  mac[i++] = srand_data;
  mac[i++] = srand_data>>8; 
  srand_data = crc16((unsigned char *)0x08007806, 6); 
  mac[i++] = srand_data;
  mac[i++] = srand_data>>8; 


  srand_data = crc16((unsigned char *)0x08007800, 12); 
  srand( (u32)srand_data );
	*/
  mac[i++] = *(unsigned char*)0x800FFF3;
  mac[i++] = *(unsigned char*)0x800FFF2;
  mac[i++] = *(unsigned char*)0x800FFF1;
  mac[i++] = *(unsigned char*)0x800FFF0;
}

