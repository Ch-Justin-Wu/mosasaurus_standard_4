#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "usart.h"
#include "bsp_can.h"
#include "offline_check.h"
//extern uint16_t supercap_volt;
//extern uint16_t supercap_per;
//void supercap(void);
//void get_supercap_data(void);

#define Low_Voltage_Mode 		0
#define Medium_Voltage_Mode		1
#define High_Voltage_Mode 		2

#define	Supercap_Connected		1
#define Supercap_Disconnected	0



typedef struct
{
    float max_power;
    float actual_power;
    uint16_t power_buffer;

    float voltage;
    float percent;

    uint8_t rx_buff[8];
    uint8_t tx_buff[8];

    offline_check_handle offline_check;
}supercap_handle;

/***********************************************************
*@Brief	供外部调用的变量
***********************************************************/
extern supercap_handle supercap_object;
extern float supercap_volt;
extern float supercap_per;
extern int Supercap_Connection_Status;
/***********************************************************
*@Brief	供外部调用的函数
***********************************************************/
void supercap(int S_Cnt, int MS_Cnt);
#endif

