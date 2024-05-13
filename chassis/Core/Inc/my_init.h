#ifndef __MY_INIT_H
#define __MY_INIT_H

#include "pid.h"
#include "chassis_move.h"
#include "tim.h"
#include "remote_control.h"
#include "bsp_referee.h"

#include "lqr.h"


#include "delay.h"
#include "led.h"
#include "bsp_uart.h"

void all_init(void);
void mode_init(void);
#endif

