#ifndef _FSM_H
#define _FSM_H

#include "stdint.h"

#define STATUS_MAX (10)

/**
 * @brief 状态所处的阶段
 *
 */
typedef enum
{
    Status_Stage_DISABLE = 0,
    Status_Stage_ENABLE,
} Enum_Status_Stage;

/**
 * @brief 状态结构体
 *
 */
typedef struct
{
    Enum_Status_Stage Status_Stage;
    uint32_t Time;
} Struct_Status;

typedef struct
{
    Struct_Status Status[STATUS_MAX];
    // 状态数量
    uint8_t Status_Num;
    // FSM当前状态
    uint8_t Now_Status_Serial ;
} Struct_FSM;

void FSM_Init(Struct_FSM *FSM, uint8_t Status_Num, uint8_t Now_Status_Serial);
void Set_FSM_Status(Struct_FSM *FSM, uint8_t Next_Status_serial);
uint8_t Get_Now_Status_Serial(Struct_FSM *FSM);
#endif
