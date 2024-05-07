#include "fsm.h"

void FSM_Init(Struct_FSM *FSM, uint8_t Status_Num, uint8_t Now_Status_Serial)
{
    FSM->Status_Num = Status_Num;
    FSM->Now_Status_Serial = 0;
    // 所有状态全刷0
    for (uint8_t i = 0; i < Status_Num; i++)
    {
        FSM->Status[i].Status_Stage = Status_Stage_DISABLE;
        FSM->Status[i].Time = 0;
    }
    // 使能初始状态
    FSM->Status[Now_Status_Serial].Status_Stage = Status_Stage_ENABLE;
}

void Set_FSM_Status(Struct_FSM *FSM, uint8_t Next_Status_serial)
{
    // 失能当前状态, 计时器清零
    FSM->Status[FSM->Now_Status_Serial].Status_Stage = Status_Stage_DISABLE;
    FSM->Status[FSM->Now_Status_Serial].Time = 0;
    // 使能下一个状态
    FSM->Status[Next_Status_serial].Status_Stage = Status_Stage_ENABLE;
    FSM->Now_Status_Serial = Next_Status_serial;
}

inline uint8_t Get_Now_Status_Serial(Struct_FSM *FSM)
{
    return FSM->Now_Status_Serial;
}

// 定义状态
typedef enum
{
    STATE_A,
    STATE_B,
    STATE_C,
    // 添加更多状态...
    STATE_NUM
} State_example;

void FSM_example(Struct_FSM *FSM)
{

    FSM_Init(FSM, 3, 0);

    // task loop 例如周期为1ms
    FSM->Status[FSM->Now_Status_Serial].Time++;
    // 自己列一个状态转移图，然后根据状态转移图写状态机
    // 根据状态转移图写状态机
    switch (Get_Now_Status_Serial(FSM))
    {
    case STATE_A:
        // 处理状态A
        // 如果满足转换条件，转换到其他状态
        // Set_FSM_Status(&FSM, STATE_B)
        break;
    case STATE_B:
        // 处理状态B
        // 如果满足转换条件，转换到其他状态
        // Set_FSM_Status(&FSM, STATE_C)
        break;
    case STATE_C:
        // 处理状态C
        // 如果满足转换条件，转换到其他状态
        // Set_FSM_Status(&FSM, STATE_A)
        break;
        // 添加更多状态...
    }
}