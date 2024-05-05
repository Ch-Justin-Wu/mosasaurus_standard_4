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

// 定义状态
typedef enum
{
    STATE_A,
    STATE_B,
    STATE_C,
    // 添加更多状态...
    STATE_NUM
} State_example;

void FSM_example(void)
{
    Struct_FSM FSM;
    FSM_Init(&FSM, 3, 0);

    // task loop 例如周期为1ms
    // 自己列一个状态转移图，然后根据状态转移图写状态机
    // 根据状态转移图写状态机
    switch (FSM.Now_Status_Serial)
    {
    case STATE_A:
        // 处理状态A
        // 如果满足转换条件，转换到其他状态
        // FSM->Now_Status_Serial = STATE_B;
        break;
    case STATE_B:
        // 处理状态B
        // 如果满足转换条件，转换到其他状态
        // FSM->Now_Status_Serial = STATE_C;
        break;
    case STATE_C:
        // 处理状态C
        // 如果满足转换条件，转换到其他状态
        // FSM->Now_Status_Serial = STATE_A;
        break;
        // 添加更多状态...
    }
}