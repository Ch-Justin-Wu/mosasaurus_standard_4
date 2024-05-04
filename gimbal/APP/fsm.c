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

void FSM_example(void)
{
    Struct_FSM FSM;
    FSM_Init(&FSM, 3, 0);
    // 状态机状态数量
    uint8_t Status_Num = 10;
    // 状态机当前状态
    uint8_t Now_Status_Serial = 0;
    // 状态机状态
    Struct_Status Status[STATUS_MAX];
    // 状态机状态所处的阶段
    Enum_Status_Stage Status_Stage;
    // 状态机状态时间
    uint32_t Time;

    // task loop 例如周期为1ms
    //自己列一个状态转移图，然后根据状态转移图写状态机
    {
        FSM.Status[Now_Status_Serial].Time++;
        switch (FSM.Status[Now_Status_Serial].Status_Stage)
        {
        case 0:
            // do something
            FSM.Status[Now_Status_Serial].Status_Stage = 1;
            break;
        case 1:
            // do something
            FSM.Status[Now_Status_Serial].Status_Stage = 2;
            break;
        case 2:
            // do something
            FSM.Status[Now_Status_Serial].Status_Stage = 0;
            break;
        }
    }
}