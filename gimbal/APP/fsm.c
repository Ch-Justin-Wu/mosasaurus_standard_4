#include "fsm.h"

void FSM_Init(Struct_FSM *FSM,uint8_t Status_Num,uint8_t Now_Status_Serial)
{
    FSM->Status_Num = Status_Num;
    FSM->Now_Status_Serial = 0;
    // 所有状态全刷0
    for (uint8_t i = 0; i < Status_Num; i++)
    {
        FSM->Status[i].Status_Stage = Status_Stage_DISABLE;
        FSM->Status[i].Time = 0;
    }
     //使能初始状态
    FSM->Status[Now_Status_Serial].Status_Stage = Status_Stage_ENABLE;
}


