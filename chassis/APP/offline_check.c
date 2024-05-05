#include "offline_check.h"

online_status_enum offline_check(offline_check_handle *ptr, uint32_t offline_trigger_cnt)
{
    if (ptr->online_cnt == ptr->last_online_cnt)
    {
        ptr->offline_cnt++;
    }
    else
    {
        ptr->offline_cnt = 0;
    }
    if (ptr->offline_cnt > offline_trigger_cnt)
    {
        ptr->online_cnt = 0;
        ptr->last_online_cnt = 0;
        ptr->status = OFFLINE;

        return OFFLINE;
    }

    ptr->last_online_cnt = ptr->online_cnt;
    ptr->status = ONLINE;
    return ONLINE;
}

inline void increment_online_count(offline_check_handle *ptr)
{
    ptr->online_cnt++;
}

// ÈıÖÖ×´Ì¬ 0 ÔÚÏß 1 ÒÉËÆÀëÏß 2 ÀëÏß
online_status_enum offline_FSM(Struct_FSM *fsm, offline_check_handle *offline_handle, uint32_t offline_trigger_cnt)
{
    fsm->Status[fsm->Now_Status_Serial].Time++;
    switch (Get_Now_Status_Serial(fsm))
    {
    // ÔÚÏß×´Ì¬
    case 0:
        offline_handle->status = ONLINE;
        if (offline_handle->online_cnt == offline_handle->last_online_cnt)
        {
            // ÔÚÏß×´Ì¬->ÒÉËÆÀëÏß×´Ì¬
            Set_FSM_Status(fsm, 1);
        }
        break;
    // ÒÉËÆÀëÏß×´Ì¬
    case 1:
        offline_handle->status = OFFLINE_SUSPECT;
        if (fsm->Status[fsm->Now_Status_Serial].Time >= offline_trigger_cnt)
        {
            // ÒÉËÆÀëÏß×´Ì¬->ÀëÏß×´Ì¬
            Set_FSM_Status(fsm, 2);
        }
        else if (offline_handle->online_cnt != offline_handle->last_online_cnt)
        {
            // ÒÉËÆÀëÏß×´Ì¬->ÔÚÏß×´Ì¬
            Set_FSM_Status(fsm, 0);
        }
        break;
    // ÀëÏß×´Ì¬
    case 2:
        offline_handle->status = OFFLINE;
        if (offline_handle->online_cnt != offline_handle->last_online_cnt)
        {
            // ÀëÏß×´Ì¬->ÔÚÏß×´Ì¬
            Set_FSM_Status(fsm, 0);
        }
        break;
    }
    return offline_handle->status;
}
