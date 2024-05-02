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
