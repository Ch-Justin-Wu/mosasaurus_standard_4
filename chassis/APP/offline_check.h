#ifndef _OFFLINE_CHECK_H_
#define _OFFLINE_CHECK_H_

#include "stdint.h"

typedef enum
{
    ONLINE,
    OFFLINE,
} online_status_enum;

typedef struct
{
    uint32_t online_cnt;
    uint32_t last_online_cnt;
    uint32_t offline_cnt;

    online_status_enum status;
} offline_check_handle;

online_status_enum offline_check(offline_check_handle *ptr, uint32_t offline_trigger_cnt);
#endif // _OFFLINE_CHECK_H_