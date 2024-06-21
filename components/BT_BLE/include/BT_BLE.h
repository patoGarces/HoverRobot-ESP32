#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#define SPP_DEBUG_MODE

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)

///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

    // SPP_IDX_NB,

    // Caracteristica settings:

    SETTINGS_IDX_SVC,
    SETTINGS_IDX_CHAR_READ_DECL,
    SETTINGS_IDX_CHAR_READ_VAL,
    SETTINGS_IDX_CHAR_CFG,
    SETTINGS_IDX_CHAR_WRITE_DECL,
    SETTINGS_IDX_CHAR_WRITE_VAL,
    // SETTINGS_IDX_NB,

    LAST_GATT_NUM
};

void btInit( char* deviceName );
uint8_t isBtConnected(void);

