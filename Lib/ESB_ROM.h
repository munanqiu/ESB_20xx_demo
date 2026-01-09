
#ifndef ING__ESB__ROM__H
#define ING__ESB__ROM__H
#include "stdint.h"

#define ING_ESB_OK    (0)
#define ING_ESB_ERROR (-1)

#define RX_CONT_OFFSET          28
#define TX_CONT_OFFSET          16

#define ESB_PHY_1M          0x0
#define ESB_PHY_2M          0x1

typedef enum
{
    INGESB_CB_RX,
    INGESB_CB_TX,
    INGESB_CB_EVENT,
    INGESB_CB_EVT_MAX
} ingesb_callback_type_t;

typedef enum
{
    INGESB_STATE_IDLE,
    INGESB_STATE_TX,
    INGESB_STATE_RX
} ingesb_work_state_t;

typedef enum
{
    INGESB_SUCCESS,            /* <=> Operate success. */
    INGESB_MODE_ERROR,         /* <=> It is not in ESB/BLE mode. */
    INGESB_TIMEOUT_ERR,        /* <=> No packet was received at expected time. */
    INGESB_CRC_ERR,            /* <=> Receive the packed but CRC failed. */
    INGESB_ERROR_TX_GOING,     /* <=> TX procedure is going. */
    INGESB_ERROR_RX_GOING,     /* <=> RX procedure is going */
    INGESB_PARAM_ERROR,        /* <=> Input parameters out of range */
} ingesb_status_t;

typedef enum
{
    MODE_PTX,
    MODE_PRX
} ingesb_work_mode_t;

typedef struct
{
    uint8_t   DataLen;
    uint8_t   pipe;
    int8_t    rssi;
    uint8_t   noack;
    uint8_t   pid;
    uint8_t   Data[252];
} ing_esb_payload_t;

typedef struct
{
    uint8_t   Mode;             /* <=> PTX or PRX */
    uint8_t   PHY;              /* <=> PHY, 0:1M, 1:2M*/
    uint8_t   AddrLen;          /* <=> Address length, 3~5 */
    uint8_t   CrcMode;          /* <=> CRC mode, 0~2 */
    uint8_t   PayloadLenMode;   /* <=> Payload length, 0:6 bits, 1:8 bits */
    uint8_t   PidMode;          /* <=> pid mode, 0:manual, 1:from PTX packet */
    uint8_t   NoAckMode;        /* <=> No ack mode, 0:ack, 1:no ack */ //to be confirmed
    uint8_t   TXPOW;            /* <=> Tx power idx, 0~63 */
    uint8_t   WhiteEn;          /* <=> Whitening enable, 0:disable, 1:enable */
    uint8_t   WhiteIdx;         /* <=> Whitening initial value, 0~63 */
    uint8_t   TransOrder;       /* <=> Transmission order, 0:LSB first, 1:MSB first */
    uint8_t   TimeOutMode;      /* <=> Timeout mode, 1:unit 625us; 0:unit 1us, 1-625us*/
    uint8_t   RxPktIntEn;       /* <=> Rx packet intterupt enable */
    uint8_t   TxPktIntEn;       /* <=> Tx packet intterupt enable */
    uint8_t   RetransCount;     /* <=> Retransmission count, 0~15 */
    uint8_t   Reserved;
    uint16_t  RetransDelay;     /* <=> Retransmission delay, uint us, 150-4095us*/
    uint16_t  Channel;          /* <=> Channel, exp:2402 */
    uint32_t  TimeOut;          /* <=> PRX rx timeout, 14Bits, Unit see TimeOutMode*/
    uint8_t   base_addr_0[4];   /* <=> Base address 0,LSB first */
    uint8_t   base_addr_1[4];   /* <=> Base address 1,LSB first */
    uint8_t   addr_prefix[8]; /* <=> Address prefix for each pipe */
} INGESB_Config_t;

typedef struct 
{
    uint32_t ecpt_base;
    uint32_t txcont_base;
    uint32_t rxcont_base;
    uint32_t rx_tx_cont_base;
    uint8_t  tx_payload_mtu;
    uint8_t  rx_payload_mtu;
    uint8_t  pipe_num;
}local_config_t;

typedef void (*f_InitHook)(INGESB_Config_t *INGESB_Config);

uint8_t ESB_crc8(uint8_t* addr, uint32_t num);
uint16_t ESB_crc16(uint8_t* addr, uint32_t num);
void ESB_StartEvent(uint32_t dealy, uint32_t ecpt_base);
void ESB_SetCont(ingesb_work_mode_t mode, ing_esb_payload_t *payload, local_config_t *config);
void InitHook_default(INGESB_Config_t *INGESB_Config);
void LLEInit_default(INGESB_Config_t *INGESB_Config, local_config_t *config, f_InitHook init_hook);
void PTXhook_default(INGESB_Config_t *INGESB_Config, ing_esb_payload_t *payload, local_config_t *Local_config, uint8_t (*loacl_esb_addr)[5]);
void PRXhook_default(INGESB_Config_t *INGESB_Config, ing_esb_payload_t *payload, local_config_t *Local_config, uint8_t (*loacl_esb_addr)[5]);
uint8_t GetData_default(uint32_t rx_base, ing_esb_payload_t *rxpacket);
#endif
