
#ifndef ING__ESB__920__H
#define ING__ESB__920__H
#include "stdint.h"
#include <stdint.h>

/*********************************** */
//Version:   "V1.0.0"
/*********************************** */
#define KEIL 1

#ifndef OPT_RAM_CODE
#define OPT_RAM_CODE
#endif

#ifdef OPT_RAM_CODE
    #define ADDITIONAL_ATTRIBUTE    __attribute__((section(".ram_code")))
#else
    #define ADDITIONAL_ATTRIBUTE
#endif

typedef enum
{
    MODE_BLE,
    MODE_ESB
} comm_mode_esb_t;

//1-252 bytes, Must be a multiple of 4
#define ING_ESB_MAX_PTX_PAYLOAD_LEN  252
//1-252 bytes, Must be a multiple of 4
#define ING_ESB_MAX_ACK_PAYLOAD_LEN  64

//max pipe num, 1-8, If the allocation of space fails, reduce this value or decrease MAX_PIPE_NUM.
#define MAX_PIPE_NUM        8

typedef void (*f_ingesb_cb)(void);
#define ROM_PATCH_ENABLE

#ifndef ROM_PATCH_ENABLE

#define ING_ESB_OK    (0)
#define ING_ESB_ERROR (-1)

#define ESB_PHY_1M          0x0
#define ESB_PHY_2M          0x1

#define RX_CONT_OFFSET          16
#define TX_CONT_OFFSET          28

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
    uint8_t   Data[ING_ESB_MAX_PTX_PAYLOAD_LEN];
} ing_esb_payload_t;

typedef struct 
{
    uint32_t ecpt_base;
    uint32_t txcont_base;
    uint32_t rxcont_base;
    uint32_t rx_tx_cont_base;
}local_config_t;

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
    uint16_t  RetransDelay;     /* <=> Retransmission delay, uint us */
    uint16_t  Channel;          /* <=> Channel, exp:2402 */
    uint32_t  TimeOut;          /* <=> PRX rx timeout, 14Bits, Unit see TimeOutMode*/
    uint8_t   base_addr_0[4];   /* <=> Base address 0,LSB first */
    uint8_t   base_addr_1[4];   /* <=> Base address 1,LSB first */
    uint8_t   addr_prefix[MAX_PIPE_NUM]; /* <=> Address prefix for each pipe */
} INGESB_Config_t;

#else
#include "ESB_ROM.h"
#endif

/**
 ****************************************************************************************
 * @brief register callback function for ingESB event
 *
 * @param[in] type                 The event, see ingesb_callback_type_t
 * @param[in] f                    The callback function
 ****************************************************************************************
 */
void ingesb_set_irq_callback(ingesb_callback_type_t type, f_ingesb_cb f);

/**
 ****************************************************************************************
 * @brief get the communication status, if communication over, get the received data
 *
 * @param[out] rxpacket        The pointer to struct to store the received data
 *                             see ing_esb_payload_t
 *
 * @return                     The status of communication
 *                             INGESB_SUCCESS          : communication ok
 *                             other                    : failed, see ingesb_status_t
 ****************************************************************************************
 */
ingesb_status_t ingesb_get_rx_data(ing_esb_payload_t *rx_packet);
/**
 ****************************************************************************************
 * @brief get the communication state, without data
 *
 * @return                     The status of communication
 *                             INGESB_SUCCESS          : communication ok
 *                             other                    : failed, see ingesb_status_t
 ****************************************************************************************
 */
ingesb_status_t ingesb_get_rx_state(void);

/**
 ****************************************************************************************
 * @brief set timeout for the PRX RX event
 *
 * @param[in] time_out         Range:0-10000
 * @param[in] mode             1:unit 625us 0-6.25s; 0:unit 1us, 1-625us
 *
 * @retrun                     the result of set timeout:
 *                             0: set success
 *                             1: failed, for the device is in TX state now
 *                             2: failed, for the device is in RX state now
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_rx_timeout(uint32_t time_out, uint8_t mode);

/**
 ****************************************************************************************
 * @brief start a RX event for ingESB PRX, and set the ack data
 * @note  make sure the work mode of ESB is MODE_PRX, or you can set it by ingesb_set_esb_work_mode
 * 
 * @param[in] payload          payload of ack, see ing_esb_payload_t
 *
 * @retrun                     The result of RX:
 *                             INGESB_SUCCESS          : rx success
 *                             INGESB_MODE_ERROR       : failed, It is not in ESB mode.
 *                             INGESB_ERROR_TX_GOING   : failed, for the device is in TX state now
 *                             INGESB_ERROR_RX_GOING   : failed, for the device is in RX state now
 ****************************************************************************************
 */
ingesb_status_t ingesb_start_rx(ing_esb_payload_t payload);

/**
 ****************************************************************************************
 * @brief start a TX event for PTX, and configure the data to be sent
 * @note  make sure the work mode of ESB is MODE_MASETR, or you can set it by ingesb_set_esb_work_mode
 * 
 * @param[in] payload          payload to be sent, see ing_esb_payload_t
 *
 * @retrun                     The result of TX:
 *                             INGESB_SUCCESS          : tx success
 *                             INGESB_MODE_ERROR       : failed, It is not in ESB mode.
 *                             INGESB_ERROR_TX_GOING   : failed, for the device is in TX state now
 *                             INGESB_ERROR_RX_GOING   : failed, for the device is in RX state now
 ****************************************************************************************
 */
ingesb_status_t ingesb_start_tx(ing_esb_payload_t payload);

/**
 ****************************************************************************************
 * @brief set the channel for the PTX/PRX
 *
 * @param[in] Channel          The channel, should be within 2300 to 2500MHz, 1MHz for a step
 *
 * @retrun                     The result of set channel:
 *                             INGESB_SUCCESS          : set success
 *                             INGESB_MODE_ERROR       : failed, It is not in ESB mode.
 *                             INGESB_ERROR_TX_GOING   : failed, for the device is in TX state now
 *                             INGESB_ERROR_RX_GOING   : failed, for the device is in RX state now
 *                             INGESB_PARAM_ERROR      : failed, for param channel out of range
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_channel(uint16_t channel);

/**
 ****************************************************************************************
 * @brief set the TX power for the PTX/PRX
 *
 * @param[in] tx_power         The TX power, should be within 0 to 63, 1 for a step
 *                             if tx_power greater than 63, set to 63
 *
 * @retrun                     The result of set TX power:
 *                             INGESB_SUCCESS          : set success
 *                             INGESB_MODE_ERROR       : failed, It is not in ESB mode.
 *                             INGESB_ERROR_TX_GOING   : failed, for the device is in TX state now
 *                             INGESB_ERROR_RX_GOING   : failed, for the device is in RX state now
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_tx_power(uint8_t tx_power);

/**
 ****************************************************************************************
 * @brief set the base address0 for PTX/PRX
 *
 * @param[in] addr             The 4 bytes base addr0
 *                             prefix[0] and base address 0 form the address of pipe 0
 ****************************************************************************************
 */
void ingesb_set_base_addr_0(uint8_t const *addr);

/**
 ****************************************************************************************
 * @brief set the base address1 for PTX/PRX
 *
 * @param[in] addr             The 4 bytes base addr0
 *                             prefix[1..7] and base address 1 form the address of pipe 1..7
 ****************************************************************************************
 */
void ingesb_set_base_addr_1(uint8_t const *addr);

/**
 ****************************************************************************************
 * @brief set the prefix address for PTX/PRX
 *
 * @param[in] addr             The 8 bytes prefix addr
 ****************************************************************************************
 */
void ingesb_set_prefix(uint8_t const *addr);

/**
 ****************************************************************************************
 * @brief set the address enable, Call once after switching to ESB mode or configuring the address. 
 *
 ****************************************************************************************
 */
void ingesb_set_address_enable(void);


/**
 ****************************************************************************************
 * @brief set the base addr1 for ack of PRX(when ack enable)
 *
 * @param[in] mode             The pid mode.
 *                             0:use the pid from own payload
 *                             1:use the PID from packet of PTX
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_PidMode(uint8_t mode);

/**
 ****************************************************************************************
 * @brief set the max retransmit count for PTX
 *
 * @param[in] cnt              retransmit count, Valid range: 0-15(inclusive).
 *                             If the input value is greater than 15, it will be clamped to 15.
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_RetransCount(uint8_t cnt);

/**
 ****************************************************************************************
 * @brief set the max retransmit delay for PTX
 * @note  the delay start time is the last bit of the previous transmission.
 *
 * @param[in] delay             retransmit delay, Valid range: 150-1000(inclusive), unit us.
 *                              If the input value is out of the range, it will be clamped 
 *                              to the nearest limit.
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_RetransDelay(uint16_t delay);

/**
 ****************************************************************************************
 * @brief set the phy for the PTX/PRX
 *
 * @param[in] phy              The phy to be set:
 *                             ESB_PHY_1M          0x0
 *                             ESB_PHY_2M          0x1
 *
 * @retrun                     The result of set phy:
 *                             INGESB_SUCCESS          : set success
 *                             INGESB_MODE_ERROR       : failed, It is not in ESB mode.
 *                             INGESB_ERROR_TX_GOING   : failed, for the device is in TX state now
 *                             INGESB_ERROR_RX_GOING   : failed, for the device is in RX state now
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_phy(uint8_t phy);

/**
 ****************************************************************************************
 * @brief get the state of ingESB, reference: ingesb_work_state_t
 * @note This function can only be called at ESB mode.
 * @param[out] state           The state of ingesb:
 *                             INGESB_STATE_IDLE       : idle
 *                             INGESB_STATE_TX         : TX state
 *                             INGESB_STATE_RX         : RX state
 * 
 * @retrun                     The result of get state:
 *                             INGESB_SUCCESS          : success
 *                             INGESB_MODE_ERROR       : failed, It is not in ESB mode.
 ****************************************************************************************
 */
ingesb_status_t ingesb_get_state(ingesb_work_state_t * state);

/**
 ****************************************************************************************
 * @brief switch from BLE to ESB
 * @note  Before calling this function, ensure that the BLE is idle (for example, the 
 *        BLE cannot be in the state of broadcast, connection, scan, etc.), otherwise 
 *        the operation will fail. 
 *
 * @param[in] config           The config of ESB, see INGESB_Config_t
 * 
 * @retrun                     The result of switch:
 *                             INGESB_SUCCESS          : success
 *                             INGESB_MODE_ERROR       : failed, It is not in BLE mode.
 *
 ****************************************************************************************
 */
ingesb_status_t ingesb_switch_to_ESB(INGESB_Config_t *config);

/**
 ****************************************************************************************
 * @brief set ESB rf parameter
 *
 * @param[in] config           The config of ESB, see INGESB_Config_t
 ****************************************************************************************
 */
void ingesb_set_parameter(INGESB_Config_t *config);

/**
 ****************************************************************************************
 * @brief switch to BLE
 * @note  Before calling this function, you must ensure that ESB is idle, otherwise, switching to 
 *        BLE mode will fail. See Return value for details. When you successfully switch to BLE mode, 
 *        a HCI_RESET_CMD_OPCODE event is generated, because at the end of this function, btstack_reset() 
 *        is called to reset the ble protocol stack.
 * 
 * @return                     The result of switching to BLE:
 *                             INGESB_SUCCESS          : switch success
 *                             INGESB_MODE_ERROR       : failed, It is not in ESB mode.
 *                             INGESB_ERROR_TX_GOING   : failed, for the device is in TX state now
 *                             INGESB_ERROR_RX_GOING   : failed, for the device is in RX state now
 ****************************************************************************************
 */
ingesb_status_t ingesb_switch_to_BLE(void);

/**
 ****************************************************************************************
 * @brief Init BLE and ingESB dual mode. 
 * @note  In ESB projects, even if you do not switch to ESB mode, this function will be forced to 
 *        initialize so that BLE can work properly. Otherwise, the interrupt system may be corrupted 
 *        because the startup file (startup_ing91600.s) has been changed. 
 ****************************************************************************************
 */
void esb_init_dual_mode(void);

/**
 ****************************************************************************************
 * @brief get the state of the device, reference: comm_mode_esb_t
 *
 * @retrun                     The result of get state:
 *                             0          : MODE_BLE
 *                             1          : MODE_ESB.
 ****************************************************************************************
 */
uint8_t ing_ble_esb_state_get(void);

/**
 ****************************************************************************************
 * @brief get the work mode of the ingESB, reference: ingesb_work_mode_t
 * @note  It's OK to be called when TX/RX going
 *
 * @param[out] state           The work mode of ingESB:
 *                                                    0: MODE_PTX
 *                                                    1: MODE_PRX
 * @return                     The result of reading the work mode of ingESB:
 *                             INGESB_SUCCESS         : success
 *                             INGESB_MODE_ERROR      : failed, It is not in ESB mode.
 ****************************************************************************************
 */
ingesb_status_t ingesb_get_esb_work_mode(ingesb_work_mode_t *mode);

/**
 ****************************************************************************************
 * @brief set the work mode of the ingESB, reference: ingesb_work_mode_t
 *
 * @param[in] state            The work mode of ingESB:
 *                                                    0: MODE_PTX
 *                                                    1: MODE_PRX
 *                             INGESB_STATE_RX        : RX state
 * @return                     The result of reading the work mode of ingESB:
 *                             INGESB_SUCCESS         : success
 *                             INGESB_MODE_ERROR      : failed, It is not in ESB mode.
 *                             INGESB_ERROR_TX_GOING   : failed, for the device is in TX state now
 *                             INGESB_ERROR_RX_GOING   : failed, for the device is in RX state now
 ****************************************************************************************
 */
ingesb_status_t ingesb_set_esb_work_mode(ingesb_work_mode_t mode);

/**
 ****************************************************************************************
 * @brief stop the current tx/rx rf event
 *
 * note: be careful to use this function. after you call it:
 *              1)the whiten is enable(ing_esb_config.WhiteEn = 0x1);
 *              2)if a timer create by platform_create_us_timer is running, the timer is disabled. make sure not to use 
 *              ingesb_rf_stop when a timer is running.
 ****************************************************************************************
 */
void ingesb_rf_stop(void);

/**
 ****************************************************************************************
 * @brief cancel the ack of PRX at RX interrupt
 *
 * note:this function is used to cancel the ack of PRX at RX interrupt. 
        It can only be used in this kind of situation:
            1)PRX RX
            2)at callback of RX interrupt
            3)called when rx is received, is no rx data received, 
            4)Call it as soon as the RX interrupt occurs, preferably within 60 microseconds.
 ****************************************************************************************
 */
void ingesb_slave_stop_ack(void);

/**
 ****************************************************************************************
 * @brief disable/enable the tx of RF
 *
 * @param[in]   enable           0:disable tx  1:enable tx
 *
 * @return                     The result of setting:
 *                             INGESB_SUCCESS         : success
 *                             INGESB_MODE_ERROR      : failed, It is not in ESB mode.
 * note: the function could be used only in ESB mode.
 *              For PRX, when disable tx, the device can receive the packet from PTX, but could not send ack.
 *              For PTX, when disable tx,the device can't send packet to PRX.
 *              This function can be used when you do not want ACK from PRX.
 ****************************************************************************************
 */
ingesb_status_t ingesb_rf_tx_set(uint8_t enable);

/**
 ****************************************************************************************
 * @brief init esb when wakeup
 *
 * note: after wakeup, esb should be init before use
 ****************************************************************************************
 */
void ingesb_lle_init(void);

/**
 ****************************************************************************************
 * @brief set the ack data at RX callback, just for PRX.
 *
 * @param[in] data             The pointer to the ack data
 * @param[in] len              the length of data
 *
 * note: only the data content can be adjusted, the data length cannot be changed. 
         the actual length is what set at ingesb_start_rx or ingesb_start_esb_rx_with_timeout.
 ****************************************************************************************
 */
void ESB_SetCont_rx_int(uint8_t *data, uint8_t len);

/**
 ****************************************************************************************
 * @brief get us time
 *
 ****************************************************************************************
 */
uint64_t app_esb_get_us_time(void);

/**
 ****************************************************************************************
 * @brief print the version of the lib.
 *
 ****************************************************************************************
 */
void ESB_mon_print_ver(void);

#endif
