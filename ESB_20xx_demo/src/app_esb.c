#include "stdio.h"
#include "string.h"
#include "ESB_920_lib.h"
#include "platform_api.h"    
#include "btstack_event.h"
#include "profile.h"
#include "app_esb.h"


static ing_esb_payload_t esb_tx_payload={
    .pipe = 3,
    .DataLen = 11,
    .pid = 0,
    .noack = 1,
    .Data = {1,2,3,4,5,6,7,8,9,11},
};

static ing_esb_payload_t esb_rx_payload = {0};

static uint8_t continus_esb = 0;

static INGESB_Config_t ing_esb_config;
static comm_mode_esb_t comm_mode = MODE_ESB;

void continus_esb_txrx_on(void)
{
    continus_esb = 1;
}

void continus_esb_txrx_off(void)
{
    continus_esb = 0;
}

void ingesb_test_do_switch_to_esb(void){
    if(ing_esb_config.Mode == MODE_PTX){
        platform_printf("DO SWITCH ESB: MASTER.\n");
    } else {
        platform_printf("DO SWITCH ESB: SLAVE.\n");
    }
    ingesb_switch_to_ESB(&ing_esb_config);
    
    ingesb_set_address_enable();
    platform_printf("switch complete\n");
}



ADDITIONAL_ATTRIBUTE static void percent_cnt(uint16_t T_CNT, ingesb_status_t status)
{
    static uint16_t test_cnt = 0;
    static uint16_t ack_cnt = 0;
    static uint16_t miss_cnt = 0;
    static uint32_t tick_start, tick_end;

    test_cnt++;
    if(status == INGESB_SUCCESS){
        ack_cnt++;
    }else{
        miss_cnt++;
    }

    if(test_cnt >= T_CNT)
    {
        tick_end = app_esb_get_us_time();
        double rate = 1000*ack_cnt/(float)(tick_end - tick_start);
        printf("Test %d packet! miss: %d,rev: %d, rate: %.3fK pack/s\r\n", T_CNT, miss_cnt, ack_cnt, rate);
        ack_cnt = 0;
        miss_cnt = 0;
        test_cnt = 0;
        tick_start = app_esb_get_us_time();
    }
}

ADDITIONAL_ATTRIBUTE static void EventIrqCallBack(void)
{
    static ingesb_work_mode_t mode;
    ingesb_status_t status;
    ingesb_get_esb_work_mode(&mode);
    status = ingesb_get_rx_data(&esb_rx_payload);
//    status = ingesb_get_rx_state();
//    printf("Event cb mode:%d status:0x%x\n", mode, status);

//        printf("len:%d, data0:0x%x, data1:0x%x, rssi:%d\n", rx_payload.DataLen, rx_payload.Data[0], rx_payload.Data[1], rx_payload.rssi);
//        printf("pipe:%d, pid:%d, noack:%d\n", rx_payload.pipe, rx_payload.pid, rx_payload.noack);
//    printf("T:%lld\n", app_esb_get_us_time());
#if(TEST_PACKET_RATE == 0)
    if(0 == status)
    {
        if(mode == MODE_PTX)
        {
            printf("[TX] ack len:%d, pipe:%d, pid:%d, noack:%d\n", esb_rx_payload.DataLen, esb_rx_payload.pipe, esb_rx_payload.pid, esb_rx_payload.noack);
        }
        else
        {
            printf("[RX] rev len:%d, pipe:%d, pid:%d, noack:%d\n", esb_rx_payload.DataLen, esb_rx_payload.pipe, esb_rx_payload.pid, esb_rx_payload.noack);
        }

        for(uint8_t i=0; i<esb_rx_payload.DataLen; i++)//(uint8_t i=0; i<esb_rx_payload.DataLen; i++)
        {
            printf("%d, ", esb_rx_payload.Data[i]);
        }
        printf("\n");
    }
    else {
        printf("[ROLE%d]err:%d\n", mode, status);
    }
    if((mode == MODE_PRX)&& continus_esb)
    {
        ingesb_start_rx(esb_tx_payload);
    }

#elif(TEST_PACKET_RATE == 1)
//debug RX loss rate    
    if(0 == status)
    {
        rev_num++;
    }
        printf("[ING20 RX]rev num:%d\n", rev_num);
        ingesb_start_rx(esb_tx_payload);

#elif(TEST_PACKET_RATE == 2)
//debug RX loss rate
    if(0 == status)
    {
        rev_num++;
    }
    
    printf("[ING20 TX]Send:%d, Rev:%d\n", sent_num, rev_num);
  
#elif(TEST_PACKET_RATE == 3)
     if(continus_esb == 1)
     {
         if(mode == MODE_PTX)
         {
//           tx_data[0] = (tx_data[0] + 1)%100;
             percent_cnt(1000, status);
             ingesb_start_tx(esb_tx_payload);
         }
         else
         {
             ingesb_start_rx(esb_tx_payload);
         }
     }
     else{
         if(mode == MODE_PTX)
         {
              printf("Event cb Tx:%d, len:%d\n", status, esb_rx_payload.DataLen);
         }
         else
         {
             printf("Event cb Rx:%d, len:%d\n", status, esb_rx_payload.DataLen);
         }
     }
#endif
}

uint8_t my_base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
uint8_t my_base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
uint8_t my_addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

void ing_esb_config_init(void)
{
    ing_esb_config.Mode              = MODE_PTX;
    ing_esb_config.PHY               = ESB_PHY_1M;
    ing_esb_config.Channel           = 2380;
    ing_esb_config.AddrLen           = 5;
    ing_esb_config.CrcMode           = 2;
    ing_esb_config.PayloadLenMode    = 1;
    ing_esb_config.PidMode           = 1;
    ing_esb_config.NoAckMode         = 0;
    ing_esb_config.WhiteEn           = 0;
    ing_esb_config.WhiteIdx          = 0x55;
    ing_esb_config.TransOrder        = 0;
    ing_esb_config.RetransCount      = 0;
    ing_esb_config.RetransDelay      = 200;
    ing_esb_config.TXPOW             = 63;
    ing_esb_config.TimeOut           = 1600;//10000;//6.25s
    ing_esb_config.TimeOutMode       = 1;
    ing_esb_config.RxPktIntEn        = 0;
    ing_esb_config.TxPktIntEn        = 0;
    memcpy(ing_esb_config.base_addr_0, my_base_addr_0, 4);
    memcpy(ing_esb_config.base_addr_1, my_base_addr_1, 4);
    memcpy(ing_esb_config.addr_prefix, my_addr_prefix, MAX_PIPE_NUM);
    ingesb_set_base_addr_0(my_base_addr_0);
    ingesb_set_base_addr_1(my_base_addr_1);
    ingesb_set_prefix(my_addr_prefix);
}

void switch_to_esb(void)
{
    ingesb_switch_to_ESB(&ing_esb_config);
}

static void RxPktIrqCallBack(void)
{
    printf("Rx int\n");
}

static void TxPktIrqCallBack(void)
{
    printf("Tx int\n");
}


void ingesb_test_init(void){
    ESB_mon_print_ver();
    ingesb_set_irq_callback(INGESB_CB_EVENT, EventIrqCallBack);
    ingesb_set_irq_callback(INGESB_CB_RX, RxPktIrqCallBack);
    ingesb_set_irq_callback(INGESB_CB_TX, TxPktIrqCallBack);
    ing_esb_config_init();
}