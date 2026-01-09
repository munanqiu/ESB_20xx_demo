#include "uart_console.h"


#include <stdio.h>
#include <string.h>

#include "platform_api.h"

#include "port_gen_os_driver.h"

#include "stdio.h"
#include "stdlib.h"
#include "ESB_920_lib.h"
#include "app_esb.h"

#define GEN_OS          ((const gen_os_driver_t *)platform_get_gen_os_driver())

#ifdef TRACE_TO_FLASH
#include "trace.h"
#endif

typedef void (*f_cmd_handler)(const char *param);

int adv_tx_power = 100;

typedef struct
{
    const char *cmd;
    f_cmd_handler handler;
} cmd_t;

const static char error[] = "error";
static char buffer[100] = {0};

static void tx_data(const char *d, const uint16_t len);

static const char help[] =  "commands:\n"
                            "h/?                                 show this\n"
                            "ble                                 switch to ble\n"
                            "esb                                 switch to esb\n"
                            "state                               get esb state\n"
                            "mode                                get esb work mode\n"
                            "phy                                 set phy\n"
                            "aa                                  set access address\n"
                            "power                               set tx power\n"
                            "channel                             set channel\n"
                            "txcon                               esb tx continus\n"
                            "txone                               tx one packet with x payload\n"
                            "stopcon                             stop esb continus tx/rx\n"
                            "rxcon                               esb rx contiuns\n"
                            "rxone                               rx one packet with x payload\n"
                            "timeout                             set rx timeout\n"
                            "rxdata                              get rx data\n"
                            "rxint                               clear rx interrupt\n"
                            "eventint                            clear tx interrupt\n"
                            ;

void cmd_help(const char *param)
{
    tx_data(help, strlen(help) + 1);
}

//void cmd_switch_to_ble(const char *param)
//{
//    continus_esb_txrx_off();
//    ingesb_test_switch_mode_trigger(MODE_BLE);
//}

//void cmd_switch_to_esb(const char *param)
//{
//    ingesb_test_switch_mode_trigger(MODE_ESB);
//}

void cmd_get_esb_state(const char *param)
{
    ingesb_work_state_t state;
    ingesb_status_t return_state;
    return_state = ingesb_get_state(&state);
    if(return_state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb state.\r\n");
        return;
    }
    switch(state)
    {
        case 0:
            platform_printf("INGESB_STATE_IDLE.\r\n");
            break;
        case 1:
            platform_printf("INGESB_STATE_TX.\r\n");
            break;
        case 2:
            platform_printf("INGESB_STATE_RX.\r\n");
            break;
        default:
            platform_printf("Unknown ingesb state.\r\n");
            break;
    }
}

void cmd_get_esb_work_mode(const char *param)
{
    ingesb_status_t state;
    ingesb_work_mode_t mode;
    state = ingesb_get_esb_work_mode(&mode);
    if(state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb state.\r\n");
        return;
    }
    switch(mode)
    {
        case 0:
            platform_printf("Work mode Master.\r\n");
            break;
        case 1:
            platform_printf("Work mode Slave.\r\n");
            break;
        default:
            platform_printf("Unknown ingesb work Mode.\r\n");
            break;
    }
}

void cmd_phy_set(const char *param)
{
    static uint8_t phy = 1;
    ingesb_status_t state;
    state = ingesb_set_phy(phy);
    if(state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb mode.\r\n");
    }
    else
    {
        platform_printf("phy:%d set result:%d.\r\n", phy, state);
        if(state == INGESB_SUCCESS)
            phy = (phy ? 0:1);
    }
}

void cmd_access_address_set(const char *param)
{
    static uint32_t aa = 0x12345611;
    ingesb_status_t state;
//    state = ingesb_set_access_address(aa);
    if(state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb mode.\r\n");
    }
    else
    {
        platform_printf("aa:%4x set result:%d.\r\n", aa, state);
        if(state == INGESB_SUCCESS)
            aa += 0x10;
    }
}

void cmd_power_set(const char *param)
{
    ingesb_status_t state;
    static uint8_t power = 63;
    state = ingesb_set_tx_power(power);
    if(state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb mode.\r\n");
    }
    else
    {
        platform_printf("power:%d set resutl:%d.\r\n", power, state);
        if(power > 3)
            power -= 30;
        else
            power = 63;
    }
}

void cmd_channel_set(const char *param)
{
    ingesb_status_t state;
    static uint16_t channel = 2380;
    state = ingesb_set_channel(channel);
    if(state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb mode.\r\n");
    }
    else
    {
        platform_printf("channel: %d set result:%d.\r\n", channel, state);
        if(state == INGESB_SUCCESS)
            channel = (channel == 2380)?2390:2380;
    }
}

void cmd_rx_timeout_set(const char *param)
{
    ingesb_status_t state;
    static uint32_t timeout = 1000;
    state = ingesb_set_rx_timeout(timeout, 1);
    if(state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb mode.\r\n");
    }
    else
    {
        platform_printf("timeout:%d set result:%d\r\n", timeout, state);
        if(state == INGESB_SUCCESS)
            timeout += 1000;
    }
}

void cmd_rx_data_get(const char *param)
{
    ingesb_status_t state;
    uint16_t i;
    static ing_esb_payload_t rx_packet;
    state = ingesb_get_rx_data(&rx_packet);
    if(state == INGESB_MODE_ERROR)
    {
        platform_printf("Not in esb mode.\r\n");
        return;
    }
    if(state == INGESB_SUCCESS)
    {
        platform_printf("Rx data:");
        for(i=0; i< rx_packet.DataLen; i++)
            platform_printf("%d ",rx_packet.Data[i]);
        platform_printf(".\r\n");
    }
    else
    {
        platform_printf("data get failed:%d.\r\n", state);
    }
}


uint8_t noack_flg = 0;


static ing_esb_payload_t tx_payload ={
    .pipe = 3,
    .DataLen = 13,
    .pid = 0,
    .noack = 1,
    .Data = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a},
};

static ing_esb_payload_t rx_payload ={
    .pipe = 3,
    .DataLen = 11,
    .pid = 0,
    .noack = 0,
    .Data = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa},
};
extern uint8_t continus_esb;
void cmd_tx_one_packet(const char *param)
{
    ingesb_status_t state;
    ingesb_set_esb_work_mode(MODE_PTX);

    state = ingesb_start_tx(tx_payload);
    tx_payload.pid = (tx_payload.pid + 1) & 0x3;
//    tx_payload.pipe = (tx_payload.pipe+1)%7;
//    printf("pipe:%d\n", tx_payload.pipe);
}

void cmd_tx_continus(const char *param)
{
    ingesb_status_t state;
    continus_esb_txrx_on();
    ingesb_set_esb_work_mode(MODE_PTX);

    ingesb_start_tx(tx_payload);
    tx_payload.pid = (tx_payload.pid + 1) & 0x3;
}

void cmd_rx_one_packet(const char *param)
{
    ingesb_status_t state;
    ingesb_set_esb_work_mode(MODE_PRX);

    state = ingesb_start_rx(rx_payload);
}

void cmd_rx_continus(const char *param)
{
    ingesb_status_t state;
    ingesb_set_esb_work_mode(MODE_PRX);
    continus_esb_txrx_on();

    ingesb_start_rx(rx_payload);
}

void cmd_stop_continus(const char *param)
{
    if(ing_ble_esb_state_get() == 0)
    {
        platform_printf("Not in esb mode.\r\n");
        return;
    }
    else
    {
        continus_esb_txrx_off();
        platform_printf("stop continus.\r\n");
    }
    
}


void cmd_shutdown(const char *param)
{
    platform_printf("shutdown\r\n");
    platform_shutdown(0, NULL, 0);
}
void *test_content = NULL;
void *test_content2 = NULL;
#define r32(a)   *(volatile uint32_t*)(a)
void cmd_test_inst1(const char *param)
{
    static uint8_t cnt = 0;
    printf("Retrans count:%d\n", cnt);
    ingesb_set_RetransCount(cnt);
    cnt++;
}

void cmd_test_inst2(const char *param)
{
    static uint16_t time = 0;
    ingesb_set_RetransDelay(time);
    printf("R delay:%d\n", time);
    time+=50;
}

static cmd_t cmds[] =
{
    {
        .cmd = "h",
        .handler = cmd_help
    },
    {
        .cmd = "?",
        .handler = cmd_help
    },
//    {
//        .cmd = "ble",
//        .handler = cmd_switch_to_ble
//    },
//    {
//        .cmd = "esb",
//        .handler = cmd_switch_to_esb
//    },
    {
        .cmd = "state",
        .handler = cmd_get_esb_state
    },    
    {
        .cmd = "mode",
        .handler = cmd_get_esb_work_mode
    },
    {
        .cmd = "phy",
        .handler = cmd_phy_set
    },
    {
        .cmd = "aa",
        .handler = cmd_access_address_set
    },
    {
        .cmd = "power",
        .handler = cmd_power_set
    },
    {
        .cmd = "channel",
        .handler = cmd_channel_set
    },
    {
        .cmd = "txone",
        .handler = cmd_tx_one_packet
    },
    {
        .cmd = "txcon",
        .handler = cmd_tx_continus
    },
    {
        .cmd = "rxone",
        .handler = cmd_rx_one_packet
    },
    {
        .cmd = "rxcon",
        .handler = cmd_rx_continus
    },
    {
        .cmd = "stopcon",
        .handler = cmd_stop_continus
    },
    {
        .cmd = "timeout",
        .handler = cmd_rx_timeout_set
    },
    {
        .cmd = "rxdata",
        .handler = cmd_rx_data_get
    },
    {
        .cmd = "test1",
        .handler = cmd_test_inst1
    },
    {
        .cmd = "test2",
        .handler = cmd_test_inst2
    },
    {
        .cmd = "shutdown",
        .handler = cmd_shutdown
    }
};

void handle_command(char *cmd_line)
{
    static const char unknow_cmd[] =  "unknown command\n";
    char *param = cmd_line;
    int i;
    while (*param)
    {
        if (*param == ' ')
        {
            *param = '\0';
            param++;
            break;
        }
        else
            param++;
    }

    for (i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
    {
        if (strcasecmp(cmds[i].cmd, cmd_line) == 0)
            break;
    }
    if (i >= sizeof(cmds) / sizeof(cmds[0]))
        goto show_help;

    cmds[i].handler(param);
    return;

show_help:
    tx_data(unknow_cmd, strlen(unknow_cmd) + 1);
    cmd_help(NULL);
}

typedef struct
{
    uint8_t busy;
    uint16_t size;
    char buf[712];
} str_buf_t;

str_buf_t input = {0};
str_buf_t output = {0};

/*
void handle_command()
{
    tx_data("response: ", 10);
    tx_data(input.buf, input.size);
}
*/

static void append_data(str_buf_t *buf, const char *d, const uint16_t len)
{
    if (buf->size + len > sizeof(buf->buf))
        buf->size = 0;

    if (buf->size + len <= sizeof(buf->buf))
    {
        memcpy(buf->buf + buf->size, d, len);
        buf->size += len;
    }
}

static gen_handle_t cmd_event = NULL;

static void console_task_entry(void *_)
{
    while (1)
    {
        GEN_OS->event_wait(cmd_event);

        handle_command(input.buf);
        input.size = 0;
        input.busy = 0;
    }
}

void uart_console_start(void)
{
    cmd_event = GEN_OS->event_create();
    GEN_OS->task_create("console",
        console_task_entry,
        NULL,
        1024,
        GEN_TASK_PRIORITY_LOW);
}

void console_rx_data(const char *d, uint8_t len)
{
    if (input.busy)
    {
        return;
    }

    if (0 == input.size)
    {
        while ((len > 0) && ((*d == '\r') || (*d == '\n')))
        {
            d++;
            len--;
        }
    }
    if (len == 0) return;

    append_data(&input, d, len);

    if ((input.size > 0) &&
        ((input.buf[input.size - 1] == '\r') || (input.buf[input.size - 1] == '\n')))
    {
        int16_t t = input.size - 2;
        while ((t > 0) && ((input.buf[t] == '\r') || (input.buf[t] == '\n'))) t--;
        input.buf[t + 1] = '\0';
        input.busy = 1;
        GEN_OS->event_set(cmd_event);
    }
}

extern void stack_notify_tx_data(void);

static void tx_data(const char *d, const uint16_t len)
{
    if ((output.size == 0) && (d[len - 1] == '\0'))
    {
        puts(d);
        return;
    }

    append_data(&output, d, len);

    if ((output.size > 0) && (output.buf[output.size - 1] == '\0'))
    {
        puts(output.buf);
        output.size = 0;
    }
}

uint8_t *console_get_clear_tx_data(uint16_t *len)
{
    *len = output.size;
    output.size = 0;
    return (uint8_t *)output.buf;
}
