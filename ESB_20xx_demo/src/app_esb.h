#ifndef APP_ESB__H
#define APP_ESB__H

#include "ESB_920_lib.h"

//0:no debug
//1:RX packet loss rate, nordic as PTX
//2:TX packet loss rate, nordic as PRX
//3:pakcet loss rate, ing20 as PTX & PRX
#define TEST_PACKET_RATE   3

void ingesb_test_do_switch_to_esb(void);
void ingesb_test_switch_mode_handler(void);
void ingesb_test_switch_mode_trigger(comm_mode_esb_t mode);

void continus_esb_txrx_on(void);
void continus_esb_txrx_off(void);

void ingesb_test_init(void);
void switch_to_esb(void);

#endif