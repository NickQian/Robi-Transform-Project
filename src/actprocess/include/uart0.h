/*  uart0 api(on pi 3B please disable bluetooth firstly)
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2016.9.16  init version for Raspberry 3B(bcm2837. Use /dev/ttyS0 instead of /dev/ttyAMA0)
    ----
*/

#include <stdint.h>

#define UART0_DEV  "/dev/ttyAMA0"    // "/dev/ttyS0" is for mini uart, not uart0

enum uart0_rcv_flg{
   UART0_RCV_FLG_NULL       = 0,
   UART0_RCV_FLG_START_RCV  = 1,
   UART0_RCV_FLG_IN_PROCESS = 2,
   UART0_RCV_FLG_LAST_READ  = 3
};


int *uart0_setup(uint32_t baudRate);
int uart0_sendBytes(uint8_t  *ptr, uint8_t nbyte);
int uart0_rcvBytes(uint8_t *ptr, uint16_t nbyte, uint8_t flag );
