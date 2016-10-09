/*  rd/wr uart0 using POSIX interface(needs to disable bluetooth firstly on Pi 3B)
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2016.9.16  init version for Raspberry 3B(bcm2837. Use /dev/ttyS0 instead of /dev/ttyAMA0)
    ----
*/

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#include "uart0.h"
#include "peri.h"

using namespace std;


//extern uint32_t *bcm2836_gpio_vm;
static int uart0_filestream = -1;


int *uart0_setup(uint32_t baudRate){
   //int uart0_filestream = -1;

   uart0_filestream = open( UART0_DEV, O_RDWR | O_NOCTTY | O_NDELAY);  // no-blocking read/write
   if (-1 == uart0_filestream){
     printf("Error - Unable to open UART. Is it used by another application?\n");
    // return -1;
   }
   // defined in termios.h:
   // Baud rate: - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, ...
   // CSIZE  - CS5, CS6, CS7, CS8
   // CLOCAL - ignore modem status lines
   // PARENB - parity enable
   // PARODD - Odd parity
   struct termios options;
   tcgetattr(uart0_filestream, &options);
   options.c_cflag = B115200 |CS8 | CLOCAL |CREAD;              // CSTOPB: 2 stop bit
   options.c_iflag = IGNPAR;
   options.c_oflag = 0;
   options.c_lflag = 0;
   tcflush(uart0_filestream, TCIFLUSH);
   tcsetattr(uart0_filestream, TCSANOW, &options);

   //cout <<"~~~ uart0 status after set(c_cflag|iflag|oflag|lflag):0x" << hex << options.c_cflag
   //            <<" 0x" << hex << options.c_iflag  << " 0x" << hex << options.c_oflag << " 0x" << hex << options.c_lflag << endl;

   //close(uart0_filestream); 

   return &uart0_filestream;
}


int uart0_sendBytes(uint8_t  *ptr, uint8_t nbyte){
   // int uart0_filestream = -1;

   // recover the tx pin as tx for sending data
   gpio_funcSel(RPi2B_GPIO_J8_8,  GPIO_FSEL_ALT0);   // uart0 tx,gpio14

   if (-1 == uart0_filestream){
       printf("Error - UART unopend before tx. Is it used by another application?\n");
       return -1;
   }
   else{
      //cout << "Info:<uart0>: writing " << (uint32_t)nbyte <<  endl;
      int count = write(uart0_filestream, ptr, nbyte      ); //int handle, void *buf, int nbyte);

     if (count < 0){
        cout << "Error: <uart0>: UART0 tx erro. Count is:" << count << ". ptr is:" << (unsigned int)ptr << endl;
        return -1;
     }
   }

   // wait until the tx data sent. Then set tx pin as input to adapt RS-485
   //tcdrain(uart0_filestream);                               // the delay of this function is generally 14ms after tx not busy
   while ( 1 ){
      if (0 == UART0_TX_Busy_Flg_Check() ){
          gpio_funcSel(RPi2B_GPIO_J8_8,  GPIO_FSEL_INPUT);   // uart0 tx,gpio14. set it as input
          break;
      }
   }

   //after the tx done. the rx also done. flush the rx
   delay_ms(2);                                // ?stable? Don't delay too much to avoid flushing the return packet
   tcflush(uart0_filestream, TCIFLUSH);        // discard the inquiry cmd received

   //close(uart0_filestream);

   return 0;
}

int uart0_rcvBytes(uint8_t *ptr, uint16_t nbyte, uint8_t flag){  // UART0_RCV_FLG_NULL,

   //int uart0_filestream = -1;

   /* if (UART0_RCV_FLG_START_RCV == flag){

       tcflush(uart0_filestream, TCIFLUSH);        // discard the inquiry cmd received
   }*/

   int rx_length = read(uart0_filestream, ptr, nbyte);

   if(rx_length < 0){
      cout << ".Warning: read uart0 but no data. rx_length is:" << rx_length << endl;
      return -1;
   }
   else if ( 0 == rx_length){
      cout << "Warning: No data read back from uart0!" << endl;
      return -1;
   }
   else{
         //cout << "Info:" << nbyte << " bytes data readed from uart0." << endl;
   }

   /*if (UART0_RCV_FLG_LAST_READ == flag){
      close(uart0_filestream);
   }*/

   return 0;

}
