#include <stdio.h>
#include <stdint.h>           // C99 int8_t, uint8_t, int16_t, uint16_t;  int32_t; uint32_t int64_t;  uint64_t
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <bitset>
#include <time.h>

#include "peri.h"

using namespace std;

#define PAGE_SIZE    (4*1024)
#define BLOCK_SIZE   (4*1024)

//----------------------- MMAP ------------------------------
//Physical Address. Will be overridden on RPi2
static volatile uint32_t *bcm2836_peripheral_base = (uint32_t*)BCM2836_PERI_BASE;
static volatile uint32_t  bcm2836_peripheral_size = 4096;
//Virtual mapped address.
static volatile uint32_t *bcm2836_peripheral_vm = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_st_vm         = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_clk_vm        = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_gpio_vm       = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_uart0_vm      = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_pwm_vm        = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_spi0_vm       = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_bsc0_vm       = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm2836_bsc1_vm       = (uint32_t *)MAP_FAILED;


//----------------------VM-----------------------------------
// map peripheral to mem
//
static void *mapmem ( size_t size, int fd, off_t offset){
    void *map = mmap(
                     NULL,                     // Any address will do
                     size,                     // map length
                     (PROT_READ| PROT_WRITE),  //Enable Read & Write
                     MAP_SHARED,               // share with other processes
                     fd,                       // file to map
                     offset                    // offset
                     );

    if (map == MAP_FAILED )    cout << "BCM2836 init: mmap failed." << endl;
   return map;                                 // void *
}

int getMmap(void){
   int  mem_fd;
   void *ptr_mapresult;
   int  ok;
   FILE *fp;

   // get address of peripheral from device tree
   if ( (fp = fopen("/proc/device-tree/soc/ranges", "rb")) != NULL ){
      unsigned char buf[4];
      fseek(fp, 4, SEEK_SET);                             // 4 is the RPi 2 DT_PERI_BASE_OFFSET
      if ( fread(buf, 1, sizeof(buf), fp) == sizeof(buf) ){
          //reinterpret_cast<uint32_t *>(
           bcm2836_peripheral_base = (uint32_t*)( buf[0] << 24 | buf[1] << 16 | buf[2] <<8 | buf[3] << 0 );
      }

      fseek( fp, 8, SEEK_SET);                           // 8 is the offset into DT file name 
      if( fread( buf, 1, sizeof(buf), fp) == sizeof(buf) ){
           bcm2836_peripheral_size = (buf[0] <<24 | buf[1] << 16 | buf[2] <<8 | buf[3] << 0);

      }

      fclose(fp);
   }

   mem_fd = -1;   ok =0;

   // open /dev/memory device
   if ( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0){
       cout << "SETUP_DEVMEM_FAIL" << endl;
       return SETUP_DEVMEM_FAIL; 
       goto exit;
   }

   //map to VM            // *mapmem (size_t size, int fd, off_t offset)
   ptr_mapresult = (uint32_t *)mapmem (
                                       (bcm2836_peripheral_size ),          //map length
                                       mem_fd,                           //file
                                       (uint32_t)bcm2836_peripheral_base //offset. must be physical address?
                                       );

   bcm2836_peripheral_vm = (volatile uint32_t *)ptr_mapresult; 


   if (bcm2836_peripheral_vm == MAP_FAILED ){
      cout << "XXX MAP_FAILED" << endl;
      goto exit;
   }

   // computer the base address in VM
   bcm2836_st_vm     = bcm2836_peripheral_vm + ADDR_ST_OFFSET/4;
   bcm2836_clk_vm    = bcm2836_peripheral_vm + ADDR_CLOCK_OFFSET/4;
   bcm2836_gpio_vm   = bcm2836_peripheral_vm + ADDR_GPIO_OFFSET/4;
   bcm2836_uart0_vm  = bcm2836_peripheral_vm + ADDR_UART0_OFFSET/4;
   bcm2836_pwm_vm    = bcm2836_peripheral_vm + ADDR_PWM_OFFSET/4;
   bcm2836_spi0_vm   = bcm2836_peripheral_vm + ADDR_SPI0_OFFSET/4;
   bcm2836_bsc0_vm   = bcm2836_peripheral_vm + ADDR_BSC0_OFFSET/4;
   bcm2836_bsc1_vm   = bcm2836_peripheral_vm + ADDR_BSC1_OFFSET/4;


   cout << "bcm2836_st_vm value is:  0x" << hex << (uint32_t)bcm2836_st_vm << endl;
   cout << "bcm2836_clk_vm value is:  0x" << hex << (uint32_t)bcm2836_clk_vm << endl;
   cout << "bcm2836_gpio_vm value is: 0x" << hex << (uint32_t)bcm2836_gpio_vm << endl;
   cout << "bcm2836_pwm_vm value is:  0x" << hex << (uint32_t)bcm2836_pwm_vm << endl;
//   cout << "bcm2836_spi0_vm value is:  0x" << hex << (uint32_t)bcm2836_spi0_vm << endl;
//   cout << "bcm2836_bsc0_vm value is:  0x" << hex << (uint32_t)bcm2836_bsc0_vm << endl;
//   cout << "bcm2836_bsc1_vm value is:  0x" << hex << (uint32_t)bcm2836_bsc1_vm << endl;

   ok =1;

exit:
   if (mem_fd >=0 ){
      //close (memfd);
      cout << "mem_fd >= 0. value is:" << mem_fd << endl;
   }

   if(!ok)
     cout << "Something wrong. MAP failed" << endl; 

  return SETUP_OK;

}

//----------------------common-------------------------
// read/write a address
// init peri:determing pin usage/do the vm map etc.
// delay some us.

int writeAddr32 (volatile uint32_t *addr, int32_t wr_data){
  *addr = wr_data;

  #ifdef   __DEBUG__
      int readValue = *addr; 
      assert (readValue != wr_data );

      if (readValue != wr_data)
           return 1;
      else
           return 0;
  #endif
}

int readAddr32(volatile uint32_t *addr){
  int rd_result;
  rd_result = *addr;
  return rd_result;
}

int peri_Init(uint32_t baudrate_spi,uint32_t baudrate_uart1, uint32_t baudrate_pwmser){
   // map firstly
   int res = 0;
   res = getMmap();

   GPIO_Init();           // set which pins as IO
   PWM_Init(1);           // enable PWM | cofig serail/pwm mode, use fifo
   UART0_Init(115200);
  // more init...


   return res;
}

void delay_ns(unsigned int millis){
   struct timespec sleeper;
   sleeper.tv_sec = (time_t)(millis / 1000);
   sleeper.tv_nsec = (long)(millis % 1000) * 1000000;
//cout << "jump into sleep...millis is " << millis  << endl;
   nanosleep(&sleeper, NULL);
//cout << "<- jump out sleep!" << endl;
}

void bitSet32(volatile uint32_t *Addr, uint8_t position, uint8_t val){
     uint32_t wrtmp = 1 << position;
 //    cout << "teeeest shift: uint32_t wrtmp value is:0x" << hex << wrtmp << endl;
     if(1 ==val){
        *Addr |=  wrtmp;
     }
     else if (0 == val){
        *Addr &= ~wrtmp;
     }
}
//------------------------ PWM ------------------------------
// use GPIO 12 as PWM0 -> J8.32 on Pi 2B board 
//       GPIO 13 as PWM0 -> J8.33 on Pi 2B board
//

uint32_t PWM_Init(int ch_num){

   volatile uint32_t *pwm_sta = bcm2836_pwm_vm + PWM_STA_OFFSET/4;
   uint32_t readValue;

   PWM_Set_Clock(PWM_CLK_DIV_1M2);      //(PWM_CLK_DIV_9K375);

   PWM_Enable(ch_num);
   cout << "...PWM IO enabled. Start PWM_Config..." << endl;

   PWM_Config(ch_num);

   readValue = *pwm_sta;
   cout << "...PWM configurated, PWM_STA register read value is:0x" << hex << readValue << endl;

   return readValue;
}

void PWM_Set_Clock(uint32_t div){    // "PWM clock and freq is controlled in CPRMAN"
     volatile uint32_t *pwm_clkcntl = bcm2836_clk_vm + CLK_PWM_CNTL_OFFSET;  //??? 
     volatile uint32_t *pwm_clkdiv  = bcm2836_clk_vm + CLK_PWM_DIV_OFFSET;
     div &= 0xfff;

     *pwm_clkdiv  = CLK_PASSWRD | (div << 12); // set divider  
     *pwm_clkcntl = CLK_PASSWRD | 0x11;  // source=osc and enable
      cout << "$$ CPRMAN clkcntl read value is 0x" << hex << *pwm_clkcntl << ". pwm_clkdiv value is 0x" << hex << *pwm_clkdiv << endl;
}


int PWM_Enable(int ch_num){      //set GPIO12 as PWM func
  #ifdef __CPU_RPI2B__
     assert(ch_num>2);
  #endif

    volatile uint32_t *gpfsel1;
    gpfsel1 = bcm2836_gpio_vm + GPFSEL1_OFFSET/4;
    cout << "GPFSEL1 read linux default value is:0x" << hex << *gpfsel1 <<endl;
 
  if (ch_num==1)   {
      gpio_funcSel( RPi2B_GPIO_J8_32,GPIO_FSEL_ALT0);
      }
  else if (ch_num==2) {
      gpio_funcSel( RPi2B_GPIO_J8_33,GPIO_FSEL_ALT0);
    }

  cout << "Set IO as PWM. GPFSEL1 read value is:0x" << hex << *gpfsel1 <<endl;

   #ifdef   __DEBUG__
       unsigned int readValue = *gpfsel1;
       unsigned int readValueShiftL = readValue<<11;
       unsigned int readValueShiftR = readValueShiftL>>29;
       assert(readValueShiftR !=3);
   #endif


  return 0;
}


void PWM_Config(int ch_num){
      /////////PWM CTRL////////////
        volatile uint32_t *pwm_ctrl = bcm2836_pwm_vm + PWM_CTRL_OFFSET/4;
        volatile uint32_t *pwm_dmac  = bcm2836_pwm_vm + PWM_DMAC_OFFSET/4; 

	if (ch_num == 2){
  	    *pwm_ctrl &= ~(1<<15);    //MSEN2.  0: PWM algorithm = N(data)/M(range). 1: M/S(range) transmission. No use in Serial mode
            *pwm_ctrl |=  1<<8;       //PWEN2: Channel 2 Enable
            *pwm_ctrl &= ~(1<<12);    //POLA2: Channel 2 Polarity
          #ifdef __PWM_SERIALISE_MODE__
	    *pwm_ctrl |=  1<<9;       //MODE2 :Serial mode.
	    *pwm_ctrl |=  1<<13;      // use FIFO
          #else
	    *pwm_ctrl &= ~(1<<9);     //MODE2
	    *pwm_ctrl &= ~(1<<13);    //USEF2.  0: use Data register. 1: use FIFO
          #endif

	}
	else if(ch_num == 1){
            bitSet32(pwm_ctrl, MSEN1, 0);   //MSEN1. 1 as M/S trasmission. 0 as N/M trasmission.  No use in Serial mode.   
delay_ns (1);
            bitSet32(pwm_ctrl, PWEN2, 0);   // disable ch 2 for FIFO
delay_ns (1);
            bitSet32(pwm_ctrl, PWEN1, 1);   // ch 1 enable
delay_ns (1);
            bitSet32(pwm_ctrl, POLA1, 0);
delay_ns (1);
          #ifdef __PWM_SERIALISE_MODE__
            bitSet32(pwm_ctrl, MODE1, 1);   // 1: Serial mode
delay_ns (1);
            bitSet32(pwm_ctrl, USEF1, 1);   // 1: use FIFO
delay_ns (1);
            bitSet32(pwm_ctrl, SBIT1, 0);   // 1: keep 1 when no data need to transfer
delay_ns (1);
            bitSet32(pwm_ctrl, RPTL1, 0);   // 1:repeat last fifo data. 0:stop transmission when fifo is empty.
delay_ns (1);
          #else
	    bitSet32(pwm_ctrl, MODE1, 0);   //0: PWM mode
delay_ns (1);
            bitSet32(pwm_ctrl, USEF1, 0);   // 0: use data reg
delay_ns (1);
          #endif
	}


       cout << "PWM_CTRL read value is:0x" << hex << *pwm_ctrl <<endl;
//       cout << "PWM_DMAC read value is 0x" << hex << *pwm_dmac << endl; 
       /////////PWM_STA default set////////////
       /////////PWM_DMAC default set///////////
       volatile uint32_t *pwm_rng1 = bcm2836_pwm_vm + PWM_RNG1_OFFSET/4;
       volatile uint32_t *pwm_dat1 = bcm2836_pwm_vm + PWM_DAT1_OFFSET/4;
       volatile uint32_t *pwm_rng2 = bcm2836_pwm_vm + (PWM_RNG2_OFFSET/4);
       volatile uint32_t *pwm_dat2 = bcm2836_pwm_vm + (PWM_RNG2_OFFSET/4); 

       *pwm_rng1 = 0x20;           // default is 32
       // *pwm_dat1 = 0xFAAFAAAA; //PWM_POS_L + 5 * ((PWM_POS_R - PWM_POS_L)/10) ;       //the number of pulses which is sent within the period defined by PWM_RNGi       
       // *pwm_rng2 = 0xFAAF;//PWM_RNG_VALUE;  
       // *pwm_dat2 = 0xFFFF ;//PWM_POS_L + 5 * ((PWM_POS_R - PWM_POS_L)/10) ;     


      cout << "PWM_RNG1 read value is:0x" << hex << *pwm_rng1 <<endl;
     // cout << "PWM_DAT1 read value is:0x" << hex << *pwm_dat1 <<endl;
     // cout << "PWM_RNG2 read value is:0x" << hex << *pwm_rng2 <<endl;
     // cout << "PWM_DAT2 read value is:0x" << hex << *pwm_dat2 <<endl;


   #ifdef   __DEBUG__
     bitset<32> readvalue(0x0);
     readvalue = *pwm_ctrl;
     cout << "PWWM_CTRL value is: " << readvalue <<end1;
     cout << "PWM->MSEN2 value is: " << readvalue[15] << end1;
     cout << "PWM->USEF2 value is: " << readvalue[13] << end1;
     cout << "PWM->POLA2 value is: " << readvalue[12] << end1;
     cout << "PWM->MODE2 value is: " << readvalue[9] << end1;
     cout << "PWM->PWEN2 value is: " << readvalue[8] << end1;
     cout << "PWM->MSEN1 value is: " << readvalue[7] << end1;
     cout << "PWM->USEF1 value is: " << readvalue[5] << end1;
     cout << "PWM->POLA1 value is: " << readvalue[4] << end1;
     cout << "PWM->MODE1 value is: " << readvalue[1] << end1;
     cout << "PWM->PWEN1 value is: " << readvalue[0] << end1;
   #endif

} 


int PWM_Set(int ch_num, int position){
     volatile uint32_t *pwm_rng1 = bcm2836_pwm_vm + PWM_RNG1_OFFSET/4;
     volatile uint32_t *pwm_dat1 = bcm2836_pwm_vm + PWM_DAT1_OFFSET/4;
     volatile uint32_t *pwm_rng2 = bcm2836_pwm_vm + PWM_RNG2_OFFSET/4;
     volatile uint32_t *pwm_dat2 = bcm2836_pwm_vm + PWM_RNG2_OFFSET/4;

   if(ch_num==1){
      *pwm_rng1 = PWM_RNG_VALUE;
      *pwm_dat1 = PWM_POS_L + position* ((PWM_POS_R - PWM_POS_L)/10) ;  
	  return 0;
   }
   else if(ch_num==2){
      *pwm_rng2 = PWM_RNG_VALUE;   
      *pwm_dat2 = PWM_POS_L + position* ((PWM_POS_R - PWM_POS_L)/10) ;  
	  return 0;
   }
   else
      return 1;

   cout << "PWM_RNG1 set done. Read value now is:0x" << hex << *pwm_rng1 <<endl;
   cout << "PWM_RNG2 set done. Read value now is:0x" << hex << *pwm_rng2 <<endl;
   cout << "PWM_DAT1 set done. Read Value now is:0x" << hex << *pwm_dat1 <<endl;
   cout << "PWM_DAT2 set donw. Read value now is:0x" << hex << *pwm_dat2 <<endl;

}


void PWM_Ser_WriteFIFO(uint32_t WrData){
   volatile  uint32_t *pwm_fif1 = bcm2836_pwm_vm + PWM_FIF1_OFFSET/4;

  *pwm_fif1 = WrData;
}

uint32_t PWM_Ser_ReadFIFO( ){
   volatile uint32_t *pwm_fif1 = bcm2836_pwm_vm + PWM_FIF1_OFFSET/4;
   uint32_t readResult;

  return readResult = *pwm_fif1;  
}

uint32_t PWM_STA_Check(){
   volatile uint32_t *pwm_sta = bcm2836_pwm_vm + PWM_STA_OFFSET/4;
//   uint32_t readValue;
//   readValue = *pwm_sta;
   return *pwm_sta; //readValue;

}


//-------------UART0(not "mini UART" which is "UART 1" resides in Auxiliary I/O)---------------------
// use GPIO 14 as TXD0 -> J8.8  on Pi 2B board.
//     GPIO 15 as RXD0 -> J8.10 on Pi 2B board.
// This is the UART0(PL011 UART) which is 16c650 compatiple. also have CTS0 and RTS0

int UART0_Init(uint32_t baudRate){

    UART0_Set_Clock(UART0_CLK_DIV_115K2 );
   UART0_Enable();
   UART0_Config(baudRate);
   return 0;
}

void UART0_Set_Clock(uint32_t div){    // "PWM clock and freq is controlled in CPRMAN"
     volatile uint32_t *uart0_clkcntl = bcm2836_clk_vm + CLK_UART0_CNTL_OFFSET; 
     volatile uint32_t *uart0_clkdiv  = bcm2836_clk_vm + CLK_UART0_DIV_OFFSET;
     div &= 0xfff;

     *uart0_clkdiv  = CLK_PASSWRD | (div << 12); // set divider
     *uart0_clkcntl = CLK_PASSWRD | 0x11;         // source=osc and enable
      cout << "$$ CPRMAN uart0 clkcntl read value is 0x" << hex << *uart0_clkcntl <<  endl;
      cout << "$$ CPRMAN uart0_clkdiv value is 0x" << hex << *uart0_clkdiv << endl;
}


int UART0_Enable(){ 

     gpio_funcSel(RPi2B_GPIO_J8_8,  GPIO_FSEL_ALT0);   // uart0 tx,gpio14
     gpio_funcSel(RPi2B_GPIO_J8_10, GPIO_FSEL_ALT0);   // rx,      gpio15

    return 0;
}

int UART0_Config(uint32_t baudRate ){
    volatile uint32_t *uart0_ck_cntl = bcm2836_clk_vm + CLK_UART0_CNTL_OFFSET;
    volatile uint32_t *uart0_ck_div  = bcm2836_clk_vm + CLK_UART0_DIV_OFFSET;

    volatile uint32_t *uart0_ctrl = bcm2836_uart0_vm + UART0_CR_OFFSET/4;
    volatile uint32_t *uart0_lcrh = bcm2836_uart0_vm + UART0_LCRH_OFFSET/4;
    volatile uint32_t *uart0_ibrd = bcm2836_uart0_vm + UART0_IBRD_OFFSET/4;
    volatile uint32_t *uart0_fbrd = bcm2836_uart0_vm + UART0_FBRD_OFFSET/4;
    volatile uint32_t *uart0_ifls = bcm2836_uart0_vm + UART0_IFLS_OFFSET/4;
    volatile uint32_t *uart0_itcr = bcm2836_uart0_vm + UART0_ITCR_OFFSET/4;


    bitSet32(uart0_ctrl, UART0_CTSEN, 0);   //
delay_ns(1);

    bitSet32(uart0_ctrl, UART0_RTSEN, 0);   //
delay_ns(1);

    bitSet32(uart0_ctrl, UART0_UARTEN, 1); 
delay_ns (1);

    bitSet32(uart0_ctrl, UART0_TXE, 1);
delay_ns (1);

    bitSet32(uart0_ctrl, UART0_RXE, 1);
delay_ns (1);

   //---------------Line control. Don't change during TXing------------- 

    bitSet32(uart0_lcrh, UART0_STP2,   0);             //1: 2 stop bit
delay_ns (1);
   *uart0_lcrh &= ~(0x7);                              //UART0_EPS UART0_PEN UART0_BRK  //No break, No parity(set to Even), 2 stop bit
delay_ns (1);
    bitSet32(uart0_lcrh, UART0_FEN,    1);             //Enable FIFO
delay_ns (1);
    bitSet32(uart0_lcrh, UART0_WLEN_H, 1);             //2'b11, set to 8bit
delay_ns (1);
    bitSet32(uart0_lcrh, UART0_WLEN_L, 1);
delay_ns (1);
    bitSet32(uart0_lcrh, UART0_SPS,    0);             //disable stick parity


    //--------------------Clk DIV---------------------------------
    double uart_clk_freq = UART0_DRIVE_CLK * 1000000;   //M;

    double baudDiv = uart_clk_freq/(16 * baudRate);
    unsigned int baudDivInt = int(baudDiv);
    double baudDivFrac = baudDiv - baudDivInt;

    *uart0_ibrd = (int)baudDiv;                     //Integer Baud rate divisor [15:0]    
    *uart0_fbrd = (int)(baudDivFrac*100);           //Fractional[5:0]


    //------------------ UART0_IFLS-----------------------------
    //011: 3/4 full;    010: 1/2 full     001:1/4 full   000:1/8 full
    bitSet32(uart0_ifls, 5,    0);
    bitSet32(uart0_ifls, 4,    1);
    bitSet32(uart0_ifls, 3,    1);

    bitSet32(uart0_ifls, 2,    0);
    bitSet32(uart0_ifls, 1,    1);
    bitSet32(uart0_ifls, 0,    1);
    

    // UART0_ITCR
    // UART0_ITIP
    // UART0_ITOP
    // UART0_TDR
    cout << "$$ UART_CK_CTL read value is 0x" << hex << *uart0_ck_cntl << endl;
    cout << "$$ UART_CK_DIV read value is 0x" << hex << *uart0_ck_div << endl;
    cout << "$$ UART0_CTRL read value is :0x" << hex << *uart0_ctrl << endl;
    cout << "$$ UART0_LCRH read value is :0x" << hex << *uart0_lcrh << endl;
    cout << "$$ UART0_IBRD read value is :0x" << hex << *uart0_ibrd << endl;
    cout << "$$ UART0_FBRD read value is :0x" << hex << *uart0_fbrd << endl;
    cout << "$$ UART0_IFLS read value is :0x" << hex << *uart0_ifls << endl;
    cout << "$$ UART0_ITCR read value is :0x" << hex << *uart0_itcr << endl;

   return 0;
}

int UART0_Send(uint8_t uartTxData){
	volatile uint32_t *uart0_dr = bcm2836_uart0_vm + UART0_DR_OFFSET/4;
        volatile uint32_t *uart0_fr = bcm2836_uart0_vm + UART0_FR_OFFSET/4;

        bitset<32> readUART0_FR = *uart0_fr;
        bool  TxFIFOFull = (readUART0_FR[5]==1);
	if ( TxFIFOFull){
                cout << "!!! UART0 FIFO Full when write in" << endl; 
		return 1;
        }
	else{
		*uart0_dr = uartTxData;                   //will be pushed to FIFO
	        return 0;
	}
}

unsigned char UART0_rxDataRdOneByte( ){
        volatile uint32_t *uart0_dr = bcm2836_uart0_vm + UART0_DR_OFFSET/4;
	volatile uint32_t *uart0_rsrecr = bcm2836_uart0_vm + UART0_RSRECR_OFFSET/4;
        uint32_t rx_Data32 = 0;
        uint8_t  rx_Data   = 0;

	bitset<32> readUART0_RSRECR  = *uart0_rsrecr;
	bool FramingError = (readUART0_RSRECR[0] == 1);
	bool ParityError  = (readUART0_RSRECR[1] == 1);
        bool BreakError   = (readUART0_RSRECR[2] == 1);

	if (FramingError|ParityError|BreakError)
		cout << "UART RX data Erro occurs: [FramingError,ParityError,BreakError] = " <<  FramingError <<  ParityError << BreakError << endl;
	else{
		rx_Data32 =  *uart0_dr;
                cout << "===> get rx data DR read value is 0x" << hex << rx_Data32 << endl;
		rx_Data = (uint8_t)rx_Data32;
                cout << "rx_Data is 0x" << hex << (uint8_t)rx_Data32 << endl;
	}

        return rx_Data;
}

int UART0_STA_Check(){
     volatile uint32_t *gpfsel1      = bcm2836_gpio_vm  + GPFSEL1_OFFSET/4;
     volatile uint32_t *uart0_fr     = bcm2836_uart0_vm + UART0_FR_OFFSET/4;
     volatile uint32_t *uart0_ris    = bcm2836_uart0_vm + UART0_RIS_OFFSET/4;
     volatile uint32_t *uart0_rsrecr = bcm2836_uart0_vm + UART0_RSRECR_OFFSET/4;

     bitset<32> readSta  = *uart0_fr;
     bool RxFifoFull = (readSta[6] ==1 );

  cout << "UART0_STA_Checking...GPFSEL1 readvalue is: 0x" << hex << *gpfsel1 << endl;
  cout << "--UART0_FR readvalue is: 0x" << hex << *uart0_fr << endl;
  cout << "--UART0_RIS readvalue is: 0x" << hex << *uart0_ris << endl;
  cout << "--UART0_RSRECR readvalue is: 0x" << hex << *uart0_rsrecr << endl;

   if (RxFifoFull)  return  1;
}

//------------------GPIO------------------------- 
// IO for sensors
//

void gpio_write(uint16_t pin_num, uint16_t value){   //??? why write 2 pins at hardware during debug? 
    volatile uint32_t *gpset0_1 = bcm2836_gpio_vm + GPSET0_OFFSET/4 + pin_num/32;  // if >32, will access to GPSEL1
    volatile uint32_t *gpclr0_1 = bcm2836_gpio_vm + GPCLR0_OFFSET/4 + pin_num/32;
    uint16_t shift = pin_num % 32;
    uint32_t wrValue = 0;
    wrValue = 1<< shift;

// cout << "$$$ in gpio_write(). shift = " << shift << ". value = " << hex << value << endl;
    if (1==value){
      *gpset0_1 |= wrValue; //(1 << shift);
    }
    else if (0==value){
      *gpclr0_1 |= wrValue; // (1 << shift);
    }
}

void GPIO_Init( ){
   volatile uint32_t *gpfsel0 = bcm2836_gpio_vm + GPFSEL0_OFFSET/4;
   volatile uint32_t *gpset0  = bcm2836_gpio_vm + GPSET0_OFFSET/4;
   volatile uint32_t *gpclr0  = bcm2836_gpio_vm + GPCLR0_OFFSET/4;
   uint32_t readValue;

     gpio_funcSel(RPi2B_GPIO_J8_7,  GPIO_FSEL_OUTPUT);  //gpio 4 as output to test

     //// Ultrasonic
     gpio_funcSel(RPi2B_GPIO_J8_29, GPIO_FSEL_OUTPUT);  // gpio 5 as ultrasonic trig
     gpio_funcSel(RPi2B_GPIO_J8_31, GPIO_FSEL_INPUT );  // gpio 6 as ultrasonic result echo
     //// Compass
     gpio_funcSel(RPi2B_GPIO_J8_27, GPIO_FSEL_ALT0);   // gpio 0 as I2C SDA0 to HMC5883
     gpio_funcSel(RPi2B_GPIO_J8_28, GPIO_FSEL_ALT0);   // gpio 1 as I2C SCL0
     //// Dust sensor DSM501A
     gpio_funcSel(RPi2B_GPIO_J8_11, GPIO_FSEL_INPUT);   // gpio 17 as I2C SDA0 to HMC5883
     gpio_funcSel(RPi2B_GPIO_J8_12, GPIO_FSEL_INPUT);   // gpio 18 as I2C SCL0
     //// Micorphone Array
     gpio_funcSel(RPi2B_GPIO_J8_7, GPIO_FSEL_OUTPUT);  // gpio 4 as 'WAKEUP' to Xunfei XFM10411
     gpio_funcSel(RPi2B_GPIO_J8_3, GPIO_FSEL_ALT0);    // gpio 2 as I2C SDA1 to XunFei XFM10411
     gpio_funcSel(RPi2B_GPIO_J8_5, GPIO_FSEL_ALT0);    // gpio 3 as I2C SCL1


     gpio_funcSel(RPi2B_GPIO_J8_38, GPIO_FSEL_ALT5);   // GPCLK0
     gpio_funcSel(RPi2B_GPIO_J8_40, GPIO_FSEL_ALT5);   // GPCLK1
    // gpio_funcSel(RPi2B_GPIO_J8_32, GPIO_FSEL_OUTPUT );

     GPIO_Set_Clock(PWM_CLK_DIV_1M2, PWM_CLK_DIV_1M2,PWM_CLK_DIV_1M2);                  // ?

     cout << "GPIO_Init()...PSEL0 read value: 0x" << hex << *gpfsel0 << endl;

     gpio_write(RPi2B_GPIO_J8_31, 1);  //execute 80ns
     gpio_write(RPi2B_GPIO_J8_31, 0);
     gpio_write(RPi2B_GPIO_J8_31, 1);  
     gpio_write(RPi2B_GPIO_J8_31, 0);
     gpio_write(RPi2B_GPIO_J8_31, 1);  
     gpio_write(RPi2B_GPIO_J8_31, 0);
     gpio_write(RPi2B_GPIO_J8_31, 1);  

}

void gpio_funcSel(uint8_t pin_num, uint8_t mode){
     volatile uint32_t *gpfsel  = bcm2836_gpio_vm + GPFSEL0_OFFSET/4 + (pin_num/10); //every sel 10 pin      uint8_t shift = (pin_num % 10)*3;
     uint8_t shift = (pin_num % 10) * 3;
     uint32_t mask = GPIO_FSEL_MASK << shift;  // shift mask to the position
     uint32_t func_value = mode << shift;
     uint32_t orgValue = *gpfsel;
     orgValue &= ~mask;                        // clear the postions required
     orgValue |= (func_value & mask);          // &mask for safe
     *gpfsel = orgValue; 
}

void GPIO_Set_Clock(uint32_t div0, uint32_t div1, uint32_t div2 ){    // "PWM clock and freq is controlled in CPRMAN"
     volatile uint32_t *gpclk0_cntl = bcm2836_clk_vm + CLK_GP0_CTL_OFFSET;
     volatile uint32_t *gpclk1_cntl = bcm2836_clk_vm + CLK_GP1_CTL_OFFSET;
     volatile uint32_t *gpclk2_cntl = bcm2836_clk_vm + CLK_GP2_CTL_OFFSET;
     volatile uint32_t *gpclk0_div  = bcm2836_clk_vm + CLK_GP0_DIV_OFFSET;
     volatile uint32_t *gpclk1_div  = bcm2836_clk_vm + CLK_GP1_DIV_OFFSET;
     volatile uint32_t *gpclk2_div  = bcm2836_clk_vm + CLK_GP2_DIV_OFFSET;


     div0 &= 0xfff;  
     div1 &= 0xfff;
     div2 &= 0xfff;

    cout << "$$ CPRMAN gp_clk0_cntl read value is 0x" << hex << *gpclk0_cntl << ". gpclk0_div  value is 0x" << hex <<  *gpclk0_div << endl;

     *gpclk0_div  = CLK_PASSWRD | (div0 << 12);
//     *gpclk1_div  = CLK_PASSWRD | (div1 << 12); 
//     *gpclk2_div  = CLK_PASSWRD | (div2 << 12);  
     *gpclk0_cntl = CLK_PASSWRD | 0x11;  // source=osc and enable
//     *gpclk1_cntl = CLK_PASSWRD | 0x11;  // source=osc and enable
//     *gpclk2_cntl = CLK_PASSWRD | 0x11;  // source=osc and enable 

   cout << "$$ CPRMAN gp_clk0_cntl read value is 0x" << hex << *gpclk0_cntl << ". gpclk0_div value is 0x" << hex << *gpclk0_div << endl;

}

