/*
    c driver to operate peripherals on Raspberry Pi boards
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.3.- change this file to do peripheral init things. Funcs are divided into seperate files such as iic.cpp/pwm.cpp/uart0.cpp - 2017.4.28 -Nick Qian
    0.2 - find that camera is disabled by this. on pi 3(bcm2837) uart now use OS driver. reserve this for further usage -2016.9.14 -Nick Qian
    0.1 - init version for Raspberry 2B(bcm283x) - 2015.12.20 - Nick Qian
    ----
*/


#include <cstdio>
//#include <cstdint>              // C99
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <unistd.h>               // getpid()
#include <cstdarg>                // va_start()  va_end()
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <bitset>
#include <time.h>
#include <string.h>
#include "mailbox.h"

#include "peri.h"
//#include "uart0.h"

using namespace std;

#define PAGE_SIZE    (4*1024)
#define BLOCK_SIZE   (4*1024)
#define PAGE_SHIFT   12


//--------------------------------- MMAP ----------------------------------------

static void *ptr_mapresult;

//Physical Address.
static volatile uint32_t *bcm283x_peripheral_base = (uint32_t*)BCM283X_PERI_BASE;
static volatile uint32_t  bcm283x_peripheral_size = 0;

//Virtual mapped address.
static volatile uint32_t *bcm283x_peripheral_vm = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_st_vm         = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_clk_vm        = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_gpio_vm       = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_uart0_vm      = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_pwm_vm        = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_pcm_vm        = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_spi0_vm       = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_bsc0_vm       = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_bsc1_vm       = (uint32_t *)MAP_FAILED;
static volatile uint32_t *bcm283x_dma_vm        = (uint32_t *)MAP_FAILED;

//----------------------------------------------------------------------------------------
static pwm_emu_ch pwm_emu_chs[PWM_DMA_EMU_CHANNELS];

static uint16_t pulse_width_incr_us = -1;

static struct{
    int handle;
    uint8_t *virt_addr;
}mbox;

static uint32_t mem_flag = 0xc;

//-------------------------------------VM-------------------------------------------------
// map peripheral to mem
//
static void *mapmem (off_t offset, size_t size, int fd)
{
    void *map = mmap(
                     NULL,                     // Any address will do
                     size,                     // map length
                     (PROT_READ| PROT_WRITE),  //Enable Read & Write
                     MAP_SHARED,               // share with other processes
                     fd,                       // file to map
                     offset                    // offset
                     );

    if (map == MAP_FAILED ){
        printf( "bcm283x init: mmap failed." );
    }
    else{
        printf("bcm283x init: mmap success. mmap size: %d. \n", size );
    }
    return map;                                 // void *
}



int getMmap(void)
{
   int  mem_fd;
   //void *ptr_mapresult;
   int  ok;
   FILE *fp;

   void *ptr_uart0_mmp;
   void *ptr_pwm_mmap;
   void *ptr_dma_mmap; 

   // get address of peripheral from device tree
   if ( (fp = fopen("/proc/device-tree/soc/ranges", "rb")) != NULL ){
      unsigned char buf[4];
      fseek(fp, 4, SEEK_SET);                             // 4 is the RPi 2 DT_PERI_BASE_OFFSET
      if ( fread(buf, 1, sizeof(buf), fp) == sizeof(buf) ){
          //reinterpret_cast<uint32_t *>(
           bcm283x_peripheral_base = (uint32_t*)( buf[0] << 24 | buf[1] << 16 | buf[2] <<8 | buf[3] << 0 );
           cout << "@~@ bcm283x_peripheral_base is:" << showbase << hex << (uint32_t)bcm283x_peripheral_base << endl;
      }

      fseek( fp, 8, SEEK_SET);                           // 8 is the offset into DT file name
      if( fread( buf, 1, sizeof(buf), fp) == sizeof(buf) ){
           bcm283x_peripheral_size = (buf[0] <<24 | buf[1] << 16 | buf[2] <<8 | buf[3] << 0);

      }

      fclose(fp);
   }

   mem_fd = -1;   ok =0;

   // open /dev/memory device
   if ( (mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0){
       cout << "SETUP_DEVMEM_FAIL. You may need do $sudo su." << endl;
       return SETUP_DEVMEM_FAIL;
       goto exit;
   }

   bcm283x_st_vm    = (volatile uint32_t *)mapmem((uint32_t)bcm283x_peripheral_base + ADDR_ST_OFFSET,    // uint32, not pointer, no need /4
                                                  ADDR_ST_LEN,
                                                  mem_fd
                                                  );
   bcm283x_clk_vm   = (volatile uint32_t *)mapmem((uint32_t)bcm283x_peripheral_base + ADDR_CLKMAG_OFFSET,
                                                  ADDR_CLKMAG_LEN,
                                                  mem_fd
                                                 );
   bcm283x_gpio_vm  = (volatile uint32_t *)mapmem((uint32_t)bcm283x_peripheral_base + ADDR_GPIO_OFFSET,
                                                  ADDR_GPIO_LEN,
                                                  mem_fd
                                                 );
   bcm283x_uart0_vm = (volatile uint32_t *)mapmem((uint32_t)bcm283x_peripheral_base + ADDR_UART0_OFFSET,
                                                  ADDR_UART0_LEN,
                                                  mem_fd
                                                  );
   bcm283x_pwm_vm   = (volatile uint32_t *)mapmem((uint32_t)bcm283x_peripheral_base + ADDR_PWM_OFFSET,
                                                  ADDR_PWM_LEN,
                                                  mem_fd
                                                 );
   bcm283x_dma_vm   = (volatile uint32_t *)mapmem((uint32_t)bcm283x_peripheral_base + ADDR_DMA_OFFSET,
                                                  ADDR_DMAS_LEN,
                                                  mem_fd
                                                 );
   bcm283x_pcm_vm   = (volatile uint32_t *)mapmem((uint32_t)bcm283x_peripheral_base + ADDR_PCM_OFFSET,
                                                  ADDR_PCM_LEN,
                                                  mem_fd
                                                 );

   //bcm283x_spi0_vm
   //bcm283x_bsc0_vm
   //bcm283x_bsc1_vm

   if(bcm283x_st_vm== NULL || bcm283x_clk_vm==NULL  || bcm283x_gpio_vm==NULL || bcm283x_uart0_vm==NULL  || \
      bcm283x_pwm_vm==NULL || bcm283x_dma_vm==NULL){
      printf("ERR: some peripherals map failed.");
     return EXIT_FAILURE;
   }
   else{
      cout << "bcm283x_gpio_vm value is: "<< showbase  << hex << (uint32_t)bcm283x_gpio_vm << endl;
      cout << "bcm283x_pwm_vm value is:  "             << hex << (uint32_t)bcm283x_pwm_vm << endl;
      cout << "bcm283x_clk_vm value is:  "             << hex << (uint32_t)bcm283x_clk_vm << endl;
      cout << "bcm283x_dma_vm value is:  "             << hex << (uint32_t)bcm283x_dma_vm << endl;
      cout << "bcm283x_st_vm value is:  "              << hex << (uint32_t)bcm283x_st_vm << endl;
      // cout << "bcm283x_spi0_vm value is:  0x" << hex << (uint32_t)bcm283x_spi0_vm << endl;
      // cout << "bcm283x_bsc0_vm value is:  0x" << hex << (uint32_t)bcm283x_bsc0_vm << endl;
      // cout << "bcm283x_bsc1_vm value is:  0x" << hex << (uint32_t)bcm283x_bsc1_vm << endl;

      ok =1;
   }


exit:
   if (mem_fd >=0 ){
      cout << "info: will close mem_fd. mem_fd >= 0. value is:" << mem_fd << endl;
      close (mem_fd);
   }

   if(!ok)
     cout << "Something wrong. MAP failed" << endl; 

  return SETUP_OK;

}


void releaseMmap()
{

    munmap(ptr_mapresult, bcm283x_peripheral_size);
    printf("Info: peripheral Mmap released.\n");
}


volatile uint32_t *get_pwm_vm(void)
{
        return bcm283x_pwm_vm;
}


volatile uint32_t *get_gpio_vm(void)
{
    return bcm283x_gpio_vm;
}


volatile uint32_t *get_uart0_vm(void)
{
    return bcm283x_uart0_vm;
}


/*-------------------------------- common -----------------------------------------
* read/write a address
* init peri:determing pin usage/do the vm map etc.
* delay some us.
*/

void writeAddr32 (volatile uint32_t *addr, int32_t wr_data)
{
    __sync_synchronize();
    *addr = wr_data;
    __sync_synchronize();

    #ifdef   __DEBUG__
      int readValue = *addr;
      assert (readValue == wr_data );
    #endif

}



void writeAddr32_nb (volatile uint32_t *addr, int32_t wr_data){

    *addr = wr_data;

    #ifdef   __DEBUG__
      int readValue = *addr;
      assert (readValue == wr_data );
    #endif

}



uint32_t readAddr32(volatile uint32_t *addr)
{
    uint32_t rd_result;

    __sync_synchronize();
    rd_result = *addr;
    __sync_synchronize();

    return rd_result;
}



void bitSet32(volatile uint32_t *Addr, uint8_t position, uint8_t val)
{
    uint32_t wrtmp = 1 << position;
    uint32_t rdtmp = readAddr32(Addr);

    if(1 ==val){
        // *Addr |=  wrtmp;
        rdtmp |= wrtmp;
    }
    else if (0 == val){
        // *Addr &= ~wrtmp;
        rdtmp &= ~wrtmp;
    }

    writeAddr32(Addr, rdtmp);

}



/** not easy to use. val will be shifted in the func **/
void bitsSet32(volatile uint32_t *addr, uint8_t start_bit, uint8_t end_bit, uint32_t val)
{
    uint32_t v = readAddr32(addr);
    uint32_t mask = 0;

    val <<= start_bit;

    for (int i = start_bit; i < end_bit-start_bit; i++){
        mask |= 1<< i;
    }

    v = (v & ~mask) | (val & mask);
    writeAddr32(addr, v);

}


/* Sleep based us delay*/
void delay_us(uint32_t us)
{
    struct timespec sleeper = {0, long(us * 1000) };

    nanosleep(&sleeper, NULL);
}


/* This is sleep based */
void delay_ms(uint32_t ms){

   struct timespec sleeper;
   sleeper.tv_sec = (time_t)(ms / 1000);
   sleeper.tv_nsec = (long)(ms * 1000000);

   nanosleep(&sleeper, NULL);

}


/* This is busy wait */
void short_delay(void)
{
    int i;
    for (i=0; i<150; i++){
        asm volatile("nop");
    }
}


//------------------------------------------ Peri Init ------------------------------------------------------

int peri_init(void)
{
    /***************
    /*  map peripheral to mem space
    /***************
    */
    int res = 0;
    res = getMmap();

    /***************
    /* GPIO Init.
    /***************
    /*UART/I2C/SPI/PWM(hw and sim) pins not included.
    /* Include Optical encoder input/ultrasonic sensor input/drive pin/temperature sensors/ etc..
    */
    GPIO_Init();

    /***************
    /* PWM_Init
    /***************
    /* Include hardwared PWM and DMA emulated IO.
    */

    PWM_Init();

    /////// UART init ///////////
    #ifndef UART0_USE_POSIX_INTERFACE
       UART0_Init(115200);
    #endif

    /////// more init... ////////


    ////// release mmap ////////
    // releaseMmap();

    /////////////////////////////
    cout << "Info: peri_Init done." << endl;

    return res;
}





/*------------------------------------------------ PWM -------------------------------------------------
* use J8.32(GPIO 12) -> PWM0 (Pi 2B/3B )
*     J8.33(GPIO 13) -> PWM1 (Pi 2B/3B )
* use DMA emulated J8.29(GPIO 5) -> PWM 2
*                  J8.31(GPIO 6) -> PWM 3
*  for EMA emu PWM, ref: https://github.com/metachris/RPIO/blob/master/source/c_pwm/pwm.c
*/

uint32_t PWM_Init(void){

    volatile uint32_t *pwm_sta = bcm283x_pwm_vm + PWM_STA_OFFSET/4;
    uint32_t readValue;

    PWM_Set_Clock(PWM_HW_CLK_DIV);            // set PWM module drive clock(div from OSC or PLL)

    PWM_Set_Freq(PWM_FREQ);                   // set PWM signal frequency

    PWM_Enable(1);                     // hardware PWM0
    PWM_Enable(2);
    cout << "Info: PWM IO enabled. Start PWM_Config..." << endl;

    PWM_Mode_Config(1);
    PWM_Mode_Config(2);

    cout << "Info:PWM configurated, PWM_STA register read value is:0x" << hex << *pwm_sta << endl;

    PWM_Emu_Init();

    cout << "Info: <PWM_Emu_Ch_Init> done. " << endl;

    return readValue;
}



//------------------------------------ DMA emu PWM -------------------------------------------


int PWM_Emu_Init(void)
{
    if(mknod(MBFILE, S_IFCHR|0600, makedev(249, 0)) < 0)
        return fatal("Failed to creat mailbox device \n");

    mbox.handle = mbox_open();
    if(mbox.handle < 0)
        return fatal("Failed to open mailbox \n");

    PWM_Emu_Ch_Init(0);
    PWM_Emu_Ch_Init(1);
}

//---------- Init single channel --------
int PWM_Emu_Ch_Init(uint32_t ch_num)                  //, uint16_t subcycle_time_us){
{
    pwm_emu_chs[ch_num].subcycle_time_us = SUBCYCLE_TIME_DEFAULT_US;           //subcycle_time_us;
    pwm_emu_chs[ch_num].num_samples = pwm_emu_chs[ch_num].subcycle_time_us / PULSE_INC_GRANU_US;
    pwm_emu_chs[ch_num].width_max   = pwm_emu_chs[ch_num].num_samples - 1;
    pwm_emu_chs[ch_num].num_cbs     = pwm_emu_chs[ch_num].num_samples * 2;
    pwm_emu_chs[ch_num].num_pages   = (pwm_emu_chs[ch_num].num_cbs *32 + pwm_emu_chs[ch_num].num_samples *4 + \
                                              PAGE_SIZE - 1) >> PAGE_SHIFT;

    printf("<PWM_Emu_Ch_Init>: subcycle_time_us: %d, num_samples: %d, width_max:%d, num_cbs: %d, num_pages:%d. \n", pwm_emu_chs[ch_num].subcycle_time_us, \
                          pwm_emu_chs[ch_num].num_samples, pwm_emu_chs[ch_num].width_max,pwm_emu_chs[ch_num].num_cbs,pwm_emu_chs[ch_num].num_pages);


    if (init_dma_virtbase(ch_num) == EXIT_FAILURE){
        printf("ERR: <init_dma_virtbase>. ch_num is: %d .\n", ch_num);
        return EXIT_FAILURE;
    }


    //if(pwm_emu_MakePageMap(ch_num) == EXIT_FAILURE){
    //    printf("ERR: <pwm_emu_MakePageMap>. ch_num is: %d .\n", ch_num);
    //    return EXIT_FAILURE;
    //}


    if (init_dma_cb(ch_num) == EXIT_FAILURE){
        printf("ERR: <init_dma_cb>. ch_num is: %d .\n", ch_num);
        return EXIT_FAILURE;
    }

    // init HW and start DMA
    init_dma_pcm_hw(ch_num);


    printf("<PWM_Emu_Ch_Init> channel %d done. \n", ch_num);

    return EXIT_SUCCESS;
}



static void init_dma_pcm_hw(uint32_t ch_num)
{
    volatile uint32_t *pcm_reg = bcm283x_pcm_vm;
    volatile uint32_t *clk_reg = bcm283x_clk_vm;
    //volatile uint32_t *pcm_cs_a = pcm_reg + PCM_CS_A_OFFSET/4;


    pcm_reg[PCM_CS_A_]  = 1;              // disable Rx + Tx, enable PCM block
    delay_us(100);

    clk_reg[CLK_PCM_CTL_] = 0x5A000006;   // Source=PLLD(500MHz)
    delay_us(100);

    clk_reg[CLK_PCM_DIV_] = 0x5A000000 | (500<<12);  // set div=500, giving 1MHz
    delay_us(100);

    clk_reg[CLK_PCM_CTL_] = 0x5A000016;  // source=PLLD and enable
    delay_us(100);

    pcm_reg[PCM_TXC_A_] = 0<<31 | 1<< 30 | 0<<20 | 0<< 16;   // 1 channel, 8 bits
    delay_us(100);

    pcm_reg[PCM_MODE_A_] = (PULSE_INC_GRANU_US * 10 -1) << 10;
    delay_us(100);

    pcm_reg[PCM_CS_A_] |= 1<<4 | 1<<3;                   // clear FIFOs
    delay_us(100);

    pcm_reg[PCM_DREQ_A_] = 64<<24  | 64<<8;     // ??? DMA Req when one slot is free
    delay_us(100);

    pcm_reg[PCM_CS_A_] |=  1<<9;                 // enable DMA
    delay_us(100);


//  Init the DMA channel 0(p46, 47)
    pwm_emu_chs[ch_num].dma_reg[DMA_CS_] = DMA_CS_RESET; // DMA channel reset
    delay_us(100);
    pwm_emu_chs[ch_num].dma_reg[DMA_CS_] = DMA_INT | DMA_END; // write to clear Int status & DMA end flag
    pwm_emu_chs[ch_num].dma_reg[DMA_CONBLK_AD_] = mem_virt_to_phys(ch_num, get_dma_cb(ch_num)); // initial CB
    pwm_emu_chs[ch_num].dma_reg[DMA_DEBUG_] = 7;              // clear debug error flags

 
    printf("~~ after <init> DMA_CS: 0x%x, CONBLK_AD:0x%x, DEBUG:0x%x,TI:0x%x,SOURCE_AD:0x%x,DEST:0x%x, LEN:0x%x,STRIDE:0x%x,NEXTCONBK:0x%x . \n",  \
                 pwm_emu_chs[ch_num].dma_reg[DMA_CS_],      pwm_emu_chs[ch_num].dma_reg[DMA_CONBLK_AD_], pwm_emu_chs[ch_num].dma_reg[DMA_DEBUG_],  \
                 pwm_emu_chs[ch_num].dma_reg[DMA_TI_],      pwm_emu_chs[ch_num].dma_reg[DMA_SOURCE_AD_], pwm_emu_chs[ch_num].dma_reg[DMA_DEST_AD_], \
                 pwm_emu_chs[ch_num].dma_reg[DMA_TXFR_LEN_],pwm_emu_chs[ch_num].dma_reg[DMA_STRIDE_],    pwm_emu_chs[ch_num].dma_reg[DMA_NEXTCONBK_] );

    printf("~~DMA_INT_ status: 0x%x, DMA_ENABLE_ status: 0x%x  \n", pwm_emu_chs[ch_num].dma_reg[DMA_INT_],pwm_emu_chs[ch_num].dma_reg[DMA_ENABLE_]);



    // Enable PCM Tx finally
    pcm_reg[PCM_CS_A_] |=  1<<2;
    delay_us(100);

    pwm_emu_chs[ch_num].dma_reg[DMA_CS_] = 0x10880001; // 0x10880001;        // go, mid priority, wait for outstanding writes


    uint32_t tmp[200][9];
    for(int i=0; i< 200; i++){
        tmp[i][0] = pwm_emu_chs[ch_num].dma_reg[DMA_CS_];
        tmp[i][1] = pwm_emu_chs[ch_num].dma_reg[DMA_CONBLK_AD_];
        tmp[i][2] = pwm_emu_chs[ch_num].dma_reg[DMA_DEBUG_];
        tmp[i][3] = pwm_emu_chs[ch_num].dma_reg[DMA_TI_];
        tmp[i][4] = pwm_emu_chs[ch_num].dma_reg[DMA_SOURCE_AD_];
        tmp[i][5] = pwm_emu_chs[ch_num].dma_reg[DMA_DEST_AD_];
        tmp[i][6] = pwm_emu_chs[ch_num].dma_reg[DMA_TXFR_LEN_];
        tmp[i][7] = pwm_emu_chs[ch_num].dma_reg[DMA_STRIDE_];
        tmp[i][8] = pwm_emu_chs[ch_num].dma_reg[DMA_NEXTCONBK_];

        delay_us(700);
    }

    for(int i=0; i< 200; i++){
        printf("~go~ CS: 0x%x,CONBLK_AD:0x%x,DEBUG:0x%x,TI:0x%x,SRC:0x%x,DEST:0x%x,LEN:0x%x,STR:0x%x,NEXT:0x%x.\n",   \
         tmp[i][0], tmp[i][1],   tmp[i][2], tmp[i][3], tmp[i][4], tmp[i][5], tmp[i][6], tmp[i][7], tmp[i][8]);
    }


}

static uint8_t *get_dma_cb(uint32_t ch_num)
{
    uint32_t align256 = 32 - ( sizeof(uint32_t) * pwm_emu_chs[ch_num].num_samples) % 32;
    //printf("-----> align256: %d, result:%p \n", align256, pwm_emu_chs[ch_num].virtbase + (sizeof(uint32_t) * pwm_emu_chs[ch_num].num_samples + align256) );
    return pwm_emu_chs[ch_num].virtbase + (sizeof(uint32_t) * pwm_emu_chs[ch_num].num_samples + align256);
}



static int init_dma_virtbase(uint32_t ch_num)
{
    pwm_emu_chs[ch_num].mem_ref = mem_alloc(mbox.handle, pwm_emu_chs[ch_num].num_pages * PAGE_SIZE, PAGE_SIZE, mem_flag);
    if(pwm_emu_chs[ch_num].mem_ref <0)
        fatal("Failed to alloc memory from VideoCore \n");
    else
        printf("mem_ref %u \n", pwm_emu_chs[ch_num].mem_ref);

    pwm_emu_chs[ch_num].bus_addr = mem_lock(mbox.handle, pwm_emu_chs[ch_num].mem_ref);
    if(pwm_emu_chs[ch_num].bus_addr == ~0){
        mem_free(mbox.handle, pwm_emu_chs[ch_num].num_pages * PAGE_SIZE);
        fatal("Failed to lock memory \n");
    }
    else
        printf("bus_addr = 0x%x \n", pwm_emu_chs[ch_num].bus_addr);

    /*pwm_emu_chs[ch_num].virtbase = (uint8_t *) mmap(NULL,
                                                    pwm_emu_chs[ch_num].num_pages * PAGE_SIZE,   // + 256/8,    // for blank produced by 256bit aligned
                                                    PROT_READ | PROT_WRITE,
                                                    MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED,
                                                    -1,              // fd, -1 means anonymous map
                                                    0                // offset
                                                    );


    if(pwm_emu_chs[ch_num].virtbase == MAP_FAILED){
        return fatal("Error: in <init_dma_virtbase>: Failed to mmap physical pages:%m \n ");
    }
    if( (unsigned long)pwm_emu_chs[ch_num].virtbase & (PAGE_SIZE -1) ){
        return fatal("Error: In <init_dma_virtbase>: Virtual address is not page aligned. \n");
    }
    printf ("<init_dma_virtbase> mmap size: %d. pwm_emu_chs[%d].virtbase is %p. \n ", pwm_emu_chs[ch_num].num_pages * PAGE_SIZE, \
                                                             ch_num,  pwm_emu_chs[ch_num].virtbase );
    */

    pwm_emu_chs[ch_num].virtbase = (uint8_t *) mapmem(BUS_TO_PHYS(pwm_emu_chs[ch_num].bus_addr), pwm_emu_chs[ch_num].num_pages * PAGE_SIZE);
    printf ("virtbase is %p. \n ", pwm_emu_chs[ch_num].virtbase);

    return EXIT_SUCCESS;
}


static int init_dma_cb(uint32_t ch_num)
{
    dma_cb_t *cbp = (dma_cb_t *) get_dma_cb(ch_num);
    uint32_t *sample = (uint32_t *) pwm_emu_chs[ch_num].virtbase;

    uint32_t phys_fifo_addr;
    uint32_t phys_gpclr0 = BUS_ADDR_PERI + ADDR_GPIO_OFFSET + GPCLR0_OFFSET;     // int32, not pointer
    int i;

    pwm_emu_chs[ch_num].dma_reg = bcm283x_dma_vm + (DMA_CH_ADDR_INC/4)*(ch_num + PWM_DMA_EMU_CH_OFFSET);      // 32-bit pointer, need "/4" 
    printf("ch %d, dma_reg:%p \n", ch_num, pwm_emu_chs[ch_num].dma_reg);

    phys_fifo_addr = BUS_ADDR_PERI + ADDR_PCM_OFFSET + PCM_FIFO_A_OFFSET;           // physical bus

    // reset complete per-sample gpio mask to 0
    memset(sample, 0, sizeof(pwm_emu_chs[ch_num].num_samples * sizeof(uint32_t))  );   // (void *s, int c, size_t n)

    ////// init cb ////
    // each sample add 2 control blocks: 1st clear gpio and jump to 2nd. 2nd: jump to next CB
    for (int i =0; i < pwm_emu_chs[ch_num].num_samples; i++){
        cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cbp->src = mem_virt_to_phys(ch_num, sample + i);
        cbp->dst = phys_gpclr0;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(ch_num, cbp + 1);
        cbp++;

        cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
        cbp->src  = mem_virt_to_phys(ch_num, sample);           // any data
        cbp->dst  = phys_fifo_addr;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(ch_num, cbp + 1);
        cbp++;
    }

    ///// the last control block links back to the first(=endless loop)
    cbp--;
    cbp->next = mem_virt_to_phys(ch_num, get_dma_cb(ch_num) );
    printf("*** last cbp->next(to head): 0x%x, pre--: 0x%x, 0x%x, 0x%x  \n", cbp->next, (cbp-1)->next, (cbp-2)->next, (cbp-3)->next );

    return EXIT_SUCCESS;
}



static uint32_t mem_virt_to_phys(uint32_t ch_num, void *virt)
{
    uint32_t offset = (uint8_t *)virt - pwm_emu_chs[ch_num].virtbase;
    //printf("*** virt:%p, offset:0x%x, virt_to_phy result:0x%x  \n", (uint8_t *)virt, offset, \
    //  pwm_emu_chs[ch_num].page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE) );
    //return pwm_emu_chs[ch_num].page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
    return pwm_emu_chs[ch_num].bus_addr + offset;

}


/*
// Get Physical address.
static int pwm_emu_MakePageMap(uint32_t ch_num)
{
    int i, fd, memfd, pid;
    int pageSize = getpagesize();
    printf("-> system page size: %d \n", pageSize);

    char pagemap_fn[64];

    pwm_emu_chs[ch_num].page_map = (dma_page_map_t*) malloc(pwm_emu_chs[ch_num].num_pages * sizeof(*pwm_emu_chs[ch_num].page_map) );
    if (pwm_emu_chs[ch_num].page_map == 0){
        return fatal("Error: <pwm_emu_MakePageMap> Failed to malloc page_map: %m\n");
    }

    //???
    memfd = open("/dev/mem", O_RDWR);
    if (memfd < 0){
        return fatal("Error: <pwm_emu_MakePageMap>: Failed to open /dev/mem: %m. you may need to do 'sudo su'  \n");
    }

    pid = getpid();
    sprintf(pagemap_fn, "/proc/%d/pagemap", pid);              // "/proc/self/pagemap"
    printf("-> pid: %d \n", pid);

    fd = open(pagemap_fn, O_RDONLY);
    if (fd < 0){
        return fatal("Error: <pwm_emu_MakePageMap>: Failed to open %s: %m\n", pagemap_fn);
    }

    if (lseek(fd, (uint32_t)pwm_emu_chs[ch_num].virtbase >> 9, SEEK_SET) != (uint32_t)pwm_emu_chs[ch_num].virtbase >> 9) {
        return fatal("Error: <pwm_emu_MakePageMap>: Failed to seek on %s: %m\n", pagemap_fn);
    }

    virtToPhys(pwm_emu_chs[ch_num].virtbase);

    for (i = 0; i < pwm_emu_chs[ch_num].num_pages; i++) {
        uint64_t pfn;
        pwm_emu_chs[ch_num].page_map[i].virtaddr = pwm_emu_chs[ch_num].virtbase + i * PAGE_SIZE;

        // Following line forces page to be allocated
        pwm_emu_chs[ch_num].page_map[i].virtaddr[0] = 0;

        if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
            return fatal("Error: <pwm_emu_MakePageMap>: Failed to read %s: %m\n", pagemap_fn);
        if (((pfn >> 55) & 0x1bf) != 0x10c)
            return fatal("Error: <pwm_emu_MakePageMap>: Page %d not present (pfn 0x%016llx)\n", i, pfn);

        pwm_emu_chs[ch_num].page_map[i].physaddr = (uint32_t) (pfn << PAGE_SHIFT) & ~0xC0000000;
        printf("~~~physaddr: 0x%x \n", pwm_emu_chs[ch_num].page_map[i].physaddr);

        virtToPhys(pwm_emu_chs[ch_num].page_map[i].virtaddr);
    }

    close(fd);
    close(memfd);
    return EXIT_SUCCESS;

}
*/


static uint32_t  virtToPhys(void *virt)
{
    int pagemapfd = open("/proc/self/pagemap", O_RDONLY);
    int pid = getpid();
    uintptr_t pgNum = (uintptr_t)(virt)/PAGE_SIZE;

    uint64_t physPage;

    int err = lseek(pagemapfd, pgNum*8, SEEK_SET);
    if(err != pgNum*8)
        printf("WARNING: virtToPhys %p failed to seek. \n", virt);

    read(pagemapfd, &physPage, 8);

    if(!physPage & (1ull << 63) )
        printf("Warning: virtToPhys %p has no physical address \n", virt);

    physPage = physPage & ~(0x1ffull << 55);    // 55-63 are flags
    uintptr_t mapped = (uintptr_t)physPage * PAGE_SIZE | 0x40000000;

    cout << "===> <virtToPhys> result:" << showbase << mapped << endl;

    close(pagemapfd);
}


//------------------ end Init things ------------------

int PWM_Emu_AddChPulse(uint32_t ch_num, int gpio, int width_start, int width)
{
    const uint32_t phys_gpset0 = BUS_ADDR_PERI + ADDR_GPIO_OFFSET + GPSET0_OFFSET;
    const uint32_t phys_gpclr0 = BUS_ADDR_PERI + ADDR_GPIO_OFFSET + GPCLR0_OFFSET;


    dma_cb_t *cbp = (dma_cb_t *) get_dma_cb(ch_num) + (width_start * 2);
    uint32_t *dp = (uint32_t *) pwm_emu_chs[ch_num].virtbase;
    printf("<AddChPulse> ch_num: %d, width:%d, width_start:%d. cbp:%p, dp:%p \n",  ch_num, width, width_start, cbp, dp);

    if (!pwm_emu_chs[ch_num].virtbase)
        return fatal("Error: emu pwm channel %d has not been initialized. \n", ch_num);

    // En or disable GPIO at this point in the cycle
    *(dp + width_start) |= 1 << gpio;
    cbp->dst = phys_gpset0;

    if( width_start+width > pwm_emu_chs[ch_num].width_max+1  ||  width_start<0){
        printf("Warning: cannot add pulse to channel %d: width_start(%d)+width(%d) exceed max_width or width_start<0. will set to 0 and max. \n", ch_num,width_start,width);
        width_start = 0;
        width = pwm_emu_chs[ch_num].width_max - 100;  /****** !!! 100 for debug  ****/
    }

    pwm_emu_sta_check(ch_num, (uint32_t)gpio, (uint32_t *)cbp, dp);      // for debug

    // Do nothing for the specified width
    for (int i=1; i<width-1; i++){
        *(dp + width_start + i) &= ~(1 << gpio);          // set this bit to 0
        cbp += 2;
        cbp->dst = phys_gpclr0;
    }

    // clear gpio at end
    *(dp + width_start + width) |= 1 << gpio;
    cbp->dst = phys_gpclr0;

    //pwm_emu_sta_check(ch_num, (uint32_t)gpio, (uint32_t *)cbp, dp);  // for debug
    printf("---------------- add pulse done. ---------------- \n");

    return EXIT_SUCCESS;
}


void pwm_emu_sta_check(uint32_t ch_num, uint32_t pin_num, uint32_t *cbp, uint32_t *dp)
{
    volatile uint32_t *gpset0_1 = bcm283x_gpio_vm + GPSET0_OFFSET/4 + pin_num/32;  // if >32, will access to GPSEL1
    volatile uint32_t *gpclr0_1 = bcm283x_gpio_vm + GPCLR0_OFFSET/4 + pin_num/32;
    volatile uint32_t *gpfsel0  = bcm283x_gpio_vm + GPFSEL0_OFFSET/4;

    volatile uint32_t *pcm_reg = bcm283x_pcm_vm;
    volatile uint32_t *clk_reg = bcm283x_clk_vm;

    printf("<emu_check> ch:%d: CS:0x%x,CONBLK_AD:0x%x,DEBUG:0x%x, TI:0x%x,SOURCE_AD:0x%x,DEST_AD:0x%x, TXFR_LEN:0x%x,STRIDE:0x%x,NEXTCONBK:0x%x, \n", ch_num, \
         pwm_emu_chs[ch_num].dma_reg[DMA_CS_],      pwm_emu_chs[ch_num].dma_reg[DMA_CONBLK_AD_], pwm_emu_chs[ch_num].dma_reg[DMA_DEBUG_], \
         pwm_emu_chs[ch_num].dma_reg[DMA_TI_],      pwm_emu_chs[ch_num].dma_reg[DMA_SOURCE_AD_], pwm_emu_chs[ch_num].dma_reg[DMA_DEST_AD_], \
         pwm_emu_chs[ch_num].dma_reg[DMA_TXFR_LEN_],pwm_emu_chs[ch_num].dma_reg[DMA_STRIDE_],    pwm_emu_chs[ch_num].dma_reg[DMA_NEXTCONBK_] );

//    printf("~ gpset0:0x%x, gpclr0:0x%x, gpfsel0:0x%x, pin5:0x%x, pin6:0x%x  \n", *gpset0_1, *gpclr0_1, *gpfsel0, (*gpfsel0 >> 18)&0x7, (*gpfsel0 >> 15)&0x7 );

//    cout << "~~~ cb1/cb2/cb3... " << hex << showbase   \
//          << *cbp     << "|" << *(cbp+1) << "|"  << *(cbp+2) << "|"  << *(cbp+3) << "|" << *(cbp+4)  << "|" << *(cbp+5)  << "|"   \
//          << *(cbp+8) << "|" << *(cbp+9) << "|"  << *(cbp+10)<< "|"  << *(cbp+11)<< "|" << *(cbp+12) << "|" << *(cbp+13) << "|"   \
//          << *(cbp+16)<< "|" << *(cbp+17)<< "|"  << *(cbp+18)<< "|"  << *(cbp+19)<< "|" << *(cbp+20) << "|" << *(cbp+21) << "|"   \
//          << *(cbp+24)<< "|" << *(cbp+25)<< "|"  << *(cbp+26)<< "|"  << *(cbp+27)<< "|" << *(cbp+28) << "|" << *(cbp+29) << "|"   \
//          << *(cbp+32)<< "|" << *(cbp+33)<< "|"  << *(cbp+34)<< "|"  << *(cbp+35)<< "|" << *(cbp+36) << "|" << *(cbp+37) << "|"   \
//          << *(cbp+40)<< "|" << *(cbp+41)<< "|"  << *(cbp+42)<< "|"  << *(cbp+43)<< "|" << *(cbp+44) << "|" << *(cbp+45) << "|"   \
//          << *(cbp+48)<< "|" << *(cbp+49)<< "|"  << *(cbp+50)<< "|"  << *(cbp+51)<< "|" << *(cbp+52) << "|" << *(cbp+53) << "|"   \
//          << *(cbp+56)<< "|" << *(cbp+57)<< "|"  << *(cbp+58)<< "|"  << *(cbp+59)<< "|" << *(cbp+60) << "|" << *(cbp+61) << "|"   \
//
//          << "| data:" << *dp  << "|" << *(dp+1)  << "|"  << *(dp+2)  << "|"   \
//                  << *(dp+pwm_emu_chs[ch_num].num_samples-2) << "|" << *(dp+pwm_emu_chs[ch_num].num_samples-1) \
//          << endl;


    printf("~~~ PCM_CS_A:0x%x, PCM_TXC_A:0x%x, PCM_MODE_A:0x%x, PCM_DREQ_A:0x%x. \n", pcm_reg[PCM_CS_A_],pcm_reg[PCM_TXC_A_],pcm_reg[PCM_MODE_A_],pcm_reg[PCM_DREQ_A_]);

}


static int pwm_emu_clear_ch(uint32_t ch_num)
{
    const uint32_t phys_gpclr0 = ADDR_GPIO_OFFSET + GPCLR0_OFFSET + BUS_ADDR_PERI;

    dma_cb_t *cbp = (dma_cb_t *)get_dma_cb(ch_num);
    uint32_t *dp = (uint32_t *)pwm_emu_chs[ch_num].virtbase;

    for (int i=0; i<pwm_emu_chs[ch_num].num_samples; i++){
        cbp->dst = phys_gpclr0;
        cbp += 2;
    }

    delay_us(pwm_emu_chs[ch_num].subcycle_time_us);

   // set all samples to 0(instead of gpio_mask)
    for (int i=0; i < pwm_emu_chs[ch_num].num_samples; i++){
        *(dp+i) = 0;
    }

    return EXIT_SUCCESS;
}



static int pwm_emu_clear_ch_gpio(uint32_t ch_num, int gpio)
{
    uint32_t *dp = (uint32_t *) pwm_emu_chs[ch_num].virtbase;

    // remove this gpio from all samples;
    for (int i=0; i < pwm_emu_chs[ch_num].num_samples; i++){
        *(dp + i) &= ~(1 << gpio);                              // just set the bit to 0
    }

    GPIO_Write((uint16_t)gpio, 0);
    return EXIT_SUCCESS;
}



static int fatal(char const *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);

    PWM_Emu_Shutdown();

    exit(EXIT_FAILURE);
}



void PWM_Emu_Shutdown(void)
{
    for (int i=0; i<PWM_DMA_EMU_CHANNELS; i++){
        pwm_emu_clear_ch(i);
        delay_us(pwm_emu_chs[i].subcycle_time_us);

        pwm_emu_chs[i].dma_reg[DMA_CS_] = DMA_CS_RESET;
        delay_us(10);

        if(pwm_emu_chs[i].virtbase != NULL){
            unmapmem(pwm_emu_chs[i].virtbase, pwm_emu_chs[i].num_pages * PAGE_SIZE);
            mem_unlock(mbox.handle, pwm_emu_chs[i].mem_ref);
            mem_free(mbox.handle, pwm_emu_chs[i].mem_ref);
            if(mbox.handle >= 0){
                mbox_close(mbox.handle);
                printf("<PWM_Emu_Shutdown>: mbox closed. \n");
            }
        }
    } // for
    exit(EXIT_FAILURE);
}



//------------------------------------------ PWM Hardware ------------------------------------------------

int PWM_Enable(uint8_t ch_num)
{
    volatile uint32_t *gpfsel1;
    gpfsel1 = bcm283x_gpio_vm + GPFSEL1_OFFSET/4;

    if (ch_num==1)   {
       GPIO_FuncSel( RPi2B_GPIO_J8_32, GPIO_FSEL_ALT0);
    }
    else if (ch_num==2) {
       GPIO_FuncSel( RPi2B_GPIO_J8_33, GPIO_FSEL_ALT0);
    }

    cout << "<PWM_Enable>: set IO usage to PWM done. GPFSEL1 read value is:" << hex << *gpfsel1 <<endl;

    return 0;
}



void PWM_Mode_Config(uint8_t ch_num)
{
    /////////PWM CTRL////////////
    volatile uint32_t *pwm_ctrl = bcm283x_pwm_vm + PWM_CTRL_OFFSET/4;
    volatile uint32_t *pwm_dmac  = bcm283x_pwm_vm + PWM_DMAC_OFFSET/4; 

    if (ch_num == 2){

        bitSet32(pwm_ctrl, PWM_MSEN2, 1);     //MSEN2.  0: N(data)/M(range). 1: M/S(range) transmission. No use in Serial mode
        short_delay();

        bitSet32(pwm_ctrl, PWM_PWEN2, 1);     //PWEN2: Channel 2 Enable
        short_delay();

        bitSet32(pwm_ctrl, PWM_POLA2, 0);     // 1: 1==low; 0: 0==low

       #ifdef __PWM_SERIALISE_MODE__

        bitSet32(pwm_ctrl, PWM_MODE2, 1);     // 1: Serial mode; 0: PWM mode
        short_delay();

        bitSet32(pwm_ctrl, PWM_USEF2, 1);     // 1: use FIFO. 0: use Data register
        short_delay();

        bitSet32(pwm_ctrl, PWM_SBIT2, 0);     // 1: keep 1 when no data need to transfer
        short_delay();

        bitSet32(pwm_ctrl, PWM_RPTL2, 0);     // 1:repeat last fifo data. 0:stop transmission when fifo is empty.
        short_delay();

       #else
        bitSet32(pwm_ctrl, PWM_MODE2, 0);     // 1: Serial mode; 0: PWM mode
        short_delay();

        bitSet32(pwm_ctrl, PWM_USEF2, 0);     // 1: use FIFO. 0: use Data register
        short_delay();
       #endif

    }
    else if(ch_num == 1){

        bitSet32(pwm_ctrl, PWM_MSEN1, 1);   //MSEN1. 1 as M/S trasmission. 0 as N/M trasmission.  No use in Serial mode.
        short_delay();

        bitSet32(pwm_ctrl, PWM_PWEN1, 1);   // ch 1 enable
        short_delay();

        bitSet32(pwm_ctrl, PWM_POLA1, 0);
        short_delay();

       #ifdef __PWM_SERIALISE_MODE__

        bitSet32(pwm_ctrl, PWM_MODE1, 1);   // 1: Serial mode
        short_delay();

        bitSet32(pwm_ctrl, PWM_USEF1, 1);   // 1: use FIFO
        short_delay();

        bitSet32(pwm_ctrl, PWM_SBIT1, 0);   // 1: keep 1 when no data need to transfer
        short_delay();

        bitSet32(pwm_ctrl, PWM_RPTL1, 0);   // 1:repeat last fifo data. 0:stop transmission when fifo is empty.
        short_delay();

       #else
        bitSet32(pwm_ctrl, PWM_MODE1, 0);   // 0: PWM mode
        short_delay();

        bitSet32(pwm_ctrl, PWM_USEF1, 0);   // 0: use data reg
        short_delay();

       #endif
    }

    cout << "<PWM_Mode_Config>: PWM_CTRL read value is:" << showbase << hex << *pwm_ctrl <<endl;

}



void PWM_Set_Clock(uint32_t div)    // "PWM clock and freq is controlled in CPRMAN"
{
    volatile uint32_t *pwm_clkcntl = bcm283x_clk_vm + CLK_PWM_CNTL_OFFSET/4;    // raspbian default: 0x96(running & using 500MHz PLLD)
    volatile uint32_t *pwm_clkdiv  = bcm283x_clk_vm + CLK_PWM_DIV_OFFSET/4;     // raspbian default: 0x5000(div=5)
    div &= 0xfff;


    // *pwm_clkdiv  = CLK_PASSWRD | (div << 12); // set divider
    //// *pwm_clkcntl = CLK_PASSWRD | 0x11;        // source=osc and enable

    cout << "<PWM_Set_Clock> done: CPRMAN pwm_clkcntl read::" << hex << *pwm_clkcntl << ". pwm_clkdiv read:" << hex << *pwm_clkdiv << endl;
}




void PWM_Set_Freq(uint8_t freq_khz)
{
    volatile uint32_t *pwm0_reg_range = bcm283x_pwm_vm + PWM_RNG1_OFFSET/4;
    volatile uint32_t *pwm1_reg_range = bcm283x_pwm_vm + PWM_RNG2_OFFSET/4;

    volatile uint32_t *pwm_dat1 = bcm283x_pwm_vm + PWM_DAT1_OFFSET/4;
    volatile uint32_t *pwm_dat2 = bcm283x_pwm_vm + PWM_DAT2_OFFSET/4;


    uint32_t pwm_hw_range  = ( PWM_PLLD_DIV_FREQ * 1000 ) / freq_khz;

    //writeAddr32_nb (pwm0_reg_range, pwm_hw_range);
    writeAddr32_nb (pwm0_reg_range, pwm_hw_range);   //0x4E20=20000
    //writeAddr32_nb (pwm1_reg_range, pwm_hw_range);
    writeAddr32_nb (pwm1_reg_range, pwm_hw_range);

    writeAddr32(pwm_dat1, pwm_hw_range/3);
    writeAddr32(pwm_dat2, pwm_hw_range/3);

    printf("<PWM_Set_Freq>> pwm_dat1 addr: %p", pwm_dat1);
    cout << "<PWM_Set_Freq>, pwm_hw_range is:" << hex << pwm_hw_range << ". Read RNG1: "  \
         << hex <<  *pwm0_reg_range << ". Read RNG2:" << *pwm1_reg_range << ". Read Dat1: " << *pwm_dat1 << ". Read Dat2:" <<  *pwm_dat2  << endl;

}



uint32_t PWM_STA_Check(void)
{
    volatile uint32_t *pwm_sta = bcm283x_pwm_vm + PWM_STA_OFFSET/4;
    //   uint32_t readValue;
    //   readValue = *pwm_sta;
    return *pwm_sta; //readValue;

}



//-------------UART0(not "mini UART" which is "UART 1" resides in Auxiliary I/O)---------------------
// use GPIO 14 as TXD0 -> J8.8  on Pi 2B board.
//     GPIO 15 as RXD0 -> J8.10 on Pi 2B board.
// This is the UART0(PL011 UART) which is 16c650 compatiple. also have CTS0 and RTS0

int UART0_Init(uint32_t baudRate)
{
    UART0_Set_Clock(UART0_CLK_DIV_115K2 );
    UART0_Enable();
    UART0_Config(baudRate);
    return 0;
}



void UART0_Set_Clock(uint32_t div)    // "PWM clock and freq is controlled in CPRMAN"
{
     volatile uint32_t *uart0_clkcntl = bcm283x_clk_vm + CLK_UART0_CNTL_OFFSET/4; 
     volatile uint32_t *uart0_clkdiv  = bcm283x_clk_vm + CLK_UART0_DIV_OFFSET/4;
     div &= 0xfff;

     *uart0_clkdiv  = CLK_PASSWRD | (div << 12); // set divider
     *uart0_clkcntl = CLK_PASSWRD | 0x11;         // source=osc and enable
     cout << "<UART0_Set_Clock> CPRMAN uart0 clkcntl read value is:" << hex << *uart0_clkcntl <<  endl;
     cout << "<UART0_Set_Clock> CPRMAN uart0_clkdiv value is:" << hex << *uart0_clkdiv << endl;
}



int UART0_Enable(void)
{
     GPIO_FuncSel(RPi2B_GPIO_J8_8,  GPIO_FSEL_ALT0);   // uart0 tx,gpio14
     GPIO_FuncSel(RPi2B_GPIO_J8_10, GPIO_FSEL_ALT0);   // rx,      gpio15

    return 0;
}



int UART0_Config(uint32_t baudRate )
{
    volatile uint32_t *uart0_ck_cntl = bcm283x_clk_vm + CLK_UART0_CNTL_OFFSET/4;
    volatile uint32_t *uart0_ck_div  = bcm283x_clk_vm + CLK_UART0_DIV_OFFSET/4;

    volatile uint32_t *uart0_ctrl = bcm283x_uart0_vm + UART0_CR_OFFSET/4;
    volatile uint32_t *uart0_lcrh = bcm283x_uart0_vm + UART0_LCRH_OFFSET/4;
    volatile uint32_t *uart0_ibrd = bcm283x_uart0_vm + UART0_IBRD_OFFSET/4;
    volatile uint32_t *uart0_fbrd = bcm283x_uart0_vm + UART0_FBRD_OFFSET/4;
    volatile uint32_t *uart0_ifls = bcm283x_uart0_vm + UART0_IFLS_OFFSET/4;
    volatile uint32_t *uart0_itcr = bcm283x_uart0_vm + UART0_ITCR_OFFSET/4;


    bitSet32(uart0_ctrl, UART0_CTSEN, 0);   //
    //delay_ms(1);
    short_delay();

    bitSet32(uart0_ctrl, UART0_RTSEN, 0);   //
    short_delay();

    bitSet32(uart0_ctrl, UART0_UARTEN, 1); 
    short_delay();

    bitSet32(uart0_ctrl, UART0_TXE, 1);
    short_delay();

    bitSet32(uart0_ctrl, UART0_RXE, 1);
    short_delay();

   //---------------Line control. Don't change during TXing------------- 

    bitSet32(uart0_lcrh, UART0_STP2,   0);             //1: 2 stop bit
    short_delay();

    *uart0_lcrh &= ~(0x7);                              //UART0_EPS UART0_PEN UART0_BRK  //No break, No parity(set to Even), 2 stop bit
    short_delay();

    bitSet32(uart0_lcrh, UART0_FEN,    1);             //Enable FIFO
    short_delay();

    bitSet32(uart0_lcrh, UART0_WLEN_H, 1);             //2'b11, set to 8bit
    short_delay();

    bitSet32(uart0_lcrh, UART0_WLEN_L, 1);
    short_delay();

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

   return 0;
}


int UART0_Send(uint8_t uartTxData)
{
	volatile uint32_t *uart0_dr = bcm283x_uart0_vm + UART0_DR_OFFSET/4;
        volatile uint32_t *uart0_fr = bcm283x_uart0_vm + UART0_FR_OFFSET/4;

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


unsigned char UART0_rxDataRdOneByte(void)
{
        volatile uint32_t *uart0_dr = bcm283x_uart0_vm + UART0_DR_OFFSET/4;
	volatile uint32_t *uart0_rsrecr = bcm283x_uart0_vm + UART0_RSRECR_OFFSET/4;
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
                cout << "===> get rx data DR read value is:" << hex << rx_Data32 << endl;
		rx_Data = (uint8_t)rx_Data32;
                cout << "rx_Data is:" << hex << (uint8_t)rx_Data32 << endl;
	}

        return rx_Data;
}


int UART0_STA_Check(void)
{
     volatile uint32_t *gpfsel1      = bcm283x_gpio_vm  + GPFSEL1_OFFSET/4;
     volatile uint32_t *uart0_fr     = bcm283x_uart0_vm + UART0_FR_OFFSET/4;
     volatile uint32_t *uart0_ris    = bcm283x_uart0_vm + UART0_RIS_OFFSET/4;
     volatile uint32_t *uart0_rsrecr = bcm283x_uart0_vm + UART0_RSRECR_OFFSET/4;

     bitset<32> readSta  = *uart0_fr;
     bool RxFifoFull = (readSta[6] ==1 );

  cout << "UART0_STA_Checking...GPFSEL1 readvalue is:" << hex << *gpfsel1 << endl;
  cout << "--UART0_FR readvalue is:" << hex << *uart0_fr << endl;
  cout << "--UART0_RIS readvalue is:" << hex << *uart0_ris << endl;
  cout << "--UART0_RSRECR readvalue is:" << hex << *uart0_rsrecr << endl;

   if (RxFifoFull)  return  1;
}


bool UART0_TX_Busy_Flg_Check(void)
{
   volatile uint32_t *uart0_fr     = bcm283x_uart0_vm + UART0_FR_OFFSET/4;

   bitset<32> readSta  = *uart0_fr;
   bool TxLineBusy = (readSta[3] ==1 );

   return TxLineBusy;
}



/*****************************************************************************************
/*   GPIO
/****************************************************************************************/

void GPIO_Write(uint16_t pin_num, uint16_t value)
{
    volatile uint32_t *gpset0_1 = bcm283x_gpio_vm + GPSET0_OFFSET/4 + pin_num/32;  // if >32, will access to GPSEL1
    volatile uint32_t *gpclr0_1 = bcm283x_gpio_vm + GPCLR0_OFFSET/4 + pin_num/32;

    uint16_t shift = pin_num % 32;
    uint32_t wrValue = 0;
    wrValue = 1<< shift;

    if (1==value){
      *gpset0_1 |= wrValue; //(1 << shift);
    }
    else if (0==value){
      *gpclr0_1 |= wrValue; // (1 << shift);
    }
}



void GPIO_Init(void)
{
   volatile uint32_t *gpfsel0 = bcm283x_gpio_vm + GPFSEL0_OFFSET/4;
   volatile uint32_t *gpset0  = bcm283x_gpio_vm + GPSET0_OFFSET/4;
   volatile uint32_t *gpclr0  = bcm283x_gpio_vm + GPCLR0_OFFSET/4;
   uint32_t readValue;


    //// Motor Optical Sens "1A & 2A".
    GPIO_FuncSel(RPi2B_GPIO_J8_11, GPIO_FSEL_INPUT );   // "1A"
    GPIO_FuncSel(RPi2B_GPIO_J8_13, GPIO_FSEL_INPUT );   // "2A"
    GPIO_Set_PullUD(RPi2B_GPIO_J8_11, GPIO_PULL_UP);
    GPIO_Set_PullUD(RPi2B_GPIO_J8_13, GPIO_PULL_UP);

    ///// Motor Emu DMA PWM //////////////
    GPIO_FuncSel(RPi2B_GPIO_J8_29,  GPIO_FSEL_OUTPUT);
    GPIO_FuncSel(RPi2B_GPIO_J8_31,  GPIO_FSEL_OUTPUT);
    GPIO_Write(RPi2B_GPIO_J8_29, 0);
    GPIO_Write(RPi2B_GPIO_J8_29, 0);

    //// Ultrasonic /////////
    //GPIO_FuncSel(RPi2B_GPIO_J8_16, GPIO_FSEL_OUTPUT);  // gpio 5 as ultrasonic trig
    //GPIO_FuncSel(RPi2B_GPIO_J8_18, GPIO_FSEL_INPUT );  // gpio 6 as ultrasonic result echo


    //// Dust sensor DSM501A //////
    //GPIO_FuncSel(RPi2B_GPIO_J8_11, GPIO_FSEL_INPUT);   //
    //GPIO_FuncSel(RPi2B_GPIO_J8_12, GPIO_FSEL_INPUT);   //


    //// I2C - SDA1/SCL1 (Compass, six-axis, ) //////
    GPIO_FuncSel(RPi2B_GPIO_J8_3, GPIO_FSEL_ALT0);    // gpio 2 as I2C SDA1 to XunFei XFM10411
    GPIO_FuncSel(RPi2B_GPIO_J8_5, GPIO_FSEL_ALT0);    // gpio 3 as I2C SCL1

     GPIO_FuncSel(RPi2B_GPIO_J8_7, GPIO_FSEL_ALT0);    // GPIO 4 as INT Input
     GPIO_Set_PullUD(RPi2B_GPIO_J8_7, GPIO_PULL_UP);


     //// GPCLK for test
     GPIO_FuncSel(RPi2B_GPIO_J8_38, GPIO_FSEL_ALT5);   // GPCLK0  gpio-20
     //GPIO_FuncSel(RPi2B_GPIO_J8_40, GPIO_FSEL_ALT5);   // GPCLK1  gpio-21
     //GPIO_FuncSel(RPi2B_GPIO_J8_31, GPIO_FSEL_ALT0);   // GPCLK2  gpio-60


     // Set GPIO clock
     GPIO_Set_Clock(OSC_DIV_1M2, OSC_DIV_150K, OSC_DIV_300K);


     //GPIO_Write(RPi2B_GPIO_J8_31, 1);  //execute 80ns
     //GPIO_Write(RPi2B_GPIO_J8_31, 0);
     //GPIO_Write(RPi2B_GPIO_J8_31, 1);
     //GPIO_Write(RPi2B_GPIO_J8_31, 0);

     cout << "Info:GPIO_Init() done. PSEL0 read value:" << hex << *gpfsel0 << endl;

}


void GPIO_FuncSel(uint8_t pin_num, uint8_t mode)
{
     volatile uint32_t *gpfsel  = bcm283x_gpio_vm + GPFSEL0_OFFSET/4 + (pin_num/10); //every sel 10 pin      uint8_t shift = (pin_num % 10)*3;
     uint8_t shift = (pin_num % 10) * 3;
     uint32_t mask = GPIO_FSEL_MASK << shift;  // shift mask to the position
     uint32_t func_value = mode << shift;
     uint32_t orgValue = *gpfsel;
     orgValue &= ~mask;                        // clear the postions required
     orgValue |= (func_value & mask);          // &mask for safe
     *gpfsel = orgValue;
}



void GPIO_Set_Clock(uint32_t div0, uint32_t div1, uint32_t div2 )
{
     volatile uint32_t *gpclk0_cntl = bcm283x_clk_vm + CLK_GP0_CTL_OFFSET/4;   // raspbian default: 0x200(hardware reset default. no clock)
     volatile uint32_t *gpclk1_cntl = bcm283x_clk_vm + CLK_GP1_CTL_OFFSET/4;   // raspbian default: 0x96(running and use PLLD 500MHz)
     volatile uint32_t *gpclk2_cntl = bcm283x_clk_vm + CLK_GP2_CTL_OFFSET/4;   // raspbian default: 0x0(not used by raspbian)
     volatile uint32_t *gpclk0_div  = bcm283x_clk_vm + CLK_GP0_DIV_OFFSET/4;   // raspbian default: 0x0
     volatile uint32_t *gpclk1_div  = bcm283x_clk_vm + CLK_GP1_DIV_OFFSET/4;   // raspbian default: 0x14000
     volatile uint32_t *gpclk2_div  = bcm283x_clk_vm + CLK_GP2_DIV_OFFSET/4;   // raspbian default: 0x0


     div0 &= 0xfff;
     div1 &= 0xfff;
     div2 &= 0xfff;


     *gpclk0_div  = CLK_PASSWRD | (div0 << 12);
     // *gpclk1_div  = CLK_PASSWRD | (div1 << 12);
     // *gpclk2_div  = CLK_PASSWRD | (div2 << 12);

     *gpclk0_cntl = CLK_PASSWRD | 0x11;  // source=osc and enable
     // *gpclk1_cntl = CLK_PASSWRD | 0x11;  // source=osc and enable  // enable this line will cause board shutdown. default the pin will output 25MHz signal
     // *gpclk2_cntl = CLK_PASSWRD | 0x11;  // source=osc and enable

     cout << "<GPIO_Set_Clock>: set done! CPRMAN gp_clk0_cntl read:" << hex << *gpclk0_cntl << ". gpclk0_div read:" << *gpclk0_div \
                                           << ". gp_clk1_cntl read:" << hex << *gpclk1_cntl << ". gpclk1_div read:" << *gpclk1_div \
                                           << ". gp_clk2_cntl read:" << hex << *gpclk2_cntl << ". gpclk2_div read:" << *gpclk2_div << endl;

}


/*******************************************************************************
/* pud is:   GPIO_PULL_OFF  = 0x00,
/*           GPIO_PULL_DOWN = 0x01,
/*           GPIO_PULL_UP   = 0x02
/********************************************************************************/
void GPIO_Set_PullUD(uint8_t pin_num, uint8_t pud)
{
    gpio_pull_ud(pud);
    short_delay();

    GPIO_PullUD_Clk(pin_num, 1);   // clock the value of pud into pin
    short_delay();

    gpio_pull_ud(GPIO_PULL_OFF);
    GPIO_PullUD_Clk(pin_num, 0);

}


/****************************************
/* Sub func of <GPIO_Set_PullUD>
/****************************************/
static void gpio_pull_ud(uint8_t pud)
{
    volatile uint32_t *paddr = bcm283x_gpio_vm + GPPUD_OFFSET/4;
    writeAddr32(paddr, pud);

}


void GPIO_PullUD_Clk(uint8_t pin_num, uint8_t on)
{
    volatile uint32_t *paddr = bcm283x_gpio_vm + GPPUDCLK0_OFFSET/4 + pin_num/32;
    uint8_t shift = pin_num % 32;
    writeAddr32(paddr, (on ? 1 : 0) << shift);

}
