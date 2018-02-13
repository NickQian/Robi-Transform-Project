/*  header file
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.2.1 - Add DMA emulated PWM. refer "RPIO" - 2017.5.26 - Nick Qian
    0.2 - uart0 use POSIX interface now.- 2016.9.17 - Nick Qian
    0.1 - init version for Raspberry 2B(bcm2836)(rd/wr uart0 directly) - 2015.12.20
    ----
*/

#ifndef _PERI_H_
  #define _PERI_H_


#define RPI_V2

#include <stdint.h>

/**********************************
* 0x7E000000: Physical bus address
* 0x20000000: Pi model v1
* 0x3F000000: Pi model v2. Pi 2B is v2
* See /proc/cpuinfo  BCM2708 for v1 and BCM2709 for v2
***********************************/

#define PHY_ADDR_PERI                  0x7E000000      // CPU bus address. virtual--MMU1-> Physical --MMU2-> bus addr(doc address)

#ifdef  RPI_V1
 #define BCM283X_PERI_BASE             0x20000000      // physical address
#elif defined(RPI_V2) || defined(RPI_V3)
 #define BCM283X_PERI_BASE             0x3F000000
#else
 #error "Error:Must define RPI_V1, RPI_V2 or RPI_V3 bases on target hardware."
#endif


//---------------------------------------- MAP Offset ----------------------------------------------
#define ADDR_ST_OFFSET                        0x3000   //0x3F003000 -> 0x7E003000
#define ADDR_ST_LEN                             0x1C
#define ADDR_DMA_OFFSET                       0x7000
#define ADDR_DMA_LEN                            0x24
#define ADDR_DMAS_LEN                          0xFFF   // from CS_0 to DEBUG_14 to INT_STATUS & ENABLE
#define ADDR_DMA_INT_STATUS_OFFSET            0x7fe0
#define ADDR_DMA_ENABLE_OFFSET                0x7ff0
#define ADDR_INTC_OFFSET                      0xb200
#define ADDR_ARM_TIMER_OFFSET               0x00B000
#define ADDR_WATCHDOG_OFFSET                0x100000
#define ADDR_CLKMAG_OFFSET                  0x101000   //cprman, "Clock Power Reset Manager"
#define ADDR_CLKMAG_LEN                         0xA8
#define ADDR_RNG_OFFSET                     0x104000
#define ADDR_GPIO_OFFSET                    0x200000
#define ADDR_GPIO_LEN                          0x100
#define ADDR_UART0_OFFSET                   0x201000
#define ADDR_UART0_LEN                          0x8C  // bytes
#define ADDR_PCM_OFFSET                     0x203000
#define ADDR_PCM_LEN                            0x24
#define ADDR_SPI0_OFFSET                    0x204000
#define ADDR_BSC0_OFFSET                    0x205000   //I2C_0
#define ADDR_PWM_OFFSET                     0x20C000
#define ADDR_PWM_LEN                            0x28
#define ADDR_BSC_SPI_OFFSET                 0x214000
#define ADDR_AUX_OFFSET                     0x215000
#define ADDR_SDHCI_OFFSET                   0x300000   // EMMC
#define ADDR_BSC1_OFFSET                    0x804000   //I2C_1
#define ADDR_BSC2_OFFSET                    0x805000
#define ADDR_USB_OFFSET                     0x980000

#define ADDR_ARM_INT_OFFSET                 ADDR_ARM_TIMER_BASE
#define ADDR_I2S_OFFSET                     ADDR_PCM_OFFSET

//-------------------------------------- GPIO --------------------------------------------

//#define ADDR_GPIO_BASE  (volatile uint32_t *) 0x7E200000         //(BCM2836_PERI_BASE + ADDR_GPIO_OFFSET)      // 0x7E200000
#define GPFSEL0_OFFSET     0x0       //pin 9 -> pin 0   [29:0]/3
#define GPFSEL1_OFFSET     0x4       //pin 19-> pin 10  [29:0]/3
#define GPFSEL2_OFFSET     0x8       //pin 29-> pin 20  [29:0]/3
#define GPFSEL3_OFFSET     0xC       //pin 39-> pin 30  [29:0]/3
#define GPSET0_OFFSET      0x1C      // set GPIO pin [0..31]
#define GPSET1_OFFSET      0x20
#define GPCLR0_OFFSET      0x28
#define GPCLR1_OFFSET      0x2C
#define GPLEV0_OFFSET      0x34
#define GPLEV1_OFFSET      0x28
#define GPPUD_OFFSET       0x94      // pull-up/down enable
#define GPPUDCLK0_OFFSET   0x98      // pull-up/down clock 0
#define GPPUDCLK1_OFFSET   0x9C      // pull-up/down clock 1


typedef enum{
    GPIO_FSEL_INPUT  = 0x0,         //3'b000
    GPIO_FSEL_OUTPUT = 0x1,         //3'b001
    GPIO_FSEL_ALT0   = 0x4,         //3'b100
    GPIO_FSEL_ALT1   = 0x5,         //3'b101
    GPIO_FSEL_ALT2   = 0x6,         //3'b110
    GPIO_FSEL_ALT3   = 0x7,
    GPIO_FSEL_ALT4   = 0x3,
    GPIO_FSEL_ALT5   = 0x2,
    GPIO_FSEL_MASK   = 0x7
}GPIO_Func_Sel;


typedef enum{
    GPIO_PULL_OFF      = 0x00,
    GPIO_PULL_DOWN     = 0x01,
    GPIO_PULL_UP       = 0x02
}GPIO_PUD_Control;


typedef enum{
     RPi2B_GPIO_J8_27 = 0,
     RPi2B_GPIO_J8_28 = 1,
     RPi2B_GPIO_J8_3  = 2,
     RPi2B_GPIO_J8_5  = 3,
     RPi2B_GPIO_J8_7  = 4,
     RPi2B_GPIO_J8_29 = 5,
     RPi2B_GPIO_J8_31 = 6,
     RPi2B_GPIO_J8_26 = 7,
     RPi2B_GPIO_J8_24 = 8,
     RPi2B_GPIO_J8_21 = 9,
     RPi2B_GPIO_J8_19 = 10,
     RPi2B_GPIO_J8_23 = 11,
     RPi2B_GPIO_J8_32 = 12,      // PWM0 - ALT0
     RPi2B_GPIO_J8_33 = 13,      // PWM1 - ALT0
     RPi2B_GPIO_J8_8  = 14,      // TXD0
     RPi2B_GPIO_J8_10 = 15,      // RXD0
     RPi2B_GPIO_J8_36 = 16,
     RPi2B_GPIO_J8_11 = 17,
     RPi2B_GPIO_J8_12 = 18,
     RPi2B_GPIO_J8_35 = 19,
     RPi2B_GPIO_J8_38 = 20,
     RPi2B_GPIO_J8_40 = 21,
     RPi2B_GPIO_J8_15 = 22,
     RPi2B_GPIO_J8_16 = 23,
     RPi2B_GPIO_J8_18 = 24,
     RPi2B_GPIO_J8_22 = 25,
     RPi2B_GPIO_J8_37 = 26,
     RPi2B_GPIO_J8_13 = 27
}RPiGPIOPin;


#define LEFT_FOOT_A_PIN    RPi2B_GPIO_J8_11
#define LEFT_FOOT_B_PIN    RPi2B_GPIO_J8_12
#define RIGHT_FOOT_A_PIN   RPi2B_GPIO_J8_13
#define RIGHT_FOOT_B_PIN   RPi2B_GPIO_J8_15

//----------------------------- CM: CPRMAN - base:0x7E101000 ------------------------------------------
// see elinux.org/BCM2835_registers#CM

#define CLK_PASSWRD          (0x5A << 24)
#define CLK_GP0_CTL_OFFSET    28      // 0x70/4
#define CLK_GP0_DIV_OFFSET    29      // 0x74/4
#define CLK_GP1_CTL_OFFSET    30      // 0x78/4
#define CLK_GP1_DIV_OFFSET    31      // 0x7C/4
#define CLK_GP2_CTL_OFFSET    32      // 0x80/4
#define CLK_GP2_DIV_OFFSET    33      // 0x84/4

#define CLK_PCM_CTL_    38            // 0x98/4.
#define CLK_PCM_DIV_    39            // 0x9C/4

#define CLK_PWM_CNTL_OFFSET   40      // 0xa0/4
#define CLK_PWM_DIV_OFFSET    41      // 0xa4/4    [31:24]: password. [23:12]:integer, [11:0]: fractional part
#define CLK_UART0_CNTL_OFFSET 60      // 0xf0/4
#define CLK_UART0_DIV_OFFSET  61      // 0xf4/4


//----------------------------------------- PWM ---------------------------------------------

#define ADDR_PWM_BASE               BCM283X_PERI_BASE + ADDR_PWM_OFFSET // will be over writen after VM
#define PWM_CTRL_OFFSET             0x0
#define PWM_STA_OFFSET              0x4
#define PWM_DMAC_OFFSET             0x8
#define PWM_RNG1_OFFSET             0x10
#define PWM_DAT1_OFFSET             0x14
#define PWM_FIF1_OFFSET             0x18
#define PWM_RNG2_OFFSET             0x20
#define PWM_DAT2_OFFSET             0x24


#define PWM_CH_LEFT_FOOT_A          1                  // HW
#define PWM_CH_LEFT_FOOT_B          2                  // HW
#define PWM_CH_RIGHT_FOOT_A         3                  // DMA emu
#define PWM_CH_RIGHT_FOOT_B         4                  // DMA emu


#define PWM_FREQ                    5                                                        // 10 = 10KHz

#define OSC_ON_BOARD_FREQ           19.2                                                     // MHz
#define PWM_HW_CLK_DIV              5                                                        //OSC_DIV_1M2  19.2/16 = 1.2MHz
#define PWM_PLLD_DIV_FREQ           100                                                      // MHz
#define PWM_HW_RANGE                ( (PWM_PLLD_DIV_FREQ*1000) /PWM_FREQ )                   // ns/ns
//#define PWM_HW_WIDTH_MAX            PWM_HW_RANGE


//#define __PWM_SERIALISE_MODE__

typedef enum{                 // OSC=19.2M . Max we can get 9.6MHz
    OSC_DIV_4K388 = 4095,
    OSC_DIV_9K375 = 2048,
    OSC_DIV_18K75 = 1024,
    OSC_DIV_37K5  = 512,
    OSC_DIV_75K   = 256,
    OSC_DIV_150K  = 128,
    OSC_DIV_300K  = 64,
    OSC_DIV_600K  = 32,
    OSC_DIV_1M2   = 16,
    OSC_DIV_1M152 = 16,
    OSC_DIV_1M152_FRAC = 666                   // max 4095
}PWM_Clk_Div;



typedef enum{
    PWM_MSEN2 = 15,
    PWM_USEF2 = 13,                 // bit position
    PWM_POLA2 = 12,
    PWM_SBIT2 = 11,
    PWM_RPTL2 = 10,
    PWM_MODE2 = 9,
    PWM_PWEN2 = 8,

    PWM_MSEN1 = 7,
    PWM_CLRF1 = 6,
    PWM_USEF1 = 5,
    PWM_POLA1 = 4,
    PWM_SBIT1 = 3,
    PWM_RPTL1 = 2,
    PWM_MODE1 = 1,
    PWM_PWEN1 = 0
}PWM_Reg_CTL;


typedef enum{      // bit position
    PWM_DMAC_ENAB    = 31,
    PWM_DMAC_PANIC_H = 15,
    PWM_DMAC_DREQ_H  = 7
}PWM_Reg_DMAC;

//------------------------------ DMA (emu PWM) ---------------------------------------
#define PWM_DMA_EMU_CHANNELS     2

#define ADDR_DMAC_CH0_OFFSET     ADDR_DMA_OFFSET                 // (0x20007000)
#define ADDR_DMAC_CH1_OFFSET    (ADDR_DMAC_CH0_OFFSET+0x100)
#define ADDR_DMAC_CH2_OFFSET    (ADDR_DMAC_CH1_OFFSET+0x100)
#define ADDR_DMAC_CH3_BASE      (ADDR_DMAC_CH2_OFFSET+0x100)
#define ADDR_DMAC_CH4_BASE      (ADDR_DMAC_CH3_OFFSET+0x100)
// .... CH5 CH6 CH7 CH8 CH9 CH10 CH11 CH12 CH13

#define DMA_CH_ADDR_INC           0x100
#define DMA_CH_ADDR_LEN           0x24

#define DMA_REG_CS_OFFSET         0x00
#define DMA_CS_                  (DMA_REG_CS_OFFSET/4)
#define DMA_REG_CONBLK_AD_OFFSET  0x04
#define DMA_CONBLK_AD_           (DMA_REG_CONBLK_AD_OFFSET/4)
#define DMA_REG_DEBUG_OFFSET      0x20
#define DMA_DEBUG_               (DMA_REG_DEBUG_OFFSET/4)


// Defaultsubcycle time
#define SUBCYCLE_TIME_DEFAULT_US 20000
#define PULSE_INC_GRANU_US       10
//#define PWM_EMU_WIDTH_MAX        (SUBCYCLE_TIME_DEFAULT_US/PULSE_INC_GRANU_US -1)


typedef enum{            // bit position
    DMA_NO_WIDE_BURSTS = 26,
    DMA_WAIT_RESP      = 3,
    DMA_D_DREQ         = 6,
    DMA_END            = 1,
    DMA_RESET          = 31,
    DMA_INT            = 2
}DMA_Ctl_Reg;

#define DMA_PER_MAP(x)  ((x) << 16)


// see page 40 of the pdf manual
struct dma_cb_t{
    uint32_t info;      // TI: transfer info
    uint32_t src;       // SOURCE_AD
    uint32_t dst;       // DEST_AD
    uint32_t length;    // TXFR_LEN
    uint32_t stride;    // 2D stride mode
    uint32_t next;      // NEXTCONBK
    uint32_t pad[2];    // _reserved_
};


struct dma_page_map_t{
    uint8_t *virtaddr;
    uint32_t physaddr;
};


struct pwm_emu_ch{
    uint8_t  *virtbase;
    uint32_t *sample;
    dma_cb_t *cb;
    dma_page_map_t *page_map;
    volatile uint32_t *dma_reg;

    //set by User
    uint32_t subcycle_time_us;

    // set by system
    uint32_t num_samples;
    uint32_t num_cbs;
    uint32_t num_pages;

    // used only for control purposes
    uint32_t width_max;
};



//-----------------------------------UART0---------------------------------------------

#define UART0_USE_POSIX_INTERFACE

#define UART0_BAUD_RATE  9600
#define UART0_DRIVE_CLK  1.876 //1.92  //1.8432. div will be 1  //25.62

#define UART0_DR_OFFSET         0x0       // FIFO depth = 16
#define UART0_RSRECR_OFFSET     0x4
#define UART0_FR_OFFSET        0x18
#define UART0_IBRD_OFFSET      0x24       // Integer Baud rate divisor
#define UART0_FBRD_OFFSET      0x28       // fractional
#define UART0_LCRH_OFFSET      0x2C       // line control
#define UART0_CR_OFFSET        0x30       // control register
#define UART0_IFLS_OFFSET      0x34       // Interupt FIFO level select reg
#define UART0_IMSC_OFFSET      0x38       // Interupt mask set/clear reg
#define UART0_RIS_OFFSET       0x3C       // Raw Interupt status reg
#define UART0_MIS_OFFSET       0x40       // Masked Interupt status reg
#define UART0_ICR_OFFSET       0x44       // Interupt clear reg
#define UART0_DMACR_OFFSET     0x48       // DMA control reg
#define UART0_ITCR_OFFSET      0x80       // Integration test control reg
#define UART0_ITIP      (volatile unsigned int *)(ADDR_UART0_BASE +0x84)  //Integration test input reg
#define UART0_ITOP      (volatile unsigned int *)(ADDR_UART0_BASE +0x88)  //Integration test output reg
#define UART0_TDR       (volatile unsgined int *)(ADDR_UART0_BASE +0x8c)  //test data reg

typedef enum{
    //LRCH
    UART0_SPS     = 7,
    UART0_WLEN_H  = 6,
    UART0_WLEN_L  = 5,
    UART0_FEN     = 4,
    UART0_STP2    = 3,
    UART0_EPS     = 2,
    UART0_PEN     = 1,
    UART0_BRK     = 0,

    // CR reg
    UART0_CTSEN  = 15,
    UART0_RTSEN  = 14,
    UART0_RTS    = 11,
    UART0_RXE    = 9,
    UART0_TXE    = 8,
    UART0_LBE    = 7,                 // loop back.
    UART0_UARTEN = 0
}UART0_Regs;

typedef enum{
    UART0_CLK_DIV_9K6      = 2048,
    UART0_CLK_DIV_19K2     = 1024,
    UART0_CLK_DIV_38K4     = 512,
    UART0_CLK_DIV_57K600   = 256,
    UART0_CLK_DIV_115K2    = 10,  //19.2M/10=1.92M         //

}UART0_ClkDiv;


//----------------------------------- PCM -------------------------------------------------

#define ADDR_PCM_BASE          (BCM283X_PERI_BASE + ADDR_PCM_OFFSET)

#define PCM_CS_A_OFFSET         0x0
#define PCM_FIFO_A_OFFSET       0x04
#define PCM_MODE_A_OFFSET       0x08
#define PCM_RXC_A_OFFSET        0x0C
#define PCM_TXC_A_OFFSET        0x10
#define PCM_DREQ_A_OFFSET       0x14
#define PCM_INTEN_A_OFFSET      0x18
#define PCM_INT_STC_A_OFFSET    0x1C
#define PCM_GRAY                0x20


#define PCM_CS_A_              (PCM_CS_A_OFFSET/4)
#define PCM_FIFO_A_            (PCM_FIFO_A_OFFSET/4)
#define PCM_MODE_A_            (PCM_MODE_A_OFFSET/4)
#define PCM_RXC_A_             (PCM_RXC_A_OFFSET/4)
#define PCM_TXC_A_             (PCM_TXC_A_OFFSET/4)
#define PCM_DREQ_A_            (PCM_DREQ_A_OFFSET/4)
#define PCM_INTEN_A_           (PCM_INTEN_A_OFFSET/4)
#define PCM_INT_STC_A_         (PCM_INT_STC_A_OFFSET/4)

//------------------------------------ Macro -------------------------------------------

#define SETUP_OK           0
#define SETUP_DEVMEM_FAIL  1
#define SETUP_MALLOC_FAIL  2
#define SETUP_MMAP_FAIL    3



//extern

int      peri_init(void);
volatile uint32_t *get_pwm_vm(void);
volatile uint32_t *get_gpio_vm(void);
volatile uint32_t *get_uart0_vm(void);

void     writeAddr32(volatile uint32_t *addr, int32_t wr_data);
void     writeAddr32_nb(volatile uint32_t *addr, int32_t wr_data);
uint32_t readAddr32(volatile uint32_t *addr);
void     delay_ms(uint32_t millis);
void     short_delay(void);
void     bitSet32(volatile uint32_t *Addr, uint8_t position, uint8_t val);


uint32_t PWM_Init(void);        // contains both pwm_hw & pwm_emu, both PWM_Enable & PWM_Config
int      PWM_Enable(uint8_t ch_num);
void     PWM_Mode_Config(uint8_t ch_num);
int      PWM_Set(int ch_num, int position);
uint32_t PWM_STA_Check(void);
void     PWM_Set_Clock(uint32_t div);
void     PWM_Set_Freq(uint8_t freq_khz);
int      PWM_Emu_Ch_Init(uint32_t ch_num);
int      PWM_Emu_AddChPulse(uint32_t ch_num, int gpio, int width_start, int width);
void     PWM_Emu_Shutdown(void);


int      UART0_Init(uint32_t baudRate);
void     UART0_Set_Clock(uint32_t div);
int      UART0_Enable();
int      UART0_Config(uint32_t baudRate );
int      UART0_Send(uint8_t uartTxData);
uint8_t  UART0_rxDataRdOneByte(void);
int      UART0_STA_Check(void);
bool     UART0_TX_Busy_Flg_Check(void);

void     GPIO_Write(uint16_t pin_num, uint16_t value);
void     GPIO_Init( );
void     GPIO_FuncSel(uint8_t pin_num, uint8_t mode);
void     GPIO_Set_Clock(uint32_t div0, uint32_t div1, uint32_t div2);
void     GPIO_Set_PullUD(uint8_t pin_num, uint8_t pud);
void     GPIO_PullUD_Clk(uint8_t pin_num, uint8_t on);

void     releaseMmap();

/////////////// Internal Funcs //////////////////
static void      dma_pcm_dly_setup(void);
static uint8_t  *get_dma_cb(uint32_t ch_num);
static int       init_dma_virtbase(uint32_t ch_num);
static int       pwm_emu_MakePageMap(uint32_t ch_num);
static int       fatal(char const *fmt, ...);
static int       init_dma_cb(uint32_t ch_num);
static uint32_t  mem_virt_to_phys(uint32_t ch_num, void *virt);
static int       pwm_emu_clear_ch(uint32_t ch_num);
static void      gpio_pull_ud(uint8_t pud);

#endif
