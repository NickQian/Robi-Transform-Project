#include <stdint.h>

#define BCM2836_PERI_BASE             0x3F000000  // 0x20000000   //0x7E000000

#define ADDR_ST_OFFSET                        0x3000   //0x7E003000
#define ADDR_DMA_OFFSET                       0x7000
#define ADDR_INTC_OFFSET                      0xb200
#define ADDR_WATCHDOG_OFFSET                0x100000
#define ADDR_CLOCK_OFFSET                   0x101000   //cprman, aka "Clock Power Reset Manager"
#define ADDR_RNG_OFFSET                     0x104000
#define ADDR_GPIO_OFFSET                    0x200000   //
#define ADDR_UART0_OFFSET                   0x201000
#define ADDR_I2S_OFFSET                     0x203000
#define ADDR_SPI0_OFFSET                    0x204000
#define ADDR_BSC0_OFFSET                    0x205000   //I2C_0
#define ADDR_PWM_OFFSET                     0x20C000
#define ADDR_SDHCI_OFFSET                   0x300000
#define ADDR_BSC1_OFFSET                    0x804000   //I2C_1
#define ADDR_BSC2_OFFSET                    0x805000


#define ADDR_ARM_TIMER_BASE                 (0x2000B000)
#define ADDR_ARM_INT_BASE   ADDR_ARM_TIMER_BASE


#define ADDR_GPIO_BASE  (volatile uint32_t *) 0x7E200000         //(BCM2836_PERI_BASE + ADDR_GPIO_OFFSET)      // 0x7E200000
#define GPFSEL0_OFFSET     0x0       //pin 9 -> pin 0   [29:0]/3
#define GPFSEL1_OFFSET     0x4       //pin 19-> pin 10  [29:0]/3
#define GPFSEL2_OFFSET     0x8       //pin 29-> pin 20  [29:0]/3
#define GPFSEL3_OFFSET     0xC       //pin 39-> pin 30  [29:0]/3 
#define GPSET0_OFFSET      0x1C          // set GPIO pin [0..31]
#define GPSET1_OFFSET      0x20
#define GPCLR0_OFFSET      0x28
#define GPCLR1_OFFSET      0x2C
#define GPLEV0_OFFSET      0x34
#define GPLEV1_OFFSET      0x28

typedef enum{
     GPIO_FSEL_INPUT  = 0x0,    //3'b000
     GPIO_FSEL_OUTPUT = 0x1,    //3'b001
     GPIO_FSEL_ALT0   = 0x4,    //3'b100
     GPIO_FSEL_ALT1   = 0x5,    //3'b101
     GPIO_FSEL_ALT2   = 0x6,    //3'b110
     GPIO_FSEL_ALT3   = 0x7,
     GPIO_FSEL_ALT4   = 0x3,
     GPIO_FSEL_ALT5   = 0x2,
     GPIO_FSEL_MASK   = 0x7 
}gpioFuncSel;

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


#define ADDR_PWM_BASE              BCM2836_PERI_BASE + ADDR_PWM_OFFSET // will be over writen after VM
#define PWM_CTRL_OFFSET             0x0
#define PWM_STA_OFFSET              0x4
#define PWM_DMAC_OFFSET             0x8
#define PWM_RNG1_OFFSET             0x10
#define PWM_DAT1_OFFSET             0x14
#define PWM_FIF1_OFFSET             0x18 
#define PWM_RNG2_OFFSET             0x20 
#define PWM_DAT2_OFFSET             0x24 


////// CM base: 0x7E101000
#define CLK_PASSWRD          (0x5A << 24)
#define CLK_GP0_CTL_OFFSET   28
#define CLK_GP0_DIV_OFFSET   29
#define CLK_GP1_CTL_OFFSET   30
#define CLK_GP1_DIV_OFFSET   31
#define CLK_GP2_CTL_OFFSET   32
#define CLK_GP2_DIV_OFFSET   33

#define CLK_PCM_CTL_OFFSET   38     // 0x98/4
#define CLK_PCM_DIV_OFFSET   39

#define CLK_PWM_CNTL_OFFSET  40     // 0xa0/4
#define CLK_PWM_DIV_OFFSET   41
#define CLK_UART0_CNTL_OFFSET 60    // 0xf0
#define CLK_UART0_DIV_OFFSET  61   // 0xf4 


#define __PWM_SERIALISE_MODE__
#define CPRMAN                                                             // P$
#define PWM_FREQ              50                                           //50$
#define PWM_PERIOD            1000/PWM_FREQ                                //20$
#define PWM_DRIVE_CLK         100                                          //MH$
#define PWM_DRIVE_CLK_PERIOD  1000/(PWM_DRIVE_CLK*1000000)                 //ca$
#define PWM_RNG_VALUE         PWM_PERIOD/PWM_DRIVE_CLK_PERIOD              // M$
#define PWM_POS_L             PWM_RNG_VALUE/(20*1) 
#define PWM_POS_R             PWM_RNG_VALUE/(20*2)

typedef enum{
    PWM_CLK_DIV_9K375 = 2048,
    PWM_CLK_DIV_18K75 = 1024,
    PWM_CLK_DIV_37K5  = 512,
    PWM_CLK_DIV_75K   = 256,
    PWM_CLK_DIV_150K  = 128,
    PWM_CLK_DIV_300K  = 64,
    PWM_CLK_DIV_600K  = 32,
    PWM_CLK_DIV_1M2   = 16,                       // OSC=19.2M ???
    PWM_CLK_DIV_1M152 = 16,
    PWM_CLK_DIV_1M152_FRAC = 666        // max 4095
}PWMClkDiv;

typedef enum{
    MSEN2 = 15,
    USEF2 = 13,                 // bit position
    POLA2 = 12,
    SBIT2 = 11,
    RPTL2 = 10,
    MODE2 = 9,
    PWEN2 = 8,

    MSEN1 = 7,
    CLRF1 = 6,
    USEF1 = 5,
    POLA1 = 4,
    SBIT1 = 3,
    RPTL1 = 2,
    MODE1 = 1,
    PWEN1 = 0
}PWM_CTL_Reg;

#define ADDR_ARM_TIMER_BASE                 (0x2000B000)
#define ADDR_ARM_INT_BASE      ADDR_ARM_TIMER_BASE
#define ADDR_DMAC_CH0_BASE                  (0x20007000)
#define ADDR_DMAC_CH1_BASE    (ADDR_DMAC_CH0_BASE+0x100)
#define ADDR_DMAC_CH2_BASE    (ADDR_DMAC_CH1_BASE+0x100)
#define ADDR_DMAC_CH3_BASE    (ADDR_DMAC_CH2_BASE+0x100)
#define ADDR_DMAC_CH4_BASE    (ADDR_DMAC_CH3_BASE+0x100)
#define ADDR_DMAC_CH5_BASE    (ADDR_DMAC_CH4_BASE+0x100)

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


#define ADDR_PCM_BASE                  (0x20203000)
#define ADDR_SPI0_BASE                 (0x20204000)
#define ADDR_BSC0_BASE                 (0x20205000)
//#define ADDR_PWM_BASE                 (020E20C000)
#define ADDR_BSC_SPI_BASE              (0x20214000)
#define ADDR_AUX_BASE                  (0x20215000)
#define ADDR_EMMC_BASE                 (0x20300000)
#define ADDR_BSC1_BASE                 (0x20804000)
#define ADDR_USB_BASE                  (0x20980000)

#define SETUP_OK    0
#define SETUP_DEVMEM_FAIL  1
#define SETUP_MALLOC_FAIL  2
#define SETUP_MMAP_FAIL    3

int      peri_Init(uint32_t baudrate_spi, uint32_t baudrate_uart1, uint32_t baudrate_pwmser);

uint32_t PWM_Init(int ch_num);        // contains PWM_Enable & PWM_Config
int      PWM_Enable(int ch_num);
void     PWM_Config(int ch_num);
int      PWM_Set(int ch_num, int position);
void     PWM_Ser_WriteFIFO(uint32_t WrData);
uint32_t PWM_Ser_ReadFIFO( );
uint32_t PWM_STA_Check();
void     PWM_Set_Clock(uint32_t div);


int writeAddr32 (volatile uint32_t *addr, int32_t wr_data);
int readAddr32(volatile uint32_t *addr);
void delay_ns(unsigned int millis);
void bitSet32(volatile uint32_t *Addr, uint8_t position, uint8_t val);

int UART0_Init(uint32_t baudRate);
void UART0_Set_Clock(uint32_t div);
int UART0_Enable();
int UART0_Config(uint32_t baudRate );
int UART0_Send(uint8_t uartTxData);
unsigned char UART0_rxDataRdOneByte( );
int UART0_STA_Check( );

void gpio_write(uint16_t pin_num, uint16_t value);
void GPIO_Init( );
void gpio_funcSel(uint8_t pin_num, uint8_t mode);
void GPIO_Set_Clock (uint32_t div0, uint32_t div1, uint32_t div2); 
