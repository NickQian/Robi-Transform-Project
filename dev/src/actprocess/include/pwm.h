/*  pwm init and gen
*   ----
*   Licensed under BSD license.
*   ----
*   0.1 - 2017.5.2  init version for Raspberry 2B(bcm2836) -- Nick Qian
*   ----
*   IO & PWM init are done in the general "peri.cpp"
*/

#ifndef _PWM_H_
#define _PWM_H_


#include "peri.h"

//#define PWM_EMU_DMA_CHANNELS     2


// Subcycle min
#define SUBCYCLE_TIME_MIN_US     3000

// Defaultsubcycle time
//#define SUBCYCLE_TIME_DEFAULT_US 20000

//#define PULSE_INC_GRANU_US       10


#define DELAY_VIA_PCM              1

#define GPIO_PIN_EMU_PWM_INA_2     RPi2B_GPIO_J8_29
#define GPIO_PIN_EMU_PWM_INB_2     RPi2B_GPIO_J8_31

#define PWM_HW_WIDTH_MAX           PWM_HW_RANGE
#define PWM_EMU_WIDTH_MAX         (SUBCYCLE_TIME_DEFAULT_US/PULSE_INC_GRANU_US -1)


typedef enum{
    pwm_INA2 = 0,    // emu
    pwm_INB2 = 1,    // emu
    pwm_INA1 = 2,
    pwm_INB1 = 3
}WfwPWM;





int  bc_pwm_set (WfwPWM pwm_ch, uint32_t pwm_width, uint32_t width_start=0);

int  pwm_hw_set (WfwPWM pwm_ch, uint32_t pwm_width, uint32_t width_start=0);
int  pwm_emu_set(WfwPWM pwm_ch, uint32_t pwm_width, uint32_t width_start=0);
int  pwm_set_init(void);
void pwm_shutdown(void);

void     PWM_Ser_WriteFIFO(uint32_t WrData);
uint32_t PWM_Ser_ReadFIFO(void);

#endif
