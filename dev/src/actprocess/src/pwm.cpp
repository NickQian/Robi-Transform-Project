
/*  bc(balance car) pwm init and gen
    ----
    Licensed under BSD license.
    ----
    0.1.1  use linux DMA funcs "dma_cap_zero"/"dma_request_channel"/"dma_map_sg"/"dmaengine_submit" ...to operate DMA??
    0.1 - 2017.5.2  init version for Raspberry 2B(bcm2836) -- Nick Qian
    ----
    output: 4 cahnnels PWM(2 hardwared, 2 DMA emulated)
    input: CMD
*/

#include <cstdio>
#include <iostream>

#include "pwm.h"

using namespace std;


static volatile uint32_t *bcm283x_pwm_vm = 0;

//static struct pwm_emu_ch pwm_emu_chs[PWM_DMA_EMU_CHANNELS];
static uint16_t pulse_width_incr_us = -1;



/*****************************************************************
/* speedRaw & PIDsum: 1% - 100%
/* <PIDsum> is unsinged
/******************************************************************/

int32_t pwmCalculate(double PIDsum, uint32_t max_width)
{
    double speedRaw;             // 1-100
    int32_t pwmValue;

    if (PIDsum > 100.0f){
        speedRaw = 100.0f;
    }
    else{
        speedRaw = PIDsum;
    }

    // scale from 0-100 to (0- pwm value)
    pwmValue = speedRaw * ((float)max_width) / 100.f;

    return pwmValue;

}



/********* Set one of the Hw or Emu PWM channels on PCB ******************
/*   ch_num = 1(hw ch 1),
              2(hw ch 2),
              3(emu_ch 0),
              4(emu_ch 1)
/************************************************************************/
int bc_pwm_set(WfwPWM pwm_ch, uint32_t pwm_width, uint32_t width_start){

    if(pwm_ch == pwm_INA1 ||  pwm_ch == pwm_INB1 ){
         pwm_hw_set(pwm_ch, pwm_width);
    }
    else if ( pwm_ch == pwm_INA2 ||  pwm_ch == pwm_INB2 ){     // 15 DMA channels + 2 hw PWM
         pwm_emu_set(pwm_ch, pwm_width);
    }
    else{
        printf("Error: Invalid PWM Channel Number. ch_num is:%d" , pwm_ch);
        return -1;
    }

    return 0;

}



/*******************************************************************************
/*   set hardware pwm dutyCycle
/*******************************************************************************/

int pwm_hw_set(WfwPWM pwm_ch, uint32_t pwm_width, uint32_t width_start)
{
    volatile uint32_t *pwm_dat1 = bcm283x_pwm_vm + PWM_DAT1_OFFSET/4;
    volatile uint32_t *pwm_dat2 = bcm283x_pwm_vm + PWM_DAT2_OFFSET/4;
    volatile uint32_t *pwm_rng1 = bcm283x_pwm_vm + PWM_RNG1_OFFSET/4;
    volatile uint32_t *pwm_rng2 = bcm283x_pwm_vm + PWM_RNG2_OFFSET/4;

    if(pwm_ch==pwm_INA1){
        writeAddr32(pwm_dat1, int32_t(pwm_width) );
        cout << "PWM_DAT1 set done. width: "<< pwm_width << ".Read:" <<  dec << *pwm_dat1 << ". PWM_RNG1: "<< *pwm_rng1 << ".PWM_DAT2:" << *pwm_dat2 <<endl;
        return 0;
    }
    else if(pwm_ch==pwm_INB1){
        writeAddr32(pwm_dat2, int32_t(pwm_width) );
        cout << "PWM_DAT2 set done. width: "<< pwm_width << ".Read:" <<  dec << *pwm_dat2 << ". PWM_RNG2: "<< *pwm_rng2 <<".PWM_DAT1:" << *pwm_dat1 <<endl;
        return 0;
    }
    else{
        return -1;
    }

}


void PWM_Ser_WriteFIFO(uint32_t WrData){

    volatile  uint32_t *pwm_fif1 = bcm283x_pwm_vm + PWM_FIF1_OFFSET/4;

    *pwm_fif1 = WrData;

}



uint32_t PWM_Ser_ReadFIFO(void){

    volatile uint32_t *pwm_fif1 = bcm283x_pwm_vm + PWM_FIF1_OFFSET/4;
    uint32_t readResult;

    return readResult = *pwm_fif1;

}



//--------------------------------------------- pwm emu ------------------------------
int pwm_emu_set(WfwPWM pwm_ch, uint32_t pwm_width, uint32_t width_start){

    int gpio = (pwm_ch==pwm_INA2) ? GPIO_PIN_EMU_PWM_INA_2 : GPIO_PIN_EMU_PWM_INB_2;

//    PWM_Emu_AddChPulse( (uint8_t)pwm_ch, gpio, width_start, pwm_width);   // Note: (int ch_num, int gpio, int width_start, int width);
    if(pwm_ch==pwm_INA2){
        PWM_Emu_AddChPulse(0, gpio, width_start, pwm_width);
    }
    else if(pwm_ch==pwm_INB2){
        PWM_Emu_AddChPulse(1, gpio, width_start, pwm_width);
    }
}



void pwm_shutdown(void){

    cout << "Warning: <pwm_shut_down> TODO"  << endl;

}



/*******************************************
 * this is ptr init for pwm set operations
 ******************************************/
int pwm_set_init(void)
{
    if(bcm283x_pwm_vm = get_pwm_vm()){
        printf("<pwm_set_init>: bcm283x_pwm_vm value is: %p. \n", bcm283x_pwm_vm);
        return 0;
    }
    else{
        printf("Error: bcm283x_pwm_vm value is: %p.", bcm283x_pwm_vm);
        return -1;
    }

}


