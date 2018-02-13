/*  c driver for optical encoder(GPIO interrupt)
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2017.4.27  init version for Raspberry 2B(bcm2836) by Nick Qian
    ----
    output: speed of the motor
    input: GPIO pin()
*/

#ifndef _OPT_ENCODER_
#define _OPT_ENCODER_


#define USE_MINI_SPEED_SENSOR
//#define USE_QUADRATURE_ENCODER


#ifdef USE_MINI_SPEED_SENSOR
   #pragma message("@Compile MSG@: Using MINI_SPEED_SENSOR at BC")


#define PIN_OPT_ENCODER_1A  RPi2B_GPIO_J8_11
#define PIN_OPT_ENCODER_1B  RPi2B_GPIO_J8_12
#define PIN_OPT_ENCODER_2A  RPi2B_GPIO_J8_13
#define PIN_OPT_ENCODER_2B  RPi2B_GPIO_J8_15

#define PULESES_PER_TURN   20


#define DEV_NAME_PHOENC_1A  "DEV_NAME_PHOENC_1A"
#define DEV_NAME_PHOENC_2A  "DEV_NAME_PHOENC_2A"

#define DEV_ID_PHOENC_1A  "DEV_ID_PHOENC_1A"
#define DEV_ID_PHOENC_2A  "DEV_ID_PHOENC_2A"


#endif   // ifdef USE_MINI_SPEED_SENSOR


struct OptEnc{
    uint8_t gpio_pin;
    int irq;
    const char *dev_name;
};

#endif
