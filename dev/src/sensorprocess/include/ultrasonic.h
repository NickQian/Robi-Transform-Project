/*  c driver for ultrasonic module( ) of RTP project
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2017.3.15  init version for Raspberry 2B(bcm2836)
    ----
    output: distance measured & inquiried by sens.py
    input: ultrasonic module(HC_SR04)
*/

#ifndef _ULTRASONIC_H_
  #define _ULTRASONIC_H_

#define USE_HC_SR04


#ifdef USE_HC_SR04
   #pragma message("@Compile MSG@: Using HC_SR04 as ultrasonic sensor")

#define PIN_ULTRASNC_TRIG    RPi2B_GPIO_J8_29
#define PIN_ULTRASNC_ECHO    RPi2B_GPIO_J8_31

#define USNC_RESOLUTION     2          //mm
#define USNC_MIN_DIST         2000     //2 cm
#define USNC_MAX_DIST       450000     //450cm

#endif    // if use HC_SR04


#endif

