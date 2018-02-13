/*  top file of balance control(integrate/launch wfw and body control to achieve balance)
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.4.26  init version for Raspberry 2B(bcm2836) -- Nick Qian
    ----
    output: PWM (<pwm_gen> in pwm.cpp)
    input: (1) info published from sensors topic
           (2) action goal
*/

#include <stdint.h>
#include <iostream>

#include <ros/ros.h>
#include <sensorprocess/optEncoder.h>
#include <sensorprocess/attt.h>
//#include <sensorprocess >

#include "balancer.h"
#include "pwm.h"

void PWM_Gen(cmdTurn, cmdTravel){

    if (cmd == TURN_LEFT){
        turnNeed = -40;
    }
    else if(cmd == TURN_RIGHT){
        turnNeed = 40;
    }
    else{
        turnNeed = 0;
    }

    



}
