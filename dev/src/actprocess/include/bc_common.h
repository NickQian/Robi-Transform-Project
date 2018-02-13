/*  common include file for bc
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.11.15  init version for Raspberry 2B(bcm2836) -- Nick Qian
    ----
*/

#ifndef __BC_COMMON_
#define __BC_COMMON_

struct cfg_wfw_t {
    uint16_t WfwSysTickFreq;                   // = 500;  //need c++11 support
    uint16_t WfwRate_Imu_Pid;                  // = 5;
    double   WfwBalanceBreakPoint;             // = 4.5;

    double Kp;
    double Ki;
    double Kd;

    double  targetAngle;                       // resting angle of robot
    double  err_tolerance; 
    uint8_t controlAngleLimit;                 // max titlting angle of robot
    uint8_t turningScale;                      //

    double Qangle;
    double Qbias;
    double Rmeasure;            // Kalman filter values
    double accYzero;
    double accZzero;                 // Accelerometer Zero values
    double leftMotorScaler;
    double rightMotorScaler;
};

extern const cfg_wfw_t cfg_wfw;                           // defined in "wfw.cpp"



#endif

