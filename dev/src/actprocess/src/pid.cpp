/*  pid module
 *  ----
 *  Licensed under BSD license.
    ----
    0.1 - 2017.3.1  init version for Raspberry 2B(bcm2836) --- Nick Qian
    ----
    output: PID Value
    input:  IMU data/Commmand/speed sensor
*/

#include "pid.h"

#include <iostream>
//#include <type_traits>

using namespace std;


//extern cfg_wfw_t cfg_wfw;

Pid::Pid(const double cfg_Kp, const double cfg_Ki, const double cfg_Kd, string PIDName):
    lastError(0.0), integratedError(0.0), currentSpeed(0.0)
{
    //static_assert( (cfg_Kp >= 0.0), "cfg_Kp is lower than 0");
    //static_assert( (cfg_Ki >= 0.0), "cfg_Ki is lower than 0");
    //static_assert( (cfg_Kd >= 0.0), "cfg_Kd is lower than 0");

    this->Kp = cfg_Kp;
    this->Ki = cfg_Ki;
    this->Kd = cfg_Kd;

    cout << "PID controller <" << PIDName << "> is established." << endl;

}



Pid::~Pid(void)
{

}



double Pid::updatePID(double target, double current, double dt)
{
    double error = target - current;
    double pTerm = Kp * error;
    integratedError += error * dt;

    double iTerm = (Ki * 100.0) * integratedError;
    double dTerm = (Kd/100.0) * (error - lastError)/dt;

    lastError = error;
    PIDValue = pTerm + iTerm + dTerm;

    currentSpeed = (currentSpeed + PIDValue * 0.004) * 0.999;   // ???

    PIDValue += currentSpeed;

    // ... TODO: Estimate velocity and steer




    return this->PIDValue;
}


void Pid::setPID_param(double cfg_Kp, double cfg_Ki, double cfg_Kd)
{
    Kp = cfg_Kp;
    Ki = cfg_Ki;
    Kd = cfg_Kd;
}


void Pid::resetPID(void)
{

    PIDValue        = 0;
    lastError       = 0;
    integratedError = 0;
    currentSpeed    = 0;

}
