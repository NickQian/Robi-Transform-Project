
/*  pid module
*   ----
*   Licensed under BSD license.
*   ----
*   0.1 - 2017.7.6  init version(ref:github.com/Lauszus/BalancingRobotFullSize ) -- Nick Qian
*/

#ifndef _PID_H_
#define _PID_H_

#include <string>

using std::string;

class Pid{

 public:

    Pid(const double cfg_Kp, const double cfg_Ki, const double cfg_Kd, string PIDName);
    ~Pid();

    double PIDValue;

    void   setPID_param(double cfg_Kp, double cfg_Ki, double cfg_Kd);
    double updatePID(double target, double current, double dt);
    void   resetPID(void);


 private:
    double lastError;            // last angle error
    double integratedError;      // integrated error
    double currentSpeed;         // estimated speed

    double Kp, Ki, Kd;           // PID parameters

};


#endif
