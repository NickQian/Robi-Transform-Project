/*  this only control the two "Wind Fire Wheels"
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.6.22  init version -- Nick Qian
    ----
    output: PWM (<pwm_gen> in pwm.cpp)
    input: (1) info published from sensors topic
*/

#include <stdint.h>
#include <stdio.h>
#include <iostream>

#include <std_msgs/String.h>

#include "wfw.h"
#include "pwm.h"
#include "peri.h"
//#include <sensorprocess/attt.h>

using namespace std;

// Need gcc 5.x or later version
const cfg_wfw_t cfg_wfw = {
    .WfwSysTickFreq  = 500,          // uint16_t
    .WfwRate_Imu_Pid = 5,            // uint16_t.  (Imu update freq)/(Pid update freq)
    .WfwBalanceBreakPoint = 60.f,    //4.5,   double

    ///// PID /////
    .Kp = 9.0f,
    .Ki = 2.0f,
    .Kd = 3.0f,

    .targetAngle = 0.f,    // 180.f
    .err_tolerance = 2.0f,
    .controlAngleLimit = 7,
    .turningScale = 1,

    ///// Kalman filter /////
    .Qangle = 0.001f,
    .Qbias = 0.003f,
    .Rmeasure = 0.03f,

    .accYzero = 0.0f,
    .accZzero = 0.0f,
    .leftMotorScaler  = 0.01,  //1.0f,
    .rightMotorScaler = 0.01    //1.0f
};


/*********************************************************************
/* global var
/*
/********************************************************************/

uint32_t kalmanTimer;
static uint32_t pidTimer;

/*************************************************
/* u(t) = Kp*e(t) + Ki[e(t) +Kd[e(t)-e(t-1)] + u0
/*************************************************/


void wfwAction::moveMotors(double PIDValue, double turning)
{
    double PIDLeft  = PIDValue - turning;
    double PIDRight = PIDValue + turning;

    /****** compensate for difference in some motors********/
    PIDLeft  *= cfg_wfw.leftMotorScaler;
    PIDRight *= cfg_wfw.rightMotorScaler;

    printf("<moveMotors>: PIDValue: %f, after scaler: %f %f, turning: %f \n", PIDValue, PIDLeft, PIDRight, turning);
    //// PID left ////

    if (PIDLeft >= 0)
        moveMotor(motor_footL, forward,   PIDLeft);
    else
        moveMotor(motor_footL, backward, -PIDLeft);

    //// PID right ////
    if (PIDRight >= 0)
        moveMotor(motor_footR, forward,   PIDRight);
    else
        moveMotor(motor_footR, backward, -PIDRight);

}



/*****************************************************************
/* speedRaw & PIDsum: 1% - 100%
/* <PIDsum> is unsinged
/******************************************************************/

int32_t wfwAction::pwmCalculate(double PIDsum, uint32_t max_width)
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


/*******************************
/*  this is moving single motor
/*  INA  |   INB  |
/*   L   |    L   | idle
/*   H   |    L   | spin +
/*   L   |    H   | spin -
/*   H   |    H   | brake
/*******************************/
void wfwAction::moveMotor(WfwMotor motor, WfwCommand direct, double PIDres)
{
    int32_t pwm_dat;

    //// left ////
    if (motor == motor_footL){
        if (direct == forward){
            pwm_dat = pwmCalculate(PIDres, PWM_EMU_WIDTH_MAX);
            bc_pwm_set(pwm_INA1, (uint32_t)pwm_dat);      // PWM_CH_LEFT_FOOT_A

            GPIO_Write(LEFT_FOOT_B_PIN, 0);            //leftB = LOW;
        }
        else{
            pwm_dat = pwmCalculate(PIDres, PWM_EMU_WIDTH_MAX);
            bc_pwm_set(pwm_INB1, (uint32_t)pwm_dat);

            GPIO_Write(LEFT_FOOT_A_PIN, 0);           // leftA = LOW
        }
    }


    //// right ////
    if (motor == motor_footR){
        if (direct == forward){
            pwm_dat = pwmCalculate(PIDres,PWM_EMU_WIDTH_MAX);
            bc_pwm_set(pwm_INA2, (uint32_t)pwm_dat);

            GPIO_Write(RIGHT_FOOT_B_PIN, 0);            // rightB = LOW;
        }
        else{
            pwm_dat = pwmCalculate(PIDres, PWM_EMU_WIDTH_MAX);
            bc_pwm_set(pwm_INB2, (uint32_t)pwm_dat);

            GPIO_Write(RIGHT_FOOT_A_PIN, 0);           // rightA = LOW
        }
    }

}


/***************************************************************
/* INA  INB
/* H  -  H    brake
/* L  -  L    standby mode
/***************************************************************/
void wfwAction::stopMotor(WfwMotor motor)           // brake
{
    if(motor == motor_footL){
        cout << "@~@: will bc_pwm_set stop LEFT_FOOT. PWM_HW_WIDTH_MAX is:" << showbase << PWM_HW_WIDTH_MAX << endl;
        bc_pwm_set(pwm_INA1, PWM_HW_WIDTH_MAX * 0.95);
        bc_pwm_set(pwm_INB1, PWM_HW_WIDTH_MAX * 0.95);
        cout << "<stopMotor> bc_pwm_set stop LEFT_FOOT Done. "  << endl;
    }

    if(motor == motor_footR){
        cout << "@~@: will bc_pwm_set RIGHT_FOOT. PWM_EMU_WIDTH_MAX is:" << PWM_EMU_WIDTH_MAX << endl;
        bc_pwm_set(pwm_INA2, PWM_EMU_WIDTH_MAX *0.95);
        bc_pwm_set(pwm_INB2, PWM_EMU_WIDTH_MAX *0.95);
        cout << "<stopMotor>: bc_pwm_set stop RIGHT_FOOT Done. "  << endl;
    }

}


void wfwAction::stbMotor(WfwMotor motor)
{
    if(motor == motor_footL){
        bc_pwm_set(pwm_INA1, 0);
        bc_pwm_set(pwm_INB1, 0);
        cout << "<stbMotor>: bc_pwm_set release LEFT_FOOT Done. "  << endl;
    }

    if(motor == motor_footR){
        bc_pwm_set(pwm_INA2, 0);
        bc_pwm_set(pwm_INB2, 0);
        cout << "<stbMotor>: bc_pwm_set release RIGHT_FOOT Done. "  << endl;
    }

}


/*******************************************************************************
/*
/*  Action interface
/*
/******************************************************************************/

wfwAction::wfwAction(std::string name):
    as_(nh_, name, false),
    action_name_(name),
    pid_(cfg_wfw.Kp, cfg_wfw.Ki, cfg_wfw.Kd, "PIDConstroller(wfw)")
{
    // do init job for PID loop/kalman filter/body control parameters
    pitch_ =0;
    turningValue_ =0;
    layingDown_ = true;                 // = true;
    steerStop_ = true;                  // = true;
    stopped_ = true;

    //register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&wfwAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&wfwAction::preemptCB, this));

    // subscribe to data topic
    sub_ = nh_.subscribe("node_sens_att/tpc_sens_att", 1, &wfwAction::analysisCB, this);       //("/tpc_sens_attt",

    as_.start();
    cout << "@~@: as_.started.... " << endl;
}



void wfwAction::goalCB()
{

    // do something


    /////////  accept the new goal /////////
    // get goal ptr
    actprocess::wfwGoalConstPtr    goalPtr;            //  boost::shared_ptr< ::actprocess::wfwGoal const> wfwGoalConstPtr;
    goalPtr = as_.acceptNewGoal();
    goal_.wfwCmd       = goalPtr->wfwCmd;
    goal_.goal_pose    = goalPtr->goal_pose;
    goal_.keep_balance = goalPtr->keep_balance;

    cout << "@~@: <as_.goalCB> keep_balance: " << goal_.keep_balance << endl;
}



void wfwAction::preemptCB(){

    ROS_INFO("!!!: Preempted", action_name_.c_str() );

    as_.setPreempted();
}




/**************************************************************
/* Imu_callback() from ros::Subsctiber. Process income data
/**************************************************************/
void wfwAction::analysisCB(const sensor_msgs::ImuConstPtr& msg)              // receive sensor data and keep balance
{
    ////// Get IMU info //////
    //printf ("@~@: Get Imu msg. Enter in <analysisCB> \n");

    double dt = Imu_dataAnalyze(msg);
    countImu_++;

    ////// Get Speed info //////
    // ...

    if (!as_.isActive() ){
        printf ("! Warning: in <analysisCB>: as_ isNotActive. \n");
        return;
    }
    else if( abs(pitch_ - cfg_wfw.targetAngle) > cfg_wfw.WfwBalanceBreakPoint){
        printf ("! Warning: in <analysisCB>: beyond balance break point. will stop motor! pitch_: %f. \n", pitch_ );
        as_.setAborted(result_);
        stopMotor(motor_footL);
        stopMotor(motor_footR);
        return;
    }

    ////// update PID & move motor //////
    if( goal_.keep_balance  && countImu_ >= cfg_wfw.WfwRate_Imu_Pid){
        countImu_ = 0;
        pid_.updatePID(cfg_wfw.targetAngle, pitch_,   dt);      // (double target,   double current, double dt)
        moveMotors(pid_.PIDValue, turningValue_);               // (double PIDValue, double turning)
    }


    ////////// feedback /////////
    feedback_.current_pitch = pitch_;
    feedback_.current_pose.position = position_;             // TODO
    //feedback_.current_pose.orientation = quaternion_;      // float64 x, y, z, w
    as_.publishFeedback(feedback_);

    //////// result ///////
    if ( abs(pitch_) < cfg_wfw.err_tolerance){
        as_.setSucceeded(result_);
        printf("!Succeed once. as_ will not be active.");
    }
    else{
    }

}


double wfwAction::Imu_dataAnalyze(const sensor_msgs::ImuConstPtr& msg)
{
    double dt = msg->header.stamp.toSec() - lastImuTimStamp_;
    lastImuTimStamp_ = msg->header.stamp.toSec();

    pitch_ = msg->orientation.y;         //kalAngleY

    //printf (" @~@ pitch_: %f, lastImuTimStamp_: %f, \n", pitch_, lastImuTimStamp_ );

    return dt;
}



//------------------------------------------------------------------------------------------------
void wfwAction::executeCB(const actprocess::wfwGoalConstPtr &goal)
{

    ros::Rate r(cfg_wfw.WfwSysTickFreq);
    bool success = true;

    r.sleep();
    if(success){
        result_.result = 3;
    }

}




wfwAction::~wfwAction(void)
{

}




//-------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_wfw");

    printf("#########################################1############################################## \n");
    peri_init();
    pwm_set_init();           // should before wfw_act constructor, or will "segmentation fault"

    printf("############################2############################# \n");
    wfwAction wfw_act(ros::this_node::getName() );

    ros::spin();

    printf("############################3 spin() out############################# \n");

    return 0;
}

