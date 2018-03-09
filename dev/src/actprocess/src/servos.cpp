/*
    ROS action server to handle request from clients "mc.py"
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.2 - 2016.12.4  add servos concurrent move 
    0.1 - 2016.7.25  init version
    ----
*/

#include <stdint.h>         //cstdint need c++11
#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actprocess/servosAction.h>

#include "futabaServo.h"
#include "peri.h"
#include "uart0.h"
#include "servos.h"

using namespace std;


class servoAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance
  actionlib::SimpleActionServer<actprocess::servosAction> as_;
  std::string action_name_;
  // create messages used to pub feedback/result
  actprocess::servosFeedback  feedback_;
  actprocess::servosResult    result_;

public:

  servoAction(std::string name):
    as_(nh_, name, boost::bind(&servoAction::executeCB, this, _1), false),
    action_name_(name)
   {
     servoConfig();
     as_.start();
   }

   ~servoAction(void)
   {
    futaba_shutdown();
    printf("In <~servoAction> and mem freed. \n");

    }

   void executeCB(const actprocess::servosGoalConstPtr &goal){
       //var
       volatile uint8_t  servoID      =  uint8_t(goal->servoID);
       volatile  int16_t goalPosition =  int16_t(goal->goalAngle);
       volatile uint16_t goalTime     = uint16_t(goal->goalTime);
       int wait_count = 0;
       int result = 0;

       ros::Rate r(10);
       bool success = true;

       // init data
       feedback_.current_position = 0;           // int16
       //feedback_.current_position.clear();
       //feedback_.current_position.push_back(0);

       ///////////// start exe action for Goal //////////////

       // servo set
       result = servoSet(servoID, goalPosition, goalTime);
       ROS_INFO("%s: servoSet Excuted, goalID:%i, goalAngle: %i, goalTime: %i", action_name_.c_str(),
                                      int(servoID), goalPosition, goalTime);

       //getServoReturnDelay(servoID);
       //setServoReturnDelay(servoID, 40);      // 20*50us=1ms. 40 is 2ms. (max=255)

       ////////////// inquire feedback ////////////
       while ( ros::ok() ){
                  cout << "====> delay 0.2s, inquire postion again, ID=" << int(servoID) << endl;
                  delay_ms(INTVL_INQ_POSITION);            // 500 = 500 ms

                  // publish the feedback
                  feedback_.current_position =  getPresentPosition(servoID);
                  cout << "@--> (3.1) pub feedback...feedback is " << dec << feedback_.current_position  << endl;

                  as_.publishFeedback(feedback_);

                  if ( ALLOWABLE_DIFF > abs(goalPosition - feedback_.current_position) ){
                       cout << "!!!Got it!!!. Will break."  << endl;
                       servoTorqueBrake(servoID);
                       success = true;
                       break;
                  }
                  else if(MAX_INQ_TIMES == wait_count){
                       cout << "Error: Max Inquiry times exceed. Servo is blocked?" << endl;
                       servoTorqueBrake(servoID);
                       success = false;
                       break;
                  }

                  wait_count++;
       }

       cout << "@--> (3.2). Goal achived. Entering sleep..." << endl;
       r.sleep();

      ///////////// write to goal.result /////////////
      if (success) {
        result_.result = 3;    //int32 status    SUCCEEDED;
        ROS_INFO("%s: Succeeded. wakeup. Goal achieved.", action_name_.c_str() );
        // set the action_state to succeeded
        as_.setSucceeded(result_);
      }

   }  // executeCB

};  // class




int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_servos");

    ROS_INFO( "@--> (1)  " );
    //cout << "[argc] is:" << argc << endl;

    peri_init( );                // do the mmap

    uart0_setup(115200);         // set uart0 speed

    //servoConfig();             // set servo parameters

    //servosAction servos(ros::this_node::getName() );
    cout << "@-->2. getName:" << ros::this_node::getName() << endl;
    servoAction servo_head_H("svr_servo_head_H" );
    servoAction servo_shoulder_L("svr_servo_shoulder_L" );
    servoAction servo_bigArm_L("svr_servo_bigArm_L" );
    servoAction servo_arm_L("svr_servo_arm_L" );
    servoAction servo_shoulder_R("svr_servo_shoulder_R" );
    servoAction servo_bigArm_R("svr_servo_bigArm_R" );
    servoAction servo_arm_R("svr_servo_arm_R" );
    servoAction servo_knee_L("svr_servo_knee_L" );
    servoAction servo_ankle_L("svr_servo_ankle_L" );
    servoAction servo_knee_R("svr_servo_knee_R" );
    servoAction servo_ankle_R("svr_servo_ankle_R" );

    cout << "@--> (3). enter in spin()" << endl;

    ros::spin();

    return 0;

}



