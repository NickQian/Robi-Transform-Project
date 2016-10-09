#!/usr/bin/env python
"""
 motion control (motor control board with BLE) -2015.11.23
 ----
 Licensed under BSD license. xxxxx
 by Nick Qian  -2015.11.13
 ----
 Inputs:  (API) from actPattern
 Outputs: actlib client request to motors driver(cpp)
"""

#import robiClib as robiC
import os, sys
import roslib
roslib.load_manifest("actprocess")
import rospy

import actionlib
from actprocess.msg import servosAction, servosGoal, servosResult
from actprocess.msg import actPatternAction, actPatternGoal

#will add to servosGoal action file next compile
head_H        = 1            #servosGoal.head_H
shoulder_L  = 2
arm_L           = 3
forearm_L    = 4
#elbow_L
shoulder_R = 5
arm_R          = 6
forearm_R   = 7
#elbow_R
knee_L        = 8
ankle_L       = 9
knee_R       = 10
ankle_R      = 11

"""
uint8  PENDING    = 0  # the goal has not yet to be processed by server
uint8  ACTIVE     = 1  # the goal us processing
uint8  PREEMPTED  = 2  # the goal received a cancel request <after> start executing

uint8  SUCCEEDED  = 3  # goal achieved
uint8  ABORTED    = 4  # by server dur to failure
uint8  REJECTED   = 5  # by server because goal was unattainable or invalid
uint8  PREEMPTING = 6  # the goal received a cancel request after start executing(adn has not yet complete exe)
uint8  RECALLING  = 7  # the goal received a cancel request <before> start executing ...
uint8  RECALLED   = 8
uint8  LOST       = 9  # cliet determine that a goal is LOST
"""


class mc() :                 # actionlib client to servo(C++ action server)
    def __init__(self):
        # build an act cliet 
        self.act_client = actionlib.SimpleActionClient('node_servos', servosAction)
        self.servo_max_wait_sec = rospy.get_param('~servo_max_wait_sec', 5)
        #self.rate = rospy.Rate(10)        
        #self.act_client.wait_for_server( )
        print ("@@@: mc initialized. waiting for server....")

    '''---------- Head ------------'''
    def head_move(self, dest = -30, speed = 1 ):     # speed: 1-9
        block = False
        
        #   dest: 0-320-640  = -1500 -> 0 -> +1500     |   10*12=120*, 200/10=20ms ->200ms 
        goal = servosGoal(servoID = head_H, goalAngle = (dest-320)*10/4,  goalTime = 200/speed)
        res = None
        try_count = 0
        
        while not rospy.is_shutdown():
            if not block:
                self.act_client.send_goal(goal)
                block = True
            elif self.servo_max_wait_sec == try_count:
                rospy.logwarn("Wait too long time for the servo action result success! will break.")
                break
            else:
                self.act_client.wait_for_result(rospy.Duration.from_sec(1.0) )
                res = self.act_client.get_result( )
                if res is not None:
                    print ("@@@@@current act_client.get_result, types are:, result.result is, type is,", res, type(res), res.result, type(res.result) )
                
                    if (servosResult.SUCCEEDED == res.result):
                        block = False
                        print ("~~~~~~~~~~~goal achieved. break. goal.goalAngle & res.result is:", goal.goalAngle, res.result)
                        break
                    else:                        
                        try_count +=1 
                        print ("~~~~~~~~~~~~waiting for goal achieved. second is", try_count)
                
        return self.act_client.get_result( )
        
    '''-----------Hands--------------'''
    def armL_up(self, amp=9, speed=9 ):
        print ('left hand uping...speed =s%', speed)
        goal = actprocess.msg.servosGoal(servoID = shoulder_L, goalAngle = amp*12,  goalTime = 200/speed)  
        self.act_client.send_goal(goal)
        self.act_client.wait_for_result(rospy.Duration.from_sec(5.0) )
        return self.act_client.get_result( )

    def armL_down(self,  dest=1, speed=9 ):
        rospy.loginfo('left hand downing...speed =s%', speed)
        goal = actprocess.msg.servosGoal(servoID = shoulder_L, goalAngle = amp*12,  goalTime = 200/speed)
       
        self.act_client.send_goal(goal)
        self.act_client.wait_for_result(rospy.Duration.from_sec(5.0) )
        return self.act_client.get_result( )

    def armR_up(self, speed, amp):
        print ('right hand uping...speed  = d%', speed)
        raise NotImplementedError


    def armR_down(self, speed, amp):
        print ('right hand downing...speed =s%', speed)
        raise NotImplementedError

    def armL_left(self, speed, amp):
        print ('left hand uping...speed =s%', speed)
        raise NotImplementedError

    def armlRight(self, speed, amp):
        print ('left hand downing...speed =s%', speed)
        raise NotImplementedError

    def armrLeft(self, speed, amp):
        print ('right hand uping...speed =s%', speed)
        raise NotImplementedError

    def armrRight(self, speed, amp):
        print ('right hand turning right...speed =s%', speed)
        raise NotImplementedError

    "------------------- Legs --------------------"
    def leg_moveForward(self, speed, distance):
        print ('moving forward...')
        goal = actprocess.msg.mcGoal()

    def leg_moveBack(self, speed, distance):
        print ('moving back...')
        raise NotImplementedError

    def leg_turnLeft (self, speed, distance):
        print ('turning left...')
        raise NotImplementedError

    def leg_turnRight(self, speed, amp):
        print ('turning right.....')
        raise NotImplementedError

"""
'''--------------Servo---------------'''
    def servoMove (ID, pos = 45, tim=1000):
        position = pos * 10
        time = tim / 10                          #tim is "ms"

        robiC.servoSet(ID, position, time )      # 0 time means no time req
"""

if __name__ ==  '__main__':
    try:
        mc = mc( )
        print ("@--> (1)-> mc.py")
        
    except rospy.ROSInterruptException:
        print ("@@ warining: program interrupted before completion")
        
    mc.head_move(1)
    mc.head_move(9)
    mc.head_move(5)
    mc.armL_up( )
    mc.armL_down( )

    """
    os.system('omxplayer -o local boring.wav')
    tts("check the fucking tts",\
        "voice_name = xioayan, text_encoding = UTF8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2");

    robiC.peri_Init(19200, 115000, 19200)
    #robiC.servoSetID(4)

    os.system('omxplayer -o local boring.wav')

    servoMove(1, 45, 300)
    servoMove(2, 40, 1000)
    servoMove(4, 90, 600)

    servoMove(1, -45, 2000)
    servoMove(2, 45,  300)
    servoMove(4, 60,  1000)

    servoMove(1, 0,   500)
    servoMove(2, 0,   2000)
    servoMove(4, 0,   2000)

    os.system('omxplayer -o local boring.wav')

    sys.exit()   
    """
 
    print "|||||||ENDED||||||. Check the fucking status."


