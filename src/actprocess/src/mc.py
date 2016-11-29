#!/usr/bin/env python

"""
 motion control (motor control board with BLE) -2015.11.23
 ----
 Licensed under BSD license.

 0.1  - 2015.11.13 by Nick Qian
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
#from actprocess.msg import actPatternAction, actPatternGoal

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


class mc() :                     # actionlib client to servo(C++ action server)
    head_H     = 1            #servosGoal.head_H
    shoulder_L = 5
    bigArm_L   = 6
    arm_L      = 7
    #elbow_L
    shoulder_R = 2
    bigArm_R   = 3
    arm_R      = 4
    #elbow_R
    knee_L     = 8
    ankle_L    = 9
    knee_R     = 10
    ankle_R    = 11
    
    def __init__(self):
        # build an act cliet 
        self.act_client = actionlib.SimpleActionClient('node_servos', servosAction)
        self.servo_max_wait_sec = rospy.get_param('~servo_max_wait_sec', 8)
        #self.rate = rospy.Rate(10)
        print ("@@@: mc..act_client.<wait_for_server>.....")
        self.act_client.wait_for_server( )
        print ("@@@: mc initialized.")


    def servo_req(self, goal):
        
        block = False        
        res = None
        try_count = 0
        
        while not rospy.is_shutdown():
            if not block:
                print ("---: mc..head_move.<wait_for_server>.....")
                self.act_client.wait_for_server( )
                print ("---: mc head_move get server")
                self.act_client.send_goal(goal)
                print ("@@@: <head_move>. goal sent:", goal)
                block = True
            elif self.servo_max_wait_sec < try_count:
                rospy.logwarn("Wait too long time for the servo action result success! will break.")
                break
            else:
                print ("@@@: <head_move>. waiting for result...")
                self.act_client.wait_for_result(rospy.Duration.from_sec(1.0) )
                res = self.act_client.get_result( )
                print ("--> res is:", res)
                if res is not None:
                    print ("Info: act_client.get_result, res,res.result:", res, res.result )
                
                    if (servosResult.SUCCEEDED == res.result):
                        block = False
                        print ("~~~~~~~~~~~goal achieved. break. goal.goalAngle & res.result is:", goal.goalAngle, res.result)
                        break
                    elif (servosResult.PENDING == res.result):
                        self.act_client.wait_for_result(rospy.Duration.from_sec(1.0) )
                        try_count +=1
                        if self.servo_max_wait_sec < try_count:
                            rospy.logwarn("Pending too long time for the servo action result success! will break.")
                            break                                                
                else:                        
                    try_count +=1 
                    print ("~~~~waiting for goal achieved. try_count:", try_count)
                
        return self.act_client.get_result( )
    
        

    '''---------- Head ------------'''
    def head_move(self, dest = -30, speed = 1, blockMode=False ):     # speed: 1-9
        """  dest: 0-320-640  =-goalAngle-=> -1500 -> 0 -> +1500     |   200/1=200,  200/10=20 ---> 200=2000ms, 20=200ms""" 
        goal = servosGoal(servoID = mc.head_H, goalAngle = (dest-320)*10/4,  goalTime = 300/speed)
        res = self.servo_req(goal)
        return res
    
        
    '''-----------Hands--------------'''
    def shoulderL_move(self, amp=9, speed =1, blockMode=False):
        
        '''print ("---: mc..shoulderL_move.<wait_for_server>.....")
        self.act_client.wait_for_server( )
        print ("---: mc shoulderL_move. Got server!")
        goal = servosGoal(servoID = mc.shoulder_L, goalAngle = amp*120,  goalTime = 200/speed)  
        self.act_client.send_goal(goal)
        print ('left hand uping...Sent goal. goal is:', goal )
        self.act_client.wait_for_result(rospy.Duration.from_sec(5.0) )
        return self.act_client.get_result( )  '''
        
        goal = servosGoal(servoID = mc.shoulder_L, goalAngle = (amp-5)*300,  goalTime = 300/speed)
        res=self.servo_req(goal)
        return res

    def shoulderR_move(self, amp=9, speed =1, blockMode=False):
        print ('Info: Right shoulder moving...speed = %d' %(speed) )
        goal = servosGoal(servoID = mc.shoulder_R, goalAngle = (amp-5)*300,  goalTime = 300/speed)
        res = self.servo_req(goal)
        return res
    
    
    def bigArmL_move(self, amp=9, speed =1, blockMode=False ):
        print ('Info: Left big Arm moving...speed = %d' %(speed) )
        goal = servosGoal(servoID = mc.bigArm_L, goalAngle = (amp-5)*300,  goalTime = 300/speed)
        res = self.servo_req(goal)
        return res

    def bigArmR_move(self, amp=9, speed =1, blockMode=False ):
        print ('Info:right big Arm moving.....speed = %d' %(speed) )
        goal = servosGoal(servoID = mc.bigArm_R, goalAngle = (amp-5)*300,  goalTime = 300/speed)
        res = self.servo_req(goal)
        return res

    def armL_move(self, amp=9, speed =1, blockMode=False):
        print ('Info: Right hand moving...speed = %d' %(speed) )
        goal = servosGoal(servoID = mc.arm_L, goalAngle = (amp-5)*300,  goalTime = 300/speed)
        res = self.servo_req(goal)
        return res


    def armR_move(self, amp=9, speed =1, blockMode=False):
        print ('Info: Right hand moving...speed = %d' %(speed) )
        goal = servosGoal(servoID = mc.arm_R, goalAngle = (amp-5)*300,  goalTime = 300/speed)
        res = self.servo_req(goal)
        return res


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

def test():

    print ("XXXXXXXXXX This is the test of <mc.py>....")     
    rospy.init_node('node_mc_test')
    r = rospy.Rate(1)
    mctl = mc( )
    
    
    try:          
        # use 640*480 camera as input range
        #mctl.head_move(240, 8)
        #mctl.head_move(400, 8)
        #mctl.head_move(320, 8)

        # range: as Robi arch limitation.
        """(5,3,4): Idle
           (4,6,7):   """
        #mctl.shoulderL_move(3, 3)         # range 2->10(front->back).   5 is arm vertical.  
        #mctl.bigArmL_move(3,   3)         # range 2->8(close->open).    5 is arm Horizontal.          
        #mctl.armL_move(2.5,      3)       # range 2->5(close->open). 
        
        
        #mctl.shoulderL_move(1.5, 8)
        # #mctl.bigArmL_move(3,   3)                      
        #mctl.armL_move(3.5,      8)
        #mctl.armL_move(2.5,      4)
       

        """(3,6,5): welcome! """
        mctl.shoulderR_move(5,  1)          # range 8.6->2.6 (front->back)  5 is vertival
        mctl.bigArmR_move(  3,  1)          # range 7.8->3   (close->open)  5 is arm Horizontal.
        mctl.armR_move(     6,  1)          # range 8.1->5   (close->open) 
        
        
        print ('>>> will sleep...')
        r.sleep()
        print ('>>> sleeped.')

            
    except rospy.ROSInterruptException:
        print ("@@ warining: program interrupted before completion")





if __name__ ==  '__main__':
    
    test()
    


    

    
