#!/usr/bin/env python
"""
    action process top file 
    Work as actionlib server, and subscriber to other topic.
    ----
    Licensed under BSD license.
    
    0.1 - 2015.11.23 init version by Nick Qian- 
    ----
    Inputs: CMD behavoir from cortexp (using actionlib server)
    Output: action commands (API) --> mc.py
    ----
"""

import random
import traceback

from mc import mc
import roslib
roslib.load_manifest('actprocess')
import rospy
import actionlib

from actprocess.msg import actPatternAction,actPatternGoal,actPatternResult,actPatternFeedback


class actPattern(mc):      # actionlib server
    _feedback = actPatternFeedback( )
    _res = actPatternResult( )
    
    def __init__(self):        
        rospy.init_node('node_actPattern')
        mc.__init__(self)
        
        self._as = actionlib.SimpleActionServer('srv_actPattern', actPatternAction, self.execute_cb, False) #name of the server

        self.r = rospy.Rate(2)                # server will act per 1s?  ?? work with sleep()?.  If no rate, work with spin()?
      
        #self.mc = mc_actclntmc( )
        self.sw_visualTrack = None
        self.roi = None
        print ("Info: actPattern initialized.")
        
    def execute_cb(self, goal):
        
        rospy.loginfo('Info: actlib server goal got in <actPattern>:'); print (goal)
        start_time = rospy.get_rostime()
        try:
            success = True
            self._feedback.crt_process = 1
            self._res.result = actPatternResult.ACTIVE

            #check preempted firstly
            if self._as.is_preempt_requested():
                rospy.loginfo("!!actPattern preempted!! " )
                self._as.set_preempted()
                self._res.result = actPatternResult.PREEMPTED
                success = False

            if (goal.op == actPatternGoal.MOVEIT_OPERATION):
                raise NotImplementedError
        
            if (goal.op == actPatternGoal.ELABORATE_OPERATION):
                raise NotImplementedError
      
            if (goal.op == actPatternGoal.PATTERN_OPERATION):

                #--- if "talk gesture"
                if goal.pattern == actPatternGoal.TALK_GESTURE:
                    rospy.logwarn("----------YYEESS!!! actPatternGoal.TALK_GESTURE !!!-----------")
                    self.talkGesture(goal.talk_gesture)
                
                
                #--- if  "visual track"   .
                if goal.pattern == actPatternGoal.VISUAL_TRACK:
                    #rospy.logWARN("XXXXDBGXXXX")
                    self.sw_visualTrack = goal.visual_track_switch
                    self.roi = goal.roi
                    self.execute_visualTrack( )
                
                #--- if "emotion language"
                if goal.pattern == actPatternGoal.EMOTION_LANG:
                    pass

                self._feedback.crt_process = 100
                

            # publish the feedback
            self._as.publish_feedback(self._feedback)

            # sleep
            self.r.sleep()

            if success:
                self._as.set_succeeded(self._res)
                self._res.result = actPatternResult.SUCCEEDED    #???
                rospy.loginfo("!!actPattern: Succeeded!! ")
        except Exception, e:
            rospy.logerr("Exception in actlib callback: %s" %str(e))
            rospy.loginfo(traceback.format_exc() )
        finally:
            rospy.logdebug("Done actionlib callback")

            


    def run(self):
        self._as.start( )
        print ("Info: actPattern Server started. enter in spin...")
        rospy.spin()                
        
    #--------------- visualTrack ----------------------
    def execute_visualTrack(self):
        if self.sw_visualTrack is True:        # local switch of motion detection
            print("@: visual track loop....self.roi.x is:", self.roi.x)
            dest = self.roi.x
            res = self.head_move(dest )
                      
            print ("~~~~~~~`actPattern finished head_move. Enter sleep...result is:", res)
            return res.result
        
        else:
            
            return None
           
    #----------------- pattern: talk gesture ------------------------
        
    def talkGesture(self, gesture):
        print ("===========> gesture is:", gesture)

        if gesture == 'TG_KILLTIME':
            self.gestureKillTime( )
            rospy.logwarn("~~~~ Done <TG_KILLTIME> ~~~")
        
        if gesture == 'TG_BORING':
            self.gestureBoring()
        if gesture == 'TG_HELLO':
            self.gestureHi( )
        if gesture == 'TG_BYE':
            self.getsureBye( )
        if gesture == 'TG_WELCOME':
            self.gestureWelcome()
        if gesture == 'TG_IDLE':
            self.gestureIdle( )

            

    def gestureWelcome(self):
        tup_welcome = ( (4,5,0),(3.5,5,0),(5,5,0) )
        
        self.shoulderR_move(tup_welcome[0][0],  tup_welcome[0][1])
        self.bigArmR_move(  tup_welcome[1][0],  tup_welcome[1][1])
        self.armR_move(     tup_welcome[2][0],  tup_welcome[2][1])

    def gestureIdle(self):        
        #tup_idle_L = ( (5,5,0),(3,5,0),(4,5,0) )
        tup_idle_L = ( (3,  5,0),(3,5,0),(3.2,5,0) )
        tup_idle_R = ( (6.7,5,0),(7,5,0),(7,5,0) )

        self.shoulderL_move(  tup_idle_L[0][0],  tup_idle_L[0][1])
        self.bigArmL_move(    tup_idle_L[1][0],  tup_idle_L[1][1])
        self.armL_move(       tup_idle_L[2][0],  tup_idle_L[2][1])
        
        self.shoulderR_move(  tup_idle_R[0][0],  tup_idle_R[0][1] )
        self.bigArmR_move(    tup_idle_R[1][0],  tup_idle_R[1][1])
        self.armR_move(       tup_idle_R[2][0],  tup_idle_R[2][1])

    def gestureKillTime(self):
        seed = random.randint(1,7)
        tup1_KillTime_L = ( (3,  4,0),(3.5, 4,0),(3.7, 4, 0) )
        tup2_KillTime_L = (2,   2, 0)
        tup3_KillTime_R = ( (6.7,4,0),(6.5, 4,0),(6.5, 4, 0) )
        #tup3_KillTime_R = (8.5, 5, 0)
        tup4_KillTime_L = (4, 2, 0)
        tup5_KillTime_R = (7.5, 2, 0)
        tup6_KillTime_HeadL = ( 200, 4, 0)
        tup7_KillTime_HeadR = (-200, 4, 0)
        
        if seed == 1:
            self.shoulderL_move(  tup1_KillTime_L[0][0],  tup1_KillTime_L[0][1])
            self.bigArmL_move(    tup1_KillTime_L[1][0],  tup1_KillTime_L[1][1])
            self.armL_move(       tup1_KillTime_L[2][0],  tup1_KillTime_L[2][1])
        
            self.gestureIdle()
            
        if seed == 2:
            self.shoulderL_move(tup2_KillTime_L[0], tup2_KillTime_L[1])
            self.gestureIdle()            

        if seed == 3:
            self.shoulderR_move(  tup3_KillTime_R[0][0],  tup3_KillTime_R[0][1])
            self.bigArmR_move(    tup3_KillTime_R[1][0],  tup3_KillTime_R[1][1])
            self.armR_move(       tup3_KillTime_R[2][0],  tup3_KillTime_R[2][1])            
            #self.shoulderR_move(tup3_KillTime_R[0], tup3_KillTime_R[1])
            
            self.gestureIdle()

        if seed == 4:
            self.bigArmL_move(tup4_KillTime_L[0], tup4_KillTime_L[1])
            self.gestureIdle()

        if seed == 5:
            self.bigArmR_move(tup5_KillTime_R[0], tup5_KillTime_R[1])
            self.gestureIdle()

        if seed == 6:
            self.head_move(tup6_KillTime_HeadL[0], tup6_KillTime_HeadL[1] )
            self.r.sleep(3)

        if seed == 7:
            self.head_move(tup7_KillTime_HeadR[0], tup7_KillTime_HeadR[1] )
            self.r.sleep(3)
            
        

    def gestureHi( self):
        tup_Hi_R         = ( (8.5,5,0),(7,5,0),(7,5,0) )
        tup_Hi_R_Realse1 = ( (8,  9,0),(7,5,0),(7,5,0) )
        tup_Hi_R_Realse2 = ( (6.7,2,0),(7,5,0),(7,5,0) )
        
        self.shoulderR_move(  tup_Hi_R[0][0],  tup_Hi_R[0][1] )
        self.bigArmR_move(    tup_Hi_R[1][0],  tup_Hi_R[1][1] )
        self.armR_move(       tup_Hi_R[2][0],  tup_Hi_R[2][1] )
        
        self.shoulderR_move(  tup_Hi_R_Realse1[0][0],  tup_Hi_R_Realse1[0][1] )
        self.shoulderR_move(  tup_Hi_R_Realse2[0][0],  tup_Hi_R_Realse2[0][1] )
        
        

    def gestureNo(self):
        self.mc.armR_left(amp=8, speed = 9)
        self.mc.armR_right(amp=5, speed = 9)
        self.mc.armR_left(amp=8, speed = 9)
        self.mc.armR_right(amp=5, speed = 9)

    def gestureYes(self):
        pass

    def gestureYes_byHead( self):
        pass

    def gestureNo_byHead( self):
          moveHead(left,30)
          moveHead(right, 60)
          moveHead(left, 60)
          moveHead(right, 40)          

    def gestureGreat(self):
        pass

    def gestureOK(self):
        pass
    

    def testGesture(self):
        """shoulderL: range 2-  >10 (front->back).   5 is arm vertical.  
           bigArmL:   range 2  ->8  (close->open).   5 is arm Horizontal.          
           armL:      range 2  ->5  (close->open).
           shoulderR: range 8.6->2.6(front->back)    5 is vertival
           bigArmR:   range 7.8->3  (close->open)    5 is arm Horizontal.
           armR:      range 8.1->5  (close->open)
        """
        
        try:
            self.talkGesture('TG_IDLE')
            self.talkGesture('TG_HELLO')
            self.talkGesture('TG_KILLTIME')
            

            print ('>>> will sleep...')
            self.r.sleep()
            print ('>>> sleeped.')

            #print ('>>> Enter in spin.')
            #rospy.spin( )
            #print ('>>> End spin wake up.')
            
        except rospy.ROSInterruptException:
            print ("@@ warining: program interrupted before completion")


    #-----------------pattern: Emotion Language ------------------------
    def emoEyeRoll(emotionType):
        if emotionType == 3:
            pass

    #--------------------pattern Follow Item -------------------------------
    def followItem():
        pass

    #================= Elaborate operation =======================
    def handlCatch(self, speed, amp):
        print ('left hand catching...speed =s%', speed)

    def handlRelease(self, speed, amp):
        print ('left hand releasing...')

    def handrCatch(self, speed, amp):
        print ('right hand catching...')

    def handrRelease(self, speed, amp):
        print ('right hand releasing...')



class gestureGen(mc):
    tup_welcome = ( (5,5,0),(2,5,0),(4,5,0) )   # (position, speed, isPreempt), should, bigArm, arm
    tup_rest = ()

    def __init__(self, ):
        mc.__init__()

    

def test_with_client():
    
    #setup a client
    act_client = actionlib.SimpleActionClient('srv_actPattern', actPatternAction)
    act_client.wait_for_server( )
    #rate = rospy.Rate(1)

    #send goal and request:
    goal = actprocess.msg.actPatternGoal(op = PATTERN_OPERATION, pattern = TALK_GESTURE, sw_visual_track = True, talk_gesture = TG_HELLO )
    #send goal
    act_client.send_goal(goal)
    act_client.wait_for_result(rospy.Duration.from_sec(5.0) )
    #return self.act_client.get_result( )
    


if __name__ == '__main__':
    act = actPattern()
    act.run()
    #act.testGesture()
    
  

   
          
    
    
