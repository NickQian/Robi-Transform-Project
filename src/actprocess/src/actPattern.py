#!/usr/bin/env python
"""
    action process top file 
    Work as actionlib server, and subscriber to other topic.
    ----
    Licensed under BSD license. 
    by Nick Qian  -- 2015.11.23
    ----
    Inputs: CMD behavoir from cortexp (using actionlib server)
    Output: action commands (API) --> mc.py
    ----
"""

from mc import mc
import roslib
roslib.load_manifest('actprocess')
import rospy
import actionlib

from actprocess.msg import actPatternAction,actPatternGoal,actPatternResult,actPatternFeedback
from mc import mc


class actPattern(mc):      # actionlib server
    _feedback = actPatternFeedback( )
    _res = actPatternResult( )
    
    def __init__(self):
        mc.__init__(self)
        rospy.init_node('node_actPattern')
        self._as = actionlib.SimpleActionServer('actPattern', actPatternAction, self.execute, False) #name of the server

        self.r = rospy.Rate(2)                # server will act per 1s?  ?? work with sleep()?.  If no rate, work with spin()?
      
        #self.mc = mc_actclntmc( )
        self.sw_visualTrack = None
        self.roi = None
        print ("Info: actPattern initialized.")
        
    def execute(self, goal):
        
        rospy.loginfo('Info: actlib server goal got in <actPattern>:', goal)
        success = True

        #check preempted firstly
        if self._as.is_preempt_requested():
            rospy.loginfo("!!actPattern preempted!! " )
            self._as.set_preempted()
            success = False                          
        
        if (goal.op == actPatternGoal.MOVEIT_OPERATION):
            raise NotImplementedError
        
        if (goal.op == actPatternGoal.ELABORATE_OPERATION):
            raise NotImplementedError
      
        if (goal.op == actPatternGoal.PATTERN_OPERATION):
            # if "talk gesture"
            if goal.pattern == actPatternGoal.TALK_GESTURE:
                self.talkGesture(goal.talk_gesture)
            #if  "visual track"   
            if goal.pattern == actPatternGoal.VISUAL_TRACK:
                self.sw_visualTrack = goal.visual_track_switch
                self.roi = goal.roi
                self.visualTrack( )
                
            #if "emotion language"
            if goal.pattern == actPatternGoal.EMOTION_LANG:
                pass
      

        # publish the feedback
        self._as.publish_feedback(self._feedback)

        # sleep
        self.r.sleep()

        if success:
            self._as.set_succeeded(self._res)
            rospy.loginfo("!!actPattern: Succeeded!! ")

        #rospy.spin( )

            

    def run(self):
        self._as.start( )
        print ("Info: actPattern Server started. enter in sleep")
        rospy.spin()
        
        #self.r.sleep()
        
        
    #---------------pattern: talk gesture ----------------------
    def visualTrack(self):
        if self.sw_visualTrack is True:        # local switch of motion detection
            print("@@@ visual track loop....self.roi.x is:", self.roi.x)
            dest = self.roi.x
            res = self.head_move(dest )
            print ("~~~~~~~`actPattern finished head_move. Enter sleep...result is:", res)
            return res.result
        
        else:
            
            return None
           

        
    def talkGesture(self, gesture):
        if gesture is TG_BORING:
            self.gestureBoring()
        if gesture is TG_HELLO:
            self.gestureHi( )
        if gesture is TG_BYE:
            self.getsureBye( )

    def gestureHi( ):
        self.mc.armR_up(amp=8, speed = 9)

    def gestureNo():
        self.mc.armR_left(amp=8, speed = 9)
        self.mc.armR_right(amp=5, speed = 9)
        self.mc.armR_left(amp=8, speed = 9)
        self.mc.armR_right(amp=5, speed = 9)

    def gestureYes():
        pass

    def gestureYes_byHead( ):
        pass

    def gestureNo_byHead( ):
          moveHead(left,30)
          moveHead(right, 60)
          moveHead(left, 60)
          moveHead(right, 40)          

    def gestureGreat():
        pass

    def gestureOK():
        pass

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



def test_with_client():
    
    #setup a client
    act_client = actionlib.SimpleActionClient('actPattern', actPatternAction)
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
  

   
          
    
    
