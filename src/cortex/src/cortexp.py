#!/usr/bin/env python
"""
 paleopallium level. As a animal processes move/SLAM etc...
 -----
 Licensed under BSD license. 
 by Nick Qian  -2015.11.13
 -----
 input:  pubed msgs and neo cmd from cortexn
 Output: action client req
"""

import random
import roslib
roslib.load_manifest('actprocess')
import rospy
import traceback

import actionlib
from actprocess.msg import actPatternAction, actPatternGoal, actPatternFeedback, actPatternResult

from cortex.msg import cortexaMsg, cortexpMsg
from cortex.msg import cortexpAction,cortexpGoal,cortexpFeedback,cortexpResult
from cortex.srv import cortexpSrv
from sensor_msgs.msg import Image, RegionOfInterest
from visualprocess.msg import visualMsg
from sensorprocess.msg import sensorsMsg            #BatteryStateMsg, compassMsg, dustMsg, motorsMsg
#from audioprocess.msg import micMsg

class cortexp( ):
     ''' Mainly process the neo-cmd: 'TRACK_ROI'/'TALK_GESTURE'/'HAND_OP'/'LEG_OP'/
     construct the space model bases his own postion(coordinate) 
     
     x, y, z =0, 0, 0
     locPlane = [x, y]
     locSpace = [locPlane, z]
     spaceView = [   [x+2,y+2, z+2],
                                   [x+2,y+2, z  ],
                                   [x+2,y-2, z+2],
                                   [x+2,y-2, z  ],
                                   [x-2,y+2, z+2],
                                   [x-2,y+2, z  ],
                                   [x-2,y-2, z+2],
                                   [x-2,y-2, z]            ]
     spaceCovered = []
     '''
     _feedback = actPatternFeedback()
     _res = actPatternResult( )
     
     def __init__(self):
          self.ooi = False
          self.roi = None
          self.motion_detected = None
          
          #self.actGoal = actprocess.msg.servosGoal(servoID= req_servoID, goalAngle = req_goalAngle, goalTime = req_goalTime)
          self.actGoal = actPatternGoal( )          
          self.mycortexpMsg = cortexpMsg()
          #print ("====>>> type of actPatternAction, actPatternGoal:", actPatternAction, actPatternGoal )

          # Insts for Interfaces
          self._as = actionlib.SimpleActionServer('CortexpAction', cortexpAction, self.cortexpActionExecute, False) #name of the server
          self.cortexp_pub = rospy.Publisher("tpc_cortexp", cortexpMsg, queue_size = 5)
          
          self.cmd_head_mode = 'TRACK_ROI'
          self.cmd_head = [7, 5]                                                                                                       #req position, speed
          self.sound_direction = 0
          self.motion_detect_block = False
          self.cmd_arms = ''          

          rospy.init_node('node_cortexp')                                                                                                  #Init node for node?? client don't need to init node?
          self.act_client = actionlib.SimpleActionClient('srv_actPattern',  actPatternAction )             #( )
          #self.act_client.wait_for_server( )                                                                                      # until the server has started up, listen for the goal
          self.rate = rospy.Rate(2)           #1Hz
          
          self.visual_cb_flag = False
          self.sub_count = 0
          self.visual_track_switch_last = False

     
     def launch(self):
          while not rospy.is_shutdown():               
               #self.sub_cortexa()
               #self.sub_sensors()
               
               self.sub_visual()
               #self.sub_mic( )
               #self.service_cortexp()                                # process cmd from cortexn
               print ("~~~ cortexa launch loop once. Will sleep...")
               self.rate.sleep( )


     #---------------------- (1)actserver to <cortexn> ---------------------
     def cortexpActionExecute(self, goal):
          """
             actlib server for cortexn requests
          """
          rospy.loginfo("actlib server goal got in <cortexp>,", goal)
          success = True

          # check preempted firstly
          if self._as.is_preempt_requested():
               rospy.loginfo("!!action server of cortexp is preempted!!")
               self._as.set_preempted()
               success = False

          # do something here

          # publish the feedback
          self._as.publish_feedback(self._feedback)

          # sleep
          self.rate.sleep()

          if success:
               self._as.set_succeeded(self._res)
               rospy.loginfo("!! actlib server: Succeeded!! ")

          
          
     #---------------------- (2)act client to <actPattern> -----------------
     def act_client_req(self, goal):    

          #req_servoID,  req_goalAngle, req_goalTime = act_req          
          self.act_client.wait_for_server( )                      # wait until the server started.  listening for goals
          self.act_client.send_goal(goal)
          self.act_client.wait_for_result(rospy.Duration.from_sec(1.0) )

          print ("~~~~~~~~~act client req(sending goal) once ~~~~~~~~")
          res = self.act_client.get_result( )

          return self.act_client.get_result( )
     

     #---------------------- (3) sub to cortexa -----------------------------
     def sub_cortexa(self ):
          rospy.Subscriber("tpc_cortexa", sensorsMsg, self.callback_tpc_cortexa)
          
          #rospy.spin( )

     def callback_tpc_cortexa(data):
          self.battery = data.battery
          self.mechanic = data.mechanic


     #---------------------- (4) sub to sensors-------------------------------
     def sub_sensors(self ):        
          rospy.Subscriber("tpc_sens", sensorsMsg, self.callback_tpc_sens)
          #rospy.spin( )

     def callback_tpc_sens(self, data):
          self.battery = data.battery
          self.mechanic = data.mechanic
          #header = data.header
          timestamp = header.stamp.to_sec( )
          print rospy.get_caller_id(), header.seq, 'Info: cortexa heard that battery= %d mechanic = %d at %12f' %(self.battery,  self.mechanic, timestamp)
                  


     #---------------------- (5) service to cortexn-------------------------------
     def service_cortexp(self):      
          self.service = rospy.Service('srv_cortexp', cortexpSrv, handle_neo_cmd )
          print ("Info: cortexp Service ready.")
          rospy.spin( )                        # keep until service is shutdown

     def handle_neo_cmd(self, req):
          print ("Info: handling request to cortexp, req is", req)
          if req.neo_cmd is 'TRACK_ROI':               
               #moveHeadForVisualTrack()
               visualTrackGoal.goalAngle = self.sound_position
               visualTrackGoal.goalTime = 0.3
               self.act_client_req(visualTrackGoal)
          elif req.neo_cmd ==  'TALK_GESTURE':
               #talkGestureGoal. =
               pass
          elif req.neo_cmd ==  'HAND_OP_CATCH':
               '''this is hard with 2 cameras to identify the postion of the obj and identify the position bases on robot itself....'''
               pass
          elif req.neo_cmd ==  'HAND_OP_REJECT':
               '''this is hard with 2 ...........................'''
               pass
          elif req.neo_cmd ==  'LEG_OP_MOVE':
               pass
          elif req.neo_cmd ==  'LEG_OP_STAND':
               pass
          
          else:
               print ('Warning: None implemented neo_cmd in <handle_neo_cmd>. cmd is %s' %req.neo_cmd)
          
          req.result = "Done"
          return req.result
     
     
     #---------------------- (6) subscriber to tpc_visual ---------------------------
     def sub_visual(self):    # (5)      
          rospy.Subscriber("tpc_visual", visualMsg, self.callback_tpc_visual) #name, data_class, callback, callback_args, queue_size, buff_size, tcp_nodelay
          self.sub_count += 1
          self.visual_cb_flag = False
          print ("@@ Sub to tpc_visual once. Enter in r.sleep()....sub_count is: ", self.sub_count)
          #rospy.spin( )    # spin() will continue loop this def
          self.rate.sleep( )
                    

     def callback_tpc_visual(self, data):
          
          #objDetects = data.objDetects
          #peopleDetects = data.peopleDetects
          #peopleRecog = data.peopleRecog
          #self.ooi = data.ooi                 # bool
          
          if self.visual_cb_flag is False:
               header = data.header
               self.roi = (data.roi.x, data.roi.y, data.roi.height, data.roi.width)
               self.motion_detected = data.motion_detected
               print ("****call back execute once. get pub msg. motion_detected/roi ", self.motion_detected, self.roi)
               if self.motion_detected  and (not self.motion_detect_block):
                    #self.actGoal.goal_id = "actPattern"
                    #self.actGoal.header =
                    self.actGoal.op = actPatternGoal.PATTERN_OPERATION
                    self.actGoal.pattern = actPatternGoal.VISUAL_TRACK
                    self.actGoal.visual_track_switch = True
                    
                    (self.actGoal.roi.x, self.actGoal.roi.y, self.actGoal.roi.height, self.actGoal.roi.width) = self.roi
                    print ('===CMD MOVE=== Will do client req in callback. switch is:, x is:', self.actGoal.visual_track_switch, self.actGoal.roi.x)
                    res = self.act_client_req(self.actGoal)
                    
                    if actPatternResult.SUCCEEDED == res.result:
                         self.mycortexpMsg.evt_visual_track = cortexpMsg.VTRACK_DONE
                         try:
                              self.cortexp_pub.publish( self.mycortexpMsg )
                         except:
                              traceback.print_exec()
                              print ("~~~~~~~~~~~~ DBG:: Pub Once!~~~~~~~~~~~")
               else:
                    self.actGoal.visual_track_switch = False               
                    print ('=--CMD NO MOVE--=Will do client req in callback. switch is:, x is:', self.actGoal.visual_track_switch, self.actGoal.roi.x)
                    #res = self.act_client_req(self.actGoal)
                    
                              
               self.visual_track_switch_last = self.motion_detected            # for memory
               
          # Set the flag
          self.visual_cb_flag = True
        

     #---------------------- (7) subscriber to tpc_mic ---------------------------------
     def sub_mic(self):   
          rospy.Subscriber('tpc_mic', micMsg, callback_mic)
          

     def callback_mic(self, data):
          #self.sound_direction = data.soundDirection
          #self.stt_result = data.txt
          self.sound_position = random.randint(-90, 90)

     """def moveHeadForVisualTrack(self, position, time):
          pass

     """
          


class newItem():
    def __init__(self):
        pass
    def matchMemoryKind():
        pass
    def newKind():
        pass
    def NewKindQuery():
        pass
    def storeTnAround():
        pass

class newPerson(newItem):
    def __init__(self):
        pass
    def matchMemoryPerson():
        pass
    def newPerson():
        pass
    def newPersonQuery():
        pass
    def storeToAround():
        pass

class newAnimal(newItem):
    def __init__(self):
        pass
    def matchMemoryAnimal():
        pass

class newPlant(newItem):
    def __init__(self):
        pass
    def matchMemoryPlant():
        pass


if __name__ == '__main__':
     cp = cortexp( )
     try:
          cp.launch()
     except rospy.ROSInterruptException:
          print "@@@: program interrupted."
          
    

