#!/usr/bin/env python
"""
 top neopallium for mem & forcast. This is the top brain.
 ----
 Licensed under BSD license.
 TODO:  ???use conversation to improve speech recognization like what human do??
 0.2: - 2016.9       Integrate into ROS     
 0.1: - 2015.11.13   Init version by Nick Qian
 ----
 Input:  msgs pubed & data in bb
 Output: service request & action client request 
"""

import sys, os
import rospy

#from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist    #TwistStamped/TwistWithCovariance/TwistWithCovarianceStamped | Vector3 | Wrench |Transform | Quaternion|Pose|Pose2D | Polygon | Point | Inertia |Accel
from sensor_msgs.msg import BatteryState, Imu, LaserScan, RegionOfInterest, Temperature, MagneticField, Range

import actionlib
#from audioprocess.msg import mouthAction,mouthGoal,mouthResult
from cortex.msg import cortexpAction, cortexpGoal, cortexpResult
from visualprocess.msg import visualMsg
from sensorprocess.msg import sensorsMsg
from bt import bt as btree

from actprocess.msg    import actPatternAction,actPatternGoal, actPatternResult, actPatternFeedback
from pi_trees_ros import *


'''
from actionprocess.action import talkGesture                     #import action file
'''

class cortexn(object):
    '''
    Heart Of the Machine/me
    '''
    def __init__(self):

        # Init node firstly
        rospy.init_node('node_cortexn')
        self.rate = rospy.Rate(1)        # 2=2Hz
        
        # build the behavior tree
        self.bt = btree()
                                                               # 100: tedious
        
        # Fetch from BB
        self.age  = self.bt.bb.age
        self.name = self.bt.bb.name
        self.totalHealth = self.bt.bb.totalHealth
        self.personality = self.bt.bb.personality
        self.playDesire = self.bt.bb.playDesire
        self.achievementDesire = self.bt.bb.achievementDesire
        self.creativeDesire = self.bt.bb.creativeDesire
        #self.environ )
       
        
        self.energy = 0

        # Update from tpc
        self.battery      = 0                           
        self.mechanic     = 0
        self.selftest     = 0
        self.vtrack_once  = 0

        # flags for subscribers
        self.flg_sub_cortexp = None
        self.flg_sub_cortexa = None
        self.flg_sub_visual = None
        self.flg_sub_sens = None


        #Insts for interfaces
        self.actclient_cortexp = actionlib.SimpleActionClient('CortexpAction', cortexpAction )
        print ("####: cortexn init done!!! ")

        

    #------- launch -------
    def launch(self):
        while not rospy.is_shutdown():
            
            self.subToExternal( )
            self.bb_update( )
            status = self.bt.BEHAVE.run( )
            print ("###################### BT run once in (cortexn). Will sleep...####################")
            self.rate.sleep()
            print ("##########sleep wakeup...########")



    def bb_update(self):       

        #--------fetch from bb to update local var --------------
        self.age  = self.bt.bb.age
        self.name = self.bt.bb.name
        self.totalHealth = self.bt.bb.totalHealth
        self.personality = self.bt.bb.personality
        self.playDesire = self.bt.bb.playDesire
        self.achievementDesire = self.bt.bb.achievementDesire
        self.creativeDesire = self.bt.bb.creativeDesire

        #------------ update bb --------------
        self.energy = (self.battery + self.mechanic + self.playDesire + self.achievementDesire + self.creativeDesire )/ 6  # 1-depressed;   50-mild   100-energy full
        self.bt.bb.energy = self.energy



            

    def subToExternal(self):
        #self.sub_cortexa( )
        self.sub_visual( )
        #self.sub_cortexp( )
        #self.sub_sens( )
            

    #---------------- (1): actclient to cortexp for big act req -------------------
    def actclient_req_cortexp(self ):                 
        client = actionlib.SimpleActionClient('bt_act_client', DoDishesAction)
        client.wait_for_server( )
        client.send_goal(DoDishGoal(en_headtrack = True))
        client.wait_for_result(rospy.Duration.from_sec(5.0) )
        

    #------------------------ (2): actclient to mouth -----------------------
    def actclient_req_mouth(self, cmd_brain, content, txt):      
        
        try:

            return resp.status
        except rospy.ServiceException, err:
            print "Warning: Service call failed: %s" %err
            
            
    #------------------------ (3): subscriber to visual ---------------------
    def sub_visual(self ):       
        rospy.Subscriber('tpc_visual', visualMsg, self.callback_tpc_visual )
        self.flg_sub_visual = False
        self.rate.sleep()
        
    
    def callback_tpc_visual(self, data):
        
        if self.flg_sub_visual is not True:
            roi = data.roi
            motion_detected = data.motion_detected
            print ("******* get msg from tpc_visual. roi/motion_detected are:", roi, motion_detected)
            #peopleDetects = data.peopleDetects
            #peopleRecog   = data.peopleRecog            
            #objDetects    = data.objDetects
            #ooi
            #roi
            self.flg_sub_visual = True


    #----------------------- (4): subscriber to cortexp --------------------------
    def sub_cortexp(self):  
        rospy.Subscriber("tpc_cortexp", cortexpMsg, slef.callback_tpc_cortexp )
        
        self.flg_sub_cortexp = False
        self.rate.sleep( )
        

    def callback_tpc_cortexp(self,data):
        vtrackEvt = data.evt_visual_track
        if cortexpMsg.VTRACK_DONE == vtrackEvt:
            self.vtrack_once = True
            
            print ("====hahahaha ====")

        # Set the flag
        self.flg_sub_cortexp = True            
   

        
    #--------------------- (5) service req to cortexp for neo_cmd----------------------------
    def proxy_cortexp(self, cmd_neo, ):            # send NEO-CMD to cortexp     
        rospy.wait_for_service('srv_cortexp')
        print ('Info: End <wait_for_service> in <client_cortexp_req>' )
        try:
            handle_neo = rospy.ServiceProxy('srv_cortexp', cortexpSrv)
            result = handle_neo(cmd_neo, queryData )                                # ?
            return result.result                                                                            # ?
        except rospy.ServiceException, err:
            print "Warning: Service call failed: %s" %err
            


    #---------------------- (6) subscriber to sens ----------------------------
    def sub_sens(self):     
        rospy.Subscriber('tpc_sens', sensorsMsg, callback_tpc_sens)
        self.flg_sub_sens = False
        self.rate.sleep()

    def callback_tpc_sens(self, data):
        self.battery = data.battery
        self.mechanic = data.mechanic
        #header = data.header
        timestamp = header.stamp.to_sec( )
        # print rospy.get_caller_id(), header.seq, 'Info: cortexa heard that battery= %d mechanic = %d at %12f' %(self.battery,  self.mechanic, timestamp)

        self.flg_sub_sens = True



    #----------------------- (7) subscriber to cortexa -------------------------
    def sub_cortexa(self):  
        rospy.Subscriber('tpc_cortexa', cortexaMsg, callback_tpc_cortexa)
        self.flg_sub_cortexa = False
        self.rate.sleep()
        

    def callback_tpc_cortexa(self, data):
        self.selftest   = data.selftest
        timestamp       = header.stamp.to_sec( )
        self.battery    = data.battery
        self.mechanic   = data.mechanic
        print (rospy.get_caller_id(), header.seq, 'Info: cortexa heard that selftest= %d  time: %12f' %(self.selftest, timestamp) )

        self.flg_sub_cortexa = True

    #-------------------------- Local func ----------------------------

        


    def basicLifeRun(self):
        if  self.energy < 20:
            print ('Info: Energy < 20, i\'m feeling depressing... let me check...')
            client_mouth_req()
        elif self.energy > 90:
            print ('AAAAAA...refreshed!!! I am feeling good! ')
        else:
           print ('basicLifeCheck: totalHealth = ', cortexa.energy )

        print ('cortexa.battery = ', cortexa.battery)
            
        if (cortexa.behavior != 'playing'):
            cortexa.play -= 1
        else:
            cortexa.play += 1

        if (cortexa.behavior =='playing') | (cortexa.behavior =='walking'):
            cortexa.mechanic -=1
        else:
            cortexa.mechanic +=1
            
        if (cortexa.behavior =='watching') |(cortexa.behavior =='talking') :
            cortexa.creative +=1
        elif (cortexa.behavior =='idle'):
            cortexa.creative -=1
        else:
            pass            

        t1_tickle = threading.Timer(2.0, self.basicLifeRun )
        t1_tickle.start()

        print ('---->')
        cortexa.energy = (cortexa.battery + cortexa.mechanic + cortexa.play + cortexa.achievement + cortexa.creative + cortexa.environ )/ 6      
        cortexa.count += 2
        servoMove(cortexa.count)
        print ('cortexa.energy = ', cortexa.energy)
        self.basicLifeCheck()


    
class meetPerson:
    cvst=list()   #conversation[]
    mem=list()
    
    def __init__(self, face):        
        if faceMatch(face, nameOut) == True:
            self.personName = nameOut
    def recognizePerson(self, face):
        self.name = opencv(face)
        return self.name
    def newPerson(self, ):
        pass
    
    def sayHi(self):
        if recognizePerson(self, face) is not 0:
            print ('Hi, my old friend, %s' %self.name)
        else:
            newPerson()


def act_client_req(goal):
    #req_servoID,  req_goalAngle, req_goalTime = act_req
    act_client = actionlib.SimpleActionClient('srv_actPattern',  actPatternAction )             #( )
          
    act_client.wait_for_server( )                      # wait until the server started.  listening for goals
    act_client.send_goal(goal)
    act_client.wait_for_result(rospy.Duration.from_sec(1.0) )

    print ("~~~~~~~~~act client req(sending goal) once ~~~~~~~~")
    res = act_client.get_result( )

    return res

def testX():
    crn = cortexn()
    
    def have_rest_actionFB_cb():
        print ("Info: In (HaveRest).have_rest_actionFB_cb....feedback is:", feedback)
    def have_rest_actionRS_cb(result):
        print ("Info: In (HaveRest).have_rest_actionRS_cb....result is:", result)

    goal_actPattern = actPatternGoal(op = actPatternGoal.PATTERN_OPERATION,
                                     pattern = actPatternGoal.TALK_GESTURE,
                                     talk_gesture = 'TG_KILLTIME')

    act_client_req(goal_actPattern)

    print ("XXXXXX---> will do SimpleActionTask()")
    
    SimpleActionTask("HAVE_REST_ACT", 'srv_actPattern', actPatternAction, goal_actPattern,
                                         feedback_cb=have_rest_actionFB_cb, done_cb = have_rest_actionRS_cb)
            
    print ("========>>>> self.goal_actPattern is:", goal_actPattern)
        
    

def main():
    crn = cortexn()
    crn.launch()





if __name__ == '__main__':
    testX()
    main()
    
    
    
