#!/usr/bin/env python

"""
 top file of the behavior tree
 ----
 Licensed under BSD license. 
 0.1: - 2016.8.5  init version by Nick Qian
 ----
 Input:  msgs pubed & data in bb
 Output: service request & action client in bridge "pi_trees_ros"
 
"""

import sys, os
import random
import rospy
from std_msgs.msg import Float32, String
import time

from geometry_msgs.msg import Twist

#from visualprocess.srv import ObjRecognizeSrv

import actionlib
from audioprocess.msg  import mouthAction,     mouthGoal,      mouthResult,      mouthFeedback
from actprocess.msg    import chargeAction,    chargeGoal,     chargeResult,     chargeFeedback
from actprocess.msg    import moveAction,      moveGoal,       moveResult,       moveFeedback
from actprocess.msg    import actPatternAction,actPatternGoal, actPatternResult, actPatternFeedback
from cortex.msg        import cortexpAction,   cortexpGoal,    cortexpResult,    cortexpFeedback
from visualprocess.msg import visualMsg
from visualprocess.srv import visualSvc, visualSvcRequest, visualSvcResponse
from audioprocess.msg  import soundMsg
from sensorprocess.msg import sensorsMsg

sys.path.append("/home/pi/pi/robi/lib/audioprocess/install")
from ap_o import mutterGen, strikeUpTalkGen, reportGen

# import pi_trees bt
from bt_lib import *
from pi_trees_ros import *
from bt_bb import BlackBoard, course_en_fp



class bt(object):
    cnt_ActMode_Idle  = 0
    cnt_ActMode_Play  = 0
    cnt_Doing_Follow  = 0           
    cnt_MindIdle = 0
    cnt_mutterGenBlock = 0
    cnt_DoingTalk = 0
    cnt_DoingMutter = 0
    
    def __init__(self):
        super(bt, self).__init__()
        # Init parameters and vars
        #setup_task_environment(self)

        # Init the BB
        self.bb = BlackBoard( )

        # rate to tick the tree 
        #rospy.init_node('node_bt')
        self.tic = rospy.Rate(2)
        

        #----- parameters -----
        self.dotfp = rospy.get_param('~dotfp', None)
        self.low_battery_threshold = rospy.get_param('~battery_threshold', 20)
        self.mechanicMaxTmpr_threshold = rospy.get_param('~mechanicMaxTmpr_threshold', 70)
        self.Param_Max_MutterGenBlock = rospy.get_param('~Param_Max_MutterGenBlock', 5)

        #-----visual -----
        self.visualSvcCMD = visualSvcRequest.FACE_RECOG       # default
        self.visualSvcROI = (0,0,400,400)
        print ("@@@@ visualSvcRequest(...): ",visualSvcRequest(cmd=self.visualSvcCMD, roi=self.visualSvcROI) )
        self.rectFacesDetected = None

        #---- sound ----
        self.stt_result = None
        self.hasSoundInEvt = None
        self.hasSoundFile = None
        self.loud_level = None
        self.soundDirection = None
        self.soundKinds = None

        #---- Nav ----
        self.goal_NavDock = None
        self.goal_ChargeRobot = 100
        self.goal_NavOoi = None
        self.currentPeople = "Default blank people"
        

        #---- Body ----
        self.mechanicMaxTmpr = None
        self.ID_hotest = None
        self.body_report = 'Nothing to report!'
        

        #---- Mind ----
        self.mindBlock = False
        self.deepMindBlock = False
        self.mutterGenBlock = False
        self.mindIdle = True
        """ "DialogMode"/"LearnCourseMode"/"SelfLearningMode"  """
        self.currentMindMode = "DialogMode"
        self.achievementDesire = self.bb.achievementDesire
        self.creativeDesire = self.bb.creativeDesire
        self.talkDesire = self.bb.talkDesire
        self.wanna    = [None]

        #---- Action ----
        self.actModeList    = ['FollowMode', 'PlayAloneMode', 'ChargeMode', 'IdleMode']
        self.currentActMode = 'IdleMode'
        self.doingList      = ['WATCHING', 'TALKING', 'MOVING']
        self.doing          = 'WATCHING'
        self.isFollowMode   = False
        self.isPlaying      = False
        self.talkGesture    = 'HELLO'

        #---- Dialog ----
        self.txtAimlMatch = 'This is Blank A-I-M-L match result'

        
        print ("+++++--> type of mouthAction, mouthGoal:", mouthAction, mouthGoal)
        print ("+++++--> actPatternAction, actPatternGoal:", actPatternAction, actPatternGoal )        
        self.zzzzz = actPatternGoal()
        print ("++++***++++--> self.zzzzz:", self.zzzzz)
        self.yyyyy = actPatternGoal()
        print ("++++***++++--> self.yyyyy:", self.yyyyy)
        self.zzzzz = actPatternGoal()
        print ("++++***++++--> self.zzzzz:", self.zzzzz)
        
        self.xxxxx = mouthGoal()
        print ("+++++--> self.xxxxx:", self.xxxxx)
        

        print("****>>>>>> cnt_ActMode_Idle is:", bt.cnt_ActMode_Idle, type(bt.cnt_ActMode_Idle))

        
               

                
        #---------------------------------------------------  The root node ---------------------------------------------------------
        self.BEHAVE = Sequence("BehaveRoot")                       #????        
        PARALLEL_ROOT = ParallelOne("Root_Parallel")
        
        #+++ L0 add  root
        self.BEHAVE.add_child(PARALLEL_ROOT)

        #---------------- L X branch
        # The top branch shoud not have any capital word which means a composite node
        

        #---------------- L5 branch
        CheckSTTLeaf  = MonitorTask("Leaf1_CHECK_STT", "tpc_sound", soundMsg, self.check_STT_cb)  
        MatchAimlLeaf = MatchAiml("Leaf_MATCH_AIML", self.txtAimlMatch )
        SpeakAimlOutLeaf  = SpeakAimlOut("Leaf_SpeakAimlOut", self.txtAimlMatch) 
        CheckLearnChanceLeaf = CheckLearnChance("Leaf_CHECK_LEARN_CHANCE" )
        LearnEnCourseLeaf = LearnEnCourse("Leaf_LEARN_EN_COURSE" )
        
        #----------------- L4 branch ...leafs
        DIALOG = Sequence("DIALOG",                  [  CheckSTTLeaf,
                                                        MatchAimlLeaf,
                                                        SpeakAimlOutLeaf      ] )
                
        LEARN_COURSE = Sequence("LEARN_COURSE",      [  CheckLearnChanceLeaf,
                                                        LearnEnCourseLeaf ] )
        
        NavDockLeaf = SimpleActionTask("Leaf3_NAV_DOCK_TASK", 'srv_Move', moveAction, self.goal_NavDock,
                                                feedback_cb=self.update_robot_position_cb, done_cb=self.navDock_cb)
        ChargeRobotLeaf = SimpleActionTask("Leaf3_CHARGE_ROBOT", 'srv_charge', chargeAction, self.goal_ChargeRobot,
                                                feedback_cb=self.update_battery_level_cb, done_cb = self.recharge_cb)
        CheckDialogInitLeaf = CheckBB_DialogInitLeaf("Leaf_CHECK_BB_DIALOG_INIT")
        CheckFollowModeLeaf = CheckBB_FollowModeLeaf("Leaf_CHECK_BB_FOLLOW_NODE", self.bb)
        CheckCourseStatusLeaf = CheckBB_CourseStatus("Leaf_CHECK_BB_COURSE_STATUS" )

        # ServiceTask( name, service,  service_type, request, result_cb=None, wait_for_service=True, timeout=5)
        ObjRecognizeLeaf = ServiceTask("Leaf2_OBJ_RECOGNIZE", "svc_visual", visualSvc, visualSvcRequest(cmd=self.visualSvcCMD, roi=self.visualSvcROI),
                                           result_cb = self.obj_recognize_cb, wait_for_service=True, timeout=8)   
                           
        RecordLeaf = Record("Leaf_RECORD" )

        #---------------  L3 branch ... leafs & composites
        #RECHARGE = Sequece("RECHARGE", [NAV_DOCK_TASK, CHARGING, CHARGE_COMPLETE], reset_after=True)        
        RECHARGE = Sequence("RECHARGE",              [ NavDockLeaf,
                                                       ChargeRobotLeaf   ] )
        
        CheckBattery = MonitorTask("Leaf1_Check_Battery", "tpc_sens", sensorsMsg, self.check_battery_cb)        
        

        TALK_MODE = Sequence( "TALK_MODE" ,         [  CheckDialogInitLeaf,
                                                       DIALOG            ] )
        
        LEARN_COURSE = Sequence ("LEARN_COURSE",    [  CheckFollowModeLeaf,
                                                       CheckCourseStatusLeaf,     # already learned? what progress is?
                                                       LEARN_COURSE      ] )
        
        OBJECT_RECOG = Sequence("OBJECT_RECOG",     [  ObjRecognizeLeaf,
                                                       RecordLeaf     ] )         # if nothing recognized?

        NAV_OOI_TASK = SimpleActionTask("Leaf1_NAV_OOI_TASK", 'srv_Move', moveAction, self.goal_NavOoi,
                                         feedback_cb=self.update_robot_position_cb, done_cb=self.navOoi_cb)
        
        CheckMechanicLeaf= MonitorTask("Leaf1_CHECK_MECHNIC", "tpc_sens", sensorsMsg, self.check_mechanic_cb)
        HaveRestLeaf = HaveRest("Leaf_HAVE_REST", self.bb, self.achievementDesire, self.creativeDesire,
                                                  self.talkDesire, bt.cnt_MindIdle, bt.cnt_ActMode_Idle )
        
        FaceDetectLeaf = MonitorTask("Leaf1_FACE_DETECT", "tpc_visual", visualMsg, self.faces_detect_cb)
        #FaceRecognizeLeaf = ServiceTask("CHARGE_ROBOT", result_cb = self.recharge_cb)
        FaceRecognizeLeaf = MonitorTask("Leaf1_FACE_RECOG", "tpc_visual", visualMsg, msg_cb = self.facesRecog_cb)
        ChatterRandomLeaf = ChatterRandom("Leaf_CHATTER_RANDOM", self.bb, self.cnt_DoingMutter)
        ReportRandomLeaf = ReportRandom("Leaf_REPORT_RANDOM", self.bb, bt.cnt_ActMode_Idle)
        CheckEnergyLeaf = CheckEnergy("Leaf_CHECK_ENERGY", self.bb)  
        ShoreArrowLeaf = ShoreArrow("Leaf_SHORE_ARROW")
        CheckCpLeaf = CheckCurrentPeople("Leaf_CHECK_CURRENT_PEOPLE",self.bb)
        FollowCpLeaf= FollowCp("Leaf_FOLLOW_CP", self.bb)
        SelectUnknownObjLeaf = SelectUnknownObj("Leaf_SELECT_UNKONWN_OBJ" )

        
        CHECK_NOT_ENERGY = Invert("CHECK_NOT_ENERGY", [CheckEnergyLeaf] )
                    
        #--------------------  L2 branch
        STAY_BATT_HEALTHY = Sequence("STAY_BATT_HLTH", [ CheckBattery,  #CHECK_BATTERY,
                                                         RECHARGE       ] )
        
        STAY_MCHN_HLTH = Sequence ("STAY_MCHN_HLTH",   [ CheckMechanicLeaf,
                                                         HaveRestLeaf   ] )
        
        FACE_RECOG  = Sequence ("FACE_RECOG",          [ FaceDetectLeaf,
                                                         FaceRecognizeLeaf ] )

        CHATTER_MODE = Selector("CHATTER_MODE",        [ ChatterRandomLeaf,
                                                         ReportRandomLeaf,
                                                         TALK_MODE,
                                                         LEARN_COURSE     ] )

        SELF_LEARN   = Sequence("SELF_LEARN",          [ CheckEnergyLeaf,
                                                         OBJECT_RECOG    ] )

        SHORE_ARROW = Selector("SHORE_ARROW",          [ ShoreArrowLeaf  ] )

        FOLLOW_MODE = Sequence("FOLLOW_MODE",          [ CheckCpLeaf,
                                                         FollowCpLeaf    ] )

        PLAY_ALONE = Sequence( "PLAY_ALONE",           [ CheckEnergyLeaf,
                                                       # RANDOM_SEE  ,
                                                         SelectUnknownObjLeaf,
                                                         NAV_OOI_TASK   ] )           #Object Of Interest

        HAVE_REST = Sequence ( "HAVE_REST",            [ CHECK_NOT_ENERGY,
                                                         HaveRestLeaf  ] )

        # Check the tpc to see the if there's motion 
        MotionDetectLeaf = MonitorTask("Leaf1_MotionDetect", "tpc_visual", visualMsg, self.motion_detect_cb)
        MothionDetcMutterLeaf = MothionDetcMutter("Leaf_MOTION_DETC_MUTTER")

        EnHeadTrackLeaf = EnableHeadTrack("Leaf_ENABLE_HEAD_TRACK" )    
      
        SoundEvtDetectLeaf = MonitorTask("Leaf1_CheckSoundEvt", "tpc_sound", soundMsg, self.check_soundEvt_cb)     
        SoundAnalyzeLeaf   = MonitorTask("Leaf1_CheckSoundAnalyze", "tpc_sound", soundMsg, self.check_soundDetailInfo_cb)



        #---------------------  L1 branch
        STAY_HEALTHY = Selector("STAY_HEALTHY",    [ STAY_BATT_HEALTHY,
                                                        STAY_MCHN_HLTH    ] )
        
        SUDDEN_SEE   = Sequence("SUDDEN_SEE",       [ MotionDetectLeaf,
                                                      EnHeadTrackLeaf,
                                                      MothionDetcMutterLeaf,
                                                      FACE_RECOG                                                                                                  
                                                      #ObjRecognizeLeaf( )
                                                      ] )
        
        SUDDEN_HEAR  = Sequence("SUDDEN_HEAR",     [ SoundEvtDetectLeaf,
                                                     SoundAnalyzeLeaf ,
                                                     EnHeadTrackLeaf ] )
        
        BRAIN_STORM  = Selector("BRAIN_STORM",     [ CHATTER_MODE,         # OK
                                                     SELF_LEARN,
                                                     SHORE_ARROW      ] )
        
        ROBOT_ACTIONS = Selector("ROBOT_ACTIONS",  [ HAVE_REST,
                                                     FOLLOW_MODE,
                                                     PLAY_ALONE      ] )               

        #+++  L1 add
        PARALLEL_ROOT.add_child( STAY_HEALTHY  )
        PARALLEL_ROOT.add_child( SUDDEN_SEE    )
        PARALLEL_ROOT.add_child( SUDDEN_HEAR   )
        PARALLEL_ROOT.add_child( BRAIN_STORM   )
        PARALLEL_ROOT.add_child( ROBOT_ACTIONS )

        #---- print -----      
        print ("Behavior tree structure: ----TBD")
        print_tree(self.BEHAVE)            #PARALLEL_ROOT
        #print_dot_tree(self.BEHAVE, dotfilepath )



    def run(self):
        while not rospy.is_shutdown( ):
            status = self.BEHAVE.run( )
            print ("XXXXXXXXXXXXXXXXXXXX BT run once. Enter in sleep.****************")
            self.tic.sleep()
            
            
            

    #-------------------------- call back ------------------------------
    """ (SimpleActionTask/ServiceTask/MonitorTask) """
    def check_battery_cb(self, msg):
        self.bb.battery_level = msg.battery
        if msg.battery is None:
            return TaskStatus.RUNNING
        elif msg.battery < self.low_battery_threshold:
            rospy.loginfo("LOW BATTERY - level:" + str(int(msg.battery ) ) ) 
            return TaskStatus.FAILURE
        else:
            rospy.loginfo("BATTERY level:" + str(int(msg.battery ) ) ) 
            return TaskStatus.SUCCESS

    def update_battery_level_cb(self):
        pass

    def check_mechanic_cb(self, msg):
        self.mechanicMaxTmpr = msg.mechanicMaxTmpr
        self.ID_hotest = msg.ID_hotest
        if self.mechanicMaxTmpr > self.mechanicMaxTmpr_threshold:
            self.body_report = 'mechanic temperature too hot %s. Will have a rest and work again.' %(str(self.mechanicMaxTmpr))


    def recharge_cb(self, result):
        rospy.loginfo("BATTERY CHARGED!")
        self.bb.battery_level = 100
        self.bb.charging = Fasle
        rospy.sleep(2)
        return True

    def motion_detect_cb(self, msg):
        rospy.logerr("In <motion_detect_cb> once. cnt_mutterGenBlock is %d" %self.cnt_mutterGenBlock)
        
        if msg.motion_detected and (self.bb.sens_blocked is False):
            print ('***********BT MSG:Motion Detected! **********')                
            self.cnt_mutterGenBlock += 1
            if self.cnt_mutterGenBlock >= self.Param_Max_MutterGenBlock:
                self.cnt_mutterGenBlock = 0                
                print ('========= will say somthing ===============>>>>> ...')
                mG = mutterGen()
                mG.run('WHAT', playit=True)
            return True
        else:
            return False        


    def faces_detect_cb(self, msg):
        self.rectFacesDetected = msg.rect_facesDetected
        #print ("Info: detects %d faces."%len(self.rectFacesDetected) )
        print ("Info: detects faces:", self.rectFacesDetected )
        return self.rectFacesDetected

    def facesRecog_cb(self, msg):
        facesRecog = msg.facesRecog
        print ("Info: Faces mboxWithTag are:", facesRecog.mboxWithTag)
        #print ("Info: Faces tracking are:", facesRecog.mboxWithTag.tag)
        return facesRecog

    def obj_recognize_cb(self, result_svc):
        print ("+++++++++Info: <obj_recognize_cb> result_svc is:", result_svc )
        
        

    def update_robot_position_cb(self, feedback):
        
        black_board.robot_postion = msg.base_position.pose.position
        pass

    def navDock_cb(self, result):
        pass

    def navOoi_cb(self, result):
        print ("Dummy: Enter in <navOoi_cb>."  )

    def check_STT_cb(self, msg):
        self.stt_result = msg.txt_stt
        self.hasSoundInEvt = msg.hasSoundInEvt
        self.hasSoundFile = msg.hasSoundFile
        self.loud_level = msg.loud_level
        #int8    soundDirection
        #string soundKinds
        print ("Get Pubed stt result: %s" %self.stt_result)
      
        

    def speak_out_cb(self, result):
        print ("Dummy: In <speak_out_cb>")

    def update_spk_progress_cb(self, feedback):
        print ("Dummy: In <update_spk_progress_cb>")
    
    def check_soundEvt_cb(self, msg):
        print ("Dummy: In <check_soundEvt_cb>")

    def check_soundDetailInfo_cb(self, msg):
        print ("Dummy: In <check_soundDetailInfo_cb>")

    def shutdown(self):
        rospy.loginfo("<shutdown() >....Stopping the robot...")
        #self.actprocess.cancel_all_goals( )
        #self.cmd_vel_pub.publish(Twist( ) )
        rospy.sleep( 1 )


#--------------------------- Func -------------------------------
    """common func.  other func changed to class because the pi_trees_lib need"""

"""def act_client_req(act_client, goal):
    #req_servoID,  req_goalAngle, req_goalTime = act_req          
    act_client.wait_for_server( )                      # wait until the server started.  listening for goals
    act_client.send_goal(goal)
    act_client.wait_for_result(rospy.Duration.from_sec(1.0) )

    print ("~~~~~~~~~act client req(sending goal) once ~~~~~~~~")
    res = self.act_client.get_result( )

    return res        
"""

#=================================Leaf=========================================

class CheckBB_FollowModeLeaf(object):
    def __init__(self, name, blackboard):
        self.name = name
        self.children = []
        print ("Dummy:Enter in %s.__init__ " %name)
        self.bb = blackboard

    def run(self):
        print ("DBG:Enter in (CheckBB_DialogInitLeaf.run)")
        self.isFollowMode = self.bb.isFollowMode        
        print ("<checkBB_FollowMode> -> self.isFollowMode:", self.isFollowMode)


            
class CheckBB_DialogInitLeaf(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in %s.__init__ " %name)

    def run(self):
        print ("Dummy: Enter in (CheckBB_DialogInitLeaf.run)")



class CheckBB_CourseStatus(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in %s.__init__ " %name)

    def run(self):
        print ("Dummy:Enter in (CheckBB_CourseStatus.run)")

 

class MothionDetcMutter(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        self.spkGoal = mouthGoal
        self.spkGoal.txt = "This is the default initialzed txt of %s" %name

    def run(self):    
        self.spkGoal.txt = "what's that?"
        print ("+++++++++++++ <MothionDetcMutter>. Will request say something...++++++++++++++++++++++")
        res = SimpleActionTask("MOTION_DETECT_MUTTER", "srv_mouth", mouthAction, self.spkGoal,
                                feedback_cb=self.update_spk_progress_cb, done_cb = self.speak_out_cb)
        res.run()
        print ("+++++++++++++>>++++++++++++++ <MothionDetcMutter>. actlib client request done+++++++++++++++++++++++++")

        return res
        
        
        
class CheckDialogInitLeaf(Task):
    def __init__(self, name):
        super(CheckDialogInitLeaf, self).__init__(name)

        self.text = None

    def run(self):
        print ("IMPL:Enter in (CheckDialogInitLeaf.run)")
        MonitorTask("CHECK_DIALOG_INIT", "tpc_sound", soundMsg, self.inquire_stt_cb )

    def inquire_stt_cb(self, msg):
        self.text = msg.txt_stt


class CheckEnergy(object):
    def __init__(self, name, blackboard):
        self.name = name
        self.children = []
        self.bb = blackboard
        
        self.energy = 10
        rospy.logdebug('Enter in (%s.__init__) "%d" ' %(name, self.energy) )

    def run(self):
    
        self.energy = self.bb.energy

        
        print("BT check energy:", self.energy)

        if  self.energy < 20:
            rospy.logwarn('Info: Enter in (CheckEnergy.run) and energy is %d' %(self.energy) )
            return False     # return energy false to have rest
        
        else:
            return True 


class  HaveRest(object):
    def __init__(self, name, blackboard,  achievementDesire, creativeDesire, talkDesire, cnt_MindIdle, cnt_ActMode_Idle ):
        self.name = name
        self.children = []
        print ("Info: Enter in (%s.__init__)" %name)
        self.bb = blackboard
        self.cnt_ActMode_Idle = cnt_ActMode_Idle
        self.achievementDesire = achievementDesire
        self.creativeDesire = creativeDesire
        self.talkDesire = talkDesire
        self.cnt_MindIdle = cnt_MindIdle
        self.RestBlock = False

        self.mG = mutterGen()
        
        self.goal_actPattern = actPatternGoal(op = actPatternGoal.PATTERN_OPERATION,
                                              pattern = actPatternGoal.TALK_GESTURE,
                                              talk_gesture = 'TG_KILLTIME')
        #self.actReq_HaveRest = SimpleActionTask("HAVE_REST_ACT", 'srv_actPattern', actPatternAction, self.goal_actPattern,
        #                                  done_cb = self.have_rest_actionRS_cb, feedback_cb=self.have_rest_actionFB_cb)


    def run(self):
        
        self.mG.run('HAVE_REST', playit=True)

        print ("self.cnt_ActMode_Idle is:", self.cnt_ActMode_Idle, type(self.cnt_ActMode_Idle))
        
        self.cnt_ActMode_Idle += 1
        self.achievementDesire -= 1
        self.creativeDesire -= 1
        self.talkDesire += 1
        self.cnt_MindIdle += 1

        self.RestBlock = self.bb.RestBlock

        
        rospy.logwarn('Info: Enter in (HaveRest.run).self.talkDesire is %d, self.RestBlock is %s ' %(self.talkDesire, str(self.RestBlock) ) )
        
        if self.RestBlock is False:
            actReq_HaveRest = SimpleActionTask("HAVE_REST_ACT", 'srv_actPattern', actPatternAction, self.goal_actPattern,
                                          done_cb = self.have_rest_actionRS_cb, feedback_cb=self.have_rest_actionFB_cb)
            #self.actReq_HaveRest.run()
            actReq_HaveRest.run()            
            print ("========>>>> self.goal_actPattern is:", self.goal_actPattern)
            self.mG.run('BORING', playit=True) 
        

    def have_rest_actionFB_cb(self, feedback):
        print ("Info: In (HaveRest).have_rest_actionFB_cb....feedback is:", feedback)

    def have_rest_actionRS_cb(self, result_state, result):
        print ("Info: In (HaveRest).have_rest_actionRS_cb....result_state & result is:", result_state, result)
        
        

        

class ChatterRandom(object):
    def __init__(self, name, blackboard, cnt_DoingMutter):
        self.name = name
        self.children = []
        print ("Info:Enter in (%s.__init__)" %name)
        self.bb = blackboard
        self.cnt_DoingMutter = cnt_DoingMutter
        self.strikeUpTalk = strikeUpTalkGen()

    def run(self):
        print ("Info:Enter in (ChatterRandom.run)")
        if self.bb.talkDesire < 80:
            print ("Info:(ChatterRandom.run)-> don't want to talk")
            return False
        
        else:                  
            print ('*********** Initiative conversation**********')
            print ('========= say somthing firstly==========>>>>> ...')
            self.strikeUpTalk.run('WHAT_DOING', playit=True)
            
            self.cnt_DoingMutter += 1
            return True

    

class ReportRandom(object):    
    
    def __init__(self, name, blackboard, cnt_ActMode_Idle):
        self.name = name
        self.children = []
        print ("Info:Enter in (%s.__init__)" %name )
        self.bb = blackboard
        self.mG = mutterGen()
        self.rG = reportGen()
        self.reportDict = {'totalHealth' :0,
                           'engergy' :0,
                           'talkDesire':0,
                           'playDesire':0,
                           'achievementDesire':0,
                           'creativeDesire':0,
                           'battery_level':0,
                           'robot_postion':0,
                           'currentActMode':0,
                           'current_people':'blank people',
                           'currentMindMode':'IdleMode'     }
         
        self.cnt_ActMode_Idle = cnt_ActMode_Idle
        
    #def update_
    
    def run(self):
        print ("Info: Enter in (ReportRandom.run), fetch data from bb" )
        self.reportDict['totalHealth'] = self.bb.totalHealth
        self.reportDict['engergy']     = self.bb.engergy
        self.reportDict['talkDesire']  = self.bb.talkDesire

        reportKey = random.choice(self.reportDict.keys())
        
        self.rG.run("Report: my %s now is: %s" %(reportKey, str(self.reportDict[reportKey]) )  )

        rospy.logwarn('Report: bb.currentActMode is %s. self.bb.playDesire is %d.' %(self.bb.currentActMode, self.bb.playDesire) )
        
        if  self.bb.currentActMode == "IdleMode" or self.bb.currentActMode == "ChargeMode":
            print ("UUUUUUUUU, self.cnt_ActMode_Idle is %d. self.bb.Threshold_ActMode_Idle is %d." %(self.cnt_ActMode_Idle, self.bb.Threshold_ActMode_Idle) )
            if self.bb.playDesire > 60 and self.cnt_ActMode_Idle > self.bb.Threshold_ActMode_Idle:
                self.mG.run('BORING', playit=True) 

        

class ShoreArrow(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in (%s.__init__)" %name)

        
    def run(self):
        print ("Dummy:Enter in (ShoreArrow.run)")
        


class EnableHeadTrack(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in (%s.__init__)" %name)

    def run(self):
        print ("Dummy:Enter in (EnableHeadTrack.run)")
    
    #if self.bb.allSens_blocked:
    #       self.MindDeepBlock = True
    



class CheckCurrentPeople(object):
    def __init__(self, name, blackboard):
        self.name = name
        self.children = []
        print ("Dbg:Enter in (%s.__init__)" %name)
        self.currentPeople = "Blank_CurrentPeople"
        self.bb = blackboard

    def run(self):
        print ("DBG:In (CheckCurrentPeople.run)")        

        self.currentPeople = self.bb.current_people


        if self.currentPeople == 'Default Blank People':
            print ("!!!!!! BT check current people is: Default Blank" )
            return False
        
        else:
            print ("BT check current people is:%s" %self.currentPeople)
            return True


class FollowCp(object):
    def __init__(self, name, blackboard):
        self.name = name
        self.children = []
        print ("Info:Enter in (%s.__init__)" %name)
        self.currentPeople = "Blank_CurrentPeople"
        self.bb = blackboard
        
    def run(self):
        print ("Info:Enter in (FollowCp.run)")
        self.currentPeople = self.bb.current_people

        if self.currentPeople == 'Default Blank People':
            print ("!!!!!! BT check current people is: Default Blank" )
            return False

        

class SelectUnknownObj(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in (%s.__init__)" %name)

        
    def run(self):
        print ("Dummy:Enter in (SelectUnknownObj.run)")

        

class Record (object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in (%s.__init__)" %name)
            
    def run(self):
        print ("Dummy:Enter in (Record.run)")


        
class rechargeRobot(object):
    def __init__(self, name, interval = 3, blackboard = None):
        super(rechargeRobot, self).__init__(name)
        #self.name = name
        #self.children = []

        self.name = name
        self.interval = interval
        self.bb = blackboard

        self.timer = 0

    def run(self):
        if self.timer == 0:
            rospy.loginfo("CHARGING the robot! ")

        if self.timer < self.interval:
            self.timer += 0.1
            rospy.sleep(0.1)
            self.bb.charging = True
            return TaskStatus.RUNNING
        else:
            return TaskStatus.SUCCESS

    def reset(self):
        self.status = None
        self.timer = 0
        

class MatchAiml(object):
    def __init__(self, name, txtAimlMatch):
        self.name = name
        self.children = []
        self.res_txt = txtAimlMatch
        print ("Info:Enter in (%s.__init__)" %name)
                
    def run(self):
        print ("Dummy:Enter in (MatchAiml.run)")
        self.res_txt = 'Blank x x x'



class SpeakAimlOut(object):
    def __init__(self, name, txt_in):
        self.name = name
        self.children = []
        
        self.spkActGoal = mouthGoal(txt_req = txt_in)    #, sentenceType = 1,  speed = 1,  mood = 1
        self.actReq_spkAimlOut = SimpleActionTask("Leaf3_SPEAK_AIML_OUT", 'srv_mouth', mouthAction, self.spkActGoal,
                                       feedback_cb=self.spk_aiml_outFB_cb, done_cb = self.spk_aiml_outRS_cb)
        

        print ("Info:Enter in (%s.__init__)" %name)
    

    def run(self):
        self.actReq_spkAimlOut.run()

        


    def spk_aiml_outFB_cb(feedback):
        print ("Info: In (SpeakAimlOut).spk_aiml_outFB_cb....feedback is:", feedback)


    def spk_aiml_outRS_cb(result):
        print ("Info: In (spk_aiml_outRS_cb).spk_aiml_outRS_cb....result is:", result)

        

class CheckLearnChance(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in (%s.__init__)" %name)
            
    def run(self):
        print ("Dummy:Enter in (CheckLearnChance.run)")
        


class CheckCourseStatus(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        print ("Dummy:Enter in (%s.__init__)" %name)
            
    def run(self):
        print ("Dummy:Enter in (CheckCourseStatus.run)")
        


class LearnEnCourse(object):
    def __init__(self, name):
        self.name = name
        self.children = []
        current_progress = "c2Ap38"
        self.progress = current_progress      #"c2Ap38"
        self.courseFile= course_en_fp + self.progress
        self.current_line = 0
        self.master_response = False

    def run(self):
        speakOut = spkOut("This is the default textof Learn course." )
        
        with open(self.courseFile) as f:     #file.next( )         file.readline( )
            for line in f:
                speakOut(line)
                if getResponse( ):
                    if evalResponse( ) > 70:
                        speakOut("Good.")
                        self.current_line += 1
                    else:
                        speakLine(line)
                else:       # timeout
                    speakLine(line)
                    self.current_line += 1               

    

if __name__ == '__main__':
    robot_bt = bt( )


