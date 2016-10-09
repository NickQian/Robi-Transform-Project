#!/usr/bin/env python
# this is the top file of the behavior tree - 2016.8.5

import rospy
from std_msgs.msg import Float32, String
import time
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Twist


from bt_lib import *
from bt_ros_bridge import *
from bt_bb import BlackBoard, course_en_fp


class bt():
    def __init__(self):
        # Init parameters and vars
        setup_task_environment(self)

        # Init the BB
        self.bb = BlackBoard( )

        # rate to tick the tree 
        #rate = rospy.get_param('-rate', 10)
        self.rate = 10
        self.tic = rospy.Rate(rate)

        self.dotfp = rospy.get_param('-dotfp', None)
                
        #---------------------------------------------------  The root node ---------------------------------------------------------
        BEHAVE = Sequence("BehaveRoot")                       #????        
        PARALLEL_ROOT = ParallelOne("Root_Parallel")
        
        #+++ L0 add  root
        BEHAVE.add_child(PARALLEL_ROOT)

        #---------------- L X branch
        # The top branch shoud not have any capital word which means a composite node
        

        #---------------- L5 branch
        CheckTTSLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)    #TODO
        MatchAimlLeaf = MatchAiml( )
        SpeakOutLeaf = ServiceTask("CHARGE_ROBOT", result_cb = self.recharge_cb)   # TODO
        CheckLearnChanceLeaf = CheckLearnChance( )
        LearnEnCourseLeaf = LearnEnCourse( )
        
        #----------------- L4 branch ...leafs
        DIALOG = Sequence("DIALOG",                                     [  CheckTTSLeaf,
                                                                                                       MatchAimlLeaf,
                                                                                                       SpeakOutLeaf                   ] )
                
        LEARN_COURSE = Sequence("LEARN_COURSE",      [  CheckLearnChanceLeaf,
                                                                                                       LearnEnCourseLeaf          ] )
        
        NavDockLeaf = SimpleActionTask("NAV_DOCK_TASK", MoveAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
        ChargeRobotLeaf = ServiceTask("CHARGE_ROBOT", result_cb = self.recharge_cb)
        CheckDialogInitLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO
        CheckFollowModeLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO
        CheckCourseStatusLeaf = CheckCourseStatus( )
        ObjRecognizeLeaf( ) = SimpleActionTask("NAV_DOCK_TASK", MoveAction, goal, reset_after=True, feedback_cb=self.update_robot_position)  # TODO
        RecordLeaf = Record( )

        #---------------  L3 branch ... leafs & composites
        CheckBattery = MonitorTask("Check_Battery", "tpc_sens", Float32, self.check_battery_cb)
        
        RECHARGE = Sequece("RECHARGE",                      [  NavDockLeaf,
                                                                                                  ChargeRobotLeaf                  ] )          #RECHARGE = Sequece("RECHARGE", [NAV_DOCK_TASK, CHARGING, CHARGE_COMPLETE], reset_after=True)

        TALK_MODE = Sequence( "TALK_MODE" ,               [  CheckDialogInitLeaf( ),
                                                                                                   DIALOG                                  ] )
        
        LEARN_COURSE = Sequence ("LEARN_COURSE", [  CheckFollowModeLeaf(),
                                                                                                   CheckCourseStatusLeaf(),     #already learned? what progress is?
                                                                                                   LEARN_COURSE                  ] )
        
        OBJECT_RECOG = Sequence("OBJECT_RECOG",   [  ObjRecognizeLeaf( ),
                                                                                                   RecordLeaf( )                       ] )               # if nothing recognized?

        CheckMechanicLeaf= MonitorTask("CHECK_Mechnic", "tpc_sens", Float32, self.check_mechanic_cb)     # TODO
        HaveRestLeaf = HaveRest()
        FaceDetectLeaf = MonitorTask("FACE_DETECT", "tpc_visual", string, self.face_detect_cb)     # TODO
        FaceRecognizeLeaf = ServiceTask("CHARGE_ROBOT", result_cb = self.recharge_cb)   # TODO
        ChatterRandomLeaf = ChatterRandom()
        ReportRandomLeaf = ReportRandom()
        CheckEnergyLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO
        ShoreArrowLeaf = ShoreArrow()
        CheckCpLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO
        FollowCpLeaf= FollowCp( )
        CheckEnergyLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO
        SelectUnknownObjLeaf = SelectUnknownObj( )
        CheckEnergyLeaf =  MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO
                    
        #--------------------  L2 branch
        STAY_BATT_HEALTHY = Sequence("STAY_BATT_HLTH",  [  CHECK_BATTERY,
                                                                                                  RECHARGE                            ] )
        
        STAY_MCHN_HLTH=Sequence ("STAY_MCHN_HLTH", [ CheckMechnicLeaf( ),
                                                                                                  HaveRestLeaf( )                    ] )
        
        FACE_RECOG       = Sequence ("FACE_RECOG",        [  FaceDetectLeaf( ),
                                                                                                   FaceRecognizeLeaf( )         ] )

        CHATTER_MODE = Selector("CHATTER_MODE",    [ ChatterRandomLeaf( ),
                                                                                                   ReportRandomLeaf( ),
                                                                                                   TALK_MODE,
                                                                                                   LEARN_COURSE                  ] )

        SELF_LEARN        = Sequence("SELF_LEARN",         [  CheckEnergyLeaf( ),
                                                                                                    OBJECT_RECOG                 ] )

        SHORE_ARROW = Selector("SHORE_ARROW",        [  ShoreArrowLeaf( )              ] )

        FOLLOW_MODE = Sequence("FOLLOW_MODE",     [ CheckCpLeaf( ),
                                                                                                  FollowCpLeaf( )                    ] )

        PLAY_ALONE = Sequence( "PLAY_ALONE",              [  CheckEnergyLeaf( ),
                                                                                                    RANDOM_SEE( ),
                                                                                                    SelectUnknownObjLeaf( ),
                                                                                                   NAV_OOI_TASK                   ] )           #Object Of Interest

        HAVE_REST = Sequence ( "HAVe_REST",                  [  CheckEnergyLeaf( )
                                                                                                    HaveRestLeaf( )                ] )

        # Check the tpc to see the if there's motion 
        MotionDetectLeaf = MonitorTask("MotionDetect", "tpc_visual", bool, self.motion_detect_cb)

        # Enable Head Tracker 
        HeadTrackLeaf = ServiceTask("HeadTrack",  actPatternAction,  service_type, request,    result_cb = self.head_track_cb, wait_for_service=True, timeout=2)   # service, service_type, request,  
        
        SoundEvtDetectLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery_cb)     # TODO        
        SoundEvtDetectLeaf = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO
        SoundAnalyzeLeaf     = MonitorTask("CHECK_BATTERY", "battert_level", Float32, self.check_battery)     # TODO


        #---------------------  L1 branch
        STAY_HEALTHY    = Selector("STAY_HEALTHY",     [ STAY_BATT_HEALTHY,
                                                                                                  STAY_MCHN_HLTH            ] )
        
        SUDDEN_SEE        = Sequence("SUDDEN_SEE",       [ MotionDetectLeaf( ),
                                                                                                            HeadTrackLeaf(), 
                                                                                                            FACE_RECOG                                                                                                  
                                                                                                  #ObjRecognizeLeaf( )
                                                                                                                                                ] )
        
        SUDDEN_HEAR     = Sequence("SUDDEN_HEAR",   [ SoundEvtDetectLeaf( ),
                                                                                                  SoundAnalyzeLeaf( ) ,
                                                                                                  HeadTrackLeaf( " ")            ] )
        
        BRAIN_STORM     = Selector("BRAIN_STORM",      [ CHATTER_MODE,
                                                                                                  SELF_LEARN,
                                                                                                  SHORE_ARROW                 ] )
        
        ROBOT_ACTIONS = Selector("ROBOT_ACTIONS", [  FOLLOW_MODE,
                                                                                                  PLAY_ALONE,
                                                                                                  HAVE_REST                        ] )               

        #+++  L1 add
        PARALLEL_ROOT.add_child( STAY_HEALTHY  )
        PARALLEL_ROOT.add_child( SUDDEN_SEE      )
        PARALLEL_ROOT.add_child( SUDDEN_HEAR   )
        PARALLEL_ROOT.add_child( BRAIN_STORM    )
        PARALLEL_ROOT.add_child( ROBOT_ACTIONS )

        #----------------- Run the Tree ------------------------      
        print ("Behavior tree structure:")
        print_tree(BEHAVE)

    # Run the tree
    def run( ):
        # run the tree
        while not rospy.is_shutdown( ):
            status = BEHAVE.run( )
            print ("************* BT run once. Enter in sleep.****************")
            self.tic.sleep()
            
            #print_dot_tree(BEHAVE, dotfilepath )

    #-------------------------- call back ------------------------------
    """ (SimpleActionTask/ServiceTask/MoniitorTask) """
    def check_battery_cb(self, msg):
        self.bb.battery_level = msg.data
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < self.low_battery_threshold:
            rospy.loginfo("LOW BATTERY - level:" + str(int(msg.data ) ) )
            return TaskStatus.FAILURE
        else:
            return TaskStatus.SUCCESS

    def recharge_cb(self, result):
        rospy.loginfo("BATTERY CHARGED!")
        self.bb.battery_level = 100
        self.bb.charging = Fasle
        rospy.sleep(2)
        return True

    def motion_detect_cb(self, msg):
        
        if msg.motion_detected and (self.bb.sens_blocked is False):
            
            print ('***********BT MSG:Motion Detected! **********')
            if 
            
            return True
        else:
            return False


    def update_robot_position(self, msg):
        black_board.robot_postion = msg.base_position.pose.position

    def shutdown(self):
        rospy.loginfo("<shutdown() >....Stopping the robot...")
#        self.actprocess.cancel_all_goals( )
 #   self.cmd_vel_pub.publish(Twist( ) )
        rospy.sleep( 1 )

#=================================Leaf=========================================
class CheckDialogInitLeaf(Task):
    def __init__(self):
        name = "CHECK_DIALOG_INIT"
        super(CheckDialogInitLeaf, self).__init__(name)

        #Inst the MonitorTask which is a subscriber
        MonitorTask("CHECK_DIALOG_INIT", "tpc_sound", string)

class  HaveRest( ):
    pass

class ChatterRandom(Task):
    pass

class ReportRandom( ):
    pass

class ShoreArrow( ):
    pass

class FollowCp( ):
    pass

class SelectUnknownObj( ):
    pass

class Record ( ):
    pass

        
class rechargeRobot(Task):
    def __init__(self, name, interval = 3, blackboard = None):
        super(rechargeRobot, self).__init__(name)

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

class MatchAiml( ):
    pass

class CheckLearnChance( ):
    pass

class CheckCourseStatus( ):
    pass


class LearnEnCourse( ):
    def __init__(self, current_progress):
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


