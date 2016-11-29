#!/usr/bin/env python
"""
 # balck board of behavior tree. Memory of the robot
 ----
 Licensed under BSD license. 
 0.1: - 2016.8.13  init version by Nick Qian
 ----
 Input:  data write in by bt.py & others...
 Output: self.xxx
 
"""

import rospy

bb_fp = "/home/pi/Documents/robotFiles/"
bb_fn = 'bt_bb.txt'
course_en_fp = "/home/pi/toby_ws/data/course/"

class BlackBoard( ):
    def __init__(self):
        self.F = bb_fp + bb_fn                 #, 'w')    #, 'w')

        self.name = "Toby"
        self.age = 3

        #----- parameters -----
        self.Threshold_ActMode_Idle = rospy.get_param('~Threshold_ActMode_Idle', 10)  

        #----- Body -----
        self.totalHealth = 99
        self.battery_level = 1
        self.characters = ['robust','energetic', 'gentle']
        self.personality = self.characters[1]
        self.robot_postion = None                                  #Point()

        #---- Action -----
        self.currentActMode = 'IdleMode'
        self.isCharging = False
        self.isFollowMode = False
        self.current_people = 'Default Blank People'

        #----- Sens -----
        self.sens_blocked = False                                  # when blocked, means ignore eyes/ear/other
        self.allSens_blocked = False       # include visual and ears

        #----- Energy, Desire -----
        #----- Mind -----
        self.MindBlock = False
        self.DeepMindBlock = False
        self.RestBlock = False
        self.mindIdle = True
        """ "DialogMode"/"LearnCourseMode"/"SelfLearningMode"/"IdleMode"  """
        self.currentMindMode = "DialogMode"
        self.achievementDesire = 1
        self.creativeDesire = 1                                  # include destruction?
        self.playDesire = 90                                          # 1: tedious
        self.talkDesire = 1
        self.engergy = 1


        
        #---- around in mind ----
        self.peopleAround  = []
        self.peopleInMem   = []
        self.peopleVI      = []    # ['father', 'mother', 'brother', 'sister', 'grandpapa', 'grandmama']
        self.animalAround  = []
        self.animalInMem   = []
        self.plantAround   = []
        self.plantInMem    = []
        self.envlist       = ['home', 'school', 'company', 'field', 'mall']
        self.env           = self.envlist[0]



        #-------------Memory----------------------------
        self.mem_people = {'master':None,
                                            'father': None,
                                            'mother': None,
                                            'pet': None,
                                            'gfather': None,
                                            'gmother': None,
                                            'buddy1': None,
                                            'buddy2': None
                                           }
        self.current_people = [ ]
        self.environList = ['home', 'house_1', 'car', 'field' ]
        self.current_environ = None

        
    '''self.hom = [name, age, disposition,
       peopleAround, peopleInMem, peopleVI,       env, envInMem, envVI,
       animalAround, animalInMem,
       plantAround, plantInMem]
    '''
        

    def reset(self):
        pass

    def add_person(self, name, age, gender, face):
        pass

    def add_environ(self, name, environ_mem):
        pass

       

    # write BB data to file (SD card/flash)
    def writeFlash(self):
        with open(self.F) as f:
            for line in f:
                print ("this is he file content:", line)



class person():
    def __init__(self, name, age, face):
        self.name = name
        self.age = age

        

if __name__ == "__main__":
    bb=BlackBoard( )
    bb.writeFlash( )
    
