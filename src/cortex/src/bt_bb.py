#!/usr/bin/env python
# balck board of behavior tree. ---2016.8.13

bb_fp = "/home/pi/Documents/robotFiles/"
bb_fn = 'bt_bb.txt'
course_en_fp = "/home/pi/toby_ws/data/course/"

    
class BlackBoard( ):
    def __init__(self):
        self.F = bb_fp + bb_fn                 #, 'w')    #, 'w')

        self.name = "Toby"
        self.age = 3
        self.battery_level = None
        self.charging = None
        self.robot_postion = None                                  #Point()

        """ "DialogMode"/"LearnCourseMode"/"SelfLearningMode"  """
        self.current_psy_mode = None
        self.energy = None
        """ "FollowMode"/"PlayAloneMode"/"RestMode"/"ChargeMode" """
        self.current_act_mode = None

        self.totalHealth = 99
        self.characters = ['robust','energetic', 'gentle']
        self.personality = characters[1]

        self.sens_blocked = False                                  # when blocked, means ignore eyes/ear/other

        self.achievementDesire = None
        self.creativeDesire = None                                  # include destruction?
        self.playDesire = None                                          # 1: tedious
        

        #------------------------------------Memory---------------------------------------
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

        #-----------------
        self.peopleAround = []
        self.peopleInMem = []
        self.peopleVI           = []    # ['father', 'mother', 'brother', 'sister', 'grandpapa', 'grandmama']
        self.animalAround = []
        self.animalInMem = []
        self.plantAround    = []
        self.plantInMem    = []
        self.envlist              = ['home', 'school', 'company', 'field', 'mall']
        self.env                    = envlist[0]
        self.behaviorList   = ['sleeping', 'charging', 'idle', 'watching', 'talking', 'walking', 'playing', 'NULL' ] # status
        self.doing                = [None]
        self.wanna              = [None]
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

    def 
        

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
    
