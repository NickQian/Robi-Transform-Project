#!/usr/bin/env python
# top file of video process (board) -2015.11.23
#  
import os, sys

import rospy
from std_msgs.msg import String

from facerec import createCsv, modelTrain, detectFaces, facesRecog
from thingrec import thingRec
#from cortexp import peopleAround, animalAround, plantAround, env

class camera:
    def __init__(self):
        self.cvFaceDataFolder = '/home/pi/Pictures/peopleInMem/'
        self.CsvFile = self.cvFaceDataFolder  + 'csv.txt'
        rospy.init_node('node_vp', anonymous=True)                          # rospy will choose a unique name for node_vp
        self.cmd_brain            = 'Run'                                                                # casual pause  stop
        self.objPositions        = [ ]
        self.objRecog             = [ ]                                     # [ (563, 'Dunge crab, king crab'),(106, 'rock crab, Alaska crab') ]
        self.peoplePositions = [ ]
        self.peopleRecog      = [ ]                                      #
        self.myView               = [self.objPositions, self.objRecog, self.peoplePositions, self.peopleRecog]                 # self.plant, self.animals
        #self.envir                  = ''
        self.ooi                        = True #(False, 0)                    # "is object of interest?"
        self.poi                        = True #(False, 0)                    # "is people of interest?"                
        createCsv( self.cvFaceDataFolder )
        modelPeople = modelTrain( self.cvFaceDataFolder )
        
    def launch(self):
        while True:
            if self.cmd_brain == 'stop':
                print ('Info: Got command from brain:', self.cmd_brain)
                break
            else:
                os.system('raspistill -t 10 -w 800 -n -h 600 -v --thumb none --exposure night --awb incandescent -o /home/pi/Pictures/x.JPG')                            #-v: verbose   --awb fluorescent
                #rcvOoi( )
                if self.ooi is False:
                    self.objPositions = objDetect( )
                    self.pubObjPositions( )
                    self.ooi = rcvOoi()
                    print ('Info: self.ooi is:', self.ooi)
                else:
                    self.objRecog = thingRec( '/home/pi/toby_ws/data/pangxie_7.jpg'  )    # '/home/pi/Pictures/x.JPG'
                    self.pubObjRecogInfo( )

                #rcvPoi( )
                if self.poi is False:
                    imagePIL, faces_rects = detectFaces('/home/pi/Pictures/x.JPG')
                    self.peoplePositions = faces_rects
                    self.pubPeoplePositions( )
                else:
                    self.peopleRecog = facesRecog(imagePIL, faces_rects)
                    self.pubPeopleRecogInfo( )       
                          

    def pubObjPositions(self):
        self.vp2brain(self.objPositions)

    def pubObjRecogInfo(self):
        self.vp2brain(self.objRecog)
        
    def pubPeoplePositions(self):
        self.vp2brain(self.peoplePositions)

    def pubPeopleRecogInfo(self):
        self.vp2brain(self.peopleRecog)

    def rcvPoi(self):
        self.brain2vp( )
    
    def rcvOoi(self, cameraIn):
        self.brain2vp( )

    def vp2brain(self, data_vp):                                                                                      # publisher
        pub = rospy.Publisher('tpc_visual', String, queue_size=10)             # pub to topic 'chatter'
        rate = rospy.Rate(2)                                                                                   # 2Hz
        while not rospy.is_shutdown( ):
            dataVisual = '%s %s' %(data_vp, rospy.get_time() )
            rospy.loginfo(dataVisual) 
            pub.publish(dataVisual)                                                                      # pub
            rate.sleep( )

    def brain2vp(self):                                                                                                                    # listener
        rospy.Subscriber('tpc_visual', String, self.callback, ('anyNickString', 4) )      # subscribe with data 
        rospy.spin( )                                                                                                                           # keeps python from exiting until the node is stopped

    def callback(self, data, args):
        rospy.loginfo('++Info++:' + rospy.get_caller_id() + ': I heard %s',  data.data, '. data is:', data)
        self.ooi = data.data      #[0]       [True, 'num_1']
        self.poi = data.data      #[1]       [False, 'do_not_care']
        #self.cmd_brain =  

    def cameraOperate(self):
        pass

    def detectPeople(self, cameraIn):
        if seeNewPeople == True:
            personsAll.append(face )
        else:
            return nameOfPerson



if __name__ ==  '__main__':
    eyeL = camera(  )
    eyeR = camera( )
    try:
        eyeL.launch(  )
        #eyeL.vp2node( )
    except rospy.ROSInterruptException:
        print ('Error: ROSInterruptException')
    except KeyboardInterrupt, e:
        print ('Exiting by keyboard interrupt...')
    print 'Exit Done.'
    


    

