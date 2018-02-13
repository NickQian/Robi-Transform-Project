#!/usr/bin/env python

"""
 top file of video process (board). Issues tpc_visual messages & work as obj/face recognize service
 ----
  Licensed under BSD license.
  0.2: move 
  0.1: 2015.11.23 init version by Nick Qian
 ----
 Input: video, cmd(serviceProxy) from cortexn
 Output: tpc_visual info, object recognize service result

"""

import os, sys

import rospy
from std_msgs.msg import String

from facerec import createCsv, modelTrain, detectFaces, facesRecog
from thingrec import thingRec
#from cortexp import peopleAround, animalAround, plantAround, env
from visualprocess.msg import visualMsg
#from visualprocess.srv import ObjRecognizeSrv
from visualprocess.srv import visualSvc, visualSvcRequest, visualSvcResponse

from motionDetect import motionDetect
import facerec
import thingrec


class vp(object):
    def __init__(self):
        self.cam_cmos = camera_cmos()
        rospy.init_node('node_vp', anonymous=True)
        

    def launch(self):
        
        try:
            self.cam_cmos.launch()
            rospy.spin()
            
        except rospy.exceptions.ROSInterruptException, e:
            print ("!!!!! :" ,e)
      
        
        

class camera_tof():   # Initiative(such as TOF camera)
    pass
    
    

class camera_cmos(object):   # Passive(such as cmos camera)
    def __init__(self):
        self.cvFaceDataFolder = '/home/pi/Pictures/peopleInMem/'
        self.CsvFile = self.cvFaceDataFolder  + 'csv.txt'

        #self.objRecogSrvc = rospy.Service('srvc_objRecog', ObjRecognizeSrv, self.handle_objRecog)
        
         
        self.cmd_brain       = 'Run'                                                                # casual pause  stop
        self.objPositions    = [ ]
        self.objRecog        = [ ]                                     # [ (563, 'Dunge crab, king crab'),(106, 'rock crab, Alaska crab') ]
        self.peoplePositions = [ ]
        self.peopleRecog     = [ ]                                      #
        self.myView          = [self.objPositions, self.objRecog, self.peoplePositions, self.peopleRecog]                 # self.plant, self.animals
        #self.envir          = ''
        self.ooi             = True #(False, 0)                    # "is object of interest?"
        self.poi             = True #(False, 0)                    # "is people of interest?"
        
        createCsv( self.cvFaceDataFolder )
        modelPeople = modelTrain( self.cvFaceDataFolder )
        
        self.MotionDetect = motionDetect( )
        
    def launch(self):
        while not rospy.is_shutdown( ):
            try:
                self.MotionDetect.run()
                rospy.spin()
                
            except ROSInterruptException, e:
                print ("!!!!!!! :" ,e)



            
        """
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
        """
                          

    def pubObjPositions(self):
        self.vp2brain(self.objPositions)

    def handle_objRecog(req):
        if req.en_recognize == True:
            (req.roi)

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

        

def main():
    
    try:
        vprocess = vp( )
        vprocess.launch( )
        
    except rospy.ROSInterruptException, e:
        print ('!!!! ROSInterruptException', e)
    except KeyboardInterrupt, e:
        print ('Exiting by keyboard interrupt...')
    else:
        print '<vp> node is Done!'


 

if __name__ ==  '__main__':

    main()
    


    

