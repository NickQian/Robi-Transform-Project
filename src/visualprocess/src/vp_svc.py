#!/usr/bin/env python

"""
 work with long time visual process service request. 1 Node, 1 services
 ----
 Licensed under BSD license. 
 0.1: 2016.11.5 init version by Nick Qian
 ----
 Input: video, cmd(serviceProxy) from cortexn
 Output: process result

"""

import os, sys

import rospy
from std_msgs.msg import String

from facerec import createCsv, modelTrain, detectFaces, facesRecog
from thingrec import thingRec
#from cortexp import peopleAround, animalAround, plantAround, env
from visualprocess.msg import visualMsg
from visualprocess.srv import visualSvc, visualSvcRequest, visualSvcResponse
#from visualprocess.srv import ObjRecognizeSrv

import motionDetect
import facerec
import thingrec


class vp_svc( ):
    def __init__(self):
        rospy.init_node('node_vp_svc')

        self.cmd_req = visualSvcRequest.FACE_RECOG
        self.roi_req = (0,0,100,100)               # default

        self.visal_svc = rospy.Service('svc_visual', visualSvc, handler=self.handle_visual_svc)
        print ("Info:ready to accept visual service request!")

        
        #self.cam_cmos = camera_cmos()
        

    def handle_visual_svc(self, req):
        self.cmd_req = req.cmd
        self.roi_req = req.roi
        
        if self.cmd_req == visualSvcRequest.FACE_RECOG:
            print ("++++++++++++++++++<handle_visual_svc> enter in [face_recog]+++++++++++++++++++++")
           
        elif self.cmd_req == visualSvcRequest.OBJ_DETECT:
            print ("+++++++++++++++ <handle_visual_svc> enter in [obj_detect]++++++++++++++++++++++")
            #objRecogNum = req.objRecogNum
            
        elif self.cmd_req == visualSvcRequest.OBJ_RECOG:
            print ("+++++++++++++++ <handle_visual_svc> enter in [thing_recog]+++++++++++++++++++++")
      
                    
        # face detect is performed after "motion detected"
    
        
def main():
    
    try:
        vpSvc = vp_svc(  )
        #vpSvc.launch(  )
    except rospy.ROSInterruptException, e:
        print ('!!!! ROSInterruptException', e)
    except KeyboardInterrupt, e:
        print ('Exiting by keyboard interrupt...')
    else:
        print '<vpSvc> service is Done! Enter in <spin>...'

    rospy.spin()
    



if __name__ ==  '__main__':

    main()

    
