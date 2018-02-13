#!/usr/bin/env python
'''
 motion detect using the opencv2 lk. bases on the code in 'rbx1'
 -----
 Licensed under BSD license. 
 0.1 : 2016-08-27 -init by Nick Qian 
 ----
 Input: video
 output: track_box, bool motionDetected
'''

import rospy
import time
import traceback
import cv2
import cv2.cv as cv
from picamera import PiCamera
from picamera.array import PiRGBArray                    #python-picamera python3-picamera
import numpy as np
from std_msgs.msg import String
#from sensor_msgs.msg import Image, RegionOfInterest #, CameraInfo
from visualprocess.msg import visualMsg #, box             # <class 'visualprocess.msg._visualMsg.visualMsg'>
from cv_bridge import CvBridge, CvBridgeError


DEBUG_L1 = True                    # class 
DEBUG_L2 = True                    # Every Frame, box info
DEBUG_L3 = True                    # paragraph
DEBUG_L4 = False
DEBUG_L5 = False 
DBG_WINDOW_CAMIN         = False
DBG_WINDOW_DETECT_BOX    = False
DBG_WINDOW_GET_KEYPOINTS = True
DBG_WINDOW_CALC_OPT_NEW_KEYPOINT = True
DBG_WINDOW_ELLIPSE_TRACK = False
DBG_WINDOW_TRACK_KEYPOINT= False



class motionDetect( ):
    def __init__(self):
        self.motionDetected     = False
        self.motionDetectedHold = False

        self.robot_view_motion      = rospy.get_param('~robot_view_motion',      False)        
        self.Vp_MD_Subtr_UseFrmDiff = rospy.get_param('~Vp_MD_Subtr_UseFrmDiff', False)
        self.Vp_MD_Subtr_UseMog     = rospy.get_param('~Vp_MD_UseSubtractorMog', True )
        self.Vp_ME_UseLkTractor     = rospy.get_param('~Vp_ME_UseLkTractor',     False)
       
        self.md_visualMsg = visualMsg( )
        self.md_pub = rospy.Publisher("tpc_visual", visualMsg, queue_size = 5)   #name.data_class,subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None
        self.pub_count = 0

        self.lk_tracker = LKTracker( )

        self.cam = PiCamera()
        self.cam.resolution = (640, 480)
        self.cam.framerate = 24
        self.rawCapture = PiRGBArray(self.cam)
        time.sleep(0.1)
        #self.cap = cv2.VideoCapture(0)
        #self.r = rospy.Rate(24)
 
        self.detect_box = (10,10, 620,460) #None   (x,y,w,h)
        self.track_box =  None   #rotated Rect (center, size, angle) | regular cvRect (x,y,w,h)
        self.keypoints = []
        self.frame_idx = 0
        self.detect_interval = 8
        self.mask = None   
        
        self.feature_size = rospy.get_param("~feature_size", 2)
        self.prev_grey = None
        self.grey = None        
        self.avg = None
        
        self.bgsMOG = cv2.BackgroundSubtractorMOG(1, 6, 0.9, .3)     #([history, nmixtures, backgroundRatio[, moiseSigma]])
        """
           history - length of the history
           nmixtures - number of Gaussian mixtures
        """
        # Subtractor parameters
        self.delta_thresh = rospy.get_param('~subtr_delta_thresh', 20)
        self.min_area = rospy.get_param('~subtr_min_area', 3000)       # ignore too small contour
        

        #good feature params
        self.gf_maxCorners   = rospy.get_param('~gf_maxCorners', 200)           # 500 | 200
        self.gf_qualityLevel = rospy.get_param("~gf_qualityLevel", 0.02)        # 0.3 | 0.02
        self.gf_minDistance  = rospy.get_param("~gf_minDistance", 7)            # 10  |  7 
        self.gf_blockSize    = rospy.get_param("~gf_blockSize", 10)             # 7   | 10        
        self.gf_useHarrisDetector = rospy.get_param("~gf_useHarrisDetector", True)
        self.gf_k            = rospy.get_param("~gf_k", 0.04)
        self.gf_params       = dict(maxCorners = self.gf_maxCorners,
                                    qualityLevel = self.gf_qualityLevel,
                                    minDistance = self.gf_minDistance,
                                    blockSize = self.gf_blockSize,
                                    useHarrisDetector = self.gf_useHarrisDetector,
                                    k = self.gf_k)
        
        if DEBUG_L1: print ('Info:motionDetect intialized.')

    def run(self):
        '''
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            self.r.sleep()
        '''
        for frame in self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):

            #--------- img pre-process----------------
            cv_image = frame.array
            self.grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.grey = cv2.equalizeHist(self.grey)
            self.grey = cv2.GaussianBlur(self.grey, (7, 7), 0)
            self.rawCapture.truncate(0)          # ????

            # for dispaly purpose
            vis = self.grey.copy()  

            #---------  Subtractor - Track every frame motion ------------
            if self.robot_view_motion is False:
                
                if self.Vp_MD_Subtr_UseMog:        
                    cnts = self.subtrMog(self.grey)                
                elif Vp_MD_Subtr_UseFrmDiff:
                    cnts = FrmDiff(self.prev_grey, self.grey) 
                else:  # use avg
                    cnts = avgDiff(self.grey)
                        

                # got diff. loop over the contours
                if len(cnts) > 0:                    
                 
                    c_max = cnts[0]
                    for c in cnts:  
                        # discard little change
                        if cv2.contourArea(c) < self.min_area:
                            continue
                        if cv2.contourArea(c_max) < cv2.contourArea(c):
                            c_max = c
                        if DBG_WINDOW_DETECT_BOX:
                            (x_, y_, w_, h_) = cv2.boundingRect(c)
                            cv2.rectangle(vis, (x_, y_), (x_+w_, y_+h_), (255,0,0), 2)
                            

                    if cv2.contourArea(c_max) > self.min_area:        # avoid many too small c in frame
                        (x, y, w, h) = cv2.boundingRect(c_max)
                        cv2.rectangle(vis, (x, y), (x+w, y+h), (0,255,0), 4)
                        if DBG_WINDOW_DETECT_BOX:
                            cv2.imshow("[1]subtractor_boundingRect", vis)
                            key = cv2.waitKey(1)   
                        self.detect_box = (x, y, w, h)
                        self.motionDetected = True
                        self.motionDetectedHold = True
                    
                        print ("~~~ Motion detected. motionDetected = True")
                        
                    else:
                        print ("////// No motion detected. motionDetected = Fasle")                


                    if self.Vp_ME_UseLkTractor:   # if do Motion Estimation, get key points firstly
                        self.keypoints = self.get_keypoints(self.grey, self.detect_box)
                        print ("@@@@@@ self.frame_idx, pre get keypoints:", self.frame_idx, self.keypoints)

                    # if have points, display them
                    if self.keypoints is not None and len(self.keypoints) > 0:
                        for x, y in self.keypoints:
                            cv2.circle(vis, (x, y), self.feature_size, (255, 0, 0 ), cv.CV_FILLED, 8, 0)

                        if DBG_WINDOW_GET_KEYPOINTS:
                            cv2.imshow("[1]get_keypoints", vis)
                            key = cv2.waitKey(10)

                    # visualMsg.motion_detected, 
                    
                    
                    #cv2.putText()                    
                
                else:                            # len(cnts) <= 0
                    self.motionDetected = False
                    #self.detect_box = None
                    print ("//// No motion detected. motionDetected = Fasle")
                    
            else:
                rospy.logwarning("Err:motion detect when robot view itself has motion. Not Implemented!")

            # -------- if find motion, track it ----------
            # LK Tracker
            if self.Vp_ME_UseLkTractor:
                print ("///// self.keypoints before lk track is:", self.keypoints)
                if self.keypoints is not None and len(self.keypoints) > 0:
                
                    self.md_visualMsg.roi = self.lk_tracker.run(self.prev_grey, self.grey, self.keypoints)                

            #----- if interval , pub info -----
            if self.frame_idx % self.detect_interval == 0:
                
                self.md_visualMsg.motion_detected = self.motionDetectedHold
                print ("$$$$$ md_visualMsg.motion_detected is:", self.md_visualMsg.motion_detected)
                (self.md_visualMsg.roi.x,self.md_visualMsg.roi.y,self.md_visualMsg.roi.height,self.md_visualMsg.roi.width) = self.detect_box
                        
                self.publish_motion( )
                
                #clear motion detected hold flag after pub
                self.motionDetectedHold = False

                if DEBUG_L2: print ('~loop A interval. motion info Pubed. frame_idx is %d' %self.frame_idx )
                
                
                

            self.frame_idx += 1
            self.prev_gray = self.grey
                             
            #clear the stream in preparation for next frame
            self.rawCapture.truncate(0)

            #if DEBUG_L2: print ('~loop once. frame_idx is %d' %self.frame_idx )
            
        
            key = cv2.waitKey(10) & 0xFF
            if key == 27: # esc
                cv2.destroyAllWindows()
                break

        # end for frames



    def subtrMog(self, grey):
        grey_bg_mog = self.bgsMOG.apply(grey, learningRate=0.001)       #, learningRate=0.001
        """self.grey_bg_mog = cv2.morphologyEx(self.grey_bg_mog,
                                               cv2.MORPH_OPEN,
                                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)) ) """
        if DBG_WINDOW_DETECT_BOX:
            cv2.imshow("bg_mog", grey_bg_mog)
            key = cv2.waitKey(1)
                
        (cnts, _) = cv2.findContours(grey_bg_mog, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return cnts   


    def FrmDiff(self, prev_grey, grey):
        if prev_grey is None:
            prev_grey = grey
            
        frameDelta = cv2.absdiff(grey, prev_grey )

        """
        cv2.threshold(src, thresh, maxval, type[, dst]) -> retval, dst
        dst - output array of the same size and type as src

        cv2.dilate(src, kernel [, dst[,anchor[,iterations[,borderType[,borderValue]]]]]) -> dst
        dst - same size & type as src
        anchor - position of the anchor within the element. default (-1, -1)
        iterations - number of times dilation is applied 
        """
        thresh = cv2.threshold(frameDelta,  self.delta_thresh, 255, cv2.THRESH_BINARY)[1]
        cv2.imshow("[1.]threshold", thresh)
        key = cv2.waitKey(10)
                
        thresh = cv2.dilate(thresh, None, iterations = 1)
        (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return cnts


    def avgDiff(self, grey):
        if self.avg is None:
            self.avg = grey.copy().astype("float")

        cv2.accumulateWeighted(grey, self.avg, 0.5)
        frameDelta = cv2.absdiff(grey, cv2.convertScaleAbs(self.avg) )
        """
            cv2.threshold(src, thresh, maxval, type[, dst]) -> retval, dst
            dst - output array of the same size and type as src

            cv2.dilate(src, kernel [, dst[,anchor[,iterations[,borderType[,borderValue]]]]]) -> dst
            dst - same size & type as src
            anchor - position of the anchor within the element. default (-1, -1)
            iterations - number of times dilation is applied 
        """
        thresh = cv2.threshold(frameDelta,   self.delta_thresh, 255, cv2.THRESH_BINARY)[1]
        cv2.imshow("[1.]threshold", thresh)
        key = cv2.waitKey(10)
                
        thresh = cv2.dilate(thresh,   None, iterations = 1)
        (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        
        cv2.imshow("[2.]dilate", thresh)
        key = cv2.waitKey(10)

        return cnts
        

    def get_keypoints(self, input_image, detect_box):
        #init mask with all black pixels
        self.mask = np.zeros_like(input_image)

        try:
            x,y,w,h = detect_box
        except:
            return None

        self.mask[y:y+h, x:x+w] = 255  # set the selected rectangle within the mask to white
       
        # why circle mask?
        #for x, y in [np.int32(tr[-1]) for tr in self.keypoints ]:
        #    cv2.circle(self.mask, (x, y), 5, 0, -1)
        self.keypoints = []

        """
         image, maxCorners, qualityLevel, minDistance [,corners [, mask[, blockSize [, useHarrisDetector [, k ]]]]] -> corners
         eigImage, tempImage are ignored now
         maxCorners:   maxium number of corners to return
         qualityLevel: min accepted quality of image corners
         minDistance:  min possible distance between the returned corners
         blockSize:    Size of an average block for computing a derivative covariation matrix
        """
        kp = cv2.goodFeaturesToTrack(input_image, mask=self.mask, **self.gf_params)      
                                     
        
        if kp is not None and len(kp) > 0:
            for x, y in np.float32(kp).reshape(-1, 2):
                self.keypoints.append((x, y))

        return self.keypoints

            

    def publish_motion(self ):        
        try:
            self.md_pub.publish( self.md_visualMsg )
            self.pub_count +=1
            print ("~~~~~DBG:: Pub once!~~~~~, msg to pub is:, pub_count is:", self.md_visualMsg, self.pub_count)
            
        except:
            traceback.print_exc()
            rospy.logerr("Error: Publishing motion detect info failed in <%s>" %'motionDetect')
            

    def stop(self):
        self.cap.release()
        cv2.destroyAllWindows
        
    

class LKTracker():
    def __init__(self):
        #super(LKTracker, self).__init__(node_name)

        self.show_text = rospy.get_param("~show_text", True)
        self.show_features = rospy.get_param("~show_features", True)
        self.show_boxes = rospy.get_param("~show_boxes", True)
        self.feature_size = rospy.get_param("~feature_size", 2)

        #lk params
        self.lk_winSize = rospy.get_param("~lk_winSize", (15, 15) )
        self.lk_maxLevel = rospy.get_param("~lk_maxLevel", 2)
        self.lk_criteria = rospy.get_param("~lk_criteria", (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01) )  #10, 0.03 | 20, 0.01  

        self.lk_params = dict(winSize = self.lk_winSize,
                              maxLevel = self.lk_maxLevel,
                              criteria = self.lk_criteria )

        self.track_len = 10
        
        self.detect_interval = 5
        self.keypoints = None
        self.marker_image = None
        self.display_image = None

        self.track_box = None
        self.keypoints = None
        self.mask = None
        self.grey = None
        self.prev_grey = None

        self.something_move = None
        

        if DEBUG_L1: print ('Info:LKTracker intialized.')        
        

    def run(self, prev_grey, grey, keypoints):    # "tracks" come from goodfeature

        self.prev_grey = prev_grey
        self.grey = grey
        self.keypoints = keypoints
        vis = grey.copy() 

        #if have not start tracking
        '''if self.track_box is None or not self.is_rect_nonzero(self.track_box):
            self.track_box = self.detect_box
            
            if DBG_WINDOW_DETECT_BOX:
                x,y,w,h = self.detect_box
                cv2.rectangle(vis, (x,y),(x+w, y+h), (0,255,0), 2)
                cv2.imshow("detect_box", vis)
                key = cv2.waitKey(10) '''

        
        if DEBUG_L2:  rospy.loginfo("start track_keypoints ....")
        
        # Now have keypoints, Track Keypoints
        self.track_box = self.track_keypoints(self.grey, self.prev_grey, keypoints )

        if self.track_box is not None:  
                
            draw_rect(vis, self.track_box, (0, 255, 0) )
                
            if DBG_WINDOW_TRACK_KEYPOINT:
                cv2.imshow("[4]tracked_rect", vis)
                key = cv2.waitKey(10)       

        return self.track_box
    

    def track_keypoints(self, grey, prev_grey, keypoints):
        img0, img1 = prev_grey, grey
        self.keypoints = keypoints

        vis = grey.copy()                    
        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(grey)

        # reshape keypoints into numpy array
        p0 = np.float32([p for p in self.keypoints]).reshape(-1, 1, 2)    #p[-1]

        """
        use cv2 for calculate the optical flow
        output: nextPts, status, err
        input: prevImg, nextImg, prevPts,[, nextPts[,status[,err[,winSize[,maxLevel[,criteria[,flags[, minEigThreshold]]]]]]]
        PrevImg  - 1st input image
        nextImg  - 2nd input img
        PrevPts  - points need to be found
        winSize  - size of the search window
        maxLevel - 0-based maximal pyramid
        
        nextPts  - output vector. calculated new position of the input feature in the second img
        status:  - output vector. 1 if the corresponding feature has been found. otherwise 0. 
        """
        p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)

        if DEBUG_L5:  print ("Info: <calcOpticalFlowPyrLK> p1, st are:", p1, st)

        if DEBUG_L3: print ('(1)')

        try:            
            #
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            if DEBUG_L5: print ("Info: <calcOpticalFlowPyrLK> p0r, st are:", p0r, st)

            d = abs(p0-p0r).reshape(-1, 2).max(-1)
                                  
            # good if distance between pairs of points < 1 pixel
            good = d < 1

            new_keypoints = []
            
            # only keep all keypoints satisfy 'good' condition
            for (x, y), good_flag  in zip(p1.reshape(-1, 2), good):   #(array([ 582.99908447,   71.28678894], dtype=float32), True)           
                if not good_flag:
                    continue
                if len(new_keypoints) > self.track_len:
                    del new_keypoints[0]
  
                new_keypoints.append( (x, y) )                

                # draw keypoint on img   (img, center, radius, color, thickness=1, lineType=8, shift=0) -> None   
                cv2.circle(vis, (x, y), self.feature_size, (255, 0, 0), cv.CV_FILLED, 8, 0)       # self.marker_image

                if DBG_WINDOW_CALC_OPT_NEW_KEYPOINT:
                    cv2.imshow("[2]_calc_opt_new_keypoint", vis)
                    key = cv2.waitKey(10)             
                    
            self.keypoints = new_keypoints
            print ("self.keypoints= new_keypoints:", self.keypoints)
            
            if DEBUG_L4:  print ("keypints is:", self.keypoints )


            # Convert to numpy array
            if len(self.keypoints) > 0:
                keypoints_array = np.float32([p for p in self.keypoints]).reshape(-1, 1, 2)            

            # if have enough points
            if len(self.keypoints) > 6:
                
                track_box = cv2.fitEllipse(keypoints_array)  # output: (x,y),(MAJor axis, MIN axis), angle
                if DEBUG_L3: print ('(2)')
 
                
            # otherwise, find the best fitting rectangle
            else:
                track_box = cv2.boundingRect(keypoints_array)       #  output: (x,y, w, h)
                self.something_move = False
                if DEBUG_L3: print ('(3)')
 
            if DEBUG_L1: print ("||| fucking Done! got track_box:")
            if DBG_WINDOW_ELLIPSE_TRACK:
                cv2.ellipse(vis, track_box, cv.CV_RGB(255,0, 0), cv.CV_FILLED)   #img, box, color[, thickness[, lineType]]
                cv2.imshow("[3]_cv2_ellipse_keypoint", vis)
                key = cv2.waitKey(1)
            
            
        except:
            track_box = None
            traceback.print_exc()
            print ("~~~ except! track_box is None")

        return track_box


#----------------------------------------------------------------------------------------------------------
def is_rect_nonzero(rect):
    try:
        (_,_,w,h) = rect
        return (w>0) and (h>0)
    except:
         try:
            ((_,_,),(w,h),a) = rect
            return (w>0) and (h>0)
         except:
            return False


def cvBox2D_to_cvRect(box):
    
    if DEBUG_L2: print ('$$ box to rect transfer->',box)
    
    try:
        if len(box) is 3:
            (center, size, angle) = box            #  ((59, 412), (178, 689), 83))
            pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
            pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
            rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
        elif len(box) is 4:
            rect = list(box)
    except:
        return [0,0,0,0]
 
    return rect

def cvRect_to_cvBox2D(roi):
    try:
        if len(roi) == 3:
            box2d = roi
        else:
            (p1_x, p1_y, width, height) = roi
            center = (int(p1_x + width / 2), int(p1_y + height / 2))
            size = (width, height)
            angle = 0
            box2d = (center, size, angle)
    except:
        return None
            
    return list(box2d)
            

def draw_rect(img, box, color):    
    x, y, w, h = cvBox2D_to_cvRect(box)
    
    if DEBUG_L2: print ('$$$ rect is->', x, y, w, h)
    
    cv2.rectangle(img, (x, y), (x+w, y+h ), color, 2)



def main():
    try:
        rospy.init_node('node_motion_detect', anonymous=True)
        md = motionDetect( )
        md.run()
        rospy.spin()
    except ROSInterruptException, e:    #KeyboardInterrupt:
        print "Shutting down motion detect node."
        cv.destroyAllWindows()


    
if __name__ == '__main__':
    main()
    '''with PiCamera() as camera:
        camera.start_preview()
        time.sleep(0)
        camera.capture('/home/pi/Desktop/image.jpg')
        camera.stop_preview()
       ''' 
        
        
