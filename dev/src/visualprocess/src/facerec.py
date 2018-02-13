#!/usr/bin/env python

"""
 face recognize bases on opencv2.4.9.
 ----
 Licensed under BSD license. See the LICENSE file in the root.
 0.1:  2016-05-07. init by by Nick Qian
 ----
 input: cam shots
 output: string(name of people)
"""

import os, sys, math
import cv2
import cv2.cv as cv
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from glob import glob    #?
import itertools as it
#from video import creat_capture
#from common import clock, draw_str
#from optparse import OptionParser
import matplotlib.pyplot as plt

DEBUG = False
ONLY_TXT_RESULT = False
pictureFolder = '/home/pi/Pictures/'
modelTrainPicFolder = pictureFolder + 'peopleInMem'
dir_cascade = '/home/pi/toby_ws/data/haarcascades/'
peopleDict = { }

#----------------------------------------people detect------------------------------------------
def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh

def drawPeopleDetections(img, rects, thickness = 1):
    for x, y, w, h in rects:
        pad_w, pad_h = int(0.15*w), int(0.05*h)
        cv2.rectangle( img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0,255,0), thickness )

def peopleDetect(image):
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector( cv2.HOGDescriptor_getDefaultPeopleDetector() )
    try:
        img = cv2.imread(image)
    except:
        print ("Error: loading image input file error in func <peopleDetect>")
    found, w = hog.detectMultiScale(img, winStride=(8,8), padding = (32,32), scale = 1.05 )
    found_filtered = [ ]
    for ri, r in enumerate(found):
        for qi, q in enumerate(found):
            if ri != qi and inside(r, q):
                break
            else:
                found_filtered.append(r)
    drawPeopleDetections(img, found)
    drawPeopleDetections(img, found_filtered, 3)
    print ('Info:<peopleDetect> %d (%d) found' % (len(found_filtered), len(found)) )
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows( )
    #if  0xFF & cv2.waitKey( ):
    #    cv2.destroyAllWindows( )
            
#----------------------------------------face detect------------------------------------------
def detectCvCall(img, cascade):
    #rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=5 )
    if len(rects) == 0:
        print ("Warning: len(rects) value is 0 in func<detectCvCall>")
    else:        
        rects[ : , 2: ]  += rects[ : , :2]
        print ("Info: len(rects) value is %d; rects[] is " %len(rects), rects )
    return rects
    
def draw_rects(img, faces, color):
    for x1, y1, x2, y2 in faces:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    
def detectFaces(imageIn):
    '''get the position of faces in pic, ro'''
    image = cv2.imread(imageIn)    #, cv2.IMREAD_GRAYSCALE )
    imagePIL = Image.open(imageIn)
    #imagePIL = Image.fromarray( image )  #, mode=0 )                      #frombuffer       fromarray(obj, mode=None)  .convert('RGB')         #cv2 to PIL
    #imagePIL.save('./imagePIL.JPG')
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # if __debug__:
    if DEBUG:
        cv2.imshow('Gray_imageIn', gray)
        cv2.waitKey( 0 )
        cv2.destroyWindow('Gray_imageIn')
        
    #gray = cv2.equalizeHist(gray)
 #   if __debug__:
    if DEBUG:
        cv2.imshow('Gray_equalizeHist', gray)
        cv2.waitKey( 0 )
        cv2.destroyWindow('Gray_equalizeHist')
        
    # .xml as input
    cascade_fns = ['haarcascade_frontalface_alt.xml', 'haarcascade_frontalface_alt_tree.xml', 'haarcascade_frontalface_alt2.xml']    #'haarcascade_frontalface_default.xml'
    eye_fns          = ['haarcascade_eye.xml', 'haarcascade_eye_tree_eyeglasses.xml', 'haarcascade_mcs_eyepair_small.xml']

    #---------------------- faces detect ------------------------------------
    for cascade_fn in cascade_fns:
        cascade_file = dir_cascade +  cascade_fn
        face_cascade = cv2.CascadeClassifier( cascade_file )
        faces_rects = detectCvCall(gray, face_cascade)
        if len(faces_rects) != 0:
            print ('Yeah! Get faces at:', faces_rects)
            break
        else:
            print ('Info: Did not recognize face using %s. continue with next cascade_fn....' %cascade_fn )

    if len(faces_rects) is 0:
        print ('XInfo: Did not find any faces in file %s' %imageIn) 
                  
    vis = image.copy( )                                             # copy the image, draw on it.
    draw_rects(vis, faces_rects, (0, 255, 0) )
    if DEBUG:                                                             #__debug__:
        cv2.imshow('NickFaceDraw', vis)
        cv2.waitKey( 2 )
        cv2.destroyWindow('NickFaceDraw') 

    #-------------------------- eyes detect ----------------------------------
    #eyesPosition =  [  [ ], [ ]  ]
    n = 1
    for (x1, y1, x2, y2) in faces_rects:
        #save ROI        
        roi_color = image[y1:y2, x1:x2]
        if DEBUG:                                                #__debug__:
            cv2.imshow('NickRoiColor', roi_color)
            cv2.waitKey( 0 )

        #os.mkdir(pictureFolder + "peopleInView"  )
        roi_color_fn = pictureFolder + "peopleInView/" + str(n) + '_roi.JPG'
        cv2.imwrite(roi_color_fn, roi_color)
        
        # Eyes Recognization using gray
        roi_gray = gray[y1:y2, x1:x2]
        for eye_fn in eye_fns:
            eye_file = dir_cascade + eye_fn
            eye_cascade = cv2.CascadeClassifier(eye_file)
            eyes_rects = detectCvCall( roi_gray, eye_cascade)        #eyes = detectCvCall( roi_gray.copy(), eye_cascade)            
            if len(eyes_rects) == 2:
                print ('Yeah! Get 2 eyes at:', eyes_rects)
                break
            else:
                print 'Info: Did not recognize 2 eyes using %s on face number %d. Continue with next cascade_fn...'  %(eye_fn, n)

        if DEBUG:
            roiCopy = roi_color.copy( )
            draw_rects(roiCopy, eyes_rects, (255, 0, 0) )                  #draw_rects( vis_roi_color,  eyes, (255, 0, 0) )
            cv2.imshow('NickRoiColor', roiCopy)
            cv2.waitKey( 0 )
            
        # Crop the face
        if len(eyes_rects) == 2:
            eyes_rects = eyesRectsExchange(eyes_rects)
            [  [x1L, y1L, x2L, y2L] ,  [x1R, y1R, x2R, y2R]] = eyes_rects            
            eyeLcenter = ( (x2L+x1L)/2,  (y2L+y1L)/2  )
            eyeRcenter = ( (x2R+x1R)/2,  (y2R+y1R)/2 )
            print ("Info:eyeLcenter and eyeRcenter are:", eyeLcenter, eyeRcenter)
            cropFace(imagePIL, (x1, y1, x2, y2),   n,  eye_left = eyeLcenter, eye_right =  eyeRcenter)
            draw_rects(roi_color, eyes_rects, (255, 0, 0) )
        elif len(eyes_rects) == 1:
            print ('Waring: Failed to recognize eyes in roi face. len ==1. eyes_rect is:', eyes_rects)
        elif len(eyes_rects) is 0:
            print ('Waring: Failed to recognize eyes in roi face. len == 0.  eyes_rect is:', eyes_rects)
        n += 1                  # end for(x1, y1, x2, y2) in faces_rects                 
    
    #cv2.waitKey(  0 )    
    #cv2.destroyAllWindows( )
    return imagePIL, faces_rects

def cropFace(imageIn, faceRect,  number,  eye_left=(0, 0), eye_right = (0, 0), offset_pct = (0.05, 0.05), dest_sz =  (200, 200)  ):    #PIL imageIn
    '''crop one face and save.
      rotate -> scale -> crop -> save (30976 pixels = 176 *176) for train
      offset_pct: percent of yhe image you want to keep next to the  eyes (h, v)
    dest_sz: size of the output image'''

    (x1, y1, x2, y2) = faceRect
    sizeOrg_h = x2 - x1
    sizeOrg_v = y2 - y1
    #offsets in original image 
    offset_h = math.floor(float(offset_pct[0]) * sizeOrg_h)               #dest_sz[0] 
    offset_v = math.floor(float(offset_pct[1]) * sizeOrg_v)               #dest_sz[1] 
    #reference = dest_sz[0] - 2.0*offset_h                       #????
    reference = sizeOrg_h * sizeOrg_v
    ##scale = float(dist) / float(reference)    #???
    scale = float(dest_sz[0] * dest_sz[1] ) / float(reference)
    print ("Info: Scale is: ", scale, "offset_h, offset_v are:", offset_h, offset_v, "sizeOrg_h, sizeOrg_v are:", sizeOrg_h, sizeOrg_v)    
    #---------------------- Rotate the image -----------------------
    #get direction
    eye_direction = (eye_right[0] - eye_left[0], eye_right[1] -eye_left[1] )
    (xL, yL) =  eye_left
    (xR, yR) = eye_right
    eyesCenter = (eyesCenterX, eyesCenterY) = ( (xL + xR)/2, (yL + yR)/2 )
    rotateCenter = ( eyesCenterX + x1, eyesCenterY + y1)                               #add roi position
    dist = calcuDistance(eye_left, eye_right)
    print ('eyeCenter is:', eyesCenter, ", dist of eyes is:", dist)
    rotation = -math.atan2(float(eye_direction[1]), float(eye_direction[0] ) )
    imgRotated = scaleRotateTranslate(imageIn, center = rotateCenter,  angle = rotation )  # scale = 0, only do rotation this time

    print imgRotated.mode, imgRotated.size
    imgRotated.save("./imgRatated.JPG")
    #------------------------ Crop the rotated image- ------------
    boxCrop = ( int(x1 - offset_h),  int(y1- offset_v), int(x2 + offset_h), int(y2 + offset_v) )
    faceCropped = imgRotated.crop( boxCrop )   # (left, upper, right, lower)
    print faceCropped.mode, faceCropped.size
    faceCropped.save("./faceCropped.JPG")
    #------------------------------- Resize-------------------------------
    face = faceCropped.resize(dest_sz, Image.ANTIALIAS)

    # show and save
    face.save("./faceResized.JPG")
    #face.show( )
    face.save(pictureFolder + "peopleInView/" + str(number) + '_crop.pgm')
    

def scaleRotateTranslate(image, angle, center = None, new_center = None, scale = None):
    if (scale is None) and (center is None):
        return image.rotate(angle=angle, resample=resample)
    nx,ny = x,y = center
    sx=sy=1.0
    if new_center:
        (nx,ny) = new_center
    if scale:
        (sx,sy) = (scale, scale)
        print ('Info: scale = ', scale)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    a = cosine/sx
    b = sine/sx
    c = x-nx*a-ny*b
    d = -sine/sy
    e = cosine/sy
    f = y-nx*d-ny*e
    print ('Info: image.size is:', image.size )
    #(szOrg_x, szOrg_y) = image.size
    #sz_new = ( int(szOrg_x * scale), int(szOrg_y * scale) ) 
    return image.transform(image.size, Image.AFFINE, (a,b,c,d,e,f), resample=Image.BICUBIC)

    
def calcuDistance(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt( dx*dx + dy*dy )

def eyesRectsExchange(eyesRects):
    [  [x1L, y1L, x2L, y2L] ,  [x1R, y1R, x2R, y2R]] = eyesRects
    
    if x1L > x1R:
        eyesRects_Ex = [   [x1R, y1R, x2R, y2R],  [x1L, y1L, x2L, y2L]  ]
        return eyesRects_Ex
    else:
        return eyesRects
                                                          
#----------------------- face Recognize(process the normalized image) ---------------------    
def createCsv(path):
    csvFile = '%s%s' %(path, '/csv.txt')
    print ("Info: csvFile is:", csvFile)
    f = open(csvFile, 'w')
    c = 0
    for dirname, dirnames, filenames in os.walk(path):
        for subdirname in dirnames:
            subject_path = os.path.join(dirname, subdirname)
            for filename in os.listdir(subject_path):
                abs_path = "%s/%s" %(subject_path, filename)
                f.write( "%s%s%s%s%d\n" %(abs_path, ";", subdirname, ':', c)  )
                print "%s%s%s%s%d" %(abs_path, ";", subdirname, ':', c)
            c += 1
    f.close()
                

def jpg2pgm_v1(path):
    for dirname, dirnames, filenames in os.walk(path):   #string, list, list
        for subdirname in dirnames:
            subject_path = os.path.join(dirname, subdirname)
            print ('Info: jpg2pgm subject path is:', subject_path )
            for filename in os.listdir(subject_path):
                try:
                    im = Image.open( os.path.join(subject_path, filename) )
                    print ('Info: jpg2pgm file processing is:', filename)                                       
                    if (filename[-4:] == '.jpg' ) or (filename[-4:] == '.JPG' ) :           # [:4] keep 4 frontend;  [-4:] keep 4 backend
                        pgmFilename = "%s%s" %(filename[:-4], '.pgm')                  # [4:] remove 4 frontend; [:-4] remove 4 backend
                        im.save( os.path.join(subject_path, pgmFilename) )
                        print ("Info: jpg->pgm converting finished", os.path.join(subject_path, pgmFilename) )
                    im = None
                except:
                    print "Unexpected error: during jpg2pgm transfer"
                    
def jpg2pgm_v2(path):
    for dirname, dirnames, filenames in os.walk(path):   #string, list, list
        for subdirname in dirnames:
            subject_path = os.path.join(dirname, subdirname)
            print ('Info: jpg2pgm subject path is:', subject_path )
            for filename in os.listdir(subject_path):
                f, e = os.path.splitext(filename)
                outfile = f + '.pgm'
                if filename != outfile:
                    try:
                        print ('Info: jpg2pgm file processing is:', filename)
                        Image.open(os.path.join(subject_path, filename) ).save(os.path.join(subject_path, outfile) )
                    except IOError:
                        print "Unexpected error: during <jpg2pgm_v2>"
                             

def normalize(X, low, high, dtype=None):
    ''' array X -> [low, high] '''
    X = np.asarray(X)
    minX, maxX = np.min(X), np.max(X)
    # to (0....1)
    X = X - float(minX)
    X = X / float( (maxX - minX) )
    # to (low...high)
    X = X * (high - low)
    X = X + low
    if dtype is None:
        return np.asarray(X)
    return np.asarray(X, dtype=dtype)  #???


def readTrainingImages(path, sz=None):            # read to cv2
    c = 0
    X, y = [], []                                       # y is label
    for dirname, dirnames, filenames in os.walk(path):   #string, list, list
        for subdirname in dirnames:
            peopleDict[c]  = subdirname
            subject_path = os.path.join(dirname, subdirname)
            for filename in os.listdir(subject_path):
                try:
                    print ('$$cv2.imread training image from %s' %filename)
                    im = cv2.imread(os.path.join(subject_path, filename), cv2.IMREAD_GRAYSCALE )
                     #resize to given size
                    if (sz is not None):
                        im = cv2.resize(im, sz)
                    X.append(np.asarray(im, dtype = np.uint8 ))
                    y.append(c)
                    #y.append(subdirname)
                except IOError, (errno, strerror):
                    print ("I/O error({0}): {1}", format(errno, strerror) )
                except:
                    print "Unexpected error:", sys.exc_info()[0]
                    raise
            c = c + 1                                                                        # for <subdirname>
            print ('Info: folder in <read_images> is:', subdirname, ' peopleDict{} is:' , peopleDict)     
    return [X, y]


def modelTrain(folder):
    (X, y) = readTrainingImages( folder )
    y = np.asarray(y, dtype=np.int32)                               # 
    model = cv2.createEigenFaceRecognizer()                 # @use default params @
    model.train(np.asarray(X), np.asarray(y) )
    return model


def faceRec( model,  fn_face):
    out_dir = "/home/pi/Pictures/peopleInView"
    try:
        face = cv2.imread(fn_face, cv2.IMREAD_GRAYSCALE )
        #print ('$$cv2.imread image to be recognized%s' %fn_face, 'size of face is:', cv2.size(face) )
        #resize to given size
        # if (face.size is not ):
        #     im = cv2.resize(im, sz)
    except IOError, (errno, strerror):
        print ("I/O error({0}): {1}", format(errno, strerror) )

    (p_label, p_confidence) = model.predict(np.asarray( face ) )
    print 'Predicted label = %d (confidence = %.2f)' %(p_label, p_confidence)

    #------------------------ plot the Eigenface --------------------
    print 'Info: model.getParams result: ', model.getParams( )     
    mean = model.getMat("mean")
    eigenvectors = model.getMat('eigenvectors')
    mean_norm   = normalize(mean, 0, 255, dtype=np.uint8)
    mean_resized = mean_norm.reshape( face.shape )
    cv2.imwrite("%s/mean.png" %(out_dir),  mean_resized)          
    #cv2.imshow('mean', mean_resized)
    #cv2.waitKey(0)
    
    #--------- turn the first 16 eigenvectors into grayscale ------
    #print ('info: len of face in <faceRec> is:', len(model) )
    for i in xrange( min( 5 , 16 ) ):   #  len(model)
        eigenvector_i = eigenvectors[:,i].reshape(face.shape)
        eigenvector_i_norm = normalize(eigenvector_i, 0, 255, dtype = np.uint8)
        cv2.imwrite('%s/eigenface_%d.png'%(out_dir, i), eigenvector_i_norm)
        #cv2.imshow('%s/eigenface_%d' %(out_dir, i), eigenvector_i_norm)
        
    #show the images:
    #cv2.waitKey(0)
    #cv2.destroyAllWindows( )
    
    return (peopleDict[p_label], p_confidence)

def drawText(img, text, position = (0, 0) ):
    draw = ImageDraw.Draw(img)
    #image =  
    #draw = ImageDraw(  )
    ft = ImageFont.truetype("/usr/share/fonts/truetype/roboto/Roboto-Bold.ttf", 40)
    draw.text( position, text, font = ft, fill = 'red')
    #img.show(title='recgResult' )                                                                #show(self, title=None, command=None)
    img.save('/home/pi/Pictures/xy.JPG')
    img_cv2 = cv2.imread('/home/pi/Pictures/xy.JPG')
    if ONLY_TXT_RESULT:
        print ('@@@: ', text,  position )
    else:
        cv2.imshow('recg_result', img_cv2)
        cv2.waitKey( 300 )

def facesRecog(imagePIL, faces_rects):
    n = 0
    result = [ ]
    for face_rect in  faces_rects:
        [x1,y1, x2, y2] = face_rect
        n += 1
        (name, confidence) = faceRec(model,  '/home/pi/Pictures/peopleInView/' + str(n) + '_crop.pgm')
        drawText(imagePIL, name + ', confidence='+str(confidence),  (x1, y1) )
        result.append((x1, y1), name, confidence)

    print ('*****len of <facesProcess> is:***', len(result), ', result is:', result)    
    return result

if __name__ == '__main__':
    #peopleDetect('/home/pi/Pictures/nick/nick_2.JPG') 
    #jpg2pgm_v1('/home/pi/Pictures/peopleInMem')
    createCsv('/home/pi/Pictures/peopleInMem')            #path, label
    model = modelTrain( modelTrainPicFolder  )

    while True:
         os.system('raspistill -t 10 -w 800 -n -h 600 -v --thumb none --exposure night --awb incandescent -o /home/pi/Pictures/x.JPG')                            #-v: verbose   --awb fluorescent
         imagePIL, faces_rects = detectFaces('/home/pi/Pictures/x.JPG')
         #imagePIL, faces_rects = detectFaces('/home/pi/Pictures/peopleInView/nana_nick_1.JPG')
         #nana_6.JPG: 0 eye;  nana_1/9.JPG: 0 face   nana_8.JPG: 1 eye
         #nick_1/5: only R eye        nick_2/3.JPG: L<-> R    nick_4.JPG: cannot recog
         facesRecog(imagePIL, faces_rects)
             
























