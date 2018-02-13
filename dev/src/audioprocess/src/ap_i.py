#!/usr/bin/env python
"""
  audio process (board) top file. 1 audio out(mouth) actionServer and 1 audio in publisher 
  ----
  Licensed under BSD license.
  0.2 - 2016.10.27 Integrate into ros
  0.1 - 2015.11.13 by Nick Qian
  ----
  OpenAL + (Julius or https://github.com/cmusphinx  ? # http://stackoverflow.com/questions/14307816/live-recognition-with-python-and-pocketsphinx
"""

import sys, os
#import robiClib as robiC
import libaudioInIFlib as audioInIF

import rospy
from std_msgs.msg import String

import actionlib
from audioprocess.msg import mouthAction, mouthGoal, mouthFeedback, mouthResult
from audioprocess.msg import soundMsg

from ap_o import spkOut


class ap_i():
    
    def __init__(self):

        self.robot_character  = rospy.get_param('~robot_character',  False)
        
        self.soundMsg = soundMsg( )
        self.sound_pub = rospy.Publisher("tpc_sound", soundMsg, queue_size = 5)

        self.micA = mic('/home/pi/toby_ws/data/speech/speech.wav')
        self.spk = spkOut("this is the default text.")
        
        rospy.init_node('node_ap_i')
        self.r = rospy.Rate(2)        
            

        
    def launch(self):
        listenResult = None
        
        while not rospy.is_shutdown():
            try:
                listenResult = self.micA.startListen()
                print ("$$listenResult is:", listenResult)
            except KeyboardInterrupt, ke:
                audioInIF.mop_Up_Mic( )      #mic_Do_MopUp
                print ("!!! Keyboard Exception occur!!! Will break...ke is", ke)
                break            
            except Exception, e:
                print("!!! Exception occur!!! Will break...", e)
            
            if listenResult == 2:
                self.soundMsg.hasSoundInEvt = True
            else:
                self.soundMsg.hasSoundInEvt = False
                
            if listenResult == 3:
                self.soundMsg.hasSoundFile = True
                self.soundInAnalyze( )
                try:
                    #sttResult = self.micA.stt_cloud()
                    sttResult = 'Dummy Speech To Text result.'
                    print ("***stt:%s***" %sttResult)
                    ##break
                    if sttResult != '':
                        self.spk.tts_cloud(sttResult)
                        self.spk.shout('ACCORD_STT_MCH')
                except KeyboardInterrupt, ke:
                    print ("!!! Keyboard Exception occur!!! Will break...ke is", ke)
                    break   
            else:
                self.soundMsg.hasSoundFile = False                
                
            # publish the info collected
            self.pub_sound_info()
            self.r.sleep()


    def pub_sound_info(self):
        self.soundMsg.txt_stt = self.micA.sttResult
        
        try:
            self.sound_pub.publish(self.soundMsg)
            print ("~~~ Pub once! ~~~ tpc is %s, hasSound & hasFile are:" %"tpc_sound", self.soundMsg.hasSoundInEvt, self.soundMsg.hasSoundFile)
            if self.soundMsg.hasSoundInEvt is True:
                print ("Pub msg txt is:%s" %self.soundMsg.txt_stt)
        except:
            traceback.print_exec()
            rospy.logerr("Error: Publish sound info failed in func <pub_sound_info>")

    def soundInAnalyze(self):
        result = None
        os.system('cp /home/pi/toby_ws/data/speech/micrec.wav /home/pi/toby_ws/data/speech/speech.wav')
        return result

    

class mic:
    #micRecognized = 0
    def __init__(self, speechIn):
        self.speechfile = speechIn
        self.listenResult = 0
        self.sttResult = "This is default mic string not stt."
        audioInIF.initAlsaMicrophone( )
        print ('Initialized mic:', self.speechfile)

    def startListen(self, ):
        self.listenResult = audioInIF.micStartListen( )
        return self.listenResult
    
    def eventDetect(self, ):
        pass

    def eventProcess(self, ):
        pass 

    def stt_cloud(self):
        self.sttResult = audioInIF.stt()
        return self.sttResult

    def stt_local(hmmd, lmdir, dictp, wavfile):
        try:
            import pocketsphinx as ps
            import sphinxbase
        except:
            print ''' Pocket sphinx and sphinxbase is not installed..'''
        speechRec = ps.Decoder(hmm = hmmd, lm = lmdir, dict = dictp)
        wavFile = file(wavfile, 'rb')   #???
        wavFile.seek(44)
        speechRec.decode_raw(wavFile)
        result = speechRec.get_hyp()

        return result[0]
    
    def detect_fx(self, audioIn):
        pass

    def detect_animal(self, audioIn):
        pass



class getResponse( ):
    pass


class evalResponse( ):
    
    pass


def main():
    apin = ap_i()
    apin.launch()

def test_loop():
    spk = spkOut("this is the default text.")
    micA = mic('/home/pi/toby_ws/data/speech/speech.wav')
    
    spk.tts_cloud("I'm so beautiful that even the toad want to bite me!")
    ''' I'm so beautiful that even the toad want to bite me. -> I'm so beautiful that evening all want to buy
    I'm so ugly that even the dog won't bite me              -> I'm so happy that didn't dog bite
    '''        
    i = 0
    while i < 5:
        os.system('cp /home/pi/toby_ws/data/speech/shout.wav  /home/pi/toby_ws/data/speech/speech.wav')
        textListen = micA.stt_cloud()
        if textListen is not None:
            print ("@Python View: stt result is: %s" %textListen)
            spk.tts_cloud(textListen)
            spk.shout('ACCORD_STT_MCH')
        else:
            print ("@Python View: Error: None stt result!")
            spk.shout('NO_STT_RSLT')
            
        i += 1
    
    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
    print 'Test TTS-STT loop Run Ended.'
    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'


def test_dialog():  # with mic audio input
    spk = spkOut("this is the default text.")
    micA = mic('/home/pi/toby_ws/data/speech/speech.wav')
    dGen = dialogGen("Tom")
    
    i = 0
    while i < 5:
        micA.startListen()
        textListen = micA.stt_cloud()

        if textListen is not None:
            textResp= dGen.AliceBot(textListen)
            print ("@@@ the AliceBot respond is: %s" %textResp)
            if textResp =='' :
                spk.shout('NO_MCH_RSLT')
            else:
                spk.tts_cloud(textResp)
                spk.shout('ACCORD_STT_MCH')    
        else:
            spk.tts_cloud("Sorry I didn't catch you. Could you please say it again?")
            spk.shout('NO_STT_RSLT')
    
        i += 1
        print ("@repeat count i is:", i)
        
    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
    print 'Test Robot Dialog Ended.'
    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
    

if __name__ == '__main__':    
    main()
    #test_dialog()
    #test_loop()
   
        
    
        
