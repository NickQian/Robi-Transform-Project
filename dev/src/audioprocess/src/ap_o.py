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
import random
from std_msgs.msg import String

import actionlib
from audioprocess.msg import mouthAction, mouthGoal, mouthFeedback, mouthResult
from audioprocess.msg import soundMsg

sys.path.append("/home/pi/toby_ws/data/speech/aiml")
import pyAIML as aiml

#import logging
import traceback
import tempfile


sentenceDict ={}
senDic ={}
''' 5 kinds of emotions for 1 sentence
senDic = {'hello, bitch1': sentenceWithEmo[1], 
          'hello, bitch2': sentenceWithEmo[2], 
          'hello, bitch3': sentenceWithEmo[3]   }
sentenceDict = {'index1.1': {'hello, bitch1': 'D:\TobyQin\hello_bitch_emo1.wav'},
                'index1.2': {'hello, bitch2': 'D:\TobyQin\hello_bitch_emo2.wav'},
                'index1.3': {'hello, bitch3': 'D:\TobyQin\hello_bitch_emo3.wav'}}
'''

#----------------Situation Dialogue---------------------
index = 'indexSD1_1'         #Situation Dialogue
sentence = 'hello, bitch'
fileLoc = 'D:\TobyQin\hello_bitch_emo1.wav'  # 'D:/TobyQin/hello_bitch_emo1.wav'
senDic[sentence] = fileLoc 
#senDic = {sentence, fileLoc}
sentenceDict[index] = senDic #sentenceDict = {index, senDic}
#-------------------EN Course----------------------
index = 'indexEC1.2.3'       #EN Course-> Grade 2-> Page 3
sentence = 'I am 2cm longer than you!'
fileLoc =   '/home/pi/toby_ws/data/speech/shout.wav'   
senDic[sentence] = fileLoc   # add
sentenceDict[index] = senDic
#-------------------------------------------------------

speechWavFolder="/home/pi/toby_ws/data/speech/"



class ap_o():
    _feedback = mouthFeedback( )
    _res = mouthResult( )
    
    def __init__(self):

        self.robot_character  = rospy.get_param('~robot_character',  False)
        self.use_CloudTTS = rospy.get_param('~use_CloudTTS', False)

        self._as = actionlib.SimpleActionServer('srv_mouth', mouthAction, self.mouth_srv_execute, False)

        self.soundMsg = soundMsg( )
     
        self.spk = spkOut("this is the default text.")
        self.dGen = dialogGen("Tom")
        
        rospy.init_node('node_ap_o')
        self.r = rospy.Rate(2)
        
            

        
    def launch(self):
        self._as.start()
        print ("Info:mouth actlib server started. Enter into Spin")
        rospy.spin()

    
    def mouth_srv_execute(self, goal):
        print ('mouth_srv_execute: goal is:', goal)
        success = True

        #check preempted firstly
        if self._as.is_preempt_requested():
            rospy.loginfo("!!ap action server preempted!! ")
            self._as.set_preempted()
            success = False

        # do tts, get wav file, and play it.
        if (goal.txt == '' ):
            self.spk.shout('NO_MCH_RSLT')
        else:
            if self.use_CloudTTS is True:
                self.spk.tts_cloud(goal.txt)
                self.spk.shout('ACCORD_STT_MCH')
            else:
                self.spk.tts_festival(goal.txt)

        # publish the feedback
        self._as.publish_feedback(self._feedback)

        # sleep
        self.r.sleep()

        if success:
            self._as.set_succeeded(self._res)
            rospy.loginfo("!! mouth action server: Succeeded!! ")   


class mutterGen():
    
    mutterList_What   = [speechWavFolder+"WhatsThat/what_is_that.wav",
                         speechWavFolder+"WhatsThat/what_is_this.wav",
                         speechWavFolder+"WhatsThat/who_is_that.wav",
                         speechWavFolder+"WhatsThat/what_is_moving.wav" ]
    
    mutterList_Boring = [speechWavFolder+"Boring/i_am_so_beautiful.wav",
                         speechWavFolder+"Boring/longer_than_you.wav",
                         speechWavFolder+"Boring/boring_1.wav",
                         speechWavFolder+"Boring/boring_2.wav",
                         speechWavFolder+"Boring/boring_3.wav"  ]

    mutterList_HaveRest = [speechWavFolder+"HaveRest/have_rest_1.wav",
                           speechWavFolder+"HaveRest/have_rest_2.wav",
                           speechWavFolder+"HaveRest/have_rest_3.wav",
                           speechWavFolder+"HaveRest/have_rest_4.wav"  ]
    
    def __init__(self):
        self.mutter = None

    def run(self, kind, playit=True):
        if kind == 'WHAT':
            self.mutter = mutterGen.mutterList_What[random.randint(0,len(mutterGen.mutterList_What)-1 )]
            
        if kind == 'BORING':
            self.mutter = mutterGen.mutterList_Boring[random.randint(0,len(mutterGen.mutterList_Boring)-1 )]

        if kind == 'HAVE_REST':
            self.mutter = mutterGen.mutterList_HaveRest[random.randint(0,len(mutterGen.mutterList_HaveRest)-1 )]
            
        if playit is True:
            os.system('aplay '+speechWavFolder+'beep0.wav')
            os.system('aplay '+self.mutter)


class strikeUpTalkGen():
    strikeUpList_What   = [speechWavFolder+"StrikeUpTalk/what_you_doing_0.wav",
                           speechWavFolder+"StrikeUpTalk/what_you_doing_1.wav"  ]
    def __init__(self):
        self.strikeUpSentence = None

    def run(self, kind, playit=True):
        if kind == 'WHAT_DOING':
            self.strikeUpSentence = strikeUpTalkGen.strikeUpList_What[random.randint(0,len(strikeUpTalkGen.strikeUpList_What)-1 )]

        if playit == True:
            os.system('aplay '+speechWavFolder+'beep4.wav')
            os.system('aplay '+self.strikeUpSentence)


class reportGen():
    def __init__(self):
        self.txt = 'blank'
        self.ttsEngine = spkOut('Blank report text')
        self.use_CloudTTS = rospy.get_param('~use_CloudTTS', False)

    def run(self, txt2report, playit=True):
        self.txt = txt2report
        
        os.system('aplay '+speechWavFolder+'beep6.wav')

        if self.use_CloudTTS is True:
            self.ttsEngine.tts_cloud(self.txt)                    
            os.system('aplay '+speechWavFolder+'shout.wav')
        else:
            self.ttsEngine.tts_festival(self.txt) 



class spkOut:
    voice = "voice_don_diphone"   #
    
    def __init__(self, spkDefaultSentence):
        self.sentence = spkDefaultSentence
        self.tts_mt_result = 'ACCORD_STT_MCH'
        
        self.sentencefile = '/home/pi/toby_ws/data/speech/shout.wav'
        print ('Initialized speak sentence:', self.sentence)
        
    def sentence2index(self):
        pass

    def tts_festival(self, stringInput):        
        txtFile = tempfile.NamedTemporaryFile(prefix='txt2synth', suffix='.txt')
        txtFilename = txtFile.name
        (wavFile, wavFilename) = tempfile.mkstemp(prefix='shoutSynthed', suffix='.wav')
        os.close(wavFile)
        
        try:
            txtFile.write(stringInput)
            txtFile.flush()
            os.system("text2wave -eval '("+ spkOut.voice +")' "+ txtFilename+" -o "+ wavFilename)
            try:
                if os.stat(wavFilename).st_size == 0:
                    raise OSError
            except OSError:
                rospy.logerr('Sound synthesis failed. festival or voice not installed?? ' )
                return
            #theVoice = self.wavFilename
        finally:
            txtFile.close()
            
        #self.shout_tempfile()
        os.system('aplay '+ wavFilename)
                
            

    def tts_local(self, speed, emotion, emphasize):
        self.file = senDic[self.sentence]
        print ('Will paly sound:', self.file)
        os.system('omxplayer -o local /home/pi/toby_ws/data/speech/shout.wav')
        #winsound.PlaySound(self.file, winsound.SND_FILENAME)
        pass
    
            
        
    def tts_cloud(self, tts_sentence_in, speed=3, emotion=0, emphasize=0 ):
        
        audioInIF.tts(tts_sentence_in)
        #"voice_name = henry, text_encoding = UTF8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2"       

        return 0

    
    def shout_tempfile(self):        
        os.system('aplay '+self.wavFilename)

 
    def shout(self, tts_mch_result_in, txt_initiative = None):
        self.tts_mt_result = tts_mch_result_in
        #os.system('aplay ./speech/speech.wav')
        if txt_initiative is None:
            if (self.tts_mt_result == 'NO_STT_RSLT'):
                os.system('aplay /home/pi/toby_ws/data/speech/no_stt_result.wav')
            elif(self.tts_mt_result == 'NO_MCH_RSLT'):
                os.system('aplay /home/pi/toby_ws/data/speech/no_match_result.wav')
            elif(self.tts_mt_result == 'ACCORD_STT_MCH'):
                os.system('aplay /home/pi/toby_ws/data/speech/shout.wav')
        else:
            os.system('aplay /home/pi/toby_ws/data/speech/beep0.wav')
            os.system('aplay /home/pi/toby_ws/data/speech/shout.wav')
            
    
    def beep(self):
        os.system('aplay /home/pi/toby_ws/data/speech/beep0.wav')

    def fxInMem(self):
        pass



class getResponse( ):
    pass



class evalResponse( ):
    
    pass



class dialogGen:
    dlgList_NothingToSay = [speechWavFolder+"NothingToSay/no_match_result0.wav",
                            speechWavFolder+"NothingToSay/no_match_result1.wav" ]
    dlgList_NotClear     = [speechWavFolder+"NotClear/no_stt_result.wav" ]
    
    def __init__(self, sessionID):
        self.sessionID = sessionID
        self.aimlKnl = aiml.Kernel()
        #self.aknl.learn("./speech/aiml/std-startup.xml")
        #self.aknl.respond("load aiml b")     #??? don't work in current version?
        #self.aknl.learn("./speech/aiml/basic_chat.aiml")
        #self.aknl.learn("./speech/aiml/sex.aiml")
        #self.aknl.learn("./speech/aiml/music.aiml")
        #self.aknl.learn("./speech/aiml/religion.aiml")
        if os.path.isfile("/home/pi/toby_ws/data/speech/roby_brain.brn"):
            self.aimlKnl.bootstrap(brainFile = "/home/pi/toby_ws/data/speech/roby_brain.brn")
        else:
            self.aimlKnl.bootstrap(learnFiles = "/home/pi/toby_ws/data/speech/aiml/std-startup.xml", commands = "load aiml b")
            self.aimlKnl.saveBrain("/home/pi/toby_ws/data/speech/roby_brain.brn")

        self.aimlKnl.getSessionData(self.sessionID)
        self.aimlKnl.setBotPredicate("Roby", "name")
        self.aimlKnl.setPredicate("dog", "Brandy", self.sessionID)   # session people's dog name is Brandy

        self.sentenceIn = "!-- this is the default dialog sentence in. You idiot. "
        self.sentenceOut = "!-- this is the default dialog sentance out. You idiot."
        print ("...Initializing class dialogGen... done")
        
    def AliceBot(self, sentenceIn ):
        self.sentenceIn  = sentenceIn
        self.sentenceOut = self.aimlKnl.respond(self.sentenceIn)
        return self.sentenceOut
        #while True: print self.ak.respond(raw_input("> "))



def gen_tts_wav(text_in, fn_wav):
    spk = spkOut("this is the default text.")
    spk.tts_cloud(text_in)
    spk.shout('ACCORD_STT_MCH') 
    os.system('cp /home/pi/toby_ws/data/speech/shout.wav  /home/pi/toby_ws/data/speech/%s' %fn_wav)



def test_aiml():
    spk = spkOut("this is the default text.")
    dGen = dialogGen("Tom")   # sessionID = key of the session dict


    textListen = ["HELLO" , "what are you?", "bite me!", "i need xxx", "YOU ARE HornY"]
    i = 0
    while i < 5:
        print ("@Python View: stt result is: %s" %textListen[i])
        spk.tts_cloud(textListen[i])
        #os.system('cp /home/pi/toby_ws/data/speech/shout.wav  /home/pi/toby_ws/data/speech/speech.wav')
        spk.shout('ACCORD_STT_MCH')
        
        if textListen is not None:
            textResp= dGen.AliceBot(textListen[i])
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
    print 'Test AIML Run Ended.'
    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'



def test():
    
    #test_aiml()

    #gen_tts_wav("yes! i'm having my rest.", "have_rest_4.wav")

    #mG = mutterGen()
    #mG.run('WHAT', playit=True)

    spk = spkOut("this is the default text.")
    spk.tts_festival("hi, Nick")


def main():
    ap_mouth = ap_o()
    ap_mouth.launch()



if __name__ == '__main__':
    
    main()
    #test()


    
        
    
        
