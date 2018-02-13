#!/usr/bin/env python
# audio process (board) top file -2015.11.13
# OpenAL + (Julius or https://github.com/cmusphinx  ? # http://stackoverflow.com/questions/14307816/live-recognition-with-python-and-pocketsphinx

import sys, os
#import robiClib as robiC

sys.path.append("/home/pi/toby_ws/data/speech/aiml")
import pyAIML as aiml
import rospy
from std_msgs.msg import String


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
fileLoc =   './speech/shout.wav'   
senDic[sentence] = fileLoc   # add
sentenceDict[index] = senDic
#-------------------------------------------------------

def pubApInfo(mic_stt_result, mouth_status):
    pub = rospy.Publisher('tpc_audio', String, queue_size = 2)
    


class mic:
    #micRecognized = 0
    def __init__(self, speechIn):
        self.speechfile = speechIn
        self.listenResult = 0
        self.sttResult = "This is not stt result. This is default mic string. You idiot! "
        print ('Initialized mic:', self.speechfile)

    def startListen(self, ):
        self.listenResult = robiC.micStartListen( )
        return self.listenResult
    
    def eventDetect(self, ):
        pass

    def eventProcess(self, ):
        pass 

    def stt_cloud(self):
        self.sttResult = robiC.stt()
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



class dialogGen:
    def __init__(self, sessionID):
        self.sessionID = sessionID
        self.aimlKnl = aiml.Kernel()
        #self.aknl.learn("./speech/aiml/std-startup.xml")
        #self.aknl.respond("load aiml b")     #??? don't work in current version?
        #self.aknl.learn("./speech/aiml/basic_chat.aiml")
        #self.aknl.learn("./speech/aiml/sex.aiml")
        #self.aknl.learn("./speech/aiml/music.aiml")
        #self.aknl.learn("./speech/aiml/religion.aiml")
        if os.path.isfile("./speech/roby_brain.brn"):
            self.aimlKnl.bootstrap(brainFile = "./speech/roby_brain.brn")
        else:
            self.aimlKnl.bootstrap(learnFiles = "./speech/aiml/std-startup.xml", commands = "load aiml b")
            self.aimlKnl.saveBrain("./speech/roby_brain.brn")

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



class spkOut:
    def __init__(self, spkDefault):
        self.sentence = spkDefault
        self.tts_mt_result = 'ACCORD_STT_MCH'
        self.sentencefile = './speech/shout.wav'
        print ('Initialized speak sentence:', self.sentence)
    def sentence2index(self):
        pass
        
    def tts_cloud(self, tts_sentence_in, speed=3, emotion=0, emphasize=0 ):
        
        robiC.tts(tts_sentence_in)
        #"voice_name = henry, text_encoding = UTF8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2"       

        return 0
 
    def shout(self, tts_mch_result_in, txt_initiative = None):
        self.tts_mt_result = tts_mch_result_in
        #os.system('aplay ./speech/speech.wav')
        if txt_initiative is None:
            if (self.tts_mt_result == 'NO_STT_RSLT'):
                os.system('aplay ./speech/no_stt_result.wav')
            elif(self.tts_mt_result == 'NO_MCH_RSLT'):
                os.system('aplay ./speech/no_match_result.wav')
            elif(self.tts_mt_result == 'ACCORD_STT_MCH'):
                os.system('aplay ./speech/shout.wav')
        else:
            os.system('aplay ./speech/nagging.wev')

    def tts_local(self, speed, emotion, emphasize):
        self.file = senDic[self.sentence]
        print ('Will paly sound:', self.file)
        os.system('omxplayer -o local ./speech/shout.wav')
        #winsound.PlaySound(self.file, winsound.SND_FILENAME)
        pass
    
    def beep(self):
        os.system('aplay ./speech/beep.wav')

    def fxInMem(self):
        pass


class getResponse( ):
    pass


class evalResponse( ):
    pass


class ap():
    def __init__(self):
        pass
    def launch(self):
        pass
        


if __name__ == '__main__':

    spk = spkOut("this is the default text.")
    #spk.tts_cloud("I'm so ugly that even the dog won't bite me!")
    ## I'm so beautiful that even the toad want to bite me. -> I'm so beautiful that evening all want to buy
    ## I'm so ugly that even the dog won't bite me              -> I'm so happy that didn't dog bite
    #os.system('aplay ./speech/shout.wav')

    micA = mic('./speech/speech.wav')
    #micA.stt_cloud()
    dGen = dialogGen("Tom")   # sessionID = key of the session dict


    textListen = ["HELLO" , "what are you?", "bite me!", "i need xxx", "YOU ARE HornY"]
    i = 0
    while i < 1:
        ##os.system('cp ./speech/shout.wav  ./speech/speech.wav')
        #micA.startListen()        
        #textListen = micA.stt_cloud()
        print ("@Python View: stt result is: %s" %textListen[i])
        spk.tts_cloud(textListen[i])
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
        
    #    if i > 5:
    #        break

    # micB = mic

    # hmdir = 'D:/TobyQin/speechmodel/hmm'
    # lmd = 'D:/TobyQin/speechmodel/lm/xxx.lm.DMP'
    # dictd = 'D:/TobyQin/speechmodel/lm/xx.dic'
    # wavfile = 'D:/TobyQin/speechmodel/speechSample.wav'
    # recognised = micHearA.decodeSpeech(hmdir, lmd, dictd, wavfile)

    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
    print 'Python Run Ended.'
    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'

    
        
    
        
