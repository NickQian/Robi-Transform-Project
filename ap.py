# audio process (board) top file -2015.11.13
# OpenAL + (Julius or https://github.com/cmusphinx  ?
# http://stackoverflow.com/questions/14307816/live-recognition-with-python-and-pocketsphinx
import sys, os
import robiClib as robiC

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

class mic:
    #micRecognized = 0
    def __init__(self, speechIn):
        self.speechfile = speechIn
        self.listenResult = 0
        print ('Initialized mic:', self.speechfile)

    def startListen(self, ):
        self.listenResult = robiC.micStartListen( )
        return self.listenResult
    
    def eventDetect(self, ):
        pass

    def eventProcess(self, ):
        pass 

    def stt_cloud(self):
        robiC.stt(self)
        #self.txtResult = speechRecognize(audioIn)
        #print ('mic2txt result:%s' %self.txtResult)
        #return self.txtResult

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



class itlDialog:
    def AliceBot(self, sentence ):
      pass 




class spkOut:
    def __init__(self, spkSomething):
        self.sentence = spkSomething
        print ('Initialized speak sentence:', self.sentence)
    def sentence2index(self):
        pass
        
    def tts_cloud(self, tts_sentence_in, speed=3, emotion=0, emphasize=0 ):
        
        robiC.tts(tts_sentence_in)
        #"voice_name = henry, text_encoding = UTF8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2"       

        return 0

    def tts_local(self, speed, emotion, emphasize):
        self.file = senDic[self.sentence]
        print ('Will paly sound:', self.file)
        os.system('omxplayer -o local ./speech/shout.wav')
        #winsound.PlaySound(self.file, winsound.SND_FILENAME)
        pass
    
    def beep(self):
        pass

    def fxInMem(self):
        pass




if __name__ == '__main__':

    spk = spkOut("I'm so ugly that even the dog won't bite me.")
    spk.tts_cloud("I'm so beautiful that even the toad want to bite me.")

    micA = mic('./speech/speech.wav')
    micA.stt_cloud()
 
    i = 0
    while i < 2:
        micA.startListen() 
        textListen = micA.stt_cloud()
        print ("@@@@@@@ STT result:", textListen)
        if textListen == None:
            spk.tts_cloud("Sorry I didn't catch you. Could you please say it again?")
        else:
            spk.tts_cloud(textListen)
            
        i += 1
        print i
    #    if i > 5:
    #        break

   #micB = mic

   # hmdir = 'D:/TobyQin/speechmodel/hmm'
   # lmd = 'D:/TobyQin/speechmodel/lm/xxx.lm.DMP'
   # dictd = 'D:/TobyQin/speechmodel/lm/xx.dic'
   # wavfile = 'D:/TobyQin/speechmodel/speechSample.wav'
   # recognised = micHearA.decodeSpeech(hmdir, lmd, dictd, wavfile)

    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'
    print 'Python call End.'
    print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'

    
        
    
        
