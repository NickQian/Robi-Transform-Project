#include <iostream>
#include <stdio.h> //<cstdio>
#include <stdlib.h>
#include <cstring>
#include <unistd.h>
#include <cassert>
#include <stdint.h>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "tts.h"

using namespace std;

wav_pcm_hdr init_wav_hdr = {
   {'R','I','F','F'},
   0,
   {'W','A','V','E'},
   {'f','m','t',' '},
   16,

   1,       //pcm
   1,       //channel =1
   16000,   // 16KHz sample
   32000,   // 32000 byte data/second
   2,       // 2 byte /sample
   16,      // sample depth

   {'d','a','t','a'},
   0
};


int tts(const char *text){       //, const char *tts_args
   const char *login_configs = "appid = 5677c22a, work_dir = . ";
   const char *filename      = shout_wav_file;
   FILE *fp                  = NULL;
   const char *sessionID     = NULL;
   uint32_t audio_len        = 0;
   wav_pcm_hdr  wav_hdr      = init_wav_hdr; //default_wav_hdr;
   int synth_status          = MSP_TTS_FLAG_STILL_HAVE_DATA;
   string str_tts            = text;

  #ifdef USE_XUNFEI_OFL_TTS
    const char *args = "engine_type = local, text_encoding = UTF8, tts_res_path = fo|res/tts/xiaoyan.jet;fo|res/tts/common.jet, sample_rate = 16000";
  #else
    #ifdef USE_XUNFEI_ONL_TTS
      const char *args = "voice_name = henry, text_encoding = UTF8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
    #endif                          // xiaoyan
  #endif

   int ret = MSP_SUCCESS;


   cout << "Start Xunfei MSPLogin... " << endl;
   ///////// login "yun"//////////
   ret = MSPLogin("18602122079", "196805", login_configs ); //NULL, NULL, login_configs);  
   if (ret != MSP_SUCCESS  ){
      cout << "MSPLogin Failed. Error code:" << ret << endl;
      goto exit;
   }
   else 
      cout << "Info:MSPLogin success."  << endl;

   //////////////tts//////////////
   ret = -1;
   assert (NULL != text);             // check the input firstly
   //{
   //   cout << "ERRor: no text as tts input." << endl;
   //   return ret;
   //}

   fp = fopen(filename, "wb");
   assert (NULL != fp);

   sessionID = QTTSSessionBegin(args, &ret);                   // tts start
   if (MSP_SUCCESS != ret){
     cout << "QTTSSessionBegin failed. Error code:" << ret << endl;
     fclose (fp);
     return ret;
   }

   ret = QTTSTextPut( sessionID, text,  (uint32_t)strlen(text),  NULL );   //QTTS text put
   if (MSP_SUCCESS != ret){
     cout << "QTTSTextPut failed, error code:" << ret << endl;
     QTTSSessionEnd(sessionID, "TextPutError");
     fclose(fp);
     return ret;
   }


   cout << "Info:@TTS text:@" << str_tts << endl;
   cout <<  "...TTS Sythesizing..." << flush;
   //////// Write WAV head to file, Collect could Data and add to wav /////////////
   fwrite(&wav_hdr, sizeof(wav_hdr), 1, fp);

cout << "UUUU" << flush;

   while(1){
    const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret); // QTTSAudioGet
    if (MSP_SUCCESS != ret)
       break;

    cout << ":" << flush;

    if (NULL != data){       //write data to fp
       fwrite(data, audio_len, 1, fp);
       wav_hdr.data_size += audio_len;
    }

    if(MSP_TTS_FLAG_DATA_END == synth_status)
       break;

    usleep(150*1000);         // 150ms

   } // while 1

   if (MSP_SUCCESS != ret){
     cout << "ERRor: QTTSAudioGet failed. error code:" << ret << endl;
     QTTSSessionEnd(sessionID, "AudioGetError");
     fclose(fp);
     return ret;
   }
   else 
     cout << "Info:QTTSAudioGet success!" << endl;

   wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8 );

   //write back to the wav head:
   fseek(fp, 4, 0);
   fwrite (&wav_hdr.size_8, sizeof(wav_hdr.size_8), 1, fp);
   fseek(fp, 40, 0);                 // shift file ptr offset to data_size
   fwrite(&wav_hdr.data_size, sizeof(wav_hdr.data_size), 1, fp);
   fclose(fp);
   fp = NULL;
   
   cout << "Info:Write head to wav...Done." << endl;
   //////// END ///////
   ret = QTTSSessionEnd(sessionID, "Normal");
   if (MSP_SUCCESS != ret){
         cout << "ERRor: QTTSSessionEnd failed. error code:" << ret << endl;
   }



  if (MSP_SUCCESS != ret){
     cout << "TTS failed. Error code:" << ret << endl;
  }

  cout << "TTS Done! Exit tts...ret is: " << ret ;

exit:
   ///// logout /////////
   //getchar();
   MSPLogout();

   cout << "MSPLogout!!" << endl;

   return ret;

}




