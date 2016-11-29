#include <cstring>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>   // perror
#include <cassert>
#include <stdbool.h>

#include "stt.h"
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"

using namespace std;

static char rec_result[BUFFER_SIZE] = {0}; //{NULL};

const char* stt(){

   int ret        = MSP_SUCCESS;
   bool rslt_login_1, rslt_login_2, rslt_login_3, rslt_login_4;
   const char *wavfilename = speech_wav_file;  //speech.wav(mono) micrec.wav   speechIn.wav
   //int upload_on  = 1;   // 1: upload 
   const char *login_params = "appid = 5677c22a, work_dir = ."; //

   char *ptr_rcg_result = &rec_result[0];
   for(int i = 0; i < BUFFER_SIZE; i++){
      rec_result[i] = 0;
   } 

   // sub, type, 
   // domain: iat-stt, search-words search, poi-place name, video, music
   // language: zh_cn(default), zh_tw, en_us
   // accent: mandarin(default), cantonese, lmz, henanese, dongbeiese 
   // sample_rate: only support 16000(default) and 8000 
   // result_type: plain, json, xml
   // result_encoding: xml, plain:gb2312, utf-8, unicode json:utf8
   const char *session_params = "sub = iat, domain = iat, language = en_us, \
       accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8";


   ///// login //////
   ret = MSPLogin(NULL, NULL, login_params);    // usr name, password, can be NULL
   if(MSP_SUCCESS != ret){
      cout << "ERROR: MSPLogin_failed. Error code: " << ret << "Try again..." << endl;
      rslt_login_1 = (bool)MSPLogin(NULL, NULL, login_params);
      rslt_login_2 = (bool)MSPLogin(NULL, NULL, login_params);
      rslt_login_3 = (bool)MSPLogin(NULL, NULL, login_params);
      rslt_login_4 = (bool)MSPLogin(NULL, NULL, login_params);
      if (false == (rslt_login_1 | rslt_login_2 | rslt_login_3 | rslt_login_4)  ){
          cout << "ERROR: Too many STT login false. Check the internet connection." << endl;
      goto exit;
      }
   }
   else 
      cout << "Info: MSPLogin successfully." << endl;

   cout << "Info: start iat....." << endl;

   run_iat (wavfilename, ptr_rcg_result, session_params);
   cout << "<== end run_iat. " << endl;

exit:
    MSPLogout();

    return rec_result;
}

int  run_iat(const char *audio_file, char * rec_result, const char* session_params){    // send wav to xunfei cloud

    const char *session_id       = NULL;
//    char rec_result[BUFFER_SIZE] = {0}; //{NULL};
    char hints[HINTS_SIZE]       = {0}; //{NULL};  //reason for end of session
    unsigned int  total_len      = 0;
    int aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;  // 2 as audio during, 4 as last data block
    int ep_stat  = MSP_EP_LOOKING_FOR_SPEECH;  // endpoint detect
    int rec_stat = MSP_REC_STATUS_SUCCESS;
    int errcode  = MSP_SUCCESS;

    FILE *f_pcm = NULL;
    char *p_pcm = NULL;
    int  pcm_count = 0;        // long?
    int  pcm_size  = 0;
    int read_size  = 0;

    assert (NULL != audio_file);    

    f_pcm = fopen(audio_file, "rb");
    assert (NULL != f_pcm);

    fseek(f_pcm, 0, SEEK_END);
    pcm_size = ftell(f_pcm);     // get the size of audio file
    //cout << "pcm_size value is:" << pcm_size << endl; 
    fseek(f_pcm, 0, SEEK_SET);

    //////allocate memory
    p_pcm = (char *)malloc(pcm_size);
    assert (NULL != p_pcm);

    ////// Read wav file
    read_size = fread( (void *)p_pcm,  1, pcm_size,  f_pcm);
    assert (read_size == pcm_size);

    //cout << "Info:start speech to text[QISRSeesionBegin]" << endl;
    session_id = QISRSessionBegin(NULL,  session_params,  &errcode);
    assert (MSP_SUCCESS == errcode);   //@@@

    while (1){
       unsigned int len = 10 * FRAME_LEN;    //write 200ms=10frames audio every time. 20ms/frame.16k sample rate, bit bit depth
       int ret = 0;

       if (pcm_size < 2* len)
          len = pcm_size;

       if ( len <= 0)
          break;

       //////aud _stat ////
       aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
       if (0 == pcm_count)
          aud_stat = MSP_AUDIO_SAMPLE_FIRST;

       cout << "." << flush;

       ret = QISRAudioWrite (session_id, (const void *)&p_pcm[pcm_count], len, aud_stat, &ep_stat, &rec_stat );
       if (MSP_SUCCESS != ret){
          cout << "ERROR: QISRAudioWrite failed. error code:" << ret << endl;
         goto iat_exit;
       }

      pcm_count += (int)len;
      pcm_size  -= (int)len;

      if  (MSP_REC_STATUS_SUCCESS == rec_stat){    // alredy has result segment 

         const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);      // check status

         if (MSP_SUCCESS != errcode){
            cout << "ERROR: QISRGetResult failed. Error code:" << errcode <<"..Try again.."<< endl;
            rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
            if (MSP_SUCCESS != errcode){
                cout << "ERROR: 2 times failure: QISRGetResult. Will exit! " << endl;
                goto iat_exit;
            }
         }

         cout << ":" << flush;

         if (NULL != rslt){
            unsigned int rslt_len = strlen(rslt);
            total_len += rslt_len;
            assert (total_len < BUFFER_SIZE);
            strncat(rec_result, rslt, rslt_len);           // connect result portions
         }
       } // portion result
     
      if (MSP_EP_AFTER_SPEECH == ep_stat)
         break;

      usleep(200 * 1000);     //simulate the interval in speech of human.  200ms==10 frame

     } // while 

     /////write  wav to cloud///////
     errcode = QISRAudioWrite (session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST, &ep_stat, &rec_stat);
     assert (MSP_SUCCESS == errcode);

//     int cnt_err = 0;

     while (MSP_REC_STATUS_COMPLETE != rec_stat){
        const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
        if ( MSP_SUCCESS != errcode ){
            cout << "Error: QISRGetResult<LAST> failed. errcode is:" << errcode << "..Try again.."<< endl;
            rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
            if (MSP_SUCCESS != errcode){
                cout << "ERROR: 2 times failure: QISRGetResult. Will exit! " << endl;
                goto iat_exit;
            }
        }
        /////// get the result from cloud////////
        if(NULL != rslt){
           unsigned int rslt_len = strlen(rslt);
           total_len += rslt_len;
           assert(total_len < BUFFER_SIZE);
           strncat(rec_result, rslt, rslt_len);
           cout << "--}" << ends;
        }
       usleep(150 * 1000);           // 150ms to avoid too much CPU occupy
      }  // while

     cout << "Info: MSP_REC_STATUS_COMPLETE. " << endl;

     cout << "@@@@ rec_result is:" ;
     for (int i = 0; i< BUFFER_SIZE; i++){
          cout << rec_result[i] ;
      }
     //cout << "\n Info: END STT." << endl;

iat_exit:
     if (NULL != f_pcm){
       fclose(f_pcm);
       f_pcm = NULL;          // reset the *
     }

     if (NULL != p_pcm){
       free(p_pcm);
       p_pcm = NULL;
     }

     QISRSessionEnd(session_id, hints);

}

int upload_userwords(){

    char         *userwords = NULL;
    unsigned int len        = 0;
    unsigned int read_len   = 0;
    FILE         *fp        = NULL;
    int          ret        = -1;

    fp = fopen("userwords.txt", "rb");
    assert (fp != NULL );

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);        // get audio file size
    fseek(fp, 0, SEEK_SET);

    userwords = (char *)malloc( len + 1);
    assert (NULL != userwords);

    read_len = fread( (void *)userwords, 1, len, fp);  // read user words
    assert (read_len == len);

    userwords[len] = '\0';

    //MSP Upload data
    MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret);
    assert (MSP_SUCCESS == ret);

upload_exit:
    if (NULL != fp){
      fclose(fp);
      fp = NULL;
    }

    if(NULL != userwords){
      free(userwords);
      userwords = NULL;
    }

    return ret; 

}

