/*
  mic driver & audio event detect & record. Sample 500ms every time
  ----
  Licensed under BSD license.
  0.2 - 2016.10.30  Integrate into ros & change work period to 500ms
  0.1 - 2016.1.26 by Nick Qian
  ----
  Output: wav record file & event flag
  Input: sound
*/

#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>           // open /close
//#include <sys/ioctl.h>      // misc operation with HW. OSS needs. Alsa may not need
//#include <linux/i2c-dev.h>
#include <alsa/asoundlib.h>   // use API
#include <sys/soundcard.h>

using namespace std;

#include "mic.h"
#include "audio_cmn.h"

wav_pcm_hdr init_wav_speech_hdr = {       //values same as TTS
   {'R','I','F','F'},
   0,       //size_8
   {'W','A','V','E'},
   {'f','m','t',' '},
   16,

   1,                    // pcm
   2,                    // channel = 2
   16000,                // 16KHz sample
   64000,                // 16KHz * 2CH * 2bytes = 32000 byte data/second
   2,                    // 2 byte /sample
   16,                   // sample depth

   {'d','a','t','a'},
   0                     // data size
};

/*
int init_alsa_mic(){

return 0;

}*/


static snd_pcm_t *handle;
static int SoundScanEvtInd  = SCAN_FLG_NULL;
static unsigned int val;
static snd_pcm_uframes_t frames;
static wav_pcm_hdr wav_hdr =  init_wav_speech_hdr;


int initAlsaMicrophone(){
    int ret = 0;
    int rc;
    //wav_pcm_hdr  wav_hdr =  init_wav_speech_hdr;
    int dir;
    snd_pcm_hw_params_t *params;


    cout << "Info: Initializing Alsa Microphone..." << ends;

    rc = snd_pcm_open(&handle,
                     "hw:1,0",    //"default",
                     SND_PCM_STREAM_CAPTURE, 
                     0);
    if(rc <0 ){
       cout << "ERROR: unable to open PCM device:" << snd_strerror(rc) << endl;
       exit(1);
    }

    snd_pcm_hw_params_alloca(&params);       // Allocate a hardware parameters object
    snd_pcm_hw_params_any(handle, params);   // fill parameters object with default values & pass the PCM handle

    //////////// set desired HW params:////////////////
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);  //Interleaved
    snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE); //signed 16bit little-endian
    snd_pcm_hw_params_set_channels(handle, params, 2);   // 2 channels(stereo)
    val = wav_hdr.samples_per_sec;     //16000; //44100;    // 44100 sample rate
    snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);

    frames = 32;            //period size = 32 frames
    snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

    /////// write parames to HW
    rc = snd_pcm_hw_params(handle, params);
    if (rc < 0){
       fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc) );
       exit(1);
    }

    cout << "Done."  << endl;

    cout << "~~~val before extract= " << val << endl;
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    cout << "~~~frames after extract= " << frames << endl;

    ////get period time, set loop period//////
    snd_pcm_hw_params_get_period_time(params, &val, &dir);
    cout << "~~~val after extract= " << val << endl;

    return ret;

}

int mop_Up_Mic(){
   int ret = 1;

   cout << "Info:mop up work of mic.cpp..." << ends;
   //fclose(fp);
   //fp = NULL;
   snd_pcm_drain(handle);    // ?
   snd_pcm_close(handle);
   ret = 0;
   cout << "End." << endl;

   return ret;
}

int micStartListen( ){
    int ret = 0;              //report get speech and rec done
    int loops;
    int cnt_dbg, cnt_loop;
    int size;
    //wav_pcm_hdr  wav_hdr =  init_wav_speech_hdr;
    char *buffer;
    const char *filename = record_wav_file;
    int rc;
    FILE *fp = NULL;

    ///////make buffer to hold one period ???
    size = frames * 4;   // 2byte/sample, 2 channels
    buffer = (char*)malloc(size);

    loops = SCAN_INTERVAL*1000 / val; //test 500ms=> 500 000

    // set the default flag for ros node scan every wakeup
    if (SCAN_FLG_EVT_END==SoundScanEvtInd){
        SoundScanEvtInd = SCAN_FLG_BLANK;
        wav_hdr.data_size = 0;
        snd_pcm_prepare(handle);     // ?
        //snd_pcm_reset(handle);     // reset PCM position
    }

    cnt_loop = 0;

    int micInEvtInd    = MIC_IDLE;
    if (SCAN_FLG_EVT_DURING==SoundScanEvtInd){
        micInEvtInd = MIC_SND_DURING;
        fp = fopen(filename, "ab+");    //ab+  rb+
        //fseek(fp, 0, SEEK_END);
        cout << "Info:file opened & fseek to:"<< ftell(fp) << endl;
    }
    else{
         fp = fopen(filename, "wb");
         assert(NULL != fp);
         cout << "Info: file opened as <wb>.fp position:"<< ftell(fp) << endl;
    }
    int silTick    = 0;

    while (loops > 0){
       //while(MIC_EVT_IDLE_== micInEvtInd){
       loops--;

       rc = snd_pcm_readi(handle, buffer, frames);   // snd_pcm_t *pcm, void *buffer, snd_pcm_uframes_t size
       if (-EPIPE == rc){   // overrun
           snd_pcm_prepare(handle);     // ?
           fprintf(stderr, "overrun occurred. prepared again.\n");
       }
       else if (rc < 0){
          fprintf(stderr, "error from read: %s\n", snd_strerror(rc) );
       }
       else if (rc != (int)frames ){
            cnt_dbg++;
            if (cnt_dbg < 6)
                fprintf(stderr, "Info: snd_pcm_readi(): rc!= frames. short read, read %d frames \n", rc);
            else
                fprintf(stderr, "Info: snd_pcm_readi(): Too many short read, will not reort more. %d frames \n", rc);
       }


       //-------search for sound. process 1 period every > --------
       int buffer_flg = BUF_FLG_BLANK;
       char *ptr_detected  = buffer;   // every loop the buffer, reset the ptr
       char *ptr_curdetect = buffer;


       for (int i = 0; i< ( (int)frames/NUM_AVG_SMPLE );  i++){

           eventDetect(&micInEvtInd, &silTick, ptr_curdetect);

           ptr_curdetect +=  4*NUM_AVG_SMPLE ;//2 channel, 16bit

           // cout << micInEvtInd << ends;
           if ( MIC_DETECT_IN == micInEvtInd ){
               cout << "(o|o)Bingo! get sound.(o|o)"  << ends;

               // set the flag for ros node process
               SoundScanEvtInd = SCAN_FLG_EVT_DURING;

               // init data_size
               wav_hdr.data_size = 0;

               buffer_flg = BUF_FLG_FI ;
               //ptr_detected = ptr_curdetect - (NUM_AVG_SMPLE * 4);    //record position
               ptr_detected = ptr_curdetect;    //record position

               // open file
               //cout <<"!will open file>" << flush;
               //fp = fopen(filename, "wb");
               //assert(NULL != fp);
               //cout << "Info: file opened to record." << endl;

           }

           if (MIC_DETECT_FS == micInEvtInd) {
              cout << "[-->__] Ahhaa! silence."<< ends;

              // set the flag for ros node process
              SoundScanEvtInd = SCAN_FLG_EVT_END;

              buffer_flg = BUF_FLG_FO;
              ptr_detected = ptr_curdetect;
           }

         }  // end for(analyzed the buff)

        ///////////////// End Once Analyze ///////////////

        ////////// flag the buffer ///////////
        if ( MIC_IDLE == micInEvtInd ){
           if  (ptr_detected == buffer) {
               buffer_flg = BUF_FLG_BLANK;
              // if (NUM_PRINT_GAP == cnt_loop)
               if (cnt_loop == 150){
                    cnt_loop = 0 ;
                    cout << "(o)" << flush;
               }
               else{
                    cnt_loop++;
               }
           }
         }   // end if


        if (MIC_SND_DURING == micInEvtInd ){
           if (ptr_detected != buffer) {   //
               buffer_flg = BUF_FLG_FI;
               //if (NUM_PRINT_GAP == cnt_loop)
               cout << "##" << flush ;
           }
           else{
              buffer_flg = BUF_FLG_DURING_FULL;
              cout << "|" << flush;
           }
        }


       //////////// write 1 period/buffer to output ////////////////
       //compute the audio length
       // no buffer contains both FI and FO because the gap is longer than buffer 

       int audio_len;
       char *ptr_wr_fadeIn  = ptr_detected-(NUM_AVG_SMPLE * 4);   // every loop the buffer, reset the ptr

       if ( BUF_FLG_FI == buffer_flg){
          // audio_len = (size - (int)(ptr_detected - buffer) );
          audio_len = (size - (int)(ptr_wr_fadeIn - buffer) );
          cout << "+@@+" << flush;

          ///////// write wav head & add rec data /////////
          fwrite (&wav_hdr, sizeof(wav_hdr), 1, fp);          //init head, will change later

          ///////// write Fade In sound data ///////////
          /*
          if (WR_FADEIN_OFFSET >= ptr_detected - buffer){
               ptr_wr_fadeIn = buffer;
               offset_fadeIn = (int)(ptr_detected - buffer);
          }
          else{
               ptr_wr_fadeIn = ptr_detected - WR_FADEIN_OFFSET;
               offset_fadeIn = WR_FADEIN_OFFSET;
          }
          fwrite(ptr_wr_fadeIn, audio_len+offset_fadeIn, 1, fp);

          wav_hdr.data_size += audio_len+offset_fadeIn;
          */
          fwrite(ptr_wr_fadeIn, audio_len, 1, fp);
          wav_hdr.data_size += audio_len;

          //cout << "@audio_len is:" <<  audio_len << "@" << ends;
       }

       if( BUF_FLG_DURING_FULL == buffer_flg){

          audio_len = size;
          fwrite(ptr_detected, audio_len, 1, fp);
          cout << ":" << flush;

          wav_hdr.data_size += audio_len;

          // set the flag for ros node process
          SoundScanEvtInd = SCAN_FLG_EVT_DURING;
        }


       // DETECT Fade out(Find Silence) == micInEvtInd
       if (BUF_FLG_FO == buffer_flg){
          audio_len = (int)(ptr_detected - buffer);
          cout << "==-->Fadeout, write last. audio_len is :" << audio_len << endl;

          fwrite(buffer, audio_len, 1, fp);
          wav_hdr.data_size += audio_len;
          ///////End of data write////////

          // write back to wav head
          fclose(fp);
          fp = fopen(filename, "rb+"); // reopen to reallocate the fp to file head
cout <<"<reopen fp>:" << ftell(fp) << flush;


          wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
          fseek(fp, 4, 0); // offset, from where
cout <<"<size_8 fp>:" << ftell(fp) << flush;
          fwrite( &wav_hdr.size_8, sizeof(wav_hdr.size_8), 1, fp);            //size_8
          fseek(fp, 40, 0);          // data_size
cout <<"<data_size fp>:" << ftell(fp) << flush;

          fwrite(&wav_hdr.data_size, sizeof(wav_hdr.data_size), 1, fp);

          cout << "+&&+. data_size is:" << wav_hdr.data_size << ". size_8 is:" << wav_hdr.size_8 << endl;

          // close file & release file ptr
          //fclose(fp);
          //fp = NULL;
          //cout << "Info:file closed" << endl;

          // set the flag for ros node process
          SoundScanEvtInd = SCAN_FLG_EVT_END;

          ret = 1;
          goto exit;             // end 1 loop
         }

         // if(rc != size){
         //    fprintf(stderr, "shot write: wrote %d bytes\n", rc);
         // }

   } // end while


exit:
   // close file & release file ptr
   fclose(fp);
   fp = NULL;
   free(buffer);
   cout << "Info:file closed & buffer released." << endl;

//return ret;
  return SoundScanEvtInd;   //micInEvtInd;

}


int eventDetect(int *sndEvtInd, int *silTick, char *sndBufIn){         // &micInEvtInd, &pcm

  // int ret       = 0;
   int volmAvg   = 0;
   int16_t sample_l, sample_r;

      if ( MIC_DETECT_FS == *sndEvtInd ){
          *sndEvtInd   = MIC_IDLE;
      }

      if (MIC_DETECT_IN == *sndEvtInd ){
           *sndEvtInd = MIC_SND_DURING;
           cout << *sndEvtInd << flush;
      }


      for (int i = 0; i < NUM_AVG_SMPLE; i++){

          // cout << "sndBufIn 0:" <<(uint16_t)(*sndBufIn) << ", 1:" <<(uint16_t)(*(sndBufIn+1)) << ", 2:" << (uint16_t)(*(sndBufIn+2)) << ", 3:" << (uint16_t)(*(sndBufIn+3)) << endl;
           sample_l =  (int16_t)( (*(sndBufIn+1) << 8) | *sndBufIn );
           sample_r =  (int16_t)( (*(sndBufIn+3) << 8) | *(sndBufIn + 2) );

          // cout << "sample_l is:" << sample_l << ". sample_r is:" << sample_r << endl;

           volmAvg += abs( sample_l  );    // 16bit * 2(left&right)
           volmAvg += abs( sample_r  );
           sndBufIn += 4;            // move the local ptr 

      }

      volmAvg /= (NUM_AVG_SMPLE * 2);
      // cout << "Ten 8 frames volmAvg value is:" << volmAvg << endl;


      if ( MIC_SND_DURING  == *sndEvtInd ){

         if ( abs(volmAvg) < GATE_NOISE) {
           if (*silTick < NUM_SILENCE_TKL) {
               *silTick += 1;
              //cout << "~(" << volmAvg << ")" ;
           }
           else {
              *sndEvtInd = MIC_DETECT_FS;
              *silTick = 0;
              cout << *sndEvtInd << flush;
           } // silTick ==  NUM_SILENCE_TKL
         } // < threshold

        if (*silTick > 0){
           if ( abs(volmAvg) > GATE_THRESHOLD ){
                *silTick = 0;
                cout << "V" <<flush;
           }
        }   //if has tick

      } // if sound during

      if ( MIC_IDLE  == *sndEvtInd ){
         if ( abs(volmAvg) > GATE_THRESHOLD ){    //find sound
            *sndEvtInd   = MIC_DETECT_IN;
            *silTick = 0;
         }
      }

     //cout << "micInStatus is: " << micInStatus << ". *sndEvtInd is: " << *sndEvtInd << endl;

  // ret = *sndEvtInd;

   return 0;

}

void showInfoAlsa(){
    int val;
    cout << "ALSA library version:" << SND_LIB_VERSION_STR << endl;
    cout << "PCM stream types:" << endl;
    for (val = 0; val <= SND_PCM_ACCESS_LAST; val++){
          cout << snd_pcm_access_name((snd_pcm_access_t)val) << endl;
    }
}





