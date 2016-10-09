#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>           // open /close
//#include <sys/ioctl.h>      // misc operation with HW. OSS needs. Alsa may not need
//#include <linux/i2c-dev.h>
#include <alsa/asoundlib.h>   // use API
#include <sys/soundcard.h>
//#include <csignal>

using namespace std;

#include "peri.h"
#include "mic.h"
#include "audio_cmn.h"

wav_pcm_hdr init_wav_speech_hdr = {       //values same as TTS
   {'R','I','F','F'},
   0,       //size_8
   {'W','A','V','E'},
   {'f','m','t',' '},
   16,

   1,       //pcm
   2,       //channel = 2
   16000,   // 16KHz sample
   64000,   // 16KHz * 2CH * 2bytes = 32000 byte data/second
   2,       // 2 byte /sample
   16,      // sample depth

   {'d','a','t','a'},
   0         // data size
};

/*
int init_alsa_mic(){

return 0;

}*/

//static int micInStatus    = SND_IDLE;

int micStartListen( ){
    int rc;
    int ret = 0;              //report get speech and rec done
    int loops;
    int evd_res;              // event detect result 
    int cnt_dbg, cnt_loop;
    int size;
    wav_pcm_hdr  wav_hdr =  init_wav_speech_hdr;
    FILE *fp = NULL;
    const char *filename = "./speech/micrec.wav";  //micrec.wav";  // speech.wav
    snd_pcm_t *handle;
    snd_pcm_hw_params_t *params;
    unsigned int val;
    int dir;
    snd_pcm_uframes_t frames;
    char *buffer;


    fp = fopen(filename, "wb");
    assert(NULL != fp);

    rc = snd_pcm_open(&handle,
                     "hw:1,0",    //"default",
                     SND_PCM_STREAM_CAPTURE, 
                     0);
   // <pre name = "code"  class = "programlisting">  
    if(rc <0 ){
       cout << "ERROR: unable to open PCM device:" << snd_strerror(rc) << endl;
       exit(1);
    }

    snd_pcm_hw_params_alloca(&params);       // Allocate a hardware parameters object
    snd_pcm_hw_params_any(handle, params);   // fill with default values

    //////////// set desired HW params:////////////////

    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);  //Interleaved
    snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE); //signed 16bit little-endian
    snd_pcm_hw_params_set_channels(handle, params, 2);   // 2 channels(stereo)

    val = wav_hdr.samples_per_sec;     //16000; //44100;    // 44100 sample rate
    snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);

    frames = 32;            //period size = 32 frames
    snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

    /////// write to the driver
    rc = snd_pcm_hw_params(handle, params);
    if (rc < 0){
       fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc) );
       exit(1);
    }

    ///////make buffer to hold one period
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    size = frames * 4;   // 2byte/sample, 2 channels
    buffer = (char*)malloc(size);


    ////get period time, set loop period//////
    snd_pcm_hw_params_get_period_time(params, &val, &dir);
    loops = 40000000 / val; //test 5000ms

    ////// write wav head & add rec data /////////
    fwrite (&wav_hdr, sizeof(wav_hdr), 1, fp);          //init head, will change later
 
    cnt_loop = 0;

    int micInEvtInd    = MIC_IDLE;   //MIC_EVT_INIT
    int silTick        = 0;

    while (loops > 0){
   //while(MIC_EVT_IDLE_== micInEvtInd){
     loops--;

     rc = snd_pcm_readi(handle, buffer, frames);   // snd_pcm_t *pcm, void *buffer, snd_pcm_uframes_t size

     if (-EPIPE == rc){   // overrun
         fprintf(stderr, "overrun occurred\n");
         snd_pcm_prepare(handle);     // ?
     } else if (rc < 0){
        fprintf(stderr, "error from read: %s\n", snd_strerror(rc) );
     } else if (rc != (int)frames ){
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
            cout << "(O|)"<< ends;
            cout << "Bingo! get sound."  << ends;

            buffer_flg = BUF_FLG_FI ;
            ptr_detected = ptr_curdetect - (NUM_AVG_SMPLE * 4);    //record position
            // break;                           // stop analyze this period
        }

        if (MIC_DETECT_FS == micInEvtInd) {
           cout << "(|O) Ahhhaa! find a silence."<< ends;

           buffer_flg = BUF_FLG_FO ;
           ptr_detected = ptr_curdetect; 

        }

      }  // end for(analyzed the buff)

     ///////////////// End Once Analyze ///////////////

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
      }

     if (MIC_SND_DURING == micInEvtInd ){
        if (ptr_detected == buffer) {
            buffer_flg = BUF_FLG_FULL;
            //if (NUM_PRINT_GAP == cnt_loop)
            cout << "|" ;
        }
    }

    //////////// write 1 period/buffer to output ////////////////
    //compute the audio length
    // no buffer contains both FI and FO because the gap is longer than buffer 
    int audio_len;

    if ( BUF_FLG_FI == buffer_flg){
       audio_len = (size - (int)(ptr_detected - buffer) );
       fwrite(ptr_detected, audio_len, 1, fp);

       wav_hdr.data_size += audio_len;

       cout << "[@@@@" <<  audio_len << "@@@@]" << ends;
    }

    if( BUF_FLG_FULL == buffer_flg){
          audio_len = size;
//          cout << "[" << audio_len << "]" << ends;
          fwrite(ptr_detected, audio_len, 1, fp);
          wav_hdr.data_size += audio_len;
    }


    // DETECT Fade out(Find Silence) == micInEvtInd
    if (BUF_FLG_FO == buffer_flg){
       audio_len = (int)(ptr_detected - buffer);
       cout << "==-->Fadeout, last data write to file.audio_len is :" << audio_len << endl;

       fwrite(buffer, audio_len, 1, fp);
       wav_hdr.data_size += audio_len;

       ///////End of write////////
       // write back to wav head
       cout << "$collect head info and write head. data_size is:" << wav_hdr.data_size  << endl;
       wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
       fseek(fp, 4, 0);
       fwrite( &wav_hdr.size_8, sizeof(wav_hdr.size_8), 1, fp);            //size_8
       fseek(fp, 40, 0);          // data_size
       fwrite(&wav_hdr.data_size, sizeof(wav_hdr.data_size), 1, fp);       //data_size
       fclose(fp);
       fp = NULL;

       ret = 1;
       goto exit;             // end the loop
      }

     // if(rc != size){
     //    fprintf(stderr, "shot write: wrote %d bytes\n", rc);
     // }

   } // while


exit:
   snd_pcm_drain(handle);
   snd_pcm_close(handle);
   free(buffer); 


   cout << "Info:End. free the buffer" << endl;


return ret;
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
           cout << *sndEvtInd;
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
              cout << *sndEvtInd ;
           } // silTick ==  NUM_SILENCE_TKL
         } // < threshold

        if (*silTick > 0){
           if ( abs(volmAvg) > GATE_THRESHOLD ){
                *silTick = 0;
                cout << "V";
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





