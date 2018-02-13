#ifndef USE_XFM10211_USB
   #define USE_XFM10211_USB 1

#define ALSA_PCM_NEW_HW_PARAMS_API

#define NUM_AVG_SMPLE   8        // frame
#define GATE_THRESHOLD  10000     // 1388    // 3276=32768/10
#define GATE_NOISE      500      //
#define NUM_SILENCE_TKL 1500     // 200ms = 399 for 16KHz //200ms = 1100 for 44100sample[frames]/s.  200ms = 8820sample/8
#define NUM_PRINT_GAP   700000   // 500ms

/*
typedef enum {

   SND_IDLE         = 0,    //silence
   SND_IN_DURING    = 1,
   SND_IN_LONG_SIL  = 2
}SndInStatus;
*/

typedef enum {
   MIC_INIT                = 0,
   MIC_IDLE                = 1,
   MIC_DETECT_IN           = 2,
   MIC_SND_DURING          = 3,
   MIC_DETECT_FS           = 4              //find silence
}MicStatusID;

typedef enum {
   BUF_FLG_BLANK    = 0,    //silence
   BUF_FLG_FI  = 1,         // fade in
   BUF_FLG_FO  = 2,         // fade out
   BUF_FLG_FULL     = 3
}Buf_Flg;

int micStartListen( );
int eventDetect(int *sndEvtInd, int *silTick, char *sndBufIn);
void showInfoAlsa();

#endif
