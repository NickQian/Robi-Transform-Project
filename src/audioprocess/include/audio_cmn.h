#ifndef AUDIO_COMMON
  #define  AUDIO_COMMON

#define shout_wav_file "/home/pi/toby_ws/data/speech/shout.wav"
#define speech_wav_file "/home/pi/toby_ws/data/speech/speech.wav"
#define record_wav_file "/home/pi/toby_ws/data/speech/micrec.wav"

//typedef 
struct wav_pcm_hdr{
   /* RIFF WAVE Chunk*/
   char      riff[4];          // "RIFF"
   int       size_8;           // uint32_t
   char      wave[4];          // "WAVE"
   /* Format Chunk*/
   char      fmt[4];           // "fmt"
   int       fmt_size;

   short int format_tag;       // uint16_t
   short int channels;
   int       samples_per_sec;    // 8000 | 6000 |11025 | 16000
   int       avg_bytes_per_sec;
   short int block_align;
   short int bits_per_sample;          // 8 | 16
   /* Data Chunk*/
   char      data[4];         //"data"
   int       data_size;      // puer data
}; //wav_pcm_hdr;

#endif
