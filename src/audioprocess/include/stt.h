#ifndef STT
   #define STT

#include "audio_cmn.h"

#define BUFFER_SIZE   4096
#define FRAME_LEN      640
#define HINTS_SIZE     100

const char* stt(void);
int  run_iat(const char *audio_file, char * rec_result, const char* session_params);
int upload_userwords();

#endif
