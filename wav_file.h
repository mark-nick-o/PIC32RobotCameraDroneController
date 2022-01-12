#ifndef __WAV_FILE_H_
#define __WAV_FILE_H_
#ifdef __cplusplus
extern "C" {
#endif
/* 
   -----------------------------------------------------------------------------
      It is a library for utilising the popular wav Format for sound data
      Written by ACP Aviation
   -----------------------------------------------------------------------------
*/
#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define WAVPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define WAVPACKED __attribute__((packed))                                     /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define WAVPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define WAVPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define WAVPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct WAVPACKED {
    /* chunk "riff" */
    char ChunkID[4u];                                                           /* "RIFF" */
    /* sub-chunk-size */
    uint32_t ChunkSize;                                                         /* 36 + Subchunk2Size */
    /* sub-chunk-data */
    char Format[4u];                                                            /* "WAVE" */
} WAV_RIFF_t;
#else
WAVPACKED(
typedef struct {
    /* chunk "riff" */
    char ChunkID[4u];                                                           /* "RIFF" */
    /* sub-chunk-size */
    uint32_t ChunkSize;                                                         /* 36 + Subchunk2Size */
    /* sub-chunk-data */
    char Format[4u];                                                            /* "WAVE" */
}) WAV_RIFF_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct WAVPACKED {
    /* sub-chunk "fmt" */
    char Subchunk1ID[4u];                                                       /* "fmt " */
    /* sub-chunk-size */
    uint32_t Subchunk1Size;                                                     /* 16 for PCM */
    /* sub-chunk-data */
    uint16_t AudioFormat;                                                       /* PCM = 1*/
    uint16_t NumChannels;                                                       /* Mono = 1, Stereo = 2, etc. */
    uint32_t SampleRate;                                                        /* 8000, 44100, etc. */
    uint32_t ByteRate;                                                          /* = SampleRate * NumChannels * BitsPerSample/8 */
    uint16_t BlockAlign;                                                        /* = NumChannels * BitsPerSample/8 */
    uint16_t BitsPerSample;                                                     /* 8bits, 16bits, etc. */
} WAV_FMT_t;
#else
WAVPACKED(
typedef struct {
    /* sub-chunk "fmt" */
    char Subchunk1ID[4u];                                                       /* "fmt " */
    /* sub-chunk-size */
    uint32_t Subchunk1Size;                                                     /* 16 for PCM */
    /* sub-chunk-data */
    uint16_t AudioFormat;                                                       /* PCM = 1*/
    uint16_t NumChannels;                                                       /* Mono = 1, Stereo = 2, etc. */
    uint32_t SampleRate;                                                        /* 8000, 44100, etc. */
    uint32_t ByteRate;                                                          /* = SampleRate * NumChannels * BitsPerSample/8 */
    uint16_t BlockAlign;                                                        /* = NumChannels * BitsPerSample/8 */
    uint16_t BitsPerSample;                                                     /* 8bits, 16bits, etc. */
}) WAV_FMT_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct WAVPACKED {
    /* sub-chunk "data" */
    char Subchunk2ID[4u];                                                       /* "data" */
    /* sub-chunk-size */
    uint32_t Subchunk2Size;                                                     /* data size */
    /* sub-chunk-data */
//    Data_block_t block;
} WAV_Data_t;
#else
WAVPACKED(
typedef struct {
    /* sub-chunk "data" */
    char Subchunk2ID[4u];                                                       /* "data" */
    /* sub-chunk-size */
    uint32_t Subchunk2Size;                                                     /* data size */
    /* sub-chunk-data */
//    Data_block_t block;
}) WAV_Data_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct WAVPACKED {
   WAV_RIFF_t riff;
   WAV_FMT_t fmt;
   WAV_Data_t dataV;
} WAV_Wav_t;
#else
WAVPACKED(
typedef struct {
   WAV_RIFF_t riff;
   WAV_FMT_t fmt;
   WAV_Data_t dataV;
}) WAV_Wav_t;
#endif

#if (defined(D_FT900) || defined(__TI_ARM__))
typedef struct WAVPACKED {
   int16_t hours;
   int16_t minutes;
   int16_t seconds;
   int16_t milliseconds;
} WAV_time_t;
#else
WAVPACKED(
typedef struct {
   int16_t hours;
   int16_t minutes;
   int16_t seconds;
   int16_t milliseconds;
}) WAV_time_t;
#endif

/* ================= Function definitions =================================== */
float32_t read_wav_block( const WAV_Wav_t rawFile, WAV_RIFF_t *riff, WAV_FMT_t *format, WAV_Data_t *dataVals );
void wav_recording_time(float32_t raw_seconds, WAV_time_t *hms);

 /*-----------------------------------------------------------------------------
 *  read_wav_block : reads the wav file and returns the duration
 *
 *  Parameters: const WAV_Wav_t rawFile, WAV_RIFF_t *riff, WAV_FMT_t *format, 
 *              WAV_Data_t *dataVals       
 *
 *  Return: float32_t
 *----------------------------------------------------------------------------*/
float32_t read_wav_block( const WAV_Wav_t rawFile, WAV_RIFF_t *riff, WAV_FMT_t *format, WAV_Data_t *dataVals )
{

  if ((riff = NULL) || ((format == NULL) || (dataVals == NULL))) return 0u; 
  riff = &rawFile.riff;                                                         /* copy the eaw data read from the file into each section */
  format = &rawFile.fmt;
  dataVals = &rawFile.dataV;
  if (format->ByteRate == 0u) return 0u;                                        /* catch divide by zero */
  return ((float32_t) dataVals->Subchunk2Size / (float32_t) format->ByteRate);  /* returns the duration */
}

 /*-----------------------------------------------------------------------------
 *  wav_recording_time : converts the recording time in seconds into a time format
 *
 *  Parameters: float32_t raw_seconds, WAV_time_t *hms    
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void wav_recording_time(float32_t raw_seconds, WAV_time_t *hms) 
{
  int16_t hours_residue = (int16_t) raw_seconds % 3600;

  hms->hours = (int16_t) raw_seconds / 3600;
  hms->minutes = hours_residue / 60;
  hms->seconds = hours_residue % 60;
  hms->milliseconds = (int16_t) ((raw_seconds - (float32_t)((hms->hours*3600) + (hms->minutes*60) + hms->seconds)) *1000.0f);
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif