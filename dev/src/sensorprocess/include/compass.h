/*  c driver for compass module of RTP project
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.3.15 init version for Raspberry 2B(bcm2836) - by Nick Qian
    ----
    output: distance measured & inquiried by sens.py
    input:  module(HMC5883L)
*/

#ifndef _COMPASS_H_
  #define _COMPASS_H_

#define __DEBUG__

#define USE_HMC5883L                    // Honeywell
#define I2C_DEV_ADDR_COMPASS  0x1E


#define PUB_RATE_CMPS_HZ   1

//0x3C
//0x3D

#ifdef USE_HMC5883L
  #pragma message("@Compile MSG: Using HMC5883L as compass sensor")

/////////////////// Reg Addr ///////////////////
#define ADDR_CFG_REG_A        0x0
#define ADDR_CFG_REG_B        0x1
#define ADDR_MODE_SEL         0x2
#define ADDR_MAG_DAT_X        0x3
#define ADDR_MAG_DAT_Y        0x5
#define ADDR_MAG_DAT_Z        0x7
#define MAG_DAT_ADDR_START    ADDR_MAG_DAT_X
#define ADDR_STATUS           0x9
#define ADDR_FLG_A            0xA
#define ADDR_FLG_B            0xB
#define ADDR_FLG_C            0xC

///////////////////// Value ////////////////////
#define CAL_THRESHOLD         0
#define MAGNETC_DECLINATION   -5.93
#define PI  3.141592653f

typedef enum{
#ifdef USE_HMC5883L
    COMPASS_SAMPLE_8       = 0b11,
    COMPASS_SAMPLE_4       = 0b10,
    COMPASS_SAMPLE_2       = 0b01,
    COMPASS_SAMPLE_1       = 0b00
#endif
}compass_samples_t;

typedef enum{
    COMPASS_DATARATE_75HZ  = 0b110,
    COMPASS_DATARATE_30HZ  = 0b101,
    COMPASS_DATARATE_15HZ  = 0b100,
    COMPASS_DATARATE_7P5_HZ= 0b011
}compass_dataRate_t;

typedef enum{
    COMPASS_RANGE_8_1GA    = 0b111,
    COMPASS_RANGE_5_6GA    = 0b110,
    COMPASS_RANGE_4_7GA    = 0b101,
    COMPASS_RANGE_4GA      = 0b100
}compass_range_t;

typedef enum{
    COMPASS_IDLE           = 0b10,
    COMPASS_SINGLE         = 0b01,
    COMPASS_CONTINOUS      = 0b00
}compass_mode_t;


typedef enum{
    CFG_MODE_NORMAL        = 0b00,
    CFG_MODE_POS_OFFSET    = 0b01,
    CFG_MODE_NEG_OFFSET    = 0b10
}compass_cfg_reg_ms;

#endif    // if use HMC5883L



void compassInit();
int  compassUpdate();
void calibrateMag();
void getCompassOffset();
int  calcCompassHeading(int *x, int *y, int *z);
void setCompassMeasureMode(compass_mode_t mode);
void setCompassDataRate(compass_dataRate_t dataRate);
void setCompassSamples(compass_samples_t samples);
void setCompassRange(compass_range_t range);

extern int16_t magX, magY, magZ;
extern float magGain[3];
extern int16_t offsetX, offsetY, offsetZ;


#endif

