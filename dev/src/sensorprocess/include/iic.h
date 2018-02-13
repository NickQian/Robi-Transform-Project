/*  i2c api (on raspberry pi 2B using "/dev/i2c-1") of RTP project
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.3.10  init version for Raspberry 2B(bcm2836) - by Nick Qian
    ----
*/

#ifndef _IIC_H_
  #define _IIC_H_


//#define __DEBUG__

#include <stdint.h>


//const char *iicFileName = (char *)"/dev/i2c-1";


/******* <linux/i2c-dev.h> *****************
struct i2c_msg{
    __u16 addr;
    unsigned short flags;
    short len;
    char *buf;
}

i2c_rdwr_ioctl_data{
    struct i2c_msg *msgs;
    __u32 nmsgs;
};

*/



int     iicWrBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *wrBuf, uint16_t num);
int     iicWrByte (uint8_t devAddr, uint8_t regAddr, uint8_t  data);
int     iicRdBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *rdBuf, uint16_t num);
uint8_t iicRdByte (uint8_t devAddr, uint8_t regAddr);

uint8_t iicRdBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t len);
int     iicWrBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t len, uint8_t data);

#endif
