/*  i2c api (on raspberry pi 2B using "/dev/i2c-1") of RTP project
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.3.11  init version for Raspberry 2B(bcm2836) - by Nick Qian
    ----
*/

// http://blog.csdn.net/onetwothreef/article/details/49488443


#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>               // memcpy
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "iic.h"
const char *iicFileName = (char *)"/dev/i2c-1";


int iicRdBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *rdBuf, uint16_t num)
{
    int fd;
    uint8_t outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuf = regAddr;

    messages[0].addr  = devAddr;
    messages[0].flags = 0;
    messages[0].len   = 1;
    messages[0].buf   = &outbuf;

    messages[1].addr  = devAddr;
    messages[1].flags = I2C_M_RD;   // | I2C_M_NOSTART 0x4000   //0x0001 read data, from slave to master
    messages[1].len   = num;
    messages[1].buf   = rdBuf;

    packets.msgs      = messages;
    packets.nmsgs     = 2;

    // open i2c bus
    if( (fd = open(iicFileName, O_RDWR) ) <0 ){
        printf("Error: failed to open I2C bus for read.\n");
        goto exit_with_err;
    }
    #ifdef __DEBUG__
    else{
        printf("DBG_Info: I2C opened for read. fd: %d. regAddr: %d \n", fd, regAddr);
    }
    #endif


    if (ioctl(fd, I2C_RDWR, &packets) < 0){    //(int fd, int cmd, (char*)argstruct )
        printf("Error: failed to acquire bus access and/or talk to slave for read.\n");
        goto exit_with_err;
    }

    /*if ( (read(fd, rdBuf, num)) != num ){
        printf("Error: failed to read from i2c bus.\n");
    }
    else{
        printf("Info:I2C read data success: %s\n", rdBuf)    */


    if (fd){
        close(fd);
    }
    return 0;

exit_with_err:

    if (fd){
        close(fd);
    }

    return -1;

}


uint8_t iicRdByte(uint8_t devAddr, uint8_t regAddr)
{
    uint8_t dataRd;

    if( iicRdBytes(devAddr, regAddr, &dataRd,  1) < 0){
        printf("Error: in <iicRdByte>. devAddr: %d, regAddr: %d. \n ", devAddr, regAddr);
        return -1;
    }
    else{
        return dataRd;
    }
}




int iicWrBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *wrBuf, uint16_t num)
{
    int fd;
    //unsigned char buffer[60] = {0};
    uint8_t *outbuf = (uint8_t *)malloc(sizeof(unsigned char)*(num+1));
    if(outbuf==NULL){
        printf("Error: malloc <iicWrBytes>");
        return -1;
    }

    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = devAddr;
    messages[0].flags = 0;
    messages[0].len   = num+1;
    messages[0].buf   = outbuf;

    // I2C Register to write & values to write
    outbuf[0] = regAddr;
    memcpy(outbuf+1, wrBuf, num);

    // Transfer the I2C packets to kernel
    packets.msgs  = messages;
    packets.nmsgs = 1;


    // open i2c bus
    if( (fd = open(iicFileName, O_RDWR) ) <0 ){
        printf("Error: failed to open I2C bus for write. fd: %d, regAddr: %d. \n", fd, regAddr);
        goto err_exit;
    }
    #ifdef __DEBUG__
    else{
        printf("DBG_Info: I2C opened for write. fd: %d, regAddr: %d.\n", fd, regAddr);
    }
    #endif

    //if (ioctl(fd, I2C_SLAVE, addr) < 0){
    if(ioctl(fd, I2C_RDWR, &packets) < 0){
        printf("Error: failed to acquire bus access and/or talk to slave during write.\n");
        goto err_exit;
    }

   /* if ( (write(fd, wrBuf, num) ) != num ){
        printf("Error: failed to write to i2c bus.\n");
    }
    else{
        printf("Info:I2C write data success: %s\n", wrBuf) */


    if (fd){
        close(fd);
    }

    return 0;

err_exit:

    if (fd){
        close(fd);
    }
    return -1;

}



int iicWrByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    if( iicWrBytes(devAddr, regAddr, &data, 1) < 0){
        printf("Error: in <iicWrByte>. devAddr: %d, regAddr: %d. \n ", devAddr, regAddr);
        return -1;
    }
    else{
        return 0;
    }

}


uint8_t iicRdBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t len)
{

    return 0;
}


int iicWrBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t len, uint8_t data)
{

    return -1;

}


