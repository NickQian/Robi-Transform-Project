/*
    c driver for compass module of RTP
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2017.3.3  init version for Raspberry 2B(bcm2836)
    ----
    output: compass info inquiried by sens.py
    input: modue with HMC5883L
*/

#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include "iic.h"
#include "compass.h"


using namespace std;


int16_t magX, magY, magZ;
float   magGain[3] = {0.0};
int16_t offsetX, offsetY, offsetZ;



void compassInit()
{

    printf ("----------------- start <compassInit> .---------------------- \n");
    // setRange(COMPASS_RANGE_1_3GA);
    // setMeasurementMode(COMPASS_CONTINOUS);
    // setDataRate(COMPASS_DATARATE_15HZ);
    // setSamples(COMPASS_SAMPLES_1)
    // mgPerDigit = 0.92f;

    uint8_t tmp = COMPASS_CONTINOUS;

    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_MODE_SEL, tmp);

    usleep(100*1000);

    calibrateMag();

    getCompassOffset();

    printf ("----------------- <compassInit> done.---------------------- \n");

}



int compassUpdate()
{
    uint8_t buffer[6] = {0};

    if(iicRdBytes(I2C_DEV_ADDR_COMPASS, MAG_DAT_ADDR_START, buffer, 6) < 0){
        printf("Error: I2C Read error occur in <compassUpdate>. \n");
        return -1;
    }

    magX = int16_t( (buffer[0] << 8 ) | buffer[1]);
    magZ = int16_t( (buffer[2] << 8 ) | buffer[3]);
    magY = int16_t( (buffer[4] << 8 ) | buffer[5]);

    printf ("Info: <compassUpdate>: magX= %d, magY= %d, magZ= %d \n", magX, magY, magZ);

    return 0;

}



void calibrateMag()
{
    // http://blog.sina.com.cn/s/blog_402c071e0102v8ie.html

    int16_t magPosOff[3] = {0,0,0};
    int16_t magNegOff[3] = {0,0,0};

    uint8_t tmp =  (COMPASS_DATARATE_15HZ << 2) | CFG_MODE_POS_OFFSET;

    printf ("----------start <calibrateMAg> ..... ------------- \n ");

    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_A, tmp);

    usleep(100*1000);

    compassUpdate();

    magPosOff[0] = magX;
    magPosOff[1] = magY;
    magPosOff[2] = magZ;

    tmp =  (COMPASS_DATARATE_15HZ << 2) | CFG_MODE_NEG_OFFSET;
    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_A, tmp);

    usleep(100*1000);
    compassUpdate();

    magPosOff[0] = magX;
    magPosOff[1] = magY;
    magPosOff[2] = magZ;

    tmp =  (COMPASS_DATARATE_15HZ << 2) | CFG_MODE_NORMAL;
    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_A, tmp);

    magGain[0] = -2500 / (float)(magNegOff[0] - magPosOff[0]);
    magGain[1] = -2500 / (float)(magNegOff[1] - magPosOff[1]);
    magGain[2] = -2500 / (float)(magNegOff[2] - magPosOff[2]);

    printf ("Info: <calibrateMAg> done: Gain X: %8.3f, Y: %8.3f, Z: %8.3f \n", magGain[0], magGain[1], magGain[2] );

}


void getCompassOffset()
{
    int xMax, xMin, yMax, yMin, zMax, zMin;

    compassUpdate();

    xMax = xMin = magX;
    yMax = yMin = magY;
    zMax = zMin = magZ;

    offsetX = offsetY = offsetZ = 0;

    printf("Info: Compass start calibrating...turn around the compass by 360 degree in 20sec... \n");

    for (int i=0; i<200; i++){
        compassUpdate();

        // calculate the maximam mag
        if (magX > xMax)
            xMax = magX;
        if (magX < xMin)
            xMin = magX;

        if (magX > xMax)
            xMax = magX;
        if (magX < xMin)
            xMin = magX;

        if (magX > xMax)
            xMax = magX;
        if (magX < xMin)
            xMin = magX;

        usleep(100*1000);
    }

    if(abs(xMax - xMin) > CAL_THRESHOLD)
        offsetX = (xMax + xMin)/2;
    if(abs(yMax - yMin) > CAL_THRESHOLD)
        offsetY = (yMax + yMin)/2;
    if(abs(zMax - zMin) > CAL_THRESHOLD)
        offsetZ = (zMax + zMin)/2;

}


int calcCompassHeading(int *x, int *y, int *z)
{
    int headingDegrees;
    float headingRadians = atan2((double)((*y)-offsetY),(double)((*x)-offsetX));

    //
    if(headingRadians < 0)
        headingRadians += 2*PI;

    headingDegrees = headingRadians * 180/PI;
    headingDegrees += MAGNETC_DECLINATION;

    //
    if(headingDegrees > 360)
        headingDegrees -= 360;

    return headingDegrees;

}


void setCompassMeasureMode(compass_mode_t mode)
{
    uint8_t d;

    d &= 0;                    // no need to read. Just clear all
    d |= mode;

    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_MODE_SEL, d);

}


void setCompassDataRate(compass_dataRate_t dataRate)
{
    uint8_t d;

    d = iicRdByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_A);
    d &= 0b11100011;                    // [D02,D01,D00]
    d |= (dataRate << 2);

    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_A, d);

}


void setCompassSamples(compass_samples_t samples)
{
    uint8_t d;

    d = iicRdByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_A);
    d &= 0b10011111;                    // [D02,D01,D00]
    d |= (samples << 5);

    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_A, d);

}


void setCompassRange(compass_range_t range)
{
    uint8_t d;

    d &= 0;                                // clear all
    d |= (range << 5);

    iicWrByte(I2C_DEV_ADDR_COMPASS, ADDR_CFG_REG_B, d);
}


/*************************************************************************
/* ROS Pub
/*
/************************************************************************/

int main(int argc, char **argv)
{
    sensor_msgs::MagneticField cmps_msg = sensor_msgs::MagneticField();

    ros::init(argc, argv, "node_sens_cmps");
    ros::NodeHandle n;

    ros::Publisher cmps_pub = n.advertise<sensor_msgs::MagneticField>("tpc_sens_cmps", 10);

    ros::Rate loop_rate(PUB_RATE_CMPS_HZ);

    ros::Time cmps_time = ros::Time::now();

    // Init firstly
    compassInit();

    while(ros::ok()){

        compassUpdate();

        // fill msg
        cmps_msg.header.stamp = ros::Time::now();
        cmps_msg.header.frame_id = "cmps";

        cmps_msg.magnetic_field.x = (double)magX;            //float64
        cmps_msg.magnetic_field.y = (double)magY;
        cmps_msg.magnetic_field.z = (double)magZ;

        cmps_msg.magnetic_field_covariance = {0};          // float64[9]

        // publish it
        cmps_pub.publish(cmps_msg);

        // SpinOnce and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}









