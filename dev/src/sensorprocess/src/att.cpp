/*  c driver for attitude module(mpu6050) of RTP project. use X axis as "roll"
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2017.3.1  init version for Raspberry 2B(bcm2836) by Nick Qian
    ----
    output: attitude info inquiried by sens.py
    input:  attitude module(i2c device 0x68)
*/

#include <fcntl.h>                        // usleep
#include <math.h>

#include <ros/ros.h>
//#include <ros/param.h>
#include <sensor_msgs/Imu.h>       // $ROS_SETUP_FOLDER/share/sensor_msgs/msg/Imu.msg   http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

//#include <actprocess/bc_common.h>
#include "iic.h"
#include "kfilter.h"
#include "att.h"

using namespace std;


// Note: need gcc 5.x or above
gyro_reg_s reg_init = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A,
    .prod_id        = 0x0C,
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B,
    .accel_cfg      = 0x1C,
    .motion_thr     = 0x1F,
    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3A,
    .pwr_mgmt_1     = 0x6B,
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6F,
    .accel_offs     = 0x06,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6D,
    .mem_start_addr = 0x6E,
    .prgm_start_h   = 0x70
};

const hw_s hw_init={
    .addr           = 0x68,    // 0xD0
    .max_fifo       = 1024,
    .num_reg        = 118,
    .temp_sens      = 340,
    .temp_offset    = -521,
    .bank_size      = 256
};


const test_s test_init = {
    .gyro_sens      = 32768/250,
    .accel_sens     = 32768/16,
    .reg_rate_div   = 0,    /* 1kHz. */
    .reg_lpf        = 1,    /* 188Hz. */
    .reg_gyro_fsr   = 0,    /* 250dps. */
    .reg_accel_fsr  = 0x18, /* 16g. */
    .wait_ms        = 50,
    .packet_thresh  = 5,    /* 5% */
    .min_dps        = 10.f,
    .max_dps        = 105.f,
    .max_gyro_var   = 0.14f,
    .min_g          = 0.3f,
    .max_g          = 0.95f,
    .max_accel_var  = 0.14f
};


static gyro_state_s st = {
    .reg = &reg_init,
    .hw = &hw_init,
     //.chip_cfg;
    .test = &test_init
};


/*--------------------- kalman filter instances and IMU data ---------------------------*/

static Kfilter  kf_x(0.001, 0.003, 0.03);   //cfg_wfw.Qangle, cfg_wfw.Qbias, cfg_wfw.Rmeasure);   // (double cfg_Q_angle, double cfg_Q_bias, double cfg_R_measure)
static Kfilter  kf_y(0.001, 0.003, 0.03);   //cfg_wfw.Qangle, cfg_wfw.Qbias, cfg_wfw.Rmeasure);
static Kfilter  kf_z(0.001, 0.003, 0.03);

static uint32_t kalmanTimer;

static Accel_Gyro AccGyroRaw;
static Att_Data  Imu; /*  ={
    .gyroXrate  =0.f,
    .gyroYrate  =0.f,
    .gyroZrate  =0.f,

    .gyroXangle =0.f,
    .gyroYangle =0.f,
    .gyroZangle =0.f,

    .roll     =0.f,        // X axis of mpu6050
    .pitch    =0.f,        // Y axis
    .yaw      =0.f,        // Z

    .kalAngleX  =0.f,
    .kalAngleY  =0.f,        // pitch
    .kalAngleZ  =0.f
};  */

static double gyroXzero,   gyroYzero,   gyroZzero;
static double accXzero =0, accYzero =0, accZzero=0;

/*----------------------------- funcs --------------------------------------------------*/

void Init_Att( )
{
    uint8_t pBuffer[6] = {2};

    printf ("----------Start <Init_Att> ...------------------ \n");
    iicWrByte(I2C_DEV_ADDR_ATT, PWR_MGMT_1,   0x80);            // reset
    usleep(100*1000);                                            // 100ms
    iicWrByte(I2C_DEV_ADDR_ATT, PWR_MGMT_1,   0x00);            // wakeup

    iicWrByte(I2C_DEV_ADDR_ATT, SMPLRT_DIV,   0x00);            // sample rate 8khz/1khz
    iicWrByte(I2C_DEV_ADDR_ATT, CONFIG,       0x00);            // LPF:256Hz

    iicWrByte(I2C_DEV_ADDR_ATT, ACCEL_CONFIG, 0x00);            // Accel +/- 2g
    iicWrByte(I2C_DEV_ADDR_ATT, GYRO_CONFIG,  GYRO_250_DIV);    // +/- 250 degree/s
    iicWrByte(I2C_DEV_ADDR_ATT, INT_ENABLE,   0x00);            // disable all interrupt
    iicWrByte(I2C_DEV_ADDR_ATT, USER_CTRL,    0x00);            // disable I2C master-slave mode
    iicWrByte(I2C_DEV_ADDR_ATT, FIFO_EN,      0x00);            // disable FIFO
    iicWrByte(I2C_DEV_ADDR_ATT, INT_PIN_CFG,  0x80);            // INT pin active low

    if( WHO_AM_I_VAL_MPU6050 == iicRdByte(I2C_DEV_ADDR_ATT, WHO_AM_I) ){

        iicWrByte(I2C_DEV_ADDR_ATT, PWR_MGMT_1, 0x01);         // set CLKSEL, PLL X
        iicWrByte(I2C_DEV_ADDR_ATT, PWR_MGMT_2, 0x00);         // both accel and gyro

        printf("---- MPU6050 init set done. ----- \n");
    }
    else{
        printf("Error: MPU6050 Init fail. <WHO AM I> is: %d.  \n ", iicRdByte(I2C_DEV_ADDR_ATT, WHO_AM_I));
    }


    //// kfilter ////
    ros::param::get("att/kfilter/Q_angle",   kf_x.Q_angle);
    ros::param::get("att/kfilter/Q_bias",    kf_x.Q_bias);
    ros::param::get("att/kfilter/R_measure", kf_x.R_measure);
    //kf_x.setRmeasure();

    ros::param::get("att/kfilter/Q_angle",   kf_y.Q_angle);
    ros::param::get("att/kfilter/Q_bias",    kf_y.Q_bias);
    ros::param::get("att/kfilter/R_measure", kf_y.R_measure);

    printf ("Ros Info: get param \"att/kfilter/Q_angle|Q_bias|R_measure\" is: %f|%f|%f. \n", kf_x.Q_angle, kf_x.Q_bias, kf_x.R_measure);


    // first read & update kalman filter & set starting angle
    updateIMUValues( &AccGyroRaw);

    // init kalman filter input & outpt & gyroXangle
    kf_x.setAngle(Imu.roll);                // first roll
    Imu.gyroXangle = Imu.roll;
    Imu.kalAngleX  = Imu.roll;

    kf_y.setAngle(Imu.pitch);               // then pitch
    Imu.gyroYangle = Imu.pitch;
    Imu.kalAngleY  = Imu.pitch;

    kf_z.setAngle(Imu.yaw);                // finally yaw
    Imu.gyroZangle = Imu.yaw;
    Imu.kalAngleZ  = Imu.yaw;

    // calibrate gyro Zero value
    while(calibrateGyro() );

    printf("------------------ MPU6050 init done. ------------------ \n");
}


// determine the gyroXzero
bool calibrateGyro()
{
    int16_t gyroXbuffer[25], gyroYbuffer[25], gyroZbuffer[25];
    for (int i=0; i<25; i++){
        attGetAccel( &AccGyroRaw);
        gyroXbuffer[i] = AccGyroRaw.gyroX;
        gyroYbuffer[i] = AccGyroRaw.gyroY;
        gyroZbuffer[i] = AccGyroRaw.gyroZ;
    }

    if(!checkMinMax(gyroXbuffer, 25, 2000) || !checkMinMax(gyroYbuffer, 25, 2000) || !checkMinMax(gyroZbuffer, 25, 2000)){
        printf("Error: Gyro calibration error. Exceed min-max value \n");
        return 1;
    }

    for (int i=0; i<25; i++){
        gyroXzero += gyroXbuffer[i];
        gyroYzero += gyroYbuffer[i];
        gyroZzero += gyroZbuffer[i];
    }

    gyroXzero /= 25;
    gyroYzero /= 25;
    gyroZzero /= 25;
    printf("<calibrateGyro> result: gyroX/Y/Zzero %f, %f, %f \n", gyroXzero, gyroYzero, gyroZzero);

    return 0;
}


bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference)
{
    int16_t min = array[0], max = array[0];
    for (int i=1; i<length; i++){
        if(array[i] < min)
            min = array[i];
        else if(array[i] > max)
            max = array[i];
    }

    return max - min < maxDifference;
}


/*****************************************************************
/*  Raw data I2C read
/*
/****************************************************************/

int attGetAccel(Accel_Gyro *ptrAccGyroRaw)
{
    uint8_t buffer[6];

    if( iicRdBytes(st.hw->addr, st.reg->raw_accel, buffer, 6) < 0 ){
        printf("Error: during <attGetAccel>. \n");
        return -1;
    }

    //printf("-> <attGetAccel>: int16_t accel_x: %d, accel_y: %d, accel_z: %d \n", (buffer[0] << 8) | buffer[1], (buffer[2] << 8) | buffer[3], (buffer[4] << 8) | buffer[5] );

    ptrAccGyroRaw->accX = int16_t( (buffer[0] << 8) | buffer[1]);
    ptrAccGyroRaw->accY = int16_t( (buffer[2] << 8) | buffer[3]);
    ptrAccGyroRaw->accZ = int16_t( (buffer[4] << 8) | buffer[5]);

    return 0;

}


int attGetGyro(Accel_Gyro *ptrAccGyroRaw)
{
    uint8_t buffer[6];

    if( iicRdBytes(st.hw->addr, st.reg->raw_gyro, buffer, 6) < 0 ){
        printf("Error: during <attGetGyro>. \n");
        return -1;
    }


    ptrAccGyroRaw->gyroX = int16_t( (buffer[0] << 8) | buffer[1] );
    ptrAccGyroRaw->gyroY = int16_t( (buffer[2] << 8) | buffer[3] );
    ptrAccGyroRaw->gyroZ = int16_t( (buffer[4] << 8) | buffer[5] );

    return 0;

}


/////////////////////// alternative method ///////////////////////////////
static int16_t readattRaw(uint8_t regAddr)
{
    uint8_t buffer[2];

    if( iicRdBytes(I2C_DEV_ADDR_ATT, regAddr, buffer, 2) <0 ){
        printf("Error: during <readattRaw>.  \n");
    }

    return (int16_t)(buffer[0]<<8 | buffer[1]);

}


int16_t readAccel_x( ){ return readattRaw(ACCEL_XOUT_H); }
int16_t readAccel_y( ){ return readattRaw(ACCEL_YOUT_H); }
int16_t readAccel_z( ){ return readattRaw(ACCEL_ZOUT_H); }
int16_t readGyro_x( ) { return readattRaw(GYRO_XOUT_H);  }
int16_t readGyro_y( ) { return readattRaw(GYRO_YOUT_H);  }
int16_t readGyro_z( ) { return readattRaw(GYRO_ZOUT_H);  }


/***************************************************************
/* Raw data process
/*
/**************************************************************/

void updateIMUValues(Accel_Gyro *ptrAccGyroRaw)
{
    attGetGyro(ptrAccGyroRaw);
    attGetAccel(ptrAccGyroRaw);

    //double accYzero, accZzero;

    ros::param::get("bc/wfw/accXzero", accXzero);
    ros::param::get("bc/wfw/accYzero", accYzero);
    ros::param::get("bc/wfw/accYzero", accZzero);

    Imu.roll  = calcRoll (&AccGyroRaw);
    Imu.pitch = calcPitch(&AccGyroRaw);
    printf ("--------------------- acc:( %d, %d, %d), gyro:(%d, %d, %d)------------------------- \n", \
      AccGyroRaw.accX,AccGyroRaw.accY,AccGyroRaw.accZ,  AccGyroRaw.gyroX,AccGyroRaw.gyroY,AccGyroRaw.gyroZ);

    printf ("--- roll:%f, pitch:%f--- \n",  Imu.roll, Imu.pitch);

}


//www.nxp.com/docs/en/application-note/AN3461.pdf
double calcRoll(Accel_Gyro *ptrAccGyroRaw)
{
    double roll = RAD_TO_DEG * ( atan2( (double)ptrAccGyroRaw->accY - accYzero,  (double)ptrAccGyroRaw->accZ - accZzero ) );
    return roll;
}



double calcPitch(Accel_Gyro *ptrAccGyroRaw)
{
    double pitch = atan(- ptrAccGyroRaw->accX / ( sqrt \
                           (ptrAccGyroRaw->accY * ptrAccGyroRaw->accY + ptrAccGyroRaw->accZ * ptrAccGyroRaw->accZ ) \
                                                 ) )* RAD_TO_DEG;
    return pitch;
}



double calcYaw(double kalAngleX, double kalAngleY)
{
    double yaw;




    return yaw;
}


double cvt_to_rate(double gyroRaw)
{
    return gyroRaw/131.0;
}

/***************************************************************
/* Read raw data & filt it
/*
/***************************************************************/
void updateAngle(ros::Time att_time)
{
    uint64_t timer = ros::Time::now().toNSec();
    uint64_t att_time_ns = att_time.toNSec();

    updateIMUValues( &AccGyroRaw);

    /////// fix -180 to 180 transition problem when accel angle jumps between -180 and 180 (0-360)
    //if ( (Imu.pitch < -90 && Imu.kalAngleY > 90) || (Imu.pitch > 90 && Imu.kalAngleY < -90) ){
    if ( (Imu.roll < 90 && Imu.kalAngleX > 270) || (Imu.roll > 270 && Imu.kalAngleX < 90) ){
        kf_x.setAngle(Imu.roll);
        Imu.kalAngleX = Imu.roll;
        Imu.gyroXangle = Imu.roll;
        printf("Warning:<updateAngle>: jump between >180 and <-180. pitch:%f, kalAngleY:%f \n", Imu.pitch, Imu.kalAngleY );
    }
    else{
        Imu.gyroXrate = cvt_to_rate(  (double)AccGyroRaw.gyroX - gyroXzero );
        Imu.gyroYrate = cvt_to_rate(  (double)AccGyroRaw.gyroY - gyroYzero );
        Imu.gyroZrate = cvt_to_rate(  (double)AccGyroRaw.gyroZ - gyroZzero );

        double dt = (double)(timer - att_time_ns) / 1000000.0;
        printf("---dt:%f,gyroXrate:%f,gyroYrate:%f \n",dt, Imu.gyroXrate, Imu.gyroYrate);

        Imu.kalAngleX = kf_x.getAngle(Imu.roll,  Imu.gyroXrate, dt);          // apply the filter
        if(abs(Imu.kalAngleX) > 90){
            Imu.gyroYrate = -Imu.gyroYrate;
            printf("#Note: Invert Y rate to calc kalAngleY. \n");
        }
        Imu.kalAngleY = kf_y.getAngle(Imu.pitch, Imu.gyroYrate, dt);

        // Gyro angle is only for debugging
        Imu.gyroXangle += Imu.gyroXrate * dt;
        Imu.gyroYangle += Imu.gyroYrate * dt;
        Imu.gyroZangle += Imu.gyroZrate * dt;

        // reset when drifted too much
        if(Imu.gyroXangle < -180 || Imu.gyroXangle > 180){   // <0  >360
            Imu.gyroXangle = Imu.kalAngleX;
            printf("Warning: gyroXangle Drift too much. reset it use kalman filter result. \n");
        }
        if(Imu.gyroYangle < -180 || Imu.gyroYangle > 180)
            Imu.gyroYangle = Imu.kalAngleY;
        if(Imu.gyroZangle < -180 || Imu.gyroZangle > 180)
            Imu.gyroZangle = Imu.kalAngleZ;

        printf( " Imu.gyroX/Y/Zrate:(%f %f %f), gyroX/Y/Zangle: (%f %f %f) \n", Imu.gyroXrate, Imu.gyroYrate, Imu.gyroZrate, \
                                  Imu.gyroXangle, Imu.gyroYangle, Imu.gyroZangle);


        printf( "<kf>:.kalAngleX: %f, kalAngleY: %f \n", Imu.kalAngleX, Imu.kalAngleY);
    }

    kalmanTimer = timer;

}


/***************************************************************
/* ROS Pub
/*
/***************************************************************/

int main(int argc, char **argv)
{
    sensor_msgs::Imu imu_msg = sensor_msgs::Imu();

    ros::init(argc, argv, "node_sens_att");
    ros::NodeHandle n("~");

    ros::Publisher att_pub = n.advertise<sensor_msgs::Imu>("tpc_sens_att", 10);

    ros::Rate loop_rate(RATE_IMU_HZ);

    ros::Time att_time = ros::Time::now();
    ros::Time att_time_last;

    // Init device
    Init_Att( );


    while(ros::ok()){

        updateAngle(att_time_last);
        att_time_last = att_time.now();

        imu_msg.header.stamp = att_time.now();
        imu_msg.header.frame_id = "imu";


        imu_msg.orientation.x = Imu.kalAngleX;
        imu_msg.orientation.y = Imu.kalAngleY;
        imu_msg.orientation.z = Imu.kalAngleZ;
        imu_msg.orientation.w = Imu.pitch;              // Debug
        imu_msg.angular_velocity.x = Imu.roll;          // debug
        imu_msg.angular_velocity.y = Imu.pitch;         // debug
        imu_msg.angular_velocity.z = Imu.yaw;           // debug
        imu_msg.linear_acceleration.x = Imu.gyroXangle;   // debug
        imu_msg.linear_acceleration.y = Imu.gyroYangle;   // debug
        imu_msg.linear_acceleration.z = Imu.gyroZangle;   // debug

       // publish it
        att_pub.publish(imu_msg);  //&imu_msg);

        ros::spinOnce();
        loop_rate.sleep();

     }

     return 0;
}

