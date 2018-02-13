/*  c driver for attitude module(mpu6050) of RTP project
    ----
    Licensed under BSD license.
    ----
    0.1 - 2017.3.12  init version for Raspberry 2B(bcm2836) -- Nick Qian
    ----
    output: attitude info inquiried by sens.py
    input: module(i2c device 0x68)
*/

#ifndef _ATT_H_
   #define _ATT_H_


#define RATE_IMU_HZ    20     //500 Hz

#define USE_MPU6050

// Regs Address
#ifdef USE_MPU6050
  #pragma message("@Compile MSG@: USING MPU6050 as attitude sensor")

///////////// Address //////////////
#define MPU6050_ADDR        0x68  //0xD0
#define I2C_DEV_ADDR_ATT   MPU6050_ADDR

#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN      0x23
#define INT_PIN_CFG  0x37
#define INT_ENABLE   0x38

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define SIGNAL_PATH_RESET 0x68     //RESET

#define USER_CTRL    0x6A          //RESET
#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C

#define FIFO_COUNTH  0x72
#define FIFO_COUNTL  0x73
#define FIFO_R_W     0x74

#define WHO_AM_I     0x75
#define WHO_AM_I_VAL_MPU6050    0x68

////////////// Values ////////////////////
#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

#define GYRO_250_DIV  0x00
#define GYRO_500_DIV  0x08
#define GYRO_1000_DIV 0x10
#define GYRO_2000_DIV 0x18


#define RAD_TO_DEG   57.2956f
#define DEG_TO_RAD   0.0174533f
#define PI           3.1415926
#define ACC_G_ZERO       16384


enum gyro_fsr_e{                    // full sccale range
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum accel_fsr_e{
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

enum lp_accel_rate_e{              // low power accel wakeup rates
    INV_LPA_1_25HZ,
    INV_LPA_5HZ,
    INV_LPA_20HZ,
    INV_LPA_40HZ
};

#define BIT_FIFO_EN  0x40


/********* Gyro driver state variables************/
struct gyro_reg_s {
    uint8_t who_am_i;
    uint8_t rate_div;
    uint8_t lpf;
    uint8_t prod_id;
    uint8_t user_ctrl;
    uint8_t fifo_en;
    uint8_t gyro_cfg;
    uint8_t accel_cfg;
    // uint8_t accel_cfg2;
    // uint8_t lp_accel_odr;
    uint8_t motion_thr;
    uint8_t motion_dur;
    uint8_t fifo_count_h;
    uint8_t fifo_r_w;
    uint8_t raw_gyro;
    uint8_t raw_accel;
    uint8_t temp;
    uint8_t int_enable;
    uint8_t dmp_int_status;
    uint8_t int_status;
    // uint8_t accel_intel;
    uint8_t pwr_mgmt_1;
    uint8_t pwr_mgmt_2;
    uint8_t int_pin_cfg;
    uint8_t mem_r_w;
    uint8_t accel_offs;
    uint8_t i2c_mst;
    uint8_t bank_sel;
    uint8_t mem_start_addr;
    uint8_t prgm_start_h;
};

struct hw_s {
    uint8_t  addr;
    uint16_t max_fifo;
    uint8_t  num_reg;
    uint16_t temp_sens;
    int16_t  temp_offset;
    uint16_t bank_size;
};

struct motion_int_cache_s {
    uint16_t gyro_fsr;
    uint8_t  accel_fsr;
    uint16_t lpf;
    uint16_t sample_rate;
    uint8_t  sensors_on;
    uint8_t  fifo_sensors;
    uint8_t  dmp_on;
};


struct chip_cfg_s{
    uint8_t gyro_fsr;
    uint8_t accel_fsr;
    uint8_t sensors;
    uint8_t lpf;
    uint8_t clk_src;
    uint8_t sample_rate;
    uint8_t fifo_enable;
    uint8_t int_enable;
    uint8_t bypass_mode;
    uint8_t accel_half;
    uint8_t lp_accel_mode;
    uint8_t int_motion_only;

    struct motion_int_cache_s cache;
    uint8_t active_low_int;
    uint8_t latched_int;
    uint8_t dmp_on;
    uint8_t dmp_loaded;
    uint16_t dmp_sample_rate;
};

struct test_s{
    uint32_t gyro_sens;
    uint32_t accel_sens;
    uint8_t reg_rate_div;
    uint8_t reg_lpf;
    uint8_t reg_gyro_fsr;
    uint8_t reg_accel_fsr;
    uint8_t wait_ms;
    uint8_t packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var; 
};

struct gyro_state_s{
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
//  struct chip_cfg_s chip_cfg;
    const struct test_s *test;
};


#endif      // USE_MPU6050


struct Att_Data{
    double roll;
    double pitch;
    double yaw;

    double kalAngleX;
    double kalAngleY;
    double kalAngleZ;

    double temperature;

    // below for debug
    double gyroXangle;
    double gyroYangle;
    double gyroZangle;

    double gyroXrate;
    double gyroYrate;
    double gyroZrate;
};


struct Accel_Gyro{
    int16_t accX;
    int16_t accY;
    int16_t accZ;

    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
};



/********************** Interface ********************/
//extern SIX_AXIS six_axis;
//extern double accAngle, gyroAngle, pitch;

void Init_Att();
void updateAngle();




/****************** Internal Func ************************************/
static int16_t readAttRaw(uint8_t regAddr);

int attGetAccel(Accel_Gyro *ptrAccGyroRaw);
int attGetGyro(Accel_Gyro *ptrAccGyroRaw);

void updateIMUValues(Accel_Gyro *ptrAccGyroRaw);

bool calibrateGyro();
bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);
double calcPitch(Accel_Gyro *ptrAccGyroRaw);
double calcRoll(Accel_Gyro *ptrAccGyroRaw);

///////////// aternative method //////////// 
int16_t readAccel_x();
int16_t readAccel_y();
int16_t readAccel_z();

int16_t readGyro_x();
int16_t readGyro_y();
int16_t readGyro_z();



#endif
