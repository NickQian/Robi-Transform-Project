#include <stdint.h>           // C99 int8_t, uint8_t, int16_t, uint16_t;  int32_t; uint32_t int64_t;  uint64_t;

//ROM
#define ADDR_MODEL_L         0x00
#define ADDR_MODEL_H         0x01
#define ADDR_FIRMWARE        0x02
#define ADDR_SERVO_ID        0x04
//Flash ROM(Variable ROM). memory map from No.4 to No.29            
#define ADDR_BAUD_RATE       0x06
#define ADDR_RETURN_DLY      0x07
#define ADDR_CW_LMT_L        0x08         // clockwise
#define ADDR_CW_LMT_H        0x09
#define ADDR_CCW_LMT_L       0x0A         // counter clockwise
#define ADDR_CCW_LMT_H       0x0B
#define ADDR_TMPT_LMT_L      0x0E         // exceeds the set value, the servo will automatically turns off its torque
#define ADDR_TMPT_LMT_H      0x0F
#define ADDR_TORQUE_SILN     0x16
#define ADDR_WARMUP_TIME     0x17
#define ADDR_CW_CPLN_MRN     0x18          // CW compliance margin
#define ADDR_CCW_CPLN_MRN    0x19
#define ADDR_CW_CPLN_SLOPE   0x1A
#define ADDR_CCW_CPLN_SLOPE  0x1B
#define ADDR_PUNCH_L         0x1C
#define ADDR_PUNCH_H         0x1D
//RAM
#define ADDR_GOAL_POS_L      0x1E
#define ADDR_GOAL_POS_H      0x1F
#define ADDR_GOAL_TIME_L     0x20
#define ADDR_GOAL_TIME_H     0x21
#define ADDR_MAX_TORQUE      0x23
#define ADDR_TORQUE_EN       0x24
#define ADDR_PRESENT_POS_L   0x2A          // Read Only 
#define ADDR_PRESENT_POS_H   0x2B          // Read Only 
#define ADDR_PRESENT_TIM_L   0x2C          // Read Only 
#define ADDR_PRESENT_TIM_H   0x2D          // Read Only 
#define ADDR_PRESENT_SPD_L   0x2E          // Read Only 
#define ADDR_PRESENT_SPD_H   0x2F          // Read Only 
#define ADDR_PRESENT_CURNT_L 0x30          // Read Only 
#define ADDR_PRESENT_CURNT_H 0x31          // Read Only 
#define ADDR_PRESENT_TMPR_L  0x32          // Read Only 
#define ADDR_PRESENT_TMPR_H  0x33          // Read Only 
#define ADDR_PRESENT_VOLT_L  0x34          // Read Only 
#define ADDR_PRESENT_VOLT_H  0x35          // Read Only 


enum ServoBaudRate{
    BRATE_9600   = 0x00, 
    BRATE_14400  = 0x01,
    BRATE_19200  = 0x02,
    BRATE_28800  = 0x03,
    BRATE_38400  = 0x04,
    BRATE_57600  = 0x05,
    BRATE_76800  = 0x06,
    BRATE_115200 = 0x07,        // default
    BRATE_153600 = 0x08,
    BRATE_230400 = 0x09
};

typedef enum {
   Flg_WriteServoRAM     = 0x00,
   Flg_WriteFlashROM     = 0x40,
   Flg_RebootServo       = 0x20,
   Flg_InitMemMap4_29    = 0x10,
   Flg_ReturnACK_NACK    = 0x01,
   Flg_ReturnMemMap0_29  = 0x03,
   Flg_ReturnMemMap30_59 = 0x05,
   Flg_ReturnMemMap20_29 = 0x07,
   Flg_ReturnMemMap42_59 = 0x09,
   Flg_ReturnMemMap30_41 = 0x0B,
   Flg_ReturnMemMapSpec  = 0x0F
}ServoFlgs;

#define      Flg_PacketReturned  0x00 

#define      OneByte    8


//-----------------------Servo Write Packet-------------------------------------
////////////Short Packet////////////

#pragma pack(1)
//typedef
struct Futaba_ShortPacket {
uint16_t   Header;
uint8_t    ID;
uint8_t    Flg;         // active 1. //[6]: write Flash Rom //[5]: Reboot Servo. //[4]: Init memory map 4~29 data to default //[0:3]: Direct Addr of return packet.
uint8_t    Adr;         // address of the data
uint8_t    Len;
uint8_t    Cnt;         // Number of servos
 int16_t   Dat[2];      // signed
uint8_t    Sum;
};


//typedef 
struct Futaba_ShortPacketCmd {
uint16_t   Header;
uint8_t    ID;
uint8_t    Flg;        //[0:3]: Direct Addr of return packet. [4]Init memory map data. [5]:Reboot Servo. [6]: write Flash Rom
uint8_t    Adr;
uint8_t    Len;
uint8_t    Cnt;
uint8_t    Sum;
};

/*
struct Futaba_ShortPacket2 {
uint16_t   Header;
uint8_t    ID;
uint8_t    Flg;         // active 1. //[6]: write Flash Rom //[5]: Reboot Servo. //[4]: Init memory map 4~29 data to default //[0:3]: Direct Addr of return packet.
uint8_t    Adr;         // address of the data
uint8_t    Len;
uint8_t    Cnt;         // Number of servos
 int16_t   Dat;         // signed
uint8_t    Sum;
};

*/
//////////////Long packet//////////////////

//typedef
struct Futaba_VID_Data {
uint8_t    VID;         // eg. 0x02 
 int16_t   Data;        // eg. 0x6400
} ;

typedef struct  {
uint16_t   Header;
uint8_t    ID;
uint8_t    Flg;                      //[0:3]: Direct Addr of return packet. [4]Init memory map data. [5]:Reboot Servo. [6]: write Flash Rom
uint8_t    Adr;
uint8_t    Len;                      // len of data for each servo
uint8_t    Cnt;
struct Futaba_VID_Data   Futaba_VID_Data[20];      // maximum
uint8_t    Sum;
} Futaba_LongPacket;

//--------------------------Servo Read packet-------------------------

//typedef
struct Futaba_ReturnPacket {
uint16_t   Header;
uint8_t    ID;
uint8_t    Flg;                      //[0:3]: Direct Addr of return packet. [4]Init memory map data. [5]:Reboot Servo. [6]: write Flash Rom
//[7]: Error Temperature Limit (Torque OFF)
//[5]: Error Temperature Alarm
//[3]: Error Write Flash ROM
//[1]: Error Received Packet
uint8_t    Adr;
uint8_t    Len;                      // len of data for each servo
uint8_t    Cnt;
uint16_t   Futaba_VID_Data[12];      // maximum
uint8_t    Sum;
} ;

#pragma pack()


void servoConfig();
uint8_t servoSet(uint8_t servoID, int16_t goalPostion, uint16_t goalTime);
void uart0_WrServoShortPacket(Futaba_ShortPacket *shortPacketWr);
void servoSetTorque(Futaba_ShortPacket *shortPacketWr, int16_t value);
int servoSetID( int16_t id);

void PWM_Ser_WriteServoShortPacket(Futaba_ShortPacket *shortPacketWr);
void PWM_Ser_WriteFIFO(uint32_t WrData);
struct Futaba_ReturnPacket *readReturnPacket();
uint32_t PWM_Ser_ReadFIFO();

