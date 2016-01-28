
#include <iostream> 
#include <stdlib.h> 

#include "peri.h"           // the low level HW access-write pwm FIFO
#include "futabaServo.h"

using namespace std;


Futaba_ShortPacket init_Futaba_ShortPacket ={
	0xAFFA,  //0xFAAF,    // LSB first in UART. MSB->LSB in PWM Serial mode: 1111->1010->1010->1111
	0xFF,                 // id = 255 = all servos
	0x00,                 // flag. 
	0x1E,                 // addr
	0x04,                 // data len, 0x02 without time, 0x04 with time info 
	0x01,                 // Cnt, 1 for short packet
	{900,500},    //0x0384 01F4,      // data. 0x0384=900=90.0degree, 0x01F4 = 500 = 5sec
	0x90                  // ID ^ Flag ^ Addr ^ Length ^ Cnt ^ 	
};


Futaba_ReturnPacket init_Futaba_ReturnPacket =
{       0xFAAF,           //1111 1010 1010 1111
	0x01,             // id = 255 = all servos
	0x09,             // flag. 
	0x00,             // addr=0x00 -> No.42 and No.59
	0x00,             // data len, 0x02 without time, 0x04 with time info 
	0x01,             // Cnt, 1 for short packet
	0x09              // ID ^ Flag ^ Addr ^ Length ^ Cnt ^ 	
};


Futaba_ShortPacketCmd init_Futaba_ShortPacketCmd =
{       0xAFFA,   //0xFAAF,             //1111 1010 1010 1111
	0x01,             // id = 255 = all servos
	0x09,             // flag. 
	0x00,             // addr=0x00 -> No.42 and No.59
	0x00,             // data len, 0x02 without time, 0x04 with time info 
	0x01,             // Cnt, 1 for short packet
	0x09              // ID ^ Flag ^ Addr ^ Length ^ Cnt ^ 	
};


////////////////Call peripheral driver////////////

void servoConfig(){
  ServoBaudRate baudRate = BRATE_115200; 
//  PWM_Set_Clock(PWM_CLK_DIV_9K375);

}

uint8_t servoSet(uint8_t servoID, int16_t goalPostion, uint16_t goalTime){     // set value, calculate Sum
	/*  90.0 degree = 900(0384H), 5 sec = 500 (01F4H)
    Hdr   ID  Flg | Adr Len Cnt Dat |          Sum
    FA AF 01  00  | 1E  04  01  84  |03 F4 01  68     //? LSB to MSB?    */
    Futaba_ShortPacket *shortPacketWr = (Futaba_ShortPacket *) malloc(sizeof( Futaba_ShortPacket));   // C++
	/////////Assemble short packet//////////
	(*shortPacketWr).Header = init_Futaba_ShortPacket.Header;
	(*shortPacketWr).ID   = servoID;

        //// Enable torque////
        servoSetTorque( shortPacketWr, 1); 

        //// goal ////
	(*shortPacketWr).Flg  = Flg_WriteServoRAM;
        shortPacketWr->Adr    = ADDR_GOAL_POS_L;
        if (goalTime == 0){
           shortPacketWr->Len    = 2;
        }
        else {
	   shortPacketWr->Len    = 4;
        }
	shortPacketWr->Cnt    = 1;
        shortPacketWr->Dat[0] = goalPostion;                    //int16_t
        shortPacketWr->Dat[1] = goalTime;                       //uint16_t
        shortPacketWr->Sum = 0;

	///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
 	int i;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len) ); i++){    // From "ID". -1 is for SUM. -4 is for Dat
	         shortPacketWr->Sum  ^= *( p + i );
        }

        uart0_WrServoShortPacket( shortPacketWr);

  delay_ns(200);
//        while(1) {
//          if (1==UART0_STA_Check() ){
  
//            cout << "CHECK STATUS BEFORE RX READ:" << endl;
//            UART0_STA_Check();
//            int k;
//            for  (k=0; k<15; k++){   // uart0 fifo depth is 16
//              UART0_rxDataRdOneByte( );
//            } 
//            goto exit; 
//          }
//        }


exit:
	return shortPacketWr->Sum;
}


void uart0_WrServoShortPacket(Futaba_ShortPacket *shortPacketWr){
       uint8_t  k = 0;
       uint8_t *p = (uint8_t *)shortPacketWr;

        for (k=0; k< (sizeof(Futaba_ShortPacket) -1 -4 + (shortPacketWr->Len) ); k++){                   // -1 for the last byte "SUM". -4 for Dat
            UART0_Send( *(p+k) );
            cout << "$$UART sending 0x" << hex << (uint16_t) *(p+k) << ". k is " << (uint16_t) k << endl;
        }

        UART0_Send(shortPacketWr->Sum);
        cout << "$$UART sending SUM 0x" << hex << (uint16_t)shortPacketWr->Sum << endl;

   //     UART0_STA_Check();
        delay_ns(500);    cout <<"...500ms later...." << endl;
        UART0_STA_Check();
}

void servoSetTorque(Futaba_ShortPacket *shortPacketWr, int16_t value){
         shortPacketWr ->Flg = Flg_WriteServoRAM;
         shortPacketWr ->Adr = ADDR_TORQUE_EN;
         shortPacketWr ->Len = 0x01;
         shortPacketWr ->Cnt = 0x01;
         shortPacketWr ->Dat[0] = value;             // 1:enable, 0:disable, 2:brake mode

         shortPacketWr->Sum = 0;
        ///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
        int i;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len) ); i++){
                 shortPacketWr->Sum  ^= *( p + i );
        }

        uart0_WrServoShortPacket( shortPacketWr);

};

int servoSetID( int16_t id){
         Futaba_ShortPacket *shortPacketWr = (Futaba_ShortPacket *) malloc(sizeof( Futaba_ShortPacket));   // C++
        /////////Write to RAM firstly//////////
        shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
        shortPacketWr ->ID     = 0xFF; // 0xFF;                 //!! Any servo connected
        shortPacketWr ->Flg    = Flg_WriteServoRAM; //Flg_WriteFlashROM;
        shortPacketWr ->Adr    = ADDR_SERVO_ID;
        shortPacketWr ->Len    = 0x1;
        shortPacketWr ->Cnt    = 0x1;
        shortPacketWr->Dat[0]  = id;                       //uint16_t
        shortPacketWr->Sum = 0;

        ///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
        int i;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len) ); i++){    // From "ID". -1 is for SUM. -4 is for Dat
                 shortPacketWr->Sum  ^= *( p + i );
        }

        cout << "Setting Servo ID...Write to RAM firstly. id is 0x:" << hex << id << endl;

        uart0_WrServoShortPacket( shortPacketWr);


        ///////////// Write Flash command after write RAM/////////////////
        shortPacketWr ->ID     = (uint8_t)id;
        shortPacketWr ->Flg    = Flg_WriteFlashROM;
        shortPacketWr ->Adr    = 0xFF;
        shortPacketWr ->Len    = 0x0;
        shortPacketWr ->Cnt    = 0x0;
        ///Sum///
        shortPacketWr->Sum = 0;
        p = (uint8_t *)shortPacketWr;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len) ); i++){    // From "ID". -1 is for $
                 shortPacketWr->Sum  ^= *( p + i );
        } 

        uart0_WrServoShortPacket( shortPacketWr);
        cout << "Set Servo ID done. (shortPacketWr ->ID) (uint8_t) is 0x:" << hex << (uint16_t)(shortPacketWr ->ID) << endl;

       return 0;
};


void PWM_Ser_WriteServoShortPacket(Futaba_ShortPacket *shortPacketWr){
       uint8_t  k = 0;
       uint32_t *p = (uint32_t *)shortPacketWr;

        cout << "size of Futaba_shortPacket is " << sizeof(Futaba_ShortPacket) << endl;
        for (k=0; k< sizeof(Futaba_ShortPacket); k++){                   // -1 for the last byte "SUM"
            PWM_Ser_WriteFIFO( *(p+k) );
            cout << "$$UART sending 0x" << hex << (uint16_t) *(p+k) << ". k is " << (uint16_t) k << endl;
        }

        cout << "->check PWM STA read value is:0x" << hex << PWM_STA_Check() << endl;

}



//-----------------------Read Func---------------------------------
struct Futaba_ReturnPacket *readReturnPacket(){
	struct Futaba_ReturnPacket *returnPacket = (struct Futaba_ReturnPacket *) malloc( sizeof(struct Futaba_ReturnPacket) );

	return returnPacket;
}

//uint32_t PWM_Ser_ReadFIFO(){
//	uint32_t readResult;
//	readResult = *PWM_FIF1;
//
//	return readResult;
//}

int getPresentPosition(uint8_t servoID ){
	int PositionResult = 0;
	Futaba_ShortPacketCmd  shortPacket = init_Futaba_ShortPacketCmd;
	Futaba_ReturnPacket    returnPacket;
	shortPacket.ID = servoID;
	
	
	return PositionResult;
}



int getPresentTemperature(uint8_t servoID ){
	int TemptrResult = 0;
	Futaba_ShortPacketCmd  shortPacket = init_Futaba_ShortPacketCmd;
	Futaba_ReturnPacket    returnPacket;
	shortPacket.ID  = servoID;

	return TemptrResult;
}

int getPresentVoltage(uint8_t servoID){
	uint32_t VoltageResult = 0;
	
	return  VoltageResult;
}

