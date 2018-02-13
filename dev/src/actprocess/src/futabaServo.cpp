/*
    Futaba servo c driver 
    ----
    Licensed under BSD license. 
    by Nick Qian
    ----
    0.2 - 2016.9.14   add support for OS based uart0 r/w on Raspberry 3B(bcm2837)
    0.1 - 2016.1.11   init version for Raspberry 2B(bcm2836)(rd/wr uart0 directly)
    ----
*/

#include <iostream>
#include <stdlib.h>

#include "uart0.h"
#include "peri.h"       // for delay_ms()/uart define/tx pin to be input
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
	0xFF                  // ID ^ Flag ^ Addr ^ Length ^ Cnt ^
};

/*
Futaba_ReturnPacket init_Futaba_ReturnPacket =
{       0xFAAF,           //1111 1010 1010 1111
	0x01,             // id = 255 = all servos
	0x09,             // flag.
	0x00,             // addr=0x00 -> No.42 and No.59
	0x00,             // data len, 0x02 without time, 0x04 with time info 
	0x01,             // Cnt, 1 for short packet
	0xFF              // ID ^ Flag ^ Addr ^ Length ^ Cnt ^
};
*/


/// for the cmd packet init only. CMD packet is short packet with (len=0, regardless of "Cnt")
Futaba_ShortPacketCmd init_Futaba_ShortPacketCmd =
{       0xAFFA,   //0xFAAF,             //1111 1010 1010 1111
	0x01,             // id = 255 = all servos
	0x09,             // flag. 0x20 for reboot.
	0x00,             // addr=0x00 -> No.42 and No.59
	0x00,             // data len  = 00 for cmd
	0x00,             // Cnt, 1 for short packet
	0xFF              // SUM
};



 /*  90.0 degree = 900(0384H), 5 sec = 500 (01F4H)
 Hdr   ID  Flg | Adr Len Cnt Dat           Sum
 FA AF 01  00  | 1E  04  01  84  03 F4 01  68     //? LSB to MSB?    */
static Futaba_ShortPacket  *shortPacketWr  = (Futaba_ShortPacket *) malloc(sizeof( Futaba_ShortPacket));       // C++
static Futaba_ReturnPacket *returnPacketRd = (Futaba_ReturnPacket *) malloc(sizeof(Futaba_ReturnPacket));      // return packet
static Futaba_LongPacket   *longPacketWr   = (Futaba_LongPacket *) malloc(sizeof (Futaba_LongPacket));         // for future usage

////////////////Call peripheral driver////////////

void servoConfig(){
   ServoBaudRate baudRate = BRATE_115200; 

   setServoReturnDelay(0xFF, 80);      // 20*50us=1ms. 40 is 2ms. (max=255)

   cout << "Info: Futaba Servos Init Done." << endl;
}

uint8_t servoSet(uint8_t servoID, int16_t goalPostion, uint16_t goalTime){     // set value, calculate Sum

	/////////Assemble short packet//////////
	(*shortPacketWr).Header = init_Futaba_ShortPacket.Header;
	(*shortPacketWr).ID   = servoID;

        //// Enable torque////
        servoTorqueEnable(servoID);
        cout << "~~~Torque Enabled." << endl;

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

	///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
 	int i;
        shortPacketWr->Sum = 0;                      // init the SUM with 0
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){    // From "ID". -1 is for SUM. -4 is for Dat
	         shortPacketWr->Sum  ^= *( p + i );
        }

        cout << "=======>send cmd.ID is 0x" << (uint16_t)servoID << ".Dat[0](position):0x"  << hex << shortPacketWr->Dat[0] << ". Dat[1](time):0x" << shortPacketWr->Dat[1] << endl;

        uart0_WrServoShortPacket( shortPacketWr);

exit:
	return shortPacketWr->Sum;
}


void uart0_WrServoShortPacket(Futaba_ShortPacket *shortPacketWr){
       uint8_t  k = 0;
       uint8_t *p = (uint8_t *)shortPacketWr;
       //uint8_t *p_Sum = &(shortPacketWr->Sum);

       #ifdef UART0_USE_POSIX_INTERFACE
          /*cout << "=====>Sent. Header:0x" << hex << shortPacketWr-> Header << ". ID:0x" <<  (uint16_t)shortPacketWr->ID 
             << ". Flg:0x" << (uint16_t)shortPacketWr->Flg << ". Adr:0x"  << (uint16_t)shortPacketWr->Adr << ".Len:0x" << (uint16_t)shortPacketWr->Len
             << ".Cnt:0x"  << (uint16_t)shortPacketWr->Cnt << ". Dat[0]:0x"  << shortPacketWr->Dat[0] << ".Dat[1]:0x"
             << shortPacketWr->Dat[1] << ". Sum is:0x"  << (uint16_t)shortPacketWr->Sum << endl;  */
          uart0_sendBytes(p, sizeof(Futaba_ShortPacket)-4 +  (shortPacketWr->Len)*(shortPacketWr->Cnt)  );   //(uint8_t  *ptr, uint8_t nbyte); -4 for Dat reserved position

       #else
          for (k=0; k< (sizeof(Futaba_ShortPacket) -1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); k++){   // -1 for "SUM". -4 for Dat reserved position
             UART0_Send( *(p+k) );
             cout << "$$UART sending 0x" << hex << (uint16_t) *(p+k) << ". k is " << (uint16_t) k << endl;
          }

          UART0_Send(shortPacketWr->Sum);
          cout << "$$UART sending SUM 0x" << hex << (uint16_t)shortPacketWr->Sum << endl;

          delay_ms(500);    cout <<"...500ms later...." << endl;
          UART0_STA_Check();
       #endif

}


//----------------------------- COMMANDS To Set ---------------------------------------------------------
void servoSetTorque(uint8_t servoID,  int16_t value){
         shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
         shortPacketWr ->ID     = servoID; // 0xFF;                 //!! Any servo connected
         shortPacketWr ->Flg    = Flg_WriteServoRAM;
         shortPacketWr ->Adr    = ADDR_MAX_TORQUE;
         shortPacketWr ->Len    = 0x01;
         shortPacketWr ->Cnt    = 0x01;
         shortPacketWr ->Dat[0] = value;             // 1:enable, 0:disable, 2:brake mode

        ///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
        int i;
        shortPacketWr->Sum = 0;                      // init the SUM with 0
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){
                 shortPacketWr->Sum  ^= *( p + i );
        }
        shortPacketWr ->Dat[0] |= int16_t( (shortPacketWr->Sum) << 8 );

        uart0_WrServoShortPacket( shortPacketWr);

};

void servoTorqueEnable(uint8_t servoID){

         shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
         shortPacketWr ->ID     = servoID; // 0xFF;                 //!! Any servo connected
         shortPacketWr ->Flg    = Flg_WriteServoRAM;
         shortPacketWr ->Adr    = ADDR_TORQUE_EN;
         shortPacketWr ->Len    = 0x01;
         shortPacketWr ->Cnt    = 0x01;
         shortPacketWr ->Dat[0] = 0x01;             // 1:enable, 0:disable, 2:brake mode

        ///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
        shortPacketWr->Sum = 0;                      // init the SUM with 0
        int i;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){

                 shortPacketWr->Sum  ^= *( p + i );

        }
        shortPacketWr ->Dat[0] |= int16_t( (shortPacketWr->Sum) << 8 );

        uart0_WrServoShortPacket( shortPacketWr);

}

void servoTorqueBrake(uint8_t servoID){

         shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
         shortPacketWr ->ID     = servoID; // 0xFF;                 //!! Any servo connect$
         shortPacketWr ->Flg    = Flg_WriteServoRAM;
         shortPacketWr ->Adr    = ADDR_TORQUE_EN;
         shortPacketWr ->Len    = 0x01;
         shortPacketWr ->Cnt    = 0x01;
         shortPacketWr ->Dat[0] = 0x02;             // 1:enable, 0:disable, 2:brake mode

        ///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
        shortPacketWr->Sum = 0;                      // init the SUM with 0
        int i;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){
                 shortPacketWr->Sum  ^= *( p + i );

        }
        shortPacketWr ->Dat[0] |= int16_t( (shortPacketWr->Sum) << 8 );

        uart0_WrServoShortPacket( shortPacketWr);

}


int servoSetID( int16_t id){
        // Futaba_ShortPacket *shortPacketWr = (Futaba_ShortPacket *) malloc(sizeof( Futaba_ShortPacket));   // C++

        /////////Write to RAM firstly//////////
        shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
        shortPacketWr ->ID     = 0xFF; // 0xFF;                 //!! Any servo connected
        shortPacketWr ->Flg    = Flg_WriteServoRAM; //Flg_WriteFlashROM;
        shortPacketWr ->Adr    = ADDR_SERVO_ID;
        shortPacketWr ->Len    = 0x1;
        shortPacketWr ->Cnt    = 0x1;
        shortPacketWr ->Dat[0]  = id;                       //uint16_t
        shortPacketWr ->Sum = 0;

        ///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
        int i;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){    // From "ID". -1 is for SUM. -4 is for Dat
                 shortPacketWr->Sum  ^= *( p + i );
        }
        shortPacketWr ->Dat[1] = (int16_t)shortPacketWr ->Sum;
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
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){    // From "ID". -1 is for $
                 shortPacketWr->Sum  ^= *( p + i );
        }
        shortPacketWr ->Dat[0] = (int16_t)shortPacketWr ->Sum;

        uart0_WrServoShortPacket( shortPacketWr);
        cout << "Set Servo ID done. (shortPacketWr ->ID) (uint8_t) is 0x:" << hex << (uint16_t)(shortPacketWr ->ID) << endl;

       return 0;
};


int setServoReturnDelay(uint8_t servoID, uint8_t retDelayValue){
        /////////Write to RAM firstly//////////
        shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
        shortPacketWr ->ID     = servoID; // 0xFF;                 //!! Any servo connected
        shortPacketWr ->Flg    = Flg_WriteServoRAM; //Flg_WriteFlashROM;
        shortPacketWr ->Adr    = ADDR_RETURN_DLY;
        shortPacketWr ->Len    = 0x1;
        shortPacketWr ->Cnt    = 0x1;
        shortPacketWr ->Dat[0] = (int16_t)retDelayValue;               //int16_t
        shortPacketWr ->Sum = 0;

        ///Sum///
        uint8_t *p = (uint8_t *)shortPacketWr;
        int i;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){    // From "ID". -1 is for SUM. -4 is for Dat
                 shortPacketWr->Sum  ^= *( p + i );
        }
        shortPacketWr ->Dat[0] |=  int16_t((shortPacketWr->Sum) << 8)  ;
        cout << "Setting return delay...Write to RAM firstly. delay value is 0x:" << hex << (uint16_t)(shortPacketWr ->Dat[0] )  << endl;

        uart0_WrServoShortPacket( shortPacketWr);


        ///////////// Write Flash command after write RAM/////////////////
        shortPacketWr ->ID     = servoID;
        shortPacketWr ->Flg    = Flg_WriteFlashROM;
        shortPacketWr ->Adr    = 0xFF;
        shortPacketWr ->Len    = 0x0;
        shortPacketWr ->Cnt    = 0x0;

        ///Sum///
        shortPacketWr->Sum = 0;
        p = (uint8_t *)shortPacketWr;
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){    // From "ID". -1 is for $
                 shortPacketWr->Sum  ^= *( p + i );
        }
        shortPacketWr ->Dat[0] = (int16_t)shortPacketWr->Sum;

        uart0_WrServoShortPacket( shortPacketWr);

        return 0;

}


/*
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

*/

//--------------------------------COMMANDS To Inquiry ---------------------------------------------------

int readReturnPacket(){
    uint8_t *ptr = (uint8_t *)returnPacketRd;
    int wait_cnt = 0;

    //-------- read format ----------
    while ( 0 > uart0_rcvBytes(ptr, 7, UART0_RCV_FLG_START_RCV) ){          // 7 means :read to "Cnt"
            if (wait_cnt < INTVL_MAX_NUM_WAIT_RETURN){

                delay_ms(INTVL_WAIT_RETURN_PACKET);

                wait_cnt ++;
            }
            else{
                cout << "Warning: Maxium wait exceed. -1 returned." << endl;
                return -1;
           }
    }

    //------ read data ------------
    if ( (*returnPacketRd).Len > 0){
          uart0_rcvBytes(ptr+7, ( (*returnPacketRd).Len + 1), UART0_RCV_FLG_LAST_READ );         // +1 for the SUM. finished read, close device
          return 0;
    }
    else{
         cout << "Error: Get return packet but No data on the return packet! data Len is: " << (*returnPacketRd).Len  << endl;
         return -1;
    }

	return 0;

}

int16_t getMemorySpecific(uint8_t servoID, uint8_t itemAddr){  // read a word every time
   int16_t res;
   uint8_t *p_sum = (&(shortPacketWr ->Cnt)) + 1;

   //////// Assemble inquiry the packet /////////
        shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
        shortPacketWr ->ID     = servoID;
        shortPacketWr ->Flg    = 0x0F;                 // F means specific address
        shortPacketWr ->Adr    = itemAddr;                 // inquiry packet. Return data will contain address info(first data address).
        shortPacketWr ->Len    = 0x02;                 // len = 0 means "CMD"
        shortPacketWr ->Cnt    = 0x00;                 // don't care
        //Sum
        uint8_t *p = (uint8_t *)shortPacketWr;
        int i;
        shortPacketWr->Sum = 0;          // init the SUM with 0
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt)  ); i++){
                 shortPacketWr->Sum  ^= *( p + i );
        }

        p_sum += (shortPacketWr->Len)*(shortPacketWr->Cnt);
        *p_sum = shortPacketWr->Sum;

        //cout << "==> Inquire MemorySpecific:" << endl;
        uart0_WrServoShortPacket(shortPacketWr);

        ////////// wait for a while ////////////
        delay_ms(UART0_INQ_RESP_INTVL);            // 1ms

        ////////// Read return data ////////////

        if ( 0 == readReturnPacket() ){
              /* cout << "@ReturnPacketRd ->Header is:0x" << hex << returnPacketRd->Header<<".ID:0x" << (uint16_t)returnPacketRd->ID 
                    <<".Flg:0x"  << (uint16_t)returnPacketRd->Flg <<".Adr:0x" << (uint16_t)returnPacketRd->Adr <<".Len:0x"<< (uint16_t)returnPacketRd ->Len
                    <<".Cnt:0x"  << (uint16_t)returnPacketRd->Cnt <<".Dat[0]:0x" << returnPacketRd ->Dat[0] <<".Dat[1]:0x"<<hex<<returnPacketRd ->Dat[1] << endl;
              */
           if ( 0xDFFD == returnPacketRd ->Header) {
               res = returnPacketRd -> Dat[0];
               //cout << "Info: Memory specific read out is:0x" << hex << res << endl;
               if ( 0x00 != returnPacketRd ->Flg) {  // if something wrong in the servo
                  cout << "WARNING: Somthing wrong in the servo. Flg in return packet is:0x" << hex << returnPacketRd ->Flg << endl;
               }

           }
           else{
               cout << "Error: Something wrong in the return packet. returnPacketRd ->Header is:0x" << hex << returnPacketRd->Header
                    <<".ID:0x"  << (uint16_t)returnPacketRd->ID << ".Flg:0x" << (uint16_t)returnPacketRd->Flg
                    <<".Adr:0x"<< (uint16_t)returnPacketRd ->Adr << ".Len:0x"<< (uint16_t)returnPacketRd ->Len << ".Cnt:0x" 
                    << (uint16_t)returnPacketRd->Cnt << endl;

              // return -1;
           }
        }
        else{
              cout << "Error: Something wrong during readReturnPacket." << endl;
        }

        return res;
}

int getMemoryArea(uint8_t servoID, uint8_t flg_addr = Flg_ReturnMemMap42_59 ){ //0_29 30_59 20_29 42_59 30_41
	int PositionResult = -1;
        uint8_t *p_sum = (&(shortPacketWr ->Cnt)) + 1;

        //////// Assemble inquiry the packet /////////
        shortPacketWr ->Header = init_Futaba_ShortPacket.Header;
        shortPacketWr ->ID     = servoID;
        shortPacketWr ->Flg    = flg_addr;
        shortPacketWr ->Adr    = 0x00;                 // format for request bag of data
        shortPacketWr ->Len    = 0x00;                 // format for request bag of data
        shortPacketWr ->Cnt    = 0x01;                 // format for request bag of data
        //Sum
        uint8_t *p = (uint8_t *)shortPacketWr;
        int i;
        shortPacketWr->Sum = 0;          // init the SUM with 0
        for (i=2; i<(sizeof(Futaba_ShortPacket)-1 -4 + (shortPacketWr->Len)*(shortPacketWr->Cnt) ); i++){
                 shortPacketWr->Sum  ^= *( p + i );
        }
        p_sum += (shortPacketWr->Len)*(shortPacketWr->Cnt);
        *p_sum = shortPacketWr->Sum;

        cout << "==> Inquire Memory Area:" << endl;
        uart0_WrServoShortPacket(shortPacketWr);

	////////// wait for a while ////////////
        delay_ms(UART0_INQ_RESP_INTVL);            // 1ms

        ////////// Read return data ////////////

        if ( 0 == readReturnPacket() ){
               cout << "@RX DAT: returnPacketRd ->Header is:0x" << hex << returnPacketRd->Header <<". ID:0x" << (uint16_t)returnPacketRd->ID 
                    << ".Flg:0x" << (uint16_t)returnPacketRd->Flg << ".Adr:0x" << (uint16_t)returnPacketRd ->Adr
                    << ".Len:0x" << (uint16_t)returnPacketRd->Len << ".Cnt:0x" << (uint16_t)returnPacketRd->Cnt << endl;

           if ( (0xDFFD == returnPacketRd ->Header) && (0x00 == (returnPacketRd->Flg)) ){
               //PositionResult = returnPacketRd -> Dat[0];
               cout << "Info: memory Area data are: 0x" << hex << returnPacketRd->Dat << endl;
               return 0;
           }
           else{
               cout << "Error: Something wrong in the return packet. returnPacketRd ->Header is:0x" << hex << returnPacketRd->Header
                    <<". ID:0x"  << (uint16_t)returnPacketRd->ID  << ".Flg:0x" << (uint16_t)returnPacketRd->Flg << ". Adr:0x" << (uint16_t)returnPacketRd ->Adr
                    << ".Len:0x" << (uint16_t)returnPacketRd->Len << ".Cnt:0x"  << (uint16_t)returnPacketRd->Cnt << endl;

               return -1;
           }
        }
        else{
              cout << "Error: Something wrong during readReturnPacket." << endl;
        }

	return 0;
}


int16_t getPresentPosition(uint8_t servoID){
    int16_t presentPosition = 0;
    presentPosition = getMemorySpecific(servoID, ADDR_PRESENT_POS_L);
    //cout << "@@current postion:" << dec << presentPosition << endl;
    return presentPosition;
}


int16_t getPresentTemperature(uint8_t servoID ){
	int16_t presentTemperature = getMemorySpecific(servoID, ADDR_PRESENT_TMPR_L);
        cout << "@@current temperature:" << dec << (int)presentTemperature << endl;
	return presentTemperature;
}

int16_t getPresentVoltage(uint8_t servoID){
	uint16_t presentVoltage = getMemorySpecific(servoID, ADDR_PRESENT_VOLT_L);
        cout << "@@current Voltage:" << dec<< (int)presentVoltage << endl;
	return  presentVoltage;
}

uint8_t getServoBaudRate(uint8_t servoID){
        int16_t presentBaudRate_ReturnDly = getMemorySpecific(servoID, ADDR_BAUD_RATE);
        uint8_t presentBaudRate = (uint8_t)( (presentBaudRate_ReturnDly << 8) >> 24 );
        cout << "@@current baud rate:" << (int)presentBaudRate << endl;
        return presentBaudRate;
}

uint8_t getServoReturnDelay(uint8_t servoID){
        int16_t presentBaudRate_ReturnDly = getMemorySpecific(servoID, ADDR_BAUD_RATE);
        uint8_t presentReturnDelay = uint8_t( (presentBaudRate_ReturnDly >> 8)  );
        cout << "@@current return delay:" << (int)presentReturnDelay << endl;
        return presentReturnDelay;
}

