/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


bool motorStop();
// Declare system global variable structure

#define B0 0x01
#define B1 0x02
#define B2 0x04
#define B3 0x08
#define B4 0x10
#define B5 0x20
#define B6 0x40
#define B7 0x80

void initialiseDelay()
{
//  TCCR5B |= (1 << WGM52 ); // Configure timer 5 for CTC mode
//  TCCR5B |= (( 1<<CS51)); // prescaler 8 will give 0.5uS @ 16MHz
}

void interuptDelay(int microS)
{
//  TCCR5B |= (1 << WGM52 ); // Set up timer source // Configure timer 5 for CTC mode
//  TCCR5B |= (( 1<<CS51));// prescaler 8 will give 0.5uS @ 16MHz
//  OCR5A = (microS*2) -1; // set up time 
//  TCNT5 = 0; // clear counter to zero
//  TIMSK5 |= (1 << OCIE5A ) ;// //enable interrupts on Output Compare A Match
  
}
void restartDelay()
{
 // TCCR5B = 0;   //  DISABLE TIMER Source ??
 // PORTG |= (1 << 0); // set RS485 direction pin to transmit
}
//ISR ( TIMER5_COMPA_vect )
//{
 // PORTG &= ~(1 << 0);  // set RS485 direction pin to read
  //TCCR5B = 0; // DISABLE TIMER Source ??
  //TIMSK5 &= ~(1 << OCIE5A ) ; // Disable CTC interrupt
//}

///---------------------------------------------------------------------------------------------------------
 
// Calculate the value needed for 
// the CTC match value in OCR4A.
//#define CTC_MATCH_OVERFLOW ((F_CPU / 1000) / 8) 
 
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/atomic.h>
volatile unsigned long timer1_milliSeconds;
long milliSecondseconds_since;
//ISR (TIMER4_COMPA_vect)
//{
//    timer1_milliSeconds++;
//}
unsigned long milliSeconds ()
{
    unsigned long milliSeconds_return;
    // Ensure this cannot be disrupted
//    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        milliSeconds_return = timer1_milliSeconds;
 //   }
     return milliSeconds_return;
}

int setUpMilli(void)
{
    // CTC mode, Clock/8
//    TCCR4B |= (1 << WGM42) | (1 << CS41);
     // Load the high byte, then the low byte
    // into the output compare
 //   OCR4AH = (CTC_MATCH_OVERFLOW >> 8);
  //  OCR4AL = CTC_MATCH_OVERFLOW;
     // Enable the compare match interrupt
   // TIMSK4 |= (1 << OCIE4A);
}

bool timeout(unsigned long startT, unsigned long timeOut)
{
    if(milliSeconds()-startT > timeOut){
//serial2_writestr("Timed Out");
      return false;
    }
    return true;
}
bool timeoutWithAlarm(unsigned long startT, unsigned long timeOut)
{
    //return true;
    if((milliSeconds()-startT) > timeOut){
//     serial2_writestr("Timed Out Alarmed");
 	motorStop();      
 	system_set_exec_alarm(EXEC_MOTOR_CONTROL_FAIL);
      return false;
    }
    return true;
}
//http://forum.htsoft.com/all/showflat.php?Cat=0&Board=pic&Number=4497&Searchpage=1&Main=4497&Words=+Pieter+A.+Britz&topic=&Search=true
unsigned short CRC16_2(const unsigned char * buf, short len)
{
  unsigned short crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++){
    crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else  {                          // Else LSB is not set
        crc >>= 1;                    // Just shift right
      }
    }
  }
// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}


#define HOST1 0x01
#define  FUNCTION_READ     0x01
#define FUNCTION_WRITE      0x02
#define WRITE_CONTROL_DATA    0x03
#define READ_CONTROL_STATUS   0x04
#define WRITE_FREQ_DATA     0x05

#define HB(x) x>>8
#define LB(x) x&0x00ff
short checkSpeed2()
{
//   serial2_writestrsp("checkSpeed2");
    static unsigned char query[8];
    static unsigned char response[32];
    static  char outBuf[8];
    int resp = 0;
    uint8_t bite = 0;
    query[0] = HOST1;
    query[1] = READ_CONTROL_STATUS;
    query[2] = 0x01;
    query[3] = 0x03; 
    unsigned short crc1 = CRC16_2(query,4);  
    query[4] = crc1&0x00ff; 
    query[5] = crc1>>8;
    serial1_send(query,6);
    unsigned long startT = milliSeconds();
    bool didntTimeOut = false;
    int repLen = 8;
    while ((resp <repLen) && (didntTimeOut = timeout(startT,1000))){
     bite =   serial1_read();
     if(bite != SERIAL_NO_DATA){
          response[resp++] = bite;
     }
   }
   unsigned char spead[2];
   spead[0] = response[5];
   spead[1] = response[4];
   itoa(*((unsigned short *)spead),outBuf,10);
//  serial2_writestrsp(outBuf);
//  iserial2_write('\n');

}
bool responce(int repLen);
void motorSpeed(unsigned long needForSpeed)
{
unsigned short reqFrequ = (needForSpeed*100)/60;
    unsigned char query[16];
    query[0] = HOST1;
    query[1] = WRITE_FREQ_DATA;
    query[2] = 0x02;
    query[3] = HB(reqFrequ);
    query[4] = LB(reqFrequ);
    unsigned short crc1 = CRC16_2(query,5);
    query[5] = crc1&0x00ff;
    query[6] = crc1>>8; 
    serial1_send(query,7);
     char outBuf[8];
    int repLen = 7;
    for(int iy=0;iy<repLen;iy++){
      itoa(query[iy],outBuf,16);
     serial1_writestrsp(outBuf);
    }
   serial1_write('\n');
    responce(7);
}      

bool checkCRC(const unsigned char *response,short repLen)
{
  static  char outBuf[8];
  if(*response!= HOST1){
//   serial2_println("GARBAGE");
    for(int iy=0;iy<repLen;iy++){
      itoa(response[iy],outBuf,16);
//     serial2_writestrsp(outBuf);
    }

  }
  unsigned short crc1 = CRC16_2(response,repLen);
  if(crc1 == 0){
//   serial2_println("CRC-OK");
    return true;
  }
// serial2_println("CRC-NOT");
  return false;
}
short checkSpeed(unsigned long requiredSpeed)
{
	const unsigned long timeOutFor24000 = 50000;
   unsigned long lastCheck = milliSeconds();
   unsigned long now;
   unsigned char spead[2];
   unsigned long longTimeOutStart = lastCheck;
   long currentSpeed = 0;
   long requestedSpeed = 0;
  do{
    if(((now = milliSeconds())-lastCheck) >2000){
      lastCheck = now;
//   serial2_writestrsp("checkSpeed");
    static unsigned char query[8];
    static unsigned char response[32];
    static  char outBuf[8];
    response[0] = 0x0;
    int resp = 0;
    uint8_t bite = 0;
    query[0] = HOST1;
    query[1] = READ_CONTROL_STATUS;
    query[2] = 0x01;
    query[3] = 0x03; 
    unsigned short crc1 = CRC16_2(query,4);  
    query[4] = crc1&0x00ff; 
    query[5] = crc1>>8;
    serial1_send(query,6);
    unsigned long startT = milliSeconds();
    bool didntTimeOut = false;
    int repLen = 8;
    while ((resp <repLen) && (didntTimeOut = timeout(startT,1000))){
     bite =   serial1_read();
     if(bite != SERIAL_NO_DATA){
          response[resp++] = bite;
     }
   }
   checkCRC(response,repLen);
   spead[0] = response[5];
   spead[1] = response[4];
   itoa(*((unsigned short *)spead),outBuf,10);
//  serial2_writestrsp(outBuf);
   itoa(requiredSpeed,outBuf,10);
//  serial2_writestrsp(outBuf);
//  iserial2_write('\n');
    }
   currentSpeed = (long) *((unsigned short *)spead);
	requestedSpeed = (long) requiredSpeed;
  }
  // will timeout if hasn't reached required speed in 25 seconds ( set above ) my motor takes about 21 with current settings
  // TODO what about slowing down
  while ((abs(currentSpeed-requestedSpeed) > 50)&& timeoutWithAlarm(longTimeOutStart,timeOutFor24000)); // tolerance of 50 rpm for rounding etc
}




void printEllapsedMs(unsigned long lastTime)
{
  unsigned char tbuff[16];
// serial2_write2str("Timer ",itoa(milliSeconds() -lastTime ,tbuff,10));
 }

bool responce(int repLen)
{
	return true;
  bool retVal = false;
// serial2_writestrsp("responce");
   static unsigned char query[8];
   static unsigned char response[32];
   static  char outBuf[8];
  int resp = 0;
  uint8_t bite = 0;
  unsigned long startT = milliSeconds();
  bool didntTimeOut = false;
  while ((resp <repLen) && (didntTimeOut = timeout(startT,1000))){
    bite =   serial1_read();
    if(bite != SERIAL_NO_DATA){
      response[resp++] = bite;
     // itoa(bite,outBuf,16);
     // serial2_writestr(outBuf);
     // iserial2_write(' ');
    }

  }
  checkCRC(response,repLen);
  if(didntTimeOut){
 //   printEllapsedMs(startT);
    retVal = true;
  }
  for(int iy=0;iy<repLen;iy++){
    itoa(response[iy],outBuf,16);
//   serial2_writestrsp(outBuf);
  }
// iserial2_write('\n');
 // serial2_writestr("response EO");
  return retVal;
}

static unsigned char  vfdstart[] = {0x01, 0x03, 0x01, 0x01, 0x31, 0x88};
static unsigned char  vfdstop[] = {0x01, 0x03, 0x01, 0x08, 0xf1, 0x8e};

#define TESTSPEED 10
bool motorStop()
{
    int countDown = 5;
    do{
//   serial2_writestr("Stop \n");
    serial1_send(vfdstop,6);
    countDown--;
  } while(  !responce(6) && (countDown));
}
void initialiseDelay();
bool motorStart()
{
    int countDown = 2;
    do{
//   	serial2_writestr("Start \n");
  	serial1_send(vfdstart,6);
	countDown--;
    }while(!responce(6)&& countDown);
}
void motorInitialise()
{
  // Initialize system upon power-up.
// iserial2_init();
  serial1_init();   // Setup serial baud rate and interrupts
//  iserial0_init();

  initialiseDelay(); //initialise delay timer for rs485 direction control
  setUpMilli(); // setup timer for milliSeconds 
// serial2_writestr("Initialise Motor \n");



}
#ifdef STANDALONE
void mainMotor()
{


  unsigned char command[32];

  unsigned char  vfdspeed400[] = {0x01, 0x05, 0x02, 0x9c, 0x40, 0xd0, 0x3c}; // SPEED 400 .00
  unsigned char  vfdspeed0[] = {0x01 ,0x05 ,0x02, 0x00 ,0x00 ,0xb8 ,0xcc}; // SPEED 0
  unsigned char  vfdspeed2[] = {0x01 ,0x05 ,0x02 ,0x00 ,0xc8 ,0xb9 ,0x5a}; // SPEED 2
  char outBuf[16];
  uint8_t bite = 0;
  motorStop();
  motorSpeed(TESTSPEED);
  motorStart();
  checkSpeed(TESTSPEED *60);

  checkSpeed2();
  _delay_ms(5000);
  do{
    serial2_writestr("Stop \n");
    serial1_send(vfdstop,6);
  } while(  !responce(6));
  

  _delay_ms(1000);

  do{
  bite =   iserial_read();
   if(bite != SERIAL_NO_DATA){
        itoa(bite,outBuf,16);
        serial2_writestr(outBuf);
        iserial2_write(' ');
   }
   bite =  iserial2_read();
   if(bite != SERIAL_NO_DATA){
      iserial1_write(bite);
   }
  }while(1);
  return 0;   /* Never reached */

}

int main()
{ 
	motorInitialise();
  	sei(); // Enable interrupts
    	mainMotor();
    	while(1);
}
#endif

