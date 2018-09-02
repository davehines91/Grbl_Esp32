/* 
  VFD.cpp

  Sept 2018 Dave Hines


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

#include "esp32-hal-timer.h"
#include "RS485Timer.h"
#include "VFD.h"
RS485Timer *transmitTimer = nullptr;
#include "serial1.h"
#include "grbl.h"

#define milliSeconds millis

static unsigned char  vfdstart[] = {0x01, 0x03, 0x01, 0x01, 0x31, 0x88};
static unsigned char  vfdstop[] = {0x01, 0x03, 0x01, 0x08, 0xf1, 0x8e};

// from Huanyang docs 
#define HOST1 0x01
#define FUNCTION_READ     0x01
#define FUNCTION_WRITE      0x02
#define WRITE_CONTROL_DATA    0x03
#define READ_CONTROL_STATUS   0x04
#define WRITE_FREQ_DATA     0x05

#define HB(x) x>>8
#define LB(x) x&0x00f


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
bool checkCRC(const unsigned char *response,short repLen)
{
  static  char outBuf[8];
  if(*response!= HOST1){
    for(int iy=0;iy<repLen;iy++){
      itoa(response[iy],outBuf,16);
    }

  }
  unsigned short crc1 = CRC16_2(response,repLen);
  if(crc1 == 0){
    debugMessage("CRC-OK");
    return true;
  }
  debugMessage("CRC-NOT");
  debugMessageH(*((unsigned char * )&crc1));
  debugMessageH(*(((unsigned char * )&crc1)+1));
  debugMessageH(response[repLen-2]);
  debugMessageH(response[repLen-1]);
  return false;
}

bool timeout(unsigned long startT, unsigned long timeOut)
{
  unsigned long now = milliSeconds();
  if(now-startT > timeOut){
    debugMessageNoLn(startT);debugMessageNoLn(" ");debugMessageNoLn(timeOut);debugMessageNoLn(" ");debugMessageNoLn(now);debugMessageNoLn(" ");
    debugMessage("Timed Out");
    return false;
  }
  return true;
}
bool timeoutWithAlarm(unsigned long startT, unsigned long timeOut)
{
  if((milliSeconds()-startT) > timeOut){
    debugMessage("timeoutWithAlarm");
    motorStop();      
    system_set_exec_alarm(EXEC_MOTOR_CONTROL_FAIL);
    return false;
  }
  return true;
}
bool responce(int repLen)
{
  bool retVal = false;
  static unsigned char response[32];
  int resp = 0;
  uint8_t bite = 0;
  unsigned long startT = milliSeconds();
  bool didntTimeOut = false;
  bool valid = true;

  while ((resp <repLen) && (didntTimeOut = timeout(startT,20000))){
    bite =  serial1_read(valid);
    if(valid){
      response[resp++] = bite;
    }
  }

  for(int iy=0;iy<repLen;iy++){
    debugMessageNoLnH(response[iy]); debugMessageNoLn(" ");
  }
  debugMessage();
  checkCRC(response,repLen);
  if(didntTimeOut){
    //   printEllapsedMs(startT);
    retVal = true;
  }
  return retVal;
}


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
  int repLen = 7;
  responce(repLen);
}      

void motorControlInit()
{
  serial1_init();
  transmitTimer = new RS485Timer(4000);// 4mS
  pinMode(VFD_SERIAL_DIRECTION_CONTROL, OUTPUT);
  digitalWrite(VFD_SERIAL_DIRECTION_CONTROL, LOW); // Read
  delay(500);
  digitalWrite(VFD_SERIAL_DIRECTION_CONTROL, HIGH); // Write
  delay(500);
  digitalWrite(VFD_SERIAL_DIRECTION_CONTROL, LOW); // Read
  
}

bool motorStart()
{
  int countDown = 2;
  do{
    serial1_send(vfdstart,6);
    debugMessage();
    countDown--;
  }while(!responce(6)&& countDown);
}

bool motorStop()
{
  int countDown = 5;
  do{
    serial1_send(vfdstop,6);
    debugMessage();
    countDown--;
  } while(  !responce(6) && (countDown));
}

long getSpeed()
{
  char spead[2];
  long currentSpeed = 0;
  static unsigned char query[8];
  static unsigned char response[32];
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
  bool valid = true;
  while ((resp <repLen) && (didntTimeOut = timeout(startT,1000))){
    bite =   serial1_read(valid);
    if(valid){
      response[resp++] = bite;
    }
  }
  checkCRC(response,repLen);
  spead[0] = response[5];
  spead[1] = response[4];
  currentSpeed = (long) *((unsigned short *)spead);
  return currentSpeed;
}
long checkSpeed(unsigned long requiredSpeed)
{
  const unsigned long timeOutFor24000 = 50000;
  unsigned long lastCheck = milliSeconds();
  unsigned long now;
  unsigned long longTimeOutStart = lastCheck;
  long currentSpeed = 0;
  long requestedSpeed = 0;
  do{
    if(((now = milliSeconds())-lastCheck) >2000){
      lastCheck = now;    
      currentSpeed = getSpeed();
      debugMessageNoLn(currentSpeed);debugMessageNoLn(" ");debugMessage(requiredSpeed);
    }
  }
  // will timeout if hasn't reached required speed in 25 seconds ( set above ) my motor takes about 21 with current settings
  // TODO what about slowing down
  while ((abs(currentSpeed-(long)requiredSpeed) > 100)&& timeoutWithAlarm(longTimeOutStart,timeOutFor24000)); // tolerance of 50 rpm for rounding etc
  
  if(abs(currentSpeed-(long)requiredSpeed) < 100){
    debugMessage("Speed Ok");
  }
  return currentSpeed;
}
