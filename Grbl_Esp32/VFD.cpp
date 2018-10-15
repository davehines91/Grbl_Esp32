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
static unsigned char  vfdReverseStart[] = {0x01, 0x03, 0x01, 0x11, 0x30, 0x44};
static unsigned char  vfdstop[] = {0x01, 0x03, 0x01, 0x08, 0xf1, 0x8e};

// from Huanyang docs 
#define HOST1 0x01
#define FUNCTION_READ     0x01
#define FUNCTION_WRITE      0x02
#define WRITE_CONTROL_DATA    0x03
#define READ_CONTROL_STATUS   0x04
#define WRITE_FREQ_DATA     0x05

#define HB(x) x>>8
#define LB(x) x&0x00ff


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

static unsigned short maxFrequency = 20000;
bool motorSpeed(unsigned long needForSpeed)
{
  return motorGTSpeed(needForSpeed);
  Serial.print("motorSpeed ");Serial.println(needForSpeed);
  unsigned short reqFrequ = (needForSpeed*100)/60;
  if(reqFrequ  > maxFrequency){
    debugMessage("Requested Speed is greater than Max Speed");
    return false;
  }
  Serial.print("reqFreq ");Serial.println(reqFrequ);
  unsigned char query[16];
  query[0] = HOST1;
  query[1] = WRITE_FREQ_DATA;
  query[2] = 0x02;
  query[3] = HB(reqFrequ);
  query[4] = LB(reqFrequ);
  Serial.print(" HB(reqFrequ); ");Serial.println(query[3],HEX);
  Serial.print(" LB(reqFrequ); ");Serial.println(query[4],HEX);
  unsigned short crc1 = CRC16_2(query,5);
  query[5] = crc1&0x00ff;
  query[6] = crc1>>8; 
  serial1_send(query,7);
  int repLen = 7;
  responce(repLen);
  return true;
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
  maxFrequency =  400000 ;//getParameter(0x5);
  Serial.print("maxFrequency ");Serial.println(maxFrequency);
}
#define FWD 1
#define REV 2
bool currentDirection = FWD;
bool reversing = false;
bool motorStart(bool forward)
{
  motorGTStart(forward);
  return true;
  Serial.println("motorStart");
  int countDown = 2;
 // unsigned char  vfdReverseStart[] = {0x01, 0x03, 0x01, 0x11, 0x00, 0x00};
 // unsigned short crc1 = CRC16_2(vfdReverseStart,4);  
 // vfdReverseStart[4] = crc1&0x00ff; 
 // vfdReverseStart[5] = crc1>>8;
 // for(int ix=0;ix<6;ix++){
 //   Serial.println(vfdReverseStart[ix],HEX);
 // }
  
  do{
    if(forward){
      serial1_send(vfdstart,6);
      if(currentDirection != FWD){
        Serial.println("Changing Direction");
        reversing = true;
      }

      currentDirection = FWD;
      Serial.println("FWD");
    }
    else{
      serial1_send(vfdReverseStart,6);
      if(currentDirection != REV){
        Serial.println("Changing Direction");
        reversing = true;
      }
      currentDirection = REV;
      Serial.println("REV");
    }
    debugMessage();
    countDown--;
  }while(!responce(6)&& countDown);
}

bool motorStop()
{
  motorGTStop();
  return true;
  
  Serial.println("motorStop");
  int countDown = 5;
  do{
    serial1_send(vfdstop,6);
    debugMessage();
    countDown--;
  } while(  !responce(6) && (countDown));
}
bool getDirection()
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
  return currentSpeed ;

  
}
long getSpeed()
{
  return getGTSpeed();
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
  return currentSpeed ;
}
long getTimeOut(long requiredSpeed)
{
  return 50000;
   // a little logic to allow motor to time to slow down if reversing
 // requiredSpeed*= spindleReversed ? 1 :-1 $X;
  
  const long  minimumTimeOut = 2500;//mS
  int currentSpeed =0;
  long timout =  abs((currentSpeed = getSpeed() *(reversing ? 1 :-1 ))- requiredSpeed);
  
  timout = (timout<minimumTimeOut) ? minimumTimeOut : timout;
  Serial.printf("Timeout %d  currSpeed %d reqSpeed %d\n",timout,currentSpeed ,requiredSpeed);
  return 50000;
  return timout;
}
long checkSpeed(unsigned long requiredSpeed)
{

  unsigned long allowableError = 50; // rpm
  //const 
  unsigned long timeOutFor24000 = 30000;
  unsigned long lastCheck = milliSeconds();
  unsigned long now;
  unsigned long longTimeOutStart = lastCheck;
  long currentSpeed = 0;
  
  timeOutFor24000  = getTimeOut(requiredSpeed);
 
  long requestedSpeed =requiredSpeed;
  do{
    if(((now = milliSeconds())-lastCheck) >2000){
      lastCheck = now;    
      currentSpeed = getGTSpeed();
      debugMessageNoLn(currentSpeed);debugMessageNoLn(" ");debugMessage(requiredSpeed);
    }
  }
  // will timeout if hasn't reached required speed in 30 seconds ( set above ) my motor takes about 21 with current settings
  while((abs(requestedSpeed - currentSpeed) > allowableError)  && timeoutWithAlarm(longTimeOutStart,timeOutFor24000));
  debugMessageNoLn("Time remaining ");debugMessage((timeOutFor24000 - (lastCheck - longTimeOutStart))/1000.0f);
  if(abs(currentSpeed-requestedSpeed) < allowableError){
    debugMessage("Speed Ok");
  }
  reversing = false;
  return currentSpeed;
}
unsigned short getParameter(uint8_t parameter)
{
  char spead[2];
  unsigned short retrievedParameter = 0;
  static unsigned char query[8];
  static unsigned char response[32];
  response[0] = 0x0;
  int resp = 0;
  uint8_t bite = 0;
  query[0] = HOST1;
  query[1] = FUNCTION_READ;
  query[2] = 0x03;
  query[3] = parameter; 
  query[4] = 0x00;
  query[5] = 0x00;
  unsigned short crc1 = CRC16_2(query,6);  
  query[6] = crc1&0x00ff; 
  query[7] = crc1>>8;
  serial1_send(query,8);
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
  if(checkCRC(response,repLen)){
    spead[0] = response[5];
    spead[1] = response[4];
    retrievedParameter = *((unsigned short *)spead);
    Serial.print("Parameter ");Serial.print(parameter); Serial.print(" ");Serial.println(retrievedParameter);
    return retrievedParameter;
  }
  return 0;
}


/* Modbus function codes */
#define READ_REGISTER    0x03
#define WRITE_SINGLE_REGISTER     0x06
#define BROADCAST_ADDRESS    0

bool gtresponce(int repLen)
{
  bool retVal = false;
  unsigned char response[32];
    bool didntTimeOut = false;
    bool valid = true;
    bool errorReported = false;
    int resp=0;
    uint8_t bite = 0;
    unsigned long startT = milliSeconds();
    while ((resp <repLen) && (didntTimeOut = timeout(startT,5000))){
      bite =  serial1_read(valid);
      if(valid){
        response[resp] = bite;
        if(resp == 1){
          if(bite <0x80){
          }
          else{
            errorReported = true;
            repLen = 5;
          }
        }
        debugMessageNoLnH(response[resp]); debugMessageNoLn(" ");
        resp++;
      }
    }
    unsigned short crcX2 = CRC16_2(response,resp); 
 //   Serial.print("CRC2 = ");
 //   Serial.println(crcX2); 
    debugMessage(" ");
    if(errorReported){
      debugMessageNoLn("gtresponce reported Error ");debugMessageH(response[2]);
    }
     if(didntTimeOut){
         retVal = true;
     }
   return retVal;
 
}

bool  motorGTSpeed(unsigned long needForSpeed)
{
  debugMessage("motorGTSpeed ");
  unsigned short reqFrequ = needForSpeed*10000/24000;
  unsigned char query[16];
  uint16_t addr = 0x2000;
  query[0] = HOST1;
  query[1] = WRITE_SINGLE_REGISTER;
  query[2] = HB(addr);
  query[3] = LB(addr);
  query[4] = HB(reqFrequ);
  query[5] = LB(reqFrequ);
  unsigned short crc1 = CRC16_2(query,6);
  query[6] = crc1&0x00ff;
  query[7] = crc1>>8; 
  serial1_send(query,8);
  int repLen = 8;
  gtresponce(repLen);
  return true;
}      

bool motorGTStart(bool forward)
{
  uint16_t addr = 0x1000;
  uint16_t val = forward ? 1 : 2;
  uint16_t repLen = 8;
  int countDown = 3;
  unsigned char message[32];
  unsigned char response[32];
  if(forward){
    if(currentDirection != FWD){
      reversing = true;   
    }
    currentDirection = FWD;
  }  
  else{
    if(currentDirection != REV){
      reversing = true;
    }
    currentDirection = REV;
  }
  do{
    countDown --;
    message[0] = HOST1;
    message[1] = WRITE_SINGLE_REGISTER;
    message[2] = addr >> 8;
    message[3] = addr & 0x00ff;;
    message[4] = val >> 8;
    message[5] = val & 0x00ff;
    unsigned short crc1 = CRC16_2(message,6);  
    message[6] = crc1&0x00ff; 
    message[7] = crc1>>8;
    delay(30);
    serial1_send(message,8);

 }while(!gtresponce(8)&& countDown);

  return true;
}


bool motorGTStop()
{
  int r;
  uint16_t addr = 0x1000;
  uint16_t val = 5;
  int repLen = 8;
  unsigned char message[32];
  message[0] = HOST1;
  message[1] = WRITE_SINGLE_REGISTER;
  message[2] = addr >> 8;
  message[3] = addr & 0x00ff;
  message[4] = val >> 8;
  message[5] = val & 0x00ff;
  unsigned short crc1 = CRC16_2(message,6);  
  message[6] = crc1&0x00ff; 
  message[7] = crc1>>8;
  delay(30);
  serial1_send(message,8);
  gtresponce(repLen);
  return true;
}
long getGTSpeed()
{
  uint16_t addr = 0x3005;
  return getGTParameter(addr);
}

uint16_t getGTParameter(uint16_t parameter)
{
  int r;
  uint16_t addr = parameter;
  uint16_t val = 1;

  unsigned char message[32];
  message[0] = HOST1;
  message[1] = READ_REGISTER;
  message[2] = addr >> 8;
  message[3] = addr & 0x00ff;
  message[4] = val >> 8;
  message[5] = val & 0x00ff;
  unsigned short crc1 = CRC16_2(message,6);  
  message[6] = crc1&0x00ff; 
  message[7] = crc1>>8;
  delay(30);
  // r = modbus_write_register(mb, addr, val);
  serial1_send(message,8);
  uint8_t spead[2];
  static unsigned char response[32];
  response[0] = 0x0;
  unsigned long startT = milliSeconds();
  bool didntTimeOut = false;
  int repLen = 7;
  bool valid = true;
  uint8_t bite = 0;
  int resp =0;
  while ((resp <repLen) && (didntTimeOut = timeout(startT,1000))){
    bite =   serial1_read(valid);
    if(valid){
      response[resp++] = bite;
    }
  }
  checkCRC(response,repLen);
  spead[0] = response[4];
  spead[1] = response[3];
  uint16_t paramValue = *((uint16_t *)spead);
  debugMessageNoLn("Parameter Value = ");debugMessageNoLnH(parameter) ;  debugMessageNoLn(" ") ;debugMessage(paramValue);
  return paramValue;

}
