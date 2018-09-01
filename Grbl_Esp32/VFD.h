#pragma once
class RS485Timer;
extern RS485Timer *transmitTimer;
bool motorStart();
bool motorStop();
void motorSpeed(unsigned long needForSpeed);
void motorControlInit();
long checkSpeed(unsigned long requiredSpeed);
long getSpeed();

#define notDEBUGSERIAL
#ifdef DEBUGSERIAL
#define debugMessage(x) Serial.println(x)
#define debugMessageNoLn(x) Serial.print(x)
#define debugMessageH(x) Serial.println(x,HEX)
#define debugMessageNoLnH(x) Serial.print(x,HEX)
#else
#define debugMessage(x) 
#define debugMessageNoLn(x)
#define debugMessageH(x) 
#define debugMessageNoLnH
#endif
