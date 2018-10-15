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
#pragma once
class RS485Timer;
extern RS485Timer *transmitTimer;
bool motorStart(bool forward=true);
bool motorStop();
bool motorSpeed(unsigned long needForSpeed);
void motorControlInit();
long checkSpeed(unsigned long requiredSpeed);
long getSpeed();
unsigned short getParameter(uint8_t parameter);
// defined in spindle_control.cpp
extern uint8_t SPINDLE_DIRECTION;
extern bool spindleReversed ;
#define DEBUGSERIAL
#ifdef DEBUGSERIAL
#define debugMessage(x) Serial.println(x)
#define debugMessageNoLn(x) Serial.print(x)
#define debugMessageH(x) Serial.println(x,HEX)
#define debugMessageNoLnH(x) Serial.print(x,HEX)
#else
#define debugMessage(x) 
#define debugMessageNoLn(x)
#define debugMessageH(x) 
#define debugMessageNoLnH(x)
#endif


bool motorGTStart(bool forward);
bool motorGTStop();
bool motorGTSpeed(unsigned long needForSpeed);
long getGTSpeed();

uint16_t getGTParameter(uint16_t parameter);
