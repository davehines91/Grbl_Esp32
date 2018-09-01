/* 
  RS485Timer.cpp

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

#include "esp_system.h"
#include "esp32-hal-timer.h"
#include "cpu_map.h"
#include "RS485Timer.h"

RS485Timer::RS485Timer(unsigned int timoutUS)
{
  pinMode(VFD_SERIAL_DIRECTION_CONTROL,OUTPUT);
  digitalWrite(VFD_SERIAL_DIRECTION_CONTROL,LOW);
  initialiseTimer(timoutUS);
}

bool RS485Timer::initialiseTimer(unsigned int timoutUS)
{
  int tNumber = 3;
  timer_ = timerBegin(tNumber, 80, true);
  timerAttachInterrupt(timer_, &onTimer, true);
  timerAlarmWrite(timer_, timeoutUS_=timoutUS, true);
  timerAlarmEnable(timer_);
  return true;
} 
bool RS485Timer::timerRunning = false;
portMUX_TYPE RS485Timer::timerMux = portMUX_INITIALIZER_UNLOCKED;
unsigned long RS485Timer::isrCalled = millis();
hw_timer_t * RS485Timer::timer_ = nullptr;
int RS485Timer::interruptCounter = 0;
int RS485Timer::totalInterruptCounter = 0;


void IRAM_ATTR RS485Timer::onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  RS485Timer::interruptCounter++;
  RS485Timer::timerRunning = false;
  digitalWrite(VFD_SERIAL_DIRECTION_CONTROL,LOW);
  timerStop(RS485Timer::timer_);
  RS485Timer::isrCalled = millis();
  portEXIT_CRITICAL_ISR(&RS485Timer::timerMux);
 
}

void RS485Timer::restartTimer()
{
   digitalWrite(VFD_SERIAL_DIRECTION_CONTROL,HIGH);
   timerWrite(timer_, 0);
   timerRestart(timer_);
   timerRunning = true;
}


