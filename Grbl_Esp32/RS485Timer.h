/* 
  RS485Timer.h

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
class RS485Timer{
  public:
    static int interruptCounter;
    static int totalInterruptCounter;
    static hw_timer_t * timer_;
    static unsigned long isrCalled;
    static portMUX_TYPE timerMux;
    static bool timerRunning;
    RS485Timer(unsigned int timoutUS);
    static void IRAM_ATTR onTimer();
    unsigned long timeoutUS_;
    bool initialiseTimer(unsigned int timoutUS);
    void restartTimer();
};
extern RS485Timer *transmitTimer;
