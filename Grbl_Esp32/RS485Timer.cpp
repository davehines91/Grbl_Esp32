
#include "esp_system.h"
#include "esp32-hal-timer.h"
#include "RS485Timer.h"
RS485Timer::RS485Timer(unsigned int timoutUS)
{
  pinMode(GPIO_NUM_22,OUTPUT);
  digitalWrite(GPIO_NUM_22,LOW);
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
  digitalWrite(GPIO_NUM_22,LOW);
  timerStop(RS485Timer::timer_);
  RS485Timer::isrCalled = millis();
  portEXIT_CRITICAL_ISR(&RS485Timer::timerMux);
 
}

void RS485Timer::restartTimer()
{
   digitalWrite(GPIO_NUM_22,HIGH);
   timerWrite(timer_, 0);
   timerRestart(timer_);
   timerRunning = true;
}


