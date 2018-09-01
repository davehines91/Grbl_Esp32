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
