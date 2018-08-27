#ifndef serial1VFD_h
#define serial1VFD_h
void motorInitialise();
bool motorStop();
bool motorStart();
void motorSpeed(unsigned long rpm);
short checkSpeed(unsigned long requiredSpeed);
#define SERIAL1_MOTOR_CONTROL
#define MOTORCONTROL

#endif
