#pragma once
void sendSoftwareInterrupt(uint32_t interruptNumber);
void sendTextToDisplay(char *msg,uint8_t lineNumber=3);
void sendPositionToDisplay(float *cartP,uint8_t coord);
void i2c_init();
