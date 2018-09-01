/*
  serial1.cpp - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
  
  2018 -  Bart Dring This file was modified for use on the ESP32
          CPU. Do not use this with Grbl for atMega328P
  Sept 2018 -   Dave Hines modified to write to ESP32 Serial port 1 for VFD RS485 control

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

#include "grbl.h"
#include "serial1.h"
#include "RS485Timer.h"
#include "VFD.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)
#define SPEED 115200
portMUX_TYPE myMutex2 = portMUX_INITIALIZER_UNLOCKED;

uint8_t serial1_rx_buffer[RX_RING_BUFFER];
uint8_t serial1_rx_buffer_head = 0;
volatile uint8_t serial1_rx_buffer_tail = 0;


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial1_get_rx_buffer_available()
{
  uint8_t rtail = serial1_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial1_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial1_rx_buffer_head-rtail)); }
  return((rtail-serial1_rx_buffer_head-1));
}
HardwareSerial RS485SerialPort(1);


// this task runs and checks for data on all interfaces, currently
// only hardware serial port is checked
// This was normally done in an interrupt on 8-bit Grbl
void serial1CheckTask(void *pvParameters)
{
  uint8_t data;
  uint8_t next_head;
  
  while(true) // run continuously
  {
    while (RS485SerialPort.available()) {     
        data = RS485SerialPort.read();
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
        vTaskEnterCritical(&myMutex2);
        next_head = serial1_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial1_rx_buffer_tail) {
          serial1_rx_buffer[serial1_rx_buffer_head] = data;
          
          serial1_rx_buffer_head = next_head;
        }
      //  Serial.println(data,HEX);
        vTaskExitCritical(&myMutex2);
    }
    vTaskDelay(1 / portTICK_RATE_MS);  // Yield to other tasks    
  }  // while(true)
}
void serial1_init()
{
//  RS485SerialPort.begin(SPEED); // pin 16=RX, pin 17=TX
  RS485SerialPort.begin(38400,SERIAL_8N1,GPIO_NUM_21,GPIO_NUM_23);// pin 21=RX, pin 23=TX // RS485SerialPort.begin(baud, config, RX_pin, TX_pin );  // works? 
  // create a task to check for incoming data

  xTaskCreatePinnedToCore(  serial1CheckTask,    // task
                          "servoSyncTask1", // name for task
                          2048,   // size of task stack
                          NULL,   // parameters
                          1, // priority
                          &serial1CheckTaskHandle, 
                          0 // core
                          ); 
  
}



void serial1_reset_read_buffer()
{
  serial1_rx_buffer_tail = serial1_rx_buffer_head;
}

// Writes one byte to the TX serial buffer. Called by main program.
void serial1_write(uint8_t data) 
{
  transmitTimer->restartTimer();
  RS485SerialPort.write((char)data);
}
// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial1_read(bool &valid)
{
  valid = true;
  uint8_t tail = serial1_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial1_rx_buffer_head == tail) {
    valid = false;
    return 0;
  } else {
    vTaskEnterCritical(&myMutex2); // make sure buffer is not modified while reading by newly read chars from the serial when we are here
    uint8_t data = serial1_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial1_rx_buffer_tail = tail;
    vTaskExitCritical(&myMutex2);
    return data;
  }
}

void serial1_send(const unsigned char *c,int siz)
{
  int ix=0;
  for(ix=0;ix<siz;ix++){
    serial1_write(c[ix]);  
  }
}

void serial1_writestr(const unsigned char *c)
{
  int ix=0;
  while(c[ix]!=0){
    serial1_write(c[ix]); 
     ix++;
  }
  serial1_write('\n');
}

static char motorMessage[64] ="Hello Speed Control interrupt A\n";
void serial1Motor()
{
  for(int ix=0;ix<strlen(motorMessage);ix++){
    serial1_write(motorMessage[ix]);
  }
}
void serial1_writestrsp(const unsigned char *c)
{
  int ix=0;
  while(c[ix]!=0){
    serial1_write(c[ix++]);  
  }
  serial1_write(' ');
}

