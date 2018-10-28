#include <arduino.h>
#include <EEPROM.h>
#include <driver/rmt.h>
#include <esp_task_wdt.h>
#include <freertos/task.h>

#include <Wire.h>

#include "LiquidCrystal_I2CESP.h"

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4,LCD_5x8DOTS);

static xQueueHandle softwareInterruptQueue = NULL;
static TaskHandle_t softwareInterruptTaskHandle = 0;

static TaskHandle_t i2cCheckTaskHandle = 0;

static xQueueHandle cartesianQueue = NULL;


volatile int serviceRequired = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
struct LcdMessage{
  uint8_t type;
  uint8_t lineNumber;
  union{
    char text[32];
    struct{
      float currentPosn[3];
      uint8_t coordSys;
    };
 };  

};



void i2cCheckTask(void *pvParameters)
{
  Wire.begin(GPIO_NUM_16,GPIO_NUM_19);
  lcd.begin();
 // Wire.setClock(800000);
  lcd.backlight();
  lcd.print("Dave's CNC ");
  lcd.setCursor(0, 2);
  lcd.print("Hello,");
  //create a queue to handle gpio event from isr
  softwareInterruptQueue = xQueueCreate(100,sizeof(LcdMessage));
  cartesianQueue = xQueueCreate(100,3 *sizeof(float));
  bool tick = false;
  float currentPosn[3];
  char tempBuf[16];
  
  while(true) // run continuously
  {
    LcdMessage message;
    if((serviceRequired > 0) ){

      portENTER_CRITICAL_ISR(&mux);
      if(serviceRequired > 0){
        serviceRequired-- ;
      }
      portEXIT_CRITICAL_ISR(&mux);
      delay(50);
    }
   // if(xQueueReceive(cartesianQueue, &currentPosn, portMAX_DELAY)) {
 /*   if(xQueueReceive(cartesianQueue, &currentPosn, 10)) {
      if(uxQueueMessagesWaiting(cartesianQueue) > 0){
      Serial.printf("Queue Length %d \n",uxQueueMessagesWaiting(cartesianQueue));
      }
      lcd.setCursor(12, 0);
      sprintf(tempBuf,"% 8.2f",currentPosn[0]);
      lcd.print(tempBuf);
      lcd.setCursor(12, 1);
      sprintf(tempBuf,"% 8.2f",currentPosn[1]);
      lcd.print(tempBuf);
      lcd.setCursor(12, 2);
      sprintf(tempBuf,"% 8.2f",currentPosn[2]);
      lcd.print(tempBuf);
      lcd.setCursor(18, 3);
      if(tick){
         lcd.print('A');       
      }
      else{
        lcd.print('V');
      }
      tick = !tick;
    }*/
    if(xQueueReceive(softwareInterruptQueue, &message, 10)) {
      if(message.type == 0){
        if(uxQueueMessagesWaiting(softwareInterruptQueue) > 0){
        Serial.printf("Queue Length %d \n",uxQueueMessagesWaiting(softwareInterruptQueue));
        }
       // Serial.printf("%d %s\n",message.lineNumber,message.text);
        lcd.setCursor(0, message.lineNumber);
        //lcd.print("Hello, world!---");
        if(message.text[0] == 0x0){
          lcd.print("                    ");
        }
        lcd.print(message.text);
      }
      else{
          for(int ix=0;ix<3;ix++){
            lcd.setCursor(12, ix);
            sprintf(tempBuf,"% 8.2f",message.currentPosn[ix]);
            lcd.print(tempBuf);
          }
          lcd.setCursor(17, 3);
          sprintf(tempBuf,"G%d",54+message.coordSys);
          lcd.print(tempBuf);
          lcd.setCursor(0, 1);
          if(tick){
             lcd.print('<');       
          }
          else{
            lcd.print('>');
          }
          tick = !tick;
      }
    }
    vTaskDelay(1 / portTICK_RATE_MS);  // Yield to other tasks 
  }    // while(true)
} 
void i2c_init()
{
  xTaskCreatePinnedToCore(  i2cCheckTask, "i2cCheckTask", 8192, NULL,10, &i2cCheckTaskHandle,0); 
}
void sendSoftwareInterrupt(uint32_t interruptNumber)
{
  xQueueSendFromISR(softwareInterruptQueue, &interruptNumber, NULL);
}

//void sendTextToDisplay(char *msg)
//{
//  xQueueSend(softwareInterruptQueue, msg, NULL);
//}
void sendTextToDisplay(char *msg,uint8_t lineNumber)
{
  LcdMessage lmsg;
  lmsg.type= 0;
  lmsg.lineNumber = lineNumber <3 ? lineNumber : 3;
  strncpy(lmsg.text,msg,32);
  if(softwareInterruptQueue != nullptr){
    xQueueSendFromISR(softwareInterruptQueue, &lmsg, NULL);
  }
}

void sendPositionToDisplay(float *cartP,uint8_t coord)
{
//  if(cartesianQueue != nullptr){
//    xQueueSendFromISR(cartesianQueue, cartP, NULL);
//  }
  LcdMessage lmsg;
  lmsg.type= 1;
  lmsg.lineNumber = 0;
  lmsg.currentPosn[0] = cartP[0];
  lmsg.currentPosn[1] = cartP[1];
  lmsg.currentPosn[2] = cartP[2];
  lmsg.coordSys=coord;
  if(softwareInterruptQueue != nullptr){
    xQueueSendFromISR(softwareInterruptQueue, &lmsg, NULL);
  }
}


