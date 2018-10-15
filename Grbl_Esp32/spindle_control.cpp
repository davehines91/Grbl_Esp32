/*
  spindle_control.cpp - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
	
	2018 -	Bart Dring This file was modifed for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P
         
	Sep 2018 Dave Hines: Added Interface to HuanYang VFD over RS485 controlled by RS485_HUANYANG_MOTORCONTROL in config.h
  
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
#ifdef RS485_HUANYANG_MOTORCONTROL
#include "VFD.h"
#endif
void spindle_init()
{
//	Serial.println("spindle_init");
#ifdef RS485_HUANYANG_MOTORCONTROL
    motorControlInit();
#else
    // use the LED control feature to setup PWM   https://esp-idf.readthedocs.io/en/v1.0/api/ledc.html
    ledcSetup(SPINDLE_PWM_CHANNEL, SPINDLE_PWM_BASE_FREQ, SPINDLE_PWM_BIT_PRECISION); // setup the channel
    ledcAttachPin(SPINDLE_PWM_PIN, SPINDLE_PWM_CHANNEL); // attach the PWM to the pin
#endif
    // Start with PWM off
	  spindle_stop();
}

void spindle_stop()
{		
//  Serial.println("spindle_stop");
#ifdef RS485_HUANYANG_MOTORCONTROL
    motorStop();
#else
    grbl_analogWrite(SPINDLE_PWM_CHANNEL, 0);
#endif

}
uint8_t SPINDLE_DIRECTION = SPINDLE_STATE_CW;
bool spindleReversed = false;
uint8_t spindle_get_state()
{	  
 #ifdef RS485_HUANYANG_MOTORCONTROL
    return(SPINDLE_DIRECTION); // Only for RS485
 #else
  // TODO Update this when direction and enable pin are added 
	if (ledcRead(SPINDLE_PWM_CHANNEL) == 0) // Check the PWM value
		return(SPINDLE_STATE_DISABLE);
#endif
}

void spindle_set_speed(uint8_t pwm_value)
{
#ifdef RS485_HUANYANG_MOTORCONTROL
   
#else
  //Serial.println("spindle_set_speed");
	grbl_analogWrite(SPINDLE_PWM_CHANNEL, pwm_value);
#endif
}

// Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
uint8_t spindle_compute_pwm_value(float rpm)
{
#ifdef RS485_HUANYANG_MOTORCONTROL
   return 0;
#else
	uint8_t pwm_value;
	pwm_value = map(rpm, settings.rpm_min, settings.rpm_max, SPINDLE_PWM_OFF_VALUE, SPINDLE_PWM_MAX_VALUE);		  
	// TODO_ESP32  .. make it 16 bit
	
  return(pwm_value);
#endif
}
void setSpindleDirection(uint8_t state)
{
//    Serial.print("setSpindleDirection ");Serial.print(state);
    if (state == SPINDLE_ENABLE_CW) {
      if(SPINDLE_DIRECTION != SPINDLE_STATE_CW){spindleReversed = true;}
      else{spindleReversed = false;}
      SPINDLE_DIRECTION = SPINDLE_STATE_CW;
//      Serial.println(" SPINDLE_ENABLE_CW ");
    } 
    else if (state == SPINDLE_ENABLE_CCW){
      if(SPINDLE_DIRECTION != SPINDLE_STATE_CCW){spindleReversed = true;}
      else{spindleReversed = false;}
      SPINDLE_DIRECTION = SPINDLE_STATE_CCW;
//      Serial.println(" SPINDLE_ENABLE_CCW ");
    }
    else{
      SPINDLE_DIRECTION = SPINDLE_DISABLE;
      spindleReversed = false;
//      Serial.println(" SPINDLE_DISABLE ");
    }
}
void spindle_set_state(uint8_t state, float rpm)
{
//  Serial.println("spindle_set_state");
  if (sys.abort) { return; } // Block during abort.
  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.    
//    Serial.println("SPINDLE_DISABLE");
    sys.spindle_speed = 0.0;    
    spindle_stop();  
  } else {
      setSpindleDirection(state);
      // TODO ESP32 Enable and direction control for pwm
    
      // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
      if (settings.flags & BITFLAG_LASER_MODE) { 
        if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
      }
      spindle_set_speed(spindle_compute_pwm_value(rpm));     
  }  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


void spindle_sync(uint8_t state, float rpm)
{
//  Serial.print("spindle_sync ");Serial.print(state);Serial.print(" ");Serial.print(sys.state);Serial.print(" ");Serial.println(rpm);
	if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state,rpm);
#ifdef RS485_HUANYANG_MOTORCONTROL
  if((rpm >0)&&(state != SPINDLE_DISABLE)){
  	if(motorSpeed((long)rpm)){
  	  motorStart(SPINDLE_DIRECTION==SPINDLE_STATE_CW);
    // Serial.print("SPINDLE_DIRECTION ");Serial.println(SPINDLE_DIRECTION);
    // Serial.print("SPINDLE_ENABLE_CW ");Serial.println(SPINDLE_STATE_CW);
    // Serial.print("SPINDLE_ENABLE_CCW ");Serial.println(SPINDLE_STATE_CCW);
    // Serial.print("SPINDLE_DIRECTION ")Serial.print(SPINDLE_DIRECTION);
      checkSpeed((long)rpm);	
  	}else{
       motorStop();
       system_set_exec_alarm(EXEC_MOTOR_CONTROL_FAIL);    
  	}
    
  }
  else{
  	motorStop();
  	//motorSpeed(rpm);
  }
#endif
}

void grbl_analogWrite(uint8_t chan, uint32_t duty)
{
	if (ledcRead(chan) != duty) // reduce unnecessary calls to ledcWrite()
	{
		ledcWrite(chan, duty);
	}
}

