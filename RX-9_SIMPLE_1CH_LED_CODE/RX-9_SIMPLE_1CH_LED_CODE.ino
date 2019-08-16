/*
 * coded by EXSEN 
 * date: 2019.07.12 
 * EMF pin connected to A0 of Arduino
 * 3.3V external power supply connected to V of RX-9 simple
 * GND connected to arduino
 * GND connected external power  supply
 * file name = RX-9_SIMPLE_1CH_CODE
 * source code = 
 * tested by arduino nano V3.0 328p
 */

#include <EEPROM.h>

//Timing Setting
int warm_up_time = 180;
unsigned long current_time = 0;
unsigned long prev_time = 0;
int set_time = 1;
//PIN
int EMF_PIN = 0;

//moving averaging of Sensor value
int averaging_count = 10;
float EMF = 0;  //raw data of EMF
float EMF_AVR[11] = {0,};
int EMF_count = 0;
float EMF_data = 0;
float EMF_SUM = 0; 
float EMF_ini = 0;

//switch
int sw = 2;
int sw_status = 0;

//EEPROM ADDRESS
int EMF_ini_ADD_01 = 7;
int EMF_ini_ADD_02 = 8;
int EMF_ini_ADD_03 = 9;
bool full_init = 0; //ERASE EEPROM data

//Status of Sensor
bool Sensor_status = 0;
bool Reset_mode = 1; //1: Every operation, reset the EMFini, 0: use EMFini from EEPROM

//STEP of co2
float V1 = 0.9060;
float V2 = 0.8345;
float V3 = 0.7557;
int STEP_status = 0;
float STEP_CO2 = 0.0;

//LED
int LED_R = 6;
int LED_G = 5;
int LED_B = 3;

void setup(){
  Serial.begin(9600);
  pinMode(sw, INPUT_PULLUP);
  pinMode(LED_R,OUTPUT); //LED
  pinMode(LED_G,OUTPUT); //LED
  pinMode(LED_B,OUTPUT); //LED
  
  digitalWrite(LED_R,HIGH);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_B,HIGH);
  

  //parameter init
  EMF_ini = EEPROM.read(EMF_ini_ADD_01)*256 + EEPROM.read(EMF_ini_ADD_02) + (float)EEPROM.read(EMF_ini_ADD_03)/100;
  //parameter init END

  //Erase EEPROM 
  if(full_init){
    for(int i = 0;i<1024;i++){
      EEPROM.write(i,0);
      delay(10);
    }
  //Erase EEPROM END
  }
}

void loop(){
  //Timing check
  current_time = millis()/1000;
  if(current_time - prev_time >= set_time){ //every 1 second, if you change set_time to 60, every 1 minute
    EMF = analogRead(EMF_PIN);
    delay(1);
    
    //Moving Averaging START
    if(EMF_count<averaging_count){
      EMF_AVR[EMF_count] = EMF;
      EMF_count++;
    }
    else if(EMF_count >= averaging_count){
      for(int i = 0;i<averaging_count;i++){
        EMF_SUM = EMF_SUM+EMF_AVR[i];
        EMF_AVR[i] = EMF_AVR[i+1];
      }
      EMF_data = EMF_SUM/averaging_count;
      
      EMF_SUM = 0;
      EMF_AVR[averaging_count] = EMF;
    }
    //Moving Averaging END

    //Take Eini
    if(current_time >= warm_up_time && Sensor_status == 0){
      if(Reset_mode){
        EMF_init(EMF_data);
      }
      Sensor_status = 1;
    }
    else{
      //do nothing
      
    }
    //Take Eini END

    //Step calculation START
    if(Sensor_status == 1){
      STEP_CO2 = EMF_data/EMF_ini;
      
      if(STEP_CO2 >= V1 && STEP_CO2 <= 1){
        STEP_status = 0;
      }
      else if(STEP_CO2 < V1 && STEP_CO2 >= V2){
        STEP_status = 1;
      }
      else if(STEP_CO2 < V2 && STEP_CO2 >= V3){
        STEP_status = 2;
      }
      else if(STEP_CO2 < V3){
        STEP_status = 3;
      }
      else if(STEP_CO2 > 1){
        EMF_init(EMF_data);
      }
    }
    //Step calculation END
    Serial.print("TIME = ");
    Serial.print(current_time);
    Serial.print(" CO2 STEP = ");
    Serial.print(STEP_status);
    Serial.print(" : ");
    switch(STEP_status){
      case 0:
              Serial.print("Fresh");
              if(Sensor_status != 1){
                analogWrite(LED_G,0);
                analogWrite(LED_R,0);
                analogWrite(LED_B,0);
              }
              else{
              analogWrite(LED_G,255);
              analogWrite(LED_R,255);
              analogWrite(LED_B,0);
              }
              break;
      case 1:
              Serial.print("Good");
              analogWrite(LED_G,0);
              analogWrite(LED_R,255);
              analogWrite(LED_B,255);
              break;
      case 2:
              Serial.print("Bad");
              analogWrite(LED_G,0);
              analogWrite(LED_R,0);
              analogWrite(LED_B,255);
              break;
      case 3:
              Serial.print("Very Bad");
              analogWrite(LED_G,255);
              analogWrite(LED_R,0);
              analogWrite(LED_B,255);
              break;
    } 
    Serial.print(", Sensor status =");
    if(Sensor_status == 0){
      Serial.print(" WR");
    }
    else{
      Serial.print(" NR");
    }
    Serial.println(" ");
    prev_time = current_time;    
  }
  //Timing check END
  
  if(digitalRead(sw) == LOW){
    EMF_init(EMF_data);   
    delay(300);
  }
}

void EMF_init(float EMF_data){
  EMF_ini = EMF_data;
  EEPROM.write(EMF_ini_ADD_01,(int)EMF_ini/256);
  EEPROM.write(EMF_ini_ADD_02,(int)EMF_ini%256);
  EEPROM.write(EMF_ini_ADD_03,(int(EMF_ini*100)%100));
}    


