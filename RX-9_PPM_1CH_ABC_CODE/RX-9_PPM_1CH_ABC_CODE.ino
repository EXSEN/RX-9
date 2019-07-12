/*
 * coded by EXSEN 
 * date: 2019.07.12 
 * EMF pin connected to A0 of Arduino
 * 3.3V external power supply connected to V of RX-9 simple
 * GND connected to arduino
 * GND connected external power  supply
 * file name = RX-9_PPM_1CH_ABC_CODE
 * tested by arduino nano V3.0 328p
 * Add ABC algorithm
 */

#include <EEPROM.h>

//EARTH
int CO2_EARTH = 414; //ABC

//Timing Setting
int warm_up_time = 180;
unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long prev_time_METI = 0; //ABC
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
int cal_A_ADD_01=4; //ppm
int cal_A_ADD_02=5; //ppm
int cal_A_ADD_03=6; //ppm
int CANU_ADD_01 = 2; //ABC
int CANU_ADD_02 = 3; //ABC
int EMF_max_ADD_01=12; //ABC
int EMF_max_ADD_02=13; //ABC
int EMF_max_ADD_03=14; //ABC


//Status of Sensor
bool Sensor_status = 0;
bool Reset_mode = 0;
int INIT_OFF = 100;

//STEP of co2
float V1 = 0.9060;
float V2 = 0.8345;
float V3 = 0.7557;
int STEP_status = 0;
float STEP_CO2 = 0.0;

//Calibration data
float cal_A = 1.703;
float cal_B = 0.2677;
float CO2_ppm = 0.0;

//Auto Calibration coeff
int32_t MEIN = 1440; //ABC
int32_t METI = 60; //ABC
float EMF_max = 0; //ABC
float THER_max = 0; //ABC
int ELTI = 0; //ABC
int upper_cut = 0;
int under_cut_count = 0;

//Operating time checking
int32_t CANU = 0; //ABC

void setup(){
  Serial.begin(9600);
  pinMode(sw, INPUT_PULLUP);

  //parameter init
  EMF_ini = EEPROM.read(EMF_ini_ADD_01)*256 + EEPROM.read(EMF_ini_ADD_02) + (float)EEPROM.read(EMF_ini_ADD_03)/100;
  cal_A = EEPROM.read(cal_A_ADD_01)*256 + EEPROM.read(cal_A_ADD_02) + (float)EEPROM.read(cal_A_ADD_03)/100; //ppm
  CANU = EEPROM.read(CANU_ADD_01)*256+(float)EEPROM.read(CANU_ADD_02); //ABC
  if(CANU<0){CANU = 0;}
  EMF_max = EEPROM.read(EMF_max_ADD_01)*256 + EEPROM.read(EMF_max_ADD_02) + (float)EEPROM.read(EMF_max_ADD_03)/100;  //ABC
  
  
  //parameter init END

  //Reset on/off
  if(CANU<INIT_OFF){
    Reset_mode = 1;
  }
  else{
    Reset_mode = 0;
  }
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
      
      PPM_CAL(STEP_CO2);
      //Define STEP START
      if(STEP_CO2 >= V1 && STEP_CO2 <= 1){
        STEP_status = 0; 
        under_cut_count = 0; //ABC
      }
      else if(STEP_CO2 < V1 && STEP_CO2 >= V2){
        STEP_status = 1;
        under_cut_count = 0; //ABC
      }
      else if(STEP_CO2 < V2 && STEP_CO2 >= V3){
        STEP_status = 2;
        under_cut_count = 0; //ABC
      }
      else if(STEP_CO2 < V3){
        STEP_status = 3;
        under_cut_count = 0; //ABC
      }
      else if(STEP_CO2 > 1){
        under_cut_count++; //ABC
        if(under_cut_count>3){ //ABC
          EMF_init(EMF_data); //ABC
          under_cut_count = 0; //ABC
        }
        else{
          //do nothing
        }
      }
      //Define STEP END

      //ABC START
      if(current_time - prev_time_METI >= METI){
        if(ELTI<MEIN){
          ELTI++;
        }
        else if(ELTI>=MEIN){
          Auto_CAL(EMF_max, EMF_data);
        }
        if(EMF_max >= EMF_data){
          //do nothing
          upper_cut = 0;
        }
        else if(EMF_max < EMF_data){
          upper_cut++;
          if(upper_cut > 3){
            EMF_max = EMF_data;
            upper_cut = 0;
          }
        }
        prev_time_METI = current_time;
      }
      //ABC END
    }
    //Step calculation END
    Serial.print("TIME = ");
    Serial.print(current_time);
    Serial.print(" CO2 ppm = ");
    Serial.print(CO2_ppm,0);
    Serial.print(" CO2 STEP = ");
    Serial.print(STEP_status);
    Serial.print(" : ");
    switch(STEP_status){
      case 0:
              Serial.print("Fresh");
              break;
      case 1:
              Serial.print("Good");
              break;
      case 2:
              Serial.print("Bad");
              break;
      case 3:
              Serial.print("Very Bad");
              break;
    } 
    Serial.print(", Sensor status =");
    if(Sensor_status == 0){
      Serial.print(" WR");
    }
    else{
      Serial.print(" NR");
    }
      Serial.print(" CANU = ");
      Serial.print(CANU);
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
  cal_A = (EMF_data/EMF_ini)+cal_B*log10(CO2_EARTH); //ABC
  EMF_max = EMF_data; //ABC
  ELTI = 0;  //ABC
  CANU++;  //ABC
  
  EEPROM.write(EMF_ini_ADD_01,(int)EMF_ini/256);
  EEPROM.write(EMF_ini_ADD_02,(int)EMF_ini%256);
  EEPROM.write(EMF_ini_ADD_03,(int(EMF_ini*100)%100));

  EEPROM.write(cal_A_ADD_01,(int)cal_A/256);
  EEPROM.write(cal_A_ADD_02,(int)cal_A%256);
  EEPROM.write(cal_A_ADD_03,(int(cal_A*100)%100));

  EEPROM.write(CANU_ADD_01,CANU/256); //ABC
  EEPROM.write(CANU_ADD_02,CANU%256); //ABC

  EEPROM.write(EMF_max_ADD_01,(int)EMF_max/256); //ABC
  EEPROM.write(EMF_max_ADD_02,(int)EMF_max%256); //ABC
  EEPROM.write(EMF_max_ADD_03,(int(EMF_max*100)%100)); //ABC    
}    

void PPM_CAL(float STEP_CO2){
  CO2_ppm = pow(10,((cal_A-STEP_CO2)/cal_B));

}


void Auto_CAL(float a, float b){
  EMF_max = b; //ABC
  EMF_ini = a; //ABC
  
  ELTI = 0; //ABC
  CANU++;  //ABC
  
  EEPROM.write(CANU_ADD_01,CANU/256); //ABC
  EEPROM.write(CANU_ADD_02,CANU%256); //ABC
  
  EEPROM.write(EMF_max_ADD_01,(int)EMF_max/256); //ABC
  EEPROM.write(EMF_max_ADD_02,(int)EMF_max%256); //ABC
  EEPROM.write(EMF_max_ADD_03,(int(EMF_max*100)%100)); //ABC
}
