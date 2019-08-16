/*
 * coded by EXSEN 
 * date: 2019.08.14  
 * EMF pin connected to A1 of Arduino
 * THER pin connected to A0 of Arduino
 * 3.3V external power supply connected to V of RX-9 simple
 * RX-9 GND connected to arduino
 * Arduino GND connected external power  supply
 * LED_R connected to D5 of Arduino
 * LED_G connected to D6 of Arduino
 * LED_B connected to D8 of Arduino
 * file name = RX-9_NEO_SIMPLE_2CH_LED_CODE
 * source code = GITHUB.com/EXSEN/RX-9
 * tested by arduino nano V3.0 328p
 */

#include <EEPROM.h>

//EARTH
int CO2_EARTH = 414; //ABC
int CO2_BASE_Compensation = 35; //EXSEN nearby road in downtown
float under_cut = 0.99;
/* if you live in city, you add more value. 
 * below is normal compensation value at your residence
 * nearby heavy traffic road = 40~80;
 * downtown = 20~40;
 * Suburb = 10~20;
 * Country apart from road = 0~10;
 * Country nearby road = 10~20;
 * 
 */



//Timing Setting
int warm_up_time = 180;
unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long prev_time_METI = 0; //ABC
int set_time = 1;


//PIN
int EMF_PIN = 1;
int THER_PIN = 0;

//moving averaging of Sensor value
int averaging_count_EMF = 10;
float EMF = 0.0;  //raw data of EMF
float EMF_AVR[11] = {0,};
int EMF_count = 0;
float EMF_data = 0;
float EMF_SUM = 0; 
float EMF_ini = 0;

int averaging_count_THER = 10;
float THER = 0.0; //2CH
float THER_AVR[11] = {0,}; //2CH
int THER_count = 0; //2CH
float THER_data = 0; //2CH
float THER_SUM = 0; //2CH
float THER_ini = 0; //2CH

//THERMISTOR constant number
#define C1 0.00230088
#define C2 0.000224
#define C3 0.00000002113323296
float Resist_0 = 15;


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
int THER_ini_ADD_01=17; //2CH
int THER_ini_ADD_02=18; //2CH
int THER_max_ADD_01=15; //2CH
int THER_max_ADD_02=16; //2CH

//Status of Sensor
bool Sensor_status = 0;
bool Reset_mode = 0;
int INIT_OFF = 100;

//STEP of co2
int V1 = 700;
int V2 = 1000;
int V3 = 2000;
int STEP_status = 0;
float STEP_CO2 = 0.0;

//Calibration data
float cal_A = 465; //<--- QR CODE DATA
float cal_B = 65.3; //<--- QR CODE DATA
float CO2_ppm = 0.0;
float DEDT = 1.00; //<--- QR CODE DATA

//Auto Calibration coeff
int32_t MEIN = 1440; //ABC, 1440 = 1 day, 2880 = 2 day
int32_t METI = 60; //ABC
float EMF_max = 0; //ABC
float THER_max = 0; //2CH
int ELTI = 0; //ABC
int upper_cut = 0;
int under_cut_count = 0;

//Operating time checking
int32_t CANU = 0; //ABC

//line off-set
//circuit resistance from RX-9 to Arduino A0, A1
float EMF_offset = 10.24;
float THER_offset = 10.24;

//LED
int LED_R = 6;
int LED_G = 5;
int LED_B = 8;

void setup(){
  Serial.begin(9600);
  pinMode(sw, INPUT_PULLUP);
  pinMode(LED_R,OUTPUT); //LED
  pinMode(LED_G,OUTPUT); //LED
  pinMode(LED_B,OUTPUT); //LED  
  digitalWrite(LED_R,HIGH);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_B,HIGH);

  digitalWrite(LED_R,LOW);
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_B,LOW);
  delay(1000);
  
  //parameter init
  EMF_ini = EEPROM.read(EMF_ini_ADD_01)*256 + EEPROM.read(EMF_ini_ADD_02) + (float)EEPROM.read(EMF_ini_ADD_03)/100;
  if(EMF_ini<0){EMF_ini = 0;} //ABC
  cal_A = EEPROM.read(cal_A_ADD_01)*256 + EEPROM.read(cal_A_ADD_02) + (float)EEPROM.read(cal_A_ADD_03)/100; //ppm
  if(cal_A<0){cal_A = 0;} //ABC
  CANU = EEPROM.read(CANU_ADD_01)*256+(float)EEPROM.read(CANU_ADD_02); //ABC
  if(CANU<0){CANU = 0;} //ABC
  EMF_max = EEPROM.read(EMF_max_ADD_01)*256 + EEPROM.read(EMF_max_ADD_02) + (float)EEPROM.read(EMF_max_ADD_03)/100;  //ABC
  if(EMF_max<0){EMF_max = 0;} //ABC
  THER_max = EEPROM.read(THER_max_ADD_01)+EEPROM.read(THER_max_ADD_02)/100; //2CH
  if(THER_max<0){THER_max = 0;} //2CH
  THER_ini = EEPROM.read(THER_ini_ADD_01) + (float)EEPROM.read(THER_ini_ADD_02)/100;
  if(THER_ini<0){THER_ini = 0;} //2CH  
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
    EMF = EMF/1024; //10 bit resolution
    EMF = EMF *5; //max voltage = 5V
    EMF = EMF / 6; // opamp value = 6
    EMF = EMF * 1000; //mV to V
    delay(1);
    THER = analogRead(THER_PIN);
    delay(1);
    THER = 1/(C1+C2*log((Resist_0*THER)/(1024-THER))+C3*pow(log((Resist_0*THER)/(1024-THER)),3))-273.15;  
    
    
    //EMF Moving Averaging START
    if(EMF_count<averaging_count_EMF){
      EMF_AVR[EMF_count] = EMF;
      EMF_count++;
    }
    else if(EMF_count >= averaging_count_EMF){
      for(int i = 0;i<averaging_count_EMF;i++){
        EMF_SUM = EMF_SUM+EMF_AVR[i];
        EMF_AVR[i] = EMF_AVR[i+1];
      }
      EMF_data = EMF_SUM/averaging_count_EMF;
      
      EMF_SUM = 0;
      EMF_AVR[averaging_count_EMF] = EMF;
    }
    //EMF Moving Averaging END

    //THER Moving Averaging START
    if(THER_count<averaging_count_THER){
      THER_AVR[THER_count] = THER;
      THER_count++;
    }
    else if(THER_count >= averaging_count_THER){
      for(int i = 0;i<averaging_count_THER;i++){
        THER_SUM = THER_SUM+THER_AVR[i];
        THER_AVR[i] = THER_AVR[i+1];
      }
      THER_data = THER_SUM/averaging_count_THER;
      
      THER_SUM = 0;
      THER_AVR[averaging_count_THER] = THER;
    }
    //THER Moving Averaging END

    //Take Eini
    if(current_time >= warm_up_time && Sensor_status == 0){
      if(Reset_mode){
        EMF_init(EMF_data, THER_data);
      }
      Sensor_status = 1;
    }
    else{
      //do nothing
    }
    //Take Eini END

    //Step calculation START
    if(Sensor_status == 1){
      //STEP_CO2 = EMF_data/EMF_ini;
      
      PPM_CAL(EMF_data,THER_data);
      //Define STEP START
      if(CO2_ppm <= V1 && CO2_ppm >= 400){
        STEP_status = 0; 
        under_cut_count = 0; //ABC
      }
      else if(CO2_ppm > V1 && CO2_ppm <= V2){
        STEP_status = 1;
        under_cut_count = 0; //ABC
      }
      else if(CO2_ppm > V2 && CO2_ppm <= V3){
        STEP_status = 2;
        under_cut_count = 0; //ABC
      }
      else if(CO2_ppm >= V3){
        STEP_status = 3;
        under_cut_count = 0; //ABC
      }
      else if(CO2_ppm <= (CO2_EARTH + CO2_BASE_Compensation) * under_cut){
        under_cut_count++; //ABC
        if(under_cut_count>5){ //ABC
          EMF_init(EMF_data, THER_data); //ABC
          under_cut_count = 0; //ABC
        }
        else{
          //do nothing
        }
      }
      //Define STEP END

      //ABC START
      if(current_time - prev_time_METI >= METI && Sensor_status == 1){
        if(ELTI<MEIN){
          ELTI++;
        }
        else if(ELTI>=MEIN){
          Auto_CAL(); //2CH, ABC
        }
        if(EMF_max >= EMF_data){
          //do nothing
          upper_cut = 0;
        }
        else if(EMF_max < EMF_data){
          upper_cut++;
          if(upper_cut > 3){
            EMF_max = EMF_data; //ppm
            THER_max = THER_data; //2CH
            upper_cut = 0; //ppm
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
    Serial.print(" EMF = ");
    Serial.print(EMF_data,0);
    Serial.print(" Cal_A = ");
    Serial.print(cal_A,0);
    Serial.print(" Cal_B = ");
    Serial.print(cal_B,0);
    
    Serial.print(" THER = ");
    Serial.print(THER_data,0);
    Serial.print(" THER_ini = ");
    Serial.print(THER_ini,0);

    Serial.print(" ELTI = ");
    Serial.print(ELTI);
    
    Serial.print(" EMF_max = ");
    Serial.print(EMF_max,0);
    Serial.print(" THER_max = ");
    Serial.print(THER_max,0);
    
    Serial.print(" CO2 STEP = ");
    Serial.print(STEP_status);
    Serial.print(" : ");
    switch(STEP_status){
      case 0:
              Serial.print("Fresh");
              if(Sensor_status != 1){
                digitalWrite(LED_G,LOW);
                digitalWrite(LED_R,LOW);
                digitalWrite(LED_B,LOW);
              }
              else{
              digitalWrite(LED_G,HIGH);
              digitalWrite(LED_R,HIGH);
              digitalWrite(LED_B,LOW);
              }
              break;
      case 1:
              Serial.print("Good");
              digitalWrite(LED_G,LOW);
              digitalWrite(LED_R,HIGH);
              digitalWrite(LED_B,HIGH);
              break;
      case 2:
              Serial.print("Bad");
              digitalWrite(LED_G,LOW);
              digitalWrite(LED_R,LOW);
              digitalWrite(LED_B,HIGH);
              break;
      case 3:
              Serial.print("Very Bad");
              digitalWrite(LED_G,HIGH);
              digitalWrite(LED_R,LOW);
              digitalWrite(LED_B,HIGH);
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
    EMF_init(EMF_data, THER_data);   
    delay(300);
  }
}

void EMF_init(float EMF_data, float THER_data){
  //EMF_ini = EMF_data;
  THER_ini = THER_data; //2CH
  EMF_max = EMF_data; //ABC
  THER_max = THER_data; //2CH
  ELTI = 0;  //ABC
  CANU++;  //ABC
  cal_A = EMF_data +log10(CO2_EARTH + CO2_BASE_Compensation)*cal_B;
  
  /*
   * EEPROM.write(EMF_ini_ADD_01,(int)EMF_ini/256);
  EEPROM.write(EMF_ini_ADD_02,(int)EMF_ini%256);
  EEPROM.write(EMF_ini_ADD_03,(int(EMF_ini*100)%100));
  */
  
  EEPROM.write(cal_A_ADD_01,(int)cal_A/256);
  EEPROM.write(cal_A_ADD_02,(int)cal_A%256);
  EEPROM.write(cal_A_ADD_03,(int(cal_A*100)%100));

  EEPROM.write(CANU_ADD_01,CANU/256); //ABC
  EEPROM.write(CANU_ADD_02,CANU%256); //ABC

  EEPROM.write(EMF_max_ADD_01,(int)EMF_max/256); //ABC
  EEPROM.write(EMF_max_ADD_02,(int)EMF_max%256); //ABC
  EEPROM.write(EMF_max_ADD_03,(int(EMF_max*100)%100)); //ABC

  EEPROM.write(THER_max_ADD_01,(int)THER_max%256); //2CH
  EEPROM.write(THER_max_ADD_02,(int(THER_max*100)%100)); //2CH

  EEPROM.write(THER_ini_ADD_01,(int)THER_ini%256); //2CH
  EEPROM.write(THER_ini_ADD_02,(int(THER_ini*100)%100)); //2CH
}    

void PPM_CAL(float EMF_data, float THER_data){
  CO2_ppm = pow(10,((cal_A-(EMF_data+DEDT*(THER_ini-THER_data)))/cal_B));
}

void Auto_CAL(){
  cal_A = EMF_max +log10(CO2_EARTH + CO2_BASE_Compensation)*cal_B;
  THER_ini = THER_data; //2CH
  EMF_max = EMF_data; //ABC
  THER_max = THER_data; //2CH
  THER_ini = THER_data; //2CH
  
  ELTI = 0; //ABC
  CANU++;  //ABC
  
  EEPROM.write(CANU_ADD_01,CANU/256); //ABC
  EEPROM.write(CANU_ADD_02,CANU%256); //ABC
  
  EEPROM.write(cal_A_ADD_01,(int)cal_A/256);
  EEPROM.write(cal_A_ADD_02,(int)cal_A%256);
  EEPROM.write(cal_A_ADD_03,(int(cal_A*100)%100));

  EEPROM.write(EMF_max_ADD_01,(int)EMF_max/256); //ABC
  EEPROM.write(EMF_max_ADD_02,(int)EMF_max%256); //ABC
  EEPROM.write(EMF_max_ADD_03,(int(EMF_max*100)%100)); //ABC

  EEPROM.write(THER_max_ADD_01,(int)THER_max%256); //2CH
  EEPROM.write(THER_max_ADD_02,(int(THER_max*100)%100)); //2CH

  EEPROM.write(THER_ini_ADD_01,(int)THER_ini%256); //2CH
  EEPROM.write(THER_ini_ADD_02,(int(THER_ini*100)%100)); //2CH
  
}

