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
int THER_PIN = 1;

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
float V1 = 0.9060;
float V2 = 0.8345;
float V3 = 0.7557;
int STEP_status = 0;
float STEP_CO2 = 0.0;

//Calibration data
float cal_A = 1.734; //<--- QR CODE DATA
float cal_B = 0.2770; //<--- QR CODE DATA
float CO2_ppm = 0.0;
float DEDT = 1.67; //<--- QR CODE DATA

//Auto Calibration coeff
int32_t MEIN = 60; //ABC
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

void setup(){
  Serial.begin(9600);
  pinMode(sw, INPUT_PULLUP);

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
    delay(5);
    THER = analogRead(THER_PIN);
    delay(5);
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
      else if(STEP_CO2 > 1.01){
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
      if(current_time - prev_time_METI >= METI){
        if(ELTI<MEIN){
          ELTI++;
        }
        else if(ELTI>=MEIN){
          Auto_CAL(EMF_max, EMF_data, THER_max, THER_data); //2CH, ABC
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
    Serial.print(" EMF_ini = ");
    Serial.print(EMF_ini,0);
    
    Serial.print(" THER = ");
    Serial.print(THER_data,0);
    Serial.print(" THER_ini = ");
    Serial.print(THER_ini,0);
    
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
    EMF_init(EMF_data, THER_data);   
    delay(300);
  }
}

void EMF_init(float EMF_data, float THER_data){
  EMF_ini = EMF_data;
  THER_ini = THER_data; //2CH
  cal_A = (EMF_data/EMF_ini)+cal_B*log10(CO2_EARTH); //ABC
  EMF_max = EMF_data; //ABC
  THER_max = THER_data; //2CH
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

  EEPROM.write(THER_max_ADD_01,(int)THER_max%256); //2CH
  EEPROM.write(THER_max_ADD_02,(int(THER_max*100)%100)); //2CH

  EEPROM.write(THER_ini_ADD_01,(int)THER_ini%256); //2CH
  EEPROM.write(THER_ini_ADD_02,(int(THER_ini*100)%100)); //2CH
}    

void PPM_CAL(float STEP_CO2){
  CO2_ppm = pow(10,((cal_A-(EMF_data+DEDT*(THER_ini-THER_data))/(EMF_ini+DEDT*(THER_ini-THER_data)))/cal_B));
}

void Auto_CAL(float EMF_data, float EMF_max, float THER_data, float THER_max){
  EMF_max = EMF_max; //ABC
  THER_max = THER_max; //2CH
  EMF_ini = EMF_data; //ABC
  THER_ini = THER_data; //2CH
  
  
  ELTI = 0; //ABC
  CANU++;  //ABC
  
  EEPROM.write(CANU_ADD_01,CANU/256); //ABC
  EEPROM.write(CANU_ADD_02,CANU%256); //ABC
  
  EEPROM.write(EMF_max_ADD_01,(int)EMF_max/256); //ABC
  EEPROM.write(EMF_max_ADD_02,(int)EMF_max%256); //ABC
  EEPROM.write(EMF_max_ADD_03,(int(EMF_max*100)%100)); //ABC

  EEPROM.write(THER_max_ADD_01,(int)THER_max%256); //2CH
  EEPROM.write(THER_max_ADD_02,(int(THER_max*100)%100)); //2CH

  
}

