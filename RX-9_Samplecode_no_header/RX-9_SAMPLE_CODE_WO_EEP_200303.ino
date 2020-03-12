 /* 
 *  coded by EXSEN
 *  date: 2020.03.27
 *  CO2 sensor is attached to ATMEGA328P, 16 Mhz, 5V
 *  Board Ver: not specified
 *  file name: RX-9_SAMPLE_CODE_WO_EEP_200303.ino
 *  you can ask about this to ykkim@exsen.co.kr
 *  
 *  CAUTIONS
 *  1. Don't use 3.3V of arduino to RX-9 V, RX-9 consume more than arduino's 3.3V output. 
 *  2. Use external 3.3V source. you can use this (https://ko.aliexpress.com/item/1996291344.html?spm=a2g0s.9042311.0.0.27424c4dytyPzF)
 *  
 */
const String VER = "RX-9_SAMPLE_CODE_WO_EEP_200303";

#define EMF_pin 0  //RX-9 E, Analog, A0 of Arduino 
#define THER_pin 1 //RX-9 T, Analog, A1 of Arduino

#define Base_line 432 
// Lowest earth CO2 concentration 2020, in Hawaii is 413.4 ppm
// You can check this data at https://www.esrl.noaa.gov/gmd/ccgg/trends/
// but as you know, Hawaii don't emit mass CO2. so we add some number to Hawaii data.
// Where you are live in 
//   - Big city and nearby huge traffic road: 40~80 ppm add to Hawaii ppm
//   - Big city and normal traffic road: 20~40 ppm
//   - Suburbs: 10~20 ppm
//   - 10 minutes on foot from traffic road, car can not reach: 0~10 ppm
//   normally gasoline engine makes huge carbon dioxide. if you use this sensor to indoor you should check your environment like above. 

const int32_t max_co2 = 6000;
// RX-9 is reliable 400 ~ 4000 ppm of CO2.
// RX-9 can measure 10000 ppm or more. but it shows little low accuracy.
// So, EXSEN recommend max_co2 as 6000 ppm.

//Timing Setting
int warm_up_time = 180; //default = 180, Rx-9 takes time to heat the core of sensor. 180 seconds is require.
unsigned long current_time = 0;
unsigned long prev_time = 0;
int set_time = 1;  // default = 1, every 1 second, output the data. 

//Moving Average
#define averaging_count 10 // default = 10, moving average count, if you want to see the data more actively use lower number or stably use higher number
float m_averaging_data[averaging_count+1][2] = {0,}; 
float sum_data[2] = {0,};
float EMF = 0; 
float THER = 0;
int averaged_count = 0;

//Sensor data
float EMF_data = 0;
float THER_data = 0;
float THER_ini = 0;

// Thermister constant
// RX-9 have thermistor inside of sensor package. this thermistor check the temperature of sensor to compensate the data
// don't edit the number
#define C1 0.00230088
#define C2 0.000224
#define C3 0.00000002113323296
float Resist_0 = 15;

//Status of Sensor
bool status_sensor = 0;

//Step of co2
int CD1 = 700; // Very fresh, In Korea, 
int CD2 = 1000; // normal 
int CD3 = 2000; // high
int CD4 = 4000; // Very high
int status_step_CD = 0;
// Standard of CO2 concenctration
/*  
 *   South of Korea
        - Mechanical ventilation: 1000 ppm
        - natural ventilation: 1500 ppm
 *   Japan 
        - School: 1500 ppm
        - Construction law: 1000 ppm
 *   WHO Europe: 920 ppm
 *   ASHRAE(US): 1000 ppm
 *   Singapore: 1000 ppm
  */
// CO2 to Human
/*
 * < 450 ppm  : outdoor co2 concentration, Very fresh and healthy             NATURE
 * ~ 700 ppm  : normal indoor concentration                                   HOME
 * ~ 1000 ppm : no damage to human but sensitive person can feel discomfort,  OFFICE
 * ~ 2000 ppm : little sleepy,                                                BUS
 * ~ 3000 ppm : Stiff shoulder, headache,                                     METRO
 * ~ 4000 ppm : Eye, throat irritation, headache, dizzy, blood pressure rise
 * ~ 6000 ppm : Increased respiratory rate
 * ~ 8000 ppm : Shortness of breath
 * ~ 10000 ppm: black out in 2~3 minutes
 * ~ 20000 ppm: very low oxygen exchange at lung. could be dead.
 */

//Calibration data
float cal_A = 372.1;        //Calibrated number
float cal_B = 63.27;        //Calibrated number
float cal_B_offset = 1.0;   //cal_B could be changed by their structure. 
float co2_ppm = 0.0;      
float co2_ppm_output = 0.0;
float DEDT = 1.0;           //Delta EMF/Delta THER, if DEDT = 1 means temperature change 1 degree in Celsius, EMF value can changed 1 mV

//Auto Calibration Coeff
unsigned long prev_time_METI = 0;
int MEIN = 120;                      //CAR: 120, HOME: 1440, Every MEIN minutes, Autocalibration is executed. 
int MEIN_common = 0;
int MEIN_start = 1;
bool MEIN_flag = 0;
int start_stablize = 300;
int METI = 60;                        //Every 60 second, check the autocalibration coef. 
float EMF_max = 0;
float THER_max = 0;
int ELTI = 0;
int upper_cut = 0;
int under_cut_count = 0;
float under_cut = 0.99;               // if co2_ppm shows lower than (Base_line * undercut), sensor do re-calculation 

// Damage recovery
// Sensor can be damaged from chemical gas like high concentrated VOC(Volatile Organic Compound), H2S, NH3, Acidic gas, etc highly reactive gas
// so if damage is come to sensor, sensor don't lose their internal calculation number about CO2. 
// this code and variant are to prevent changing calculation number with LOCKING
bool damage_cnt_fg = 0;
unsigned int damage_cnt = 0;
unsigned int ppm_max_cnt = 0;
float cal_A_LOG[2][10] = {0,};
bool cal_A_LOCK = 0;
float cal_A_LOCK_value = 0.0;
float emf_LOCK_value = 0.0;
unsigned int LOCK_delta = 50;
unsigned int LOCK_disable = 5;
unsigned int LOCK_timer = 5;
unsigned int LOCK_timer_cnt = 0;
unsigned int S3_cnt = 0;

//debug
bool debug = 0;
bool display_mode = 0;

//command
const int timeout = 1000; //UART timeout


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(timeout);
  delay(1000);
  cal_A_LOCK = 0;
  damage_cnt_fg = 0;
  damage_cnt = 0;

}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = millis()/1000;
  if(current_time - prev_time >= set_time){   
    warm_up_chk();
    ppm_cal();
    DMG_REC();
    DMG_5000();
    step_cal_CD();
    auto_calib_co2();
    display_data();    
    prev_time = current_time;
  }  
}

void warm_up_chk(){
  if(current_time < warm_up_time){
    status_sensor = 0;
  }
  else if(current_time >= warm_up_time && status_sensor == 0){
    status_sensor = 1;
    sensor_reset();
  }
}

void ppm_cal(){
  EMF = analogRead(EMF_pin);
  EMF = EMF/1024;   // 10 bits, Change the number if your MCU have other resolution
  EMF = EMF * 5;    // 5V     , Change the number if your MCU have other voltage
  EMF = EMF / 6;    //OPAMP   , Don't change!
  EMF = EMF *1000;  //V to mV conversion
  delay(1);
  THER = analogRead(THER_pin);
  THER = 1/(C1+C2*log((Resist_0*THER)/(1024-THER))+C3*pow(log((Resist_0*THER)/(1024-THER)),3))-273.15;
  delay(1);

  // Moving Average START --->
  m_averaging_data[averaged_count][0] = EMF;
  m_averaging_data[averaged_count][1] = THER;
  
  if(averaged_count < averaging_count){    
    averaged_count++;
  }
  else if(averaged_count >= averaging_count){
    for(int i = 0; i < averaging_count;i++){
      sum_data[0] = sum_data[0] + m_averaging_data[i][0]; //EMF
      sum_data[1] = sum_data[1] + m_averaging_data[i][1]; //THER
      for(int j = 0; j<2;j++){
        m_averaging_data[i][j] = m_averaging_data[i+1][j];
      }
    }
    EMF_data = sum_data[0]/averaging_count;
    THER_data = sum_data[1]/averaging_count;
    
    sum_data[0] = 0;
    sum_data[1] = 0;    
  }
  // <---Moving Average END

  // CO2 Concentratio Calculation START --->
  co2_ppm = pow(10,((cal_A-(EMF_data+DEDT*(THER_ini-THER_data)))/(cal_B*cal_B_offset)));
  co2_ppm = co2_ppm * 100 / 100;
  if(co2_ppm > max_co2){
    co2_ppm_output = max_co2;
  }
  else if(co2_ppm <= Base_line){
    co2_ppm_output = Base_line;
  }
  else{
    co2_ppm_output = co2_ppm;
  }

  if(co2_ppm <= Base_line * under_cut){
    under_cut_count++;
    if(under_cut_count>5){
      under_cut_count = 0;
      sensor_reset();
    }
  }
  else {
    // do nothing
  }
  // <--- CO2 Concentratio Calculation END   
}

void sensor_reset(){
  if(cal_A_LOCK == 0){
    THER_ini = THER_data;    
    EMF_max = EMF_data;
    THER_max = THER_data;
    ELTI = 0;
    cal_A = EMF_data +log10(Base_line)*(cal_B*cal_B_offset);
  }
}

void step_cal_CD(){
  if(status_sensor == 1){
    if(co2_ppm <CD1){
      status_step_CD = 0;
      
    }
    else if(co2_ppm >= CD1 && co2_ppm < CD2){
      status_step_CD = 1;
    
    }
    else if(co2_ppm >= CD2 && co2_ppm <CD3){
      status_step_CD = 2;
   
    }
    else if(co2_ppm >= CD3 && co2_ppm < CD4){
      status_step_CD = 3;
 
    }
    else if(co2_ppm >= CD4){
      status_step_CD = 4;

    }
  }
}

void auto_calib_co2(){
  if(current_time < start_stablize && MEIN_flag == 0){
    MEIN_flag = 1;
    MEIN_common = MEIN_start;
  }
  else if(current_time >= start_stablize + 1 && MEIN_flag == 1){
    MEIN_common = MEIN;
    //if(display_mode){Serial.println("MEIN_common = MEIN");}
  }
  if(current_time - prev_time_METI >= METI && status_sensor == 1){
    if(ELTI < MEIN_common){
      if(cal_A_LOCK == 0){
        ELTI++;
      }
      else{
        LOCK_timer_cnt++;
      }
    }
    else if(ELTI >= MEIN_common){
      if(cal_A_LOCK == 0){
        cal_A = (EMF_max + DEDT*(THER_max - THER_data)) +log10(Base_line)*(cal_B*cal_B_offset);
        THER_ini = THER_data;
        EMF_max = EMF_data;
        THER_max = THER_data;
        ELTI = 0;        
      }
      if (damage_cnt_fg == 1)
        {
                damage_cnt++;
        }
    }
    if(EMF_max >= EMF_data){
      upper_cut = 0;
    }
    else if(EMF_max < EMF_data){
      upper_cut++;
      if(upper_cut >3){
        EMF_max = EMF_data;
        THER_max = THER_data;
        upper_cut = 0;
      }
    }
    prev_time_METI = current_time;
  }
}

void DMG_REC(){
  for(int i = 0; i < 10; i++){
    cal_A_LOG[0][i] = cal_A_LOG[0][i+1];
    cal_A_LOG[1][i] = cal_A_LOG[1][i+1];
  }
    cal_A_LOG[0][9] = cal_A;
    cal_A_LOG[1][9] = EMF_data;
    if(status_sensor == 1){
      if((cal_A_LOG[1][9] - cal_A_LOG[1][2] >= LOCK_delta) && (cal_A_LOG[1][8] - cal_A_LOG[1][1] >= LOCK_delta) && (cal_A_LOG[1][7] - cal_A_LOG[1][0] >= LOCK_delta)){
        if(cal_A_LOCK == 0){
          cal_A_LOCK = 1;
          cal_A_LOCK_value = cal_A_LOG[0][0];
          emf_LOCK_value = cal_A_LOG[1][0];
          cal_A = cal_A_LOCK_value;
          //if(debug); Serial.println("S1 ---- cal_A_LOG[1][0] = " + cal_A_LOG[1][0] + "cal_A_LOG[1][9] = " + cal_A_LOG[1][9]);
         }
      }
      else if((cal_A_LOG[1][2] > 375 - LOCK_delta) && (cal_A_LOG[1][1] > 375 - LOCK_delta) && (cal_A_LOG[1][0] > 375 - LOCK_delta) && (cal_A_LOG[1][2] <= 375 - LOCK_disable) && (cal_A_LOG[1][1] <= 375 - LOCK_disable) && (cal_A_LOG[1][0] <= 375 - LOCK_disable)){
        if((cal_A_LOG[1][7] > 375) && (cal_A_LOG[1][8] > 375) && (cal_A_LOG[1][9] > 375)){
          if(cal_A_LOCK == 0){
            cal_A_LOCK = 1;
            cal_A_LOCK_value = cal_A_LOG[0][0];
            emf_LOCK_value = cal_A_LOG[1][0];
            cal_A = cal_A_LOCK_value;
            //if(debug); Serial.println("S2 ---- cal_A_LOG[1][0] = " + cal_A_LOG[1][0] + "cal_A_LOG[1][9] = " + cal_A_LOG[1][9]);
          }
        }
      }
      else{
        //do nothing
      }
    }
    if(cal_A_LOCK == 1){
      if(cal_A_LOG[1][9] - emf_LOCK_value < LOCK_disable){
        S3_cnt++;
        if(S3_cnt >= 10){
          S3_cnt = 0;
          cal_A_LOCK = 0;
          ELTI = 0;
          THER_ini = THER_data; 
          EMF_max = EMF_data; 
          THER_max = THER_data; 
          LOCK_timer_cnt = 0;
        }
        else{
          //do nothing
        }
      }
      else if(LOCK_timer_cnt >= LOCK_timer){
        cal_A_LOCK = 0;
        ELTI = 0;
        THER_ini = THER_data; 
        EMF_max = EMF_data; 
        THER_max = THER_data; 
        LOCK_timer_cnt = 0;
      }
      else{
        S3_cnt = 0;
      }
    }
    else{
      //do nothing
    }
}

void display_data(){
  if(display_mode == 0){
    Serial.print("# ");
    if(co2_ppm <= 999){
      Serial.print("0");
      Serial.print(co2_ppm_output,0);
    }
    else{
      Serial.print(co2_ppm_output,0);
    }
    Serial.print(" ");
   
    if(status_sensor == 0){
      Serial.print("WU");
    }
    else{
      Serial.print("NR");
    }
    Serial.println("");
    }
    else if(display_mode == 1){
      Serial.print("T ");
      Serial.print(current_time);
      Serial.print(" # ");
      if(co2_ppm <= 999){
        Serial.print("0");
        Serial.print(co2_ppm,0);
      }
      else{
        Serial.print(co2_ppm,0);
      }
      Serial.print(" | CS: ");
      Serial.print(status_step_CD);
      Serial.print(" | ");
      Serial.print(EMF_data);
      Serial.print(" mV | ");
      Serial.print(THER_data);
      
      if(status_sensor == 0){
        Serial.print(" oC | WU");
      }
      else{
        Serial.print(" | NR");
      }
      Serial.println("");      
    }
}

void DMG_5000()
{
    if (status_sensor == 1) {
        if (co2_ppm_output >= 5000) {
            if (ppm_max_cnt > 60) {
                MEIN_common = 3;
                damage_cnt_fg = 1;
                ppm_max_cnt = 0;
            }
            else {
                ppm_max_cnt++;
            }
        }
        else {
            ppm_max_cnt = 0;
        }
    }
    if (damage_cnt > 5) {
        MEIN_common = MEIN;
        damage_cnt = 0;
        damage_cnt_fg = 0;
    }
    else {
        //do nothing
    }
}
