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
 *  All remarks are written under the code to explain
 *  모든 주석은 설명하고자 하는 코드 아래에 쓰여집니다.
 */
const String VER = "RX-9_SAMPLE_CODE_WO_EEP_200312";

#define EMF_pin 0  //RX-9 E, Analog, A0 of Arduino, 아날로그 핀 연결 번호
#define THER_pin 1 //RX-9 T, Analog, A1 of Arduino, 아날로그 핀 연결 번호

#define Base_line 432 
/* 지구의 최저 이산화탄소 농도, 국제 표준 이산화탄소 농도인 하와이 측정값은 2020년에 413.4 ppm 이었음
 * 하지만, 하와이는 매우 청정한 구역이기 때문에 우리가 지내는 장소와는 최저 농도가 차이가 있음
 * 이를 보정하기 위해 본 센서의 최저농도를 하와이의 농도보다는 약간 더 높게 설정할 필요가 있음
 * 보정 수치
 *    - 대도시, 주변에 교통량이 많은 경우: +40~80 ppm
 *    - 대도시, 보통 통행량: +20~40 ppm
 *    - 교외 지역: + 10~20 ppm
 *    - 도로로부터 10분이상 떨어진 곳, 차량이 근접할 수 없는 곳: + 0~10 ppm
 * 일반적으로 가솔린 엔진 기관에서 이산화탄소가 발생하기 때문에 차량의 이동이 잦은 곳에서는 최저 이산화탄소 농도가 높음 
*/
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
/*  RX-9은 400~4000 ppm 의 농도에서 신뢰할 수 있는 수치를 나타냄
 *  그 이상의 농도에서도 측정은 가능하지만, 정확도가 떨어짐
 *  그래서 EXSEN은 4000 ppm 혹은 6000 ppm을 최대 농도로 제한할 것을 추천함
 */
// RX-9 is reliable 400 ~ 4000 ppm of CO2.
// RX-9 can measure 10000 ppm or more. but it shows little low accuracy.
// So, EXSEN recommend max_co2 as 6000 ppm.

//Timing Setting
int warm_up_time = 180; //default = 180, Rx-9 takes time to heat the core of sensor. 180 seconds is require.
unsigned long current_time = 0;
unsigned long prev_time = 0;
int set_time = 1;  // default = 1, every 1 second, output the data. 
/*
 * 특정 주기별로 센서의 값을 계산하고 출력하기 위한 Timing 관련 변수
 * 기본적으로 warm-up 시간은 3분, 180초로 함
 * 센서 출력 및 계산 주기는 1초(set_time)으로 함
 */
 
//Moving Average
#define averaging_count 10 // default = 10, moving average count, if you want to see the data more actively use lower number or stably use higher number
float m_averaging_data[averaging_count+1][2] = {0,}; 
float sum_data[2] = {0,};
float EMF = 0; 
float THER = 0;
int averaged_count = 0;
/*
 * 10개의 데이터를 이동평균하는 것이 default
 * 센서의 변화가 더 빠르기를 원하는 경우 이 값(averaging_count)를 적은 수로 변경하고
 * 센서가 더 안정적인 값을 출력하기를 원하는 경우 이 값(averaging_count)를 더 큰 값을 변경
 * 이동평균은 E핀과 T핀에서 입력된 값에 대해서 수행함
 * EMF: 이동평균 전 값, 현재 실시간 데이터
 * THER: 이동평균 전 값, 현재 실시간 데이터
 * 
 */
//Sensor data
float EMF_data = 0;
float THER_data = 0;
float THER_ini = 0;
/* 
 *  센서 데이터를 저장하기 위한 변수
 *  EMF_data: 이동평균 후 값
 *  THER_data: 이동평균 후 값
 *  THER_ini: 써미스터의 기준 값, 3분 워밍없이 끝난 직후의 값
 */
// Thermister constant
// RX-9 have thermistor inside of sensor package. this thermistor check the temperature of sensor to compensate the data
// don't edit the number
#define C1 0.00230088
#define C2 0.000224
#define C3 0.00000002113323296
float Resist_0 = 15;
/*
 * 써미스터의 입력값을 섭씨로 변경하기 위한 상수값
 * 변경할 필요 없음
 * Resist_0: 15kohm
 */
//Status of Sensor
bool status_sensor = 0;
/*
 * 센서 상태
 * 워밍업 이후에는 1, 이전에는 0
 */

//Step of co2
int CD1 = 700; // Very fresh, In Korea, 
int CD2 = 1000; // normal 
int CD3 = 2000; // high
int CD4 = 4000; // Very high
int status_step_CD = 0;
/*
 * 단계표시용 기준 농도
 * 400~700 ppm:   매우 좋음
 * 700~1000 ppm:  보통
 * 1000~2000 ppm: 높음
 * 2000~4000 ppm: 매우 높음
 * 수치는 사용하는 어플리케이션이나 상황에 따라 변경
 * 일반적으로 한국의 기준은
 *  기계환기인경우(환풍기를 사용하는 경우): 1000 ppm 이상인 경우 환기 권장
 *  자연환기인경우(창문을 열어서 환기하는 경우): 1500 ppm 이상인 경우 환기 권장
 */
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
/*
 * cal_A: 교정 계수
 * cal_B: 교정 계수
 * cal_B_offset: 센서가 특정 기구물에 삽입되게 되면 기구물의 영향을 받게 되므로, 기구물 오프셋을 설정해줘야 함, 기구물이 완성된 다음에 농도테스트로 설정이 가능함
 * co2_ppm: 계산된 co2 농도
 * co2_ppm_output: 계산된 co2 농도를 min/max를 내의 값으로 표현하기 위한 변수
 * DEDT: delta EMF/delta THER, 온도에 따른 EMF 변화량
 * RX-9 Simple은 cal_A와 cal_B를 특별히 넣지 않고 특정값으로 계산함
 * RX-9은 cal_A와 cal_B값이 제품 뒤에 QR 코드로 명시가 되어 있으며, 해당 값을 제품마다 1:1로 넣어서 ppm을 계산할 수 있도록 해야 함 
 * 알고리즘 내에서 cal_A는 계속 변화하고 업데이트가 되는 반면 cal_B는 변화하지 않음
 * 
 */
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
/*
 * prev_time_METI: 특정 주기적으로 자동보정을 실시하기 위한 변수
 * MEIN: 센서를 자동차에서 사용하는 경우 120분 주기로 실시, 일반가정인 경우 1440분(하루, 24시간) 주기로 실시
 * MEIN_common: 설정한 MEIN을 상황별로 변경해주기위한 변수, 예를 들어 센서가 데미지를 입었거나, 구동 초기인 경우 MEIN_common의 값을 변경하여 자동보정 주기가 달라질 수 있도록 설정하여 사용함
 * MEIN_start: 센서를 구동하기 시작한지 얼마 안 되었을 때, 센서값이 불안정한 경우가 있어서, 5분 이내에는 자동보정이 1분주기로 실행될 수 있도록함
 * MEIN_flag: MEIN의 값을 구동 초기 인지, 구동 초기가 아닌지에 따라서 변경해주기 위한 변수
 * start_stablize: 센서를 장시간 비가동한 경우, 센서를 전원 인가 후 안정화 시퀀스를 돌리기 위한 시간변수, default = 300 초
 * METI: 자동보정시 사용하는 변수, 자동보정을 하기 위해서 1주기(MEIN의 횟수) 내에 최대 EMF값(최소 ppm 값, EMF와 ppm은 반비례 관계)과 최대 EMF 값일 때
 *       THER값을 기억하여 MEIN의 횟수가 만료될 때, 최대 EMF값을 Base_line에서 설정한 값으로 변경해주는데, 이 최대 EMF값과 THER값을 현재의 값과 비교하여
 *       더 높은 EMF 값을 기억하게 되는데, 이 비교하는 주기에 대한 수치임. 60인 경우 60초에 한번씩 EMF 값을 비교함.
 *       이 횟수는 MEIN과 연동이 되기 때문에 하루 주기로 자동보정이 돌아가게 하기 위해서 METI가 60인경우 MEIN이 1440이라면, 1주기는 1440분
 *       이라, 하루마다 1회씩 작동함. 
 *       METI가 30인경우 하루 주기로 자동보정이 작동하게 하기 위해서는 MEIN을 2880으로 설정해야 함.
 * EMF_max: 자동보정 1주기 내에 METI에서 설정한 수치마다 비교하여 기억하는 최대 EMF값(최소 ppm 값)     
 * THER_max: EMF_max를 업데이트할 때, 함께 기억하는 값. 실제로 THER의 최대값은 아니고, EMF_max값이 업데이트될 때, 함께 업데이트 됨
 */
// Damage recovery
// Sensor can be damaged from chemical gas like high concentrated VOC(Volatile Organic Compound), H2S, NH3, Acidic gas, etc highly reactive gas
// so if damage is come to sensor, sensor don't lose their internal calculation number about CO2. 
// this code and variant are to prevent changing calculation number with LOCKING
bool damage_cnt_fg = 0;
unsigned int damage_cnt = 0;
unsigned int ppm_max_cnt = 0;
float cal_A_LOG[2][20] = {0,};
bool cal_A_LOCK = 0;
float cal_A_LOCK_value = 0.0;
float emf_LOCK_value = 0.0;
unsigned int LOCK_delta = 50;
unsigned int LOCK_disable = 5;
unsigned int LOCK_timer = 15;
unsigned int LOCK_timer_cnt = 0;
unsigned int S3_cnt = 0;
/*
 *  센서는 데미지를 받을 수 있습니다. 다양한 종류의 데미지가 존재할 수 있으나, 가장 일반적인 데미지는 VOC와 같은 반응성이 높은 가스가
 *  센서로 고농도로 인입되는 경우 센서의 값이 흔들릴 수 있음
 *  이런 경우에 센서의 수치를 빠르게 회복시키기 위한 함수와 그에 대한 변수들.
 *  
 *  damage_cnt_fg: 센서가 데미지를 받아서 일반적인 실내에서 발생할 수 없는 높은 ppm이 발생한 경우, 1분 이상 해당 높은 농도가 유지된 경우 
 *                damage_cnt_fg의 값을 1로 변경함, 일반적인 경우 0
 *  damage_cnt: damage_cnt_fg가 1인 경우 자동보정 주기를 1분마다 진행될 수 있도록 변경하고 자동 보정을 몇 회 진행했을 지를 카운트함
 *  ppm_max_cnt: damage_cnt_fg에서 참조하고 있는 높은 농도가 유지된 경우 몇 초나 유지가 되어야 damage_cnt_fg를 1로 변경할지를 카운트함
 *  cal_A_LOG: 센서가 일시적으로 강한 데미지를 받았는지를 확인하기 위한 배열, 17초 전 데이터와 현재 데이터를 비교하여 급격한 변화가 발생한 경우
 *            데미지를 받았다고 판단하기 위해 17초 전 데이터를 보관함
 */
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
  // <--- CO2 Concentration Calculation END   
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
  else if(current_time >= start_stablize + 1 && MEIN_flag == 1 && damage_cnt_fg == 0){
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
  for(int i = 0; i < 19; i++){
    cal_A_LOG[0][i] = cal_A_LOG[0][i+1];
    cal_A_LOG[1][i] = cal_A_LOG[1][i+1];
  }
    cal_A_LOG[0][19] = cal_A;
    cal_A_LOG[1][19] = EMF_data;
    if(status_sensor == 1){
      if((cal_A_LOG[1][19] - cal_A_LOG[1][2] >= LOCK_delta) && (cal_A_LOG[1][18] - cal_A_LOG[1][1] >= LOCK_delta) && (cal_A_LOG[1][17] - cal_A_LOG[1][0] >= LOCK_delta)){
        if(cal_A_LOCK == 0){
          cal_A_LOCK = 1;
          cal_A_LOCK_value = cal_A_LOG[0][0];
          emf_LOCK_value = cal_A_LOG[1][0];
          cal_A = cal_A_LOCK_value;
          //if(debug); Serial.println("S1 ---- cal_A_LOG[1][0] = " + cal_A_LOG[1][0] + "cal_A_LOG[1][9] = " + cal_A_LOG[1][9]);
         }
      }
      else if((cal_A_LOG[1][2] > 540 - LOCK_delta) && (cal_A_LOG[1][1] > 540 - LOCK_delta) && (cal_A_LOG[1][0] > 540 - LOCK_delta) && (cal_A_LOG[1][2] <= 540 - LOCK_disable) && (cal_A_LOG[1][1] <= 540 - LOCK_disable) && (cal_A_LOG[1][0] <= 540 - LOCK_disable)){
        if((cal_A_LOG[1][17] > 540) && (cal_A_LOG[1][18] > 540) && (cal_A_LOG[1][19] > 540)){
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
      if(EMF_data - emf_LOCK_value < LOCK_disable){
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
                MEIN_common = 2;
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
