/*
 * coded by EXSEN 
 * date: 2019.09.30  
 * EMF pin connected to A0 of Arduino
 * THER pin connected to A1 of Arduino
 * 3.3V external power supply connected to V of RX-9 simple
 * RX-9 GND connected to arduino
 * Arduino GND connected external power  supply
 * file name = RX-9_PPM_2CH_ABC_ORIGINAL_CODE_190930
 * source code = GITHUB.com/EXSEN/RX-9
 * tested by arduino nano V3.0 328p
 */

#include <EEPROM.h>

//EARTH
int CO2_EARTH = 414; //ABC
int CO2_BASE_Compensation = 35; //EXSEN nearby road in downtown
float under_cut = 0.99; //최저값의 99% 이하의 값이 되면 센서값을 업데이트함(EMF_INIT())
/* 센서를 설치할 장소가 도시인 경우 지구 CO2 농도(CO2_EARTH)에 값을 더해줘야 합니다.
 * 주변에 차량이동이 많은 곳인경우: 40~80
 * 차량이동이 적지만, 왕복 4차선 정도의 도로가 있고, 차량 통행이 잦은 경우: 20~60
 * 도로가 인접해있지는 않지만, 도심인 경우: 20~40
 * 교외인경우 10~20
 * 통행량이 적은 왕복 2차선 주변인 경우: 10~20
 * 도로와 걸어서 10분 이상 떨어진 경우: 0~10
 * 도로와 이산화탄소의 농도 최저값을 연결시킨 이유는 실제로 가솔린 기관에서 CO2가 많이 발생하기 때문에
 * 출/퇴근 시간에는 이산화탄소 농도가 그렇지 않은 시간에 비해 약 50 ppm 정도 차이가 날 정도로 차량에서
 * 발생하는 이산화탄소 농도가 주변에 미치는 영향이 큽니다. 
 * if you live in city, you add more value. 
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
/*Warming up 시간은 센서를 특정 온도까지 가열하는데 걸리는 시간입니다.
 * 센서가 완전히 식은(1일 이상 비가동) 상태에서는 수시간에서 수십시간의 안정화 시간이 필요합니다.
 * 많은 시간이 필요한 이유는 센서 내부에 흡착된 수분, 기타 가스상들을 제거하기 까지 걸리는 시간과
 * 그것을 보상하는 알고리즘이 원할하게 돌아가게 하기 위한 시간입니다. 
 * 센서를 계속해서 사용하던 중이라면 warm-up 시간이면 충분히 안정화가 됩니다. 
 */
unsigned long current_time = 0; //1초마다 값을 계산하고 출력하기 위해 타이밍 계산용 변수입니다.
unsigned long prev_time = 0;  //1초마다 값을 계산하고 출력하기 위해 타이밍 계산용 변수입니다.
unsigned long prev_time_METI = 0; //ABC 자동보정을 주기적으로 실행하기 위한 변수입니다.
int set_time = 1; //set_time: 1인경우 1초마다, 10인경우 10초마다


//PIN
int EMF_PIN = 0; //아두이노 아날로그 핀 0번에 RX-9의 E 핀이 연결된 경우
int THER_PIN = 1; // 아두이노 아날로그 핀 1번에 RX-9의 T 핀이 연결된 경우

//moving averaging of Sensor value
int averaging_count_EMF = 10; 
//몇 포인트에 대한 이동평균을 계산할 것인가에 대한 변수, 10이면 10개, 100이면 100개, 
//숫자가 커지면 반응이 느려지는 대신에 데이터가 튀지않고 깔끔해지고 숫자가 작아지면 
//반응이 빨라지는 대신이 데이터가 다소 튈 수 있음
float EMF = 0.0;  //raw data of EMF, 이동 평균 계산하기 전 EMF 값
float EMF_AVR[11] = {0,}; //평균값을 저장하기 위한 공간, averaging_count가 10이기 때문에 11로 설정함, averaging_count가 커지면 더 큰 값으로 변경해야 함
int EMF_count = 0; //이동 평균 관련 변수, EMF_count가 averaging_count 보다 작은 경우에는 이동평균을 계산하지 않음, 그렇기 때문에 그 시간에는 EMF 값이 업데이트 되지 않음
float EMF_data = 0;//이동 평균한 EMF 값
float EMF_SUM = 0; //이동 평균 관련 변수


int averaging_count_THER = 10; //EMF의 이동평균과 동일함, THER의 이동평균 관련 변수
float THER = 0.0; //2CH
float THER_AVR[11] = {0,}; //2CH
int THER_count = 0; //2CH
float THER_data = 0; //2CH
float THER_SUM = 0; //2CH
float THER_ini = 0; //2CH

//THERMISTOR constant number
//THERMISTOR 계산을 위한 상수입니다. 변경 불필요
#define C1 0.00230088
#define C2 0.000224
#define C3 0.00000002113323296
float Resist_0 = 15;


//switch
//스위치를 연결한 경우, Manual cal을 하드웨어로 동작시킬 경우 필요합니다.
int sw = 2;
int sw_status = 0;

//EEPROM ADDRESS
bool full_init = 0; //ERASE EEPROM data, 아두이노는 그대로 인데, 센서가 변경되는 경우 EEPROM에 쓰여 있는 값을 모두 초기화해야 오동작이 없음
//cal_A값 저장용 ADDRESS, cal_A는 RX-9의 뒷면에 붙어 있는 QR code에 적혀있는 정보
int cal_A_ADD_01=4; //ppm
int cal_A_ADD_02=5; //ppm
int cal_A_ADD_03=6; //ppm
//CANU 저장용 ADDRESS, CANU는 CAlibration NUmber의 약자로서 Calibration을 몇 번 진행했는지에 대한 정보임
//Manual CAL과 ABC의 횟수가 이에 해당되고 두 값은 공통으로 1씩 증가함
int CANU_ADD_01 = 2; //ABC
int CANU_ADD_02 = 3; //ABC
//EMF_max, ABC 계산용 변수, 특정 주기 내에 최대 EMF_max 값을 로깅하여 그 값을 기준으로 센서를 자동보정함
int EMF_max_ADD_01=12; //ABC
int EMF_max_ADD_02=13; //ABC
int EMF_max_ADD_03=14; //ABC
//THER_ini 온도보상을 하기위한 변수
int THER_ini_ADD_01=17; //2CH
int THER_ini_ADD_02=18; //2CH
//EMF_max와 마찬가지로 자동보정을 하기 위해 EMF_max일 때의 THER값을 받아와서 THER_max로 저장함
int THER_max_ADD_01=15; //2CH
int THER_max_ADD_02=16; //2CH

//Status of Sensor
bool Sensor_status = 0; //warming-up 시간 내에는 0, 이후에는 1
bool Reset_mode = 0; //setup()에서 init_off와 canu의 숫자를 비교해서 껏다 켤 때마다 EMF_init()를 할 것인지
// 아니면 지정한 숫자만큼한 할 것인지
// 아에 하지 않을 것인지를 결정함, 단순 변수이므로 변경 불필요
int INIT_OFF = 0;
// 중요 변수(INIT_OFF)
// 위 Reset_mode에서 설명한 것과 같이 초기 리셋을 할 것인지 몇번할 것인지를 결정하는 중요한 변수
// 0인 경우: 켤 때마다 warming up 종료 후 리셋
// 특정 수인 경우: CANU가 해당 숫자가 될 때 까지 리셋
// 1인 경우: 최초 1회만 리셋하고 이후 리셋하지 않음, 사실상 해제와 같음
// 시스템에서 Manual CAL을 할 수 있도록 하드웨어/소프트웨어 기능을 사용자에게 제공하는 경우에는
// 100 정도의 값을 설정하는 것을 권장하고, 고객이 직접 Manual CAL을 실행할 수 있도록 가이드해야함
// Manual CAL의 조건, 주변이 완전히 환기가 되었고 센서 구동이 30분 이상 계속된 상태에서 manual_cal 실행
// Manual/CAL을 고객이 본인의 의지로 할 수 없다면 0으로 설정하기를 권장함

//STEP of co2
//RX-9 Simple의 경우, 단계표시를 할 때, 몇 ppm 기준으로 단계를 나눌 것인지의 기준
int V1 = 700; //700 ppm 미만: 매우 좋음, 700 ppm 이상: 좋음, 
int V2 = 1000; //1000 ppm 이상: 나쁨
int V3 = 2000; //2000 ppm 이상: 매우 나쁨
int STEP_status = 0;
//STEP_status = 0: 0단계: 매우 좋음
//1단계: 좋음
//2단계: 나쁨
//3단계: 매우 나쁨

//Calibration data
//RX-9의 뒷면에 붙어 있는 QR code 교정값
float cal_A = 372.1; //<--- QR CODE DATA
float cal_B = 56.0; //<--- QR CODE DATA
float cal_B_offset = 1.0; //기구별로 cal_B값이 다르게 구현되기 때문에 완제품 완성 후 보정값 적용해야 함
float CO2_ppm = 0.0;
float DEDT = 1.00; //<--- QR CODE DATA, 기본적으로 1.00을 기본으로 함, 기구물이 완성되면 해당 기구물상수로 넣어주면 됨. 굳이 업데이트하지 않아도 됌

//Auto Calibration coeff
//자동보정(ABC)를 실행하기 위한 변수들
int32_t MEIN = 1440; //자동보정 주기, Default = 1440, 1440 = 1 day, 2880 = 2 day
int32_t METI = 60; //ABC, EMF_max를 비교하기 위한 주기, 60인경우, 60초마다 1회 EMF_max 값을 비교하여 필요한 경우 EMF_max를 업데이트함
float EMF_max = 0; //ABC
float THER_max = 0; //2CH
int ELTI = 0; //ABC, ELapsed TIme의 약자, METI가 60인 경우 60초에 한번씩 비교하는데, 1회 비교시 ELTI가 1씩 증가함, ELTI가 MEIN에 설정한 값과 같거나 커지면 자동보정을 실행함
int upper_cut = 0; 
//EMF_max를 업데이트하기 위한 변수, 일시적인 EMF의 튐이 자동보정의 계수로 사용되면 안 되기 때문에
// 장시간 동안 EMF_max보다 현재 EMF_data가 높으면 그 때, EMF_max를 업데이트함, 
// 장시간: 현재 계산수준으로 METI x 3: 3분
int under_cut_count = 0; //현재 출력되는 ppm 값이 CO2_EARTH + CO2_BASE_Compensation 보다 작은 경우 count


//Operating time checking
int32_t CANU = 0; //ABC, CANU는 CAlibration NUmber의 약자로서 Calibration을 몇 번 진행했는지에 대한 정보임


void setup(){
  Serial.begin(9600); //아두이노 - PC간 시리얼 연결 속도, Default = 9600
  pinMode(sw, INPUT_PULLUP); //Manual CAL용 스위치 핀모드 설정
  delay(1000);
  
  //parameter init
  //주요 파라미터들을 EEPROM의 값을 불러와서 계산에 사용하기 위해서 초기화 시켜줌
  
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
  //3분 리셋을 할 것인지 말 것인지를 정하는 부분
  if(CANU<INIT_OFF || INIT_OFF == 0){
    Reset_mode = 1;
  }
  else{
    Reset_mode = 0;
  }
  
  //Erase EEPROM 
  //EEPROM을 초기화하기 위한 부분
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

    //3분 리셋
    if(current_time >= warm_up_time && Sensor_status == 0){
      if(Reset_mode){
        EMF_init(EMF_data, THER_data);
      }
      Sensor_status = 1;
    }
    else{
      //do nothing
    }
    //3분 리셋 END

    //Step, ppm calculation START
    if(Sensor_status == 1){
      PPM_CAL(EMF_data,THER_data); //warming-up 종료 후 ppm 계산
      //Define STEP START
      if(CO2_ppm <= V1 && CO2_ppm >= (CO2_EARTH + CO2_BASE_Compensation)){
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
      else if(CO2_ppm <= ((CO2_EARTH + CO2_BASE_Compensation) * under_cut)){
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
    //Step, ppm calculation END
    
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
              
              if(Sensor_status != 1){
              Serial.print("W/U");
              }
              else{
              Serial.print("Fresh");
              }
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
  THER_ini = THER_data; //2CH
  EMF_max = EMF_data; //ABC
  THER_max = THER_data; //2CH
  ELTI = 0;  //ABC
  CANU++;  //ABC
  cal_A = EMF_data +log10(CO2_EARTH + CO2_BASE_Compensation)*(cal_B*cal_B_offset);
  
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
  CO2_ppm = pow(10,((cal_A-(EMF_data+DEDT*(THER_ini-THER_data)))/(cal_B*cal_B_offset)));
}

void Auto_CAL(){
  //cal_A = EMF_max +log10(CO2_EARTH + CO2_BASE_Compensation)*cal_B;
  cal_A = (EMF_max + DEDT*(THER_max - THER_data)) +log10(CO2_EARTH + CO2_BASE_Compensation)*(cal_B*cal_B_offset);
  THER_ini = THER_data; //2CH
  EMF_max = EMF_data; //ABC
  THER_max = THER_data; //2CH
  //THER_ini = THER_data; //2CH, delete 191106
  
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

