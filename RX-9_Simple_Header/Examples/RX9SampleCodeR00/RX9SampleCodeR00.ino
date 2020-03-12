/*  RX-9 Simple Sample Code
 *  date: 2020.03.03
 *  Carbon Dioxide Gas sensor(RX-9) with
 *  ATMEGA328p, 16Mhz, 5V
 *  file name: RX9SampleCodeR00
 *  
 *  RX-9 have 4 pin
 *  E: EMF
 *  T: Thermistor for sensor
 *  G: GND
 *  V: 3.3V > 200 mA
 *  
 *  this sensor is reliable between air ~ 4000 ppm of CO2
 *  but you can use this sensor over the 4000 ppm
 *  this code can show 6000 ppm as max. 
 *  normally max CO2 concentration is 6000 ppm but if you want to use this higher level, contact me. (ykkim@exsen.co.kr)
 *  
 */
 
#include "RX9Simple.h"
#define EMF_pin 0   // RX-9 E with A0 of arduino
#define THER_pin 1  // RX-9 T with A1 of arduino
#define ADCvolt 5
#define ADCResol 1024
#define Base_line 432
#define meti 60  
#define mein 120 //Automotive: 120, Home or indoor: 1440


//CO2 Step range
#define cr1  700      // Base_line ~ cr1
#define cr2  1000     // cr1 ~ cr2
#define cr3  2000     // cr2 ~ cr3
#define cr4  4000     // cr3 ~ cr4 and over cr4

// Thermister constant
// RX-9 have thermistor inside of sensor package. this thermistor check the temperature of sensor to compensate the data
// don't edit the number
#define C1 0.00230088
#define C2 0.000224
#define C3 0.00000002113323296
float Resist_0 = 15;

//Timing
unsigned int time_s = 0;
unsigned int time_s_prev = 0;
unsigned int time_s_set = 1;

extern volatile unsigned long timer0_millis;

//CO2 Value
int status_sensor = 0;

unsigned int co2_step = 0;
float EMF = 0;
float THER = 0;

RX9Simple RX9(Base_line, meti, mein, cr1, cr2, cr3, cr4);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  time_s = millis()/1000;
  if(time_s - time_s_prev >= time_s_set){
    time_s_prev = time_s;
    //every 1 sec
    //read EMF data from RX-9, RX-9 Simple START-->
    EMF = analogRead(EMF_pin);
    delay(1);
    EMF = EMF / (ADCResol - 1);
    EMF = EMF * ADCvolt;
    EMF = EMF / 6;
    EMF = EMF * 1000;
    // <-- read EMF data from RX-9, RX-9 Simple END
    
    //read THER data from RX-9, RX-9 Simple START-->
    THER = analogRead(THER_pin);
    delay(1);
    THER = 1/(C1+C2*log((Resist_0*THER)/(ADCResol-THER))+C3*pow(log((Resist_0*THER)/(ADCResol-THER)),3))-273.15;
    // <-- read THER data from RX-9, RX-9 Simple END
    
    status_sensor = RX9.status_co2();//read status_sensor, status_sensor = 0 means warming up, = 1 means stable
    RX9.cal_co2(EMF,THER); //calculation carbon dioxide gas concentration. 
    co2_step = RX9.step_co2(); //read steps of carbon dioixde gas concentration. you can edit the step range with cr1~cr4 above.
    
    Serial.print("# "); //Starting letter
    if(status_sensor){
      // you can edit below code to change color of LED or text message by co2_step
      if(co2_step == 0){
      Serial.print("Step 1(~cr1)");
        //400 ppm ~cr1
      }
      else if(co2_step == 1){
        Serial.print("Step 2(cr1~cr2)");
        //cr1 ~ cr2 range
      }
      else if(co2_step == 2){
        Serial.print("Step 3(cr2~cr3)");
        //cr2 ~ cr3 range
      }
      else if(co2_step == 3){
        Serial.print("Step 4(cr3~cr4)");
        //cr3 ~ cr4 range
      }
      else{
        Serial.print("Step 5(cr4~)");
        //over cr4 range
      } 
    }
    else{
      Serial.print("Warming up"); 
      //it takes about 3 minutes to heating the sensor.
      //while warming up. data is not correct. 
    }   
    Serial.println(""); //CR LF
  }  
}
