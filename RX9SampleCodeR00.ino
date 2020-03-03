/* RX-9 Simple Sample Code
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
#define cr1  700
#define cr2  1000
#define cr3  2000
#define cr4  4000

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
unsigned int co2_ppm = 0;
unsigned int co2_step = 0;
float EMF = 0;
float THER = 0;

RX9Simple RX9(Base_line, meti, mein, cr1, cr2, cr3, cr4);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //RX9Simple.begin(Base_line, meti, mein, cr1, cr2, cr3, cr4);
}

void loop() {
  // put your main code here, to run repeatedly:
  time_s = millis()/1000;
  if(time_s - time_s_prev >= time_s_set){
    time_s_prev = time_s;
    EMF = analogRead(EMF_pin);
    delay(1);
    EMF = EMF / (ADCResol - 1);
    EMF = EMF * ADCvolt;
    EMF = EMF / 6;
    EMF = EMF * 1000;
       
    THER = analogRead(THER_pin);
    delay(1);
    THER = 1/(C1+C2*log((Resist_0*THER)/(ADCResol-THER))+C3*pow(log((Resist_0*THER)/(ADCResol-THER)),3))-273.15;
    
    status_sensor = RX9.status_co2();
    co2_ppm = RX9.cal_co2(EMF,THER); //RX-9 
    co2_step = RX9.step_co2(); //RX-9 Simple    
    Serial.print("# "); 
    if(co2_ppm <1000){
      Serial.print("0");
    }
    else{      
    }
    Serial.print(co2_ppm); Serial.print(" ");
    if(status_sensor){
      Serial.print("NR"); 
    }
    else{
      Serial.print("WU");
    }
    Serial.println("");
  }  
}
