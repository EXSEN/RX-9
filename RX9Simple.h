/*
	RX9Simple.h - Library for RX-9 Carbon dioxide gas sensor
	This header require EMF pin and THER pin no. 
	and Step level of Carbon dioxide gas concentration (ppm)
	this code returns step level
	
	Created by Joseph Kim (ykkim) in EXSEN Mar 2, 2020 (ykkim@exsen.co.kr)
	you can search the sensor and specification sheet via www.exsen.co.kr
	Released into the public domain
*/


#ifndef RX9Simple_h
#define RX9Simple_h
#define averaging_count 10


#include "Arduino.h"

class RX9Simple
{
	public:
		RX9Simple(int baseline, int meti, int mein, int cr1, int cr2, int cr3, int cr4);
		//void begin(int baseline,int meti, int mein, int cr1, int cr2, int cr3, int cr4);
		void warm_up_chk();
		int status_co2();
		int cal_co2();
		int cal_co2(float EMF, float THER); //���ο� ppm���
		int step_co2();
		int step_co2(float EMF, float THER); //���ο� ppm���
		void auto_calib_co2();
		void sensor_reset();		
		void DMG_REC();
		void DMG_5000();
	
		

	private:
		int _baseline;
		int _meti;
		int _mein;
		int _cr1;
		int _cr2;
		int _cr3;
		int _cr4;
		float _EMF;
		float _THER;		
};

#endif
