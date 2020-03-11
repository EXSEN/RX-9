/*
	RX9QR.h - Library for RX-9 Carbon dioxide gas sensor
	This header require EMF pin, THER pin no, cal_A, cal_B
	and Step level of Carbon dioxide gas concentration (ppm)
	this code returns step level and CO2 concentration no.
	
	Created by Joseph Kim (ykkim) in EXSEN Mar 4, 2020 (ykkim@exsen.co.kr)
	you can search the sensor and specification sheet via www.exsen.co.kr
	Released into the public domain
*/


#ifndef RX9QR_h
#define RX9QR_h
#define averaging_count 10


#include "Arduino.h"

class RX9QR
{
	public:
		RX9QR(float cal_A, float cal_B, int baseline, int meti, int mein, int cr1, int cr2, int cr3, int cr4);		
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
    float _cal_A;
    float _cal_B;
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
