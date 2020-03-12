#include "Arduino.h"
#include "RX9Simple.h"

//VERSION
static const String VER = "RX-9_Simple_operating_header_R1";
//Timing
static unsigned long sec_1cnt = 0;

//Auto Calibration Coeff
static unsigned long prev_time_METI = 0;
static int MEIN = 0;                      //CAR: 120, HOME: 1440, Every MEIN minutes, Autocalibration is executed. 
static int MEIN_common = 0;
static int MEIN_start = 1;
static bool MEIN_flag = 0;
static int start_stablize = 300;
//int METI = 60;                        //Every 60 second, check the autocalibration coef. 
static float EMF_max = 0;
static float THER_max = 0;
static int ELTI = 0;
static int upper_cut = 0;
static int under_cut_count = 0;
static float under_cut = 0.99;               // if co2_ppm shows lower than (Base_line * undercut), sensor do re-calculation 


//CO2 Gas Concentration
static int Base_line = 432;
static const int max_co2 = 6000;


//Prepare the sensor
static int warm_up_time = 180;
static int _status_sensor = 0;

//Moving Averaging
static float m_averaging_data[averaging_count + 1][2] = { 0, };
static float sum_data[2] = { 0, };
//float EMF = 0;
//float THER = 0;
static int averaged_count = 0;
static float EMF_data = 0;
static float THER_data = 0;
static float THER_ini = 0;

//Step of co2
static int status_step_CD = 0;


//Calibration data
static float cal_A = 372.1;        //Calibrated number
static float cal_B = 63.27;        //Calibrated number
static float cal_B_offset = 1.0;   //cal_B could be changed by their structure. 
static float _co2_ppm = 0.0;
static float co2_ppm_output = 0.0;
static float DEDT = 1.0;           //Delta EMF/Delta THER, if DEDT = 1 means temperature change 1 degree in Celsius, EMF value can changed 1 mV

// Damage recovery
// Sensor can be damaged from chemical gas like high concentrated VOC(Volatile Organic Compound), H2S, NH3, Acidic gas, etc highly reactive gas
// so if damage is come to sensor, sensor don't lose their internal calculation number about CO2. 
// this code and variant are to prevent changing calculation number with LOCKING
static bool damage_cnt_fg = 0;
static unsigned int damage_cnt = 0;
static unsigned int ppm_max_cnt = 0;
static float cal_A_LOG[2][20] = { 0, };
static bool cal_A_LOCK = 0;
static float cal_A_LOCK_value = 0.0;
static float emf_LOCK_value = 0.0;
static unsigned int LOCK_delta = 50;
static unsigned int LOCK_disable = 5;
static unsigned int LOCK_timer = 15;
static unsigned int LOCK_timer_cnt = 0;
static unsigned int S3_cnt = 0;


RX9Simple::RX9Simple(int base_line, int meti, int mein, int cr1, int cr2, int cr3, int cr4)
{
    _baseline = base_line;
    _meti = meti;
    _mein = mein;
    _cr1 = cr1;
    _cr2 = cr2;
    _cr3 = cr3;
    _cr4 = cr4;
	}
int RX9Simple::status_co2()
{
    return _status_sensor;
}

int RX9Simple::cal_co2()
{
  
}
int RX9Simple::cal_co2(float EMF, float THER)
{
    _EMF = EMF;
    _THER = THER;
    //1sec count
    sec_1cnt++;
    warm_up_chk();
    //Moving Averaging START -->
    m_averaging_data[averaged_count][0] = _EMF;
    m_averaging_data[averaged_count][1] = _THER;
    if (averaged_count < averaging_count) {
        averaged_count++;
    }
    else if (averaged_count >= averaging_count) {
        for (int i = 0; i < averaging_count; i++) {
            sum_data[0] = sum_data[0] + m_averaging_data[i][0]; //EMF
            sum_data[1] = sum_data[1] + m_averaging_data[i][1]; //THER
            for (int j = 0; j < 2; j++) {
                m_averaging_data[i][j] = m_averaging_data[i + 1][j];
            }
        }
        EMF_data = sum_data[0] / averaging_count;
        THER_data = sum_data[1] / averaging_count;

        sum_data[0] = 0;
        sum_data[1] = 0;
    }
    // <---Moving Average END

    // CO2 Concentratio Calculation START --->
    _co2_ppm = pow(10, ((cal_A - (EMF_data + DEDT * (THER_ini - THER_data))) / (cal_B * cal_B_offset)));
    _co2_ppm = _co2_ppm * 100 / 100;
    if (_co2_ppm > max_co2) {
        co2_ppm_output = max_co2;
    }
    else if (_co2_ppm <= Base_line) {
        co2_ppm_output = Base_line;
    }
    else {
        co2_ppm_output = _co2_ppm;
    }

    if (_co2_ppm <= Base_line * under_cut) {
        under_cut_count++;
        if (under_cut_count > 5) {
            under_cut_count = 0;
            sensor_reset();
        }
    }
    else {
        // do nothing
    }
    DMG_REC();
    DMG_5000();
    auto_calib_co2();
    return co2_ppm_output;
    // <--- CO2 Concentratio Calculation END   
}

int RX9Simple::step_co2() 
{
	
    if (_status_sensor == 1) {
        if (co2_ppm_output < _cr1) {
            status_step_CD = 0;
        }
        else if (co2_ppm_output >= _cr1 && co2_ppm_output < _cr2) {
            status_step_CD = 1;
        }
        else if (co2_ppm_output >= _cr2 && co2_ppm_output < _cr3) {
            status_step_CD = 2;
        }
        else if (co2_ppm_output >= _cr3 && co2_ppm_output < _cr4) {
            status_step_CD = 3;
        }
        else if (co2_ppm_output >= _cr4) {
            status_step_CD = 4;
        }
    }
    return status_step_CD;
}
void RX9Simple::sensor_reset()
{
    if (cal_A_LOCK == 0) {
        THER_ini = THER_data;
        EMF_max = EMF_data;
        THER_max = THER_data;
        ELTI = 0;
        cal_A = EMF_data + log10(Base_line) * (cal_B * cal_B_offset);
    }
}
void RX9Simple::DMG_5000()
{
    if (_status_sensor == 1) {
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
        MEIN_common = _mein;
        damage_cnt = 0;
        damage_cnt_fg = 0;
    }
    else {
        //do nothing
    }
}
void RX9Simple::warm_up_chk()
{
	if (sec_1cnt < warm_up_time) {
		_status_sensor = 0;
	}
	else if (sec_1cnt >= warm_up_time && _status_sensor == 0) {
		_status_sensor = 1;
		sensor_reset();
	}
}
void RX9Simple::auto_calib_co2() 
{
    if(sec_1cnt < start_stablize && MEIN_flag == 0) {
        MEIN_flag = 1;
        MEIN_common = MEIN_start;
    }
    else if (sec_1cnt >= start_stablize + 1 && MEIN_flag == 1 && damage_cnt_fg == 0) {
        MEIN_common = _mein;
      
    }
    if (sec_1cnt - prev_time_METI >= _meti && _status_sensor == 1) {
        if (ELTI < MEIN_common) {
            if (cal_A_LOCK == 0) {
                ELTI++;
            }
            else {
                LOCK_timer_cnt++;
            }
        }
        else if (ELTI >= MEIN_common) {
            if (cal_A_LOCK == 0) {
                cal_A = (EMF_max + DEDT * (THER_max - THER_data)) + log10(Base_line) * (cal_B * cal_B_offset);
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
        if (EMF_max >= EMF_data) {
            upper_cut = 0;
        }
        else if (EMF_max < EMF_data) {
            upper_cut++;
            if (upper_cut > 3) {
                EMF_max = EMF_data;
                THER_max = THER_data;
                upper_cut = 0;
            }
        }
        prev_time_METI = sec_1cnt;
    }
}
void RX9Simple::DMG_REC()
{
    for (int i = 0; i < 19; i++) {
        cal_A_LOG[0][i] = cal_A_LOG[0][i + 1];
        cal_A_LOG[1][i] = cal_A_LOG[1][i + 1];
    }
    cal_A_LOG[0][19] = cal_A;
    cal_A_LOG[1][19] = EMF_data;
    if (_status_sensor == 1) {
        if ((cal_A_LOG[1][19] - cal_A_LOG[1][2] >= LOCK_delta) && (cal_A_LOG[1][18] - cal_A_LOG[1][1] >= LOCK_delta) && (cal_A_LOG[1][17] - cal_A_LOG[1][0] >= LOCK_delta)) {
            if (cal_A_LOCK == 0) {
                cal_A_LOCK = 1;
                cal_A_LOCK_value = cal_A_LOG[0][0];
                emf_LOCK_value = cal_A_LOG[1][0];
                cal_A = cal_A_LOCK_value;
                
            }
        }
        else if ((cal_A_LOG[1][2] > 540 - LOCK_delta) && (cal_A_LOG[1][1] > 540 - LOCK_delta) && (cal_A_LOG[1][0] > 540 - LOCK_delta) && (cal_A_LOG[1][2] <= 540 - LOCK_disable) && (cal_A_LOG[1][1] <= 540 - LOCK_disable) && (cal_A_LOG[1][0] <= 540 - LOCK_disable)) {
            if ((cal_A_LOG[1][17] > 540) && (cal_A_LOG[1][18] > 540) && (cal_A_LOG[1][19] > 540)) {
                if (cal_A_LOCK == 0) {
                    cal_A_LOCK = 1;
                    cal_A_LOCK_value = cal_A_LOG[0][0];
                    emf_LOCK_value = cal_A_LOG[1][0];
                    cal_A = cal_A_LOCK_value;
                }
            }
        }
        else {
            //do nothing
        }
    }
    if (cal_A_LOCK == 1) {
        if (EMF_data - emf_LOCK_value < LOCK_disable) {
            S3_cnt++;
            if (S3_cnt >= 10) {
                S3_cnt = 0;
                cal_A_LOCK = 0;
                ELTI = 0;
                THER_ini = THER_data;
                EMF_max = EMF_data;
                THER_max = THER_data;
                LOCK_timer_cnt = 0;
            }
            else {
                //do nothing
            }
        }
        else if (LOCK_timer_cnt >= LOCK_timer) {
            cal_A_LOCK = 0;
            ELTI = 0;
            THER_ini = THER_data;
            EMF_max = EMF_data;
            THER_max = THER_data;
            LOCK_timer_cnt = 0;
        }
        else {
            S3_cnt = 0;
        }
    }
    else {
        //do nothing
    }
}
