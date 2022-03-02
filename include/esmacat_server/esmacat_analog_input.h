// version log of esmacat_analog_input_class
// - ver1p0, 20180417, Mok: the class is working correctly, see the example below to understand how to use this class. Simply setup some parameters and get analog input

// //  Example of analog input
//
//void inteliCAT_Application::init(std::vector<InteliCAT_Slave*>* pICAT_slave){
//    ICAT_slave = pICAT_slave;
//    ecat_ai = (esmacat_analog_input*)(*ICAT_slave)[0];
//    cycle_count = 0;
//}

//void inteliCAT_Application::setup(){
//    // do something for initial setup
//    ecat_ai->set_system_param_configure_analog_input(DIFF_NEG24V_POS24V,SINGLE_ENDED_0V_POS12V,DIFF_NEG24V_POS24V,DIFF_NEG24V_POS24V);
//}

//int inteliCAT_Application::loop(double time_elapsed){
//    // do something for the loop
//    cycle_count++;              // counting the loop number
//    esmacat_err err_code;       // to get error code during programming
//    float esmacat_ai;           // analog input value will be stored here

//    ecat_ai->set_usr_led(1);    // turn on user LED
//    if( cycle_count%100 == 1 ){         // let's print once every 100 loop
//        cout << cycle_count << "\t";
//        for (int i=0;i<8;i++){
//            err_code = ecat_ai->get_analog_input_mV(i,&esmacat_ai);    // get analog input
//            if (err_code != NO_ERR) cout << "error code: " << err_code << endl;
//            cout << i << ": " <<  esmacat_ai << "\t";
//        }
//        cout << endl;
//    }
//    return 0;
//}

#ifndef ESMACAT_ANALOG_INPUT_H
#define ESMACAT_ANALOG_INPUT_H

#define ESMACAT_ANALOG_INPUT_MAX_ANALOG_INPUT_INDEX 16
#define ESMACAT_ANALOG_INPUT_CONFIG_REGISTER 0x0701

#include "esmacat_base_module.h"
#include "esmacat_slave.h"
#include "error_ilst_esmacat.h"
#include <iostream>

enum analog_input_config {  // for details, see the datasheet of max1300
    SINGLE_ENDED_NEG3V_POS3V = 0b0001,
    SINGLE_ENDED_NEG6V_0V = 0b0010,
    SINGLE_ENDED_0V_POS6V = 0b0011,
    SINGLE_ENDED_NEG6V_POS6V = 0b0100,
    SINGLE_ENDED_NEG12V_0V = 0b0101,
    SINGLE_ENDED_0V_POS12V = 0b0110,
    SINGLE_ENDED_NEG12V_POS12V = 0b0111,
    DIFF_NEG6V_POS6V = 0b1001,
    DIFF_NEG12V_POS12V = 0b1100,
    DIFF_NEG24V_POS24V = 0b1111
};

struct esmacat_analog_input_option{ // for setting up analog input configuration
    analog_input_config ai_config_ch0to3;
    analog_input_config ai_config_ch4to7;
    analog_input_config ai_config_ch8to11;
    analog_input_config ai_config_ch12to15;
    esmacat_analog_input_option(){                  // Initial setup
        ai_config_ch0to3 = SINGLE_ENDED_0V_POS6V;
        ai_config_ch4to7 = SINGLE_ENDED_0V_POS6V;
        ai_config_ch8to11 = SINGLE_ENDED_0V_POS6V;
        ai_config_ch12to15 = SINGLE_ENDED_0V_POS6V;
    }
};

class esmacat_analog_input: public esmacat_base_module
{
    float IN_AI[16];
    uint16_t raw_data_from_adc[16];
    float ai_offset[16];
    float ai_full_scale_range[16];
    ADC_input_single_ended_diff ai_single_ended_or_diff[16];
public:
    esmacat_analog_input();
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);     // for generic ecat_data_process for all esmacat slaves
    void ecat_data_process_analog_input(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);            // for specifically for analog input class
    esmacat_err get_analog_input_mV(int index_of_analog_input, float* analog_input_value);                 // get analog input value converted to mV
    float get_analog_input_mV(int index_of_analog_input, esmacat_err* e = NULL);                                  // get analog input value converted to mV
    esmacat_err get_raw_analog_input(int index_of_analog_input, uint16_t* raw_analog_input_value);      // get raw analog input value from max1300, unit value
    esmacat_err set_system_param_configure_analog_input( analog_input_config ai_config_ch0to3, analog_input_config ai_config_ch4to7,    // set configuration of EsmaCAT Analog Input Slave board
                                                         analog_input_config ai_config_ch8to11, analog_input_config ai_config_ch12to15);
    esmacat_err set_system_param_configure_analog_input(esmacat_analog_input_option ai_option);    // set configuration of EsmaCAT Analog Input Slave board via input option
};
#endif // ESMACAT_ANALOG_INPUT_H
