#ifndef ESMACAT_LOADCELL_INTERF_H
#define ESMACAT_LOADCELL_INTERF_H

#define ESMACAT_LOADCELL_INTERFACE_MAX_ANALOG_INPUT_INDEX 16

#include "esmacat_base_module.h"
#include "error_ilst_esmacat.h"
#include <iostream>

enum PGA_value {PGA1,PGA2,PGA4,PGA8,PGA16,PGA32,PGA64};
enum loadcell_interface_configuration_address{
    ESMACAT_LOADCELL_INTERF_CONIG_ADDR =0x0601
};

struct esmacat_loadcell_interface_option{
    bool buff_on_ADC_ch_0_7;
    bool buff_on_ADC_ch_8_15;
    int PGA_ch_0_7;
    int PGA_ch_8_15;
    ADC_input_single_ended_diff single_ended_diff_ch_0_1;
    ADC_input_single_ended_diff single_ended_diff_ch_2_3;
    ADC_input_single_ended_diff single_ended_diff_ch_4_5;
    ADC_input_single_ended_diff single_ended_diff_ch_6_7;
    ADC_input_single_ended_diff single_ended_diff_ch_8_9;
    ADC_input_single_ended_diff single_ended_diff_ch_10_11;
    ADC_input_single_ended_diff single_ended_diff_ch_12_13;
    ADC_input_single_ended_diff single_ended_diff_ch_14_15;
    esmacat_loadcell_interface_option(){
        buff_on_ADC_ch_0_7 = false;
        buff_on_ADC_ch_8_15 = false;
        PGA_ch_0_7 = PGA1;
        PGA_ch_8_15 = PGA1;
        single_ended_diff_ch_0_1 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_2_3 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_4_5 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_6_7 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_8_9 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_10_11 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_12_13 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_14_15 = SINGLE_ENDED_INPUT;
    }
 };

class esmacat_loadcell_interface : public esmacat_base_module
{
    int IN_AI[16];
    uint16_t raw_data_from_adc[16];
    int  PGA_param;
    esmacat_loadcell_interface_option li_opt;
public:

    esmacat_loadcell_interface();
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
    void ecat_data_loadcell_interface(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);
    float get_analog_input(int index_of_analog_input);
    esmacat_err get_raw_analog_input(int index_of_analog_input, uint16_t* raw_analog_input_value);
    esmacat_err set_system_param_configure_loadcell_interface_adc(esmacat_loadcell_interface_option li_option);
};

#endif // ESMACAT_LOADCELL_INTERF_H
