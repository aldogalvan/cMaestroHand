#include "esmacat_analog_input.h"

esmacat_analog_input::esmacat_analog_input()
{
    esmacat_slave_product_id = ESMACAT_ANALOG_INPUT_ID;
    std::cout <<"esmacat Loadcell Interface Object is created" << std::endl;
    for(int i=0;i<16;i++){
        IN_AI[i] = 0;
        raw_data_from_adc[i] = 0;
    }
    for(int i=0;i<16;i++){  // default fsr and offset of esmacat ai board
        ai_full_scale_range[i] = 6.0 * 4096.0;
        ai_offset[i] = -3.0 * 4096.0;
    }
}

void esmacat_analog_input::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){
    ecat_data_process_base_module(ec_slave_outputs,ec_slave_inputs);    // for base module
    ecat_data_process_analog_input(ec_slave_outputs,ec_slave_inputs);   // for anlog input module
}

void esmacat_analog_input::ecat_data_process_analog_input(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs){
        for (int i=0;i<16;i++){
            raw_data_from_adc[i] = *(ec_slave_inputs+7+i*2);            // decode  analog input value
            raw_data_from_adc[i] = (raw_data_from_adc[i] << 8) |  *(ec_slave_inputs+6+i*2); // decode  analog input value
//            printf ("%d: %d\t",i,raw_data_from_adc[i]);
        }
//        printf ("\n");
}

esmacat_err esmacat_analog_input::get_raw_analog_input(int index_of_analog_input, uint16_t* raw_analog_input_value){
    if (index_of_analog_input < 0 || index_of_analog_input > ESMACAT_ANALOG_INPUT_MAX_ANALOG_INPUT_INDEX ){
        return ERR_ANALOG_INPUT_OUT_OF_CHANNEL;
    }
    else{
        *raw_analog_input_value = raw_data_from_adc[index_of_analog_input];
        return NO_ERR;
    }
}

esmacat_err esmacat_analog_input::set_system_param_configure_analog_input( analog_input_config ai_config_ch0to3, analog_input_config ai_config_ch4to7,
                                                                           analog_input_config ai_config_ch8to11, analog_input_config ai_config_ch12to15){
    analog_input_config ai_config_array[4];
    ai_config_array[0] = ai_config_ch0to3;
    ai_config_array[1] = ai_config_ch4to7;
    ai_config_array[2] = ai_config_ch8to11;
    ai_config_array[3] = ai_config_ch12to15;

    for (int i=0;i<4;i++){
        if(ai_config_array[i] == SINGLE_ENDED_0V_POS6V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 1.5;
                ai_offset[i*4+j] = 0.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_0V_POS12V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = 0.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG3V_POS3V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 1.5;
                ai_offset[i*4+j] = -0.75 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG6V_0V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 1.5;
                ai_offset[i*4+j] = -1.5 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG6V_POS6V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = -1.5 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG12V_0V){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = -3.0 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG12V_POS12V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 6.0;
                ai_offset[i*4+j] = 4096.0 * -3.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == DIFF_NEG6V_POS6V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = 4096.0 * -1.5;
                ai_single_ended_or_diff[i*4+j] = DIFFERENTIAL_INPUT;
            }
        }
        else if(ai_config_array[i] == DIFF_NEG12V_POS12V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 6.0;
                ai_offset[i*4+j] = 4096.0 * -3.0;
                ai_single_ended_or_diff[i*4+j] = DIFFERENTIAL_INPUT;
            }
        }
        else if(ai_config_array[i] == DIFF_NEG24V_POS24V ){
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 12.0;
                ai_offset[i*4+j] = 4096.0 * -6.0;
                ai_single_ended_or_diff[i*4+j] = DIFFERENTIAL_INPUT;
            }
        }
        else{
            return  ERR_ANALOG_INPUT_NO_SUCH_CONFIG;
        }
    }

    uint16_t esmacat_analog_input_config = 0;
    esmacat_analog_input_config = ai_config_ch12to15;
    esmacat_analog_input_config = (esmacat_analog_input_config <<4) | ai_config_ch8to11;
    esmacat_analog_input_config = (esmacat_analog_input_config <<4) | ai_config_ch4to7;
    esmacat_analog_input_config = (esmacat_analog_input_config <<4) | ai_config_ch0to3;
    add_system_parameters_in_queue(0x0701,esmacat_analog_input_config);
    return NO_ERR;
}

esmacat_err esmacat_analog_input::set_system_param_configure_analog_input(esmacat_analog_input_option ai_option){
    set_system_param_configure_analog_input(ai_option.ai_config_ch0to3, ai_option.ai_config_ch4to7,
                                            ai_option.ai_config_ch8to11, ai_option.ai_config_ch12to15);
}

esmacat_err esmacat_analog_input::get_analog_input_mV(int index_of_analog_input, float* analog_input_value){
    uint16_t raw_ai_value;
    if ( ai_single_ended_or_diff[index_of_analog_input] == SINGLE_ENDED_INPUT) {
        // do nothing
    }
    else{
        if ( index_of_analog_input %2 ==1)  index_of_analog_input--;
    }
    esmacat_err result_of_get_raw_analog_input = get_raw_analog_input( index_of_analog_input, &raw_ai_value);
    if ( result_of_get_raw_analog_input  != NO_ERR ) {
        *analog_input_value  = 0;
        return result_of_get_raw_analog_input;
    }
    *analog_input_value = ai_offset[index_of_analog_input] + ai_full_scale_range[index_of_analog_input] * ( (float)raw_ai_value/(float)UINT16_MAX );
    return NO_ERR;
}

float esmacat_analog_input::get_analog_input_mV(int index_of_analog_input, esmacat_err* e ){
    float analog_input_value = 0;
    esmacat_err err = get_analog_input_mV(index_of_analog_input,&analog_input_value);
    if (e != NULL) *e = err;
    return analog_input_value;
}
