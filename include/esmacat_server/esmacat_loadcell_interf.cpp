#include "esmacat_loadcell_interf.h"

esmacat_loadcell_interface::esmacat_loadcell_interface(){
    esmacat_slave_product_id = ESMACAT_LOADCELL_INTERFACE_ID;
    std::cout <<"esmacat Loadcell Interface Object is created" << std::endl;
    for(int i=0;i<16;i++){
        IN_AI[i] = 0;
        raw_data_from_adc[i] = 0;
    }
    PGA_param = 0;
}

void esmacat_loadcell_interface::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){
    ecat_data_process_base_module(ec_slave_outputs,ec_slave_inputs);
    ecat_data_loadcell_interface(ec_slave_outputs,ec_slave_inputs);
}

float esmacat_loadcell_interface::get_analog_input(int index_of_analog_input){
    int16_t int_v = (int16_t)(raw_data_from_adc[index_of_analog_input]);
    return 5000.0 * (float)int_v/ float(INT16_MAX) ;
}

void esmacat_loadcell_interface::ecat_data_loadcell_interface(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs){
//    uint16_t raw_data_from_adc[16];
    for (int i=0;i<16;i++){
        raw_data_from_adc[i] = *(ec_slave_inputs+7+i*2);            // decode  system_parameter_value
        raw_data_from_adc[i] = (raw_data_from_adc[i] << 8) |  *(ec_slave_inputs+6+i*2);
//        printf("%d %d %d\t", *(ec_slave_inputs+7+i*2),*(ec_slave_inputs+6+i*2),raw_data_from_adc[i]);
//        printf("%d: %d\t", i,raw_data_from_adc[i]);
    }
//    printf("\n");
}

esmacat_err esmacat_loadcell_interface::get_raw_analog_input(int index_of_analog_input, uint16_t* raw_analog_input_value){
    if (index_of_analog_input < 0 || index_of_analog_input > ESMACAT_LOADCELL_INTERFACE_MAX_ANALOG_INPUT_INDEX ){
        return ERR_LOADCELL_INTERF_OUT_OF_CHANNEL;
    }
    else{
        *raw_analog_input_value = raw_data_from_adc[index_of_analog_input];
        return NO_ERR;
    }
}
esmacat_err esmacat_loadcell_interface::set_system_param_configure_loadcell_interface_adc(esmacat_loadcell_interface_option li_option){
    uint16_t configuration_param = 0;
    configuration_param |= li_option.single_ended_diff_ch_0_1;
    configuration_param |= li_option.single_ended_diff_ch_2_3<< 1;
    configuration_param |= li_option.single_ended_diff_ch_4_5<< 2;
    configuration_param |= li_option.single_ended_diff_ch_6_7 << 3;
    configuration_param |= li_option.single_ended_diff_ch_8_9 << 4;
    configuration_param |= li_option.single_ended_diff_ch_10_11 << 5;
    configuration_param |= li_option.single_ended_diff_ch_12_13 << 6;
    configuration_param |= li_option.single_ended_diff_ch_14_15 << 7;
    configuration_param |= li_option.PGA_ch_0_7 << 8;
    configuration_param |= li_option.PGA_ch_8_15 << 11;
    configuration_param |= li_option.buff_on_ADC_ch_0_7 << 14;
    configuration_param |= li_option.buff_on_ADC_ch_8_15 << 15;
    add_system_parameters_in_queue(ESMACAT_LOADCELL_INTERF_CONIG_ADDR,configuration_param);
    return NO_ERR;
}
