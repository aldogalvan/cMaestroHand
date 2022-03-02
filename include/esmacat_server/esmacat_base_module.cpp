#include "esmacat_base_module.h"

esmacat_base_module::esmacat_base_module()
{
    IN_SYS_DIG = 0;
    for (int i=0;i<7;i++){
        IN_BASE_DI[i] = 0;
        OUT_BASE_DO[i] = 0;
        ICAT_base_dio_direction[i] = OUTPUT;
    }
    OUT_USR_LED = 0;

    IN_system_parameter_type = 0;
    IN_system_parameter_value = 0;
    OUT_system_parameter_type = 0;
    OUT_system_parameter_value = 0;
}

void esmacat_base_module::set_usr_led(bool led_on){
    OUT_USR_LED = led_on;
}

void esmacat_base_module::set_digital_output_base_module(int index_of_digital_output, bool digital_output_value){
    if (index_of_digital_output < 0 || index_of_digital_output>ESMACAT_BASE_NUMBER_OF_DIO){
        perror("Error: Base DIO index is out of range");
    }
    else{
        if (ICAT_base_dio_direction[index_of_digital_output] != OUTPUT ){
            perror("Error: direction of DIO is not output");
        }
        else{
            OUT_BASE_DO[index_of_digital_output] = digital_output_value;
        }
    }
}

bool esmacat_base_module::get_digital_input_base_module(int index_of_digital_input){
    if (index_of_digital_input < 0 || index_of_digital_input >ESMACAT_BASE_NUMBER_OF_DIO){
        perror("Error: Base DIO index is out of range");
        return 0;
    }
    else{
        if (ICAT_base_dio_direction[index_of_digital_input] != INPUT ){
            perror("Error: direction of DIO is not input");
            return 0;
        }
        else{
            return IN_BASE_DI[index_of_digital_input];
        }
    }
}

void esmacat_base_module::ecat_data_process_base_module(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs){
    // decoding and ecoding ethercat data
    // ec_slave_inputs+0
    // ec_slave_inputs+2 - ec_slave_inputs+5
    // ec_slave_outputs+0
    // ec_slave_outputs+2 - ec_slave_outputs+5
    // are reserved for base module

    uint8_t temp_digital_in_output_bundle = 0;
    bool temp_digital_array_unbundled[8];

    temp_digital_in_output_bundle = *ec_slave_inputs;
    for (int i=0;i<8;i++){
        temp_digital_array_unbundled[i]=  (( temp_digital_in_output_bundle & (0b00000001 << i)) != 0);
    }
    IN_SYS_DIG = temp_digital_array_unbundled[0];               // decode  IN_SYS_DIG
    for (int i=0;i<7;i++){
        IN_BASE_DI[i]=  temp_digital_array_unbundled[i+1];      // decode  IN_BASE_DI
    }


    IN_system_parameter_type = *(ec_slave_inputs+3);            // decode  system_parameter_type
    IN_system_parameter_type = (IN_system_parameter_type << 8) +  *(ec_slave_inputs+2);

    IN_system_parameter_value= *(ec_slave_inputs+5);            // decode  system_parameter_value
    IN_system_parameter_value = (IN_system_parameter_value << 8) +  *(ec_slave_inputs+4);

//    printf("Rec %04X %04X \n", IN_system_parameter_type, IN_system_parameter_value); // gives 12AB

    // from here, the output is encoded for ethercat
    temp_digital_in_output_bundle = 0;
    temp_digital_in_output_bundle = OUT_USR_LED;                        // encode for usr led
    for (int i=0;i<7;i++){
        temp_digital_in_output_bundle = temp_digital_in_output_bundle  | (OUT_BASE_DO[i] << i+1);
    }
    *(ec_slave_outputs) = temp_digital_in_output_bundle ;               // encode for base digital output

    *(ec_slave_outputs+2)  = OUT_system_parameter_type & 0x00ff;        // encode system param type
    *(ec_slave_outputs+3)  = OUT_system_parameter_type >> 8;
    *(ec_slave_outputs+4)  = OUT_system_parameter_value & 0x00ff;       // encode system param value
    *(ec_slave_outputs+5)  = OUT_system_parameter_value >> 8;

//    printf("Sent %04X %04X \n", OUT_system_parameter_type, OUT_system_parameter_value); // gives 12AB
}

void esmacat_base_module::set_system_param_base_module_dio_direction(ICAT_DIO_DIRECTION dio_direction[7]){
    uint16_t dio_direction_bit_array = 0;
    for (int i=0;i<7;i++){
        ICAT_base_dio_direction[i] = dio_direction[i];
        dio_direction_bit_array = dio_direction_bit_array | (dio_direction[i] << i);
    }
    add_system_parameters_in_queue(0x0011,dio_direction_bit_array);
}

void esmacat_base_module::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){

}
