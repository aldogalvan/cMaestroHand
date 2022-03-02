#include "esmacat_slave.h"

esmacat_slave::esmacat_slave()
{
//    number_of_intelicat_prod = NUMBER_OF_INTELICAT_PRODUCTS;
    slave_index = 0;
}

esmacat_slave::~esmacat_slave(){
    std::cout << "esmacat_slave object is destructed"  << std::endl;
}

void esmacat_slave::set_slave_index(int inx){
    slave_index = inx;
}

void esmacat_slave::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){

}

int esmacat_slave::flush_one_set_of_system_parameters(){
    // set system parameters for multifunction
    if (system_parameter_type_queue.size() == 0 ){
        OUT_system_parameter_type = 0;
        OUT_system_parameter_value = 0;
        return 0;
    }
    else{
        OUT_system_parameter_type = system_parameter_type_queue.front();
        OUT_system_parameter_value = system_parameter_value_queue.front();
//        std::cout << "IN " << IN_system_parameter_type  << " " << IN_system_parameter_value << std::endl;
        if (IN_system_parameter_type == OUT_system_parameter_type ){
            std::cout << "succeed for setup, setup type: " << IN_system_parameter_type  << "  setup value: " << IN_system_parameter_value << std::endl;
            system_parameter_type_queue.pop();
            system_parameter_value_queue.pop();
        }
        return system_parameter_type_queue.size();
    }
}

void esmacat_slave::add_system_parameters_in_queue(uint16_t system_parameter_type, uint16_t system_parameter_value){
    if (system_parameter_type_queue.size() > MAX_BUFFER_OF_SYSTEM_PARAMETER || system_parameter_value_queue.size() > MAX_BUFFER_OF_SYSTEM_PARAMETER){
        std::cout << "You cannot queue the system parameters more than " << MAX_BUFFER_OF_SYSTEM_PARAMETER << std::endl;
    }
    else{
        system_parameter_type_queue.push(system_parameter_type);
        system_parameter_value_queue.push(system_parameter_value);
    }
}

uint16_t esmacat_slave::float_2_uint16(float input){
    union{
        float f_number;
        uint8_t bytes[4];
    }temp_for_conv;
    temp_for_conv.f_number = input;
    return ((temp_for_conv.bytes[2] << 8) | temp_for_conv.bytes[3] );
}

float esmacat_slave::uint16_2_float(uint16_t input){
    union{
        float f_number;
        uint8_t bytes[4];
    }temp_for_conv;
    temp_for_conv.bytes[2] = input >> 8;
    temp_for_conv.bytes[3] = input & 0x00ff;
    return temp_for_conv.f_number;
}
