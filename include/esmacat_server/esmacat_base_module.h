#ifndef ESMACAT_BASE_MODULE_H
#define ESMACAT_BASE_MODULE_H

#define ESMACAT_BASE_NUMBER_OF_DIO 7

#include <iostream>
#include "esmacat_slave.h"

class esmacat_base_module : public esmacat_slave{
private:
    bool IN_SYS_DIG;        // System digital input
    bool IN_BASE_DI[7];     // Digital input of base module
    bool OUT_USR_LED;       // User led on base module
    bool OUT_BASE_DO[7];    // Digital output of base module
    bool ICAT_base_dio_direction[7];


public:
    esmacat_base_module();
    virtual void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
    bool get_digital_input_base_module(int index_of_digital_input);
    void set_digital_output_base_module(int digital_output_index, bool digital_output_value);
    void set_usr_led(bool led_on);
    void ecat_data_process_base_module(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);
    void set_system_param_base_module_dio_direction(ICAT_DIO_DIRECTION dio_direction[7]);
};

#endif // ESMACAT_BASE_MODULE_H



