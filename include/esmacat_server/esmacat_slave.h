#ifndef ESMACAT_SLAVE_H
#define ESMACAT_SLAVE_H

#define ESMACAT_MULITFUCTION_ID         0x00010101
#define ESMACAT_MOTOR_DRIVER_ID         0x00020101
#define ESMACAT_SEA_DRIVER_ID           0x00020301
#define ESMACAT_LOADCELL_INTERFACE_ID   0x00030101
#define ESMACAT_ANALOG_INPUT_ID         0x00040101

#define MAX_BUFFER_OF_SYSTEM_PARAMETER  30

#include <iostream>
#include <queue>
#include <vector>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

//typedef enum {
//    ICAT_BASE_MODULE,
//    ICAT_MF_MODULE,
//    ICAT_MD_MODULE,
//    ICAT_LI_MODULE
//}ICAT_Module_name;

typedef enum{
    INPUT,
    OUTPUT
}ICAT_DIO_DIRECTION;

enum ADC_input_single_ended_diff {
    SINGLE_ENDED_INPUT,
    DIFFERENTIAL_INPUT
};

class esmacat_slave
{
    int slave_index;
protected:
    uint32_t esmacat_slave_product_id;
public:
    uint16_t IN_system_parameter_type;
    uint16_t IN_system_parameter_value;
    uint16_t OUT_system_parameter_type;
    uint16_t OUT_system_parameter_value;
    std::queue <uint16_t> system_parameter_type_queue;
    std::queue <uint16_t> system_parameter_value_queue;
    esmacat_slave();

    void set_slave_index(int inx);
    virtual void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
    virtual ~esmacat_slave();
    virtual int flush_one_set_of_system_parameters();
    virtual void add_system_parameters_in_queue(uint16_t system_parameter_type, uint16_t system_parameter_value);
    uint16_t float_2_uint16(float input);
    float uint16_2_float(uint16_t input);
    uint32_t get_esmacat_product_id(){return esmacat_slave_product_id;}
};

#endif // ESMACAT_SLAVE_H
