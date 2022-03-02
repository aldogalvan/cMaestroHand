#ifndef ESMACAT_MOTORDRIVER_H
#define ESMACAT_MOTORDRIVER_H

#define ESMACAT_MD_MIN_PWM_OUTPUT      0
#define ESMACAT_MD_MAX_PWM_OUTPUT      10000
#define ESMACAT_MD_MAX_SET_VALUE       1
#define ESMACAT_MD_MIN_SET_VALUE       -1
#define ESMACAT_MD_NUMBER_OF_EXT_ANALOG_INPUT       2
#define ESMACAT_MD_NUMBER_OF_ESCON_ANALOG_INPUT     2
#define ESMACAT_MD_PWM_10P_OUTPUT_OFFSET           0

#include "esmacat_base_module.h"
#include "error_ilst_esmacat.h"
#include <iostream>
#include <math.h>

class esmacat_motor_driver : public esmacat_base_module
{
    uint16_t IN_MD_ANLG[2];     // analog input from external source, raw value from ADC
    uint16_t IN_ESCON_ANLG[2];  // analog input from ESCON
    int32_t IN_Quad_encoder;    // quadrature encoder reading value
    bool OUT_MD_enable;         // enable parameter for ESCON enable
    bool OUT_MD_direction;      // motor direction parameter for ESCON
    bool IN_MD_FAULT;           // true if there is any ESCON error
    void ecat_data_process_motor_driver(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs); // EtherCAT communication for motor driver
    float position_control_p_gain;
    float position_control_i_gain;
    float position_control_d_gain;
    float integ_position_error;
    float prev_position_error;
    float max_integ_position_error;
    float max_vel_qc_per_ms;

protected:
    int32_t OUT_setpoint;       // set value for ESCON, this variable is used for both direct control and position control
    int esmacat_motor_driver_control_mode_value;    //  control mode variable
    esmacat_err set_motor_direction(bool direction);                // change the direction of motor
    esmacat_err set_escon_pwm(unsigned int pwm_value);              // pwm_value 10000 is pwm duty cycle 100%
    uint16_t get_IN_MD_ANLG(int ai_index){return IN_MD_ANLG[ai_index];}
    uint16_t get_IN_ESCON_ANLG(int ai_index){return IN_ESCON_ANLG[ai_index];}
    float esmacat_app_one_cycle_time_sec;

public:
    enum esmacat_motor_driver_control_mode{ // control mode is selected among these options
        direct_control,
        position_control
    };
    esmacat_motor_driver();
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop); // EtherCAT communication
    void set_ESCON_enable_switch(bool enable_on);                   // turn ON/OFF enable switch of ESCON
    esmacat_err set_ESCON_setpoint(float setpoint);  // set setvalue of ESCON between -1 and 1
    esmacat_err set_desired_position(int32_t desired_position);     // set desired position with a position controller of ESCON slave
    float get_analog_input_from_external_source(int index_of_analog_input, esmacat_err* err_code = 0);  // get analog input from external sources in mV
    float get_analog_input_from_ESCON(int index_of_analog_input, esmacat_err* err_code = 0);            // get analog input from ESCON in mV
    int32_t get_encoder_counter();  // get encoder counter, unit is qc
    bool get_ESCON_fault_signal();
    void set_system_param_control_mode(esmacat_motor_driver_control_mode mode);                 // select control mode of SEA
    void set_system_param_encoder_clear();
    void set_system_param_universal_output_type(uint16_t value);
    void set_position_control_pid_gain(float p, float d, float i);
    void set_max_allowable_integrated_error_for_position_control(uint32_t max_value){max_integ_position_error;}
    void set_max_velocity_in_position_control_qc_p_ms(uint32_t v){max_vel_qc_per_ms=v;}
    void set_esmacat_app_one_cycle_time_sec(double t){esmacat_app_one_cycle_time_sec = t;}
};
#endif // ESMACAT_MOTORDRIVER_H
