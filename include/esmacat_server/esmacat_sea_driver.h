#ifndef ESMACAT_SEA_DRIVER_H
#define ESMACAT_SEA_DRIVER_H

#define ESMACAT_SEA_ADC_OFFSET_MV   -1.5*4096.0
#define ESMACAT_SEA_ADC_FSR_MV      3.0*4096.0

#include "esmacat_motordriver.h"

struct esmacat_sea_driver_option{
    float loadcell_zero_offset_mV;          // offset mV in loadcell reading
    float loadcell_calibration_mV_to_mNm;    // calibration parameter, slope mV to Nm
    float torque_constant_mNm_per_mA;     // torque constant of Motor
    int sea_gear_ratio;                // gear ratio
    float sea_gear_power_efficiency;  // gear power efficiency
    float setpoint_to_mA_conversion_in_ESCON;    // ESCON setup parameter
    float torque_control_p_gain;
    float torque_control_i_gain;
    float torque_control_d_gain;
    float position_control_p_gain;
    float position_control_i_gain;
    float position_control_d_gain;
    esmacat_sea_driver_option()
    {
        loadcell_zero_offset_mV = 0;          // offset mV in loadcell reading
        loadcell_calibration_mV_to_mNm = 0;    // calibration parameter, slope mV to Nm
        torque_constant_mNm_per_mA = 0;     // torque constant of Motor
        sea_gear_ratio = 0;                // gear ratio
        sea_gear_power_efficiency = 0;  // gear power efficiency
        setpoint_to_mA_conversion_in_ESCON = 0;    // ESCON setup parameter
        torque_control_p_gain = 0;
        torque_control_i_gain = 0;
        torque_control_d_gain = 0;
        position_control_p_gain = 0;
        position_control_i_gain = 0;
        position_control_d_gain = 0;
    }
};

class esmacat_sea_driver : public esmacat_motor_driver{

    void ecat_data_process_sea_driver(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs); // ethercat data only for sea driver is processed here
    bool IN_MD_FAULT;   // true if there is any ESCON error
    uint16_t IN_absolute_enc;   // absolute encoder data, range is 0 - 4095 for 360 deg
    esmacat_sea_driver_option sea_option;
    bool esmacat_sea_parameters_ready;
    float loadcell_offset_voltage_mV;
    float loadcell_calibration_mV_to_mNm;
    float loadcell_zero_offset_mV;          // offset mV in loadcell reading
    float torque_constant_mNm_per_mA;     // torque constant of Motor
    int sea_gear_ratio;                // gear ratio
    float sea_gear_power_efficiency;  // gear power efficiency
    float setpoint_to_mA_conversion_in_ESCON;    // ESCON setup parameter
    float torque_control_p_gain;
    float torque_control_i_gain;
    float torque_control_d_gain;
    float position_control_p_gain;
    float position_control_i_gain;
    float position_control_d_gain;
    float integ_torque_error;
    float deriv_torque_error;
    float prev_torque_error;
    float filtered_vel_qc_per_sec;
    float filtered_load_mNm;
    float filtered_setpoint;
    int32_t prev_pos;

//    void set_system_param_loadcell_zero_offset_mV(float value);
//    void set_system_param_loadcell_calibration_mV_to_mNm(float value);
//    void set_system_param_torque_constant_mNm_per_mA(uint16_t value);
//    void set_system_param_sea_gear_ratio (uint16_t value);
//    void set_system_param_gear_power_efficiency_percentage(uint16_t value);
//    void set_system_param_setpoint_to_mA_conversion_in_ESCON(uint16_t value);
//    void set_system_param_torque_control_p_gain(float p_gain);
//    void set_system_param_torque_control_i_gain(float i_gain);
//    void set_system_param_torque_control_d_gain(float d_gain);
public:
    esmacat_sea_driver();   // constructor
    enum esmacat_motor_driver_control_mode{ // control mode of sea driver
        direct_control,
        position_control,
        torque_control
    };
    esmacat_err set_system_param_sea_option(esmacat_sea_driver_option* op);
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop); // ethercat data process
    float get_analog_input_from_external_source(int index_of_analog_input, esmacat_err* err_code = NULL);  // get analog input from external source
    float get_analog_input_from_ESCON(int index_of_analog_input, esmacat_err* err_code = NULL);    // escon values, range is from -6000mV to +6000mV
    float get_load_mNm();
    float get_filtered_load_mNm();
    esmacat_err set_desired_torque_mNm(float desired_torque_mNm);                                 // set desired torque in mNm, the reference is loadcell value
    void set_system_param_control_mode(esmacat_motor_driver_control_mode mode);                 // select control mode of SEA
    bool get_ESCON_fault_signal();
    uint16_t get_absolute_encoder_raw();
    void set_system_param_universal_output_type(uint16_t value);
    void set_system_param_position_control_max_velocity_qc_p_loop(uint16_t value);
    void set_system_param_torque_control_max_diff_mNm_p_loop(uint16_t value);
    void set_torque_control_pid_gain(float p, float d, float i);


};
#endif // ESMACAT_SEA_DRIVER_H
