#include "esmacat_sea_driver.h"


esmacat_sea_driver::esmacat_sea_driver()
{
    std::cout <<"EsmaCAT SEA Driver Object is created by interiting Motor Driver Class" << std::endl;
    esmacat_slave_product_id = ESMACAT_SEA_DRIVER_ID;   // the object needs to know its ID to avoid wrong assignment
    IN_absolute_enc = 0;
    IN_MD_FAULT = 0;
    sea_option.loadcell_zero_offset_mV = 0;
    sea_option.loadcell_calibration_mV_to_mNm = 0;
    sea_option.torque_constant_mNm_per_mA = 0;
    sea_option.sea_gear_ratio = 0;
    sea_option.sea_gear_power_efficiency = 0;
    sea_option.setpoint_to_mA_conversion_in_ESCON = 0;
    sea_option.torque_control_p_gain = 0;
    sea_option.torque_control_i_gain = 0;
    sea_option.torque_control_d_gain = 0;
    sea_option.position_control_p_gain = 0;
    sea_option.position_control_i_gain = 0;
    sea_option.position_control_d_gain = 0;
    esmacat_sea_parameters_ready = 0;
    loadcell_offset_voltage_mV = 0;
    loadcell_calibration_mV_to_mNm = 0;
    integ_torque_error = 0;
    deriv_torque_error = 0;
    prev_torque_error = 0;
    filtered_vel_qc_per_sec = 0;
    filtered_setpoint = 0;
}

void esmacat_sea_driver::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){
    esmacat_motor_driver::ecat_data_process(ec_slave_outputs, oloop, ec_slave_inputs, iloop);   // EtherCAT data process for parent
    ecat_data_process_sea_driver(ec_slave_outputs,ec_slave_inputs); // then for SEA
}

void esmacat_sea_driver::ecat_data_process_sea_driver(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs){
    // get digital data, and extract the IN_MD_FAULT
    uint8_t temp_digital_in_output_bundle = 0;
    bool temp_digital_array_unbundled[8];
    temp_digital_in_output_bundle = *(ec_slave_inputs+1);
    for (int i=0;i<8;i++){
        temp_digital_array_unbundled[i]=  (( temp_digital_in_output_bundle & (0b00000001 << i)) != 0);
    }
    IN_MD_FAULT = temp_digital_array_unbundled[0];
    // get absolute encoder value
    IN_absolute_enc = *(ec_slave_inputs+19);
    IN_absolute_enc = (IN_absolute_enc << 8) + *(ec_slave_inputs + 18);
}

float esmacat_sea_driver::get_analog_input_from_external_source(int index_of_analog_input, esmacat_err* err_code){
    if (index_of_analog_input > ESMACAT_MD_NUMBER_OF_ESCON_ANALOG_INPUT || index_of_analog_input < 0  ){
        if (err_code!= NULL) *err_code = ERR_MOTOR_DRIVER_OUT_OF_EXT_ANALOG_INPUT_INDEX ;
        perror("ERR_MOTOR_DRIVER_OUT_OF_EXT_ANALOG_INPUT_INDEX ");
        return 0.0;
    }
    else{
        if (err_code!= NULL) *err_code = NO_ERR;
        return ESMACAT_SEA_ADC_OFFSET_MV + ESMACAT_SEA_ADC_FSR_MV * (float)(get_IN_MD_ANLG(index_of_analog_input))/(float)UINT16_MAX;
    }
}

float esmacat_sea_driver::get_analog_input_from_ESCON(int index_of_analog_input, esmacat_err* err_code){
    if (index_of_analog_input > ESMACAT_MD_NUMBER_OF_ESCON_ANALOG_INPUT || index_of_analog_input < 0  ){
        if (err_code!=0) *err_code = ERR_MOTOR_DRIVER_OUT_OF_ESCON_ANALOG_INPUT_INDEX;
        perror("ERR_MOTOR_DRIVER_OUT_OF_ESCON_ANALOG_INPUT_INDEX ");
        return 0.0;
    }
    else{
        if (err_code!=0) *err_code = NO_ERR;
        return ESMACAT_SEA_ADC_OFFSET_MV + ESMACAT_SEA_ADC_FSR_MV * (float)(get_IN_ESCON_ANLG(index_of_analog_input))/(float)UINT16_MAX;
    }
}

//void esmacat_sea_driver::set_system_param_loadcell_zero_offset_mV(float value){
//    loadcell_offset_voltage_mV = value;
//    uint16_t loadcell_zero_offset_mV_MSB_of_float  = float_2_uint16(value);
//    add_system_parameters_in_queue(0x0605,loadcell_zero_offset_mV_MSB_of_float );
//}

//void esmacat_sea_driver::set_system_param_loadcell_calibration_mV_to_mNm(float value){
//    loadcell_calibration_mV_to_mNm = value;
//    uint16_t loadcell_calibration_mV_to_mNm_MSB_of_float  = float_2_uint16(value);
//    add_system_parameters_in_queue(0x0606,loadcell_calibration_mV_to_mNm_MSB_of_float);
//}

//void esmacat_sea_driver::set_system_param_torque_constant_mNm_per_mA(uint16_t value){
//    add_system_parameters_in_queue(0x0607,value);
//}

//void esmacat_sea_driver::set_system_param_sea_gear_ratio (uint16_t value){
//    add_system_parameters_in_queue(0x0608,value);
//}

//void esmacat_sea_driver::set_system_param_gear_power_efficiency_percentage(uint16_t value){
//    add_system_parameters_in_queue(0x0609,value);
//}

//void esmacat_sea_driver::set_system_param_setpoint_to_mA_conversion_in_ESCON(uint16_t value){
//    sea_option.setpoint_to_mA_conversion_in_ESCON = value;
//    add_system_parameters_in_queue(0x060A,value);
//}

//void esmacat_sea_driver::set_system_param_torque_control_p_gain(float value){
//    uint16_t value_converted_into_uint16 = float_2_uint16(value);
//    add_system_parameters_in_queue(0x060B,value_converted_into_uint16 );
//}

//void esmacat_sea_driver::set_system_param_torque_control_i_gain(float value){
//    uint16_t value_converted_into_uint16 = float_2_uint16(value);
//    add_system_parameters_in_queue(0x060C,value_converted_into_uint16 );
//}

//void esmacat_sea_driver::set_system_param_torque_control_d_gain(float value){
//    uint16_t value_converted_into_uint16 = float_2_uint16(value);
//    add_system_parameters_in_queue(0x060D,value_converted_into_uint16 );
//}

void esmacat_sea_driver::set_system_param_universal_output_type(uint16_t value){
    // 1: set value
    // 2: filtered desired position or torque
    add_system_parameters_in_queue(0x060E,value);
}

void esmacat_sea_driver::set_system_param_position_control_max_velocity_qc_p_loop(uint16_t value){
    add_system_parameters_in_queue(0x060F,value);
}

void esmacat_sea_driver::set_system_param_torque_control_max_diff_mNm_p_loop(uint16_t value){
    add_system_parameters_in_queue(0x0610,value);
}


void esmacat_sea_driver::set_system_param_control_mode(esmacat_motor_driver_control_mode mode){
    esmacat_motor_driver_control_mode_value = mode;
//    add_system_parameters_in_queue(0x0601,mode);
}

esmacat_err esmacat_sea_driver::set_desired_torque_mNm(float desired_torque_mNm){
    if (desired_torque_mNm - get_filtered_load_mNm() > 1000) desired_torque_mNm  = get_filtered_load_mNm()+1000;
    if (desired_torque_mNm - get_filtered_load_mNm() < -1000) desired_torque_mNm  = get_filtered_load_mNm()-1000;

    float p = get_encoder_counter();
    float vel_qc_per_sec = (p - prev_pos) / esmacat_app_one_cycle_time_sec;
    prev_pos = p;
    filtered_vel_qc_per_sec = 0.0*filtered_vel_qc_per_sec+1.0*vel_qc_per_sec;
    float demanded_motor_torque_mNm = desired_torque_mNm/((float)sea_gear_ratio)/(sea_gear_power_efficiency);
    float setpoint_feedforward =demanded_motor_torque_mNm/torque_constant_mNm_per_mA/setpoint_to_mA_conversion_in_ESCON;
    float threshold_velocity_for_friction_compensation = 5000;
//    float friction_compensation_setpoint = 0.01;
    float friction_compensation_setpoint = 0.0;
    if (filtered_vel_qc_per_sec < threshold_velocity_for_friction_compensation && filtered_vel_qc_per_sec > -threshold_velocity_for_friction_compensation){ // static condition
        // do nothing;
    }
    else{
        if (filtered_vel_qc_per_sec > threshold_velocity_for_friction_compensation){
            setpoint_feedforward -= friction_compensation_setpoint;
        }
        else if (filtered_vel_qc_per_sec < -threshold_velocity_for_friction_compensation){
            setpoint_feedforward += friction_compensation_setpoint;
        }
    }
    float load_err = get_filtered_load_mNm() - desired_torque_mNm;
    float setpoint_feedback_p_gain = torque_control_p_gain * load_err;
    integ_torque_error += load_err * esmacat_app_one_cycle_time_sec;
    if(integ_torque_error > 1000) integ_torque_error = 1000;
    else if(integ_torque_error < -1000) integ_torque_error = -1000;
    float setpoint_feedback_i_gain = torque_control_i_gain * integ_torque_error;
    deriv_torque_error = (load_err - prev_torque_error)/esmacat_app_one_cycle_time_sec;
    prev_torque_error = load_err;
    float setpoint_feedback_d_gain = torque_control_d_gain * deriv_torque_error;
    float setpoint_feedback = setpoint_feedback_p_gain + setpoint_feedback_i_gain + setpoint_feedback_d_gain;
//    printf("p:%f, i:%f\t d:%f\t",setpoint_feedback_p_gain,setpoint_feedback_i_gain,setpoint_feedback_d_gain);
    float setpoint = setpoint_feedback + setpoint_feedforward;
//    float setpoint = setpoint_feedback ;
//    set_ESCON_setpoint(setpoint);

    filtered_setpoint = 0.00*filtered_setpoint + 1.00*setpoint;
    set_ESCON_setpoint(filtered_setpoint);


}

bool esmacat_sea_driver::get_ESCON_fault_signal(){
    return IN_MD_FAULT;
}

uint16_t esmacat_sea_driver::get_absolute_encoder_raw(){
    return  IN_absolute_enc;
}

esmacat_err esmacat_sea_driver::set_system_param_sea_option(esmacat_sea_driver_option* op){
    loadcell_calibration_mV_to_mNm = op->loadcell_calibration_mV_to_mNm;
    loadcell_zero_offset_mV = op->loadcell_zero_offset_mV;
    sea_gear_power_efficiency = op->sea_gear_power_efficiency;
    sea_gear_ratio = op->sea_gear_ratio;
    setpoint_to_mA_conversion_in_ESCON = op->setpoint_to_mA_conversion_in_ESCON;
    torque_constant_mNm_per_mA = op->torque_constant_mNm_per_mA;
    torque_control_p_gain = op->torque_control_p_gain*0.001;
    torque_control_d_gain = op->torque_control_d_gain*0.001;
    torque_control_i_gain = op->torque_control_i_gain*0.001;

//    bool error_flag = 0;
//    // detect any problem in the option
//    if ( op->loadcell_calibration_mV_to_mNm == 0 ) error_flag = 1;
//    if ( op->torque_constant_mNm_per_mA == 0 ) error_flag = 1;
//    if ( op->sea_gear_ratio == 0 ) error_flag = 1;
//    if ( op->sea_gear_power_efficiency_percentage == 0 ) error_flag = 1;
//    if ( op->setpoint_to_mA_conversion_in_ESCON == 0 ) error_flag = 1;
//    // if there is any error
//    if (error_flag) {
//        perror ("ERR_MOTOR_DRIVER_PARAMETER_EXPORT");
//        return ERR_MOTOR_DRIVER_INSUFFICIENT_PARAMETER;
//    }
//    else{
//        set_system_param_loadcell_zero_offset_mV(op->loadcell_zero_offset_mV);
//        set_system_param_loadcell_calibration_mV_to_mNm(op->loadcell_calibration_mV_to_mNm);
//        set_system_param_torque_constant_mNm_per_mA(op->torque_constant_mNm_per_mA);
//        set_system_param_sea_gear_ratio(op->sea_gear_ratio);
//        set_system_param_gear_power_efficiency_percentage(op->sea_gear_power_efficiency_percentage);
//        set_system_param_setpoint_to_mA_conversion_in_ESCON(op->setpoint_to_mA_conversion_in_ESCON);

//        set_system_param_torque_control_p_gain(op->torque_control_p_gain);
//        set_system_param_torque_control_i_gain(op->torque_control_i_gain);
//        set_system_param_torque_control_d_gain(op->torque_control_d_gain);
//        esmacat_sea_parameters_ready = 1;
//        return NO_ERR;
//    }
}

float esmacat_sea_driver::get_load_mNm(){
    float measured_load_cell_voltage_mV = get_analog_input_from_external_source(0);
    if ( loadcell_calibration_mV_to_mNm == 0){
        perror("ERR_SEA_LOADCELL_PARAMETER_NOT_LOADED");
    }
    return (measured_load_cell_voltage_mV - loadcell_offset_voltage_mV) * loadcell_calibration_mV_to_mNm;
}

float esmacat_sea_driver::get_filtered_load_mNm(){
    filtered_load_mNm = filtered_load_mNm*0.00 + get_load_mNm()*1.00;
    return filtered_load_mNm;
}

void esmacat_sea_driver::set_torque_control_pid_gain(float p, float d, float i){
    torque_control_p_gain = p;
    torque_control_d_gain = d;
    torque_control_i_gain = i;
}
