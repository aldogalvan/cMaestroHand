#include "esmacat_motordriver.h"

esmacat_motor_driver::esmacat_motor_driver()
{
    std::cout <<"EsmaCAT Motor Driver Object is created" << std::endl;
    esmacat_slave_product_id = ESMACAT_MOTOR_DRIVER_ID;
    for (int i=0;i<2;i++){
        IN_MD_ANLG[i] = 0;
        IN_ESCON_ANLG[i] = 0;
    }
    IN_Quad_encoder = 0;
    OUT_MD_enable = 0;
    OUT_MD_direction = 0;
    OUT_setpoint = 0;
    IN_MD_FAULT = 0;
    esmacat_motor_driver_control_mode_value = direct_control;
    integ_position_error = 0;
    prev_position_error = 0;
    position_control_p_gain = 0;
    position_control_i_gain = 0;
    position_control_d_gain = 0;
    max_integ_position_error = 10000.0;
    max_vel_qc_per_ms = 1000;
    esmacat_app_one_cycle_time_sec = 0.001;
}

void esmacat_motor_driver::set_ESCON_enable_switch(bool enable_on){
    OUT_MD_enable  = enable_on;
}

esmacat_err esmacat_motor_driver::set_motor_direction(bool direction){
    OUT_MD_direction = direction;
    return NO_ERR;
}

esmacat_err esmacat_motor_driver::set_escon_pwm(unsigned int pwm_value){
    if (pwm_value > ESMACAT_MD_MAX_PWM_OUTPUT){
        pwm_value = ESMACAT_MD_MAX_PWM_OUTPUT;
        perror("ERR_MOTOR_DRIVER_MAX_PWM_REACHED");
        return ERR_MOTOR_DRIVER_MAX_PWM_REACHED;
    }
    else if(pwm_value < ESMACAT_MD_MIN_PWM_OUTPUT)   {
        pwm_value = ESMACAT_MD_MIN_PWM_OUTPUT;
        perror("ERR_MOTOR_DRIVER_MIN_PWM_REACHED");
        return ERR_MOTOR_DRIVER_MIN_PWM_REACHED;
    }
    else {
        OUT_setpoint = pwm_value;
        return NO_ERR;
    }
}

esmacat_err esmacat_motor_driver::set_ESCON_setpoint(float set_point){
    // set value must be between -1 and 1
    esmacat_err e = NO_ERR;
    if (set_point > ESMACAT_MD_MAX_SET_VALUE ){
        set_point = ESMACAT_MD_MAX_SET_VALUE;
        perror ("ERR_MOTOR_DRIVER_OUT_OF_SET_VALUE_RANGE");
        e= ERR_MOTOR_DRIVER_OUT_OF_SET_VALUE_RANGE;
    }
    else if (set_point < ESMACAT_MD_MIN_SET_VALUE ){
        set_point = ESMACAT_MD_MIN_SET_VALUE ;
        e = ERR_MOTOR_DRIVER_OUT_OF_SET_VALUE_RANGE;
        perror ("ERR_MOTOR_DRIVER_OUT_OF_SET_VALUE_RANGE");
    }

    set_motor_direction(0); // from firmware version EsmaCAT Motor Driver 250 v1.64, PWM changes both the direction and amplitude.
    set_escon_pwm( (set_point+1)/2.0 * 8000 + 1000 - ESMACAT_MD_PWM_10P_OUTPUT_OFFSET );

    return e;
}

float esmacat_motor_driver::get_analog_input_from_external_source(int index_of_analog_input, esmacat_err* err_code){
    if (index_of_analog_input > ESMACAT_MD_NUMBER_OF_EXT_ANALOG_INPUT || index_of_analog_input < 0  ){
        if (err_code!=0) *err_code = ERR_MOTOR_DRIVER_OUT_OF_EXT_ANALOG_INPUT_INDEX;
        return 0.0;
    }
    else{
        if (err_code!=0) *err_code = NO_ERR;
        return 5.0*IN_MD_ANLG[index_of_analog_input]/4096.0;
    }
}

float esmacat_motor_driver::get_analog_input_from_ESCON(int index_of_analog_input, esmacat_err* err_code){
    if (index_of_analog_input > ESMACAT_MD_NUMBER_OF_ESCON_ANALOG_INPUT || index_of_analog_input < 0  ){
        if (err_code!=0) *err_code = ERR_MOTOR_DRIVER_OUT_OF_ESCON_ANALOG_INPUT_INDEX;
        return 0.0;
    }
    else{
        if (err_code!=0) *err_code = NO_ERR;
        return 5.0*IN_ESCON_ANLG[index_of_analog_input]/4096.0;
    }
}

int32_t esmacat_motor_driver::get_encoder_counter(){
    return IN_Quad_encoder;
}

void esmacat_motor_driver::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){
    ecat_data_process_base_module(ec_slave_outputs,ec_slave_inputs);    // for base module
    ecat_data_process_motor_driver(ec_slave_outputs,ec_slave_inputs);   // for motor driver
}

void esmacat_motor_driver::ecat_data_process_motor_driver(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs){
    //  decoding and ecoding ethercat data
    uint8_t temp_digital_in_output_bundle = 0;

    // get digital data, and extract the IN_MD_FAULT
    bool temp_digital_array_unbundled[8];
    temp_digital_in_output_bundle = *(ec_slave_inputs+1);
    for (int i=0;i<8;i++){
        temp_digital_array_unbundled[i]=  (( temp_digital_in_output_bundle & (0b00000001 << i)) != 0);
    }
    IN_MD_FAULT = temp_digital_array_unbundled[0];

    for (int i=0;i<2;i++){
        IN_MD_ANLG[i] = *(ec_slave_inputs+6+1 + 2*i );          // decode external analog input
        IN_MD_ANLG[i] = (IN_MD_ANLG[i] << 8) + *(ec_slave_inputs + 6 + 2*i);
    }
    for (int i=0;i<2;i++){
        IN_ESCON_ANLG[i] = *(ec_slave_inputs+10+1 + 2*i );      // decode escon analog input
        IN_ESCON_ANLG[i] = (IN_ESCON_ANLG[i] << 8) + *(ec_slave_inputs + 10 + 2*i);
    }
    IN_Quad_encoder = *(ec_slave_inputs+17);                    // decode qei input
    IN_Quad_encoder = (IN_Quad_encoder<<8) + *(ec_slave_inputs+16);
    IN_Quad_encoder = (IN_Quad_encoder<<8) + *(ec_slave_inputs+15);
    IN_Quad_encoder = (IN_Quad_encoder<<8) + *(ec_slave_inputs+14);

    // from here, the output is encoded for ethercat
    temp_digital_in_output_bundle = OUT_MD_enable;                      // encode for ESCON enable
    temp_digital_in_output_bundle = temp_digital_in_output_bundle  | (OUT_MD_direction<<1);     // encode for ESCON direction
    *(ec_slave_outputs+1)  = temp_digital_in_output_bundle;             // assign the digital values in the ethercat packet
    *(ec_slave_outputs+6)  = (OUT_setpoint & 0x000000ff) >> 0;          // encode the setpoint
    *(ec_slave_outputs+7)  = (OUT_setpoint & 0x0000ff00) >> 8;
    *(ec_slave_outputs+8)  = (OUT_setpoint & 0x00ff0000) >> 16;
    *(ec_slave_outputs+9)  = (OUT_setpoint & 0xff000000) >> 24;
}


esmacat_err esmacat_motor_driver::set_desired_position(int32_t desired_position){

    int32_t filtered_desired_position =0;       // low-pass filtered desired position
    int32_t curr_pos = get_encoder_counter();   // current angle

    // the three lines below converts a step input into a ramp input
    if ( (desired_position - curr_pos) > max_vel_qc_per_ms) filtered_desired_position = curr_pos + max_vel_qc_per_ms;
    else if ( (desired_position - curr_pos) < -max_vel_qc_per_ms) filtered_desired_position = curr_pos - max_vel_qc_per_ms;
    else filtered_desired_position = desired_position;

    double pos_error = curr_pos-filtered_desired_position;
    double non_filtered_pos_error = curr_pos - desired_position;
    double deriv_error = (non_filtered_pos_error- prev_position_error) / esmacat_app_one_cycle_time_sec;
    prev_position_error = non_filtered_pos_error;
    integ_position_error += pos_error * esmacat_app_one_cycle_time_sec;
    if (integ_position_error > max_integ_position_error) {
        integ_position_error = max_integ_position_error;
        //printf("Hit the max integrated position error\n");
    }

    if (integ_position_error < -max_integ_position_error ) {
        integ_position_error = -max_integ_position_error;
        //printf("Hit the max integrated position error\n");
    }

    float spp = position_control_p_gain*pos_error; // setpoint for p-gain
    float spd = position_control_d_gain*deriv_error; // setpoint for d-gain
    float spi = position_control_i_gain*integ_position_error; // setpoint for i-gain
    float sp = spp + spd + spi;
    if (sp > 1) sp =1;
    if (sp < -1) sp = -1;
    set_ESCON_setpoint( sp );
    return NO_ERR;
}


void esmacat_motor_driver::set_system_param_encoder_clear(){
    add_system_parameters_in_queue(0x0500,0);
}

void esmacat_motor_driver::set_system_param_control_mode(esmacat_motor_driver_control_mode mode){
    esmacat_motor_driver_control_mode_value = mode;
}

void esmacat_motor_driver::set_system_param_universal_output_type(uint16_t value){
    // 1: set value
    // 2: filtered desired position or torque
    // 3: loop count
    add_system_parameters_in_queue(0x060E,value);
}

void esmacat_motor_driver::set_position_control_pid_gain(float p, float d, float i){
    position_control_p_gain = p* 0.001;
    position_control_d_gain = d* 0.001;
    position_control_i_gain = i* 0.001;
}


bool esmacat_motor_driver::get_ESCON_fault_signal(){
    return IN_MD_FAULT;
}
