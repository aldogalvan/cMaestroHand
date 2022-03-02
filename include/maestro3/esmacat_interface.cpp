#include "esmacat_interface.h"

Esmacat_Interface::Esmacat_Interface()
{

    printf("Esmacat_Interface Constructed!!!\n");
    //    time_t t = time(0);                   FILE WRITING FOR DEBUG
    //    struct tm* now = localtime(&t);
    //    char buffer[80];
    //    strftime(buffer,80,"MotorTest_%Y%m%d%H%M%S.csv",now);
    //    myfile.open ((const char*)(buffer),ios::out);
    //    cycle_count = 0;
    //    state = 0;
}


Esmacat_Interface::~Esmacat_Interface(){
//    myfile.close();
}

void Esmacat_Interface::esmacatInterfaceConnect(esmacat_motor_driver* pDriverArray[]){

    md_array = pDriverArray; //assign constant pointer of motor driver array from esmacat app to pointer for use in hw_interface
}

void Esmacat_Interface::esmacatInterfaceSetup(){

    //initial setup
    ICAT_DIO_DIRECTION dig_io[7] = {INPUT,INPUT,INPUT,INPUT,INPUT,INPUT,INPUT};

    //TODO: Implement tuning procedure for motors then set gains based on individual tuning

    for(int i=0; i< NUMBER_OF_DRIVERS; i++){
        md_array[i]->set_system_param_base_module_dio_direction(dig_io);
        md_array[i]->set_position_control_pid_gain(0.8,0.12,0); //Conservative gains: 0.12, 0.01, 0
        md_array[i]->set_max_velocity_in_position_control_qc_p_ms(4000); //8000
        md_array[i]->set_system_param_control_mode(esmacat_motor_driver::position_control);
        md_array[i]->set_system_param_encoder_clear();
    }

}


void Esmacat_Interface::getEncoderData(double* enc_raw_data){
    for(int i = 0; i < NUMBER_OF_DRIVERS; i++){
        enc_raw_data[i] = md_array[i]->get_encoder_counter();
    }
}

void Esmacat_Interface::getAnalogSensorData(double* analog_raw_data){
    int j = 0; //sensor index
    for(int i = 0; i < NUMBER_OF_DRIVERS; i++){
        analog_raw_data[j] = md_array[i]->get_analog_input_from_external_source(0);
        j++;
        analog_raw_data[j] = md_array[i]->get_analog_input_from_external_source(1);
        j++;
    }
}

void Esmacat_Interface::getDigitalSensorData(bool* digital_raw_data){
    for(int i = 0; i < NUMBER_OF_DRIVERS; i++){
        digital_raw_data[i] = md_array[i]->get_digital_input_base_module(0);
    }
}
bool Esmacat_Interface::isESTOP(){
    // TODO: Find stable electrical implementation
    return !md_array[0]->get_digital_input_base_module(1); //ESTOP indicator pin
}
void Esmacat_Interface::setMotorPositions(double* motor_setpoints){
    for(int i = 0; i < NUMBER_OF_DRIVERS; i++){
        md_array[i]->set_ESCON_enable_switch(1);
        md_array[i]->set_desired_position((int32_t)motor_setpoints[i]);
    }
}
