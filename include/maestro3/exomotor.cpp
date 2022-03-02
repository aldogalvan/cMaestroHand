/*
 * CExoMotor.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: Ashish.Lab
 */

#include "exomotor.h"

ExoMotor::ExoMotor()
{
    pHW_interface = 0;
    motor_set_value_rad = 0;
    motor_angle_rad = 0;
    index_of_motor=0;
}

ExoMotor::ExoMotor(HW_Interface* pHW_interface_){
    ExoMotor();
    SetHWInterface(pHW_interface_);
}

ExoMotor::~ExoMotor() {
    // TODO Auto-generated destructor stub

}

void ExoMotor::AssignIndexOfMotorInHWInterface(int index_of_motor_in_hw_interface){
    // connect the set_value in HW interface and the set value of this
    double* pointer_motor_set_value_rad_array;
    double* pointer_motor_angle_rad_array;
    pointer_motor_set_value_rad_array = pHW_interface->GetMotorSetValueRadArray();
    pointer_motor_angle_rad_array = pHW_interface->GetMotorAngleRadArray();
    motor_set_value_rad = pointer_motor_set_value_rad_array + index_of_motor_in_hw_interface;
    motor_angle_rad =pointer_motor_angle_rad_array + index_of_motor_in_hw_interface;
    index_of_motor = index_of_motor_in_hw_interface;
}

bool ExoMotor::SetMotorSetValueRad(double motor_set_value_rad_){
    *motor_set_value_rad = motor_set_value_rad_;
//	cout << "set motor value: " << *motor_set_value_rad << endl;
//	cout << "from pointer: " << *(pHW_interface->GetMotorSetValueRadArray() + 6 ) << endl;
    return true;
}

bool ExoMotor::ExitMotor(int exit_angle_change){


    cout<<"Motor " << index_of_motor << "'s final motor pos: " <<  GetMotorAngleRad()<<endl;

    return true;
}
