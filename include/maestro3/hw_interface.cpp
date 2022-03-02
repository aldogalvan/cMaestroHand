/*
 * CHWInterface.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: Ashish.Lab
 */

#include "hw_interface.h"

HW_Interface::HW_Interface() {
        // TODO Auto-generated constructor stub
    printf("Constructing HW_Interface...\n");
        exo_joint_angle_sensor_rad_array=0;
        motor_angle_tick_array=0;
        motor_angle_rad_array=0;
        motor_set_value_tick_array=0;
        motor_set_value_rad_array=0;
        //paria's addition
        exo_joint_angle_sensor_volts_array=0;
        printf("HW_Interface Constructed!!!\n");
}

HW_Interface::~HW_Interface() {
        // TODO Auto-generated destructor stub

}

void HW_Interface::UpdateAllSensorData(){
        ScanAllSensorData();
        CalibrateAllSensorData();
}

void HW_Interface::UpdateAllOutputData(){
        ConvertOutputUnit();
        FlushAllOutputData();
}

void HW_Interface::SetMotorSetValueDegArray(double* a,int size_of_a){
        for (int i=0;i<size_of_a;i++){
                motor_set_value_rad_array[i] = a[i];
        }
}
