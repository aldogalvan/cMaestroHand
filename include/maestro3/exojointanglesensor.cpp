/*
 * CExoJointAngleSensor.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: Ashish.Lab
 */

#include "exojointanglesensor.h"

ExoJointAngleSensor::ExoJointAngleSensor()
{
    // TODO Auto-generated constructor stub
    exo_joint_angle_rad=0;
    pHW_interface = 0;
    //Paria's addition
    exo_joint_angle_volt =0;

}

ExoJointAngleSensor::~ExoJointAngleSensor() {
    // TODO Auto-generated destructor stub
}

void ExoJointAngleSensor::AssignIndexOfExoJointAngleSensorInHWInterface(int index_of_exo_joint_angle_sensor_in_hw_interface){
    // connect the sensor value of this to sensor value of HW interface by pointer

    // is adding the index ok? Does it give the correct value?
    exo_joint_angle_rad = pHW_interface->GetExoJointAngleSensorRadArray() + index_of_exo_joint_angle_sensor_in_hw_interface;
    exo_joint_angle_volt = pHW_interface->GetExoJointAngleSensorVoltArray() + index_of_exo_joint_angle_sensor_in_hw_interface;

}
