/*
 * CExoJointAngleSensor.h
 *
 *  Created on: Jul 2, 2015
 *      Author: Ashish.Lab
 */

#ifndef EXOJOINTANGLESENSOR_H
#define EXOJOINTANGLESENSOR_H

#include "hw_interface.h"

class ExoJointAngleSensor
{

private:
    double* exo_joint_angle_rad;
    double* exo_joint_angle_volt;
    HW_Interface* pHW_interface;

public:
    ExoJointAngleSensor();
    virtual ~ExoJointAngleSensor();
    void SetHWInterface(HW_Interface* pHW_interface_){pHW_interface=pHW_interface_;}
    void AssignIndexOfExoJointAngleSensorInHWInterface(int index_of_exo_joint_angle_sensor_in_hw_interface);
    const double GetExoJointAngleRad(){return *exo_joint_angle_rad;}
    const double GetExoJointAngleVolt(){return *exo_joint_angle_volt;}
};

#endif // EXOJOINTANGLESENSOR_H
