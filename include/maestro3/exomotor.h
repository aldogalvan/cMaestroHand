/*
 * CExoMotor.h
 *
 *  Created on: Jul 2, 2015
 *      Author: Ashish.Lab
 */

#ifndef EXOMOTOR_H
#define EXOMOTOR_H

#include "hw_interface.h"
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

class ExoMotor
{
    double* motor_set_value_rad;
    double* motor_angle_rad;
    HW_Interface* pHW_interface;
    int index_of_motor;
public:
    virtual ~ExoMotor();
    ExoMotor();
    ExoMotor(HW_Interface* pHW_interface_);
    void SetHWInterface(HW_Interface* pHW_interface_){	pHW_interface=pHW_interface_;	};
    void AssignIndexOfMotorInHWInterface(int index_of_motor_in_hw_interface);
    bool SetMotorSetValueRad(double motor_set_value_rad_);
    const double GetMotorAngleRad(){return *motor_angle_rad;};
    bool ExitMotor(int exit_angle_change);
};

#endif // EXOMOTOR_H
