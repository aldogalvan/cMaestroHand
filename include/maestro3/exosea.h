/*
 * exosea.h
 *
 *  Created on: Jul 2, 2015
 *  Updated on: Jul 12, 2018
 *      Author: Ashish.Lab
 */

#ifndef EXOSEA_H
#define EXOSEA_H

#include "exojointanglesensor.h"
#include "exomotor.h"
#include <iostream>
#include <math.h>

class ExoSEA
{

private:
    ExoJointAngleSensor* sea_joint_angle_sensor;
    //	Exo_Motor sea_motor;
    double sea_joint_compression_spring_stiffness;
    double sea_joint_pulley_radius;
    double sea_motor_pulley_radius;
    double torque_error_integral;
    double torque_PID_kp, torque_PID_kd, torque_PID_ki;
    double lower_motor_angle_limit, upper_motor_angle_limit;
    double initial_sea_joint_angle_rad;
    ExoMotor sea_motor;
public:
    ExoSEA();
    virtual ~ExoSEA();
    void SetJointAngleSensorSEA(ExoJointAngleSensor* sea_joint_angle_sensor_);
    void SetMotorIndexSEA(HW_Interface* pHW_interface_, int sea_motor_index);
    void SetSEAParameters(double sea_joint_compression_spring_stiffness_,double torque_PID_kp_,
    double torque_PID_kd_, double torque_PID_ki_,double lower_motor_angle_limit_, double upper_motor_angle_limit_, double sea_joint_pulley_radius_);
    bool TorqueControl(double desired_torque, double desired_torque_derivative, double &desired_motor_angle_rad, double &actual_torque, int motor_number);
    bool TorqueControlfeedforwardmotor(double desired_torque, double desired_torque_derivative, double &desired_motor_angle_rad, double &actual_torque, int motor_number);
    const double GetMotorAngleRad(){return sea_motor.GetMotorAngleRad();}
    const double GetJointAngleRad(){return sea_joint_angle_sensor->GetExoJointAngleRad();}
    const double GetInitialJointAngleRad(){return initial_sea_joint_angle_rad;}
    double GetActualTorque();
    bool SetDirectMotorSetValueRad(double motor_set_value_rad_){sea_motor.SetMotorSetValueRad( motor_set_value_rad_);return 1;}
    double GetMotorAngleLowerLimit() {return lower_motor_angle_limit;}
    double GetMotorAngleUpperLimit() {return upper_motor_angle_limit;}
    double GetMotorPulleyRadius() {return sea_motor_pulley_radius;}
    double GetJointPulleyRadius() {return sea_joint_pulley_radius;}
    double GetSpringStiffness() {return sea_joint_compression_spring_stiffness;}
    bool ExitSEA(int exit_angle_change);

};

#endif // EXOSEA_H
