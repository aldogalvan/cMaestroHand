/*
 * exosea.cpp
 *
 *  Created on: Jul 2, 2015
 *  Updated on: Jul 12, 2018
 *      Author: Ashish.Lab
 */

#include "exosea.h"

using namespace std;

ExoSEA::ExoSEA()
{
    // TODO Auto-generated constructor stub
    sea_joint_angle_sensor = 0;
    initial_sea_joint_angle_rad = 0;
    sea_joint_compression_spring_stiffness = 0;
    sea_joint_pulley_radius = 0.025/2; // m
    sea_motor_pulley_radius = 0.027/2+0.00066/2;//1.09375*0.0254; // m //From D2 of misumi P/N: MBRAC30-1.5-6
    torque_error_integral = 0;
    torque_PID_kp = 0;
    torque_PID_ki = 0;
    torque_PID_kd = 0;
    lower_motor_angle_limit = 0;
    upper_motor_angle_limit = 0;
}

ExoSEA::~ExoSEA() {
    // TODO Auto-generated destructor stub
}

void ExoSEA::SetJointAngleSensorSEA(ExoJointAngleSensor* sea_joint_angle_sensor_){
    sea_joint_angle_sensor = sea_joint_angle_sensor_;
    initial_sea_joint_angle_rad = sea_joint_angle_sensor->GetExoJointAngleRad();
}

void ExoSEA::SetMotorIndexSEA(HW_Interface* pHW_interface_, int sea_motor_index){
    sea_motor.SetHWInterface(pHW_interface_);
    sea_motor.AssignIndexOfMotorInHWInterface(sea_motor_index);
}

void ExoSEA::SetSEAParameters(double sea_joint_compression_spring_stiffness_,double torque_PID_kp_,
        double torque_PID_kd_, double torque_PID_ki_,double lower_motor_angle_limit_, double upper_motor_angle_limit_, double sea_joint_pulley_radius_)
{
    sea_joint_compression_spring_stiffness = sea_joint_compression_spring_stiffness_;
    torque_PID_kp = torque_PID_kp_;
    torque_PID_kd = torque_PID_kd_;
    torque_PID_ki = torque_PID_ki_;
    lower_motor_angle_limit = lower_motor_angle_limit_;
    upper_motor_angle_limit = upper_motor_angle_limit_;
    sea_joint_pulley_radius = sea_joint_pulley_radius_;
}

double ExoSEA::GetActualTorque(){
    double joint_angle_rad, motor_angle_rad, actual_torque;

    // Reading angle data
    joint_angle_rad = sea_joint_angle_sensor->GetExoJointAngleRad();
    motor_angle_rad= sea_motor.GetMotorAngleRad();

    // PID torque control
    actual_torque = 2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius*
            (motor_angle_rad*sea_motor_pulley_radius-((joint_angle_rad-initial_sea_joint_angle_rad)*sea_joint_pulley_radius));

    return actual_torque;
}

bool ExoSEA::TorqueControl(double desired_torque, double desired_torque_derivative, double &desired_motor_angle_rad, double &actual_torque, int motor_number){

    double joint_angle_rad;
    double motor_angle_rad=0;
    double joint_angular_velocity_rad = 0; // rad/s
    double motor_angular_velocity_rad = 0; // rad/s
    double actual_torque_derivative, torque_error, torque_error_derivative;

    // Reading angle data
    joint_angle_rad = sea_joint_angle_sensor->GetExoJointAngleRad();
    motor_angle_rad= sea_motor.GetMotorAngleRad();

    // PID torque control
    actual_torque = 2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius*
            (motor_angle_rad*sea_motor_pulley_radius-((joint_angle_rad-initial_sea_joint_angle_rad)*sea_joint_pulley_radius));

//	cout<<"initial sea joint angle: "<<initial_sea_joint_angle_rad*180/M_PI<<endl;
    actual_torque_derivative = 2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius*
            (motor_angular_velocity_rad*sea_motor_pulley_radius-joint_angular_velocity_rad*sea_joint_pulley_radius);
    torque_error = desired_torque-actual_torque;
    torque_error_derivative = desired_torque_derivative-actual_torque_derivative;
    torque_error_integral = torque_error_integral+torque_error; // Ki should take care of dt, so not multiplying dt

//	cout <<"Torque (Actual: "<<actual_torque<<"  "<<"Desired: "<<desired_torque<< ")" <<endl;

    desired_motor_angle_rad = (1/sea_motor_pulley_radius)*(desired_torque/(2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius)+
            sea_joint_pulley_radius*(joint_angle_rad-initial_sea_joint_angle_rad))+
            torque_PID_kp*(torque_error)+torque_PID_kd*(torque_error_derivative)+
            torque_PID_ki*torque_error_integral;

    // Checking motor angle for limits
//    if(desired_motor_angle_rad>lower_motor_angle_limit && desired_motor_angle_rad<upper_motor_angle_limit)
//    {
////		cout<< "Motor Angle (Desired: "<< desired_motor_angle_rad*180/M_PI<<"  "
////			<< "Actual: "<< sea_motor.GetMotorAngleRad()*180/M_PI<<")"<<endl;
//        sea_motor.SetMotorSetValueRad(desired_motor_angle_rad);
//    }
//    else
//    {
//        cout <<"Desired Motor Angle: "<<desired_motor_angle_rad<<endl;
//        cout<<"Exceeded Motor Angle Limit at Motor Number ==>"<<motor_number<<endl;
//        return false;
//    }

    desired_motor_angle_rad = max(lower_motor_angle_limit,min(upper_motor_angle_limit,desired_motor_angle_rad)); //adjust to keep within motor limits
//    cout <<"Desired Motor Angle: "<<desired_motor_angle_rad<<endl;
    sea_motor.SetMotorSetValueRad(desired_motor_angle_rad);

    if(desired_motor_angle_rad==lower_motor_angle_limit || desired_motor_angle_rad==upper_motor_angle_limit){
        cout<<"Exceeded Motor Angle Limit at Motor Number ==>"<<motor_number<<endl;
    }

    return true;
}

bool ExoSEA::TorqueControlfeedforwardmotor(double desired_torque, double desired_torque_derivative, double &desired_motor_angle_rad, double &actual_torque, int motor_number){

    double joint_angle_rad;
    double motor_angle_rad=0;
//	double joint_angular_velocity_rad = 0; // rad/s
//	double motor_angular_velocity_rad = 0; // rad/s

    // Reading angle data
    joint_angle_rad = sea_joint_angle_sensor->GetExoJointAngleRad();
    motor_angle_rad= sea_motor.GetMotorAngleRad();

    // PID torque control
    actual_torque = 2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius*
            (motor_angle_rad*sea_motor_pulley_radius-((joint_angle_rad-initial_sea_joint_angle_rad)*sea_joint_pulley_radius));

//	cout<<"initial sea joint angle: "<<initial_sea_joint_angle_rad*180/M_PI<<endl;
//	actual_torque_derivative = 2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius*
//			(motor_angular_velocity_rad*sea_motor_pulley_radius-joint_angular_velocity_rad*sea_joint_pulley_radius);
//	torque_error = desired_torque-actual_torque;
//	torque_error_derivative = desired_torque_derivative-actual_torque_derivative;
//	torque_error_integral = torque_error_integral+torque_error; // Ki should take care of dt, so not multiplying dt

//	cout <<"Torque (Actual: "<<actual_torque<<"  "<<"Desired: "<<desired_torque<< ")" <<endl;

//	desired_motor_angle_rad = (1/sea_motor_pulley_radius)*(desired_torque/(2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius)+
//			sea_joint_pulley_radius*(joint_angle_rad-initial_sea_joint_angle_rad));//+
//			torque_PID_kp*(torque_error)+torque_PID_kd*(torque_error_derivative)+
//			torque_PID_ki*torque_error_integral;

    desired_motor_angle_rad = (1/sea_motor_pulley_radius)*(desired_torque/(2*sea_joint_compression_spring_stiffness*sea_joint_pulley_radius));//+


    // Checking motor angle for limits
    if(desired_motor_angle_rad>lower_motor_angle_limit && desired_motor_angle_rad<upper_motor_angle_limit)
    {
//		cout<< "Motor Angle (Desired: "<< desired_motor_angle_rad*180/M_PI<<"  "
//			<< "Actual: "<< sea_motor.GetMotorAngleRad()*180/M_PI<<")"<<endl;
        sea_motor.SetMotorSetValueRad(desired_motor_angle_rad);
    }
    else
    {
        cout <<"Desired Motor Angle: "<<desired_motor_angle_rad<<endl;
        cout<<"Exceeded Motor Angle Limit at Motor Number ==>"<<motor_number<<endl;
        return false;
    }

    return true;
}


bool ExoSEA::ExitSEA(int exit_angle_change){
    return sea_motor.ExitMotor(exit_angle_change);
}
