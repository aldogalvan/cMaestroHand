//
// Created by aldo on 11/23/21.
//

#ifndef MAESTRO_HAPTICS_MY_APP_H
#define MAESTRO_HAPTICS_MY_APP_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <ctime>
#include <string.h>
#include <fcntl.h>
#include "hwinterface_esmacat.h"
#include "esmacat_application.h"
#include "exojointanglesensor.h"
#include "exomotor.h"
#include "exosea.h"


using namespace std;

class my_app : public esmacat_application
{
private:

    // * Communication Setup *//
    void connect();
    void setup();
    void loop();

    // Pointers to the Esmacat drivers
    esmacat_motor_driver* ecat_md1;
    esmacat_motor_driver* ecat_md2;
    esmacat_motor_driver* ecat_md3;
    esmacat_motor_driver* ecat_md4;
    esmacat_motor_driver* ecat_md5;
    esmacat_motor_driver* ecat_md6;
    esmacat_motor_driver* ecat_md7;
    esmacat_motor_driver* ecat_md8;
    esmacat_motor_driver* ecat_array[NUMBER_OF_DRIVERS];

    struct Index_Finger
    {
        // Mayble use later if want a cleaner code
        // vector<ExoJointAngleSensor*> sensor_array;

        ExoJointAngleSensor exo_joint_angle_sensor_MCP_abad;
        ExoJointAngleSensor exo_joint_angle_sensor_MCP_fe;
        ExoJointAngleSensor exo_joint_angle_sensor_MCP_PIP;
        ExoJointAngleSensor exo_joint_angle_sensor_PIP;
        ExoJointAngleSensor exo_joint_angle_sensor_DIP;
        ExoSEA exo_sea_MCP;
        ExoSEA exo_sea_PIP;

        // smoothed sensor values
        double smoothed_sensor_values[5] = {0,0,0,0,0};
    } indexFinger;


    struct Middle_Finger
    {
        //vector<ExoJointAngleSensor*> sensor_array;

        ExoJointAngleSensor exo_joint_angle_sensor_MCP_abad;
        ExoJointAngleSensor exo_joint_angle_sensor_MCP_fe;
        ExoJointAngleSensor exo_joint_angle_sensor_MCP_PIP;
        ExoJointAngleSensor exo_joint_angle_sensor_PIP;
        ExoJointAngleSensor exo_joint_angle_sensor_DIP;
        ExoSEA exo_sea_MCP;
        ExoSEA exo_sea_PIP;

        // smoothed sensor values
        double smoothed_sensor_values[5] = {0,0,0,0,0};

    } middleFinger;

    struct Thumb
    {
        // vector<ExoJointAngleSensor>

        ExoJointAngleSensor exo_joint_angle_sensor_CMC_abad;
        ExoJointAngleSensor exo_joint_angle_sensor_CMC_fe;
        ExoJointAngleSensor exo_joint_angle_sensor_CMC_MCP;
        ExoJointAngleSensor exo_joint_angle_sensor_MCP;
        ExoJointAngleSensor exo_joint_angle_sensor_MCP_IP;
        ExoJointAngleSensor exo_joint_angle_sensor_IP;
        ExoSEA exo_sea_CMC_fe;
        ExoSEA exo_sea_CMC_abad;
        ExoSEA exo_sea_MCP;
        ExoSEA exo_sea_IP;

        // smoothed sensor values
        double smoothed_sensor_values[6] = {0,0,0,0,0,0};

    } thumb;

    ofstream myfile;

    // * Hand Exo Structure Setup * //
    int running_cnt;
    double startTime;
    bool run_flag; //false if failed
    HWInterface_Esmacat* pHW_interface;


    bool motorsHomed;

public:
    my_app();
    virtual ~my_app();
    bool run(double time);
    bool updateJointAngles(double* joint_angles);
    bool commandJointTorque(double *joint_torques,double* desired_angle, double* actual_torque);
    void smoothSensorData(const int N);

    bool ExitManager();
    int kbhit();


};
#endif //MAESTRO_HAPTICS_MY_APP_H
