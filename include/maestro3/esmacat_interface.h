#ifndef ESMACAT_INTERFACE_H
#define ESMACAT_INTERFACE_H

#define NUMBER_OF_DRIVERS 8
#define NUMBER_OF_JOINT_ANGLE_SENSORS 16

#include <iostream>
#include "math.h"
#include <stdio.h>

#include "esmacat_motordriver.h"


//class HandExoManager; //forward declare manager to access methods for index setting,
//class esmacat_motor_driver; //forward declare for access of motor driver pointers

using namespace std;

class Esmacat_Interface
{
private:
    esmacat_motor_driver** md_array;

public:
    Esmacat_Interface();
    ~Esmacat_Interface();
    void esmacatInterfaceConnect(esmacat_motor_driver* pDriverArray[]);
    void esmacatInterfaceSetup();
    int getOutputArraySize(){return NUMBER_OF_DRIVERS;}
    int getJointSensorArraySize(){return NUMBER_OF_JOINT_ANGLE_SENSORS;}
    void getEncoderData(double *enc_raw_data);
    void getAnalogSensorData(double* analog_raw_data);
    void getDigitalSensorData(bool* digital_raw_data);
    bool isESTOP();
    void setMotorPositions(double* motor_setpoints);
};

#endif // ESMACAT_INTERFACE_H
