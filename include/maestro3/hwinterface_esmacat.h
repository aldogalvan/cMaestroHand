/*
 * hwinterface_esmacat.h
 *
 *  Created on: Jul 1, 2015
 *  Modified on: Jun 8, 2018
 *      Author: Ashish.Lab
 */

#ifndef HWINTERFACE_ESMACAT_H
#define HWINTERFACE_ESMACAT_H

#include <iostream>
#include <new>
#include <cmath>
#include "hw_interface.h"
#include "esmacat_interface.h"

using namespace std;

class HWInterface_Esmacat : public HW_Interface, public Esmacat_Interface{

private:
    double exo_joint_angle_volts_to_rad_scale[NUMBER_OF_JOINT_ANGLE_SENSORS];
    double exo_joint_angle_volts_offset[NUMBER_OF_JOINT_ANGLE_SENSORS];
    double motor_angle_tick_to_rad_scale[NUMBER_OF_DRIVERS];  // for maxon combination 512420
    double motor_angle_tick_offset[NUMBER_OF_DRIVERS];
//        double motor_set_value_conversion_rad_to_mV_scale[8];
//        double motor_set_value_conversion_rad_to_mV_offset[8];

    enum homeState {retractOffSensor, advanceToSensor, homing, homed};
    homeState motorState[NUMBER_OF_DRIVERS];
    bool* homeSensorArray;
    bool homingComplete;
    int hCycleCount;
    int hCycleOffset[8];
    constexpr static double motorAdvanceVelocity = -0.05*2*M_PI*1E-3;

public:

    //movaghati!
//    double* exo_joint_angle_sensor_volts_array;


    HWInterface_Esmacat();
    bool ScanAllSensorData();
//	void GetRawSensorData(int32_t* CRIO_sensor_raw_array_){CRIO_sensor_raw_array_ = CRIO_sensor_raw_array; };
    void CalibrateAllSensorData();
    bool FlushAllOutputData();
    void ConvertOutputUnit();
    virtual ~HWInterface_Esmacat();
    bool homeMotors();
    //paria's addition to not home motor 8
    bool homeMotors_but_eight();

    bool calibrateSEAJointSensors();
    bool calibrateObserverJointSensors();
};

#endif // HWINTERFACE_ESMACAT_H
