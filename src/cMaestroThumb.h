//
// Created by aldo on 1/24/22.
//
#include "chai3d.h"
#include <Eigen/Dense>

#ifndef MAESTRO_CHAI3D_CMAESTROTHUMB_H
#define MAESTRO_CHAI3D_CMAESTROTHUMB_H

class cMaestroThumb : public chai3d::cGenericObject {

public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cPhantomDevice.
    cMaestroThumb();

    //! Destructor of cPhantomDevice.
    virtual ~cMaestroThumb();

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    //! This method updates the data from the Maestro class
    bool updateJointAngles(double* joint_angles);

    //! This method commands the desired joint torque
    bool commandJointTorque(double* joint_torque);

    //! This method returns the position of the device.
    virtual bool getPosition(chai3d::cVector3d& a_position);

    //! This method returns orientation frame of the haptic device end-effector.
    virtual bool getRotation(chai3d::cMatrix3d& a_rotation);

    //! This method sets the force of the device
    virtual bool setForce(chai3d::cVector3d& a_force);



};

#endif //MAESTRO_CHAI3D_CMAESTROTHUMB_H
