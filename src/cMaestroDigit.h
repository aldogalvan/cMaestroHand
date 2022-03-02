#include "chai3d.h"
#include "cHand.h"
#include <Eigen/Dense>


//
// Created by aldo on 1/24/22.
//

//==============================================================================

//------------------------------------------------------------------------------
class cMaestroDigit;
//------------------------------------------------------------------------------

//==============================================================================

#ifndef MAESTRO_CHAI3D_CMAESTRODIGIT_H
#define MAESTRO_CHAI3D_CMAESTRODIGIT_H

class cMaestroDigit : public chai3d::cGenericObject
{

public:
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cPhantomDevice.
    cMaestroDigit(chai3d::cHand* a_hand);

    //! Destructor of cPhantomDevice.
    virtual ~cMaestroDigit();

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    //! This method updates the data from the Maestro class
    void updateJointAngles(double* joint_angles);

    //! This method commands the desired joint torque
    bool commandJointTorque(double* joint_torque);

    // compute fake angles for finger exoskeleton (use for debugin)
    void pseudoComputeJointAnglesFinger(double joint_angle_sensor_MCP, double joint_angle_sensor_PIP);

    // return the vector of finger angles
    double* getJointAngles(void);

    //! This method computes the actual joint angles from finger exoskeleton angles
    Eigen::Vector4d computeJointAnglesFinger(double joint_angle_sensor_MCP_abad, double joint_angle_sensor_MCP_fe, double joint_angle_sensor_MCP_PIP);

    /*
    //! This method computes the actual joint angles from thumb exoskeleton angles
    Eigen::Vector2d computeJointAnglesThumb(double joint_angle_sensor_MCP, double joint_angle_sensor_CMC_MCP);
    */

    //! This method computes the fingertip position from joint angles
    void computeForwardKinematicsFinger(void);

    //! This method computes the inverse dynamics for the finger
    Eigen::Vector2d computeInverseDynamics_Finger(Eigen::Vector3d force);

    //! This method commands the desired torque back to esmacat finger class
    void commandExoTorqueFinger(double exo_torque_MCP, double exo_torque_PIP);

    void AssignIndexOfExoJointAngleSensorArrayHWInterface(int index_exo_joint_angle_sensor_MCP_abd, int index_exo_joint_angle_sensor_MCP_SEA, int index_exo_joint_angle_sensor_btw_SEAs, int index_exo_joint_angle_sensor_PIP_SEA, int index_exo_joint_angle_sensor_DIP_flex);
    void AssignIndexOfExoMotorArrayHWInterface(int index_exo_motor_for_MCP_SEA, int index_exo_motor_for_PIP_SEA);

    //! This method updates the graphics
    void updateGraphics();

public:

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS
    //--------------------------------------------------------------------------

    //! Tool object represnting the tip
    Eigen::Vector3d cursor_TIP;

    //! Tool object representing the DIP
    Eigen::Vector3d cursor_DIP;

    //! Tool object represeting the PIP
    Eigen::Vector3d cursor_PIP;

    //! Tool object representing the MCP
    Eigen::Vector3d cursor_MCP;

    //! Joint angles for each finger in radians
    Eigen::Vector4d hIdxJointAngles;

    //! Segment lengths for each finger
    Eigen::Vector3d hIdxSegLengths;

    //! Virtual Joint torques for each digit
    Eigen::Vector3d hIdxJointTorques;

protected:

    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

    // set the desired torque
    double exo_desired_torque_MCP;
    double exo_desired_torque_PIP;

    // angles of interest
    double ang_sensor_MCP_abad;
    double ang_sensor_MCP_fe;
    double ang_sensor_MCP_PIP;
    double ang_sensor_PIP;
    double ang_sensor_DIP;

    // finger joint angles
    double joint_angles[4];

private:

    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS
    //--------------------------------------------------------------------------

    // Pointer to cHand pointer
    chai3d::cHand** h_hand;



};

#endif //MAESTRO_CHAI3D_CMAESTRODIGIT_H
