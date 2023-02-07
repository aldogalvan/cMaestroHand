//
// Created by aldo on 1/24/22.
//
#include "chai3d.h"
#include <Eigen/Dense>

//==============================================================================
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace Eigen;
//==============================================================================

#ifndef MAESTRO_CHAI3D_CMAESTROTHUMB_H
#define MAESTRO_CHAI3D_CMAESTROTHUMB_H

class cMaestroThumb {

public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cMaestroThumb.
    cMaestroThumb()
    {
        // resize the theta vector and set to zero
        theta.resize(7);
        theta.setZero();
        theta_proxy.resize(7);
        theta_proxy.setZero();

        // tranformation from space to body in zero configuration
        M <<    1 , 0  , 0  , CMC_x + CMC_MCP + MCP_IP + IP_TIP,
                0 , 0  , 1  , CMC_y                            ,
                0 , -1 , 0  , 0                                ,
                0 , 0  , 0  , 1                                ;

        // create the twist matrix
        S_space.resize(6,7);
        S_space << 1  , 0 , 0 , 0      , 0       , 0               , 0                        ,
                   0  , 1 , 0 , 1      , 0       , 0               , 0                        ,
                   0  , 0 , 1 , 0      , -1      , -1              , -1                       ,
                   0  , 0 , 0 , 0      , -CMC_y  , -CMC_y          , -CMC_y                   ,
                   0  , 0 , 0 , 0      , CMC_x   , CMC_x + CMC_MCP , CMC_x + CMC_MCP + MCP_IP ,
                   0  , 0 , 0 , -CMC_x , 0       , 0               , 0;


        B_body.resize(6,7);
        B_body <<  1      , 0                                 , 0                                 , 0                         , 0                         , 0                , 0      ,
                   0      , 0                                 , -1                                , -1                        , 0                         , -1               , -1     ,
                   0      , 1                                 , 0                                 , 0                         , 1                         , 0                , 0      ,
                   0      , 0                                 , -CMC_y                            , 0                         , 0                         , 0                , 0      ,
                   -CMC_y , CMC_x + CMC_MCP + MCP_IP + IP_TIP , 0                                 , 0                         , CMC_MCP + MCP_IP + IP_TIP , 0                , 0      ,
                   0      , 0                                 , CMC_x + CMC_MCP + MCP_IP + IP_TIP , CMC_MCP + MCP_IP + IP_TIP , 0                         , MCP_IP + IP_TIP  , IP_TIP;

        // sets the bounds for the displacement of the joints
        double pi = 3.14;
        ub.resize(4); lb.resize(4);
        ub << pi, pi, pi, 0 ;
        lb << -pi, -pi , -pi , -pi ;
    }

    //! Destructor of cPhantomDevice.
    virtual ~cMaestroThumb(){};

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    // hand proxy algorithm (no constraints)
    // collision is a flag that determines whether it is necessary to compute IK
    // i.e. if theta_proxy = theta
    Vector3d computeHandProxy(const Vector3d a_pos, bool collision = false);


    // this function computes the forward kinematics and returns fingertip pos
    Vector3d updateJointAngles(double* robot_angles , Vector3d a_global_pos, Vector3d a_globalRot);

    // this function computes the forward kinematics using homogeneous transformation matrices
    // DEPRECATED
    Matrix4d computeFK(VectorXd a_theta);

    // this function computes the forward kinematic to the joint
    Matrix4d computeFKToJointSpaceFrame(VectorXd a_theta);

    // this function computes the forward kinematic to the joint
    Matrix4d computeFKToJointBodyFrame(VectorXd a_theta);

    // this function computes the forward kinematics using the formula of matrix exponentials
    Matrix4d computeFKSpaceFrame(VectorXd a_theta);

    // this function computes the forward kinematics using the formula of matrix exponentials
    Matrix4d computeFKBodyFrame(VectorXd a_theta);

    // this function computes the jacobian in space frame
    MatrixXd computeSpaceJacobian(VectorXd a_theta);

    // this function computes the jacobian in body frame
    MatrixXd computeBodyJacobian(VectorXd a_theta);

    // this method computes an optimization problem to find desired angular displacement
    void computeOptimization(const Vector3d a_goalPos, const int a_maxIts = 10, const double ep = 0.001);

    // this method commands a new joint torque
    double* commandJointTorqueProxy(double K , double B);

    // return the vector of actual finger angles
    VectorXd getJointAngles(void);

    // return the vector of proxy anggles
    VectorXd getProxyJointAngles(void);

    // computes the inverse dynamics of the finger
    double* computeInverseDynamics(const Eigen::Vector3d force);

    // this function computes numerical inverse kinematics to desired position in the body frame
    bool computeIKBodyFrame(const MatrixXd T, MatrixXd& Tsb, VectorXd& a_theta,
                             double eomg = 0.001, double ev = 0.001);

    // this function computes numerical inverse kinematics to desired position
    bool computeIKSpaceFrame(const MatrixXd T, MatrixXd& Tsb, VectorXd& a_theta,
                              double eomg = 0.001, double ev = 0.001);

    //  this function computes the forward kinematics (pulley to joint)
    void computeForwardKinematics(double phi3, double psy2);


public:

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS
    //--------------------------------------------------------------------------

    // the global position of the hand
    Vector3d global_pos;

    // position of the tool tip
    Vector3d tip_pos;

    // position of the proxy
    Vector3d proxy_pos;


protected:

    //! for testing delete later
    int counter = 0;

    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

    // the array for joint angles
    VectorXd theta;

    // the array for proxy joint angles
    VectorXd theta_proxy;

    // the bounds for the joints
    VectorXd ub;
    VectorXd lb;

    // set the desired torque
    double exo_desired_torque_CMC_fe = 0;
    double exo_desired_torque_CMC_abad = 0;
    double exo_desired_torque_MCP = 0;
    double exo_desired_torque_IP = 0;

    // transformation from base to end
    Matrix4d M;

    // current transformation to tip in body frame
    MatrixXd T_body;

    //current transformation to tip in space frame
    MatrixXd T_space;

    // current transformation to proxy tip in body
    Matrix4d T_proxy_body;

    // current transformation to proxy tip in space
    Matrix4d T_proxy_space;

    // segment lengths from center of hand to MCP index
    double CMC_x = -0.060; double CMC_y = 0.040;

    // segment from MCP to PIP
    double CMC_MCP = 0.04622;

    //segment from PIP to DIP
    double MCP_IP = 0.03157;

    //segment from DIP to TIP
    double IP_TIP = 0.02147;

    // jacobian matrix in space frame
    MatrixXd J_s;

    // jacobian matrix in body frame
    MatrixXd J_b;

    // screw axis in space frame
    MatrixXd S_space;

    // screw axis in the body frame
    MatrixXd B_body;


};

#endif //MAESTRO_CHAI3D_CMAESTROTHUMB_H
