#include "chai3d.h"
#include "cHand.h"
#include <Eigen/Dense>

//==============================================================================
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace Eigen;
//==============================================================================

#ifndef MAESTRO_CHAI3D_CMAESTRODIGIT_H
#define MAESTRO_CHAI3D_CMAESTRODIGIT_H

class cMaestroDigit
{

public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cMaestroDigit
    cMaestroDigit()
    {

        // resize the theta vector and set to zero
        theta.resize(7);
        theta.setZero();
        theta_proxy.resize(7);
        theta_proxy.setZero();

        // tranformation from space to body in zero configuration
        M << 1 , 0 , 0 , MCP_PIP + PIP_DIP + DIP_TIP,
             0 , 1 , 0 , MCP                       ,
             0 , 0 , 1 , 0                          ,
             0 , 0 , 0 , 1                          ;

        // create the twist matrix
        S_space.resize(6,7);
        S_space << 1  , 0 , 0 , 0  , 0 , 0       , 0                ,
                0  , 1 , 0 , 0   , 1 , 1       , 1                ,
                0  , 0 , 1 , 1   , 0 , 0       , 0                ,
                0  , 0 , 0 , MCP , 0 , 0       , 0                ,
                0  , 0 , 0 , 0   , 0 , 0       , 0                ,
                0  , 0 , 0 , 0   , 0 , MCP_PIP , MCP_PIP + PIP_DIP;


        B_body.resize(6,7);
        B_body <<  1  , 0                           , 0                            , 0                           , 0                               , 0                    , 0       ,
                0  , 1                              , 0                            , 0                           , 1                               , 1                    , 1       ,
                0  , 0                              , 1                            , 1                           , 0                               , 0                    , 0       ,
                0  , 0                              , -MCP                         , 0                           , 0                               , 0                    , 0       ,
                0  , 0                              , MCP_PIP + PIP_DIP + DIP_TIP  , MCP_PIP + PIP_DIP + DIP_TIP , 0                               , 0                    , 0       ,
                MCP, -(MCP_PIP + PIP_DIP + DIP_TIP) , 0                            , 0                           ,  -(MCP_PIP + PIP_DIP + DIP_TIP) , -(PIP_DIP + DIP_TIP) , -DIP_TIP;

    };

    //! Destructor of cMaestroDigit
    virtual ~cMaestroDigit(){};

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    // hand proxy algorithm (no constraints)
    // collision is a flag that determines whether it is necessary to compute IK
    // i.e. if theta_proxy = theta
    Vector3d computeHandProxy(const Vector3d a_pos, bool collision = false);

    // hand proxy algorithm (stay on point VF)
    Vector3d computeHandProxySOP(const Vector3d a_pos, const double tol = 0.01, bool collision = false);

    // hand proxy algorithm (wall VF)
    Vector3d computeHandProxyWall(const Vector3d a_pos, const double tol = 0.01 , bool collision = false);

    // this function computes the forward kinematics and returns fingertip pos
    Vector3d updateJointAngles(double* robot_angles , Vector3d a_global_pos, Vector3d a_globalRot);

    // this function computes the forward kinematics using homogeneous transformation matrices
    // DEPRECATED
    Matrix4d computeFK(VectorXd a_theta);

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
    bool computeIKBodyFrame(const Matrix4d T, Matrix4d& Tsb, const VectorXd a_theta0, VectorXd a_theta,
                                int max_it = 20, double eomg = 0.005, double ev = 0.005);

    // this function computes numerical inverse kinematics to desired position
    bool computeIKSpaceFrame(const Matrix4d T, Matrix4d& Tsb, const VectorXd a_theta0, VectorXd a_theta,
                                 int max_it = 20, double eomg = 0.005, double ev = 0.005);


protected:

    // this function creates a stay on point virtual fixture
    void stayOnPointVF(MatrixXd& A ,VectorXd& b, int n , int m , Vector3d pos_des,
                       Vector3d pos_cur , Vector3d dir_des, Vector3d dir_cur);

    // this function creates a virtual wall virtual fixture
    void virtualWallVF(MatrixXd& A, VectorXd& b );

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

    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

    // the array for joint angles
    VectorXd theta;

    // the array for proxy joint angles
    VectorXd theta_proxy;

    // the array for offset angles
    VectorXd theta_offset;

    // set the desired torque
    double exo_desired_torque_MCP = 0;
    double exo_desired_torque_PIP = 0;

    // transformation from base to end
    Matrix4d M;

    // current transformation to tip in body frame
    Matrix4d T_body;

    //current transformation to tip in space frame
    Matrix4d T_space;

    // current transformation to proxy tip in body
    Matrix4d T_proxy_body;

    // current transformation to proxy tip in space
    Matrix4d T_proxy_space;

    // segment length from center of hand to MCP index
    double MCP = 0.025;

    // segment from MCP to PIP
    double MCP_PIP = 0.03978;

    //segment from PIP to DIP
    double PIP_DIP = 0.02238;

    //segment from DIP to TIP
    double DIP_TIP = 0.01566;

    // jacobian matrix in space frame
    MatrixXd J_s;

    // jacobian matrix in body frame
    MatrixXd J_b;

    // screw axis in space frame
    MatrixXd S_space;

    // screw axis in the body frame
    MatrixXd B_body;


};

#endif //MAESTRO_CHAI3D_CMAESTRODIGIT_H
