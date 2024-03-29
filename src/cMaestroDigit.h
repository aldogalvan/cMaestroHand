

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
    cMaestroDigit(bool isMid = false)
    {

        if (isMid)
        {
            MCP = 0.00;
            MCP_PIP = 0.04463;
            PIP_DIP = 0.02633;
            DIP_TIP = 0.0173778;

        }


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


        // tranformation from space to body in zero configuration
        M_mcp_to_tip << 1 , 0 , 0 , MCP_PIP + PIP_DIP + DIP_TIP,
                0 , 1 , 0 , 0                       ,
                0 , 0 , 1 , 0                          ,
                0 , 0 , 0 , 1                          ;

        // transformation from hand to MCP
        M_to_mcp << 1 , 0 , 0 , 0,
                0 , 1 , 0 , MCP                       ,
                0 , 0 , 1 , 0                          ,
                0 , 0 , 0 , 1                          ;

        // create the twist matrix
        S_space.resize(6,7);
        S_space << 1  , 0 , 0 , 0   , 0 , 0       , 0                ,
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

        double pi = 3.14;
        // sets the bounds for the displacement of the joints
        ub.resize(4); lb.resize(4);
        ub << pi, pi, pi, pi;
        lb << -pi, -pi/4, 0 , 0;
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


    // this function computes the forward kinematics and returns fingertip pos
    Vector3d updateJointAngles(double* robot_angles , Vector3d a_global_pos, Vector3d a_globalRot);

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
    bool computeIKSpaceFrame(const MatrixXd& T, MatrixXd& Tsb, VectorXd& a_theta,
                                 double eomg = 0.0005, double ev = 0.001);

    //  this function computes the forward kinematics (pulley to joint)
    void computeForwardKinematics(double phi3, double psy2);

    // this function computes and returns the robot jacobian
    double* computeRobotJacobian(double phi3, double psy2);

    // Function Declarations
    static void Jacobian_RobotToFinger_toCpp(const double q_vec[2], const double L_vec[3], const double F_vec[6], double J[8]);

    // this function computes the joint torques
    void computeJointTorque(const double K, const double B, double* torque);


    //this function returns the joint torque
    void getJointTorque(double* digit_torque);

public:

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS
    //--------------------------------------------------------------------------

    int counter = 0;

    // the global position of the hand
    Vector3d global_pos;

    // position of the tool tip
    Vector3d tip_pos;

    // position of the proxy
    Vector3d proxy_pos;

    // length of the finger



protected:

    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

    // angles
    // actual senspr values
    double theta_MCP_ABAD;
    double theta_MCP_ABAD_offset = 2.41;
    double theta_MCP;
    double theta_MCP_offset = 0.80;
    double theta_PIP;
    double theta_PIP_offset = 2.01;
    double theta_DIP;

    // the bounds for the joints
    VectorXd ub;
    VectorXd lb;

    // the array for joint angles
    VectorXd theta;

    // the array for proxy joint angles
    VectorXd theta_proxy;

    // the last proxy joint angles
    VectorXd theta_last;

    // angular velocity
    VectorXd theta_dot;

    // the array for offset angles
    VectorXd theta_offset;

    // set the desired torque
    double exo_desired_torque_MCP = 0;
    double exo_desired_torque_PIP = 0;

    // transformation from base to end
    Matrix4d M;

    // transformation from MCP to fingertip
    Matrix4d M_mcp_to_tip;

    // transformation to MCP
    Matrix4d M_to_mcp;

    // current transformation to tip in body frame
    MatrixXd T_body;

    //current transformation to tip in space frame
    MatrixXd T_space;

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

    // the robot jacobian
    MatrixXd jacobian;
    double* robot_jacobian;


};

#endif //MAESTRO_CHAI3D_CMAESTRODIGIT_H
