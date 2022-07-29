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
using namespace chai3d;
//==============================================================================

#ifndef MAESTRO_CHAI3D_CMAESTRODIGIT_H
#define MAESTRO_CHAI3D_CMAESTRODIGIT_H

class cMaestroDigit : public cGenericObject
{

public:
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cMaestroDigit
    cMaestroDigit()
    {
        // create the twist matrix
        screw.resize(6,5);

        screw << 0 , 0 , 0 , 0 , 0,
                0 , 1 , 1 , 1 , 0,
                1 , 0 , 0 , 0 , 0,
                MCP , 0 , MCP_PIP, MCP_PIP + PIP_DIP ,  MCP_PIP + PIP_DIP + DIP_TIP,
                0 , 0 , 0 , 0 , 0,
                0 , 0 , 0 , 0 , 0;


    };

    //! Destructor of cMaestroDigit
    virtual ~cMaestroDigit(){};

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    // this function creates a stay on point virtual fixture
    void stayOnPointVF(Eigen::MatrixXd& A ,Eigen::VectorXd& b, int n , int m , Eigen::Vector3d pos_des,
                                      Eigen::Vector3d pos_cur , Eigen::Vector3d dir_des, Eigen::Vector3d dir_cur);

    // this function creates a virtual wall virtual fixture
    void virtualWallVF(Eigen::MatrixXd& A, Eigen::VectorXd& b );

    // this function computes the forward kinematics and returns fingertip pos
    Eigen::Vector3d updateJointAngles(double* robot_angles , Eigen::Vector3d a_global_pos);

    // this function computes the forward kinematics
    Eigen::Matrix4d computeForwardKinematics(double a_ang_MCP_fe, double a_ang_MCP_abad,
                                     double a_ang_PIP, double a_ang_DIP);

    // this function computes the jacobian
    void computeJacobian(double a_ang_MCP_fe, double a_ang_MCP_abad,
                                        double a_ang_PIP, double a_ang_DIP);

    // this method computes an optimization problem to find desired angular displacement
    void computeOptimization(const Eigen::Vector3d a_goalPos, const int a_maxIts = 10, const double ep = 0.001);

    // this method commands a new joint torque
    double* commandJointTorque(double K , double B);

    // return the vector of finger angles
    void getJointAngles(double* joint_angles);

    // Compute parameters
    double* M3KL1(const double A1, const double B1, const double C1,
                  const double PHI1, const double PHI3);

    // computes the inverse dynamics of the finger
    double* computeInverseDynamics(const Eigen::Vector3d force);

    // this method makes the special euclidean matrix
    Eigen::Matrix4d SE3(Eigen::Matrix3d a_rot, Eigen::Vector3d a_tr);

    // axis angle to rotation
    Eigen::Matrix3d aa2rot(const Eigen::Vector3d a_axis, const double a_angle);

    // vector to skew symmetric form
    Eigen::Matrix3d vec2skew(const Eigen::Vector3d);

    // this method commands a new joint torque
    Eigen::Matrix4d computeTransformAtoB(const Eigen::Matrix4d A , const Eigen::Matrix4d B);

    // this method changes twist vector to matrix represenation
    Eigen::Matrix4d twist2mat(const Eigen::VectorXd& a_twist);

    // homogeneous transformation to ajoint representation
    Eigen::MatrixXd adjointRepresentation(const Eigen::MatrixXd a_T);

public:

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS
    //--------------------------------------------------------------------------

    // the global position of the hand
    Eigen::Vector3d global_pos;

    // position of the tool tip
    Eigen::Vector3d tip_pos;

    // position of the proxy
    Eigen::Vector3d proxy_pos;


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

    // actual joint angles
    double ang_MCP_abad = 0;
    double ang_MCP_fe;
    double ang_PIP;
    double ang_DIP;

    // proxy angles
    Eigen::VectorXd theta_loop;

    // base transformation
    Eigen::Matrix4d M;

    // transformation base
    Eigen::Matrix4d T1 ;

    // transformation PIP
    Eigen::Matrix4d T2;

    // transformation DIP
    Eigen::Matrix4d T3;

    // transformation TIP
    Eigen::Matrix4d T4;

    // transformation
    Eigen::Matrix4d M_T1;

    //transformation
    Eigen::Matrix4d M_T2;

    // transofrmation
    Eigen::Matrix4d M_T3;

    // transofrmation
    Eigen::Matrix4d M_T4;

    // segment length from center of hand to MCP index
    double MCP = 0.025;

    // segment from MCP to PIP
    double MCP_PIP = 0.03978;

    //segment from PIP to DIP
    double PIP_DIP = 0.02238;

    //segment from DIP to TIP
    double DIP_TIP = 0.01566;

    // jacobian matrix in space frame
    Eigen::MatrixXd j_s;

    // screw axis
    Eigen::MatrixXd screw;

    // body twist
    Eigen::MatrixXd twist_b;

    // space twist
    Eigen::MatrixXd twist_s;




};

#endif //MAESTRO_CHAI3D_CMAESTRODIGIT_H
