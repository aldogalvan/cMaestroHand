//
// Created by aldo on 1/24/22.
//

#include "cMaestroThumb.h"
#include "mr.h"

using namespace mr;
// NOTE :  ALL FORWARD KINEMATICS ARE COMPUTED IN THE SPACE FRAME. BODY
// FRAME WILL REQUIRE SOME ADJUSTMENT
// INDICES
// 0,1,2 : alpha,beta,gamma correspondng to hand rotation
// 3: CMC abduction/adduction
// 4: CMC flex,ext
// 5: MCP flex,ext
// 6: IP flex,ext

// -----------------------------------------------------------------//

Vector3d cMaestroThumb::computeHandProxy(const Vector3d a_pos, bool collision)
{
    if (collision)
    {
        VectorXd theta_temp = theta_proxy.tail(4);
        MatrixXd T_des = T_space;
        T_des.block<3, 1>(0, 3) = a_pos;
        auto flag = computeIKSpaceFrame(T_des, T_space, theta_temp);

        if(flag)
        {
            T_proxy_space = T_space;
            theta_proxy.tail(4) = theta_temp;
            //std::cout << "Converged Successfully" << std::endl;
            return T_proxy_space.block<3, 1>(0, 3);
        }
        else
        {
            std::cout << "Failed to Converge" << std::endl;
            T_proxy_space = T_space;
            theta_proxy = theta;
            theta_proxy.tail(4) = theta_temp;
            return a_pos;
        }
    }
    else
    {
        T_proxy_space = T_space;
        theta_proxy = theta;
        return a_pos;
    }
}

// -----------------------------------------------------------------//

Vector3d cMaestroThumb::updateJointAngles(double *joint_angles , Vector3d a_globalPos, Vector3d a_globalRot)
{

    global_pos = a_globalPos;


    theta(0) = a_globalRot(0);
    theta(1) = a_globalRot(1);
    theta(2) = a_globalRot(2);
    theta(3) = 0;//100 * (3.14 / 180);
    theta(4) = 0;//10 * (3.14 / 180);//joint_angles[0];
    theta(5) = 0;//10 * (3.14 / 180);//joint_angles[1];
    theta(6) = 0;//10 * (3.14 / 180);//0.33 * joint_angles[1];

    //auto T = computeFKSpaceFrame(theta);
    T_body = computeFKBodyFrame(theta);

    return T_body.block<3,1>(0,3);
}

// -----------------------------------------------------------------//

Matrix4d cMaestroThumb::computeFKSpaceFrame(VectorXd a_theta)
{
    auto ret = FKinSpace(M,S_space,a_theta);
    return ret;
}

// -----------------------------------------------------------------//

Matrix4d cMaestroThumb::computeFKBodyFrame(VectorXd a_theta)
{
    auto ret = FKinBody(M,B_body,a_theta);
    return ret;
}

// -----------------------------------------------------------------//

bool cMaestroThumb::computeIKBodyFrame(const MatrixXd T, MatrixXd& Tsb, VectorXd& a_theta,
                                       int max_it, double eomg, double ev)
{
    MatrixXd B = B_body.block<6,4>(0,3);
    return IKinBody(B,M,T,Tsb,a_theta,eomg,ev);
}

// -----------------------------------------------------------------//

bool cMaestroThumb::computeIKSpaceFrame(const MatrixXd T, MatrixXd& Tsb,  VectorXd& a_theta,
                                        int max_it, double eomg, double ev)
{
    MatrixXd S = S_space.block<6,4>(0,3);
    return IKinSpace(S,M,T,Tsb,a_theta,eomg,ev);
}

// -----------------------------------------------------------------//

MatrixXd cMaestroThumb::computeSpaceJacobian(VectorXd a_theta)
{
    return JacobianSpace(S_space,theta);
}

// -----------------------------------------------------------------//

MatrixXd cMaestroThumb::computeBodyJacobian(VectorXd a_theta)
{
    return JacobianBody(B_body,theta);
}

// -----------------------------------------------------------------//

VectorXd cMaestroThumb::getJointAngles(void)
{
    return theta.tail(4);
}

// -----------------------------------------------------------------//

VectorXd cMaestroThumb::getProxyJointAngles(void)
{
    return theta_proxy.tail(4);
}


// -----------------------------------------------------------------//

double* cMaestroThumb::commandJointTorqueProxy(double K ,  double B)
{

    auto joint_torque = K*(theta_proxy - theta);

    double pjoint_torque[4];
    pjoint_torque[0] = joint_torque(0);
    pjoint_torque[1] = joint_torque(1);
    pjoint_torque[2] = joint_torque(2);
    pjoint_torque[3] = joint_torque(3);

    return pjoint_torque;
}

// -----------------------------------------------------------------//
