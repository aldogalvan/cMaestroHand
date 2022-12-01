//
// Created by aldo on 1/24/22.
//

#include "cMaestroThumb.h"
#include "mr.h"

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
        Matrix4d T_des = T_body;
        T_des.block<3, 1>(0, 3) = a_pos;
        if(computeIKBodyFrame(T_des, T_proxy_body, theta,theta_proxy))
        {
            return T_proxy_body.block<3, 1>(0, 3);
        }
        else
        {
            return a_pos;
        }
    }
    else
    {
        T_proxy_body = T_body;
        theta_proxy = theta;
        return a_pos;
    }
}

// -----------------------------------------------------------------//

Vector3d cMaestroThumb::computeHandProxySOP(const Vector3d a_pos, const double tol , bool collision)
{

}

// -----------------------------------------------------------------//

Vector3d cMaestroThumb::computeHandProxyWall(const Vector3d a_pos, const double tol, bool collision)
{


}
// -----------------------------------------------------------------//

void cMaestroThumb::computeOptimization(const Eigen::Vector3d a_goalPos, const int a_maxIts , const double ep)
{

}

// -----------------------------------------------------------------//

void cMaestroThumb::stayOnPointVF(Eigen::MatrixXd &A, Eigen::VectorXd &b, int n, int m, Eigen::Vector3d pos_des,
                                  Eigen::Vector3d pos_cur, Eigen::Vector3d dir_des, Eigen::Vector3d dir_cur)
{

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

double* cMaestroThumb::M3KL1(const double A1, const double B1, const double C1, const double PHI1, const double PHI3)
{
    double t2 = cos(PHI1); double t3 = pow(A1,2); double t4 = pow(B1,2);
    double t5 = pow(C1,2); double t8 = PHI3/2.0; double t6 = pow(t2,2);
    double t7 = pow(A1,t2); double t9 = -t5; double t10 = tan(t8); double t11 = -t7;
    double t12 = B1*t7*2.0; double t14 = pow(t10,2); double t15 = B1*t10*2.0;
    double t16 = t3*t6*2.0; double t20 = B1*t7*t10*4.0; double t13 = -t12;
    double t17 = t14+1.0; double t18 = C1*t14; double t19 = t7*t14; double t21 = t4*t14;
    double t22 = t5*t14; double t23 = -t20; double t24 = t11*t14; double t25 = t12*t14;
    double t26 = t9*t14; double t27 = t14*t16; double t28 = C1+t11+t15+t18+t24;
    double t30 = t4+t9+t13+t16+t21+t23+t25+t26+t27; double t29 = 1.0/t28;
    double t31 = t17*t30; double t32 = sqrt(t31);
    double KL1[2];
    KL1[0] = atan(t29*(-B1+t7+t19+B1*t14)-t29*t32)*2.0;
    KL1[1] = t32/t17;
    return KL1;
}

// -----------------------------------------------------------------//

bool cMaestroThumb::computeIKBodyFrame(const Matrix4d T, Matrix4d& Tsb, const VectorXd a_theta0, VectorXd a_theta,
                                       int max_it, double eomg, double ev)
{
    return IKinBody(B_body,M,T,Tsb,a_theta0,a_theta,eomg,ev,max_it);
}

// -----------------------------------------------------------------//

bool cMaestroThumb::computeIKSpaceFrame(const Matrix4d T, Matrix4d& Tsb, const VectorXd a_theta0, VectorXd a_theta,
                                        int max_it, double eomg, double ev)
{
    return IKinBody(B_body,M,T,Tsb,a_theta0,a_theta,eomg,ev,max_it);
}