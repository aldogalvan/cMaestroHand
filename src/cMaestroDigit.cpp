//
// Created by aldo on 1/24/22.
//

#include "cMaestroDigit.h"
#include "mr.h"

// NOTE :  ALL FORWARD KINEMATICS ARE COMPUTED IN THE SPACE FRAME. BODY
// FRAME WILL REQUIRE SOME ADJUSTMENT
// INDICES
// 0,1,2 : alpha,beta,gamma correspondng to hand rotation
// 3: MCP abduction/adduction
// 4: MCP flex,ext
// 5: PIP flex,ext
// 6: DIP flex,ext

// -----------------------------------------------------------------//

Vector3d cMaestroDigit::computeHandProxy(const Vector3d a_pos, bool collision)
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

Vector3d cMaestroDigit::computeHandProxySOP(const Vector3d a_pos, const double tol , bool collision)
{

}

// -----------------------------------------------------------------//

Vector3d cMaestroDigit::computeHandProxyWall(const Vector3d a_pos, const double tol, bool collision)
{

}

// -----------------------------------------------------------------//

double* cMaestroDigit::computeInverseDynamics(const Eigen::Vector3d force)
{
    /*
    double torque[2];

    Vector3d P0 = M_T0.block<3,1>(0,3);
    Vector3d P1 = M_T1.block<3,1>(0,3) ;
    Vector3d P2 = M_T2.block<3,1>(0,3) ;
    Vector3d P3 = M_T3.block<3,1>(0,3) ;

    Vector3d L3 = P3 - P2 ;
    Vector3d L2 = P2 - P1 ;
    Vector3d L1 = P1 - P0 ;

    // to do: dot product to extract torques only in the z axis
    Vector3d torques_P2 = L3.cross(force) ;  // + torques_P3 when generalizing
    Vector3d torques_P1 = L2.cross(force) + torques_P2 ;
    Vector3d torques_P0 = L1.cross(force) + torques_P1 ;

    //
    double joint_torque_MCP = 0.001*torques_P0.dot(Eigen::Vector3d  (0,0,1)); //   [N m]
    double joint_torque_PIP = 0.001*torques_P1.dot(Eigen::Vector3d  (0,0,1)); //   [N m]
    double joint_torque_DIP = 0.001*torques_P2.dot(Eigen::Vector3d  (0,0,1)); //   [N m]

    // compute exo joint torques
    double exo_torque_MCP = joint_torque_MCP;
    double exo_torque_PIP = -joint_torque_PIP;

    // returns the torque
    torque[0]= exo_torque_MCP ; torque[1] =  exo_torque_PIP;

    return torque;
     */
}

// -----------------------------------------------------------------//

void cMaestroDigit::computeOptimization(const Eigen::Vector3d a_goalPos, const int a_maxIts , const double ep)
{

}

// -----------------------------------------------------------------//

void cMaestroDigit::stayOnPointVF(Eigen::MatrixXd &A, Eigen::VectorXd &b, int n, int m, Eigen::Vector3d pos_des,
                                       Eigen::Vector3d pos_cur, Eigen::Vector3d dir_des, Eigen::Vector3d dir_cur)
{

}

// -----------------------------------------------------------------//

 Eigen::Vector3d cMaestroDigit::updateJointAngles(double *robot_angles , Vector3d a_globalPos, Vector3d a_globalRot)
{

    global_pos = a_globalPos;

    //auto joint_angles = M3KL1(robot_angles[0], robot_angles[1], robot_angles[2],
      //                        robot_angles[3], robot_angles[4]);

    theta(0) = a_globalRot(0);
    theta(1) = a_globalRot(1);
    theta(2) = a_globalRot(2);
    theta(3) = 0 * 10 * (3.14 / 180);
    theta(4) = 0 * 10 * (3.14 / 180); //0*joint_angles[0];
    theta(5) = 0 * 10 * (3.14 / 180); //0*joint_angles[1];
    theta(6) = 0 * 10 * (3.14 / 180); //*0.33 * joint_angles[1];

    //auto T = computeFKSpaceFrame(theta);
    T_body = computeFKBodyFrame(theta);

    return T_body.block<3,1>(0,3);
}

// -----------------------------------------------------------------//

Matrix4d cMaestroDigit::computeFK(VectorXd a_theta)
{
    /*
    // transformation from hand to mcp (always angle zero)
    M = SE3(aa2rot(Eigen::Vector3d(0,0,1),a_ang_MCP_abad),Eigen::Vector3d(0,MCP,0));

    // transformation for MCP abad (always zero too)
    T0 = SE3(aa2rot(Eigen::Vector3d(0,1,0),a_ang_MCP_fe),Eigen::Vector3d(0,0,0));

    // transformation for MCP fe
    T1 = SE3(aa2rot(Eigen::Vector3d(0,1,0),a_ang_PIP),Eigen::Vector3d(MCP_PIP,0,0));

    // transformation for abad (always zero too)
    T2 = SE3(aa2rot(Eigen::Vector3d(0,1,0),a_ang_DIP),Eigen::Vector3d(PIP_DIP,0,0));

    // transformation for DIP fe
    T3 = SE3(aa2rot(Eigen::Vector3d(0,0,0),0),Eigen::Vector3d(DIP_TIP,0,0));

    // transformation from M to T1
    M_T0 = computeTransformAtoB(M,T0);

    // transformation from M to T2
    M_T1 = computeTransformAtoB(M_T0,T1);

    // transformation from M to T3
    M_T2 = computeTransformAtoB(M_T1,T2);

    // transformation from M to T3
    M_T3 = computeTransformAtoB(M_T2,T3);

    return M_T4;*/
}

// -----------------------------------------------------------------//

Matrix4d cMaestroDigit::computeFKSpaceFrame(VectorXd a_theta)
{
    auto ret = FKinSpace(M,S_space,a_theta);
    return ret;
}

// -----------------------------------------------------------------//

Matrix4d cMaestroDigit::computeFKBodyFrame(VectorXd a_theta)
{
    auto ret = FKinBody(M,B_body,a_theta);
    return ret;
}

// -----------------------------------------------------------------//

MatrixXd cMaestroDigit::computeSpaceJacobian(VectorXd a_theta)
{
    return JacobianSpace(S_space,theta);
}

// -----------------------------------------------------------------//

MatrixXd cMaestroDigit::computeBodyJacobian(VectorXd a_theta)
{
    return JacobianBody(B_body,theta);
}

// -----------------------------------------------------------------//

VectorXd cMaestroDigit::getJointAngles(void)
{
    return theta.tail(4);
}

// -----------------------------------------------------------------//

VectorXd cMaestroDigit::getProxyJointAngles(void)
{
    return theta_proxy.tail(4);
}


// -----------------------------------------------------------------//

double* cMaestroDigit::commandJointTorque(double K ,  double B)
{

    double joint_torque[2];

    joint_torque[0] = exo_desired_torque_MCP;
    joint_torque[1] = exo_desired_torque_PIP;

    return joint_torque;
}

// -----------------------------------------------------------------//

double* cMaestroDigit::M3KL1(const double A1, const double B1, const double C1, const double PHI1, const double PHI3)
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

bool cMaestroDigit::computeIKBodyFrame(const Matrix4d T, Matrix4d& Tsb, const VectorXd a_theta0, VectorXd a_theta,
                                           int max_it, double eomg, double ev)
{
    return IKinBody(B_body,M,T,Tsb,a_theta0,a_theta,eomg,ev,max_it);
}

// -----------------------------------------------------------------//

bool cMaestroDigit::computeIKSpaceFrame(const Matrix4d T, Matrix4d& Tsb, const VectorXd a_theta0, VectorXd a_theta,
                                            int max_it, double eomg, double ev)
{
    return IKinBody(B_body,M,T,Tsb,a_theta0,a_theta,eomg,ev,max_it);
}
