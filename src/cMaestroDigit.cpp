//
// Created by aldo on 1/24/22.
//

#include "cMaestroDigit.h"

double* cMaestroDigit::computeInverseDynamics(const Eigen::Vector3d force)
{
    double torque[2];

    Eigen::Vector3d P0 = M_T1.block<3,1>(0,3);
    Eigen::Vector3d P1 = M_T2.block<3,1>(0,3) ;
    Eigen::Vector3d P2 = M_T3.block<3,1>(0,3) ;
    Eigen::Vector3d P3 = M_T4.block<3,1>(0,3) ;

    Eigen::Vector3d L3 = P3 - P2 ;
    Eigen::Vector3d L2 = P2 - P1 ;
    Eigen::Vector3d L1 = P1 - P0 ;

    // to do: dot product to extract torques only in the z axis
    Eigen::Vector3d torques_P2 = L3.cross(force) ;  // + torques_P3 when generalizing
    Eigen::Vector3d torques_P1 = L2.cross(force) + torques_P2 ;
    Eigen::Vector3d torques_P0 = L1.cross(force) + torques_P1 ;

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
}

void cMaestroDigit::computeOptimization(const Eigen::Vector3d a_goalPos, const int a_maxIts , const double ep)
{
    // first generate the desired transformation to proxy in space frame
    Eigen::Matrix4d Tsd;
    // set current rotation as desired rotation
    Tsd.block<3,3>(0,0) = M_T4.block<3,3>(0,0);
    Tsd.block<3,1>(0,3) = a_goalPos;
    Tsd.block<1,4>(3,0) << 0 , 0 , 0 , 1;
    auto J_alpha = j_s.block<3,5>(0,0);
    auto J_ep = j_s.block<3,5>(3,0);

    int i = 0;

    double D = (a_goalPos - tip_pos).squaredNorm();

    if (D <= pow(ep,2))
        return;

    computeJacobian(ang_MCP_fe,ang_MCP_abad,ang_PIP,ang_DIP);
    do
    {

        computeForwardKinematics(ang_MCP_fe,ang_MCP_abad,ang_PIP,ang_DIP);

        computeJacobian(ang_MCP_fe,ang_MCP_abad,ang_PIP,ang_DIP);

        D = (a_goalPos - proxy_pos).squaredNorm();

        double er;
        if ( D <= pow(ep,2))
        {
            return;
        }
        i++;
    } while(i < a_maxIts);
}

void cMaestroDigit::stayOnPointVF(Eigen::MatrixXd &A, Eigen::VectorXd &b, int n, int m, Eigen::Vector3d pos_des,
                                       Eigen::Vector3d pos_cur, Eigen::Vector3d dir_des, Eigen::Vector3d dir_cur)
{

}

// Function to update the hand joint angles
 Eigen::Vector3d cMaestroDigit::updateJointAngles(double *robot_angles ,  Eigen::Vector3d a_global_pos)
{

    /*
    // Values in degrees (Dec 3 2021)
    ang_sensor_MCP_abad = exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad();
    ang_sensor_MCP_fe = exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad();
    ang_sensor_MCP_PIP = exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad();
    ang_sensor_PIP = exo_joint_angle_sensor_PIP.GetExoJointAngleRad();
    ang_sensor_DIP = exo_joint_angle_sensor_DIP.GetExoJointAngleRad();
    */

    global_pos = a_global_pos;

    ang_sensor_MCP_abad = robot_angles[0];
    ang_sensor_MCP_fe = robot_angles[1];
    ang_sensor_MCP_PIP = robot_angles[2];
    ang_sensor_PIP = robot_angles[3];
    ang_sensor_DIP = robot_angles[4];

    /*
    auto joint_angles = M3KL1(ang_sensor_MCP_abad, ang_sensor_MCP_fe, ang_sensor_MCP_PIP,
                              ang_sensor_PIP, ang_sensor_DIP);
    */

    ang_MCP_fe = 100 * 10 * (3.14 / 180) ;//joint_angles[0];
    ang_PIP = 100 * 10 * (3.14 / 180) ;//joint_angles[1];
    ang_DIP = 100 * 10 * (3.14 / 180) ; //ang_PIP*0.67;

    auto ret = computeForwardKinematics(ang_MCP_fe,
                                        ang_MCP_abad,ang_PIP,ang_DIP);

    return tip_pos;

}

Eigen::Matrix4d cMaestroDigit::computeForwardKinematics(double a_ang_MCP_fe, double a_ang_MCP_abad,
                                                double a_ang_PIP, double a_ang_DIP)
{

    // transformation from hand to mcp (always angle zero)
    M = SE3(aa2rot(Eigen::Vector3d(0,0,1),a_ang_MCP_abad),Eigen::Vector3d(0,MCP,0));

    // transformation for MCP abad (always zero too)
    T1 = SE3(aa2rot(Eigen::Vector3d(0,1,0),a_ang_MCP_fe),Eigen::Vector3d(0,0,0));

    // transformation for MCP fe
    T2 = SE3(aa2rot(Eigen::Vector3d(0,1,0),a_ang_PIP),Eigen::Vector3d(MCP_PIP,0,0));

    // transformation for abad (always zero too)
    T3 = SE3(aa2rot(Eigen::Vector3d(0,1,0),a_ang_DIP),Eigen::Vector3d(PIP_DIP,0,0));

    // transformation for DIP fe
    T4 = SE3(aa2rot(Eigen::Vector3d(0,0,0),0),Eigen::Vector3d(DIP_TIP,0,0));

    // transformation from M to T1
    M_T1 = computeTransformAtoB(M,T1);

    // transformation from M to T2
    M_T2 = computeTransformAtoB(M_T1,T2);

    // transformation from M to T3
    M_T3 = computeTransformAtoB(M_T2,T3);

    // transformation from M to T3
    M_T4 = computeTransformAtoB(M_T3,T4);

    return M_T4;
}

inline void cMaestroDigit::computeJacobian(double a_ang_MCP_fe, double a_ang_MCP_abad,
                                    double a_ang_PIP, double a_ang_DIP)
{


    j_s.col(0) = screw.col(0);
    j_s.col(1) = adjointRepresentation(M)*screw.col(1);
    j_s.col(2) = adjointRepresentation(M_T1)*screw.col(2);
    j_s.col(3) = adjointRepresentation(M_T2)*screw.col(3);
    j_s.col(4) = adjointRepresentation(M_T3)*screw.col(4);

}


void cMaestroDigit::getJointAngles(double* joint_angles)
{
    joint_angles[0] = ang_MCP_fe;
    joint_angles[1] = ang_PIP;
    joint_angles[2] = ang_DIP;
}

double* cMaestroDigit::commandJointTorque(double K ,  double B)
{

    double joint_torque[2];

    joint_torque[0] = exo_desired_torque_MCP;
    joint_torque[1] = exo_desired_torque_PIP;

    return joint_torque;
}

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

inline Eigen::Matrix4d cMaestroDigit::SE3(Eigen::Matrix3d a_rot, Eigen::Vector3d a_tr)
{
    Eigen::Matrix4d ret;
    ret.block<3,3>(0,0) = a_rot;
    ret.block<3,1>(0,3) = a_tr;
    ret.block<1,4>(3,0) << 0 , 0 , 0 , 1;
    return ret;
}

inline Eigen::Matrix3d cMaestroDigit::aa2rot(const Eigen::Vector3d a_axis, const double a_angle)
{
    Eigen::Matrix3d skew;
    skew = vec2skew(a_axis);

    Eigen::Matrix3d ret = Eigen::Matrix3d().setIdentity() + sin(a_angle)*skew + (1-cos(a_angle))*skew*skew;

    return ret;
}

inline Eigen::Matrix3d cMaestroDigit::vec2skew(const Eigen::Vector3d a_axis)
{
    Eigen::Matrix3d ret;
    ret << 0 , -a_axis(2), a_axis(1),
            a_axis(2), 0, -a_axis(0),
            -a_axis(1), a_axis(0), 0;

    return ret;
}

inline Eigen::Matrix4d cMaestroDigit::computeTransformAtoB(const Eigen::Matrix4d A , const Eigen::Matrix4d B)
{
    Eigen::Matrix4d ret;
    //rotation
    ret.block<3,3>(0,0) = A.block<3,3>(0,0)*B.block<3,3>(0,0);
    //translation
    ret.block<3,1>(0,3) = A.block<3,3>(0,0)*B.block<3,1>(0,3) + A.block<3,1>(0,3);
    // bottom
    ret.block<1,4>(3,0) << 0 , 0 , 0 , 1;

    return ret;
}

inline Eigen::Matrix4d cMaestroDigit::twist2mat(const Eigen::VectorXd& a_twist)
{

}

inline Eigen::MatrixXd cMaestroDigit::adjointRepresentation(const Eigen::MatrixXd a_T)
{
    Eigen::MatrixXd ret(6,6);

    ret.block<3,3>(0,0) = a_T.block<3,3>(0,0);
    ret.block<3,3>(0,3) = Eigen::Matrix3d().setZero();
    ret.block<3,3>(3,0) = vec2skew(a_T.block<3,1>(0,4))*a_T.block<3,3>(0,0);
    ret.block<3,3>(0,3) = a_T.block<3,3>(0,0);

}