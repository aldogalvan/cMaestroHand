
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

void cMaestroDigit::computeOptimization(const Eigen::Vector3d a_goalPos, const int a_maxIts , const double ep)
{

}

// -----------------------------------------------------------------//

void cMaestroDigit::stayOnPointVF(Eigen::MatrixXd &A, Eigen::VectorXd &b, int n, int m, Eigen::Vector3d pos_des,
                                       Eigen::Vector3d pos_cur, Eigen::Vector3d dir_des, Eigen::Vector3d dir_cur)
{

}

// -----------------------------------------------------------------//

 Eigen::Vector3d cMaestroDigit::updateJointAngles(double *joint_angles , Vector3d a_globalPos, Vector3d a_globalRot)
{

    global_pos = a_globalPos;



    theta(0) = a_globalRot(0);
    theta(1) = a_globalRot(1);
    theta(2) = a_globalRot(2);
    theta(3) = 0 * 10 * (3.14 / 180);
    theta(4) = joint_angles[0];
    theta(5) = joint_angles[1];
    theta(6) = 0.33 * joint_angles[1];

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

double* cMaestroDigit::commandJointTorqueProxy(double K ,  double B)
{

    auto joint_torque = K*(theta_proxy - theta);

    double pjoint_torque[2];
    pjoint_torque[0] = joint_torque(0);
    pjoint_torque[1] = joint_torque(1);

    return pjoint_torque;
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
