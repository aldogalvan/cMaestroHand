
#include "cMaestroDigit.h"
#include "mr.h"
# define pi           3.14159265358979323846  /* pi */

using namespace mr;

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

 Eigen::Vector3d cMaestroDigit::updateJointAngles(double *joint_angles , Vector3d a_globalPos, Vector3d a_globalRot)
{

    global_pos = a_globalPos;

     std::cout << "ABAD: " << joint_angles[0] << "  MCP_FE: " << joint_angles[1] << "  PIP_FE: " << joint_angles[3] << std::endl;
    // APPROXIMATE OFFSETS IN THE ZERO CONFIGURATION
    // SUBJECT TO CHANGE!
    // ABAD: 2.668 MCP_FE: 1.970 PIP_FE: 0.787

    // TODO: MERGE JACOBIAN FUNCTIONS
    robot_jacobian = cMaestroDigit::computeRobotJacobian(joint_angles[1],joint_angles[3]);
    computeForwardKinematics(joint_angles[1],joint_angles[3]);

    theta(0) = a_globalRot(0);
    theta(1) = a_globalRot(1);
    theta(2) = a_globalRot(2);
    theta(3) =  2.668 + pi - joint_angles[0];
    theta(4) = theta_MCP - 3.14/7 - 2*pi;
    theta(5) = theta_PIP;
    theta(6) = 0.33 * theta_PIP;

    //auto T = computeFKSpaceFrame(theta);
    T_space = computeFKSpaceFrame(theta);

    return T_space.block<3,1>(0,3);
}

// -----------------------------------------------------------------//

Matrix4d cMaestroDigit::computeFKToJointSpaceFrame(VectorXd a_theta)
{

    auto ret = FKinSpace(M,S_space.block<6,3>(0,0),a_theta);
    return ret;
}

// -----------------------------------------------------------------//

Matrix4d cMaestroDigit::computeFKToJointBodyFrame(VectorXd a_theta)
{
    auto ret = FKinBody(M,B_body.block<6,3>(0,0),a_theta);
    return ret;
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
    //std::cout << theta_proxy.tail(4).transpose() <<std::endl;
    return theta_proxy.tail(4);
}


// -----------------------------------------------------------------//

double* cMaestroDigit::commandJointTorqueProxy(double K ,  double B , double dt)
{

    auto joint_torque = K*(theta_proxy - theta);

    double pjoint_torque[2];
    pjoint_torque[0] = joint_torque(0);
    pjoint_torque[1] = joint_torque(1);

    return pjoint_torque;
}

// -----------------------------------------------------------------//

double* cMaestroDigit::commandJointTorqueInverseDynamics(double K ,  double B)
{

    auto joint_torque = K*(theta_proxy - theta);

    double pjoint_torque[2];
    pjoint_torque[0] = 0;
    pjoint_torque[1] = 0;

    return pjoint_torque;
}


// -----------------------------------------------------------------//

bool cMaestroDigit::computeIKBodyFrame(const MatrixXd T, MatrixXd& Tsb, VectorXd& a_theta,
                                           int max_it, double eomg, double ev)
{
    MatrixXd B = B_body.block<6,4>(0,3);
    return IKinBody(B,M,T,Tsb,a_theta,eomg,ev);
}

// -----------------------------------------------------------------//

bool cMaestroDigit::computeIKSpaceFrame(const MatrixXd T, MatrixXd& Tsb,  VectorXd& a_theta,
                                            int max_it, double eomg, double ev)
{
    MatrixXd S = S_space.block<6,4>(0,3);
    return IKinSpace(S,M,T,Tsb,a_theta,eomg,ev);
}

// -----------------------------------------------------------------//

// this function computes and returns the robot jacobian
double* cMaestroDigit::computeRobotJacobian(double phi3, double psy2)
{

    //FORWARD KINEMATICS
//    cout << "Start Forward Kinematics";
    double L1, L2, L3, xA, yA, Px, Py, Ix, Iy, Dx;
    L1 = 74;    L2 = 47.5;      L3 = 41.5;
    xA = 30;    yA = 45;    Px = 40;    Py = 17.5;    Ix = 15;    Iy = 15;

    double q1 = phi3 + 25*(M_PI/180);
    double q2 = psy2 + 154*(M_PI/180);

    double q_vec[2] = {q1, q2};
    double L_vec[3] = {L1, L2, L3};
    double F_vec[6] = {xA, yA, Px, Py, Ix, Iy};
    double theta_finger[4];

    double J[8];
    cMaestroDigit::Jacobian_RobotToFinger_toCpp(q_vec,L_vec,F_vec, J);

    return J;
}

// -----------------------------------------------------------------//

void cMaestroDigit::computeForwardKinematics(double phi3, double psy2)
{
    // APPROXIMATE OFFSETS IN THE ZERO CONFIGURATION
    // SUBJECT TO CHANGE!
    // ABAD: 2.668 MCP_FE: 1.970 PIP_FE: 0.787

    // BASED ON MODEL THE APPROXIMATE PULLEY ANGLES AT THE ZERO CONFINGURATION
    // MCP/PHI3 : 22*pi/180 + (pi - math.atan(45/30))
    // PIP/PSY2 : 40*pi/180 + (pi - math.atan(45/30))

    // OFFSETS
    phi3 = phi3 - ((1.970) - (22*pi/180 + (pi - atan(45/30))));
    psy2 = psy2 - ((0.787) - (40*pi/180 + (pi - atan(45/30))));

    // parameters KL1
    double a1 = 54;
    double b1 = 75;
    double c1 = 17.5;
    double l1 = 45;
    double phi1 = atan(45/30);

    double a2 = 41.50;
    double b2 = sqrt(pow(15,2)+pow(12,2));
    double d2 = 48.00;
    double psys = atan(15/15);

    double phi4 =2*atan((a1 - b1 - pow(((pow(a1,2) + pow(b1,2) - pow(c1,2) + 2*a1*b1*(2*pow(cos(phi3/2),2) - 1))/pow(cos(phi3/2),4)),(1/2))*(cos(phi3)/2 + 1/2) + b1*(cos(phi3) + 1))/(c1 - b1*sin(phi3)));
    double d1 = pow(((pow(a1,2) + pow(b1,2) - pow(c1,2) + 2*a1*b1*(2*pow(cos(phi3/2),2) - 1))/pow(cos(phi3/2),4)),(1/2))*(cos(phi3)/2 + 1/2);

    double psy1 = atan(pow(c1,2) / (l1 - d1));
    double c2 = sqrt(pow(c1,2) + pow((l1 - d1),2));

    double psy3 = -2*atan((((cos(psy2) + 1)*pow((-(pow(a2,4) + pow(b2,4) + pow(c2,4) + pow(d2,4) - 2*pow(a2,2)*pow(b2,2) - 2*pow(a2,2)*pow(c2,2) + 2*pow(a2,2)*pow(d2,2) - 2*pow(b2,2)*pow(c2,2) - 2*pow(b2,2)*pow(d2,2) - 2*pow(c2,2)*pow(d2,2) + 4*pow(a2,2)*pow(d2,2)*pow(cos(psy2),2) + 4*a2*pow(d2,3)*cos(psy2) + 4*pow(a2,3)*d2*cos(psy2) - 4*a2*pow(b2,2)*d2*cos(psy2) - 4*a2*pow(c2,2)*d2*cos(psy2))/pow(cos(psy2/2),4)),(1/2)))/2 - 2*a2*b2*sin(psy2))/(pow(a2,2) + 2*cos(psy2)*a2*b2 + 2*cos(psy2)*a2*d2 + pow(b2,2) + 2*b2*d2 - pow(c2,2) + pow(d2,2)));
    double psy4 = 2*atan((((cos(psy2) + 1)*pow((-(pow(a2,4) + pow(b2,4) + pow(c2,4) + pow(d2,4) - 2*pow(a2,2)*pow(b2,2) - 2*pow(a2,2)*pow(c2,2) + 2*pow(a2,2)*pow(d2,2) - 2*pow(b2,2)*pow(c2,2) - 2*pow(b2,2)*pow(d2,2) - 2*pow(c2,2)*pow(d2,2) + 4*pow(a2,2)*pow(d2,2)*pow(cos(psy2),2) + 4*a2*pow(d2,3)*cos(psy2) + 4*pow(a2,3)*d2*cos(psy2) - 4*a2*pow(b2,2)*d2*cos(psy2) - 4*a2*pow(c2,2)*d2*cos(psy2))/pow(cos(psy2/2),4)),(1/2)))/2 + 2*a2*c2*sin(psy2))/(pow(a2,2) + 2*cos(psy2)*a2*c2 + 2*cos(psy2)*a2*d2 - pow(b2,2) + pow(c2,2) + 2*c2*d2 + pow(d2,2)));

    theta_MCP = 2*pi - phi1 - phi4;
    theta_PIP = - psy3;

    //std::cout << "phi3 :" << phi3 << "psy2 :" << psy2 << "theta_MCP : " << theta_MCP << "theta_PIP: " << theta_PIP << std::endl;
    //theta_b2_c2 = 2*pi - (pi - psy2) - psy4 - (pi - psy2 - psy3);
    //PIP = 2*pi - theta_b2_c2 - psy1 - self.idx.psys;
    //MCP = 2*pi - self.idx.phi1 - phi4;

}

// Function Definitions
void cMaestroDigit::Jacobian_RobotToFinger_toCpp(const double q_vec[2], const double L_vec[3],
                                  const double F_vec[6], double J[8])
{
    double x_tmp;
    double b_x_tmp;
    double c_x_tmp;
    double x;
    double a_tmp_tmp;
    double a_tmp;
    double b_a_tmp_tmp;
    double c_a_tmp_tmp;
    double J_tmp;
    double b_J_tmp;
    double J_tmp_tmp_tmp;
    double J_tmp_tmp;
    double d_a_tmp_tmp;
    double e_a_tmp_tmp;
    double b_a_tmp;
    double f_a_tmp_tmp;
    double c_a_tmp;
    double d_a_tmp;
    double e_a_tmp;
    double f_a_tmp;
    double g_a_tmp;
    double h_a_tmp;
    double i_a_tmp;
    double g_a_tmp_tmp;
    double h_a_tmp_tmp;
    double i_a_tmp_tmp;
    double j_a_tmp_tmp;
    double k_a_tmp_tmp;
    double j_a_tmp;
    double l_a_tmp_tmp;
    double k_a_tmp;
    double l_a_tmp;
    double m_a_tmp;
    double m_a_tmp_tmp;
    double n_a_tmp;
    double n_a_tmp_tmp;
    double o_a_tmp;
    double o_a_tmp_tmp;
    double p_a_tmp;
    double q_a_tmp;
    double b_J_tmp_tmp;
    double c_J_tmp;
    double d_J_tmp;
    double e_J_tmp;
    double f_J_tmp;
    double g_J_tmp;
    double h_J_tmp;
    double i_J_tmp;
    double j_J_tmp;
    double k_J_tmp;

    x_tmp = std::sin(q_vec[0]);
    b_x_tmp = std::cos(q_vec[0]);
    c_x_tmp = F_vec[3] * F_vec[3];
    x = std::sqrt(((((L_vec[0] * L_vec[0] - 2.0 * b_x_tmp * L_vec[0] * F_vec[0]) -
                     2.0 * x_tmp * L_vec[0] * F_vec[1]) - c_x_tmp) + F_vec[0] *
                                                                     F_vec[0]) + F_vec[1] * F_vec[1]);
    J[0] = (2.0 * L_vec[0] * F_vec[0] * x_tmp - 2.0 * L_vec[0] * F_vec[1] *
                                                b_x_tmp) / (2.0 * x);
    J[4] = 0.0;
    a_tmp_tmp = L_vec[0] * x_tmp;
    a_tmp = (F_vec[3] + F_vec[1]) - a_tmp_tmp;
    b_a_tmp_tmp = L_vec[0] * b_x_tmp;
    c_a_tmp_tmp = (x + F_vec[0]) - b_a_tmp_tmp;
    J_tmp = a_tmp * a_tmp;
    b_J_tmp = c_a_tmp_tmp * c_a_tmp_tmp / J_tmp + 1.0;
    J[1] = -(2.0 * ((a_tmp_tmp + (2.0 * L_vec[0] * F_vec[0] * std::sin(q_vec[0]) -
                                  2.0 * L_vec[0] * F_vec[1] * std::cos(q_vec[0])) / (2.0 * x)) /
                    a_tmp + b_a_tmp_tmp * c_a_tmp_tmp / J_tmp)) / b_J_tmp;
    J[5] = 0.0;
    J_tmp_tmp_tmp = std::cos(q_vec[1]);
    J_tmp_tmp = std::sin(q_vec[1]);
    a_tmp_tmp = F_vec[2] - x;
    b_a_tmp_tmp = L_vec[1] * L_vec[1];
    c_a_tmp_tmp = F_vec[5] * F_vec[5];
    d_a_tmp_tmp = F_vec[4] * F_vec[4];
    e_a_tmp_tmp = L_vec[2] * L_vec[2];
    b_a_tmp = (((b_a_tmp_tmp - c_a_tmp_tmp) - d_a_tmp_tmp) + e_a_tmp_tmp) +
              c_x_tmp;
    f_a_tmp_tmp = 2.0 * J_tmp_tmp_tmp;
    c_a_tmp = f_a_tmp_tmp + 2.0 * L_vec[1] / L_vec[2];
    d_a_tmp = 2.0 * L_vec[1] * J_tmp_tmp_tmp;
    e_a_tmp = a_tmp_tmp * a_tmp_tmp;
    x_tmp = c_x_tmp + e_a_tmp;
    f_a_tmp = std::sqrt(x_tmp);
    b_x_tmp = b_a_tmp + e_a_tmp;
    g_a_tmp = L_vec[2] * f_a_tmp;
    h_a_tmp = (c_a_tmp - b_x_tmp / g_a_tmp) - d_a_tmp / f_a_tmp;
    i_a_tmp = L_vec[1] / L_vec[2];
    g_a_tmp_tmp = 2.0 * d_a_tmp_tmp;
    h_a_tmp_tmp = 2.0 * c_a_tmp_tmp;
    i_a_tmp_tmp = 2.0 * b_a_tmp_tmp;
    j_a_tmp_tmp = 2.0 * e_a_tmp_tmp;
    k_a_tmp_tmp = 2.0 * c_x_tmp;
    j_a_tmp = (((i_a_tmp_tmp - h_a_tmp_tmp) - g_a_tmp_tmp) + j_a_tmp_tmp) +
              k_a_tmp_tmp;
    l_a_tmp_tmp = 4.0 * J_tmp_tmp_tmp;
    k_a_tmp = l_a_tmp_tmp + 4.0 * L_vec[1] / L_vec[2];
    l_a_tmp = 4.0 * L_vec[1] * J_tmp_tmp_tmp;
    m_a_tmp = 2.0 * J_tmp_tmp;
    m_a_tmp_tmp = 2.0 * e_a_tmp;
    n_a_tmp = j_a_tmp + m_a_tmp_tmp;
    n_a_tmp_tmp = L_vec[1] / f_a_tmp + 1.0;
    o_a_tmp = (i_a_tmp + J_tmp_tmp_tmp * n_a_tmp_tmp) + b_x_tmp / (2.0 * L_vec[2] *
                                                                   f_a_tmp);
    g_a_tmp = (k_a_tmp - n_a_tmp / g_a_tmp) - l_a_tmp / f_a_tmp;
    o_a_tmp_tmp = J_tmp_tmp * J_tmp_tmp;
    p_a_tmp = std::sqrt(2.0 * o_a_tmp_tmp + o_a_tmp * g_a_tmp / 2.0);
    q_a_tmp = m_a_tmp - 1.4142135623730951 * p_a_tmp;
    b_J_tmp_tmp = 2.0 * L_vec[0] * F_vec[0] * std::sin(q_vec[0]) - 2.0 * L_vec[0] *
                                                                   F_vec[1] * std::cos(q_vec[0]);
    J_tmp = b_J_tmp_tmp * a_tmp_tmp;
    c_J_tmp = L_vec[2] * x;
    d_J_tmp = 2.0 * L_vec[2] * x;
    e_J_tmp = 2.0 * b_J_tmp_tmp * a_tmp_tmp;
    f_J_tmp = L_vec[1] * J_tmp_tmp_tmp * b_J_tmp_tmp * a_tmp_tmp;
    g_J_tmp = 4.0 * L_vec[2] * x;
    h_J_tmp = pow(x_tmp, 1.5);
    i_J_tmp = x * h_J_tmp;
    j_J_tmp = c_J_tmp * f_a_tmp;
    k_J_tmp = d_J_tmp * h_J_tmp;
    b_x_tmp *= J_tmp;
    x_tmp = 2.0 * p_a_tmp;
    p_a_tmp = F_vec[3] * b_J_tmp_tmp / (2.0 * x * (c_x_tmp / e_a_tmp + 1.0) *
                                        e_a_tmp);
    J[2] = ((p_a_tmp - 2.0 * ((L_vec[0] * std::sin(q_vec[0]) + (2.0 * L_vec[0] *
                                                                F_vec[0] * std::sin(q_vec[0]) - 2.0 * L_vec[0] * F_vec[1] * std::
    cos(q_vec[0])) / (2.0 * x)) / ((F_vec[3] + F_vec[1]) - L_vec[0] *
                                                           std::sin(q_vec[0])) + L_vec[0] * std::cos(q_vec[0]) * ((x + F_vec[0])
                                                                                                                  - L_vec[0] * std::cos(q_vec[0])) / (a_tmp * a_tmp)) / b_J_tmp) -
            2.0 * ((2.0 * J_tmp_tmp - 1.4142135623730951 * std::sqrt(2.0 *
                                                                     (J_tmp_tmp * J_tmp_tmp) + ((i_a_tmp + J_tmp_tmp_tmp * (L_vec[1] /
                                                                                                                            std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp) + 1.0)) + (b_a_tmp + a_tmp_tmp *
                                                                                                                                                                                            a_tmp_tmp) / (2.0 * L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) *
                                                                                               ((k_a_tmp - (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) / (L_vec[2] *
                                                                                                                                                        std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) - l_a_tmp / std::sqrt(c_x_tmp +
                                                                                                                                                                                                                           a_tmp_tmp * a_tmp_tmp)) / 2.0)) * ((b_x_tmp / k_J_tmp - J_tmp / j_J_tmp) +
                                                                                                                                                                                                                                                              L_vec[1] * J_tmp_tmp_tmp * b_J_tmp_tmp * a_tmp_tmp / i_J_tmp) /
                   (h_a_tmp * h_a_tmp) + 1.4142135623730951 * (o_a_tmp * ((d_a_tmp
                                                                           * b_J_tmp_tmp * a_tmp_tmp / i_J_tmp - e_J_tmp / j_J_tmp) + J_tmp *
                                                                                                                                      n_a_tmp / k_J_tmp) / 2.0 - ((b_x_tmp / (g_J_tmp * h_J_tmp) - J_tmp /
                                                                                                                                                                                                   (d_J_tmp * f_a_tmp)) + f_J_tmp / (2.0 * x * h_J_tmp)) * g_a_tmp /
                                                                                                                                                                 2.0) / (x_tmp * h_a_tmp)) / (q_a_tmp * q_a_tmp / (h_a_tmp * h_a_tmp)
                                                                                                                                                                                              + 1.0)) - 1.0;
    J[6] = -(2.0 * ((f_a_tmp_tmp + 1.4142135623730951 * (((4.0 * J_tmp_tmp - 4.0 *
                                                                             L_vec[1] * J_tmp_tmp / f_a_tmp) * o_a_tmp / 2.0 - l_a_tmp_tmp * J_tmp_tmp) +
                                                         J_tmp_tmp * n_a_tmp_tmp * g_a_tmp / 2.0) / x_tmp) / h_a_tmp +
                    q_a_tmp * (m_a_tmp - 2.0 * L_vec[1] * J_tmp_tmp / f_a_tmp) /
                    (h_a_tmp * h_a_tmp))) / (q_a_tmp * q_a_tmp / (h_a_tmp *
                                                                  h_a_tmp) + 1.0);
    a_tmp = d_a_tmp_tmp + c_a_tmp_tmp;
    f_a_tmp = std::sqrt(a_tmp);
    b_x_tmp = L_vec[2] * f_a_tmp;
    a_tmp = (((a_tmp + b_a_tmp_tmp) + e_a_tmp_tmp) - c_x_tmp) - e_a_tmp;
    e_a_tmp = (c_a_tmp + a_tmp / b_x_tmp) + d_a_tmp / f_a_tmp;
    b_a_tmp_tmp = L_vec[1] / f_a_tmp - 1.0;
    b_x_tmp = (k_a_tmp + l_a_tmp / f_a_tmp) + (((((g_a_tmp_tmp + h_a_tmp_tmp) +
                                                  i_a_tmp_tmp) + j_a_tmp_tmp) - k_a_tmp_tmp) - m_a_tmp_tmp) / b_x_tmp;
    a_tmp = (J_tmp_tmp_tmp * b_a_tmp_tmp - i_a_tmp) + a_tmp / (2.0 * L_vec[2] *
                                                               f_a_tmp);
    g_a_tmp = std::sqrt(4.0 * o_a_tmp_tmp - a_tmp * b_x_tmp);
    n_a_tmp = m_a_tmp - g_a_tmp;
    b_J_tmp = c_J_tmp * f_a_tmp;
    h_J_tmp = 2.0 * g_a_tmp;
    J[3] = (2.0 * ((J_tmp * b_x_tmp / (d_J_tmp * f_a_tmp) + e_J_tmp * a_tmp /
                                                            b_J_tmp) / (h_J_tmp * e_a_tmp) - J_tmp * n_a_tmp / (b_J_tmp *
                                                                                                                (e_a_tmp * e_a_tmp))) / (n_a_tmp * n_a_tmp / (e_a_tmp * e_a_tmp) +
                                                                                                                                         1.0) - 2.0 * ((m_a_tmp - 1.4142135623730951 * std::sqrt(2.0 *
                                                                                                                                                                                                 (J_tmp_tmp * J_tmp_tmp) + ((i_a_tmp + J_tmp_tmp_tmp * (L_vec[1] /
                                                                                                                                                                                                                                                        std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp) + 1.0)) + (b_a_tmp + a_tmp_tmp *
                                                                                                                                                                                                                                                                                                                        a_tmp_tmp) / (2.0 * L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) *
                                                                                                                                                                                                                           ((k_a_tmp - (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) / (L_vec[2] *
                                                                                                                                                                                                                                                                                    std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) - l_a_tmp / std::sqrt(c_x_tmp +
                                                                                                                                                                                                                                                                                                                                                       a_tmp_tmp * a_tmp_tmp)) / 2.0)) * ((J_tmp * (b_a_tmp + a_tmp_tmp * a_tmp_tmp)
                                                                                                                                                                                                                                                                                                                                                                                           / (d_J_tmp * pow(c_x_tmp + a_tmp_tmp * a_tmp_tmp, 1.5)) - J_tmp /
                                                                                                                                                                                                                                                                                                                                                                                                                                                     (c_J_tmp * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) + f_J_tmp /
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               (x * pow(c_x_tmp + a_tmp_tmp * a_tmp_tmp, 1.5))) / (h_a_tmp *
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   h_a_tmp) + 1.4142135623730951 * (((i_a_tmp + J_tmp_tmp_tmp * (L_vec
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 [1] / std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp) + 1.0)) + (b_a_tmp +
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             a_tmp_tmp * a_tmp_tmp) / (2.0 * L_vec[2] * std::sqrt(c_x_tmp +
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  a_tmp_tmp * a_tmp_tmp))) * ((2.0 * L_vec[1] * std::cos(q_vec[1]) * (2.0 *
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      L_vec[0] * F_vec[0] * std::sin(q_vec[0]) - 2.0 * L_vec[0] * F_vec[1] * std::
    cos(q_vec[0])) * (F_vec[2] - x) / (x * pow(c_x_tmp + a_tmp_tmp * a_tmp_tmp,
                                               1.5)) - e_J_tmp / (c_J_tmp * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) +
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              J_tmp * (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) / (d_J_tmp * pow
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (c_x_tmp + a_tmp_tmp * a_tmp_tmp, 1.5))) / 2.0 - ((J_tmp *
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (b_a_tmp + a_tmp_tmp * a_tmp_tmp) / (g_J_tmp * pow(c_x_tmp +
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            a_tmp_tmp * a_tmp_tmp, 1.5)) - J_tmp / (d_J_tmp * std::sqrt(c_x_tmp +
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        a_tmp_tmp * a_tmp_tmp))) + f_J_tmp / (2.0 * x * pow(c_x_tmp + a_tmp_tmp *
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      a_tmp_tmp, 1.5))) * ((k_a_tmp - (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) /
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) - l_a_tmp
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 / std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp)) / 2.0) / (2.0 * std::
    sqrt(2.0 * (J_tmp_tmp * J_tmp_tmp) + ((i_a_tmp + J_tmp_tmp_tmp *
                                                     (L_vec[1] / std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp) + 1.0)) + (b_a_tmp +
                                                                                                                       a_tmp_tmp * a_tmp_tmp) / (2.0 * L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp *
                                                                                                                                                                                      a_tmp_tmp))) * ((k_a_tmp - (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) /
                                                                                                                                                                                                                 (L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) -
                                                                                                                                                                                                      l_a_tmp / std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp)) / 2.0)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         * ((c_a_tmp - (b_a_tmp + a_tmp_tmp * a_tmp_tmp) / (L_vec[2] * std::
    sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) - d_a_tmp / std::sqrt(c_x_tmp +
                                                                  a_tmp_tmp * a_tmp_tmp)))) / (q_a_tmp * q_a_tmp / (h_a_tmp *
                                                                                                                    h_a_tmp) + 1.0)) + p_a_tmp;
    J_tmp = 4.0 * L_vec[1] * std::sin(q_vec[1]);
    b_J_tmp = 2.0 * L_vec[1] * std::sin(q_vec[1]);
    J[7] = 2.0 * ((f_a_tmp_tmp - ((8.0 * J_tmp_tmp_tmp * J_tmp_tmp + (4.0 * std::
    sin(q_vec[1]) + J_tmp / f_a_tmp) * a_tmp) + J_tmp_tmp * b_a_tmp_tmp *
                                                b_x_tmp) / h_J_tmp) / e_a_tmp + (m_a_tmp + b_J_tmp / f_a_tmp) * n_a_tmp /
                                                                                (e_a_tmp * e_a_tmp)) / (n_a_tmp * n_a_tmp / (e_a_tmp * e_a_tmp)
                                                                                                        + 1.0) - 2.0 * ((f_a_tmp_tmp + 1.4142135623730951 * (((4.0 * std::sin(q_vec
                                                                                                                                                                              [1]) - J_tmp / std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp)) * ((i_a_tmp +
                                                                                                                                                                                                                                             J_tmp_tmp_tmp * (L_vec[1] / std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp) + 1.0))
                                                                                                                                                                                                                                            + (b_a_tmp + a_tmp_tmp * a_tmp_tmp) / (2.0 * L_vec[2] * std::sqrt(c_x_tmp +
                                                                                                                                                                                                                                                                                                              a_tmp_tmp * a_tmp_tmp))) / 2.0 - 4.0 * std::cos(q_vec[1]) * std::sin(q_vec[1]))
                                                                                                                                                             + J_tmp_tmp * (L_vec[1] / std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp) + 1.0) *
                                                                                                                                                               ((k_a_tmp - (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) / (L_vec[2] * std::
                                                                                                                                                               sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) - l_a_tmp / std::sqrt(c_x_tmp +
                                                                                                                                                                                                                             a_tmp_tmp * a_tmp_tmp)) / 2.0) / (2.0 * std::sqrt(2.0 * (J_tmp_tmp *
                                                                                                                                                                                                                                                                                      J_tmp_tmp) + ((i_a_tmp + J_tmp_tmp_tmp * (L_vec[1] / std::sqrt(c_x_tmp +
                                                                                                                                                                                                                                                                                                                                                     a_tmp_tmp * a_tmp_tmp) + 1.0)) + (b_a_tmp + a_tmp_tmp * a_tmp_tmp) / (2.0 *
                                                                                                                                                                                                                                                                                                                                                                                                                           L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) * ((k_a_tmp -
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) / (L_vec[2] * std::sqrt(c_x_tmp +
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         a_tmp_tmp * a_tmp_tmp))) - l_a_tmp / std::sqrt(c_x_tmp + a_tmp_tmp *
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  a_tmp_tmp)) / 2.0))) / ((c_a_tmp - (b_a_tmp + a_tmp_tmp * a_tmp_tmp) /
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     (L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) - d_a_tmp / std::
    sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp)) + (m_a_tmp - 1.4142135623730951 * std::
    sqrt(2.0 * (J_tmp_tmp * J_tmp_tmp) + ((i_a_tmp + J_tmp_tmp_tmp * (L_vec[1] /
                                                                      std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp) + 1.0)) + (b_a_tmp + a_tmp_tmp *
                                                                                                                                      a_tmp_tmp) / (2.0 * L_vec[2] * std::sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) *
                                         ((k_a_tmp - (j_a_tmp + 2.0 * (a_tmp_tmp * a_tmp_tmp)) / (L_vec[2] * std::
                                         sqrt(c_x_tmp + a_tmp_tmp * a_tmp_tmp))) - l_a_tmp / std::sqrt(c_x_tmp +
                                                                                                       a_tmp_tmp * a_tmp_tmp)) / 2.0)) * (m_a_tmp - b_J_tmp / std::sqrt(c_x_tmp +
                                                                                                                                                                        a_tmp_tmp * a_tmp_tmp)) / (h_a_tmp * h_a_tmp)) / (q_a_tmp * q_a_tmp /
                                                                                                                                                                                                                          (h_a_tmp * h_a_tmp) + 1.0);
}