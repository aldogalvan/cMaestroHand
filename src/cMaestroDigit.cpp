//
// Created by aldo on 1/24/22.
//

#include "cMaestroDigit.h"

//==============================================================================
/*!
    Constructor of cMaestroDigit.
*/
//==============================================================================

cMaestroDigit::cMaestroDigit(chai3d::cHand* a_hand)
{
    // parameters
    double a_radius = 1.0;
    chai3d::cMaterialPtr a_material;

    // set cHand pointer to pointer
    *h_hand = a_hand;

}

//==============================================================================
/*!
    Destructor of cMaestroDigit.
*/
//==============================================================================

cMaestroDigit::~cMaestroDigit()
{

}


//! ROBOT CLASS!!!!!!!!!!
void cMaestroDigit::pseudoComputeJointAnglesFinger( double joint_angle_sensor_MCP, double joint_angle_sensor_MCP_PIP)
{
    double offset_MCP  = 0*M_PI/180; // [rad]
    double offset_PIP  = 0*M_PI/180; // [rad]
    double DIP_PIP_ratio = 0.77;

    double MCP = 2*(joint_angle_sensor_MCP + offset_MCP);
    double PIP = joint_angle_sensor_MCP_PIP + offset_PIP;
    double DIP = DIP_PIP_ratio*PIP;

    joint_angles[0] = 0;
    joint_angles[1] = MCP;
    joint_angles[2] = PIP;
    joint_angles[3] = DIP;

}

// Function to compute finger joint angles, inputs are in radians
Eigen::Vector4d cMaestroDigit::computeJointAnglesFinger(double joint_angle_sensor_MCP_abad, double joint_angle_sensor_MCP_fe, double joint_angle_sensor_MCP_PIP)
{

    // These values vary depending on the finger and the adjustment of sensor
    double offset_MCP_fe  = -90*M_PI/180; // [rad]
    double offset_MCP_PIP =  45*M_PI/180; // [rad]


    // This value is constant but dependent on the finger
    double DIP_PIP_ratio = 0.77; // coupling factor between PIP  (index finger 0.77, middle finger 0.75) []


    // User Finger Constants, depend on the subject and finger (inter-subject & intra-subject variability)

    // Subject dependent constants for first kinematic loop (crank-slider 4-bar)
    double theta_MGB= 120*M_PI/180; // metacarpal ground to base link a1 angle  [rad]

    double a1 = 40; // Crank-Slider Link a length [mm]
    double b1 = 73; // Crank-Slider Link b length [mm]
    double c1 = 15; // Crank-Slider Link c length [mm]

    double l1= 45; // Proximal Phalanx Length [mm]

    // Subject dependent constants for second kinematic loop (standard 4-bar)
    double a2 = 47; // 4-bar Link a length [mm]
    double b2 = 42; // 4-bar Link b length [mm]
    double c2 = 20; // 4-bar Link c length [mm]

    //! I don't know why but the equations below kind of work but they are definitely wrong lol (to be used as last resource)
    /*
    double theta_3_cs = asin((a1*( M_PI - joint_angle_sensor_MCP_fe - offset_MCP_fe - theta_MGB) - c1)/(a1 - b1))  ;
    double theta_2_cs = asin( ( (b1*sin(theta_3_cs ) + c1) / a1 ) ) ; // [rad] cross circuit
    double d1 =  -(a1*cos(theta_2_cs) - b1*cos(theta_3_cs));
    double joint_angle_FE_MCP =  theta_MGB - theta_2_cs;
    */
    // First kinematic loop  Variables (Cross Circuit Crank Slider)

    double theta_in = joint_angle_sensor_MCP_fe + offset_MCP_fe - M_PI;

    double Z1 = (-b1 + (a1/cos(theta_in)) );
    double Z2 = (tan(theta_in)*a1)/Z1;
    double Z3 = c1/Z1;

    double A1 = pow(a1,2)/pow(Z2,2) + pow(b1,2);
    double B1 = 2*b1*c1 -2*pow(a1,2)*Z3/pow(Z2,2) ;
    double C1 = pow(a1,2)*pow(Z3,2)/pow(Z2,2) - pow(a1,2) + pow(c1,2);

    //! Crank-slider angle 3 [rad]
    // double theta_3_cs = asin(- B1 - sqrt(pow(B1,2)-4*A1*C1) / (2*A1));
    // double theta_3_cs = asin(- B1 + sqrt(pow(B1,2)-4*A1*C1) / (2*A1)); // other solution
    double theta_3_cs = (- B1 - sqrt(pow(B1,2)-4*A1*C1) / (2*A1));

    // Crank-slider angle 2 [rad]
    // double theta_2_cs = asin(-1*((b1*sin(theta_3_cs-M_PI)+c1)/a1)); // [rad] crossed circuit
    double theta_2_cs = asin(-1*((b1*sin(theta_3_cs-M_PI)+c1)/a1));

    // Slider position [mm]
    double d1 =  a1*cos(theta_2_cs) - b1*cos(theta_3_cs);


    // Second Kinematic loop
    // Variable ground offset angle [rad]
    double theta_1_std = atan(c1/(l1-d1));

    // Variable ground Linkage Length [mm]
    double d2 = (l1 - d1)/cos(theta_1_std);

    // angle 2 of standard 4-bar
    double theta_2_std =  theta_1_std + theta_3_cs + joint_angle_sensor_MCP_PIP + offset_MCP_PIP;

    //  Open circuit equations for standard
    double k1 = d2/a2; double k2 = d2/c2; double k3 = (pow(a2,2) - pow(b2,2) + pow(c2,2) + pow(d2,2))/(2*a2*c2);

    double A2 = cos(theta_2_std) - k1 - k2*cos(theta_2_std) + k3;
    double B2 = -2*sin(theta_2_std);
    double C2 = k1 - (k2 + 1)*cos(theta_2_std) + k3;

    double theta_4_std = 2*atan((- B2 - sqrt(pow(B2,2)-4*A2*C2) ) / (2*A2));


    // Compute Joint Angles
    double joint_angle_FE_MCP =  theta_MGB - theta_2_cs;
    double joint_angle_FE_PIP = theta_4_std - theta_1_std;
    double joint_angle_FE_DIP = DIP_PIP_ratio*joint_angle_FE_PIP ;

    Eigen::Vector4d joint_angles;
    joint_angles = Eigen::Vector4d(0,joint_angle_FE_MCP,joint_angle_FE_PIP,joint_angle_FE_DIP);

    // std::cout << (joint_angle_sensor_MCP_fe + offset_MCP_fe)*180/M_PI << endl; // Value should be zero when proximal and metacarpal interfaces align
    // std::cout << theta_in*180/M_PI<< endl; // should read -180 when finger extended
    // std::cout << Z1 << ',' << Z2 << ','<< Z3 << ',' << endl;
    // std::cout << theta_3_cs << endl;  // value becomes nan only when crossing singularity
    // std::cout << theta_3_cs*180/M_PI << ',' << theta_2_cs*180/M_PI << ','<< d1 << ',' << joint_angle_FE_MCP*180/M_PI << endl;
    // std::cout << joint_angle_FE_MCP*180/M_PI<< endl;
    // std::cout << joint_angles << endl;
    //
    // std::cout.flush();

    return joint_angles;
}

/*
// Function to compute actual joint angles
Eigen::Vector4d hMaestroHand::computeJointAnglesThumb(double joint_angle_sensor_MCP, double joint_angle_sensor_CMC_MCP)
{
    Eigen::Vector4d joint_angles;

    return joint_angles;
}
 */

// Function to update the hand joint angles
void cMaestroDigit::updateJointAngles(double* joint_angles)
{

    /*
    // Values in degrees (Dec 3 2021)
    ang_sensor_MCP_abad = exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad();
    ang_sensor_MCP_fe = exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad();
    ang_sensor_MCP_PIP = exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad();
    ang_sensor_PIP = exo_joint_angle_sensor_PIP.GetExoJointAngleRad();
    ang_sensor_DIP = exo_joint_angle_sensor_DIP.GetExoJointAngleRad();
    */

    ang_sensor_MCP_abad = joint_angles[0];
    ang_sensor_MCP_fe = joint_angles[1];
    ang_sensor_MCP_PIP = joint_angles[2];
    ang_sensor_PIP = joint_angles[3];
    ang_sensor_DIP = joint_angles[4];

    // computeJointAnglesFinger();
    pseudoComputeJointAnglesFinger(0,0);

}

double* cMaestroDigit::getJointAngles()
{
    return joint_angles;
}

bool cMaestroDigit::commandJointTorque(double *joint_torque)
{
    joint_torque[0] = exo_desired_torque_MCP;
    joint_torque[1] = exo_desired_torque_PIP;

    return true;
}


// Function to compute forward kinematics (finger joints to finger tip)
void cMaestroDigit::computeForwardKinematicsFinger(void)
{

    /*
    // User finger Parameters
    double l1 = hIdxSegLengths(0); // 45 [mm]
    double l2 = hIdxSegLengths(1); // 25 [mm]
    double l3 = hIdxSegLengths(2); // 20 [mm]

    // Finger joint angles
    double joint_angle_abad_MCP = hIdxJointAngles(0);
    double joint_angle_fe_MCP   = hIdxJointAngles(1);
    double joint_angle_PIP      = hIdxJointAngles(2);
    double joint_angle_DIP      = hIdxJointAngles(3);

    //! change to matrix operations when integrating MCP abduction and adduction
    // P0: MCP coordinates w.r.t world frame
    double xMCP = cursor_MCP->getGlobalPos().x();
    double yMCP = cursor_MCP->getGlobalPos().y();
    double zMCP = cursor_MCP->getGlobalPos().z();

    // P1: PIP coordinates w.r.t world frame
    double xPIP = xMCP + l1*sin(joint_angle_fe_MCP) ;
    double yPIP = yMCP + l1*cos(joint_angle_fe_MCP) ;
    double zPIP = zMCP ;

    // P2: DIP coordinates with respect to  P0
    double xDIP = xPIP + l2*sin( joint_angle_fe_MCP + joint_angle_PIP) ;
    double yDIP = yPIP + l2*cos( joint_angle_fe_MCP + joint_angle_PIP) ;
    double zDIP = zPIP + 0 ;

    // P3: Finger tip coordinates with respect to  P0
    double xft = xDIP +l3*sin(joint_angle_fe_MCP + joint_angle_PIP + joint_angle_DIP ) ;
    double yft = yDIP +l3*cos(joint_angle_fe_MCP + joint_angle_PIP + joint_angle_DIP ) ;
    double zft = zDIP + 0 ;


    cursor_PIP->setLocalPos(chai3d::cVector3d(xPIP,yPIP,zPIP))  ; // PIP joint position w.r.t World Frame
    cursor_DIP->setLocalPos(chai3d::cVector3d(xDIP,yDIP,zDIP) ); // DIP joint position w.r.t World Frame
    cursor_TIP->setLocalPos(chai3d::cVector3d(xft,yft,zft)) ; // Finger-tip position w.r.t World Frame

    //hIdxFingerVel = (hIdxFingerPos + hIdxLastPos);

    // std::cout << hIdxMCPpos <<endl; std::cout <<'-'<< endl;
    // std::cout << joint_angle_fe_MCP <<','<<joint_angle_PIP<<','<< joint_angle_DIP << endl;
    // std::cout << xPIP <<','<< yPIP<<','<< zPIP << endl;
    // std::cout << xDIP <<','<< yDIP<<','<< zDIP << endl;
    // std::cout << xft <<','<< yft<<','<< zft << endl;
    // std::cout.flush();
    */
}


// Function to computer inverse dynamics
Eigen::Vector2d cMaestroDigit::computeInverseDynamics_Finger(Eigen::Vector3d force)
{
    /*
    Eigen::Vector3d P0(cursor_MCP->getLocalPos().x(),
                       cursor_MCP->getLocalPos().y(),
                       cursor_MCP->getLocalPos().z());
    Eigen::Vector3d P1(cursor_PIP->getLocalPos().x(),
                       cursor_PIP->getLocalPos().y(),
                       cursor_PIP->getLocalPos().z());
    Eigen::Vector3d P2(cursor_DIP->getLocalPos().x(),
                       cursor_DIP->getLocalPos().y(),
                       cursor_DIP->getLocalPos().z());
    Eigen::Vector3d P3(cursor_TIP->getLocalPos().x(),
                       cursor_TIP->getLocalPos().y(),
                       cursor_TIP->getLocalPos().z());

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

    // std::cout << force(0)<<','<<force(1)<<','<<force(2)<< endl;
    // std::cout << L3(0)<<','<< L3(1)<<','<< L3(2) << endl;
    // std::cout << joint_torque_DIP(0)<<','<<joint_torque_DIP(1)<<','<<joint_torque_DIP(2)<< endl;
    // std::cout << joint_torque_DIP << endl;
    // std::cout << joint_torque_PIP << endl;
    // std::cout << joint_torque_MCP << endl;
    //std::cout << joint_torque_MCP<<','<< joint_torque_PIP << endl;
    //std::cout.flush();



    // compute exo joint torques
    double exo_torque_MCP = joint_torque_MCP;
    double exo_torque_PIP = -joint_torque_PIP;

    return Eigen::Vector2d (exo_torque_MCP, exo_torque_PIP);
    */
}


//! Command the desired exo torque back to esmacat finger class
void cMaestroDigit::commandExoTorqueFinger(double exo_torque_MCP, double exo_torque_PIP)
{
    exo_desired_torque_MCP = exo_torque_MCP;
    exo_desired_torque_PIP = exo_torque_PIP;
}

//! Updates the visualizer of hand
void cMaestroDigit::updateGraphics()
{

}

