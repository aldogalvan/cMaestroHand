//
// Created by aldo on 11/23/21.
//

#include "my_app.h"

my_app::my_app()
{
    cout<< "Constructing my project..." << endl;
    cout<< "My project constructed!!!" <<endl;
}

my_app::~my_app(){

    // Deletes the interface
    delete pHW_interface;

    cout << "My project Destructed!!!" <<endl;
}

bool my_app::updateJointAngles(double *joint_angles)
{

    bool flag = false;

    //! Update the index finger
    joint_angles[0] = indexFinger.smoothed_sensor_values[0]; // indexFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad();
    joint_angles[1] = indexFinger.smoothed_sensor_values[1]; // indexFinger.exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad();
    joint_angles[2] = indexFinger.smoothed_sensor_values[2]; // indexFinger.exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad();
    joint_angles[3] = indexFinger.smoothed_sensor_values[3];
    joint_angles[4] = indexFinger.smoothed_sensor_values[4];

    //! Update the middle finger
    joint_angles[5] = middleFinger.smoothed_sensor_values[0];
    joint_angles[6] = middleFinger.smoothed_sensor_values[1];
    joint_angles[7] = middleFinger.smoothed_sensor_values[2];
    joint_angles[8] = middleFinger.smoothed_sensor_values[3];
    joint_angles[9] = middleFinger.smoothed_sensor_values[4];

    //! Update the thumb
    joint_angles[10] = thumb.smoothed_sensor_values[0]; // [deg]
    joint_angles[11] = thumb.smoothed_sensor_values[1];     // [deg]
    joint_angles[12] = thumb.smoothed_sensor_values[2];   // [deg]
    joint_angles[13] = thumb.smoothed_sensor_values[3];           // [deg]
    joint_angles[14] = thumb.smoothed_sensor_values[4];     // [deg]
    joint_angles[15] = thumb.smoothed_sensor_values[5];

    flag = true;
    return flag;
}

void my_app::smoothSensorData(const int N)
{
    //! Smooth the index finger
    indexFinger.smoothed_sensor_values[0] = indexFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad();//-= indexFinger.smoothed_sensor_values[0] / N;
    indexFinger.smoothed_sensor_values[1] = indexFinger.exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad(); //-= indexFinger.smoothed_sensor_values[1] / N;
    indexFinger.smoothed_sensor_values[2] = indexFinger.exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad(); //-= indexFinger.smoothed_sensor_values[2] / N;
    indexFinger.smoothed_sensor_values[3] = indexFinger.exo_joint_angle_sensor_PIP.GetExoJointAngleRad(); //-= indexFinger.smoothed_sensor_values[3] / N;
    indexFinger.smoothed_sensor_values[4] = indexFinger.exo_joint_angle_sensor_DIP.GetExoJointAngleRad();//-= indexFinger.smoothed_sensor_values[4] / N;

    //indexFinger.smoothed_sensor_values[0] += indexFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad() / N;
    //indexFinger.smoothed_sensor_values[1] += indexFinger.exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad() / N;
    //indexFinger.smoothed_sensor_values[2] += indexFinger.exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad() / N;
    //indexFinger.smoothed_sensor_values[3] += indexFinger.exo_joint_angle_sensor_PIP.GetExoJointAngleRad() / N;
    //indexFinger.smoothed_sensor_values[4] += indexFinger.exo_joint_angle_sensor_DIP.GetExoJointAngleRad() / N;

    //! Update the middle finger
    middleFinger.smoothed_sensor_values[0] = middleFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad();
    middleFinger.smoothed_sensor_values[1] = middleFinger.exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad();
    middleFinger.smoothed_sensor_values[2] = middleFinger.exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad();
    middleFinger.smoothed_sensor_values[3] = middleFinger.exo_joint_angle_sensor_PIP.GetExoJointAngleRad();
    middleFinger.smoothed_sensor_values[4] = middleFinger.exo_joint_angle_sensor_DIP.GetExoJointAngleRad();

    //middleFinger.smoothed_sensor_values[0] += middleFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad() / N;
    //middleFinger.smoothed_sensor_values[1] += middleFinger.exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad() / N;
    //middleFinger.smoothed_sensor_values[2] += middleFinger.exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad() / N;
    //middleFinger.smoothed_sensor_values[3] += middleFinger.exo_joint_angle_sensor_PIP.GetExoJointAngleRad() / N;
    //middleFinger.smoothed_sensor_values[4] += middleFinger.exo_joint_angle_sensor_DIP.GetExoJointAngleRad() / N;

    //! Update the thumb
    thumb.smoothed_sensor_values[0] = thumb.exo_joint_angle_sensor_CMC_abad.GetExoJointAngleRad() ; // [deg]
    thumb.smoothed_sensor_values[1] = thumb.exo_joint_angle_sensor_CMC_fe.GetExoJointAngleRad();    // [deg]
    thumb.smoothed_sensor_values[2] = thumb.exo_joint_angle_sensor_CMC_MCP.GetExoJointAngleRad();   // [deg]
    thumb.smoothed_sensor_values[3] = thumb.exo_joint_angle_sensor_MCP.GetExoJointAngleRad();          // [deg]
    thumb.smoothed_sensor_values[4] = thumb.exo_joint_angle_sensor_MCP_IP.GetExoJointAngleRad();   // [deg]
    thumb.smoothed_sensor_values[5] = thumb.exo_joint_angle_sensor_IP.GetExoJointAngleRad();

    //thumb.smoothed_sensor_values[0] += thumb.exo_joint_angle_sensor_CMC_abad.GetExoJointAngleRad() / N; // [deg]
    //thumb.smoothed_sensor_values[1] += thumb.exo_joint_angle_sensor_CMC_fe.GetExoJointAngleRad() / N;     // [deg]
    //thumb.smoothed_sensor_values[2] += thumb.exo_joint_angle_sensor_CMC_MCP.GetExoJointAngleRad() / N;   // [deg]
    //thumb.smoothed_sensor_values[3] += thumb.exo_joint_angle_sensor_MCP.GetExoJointAngleRad() / N;           // [deg]
    //thumb.smoothed_sensor_values[4] += thumb.exo_joint_angle_sensor_MCP_IP.GetExoJointAngleRad() / N;     // [deg]
    //thumb.smoothed_sensor_values[5] += thumb.exo_joint_angle_sensor_IP.GetExoJointAngleRad() / N;
}

bool my_app::commandJointTorque(double *joint_torques,double* desired_angle, double* actual_torque)
{

    //! Command index finger
    if(!indexFinger.exo_sea_MCP.TorqueControl(joint_torques[0], 0, desired_angle[0], actual_torque[0],1))
        return false;

    if(!indexFinger.exo_sea_PIP.TorqueControl(joint_torques[1], 0, desired_angle[1], actual_torque[1],0))
        return false;

    //! Command middle finger
    if(!indexFinger.exo_sea_MCP.TorqueControl(joint_torques[2], 0, desired_angle[2], actual_torque[2],1))
        return false;

    if(!indexFinger.exo_sea_PIP.TorqueControl(joint_torques[3], 0, desired_angle[3], actual_torque[3],0))
        return false;

    //! Command thumb
    if(!thumb.exo_sea_CMC_fe.TorqueControl(joint_torques[4], 0, desired_angle[4], actual_torque[4],0))
        return false;

    if(!thumb.exo_sea_CMC_abad.TorqueControl(joint_torques[5], 0, desired_angle[5], actual_torque[5],0))
        return false;

    if(!thumb.exo_sea_MCP.TorqueControl(joint_torques[6], 0, desired_angle[6], actual_torque[6],0))
        return false;

    if(!thumb.exo_sea_IP.TorqueControl(joint_torques[7], 0, desired_angle[7], actual_torque[7],0))
        return false;


    return true;
}


void my_app::connect() {

    cout << "Connecting to EtherCAT..." << endl;

    // Assigns the slaves
    assign_esmacat_slave_index(ecat_md1, 0);
    assign_esmacat_slave_index(ecat_md2, 1);
    assign_esmacat_slave_index(ecat_md3, 2);
    assign_esmacat_slave_index(ecat_md4, 3);
    assign_esmacat_slave_index(ecat_md5, 4);
    assign_esmacat_slave_index(ecat_md6, 5);
    assign_esmacat_slave_index(ecat_md7, 6);
    assign_esmacat_slave_index(ecat_md8, 7);

    // Stores slaves in an array
    ecat_array[0] = ecat_md1;
    ecat_array[1] = ecat_md2;
    ecat_array[2] = ecat_md3;
    ecat_array[3] = ecat_md4;
    ecat_array[4] = ecat_md5;
    ecat_array[5] = ecat_md6;
    ecat_array[6] = ecat_md7;
    ecat_array[7] = ecat_md8;

    // Initializes the interface
    pHW_interface = new HWInterface_Esmacat();
    pHW_interface->esmacatInterfaceConnect(ecat_array);

    // Assign all index phw interfacnesx adn parameters
    indexFinger.exo_joint_angle_sensor_MCP_abad.SetHWInterface(pHW_interface);
    indexFinger.exo_joint_angle_sensor_MCP_fe.SetHWInterface(pHW_interface);
    indexFinger.exo_joint_angle_sensor_MCP_PIP.SetHWInterface(pHW_interface);
    indexFinger.exo_joint_angle_sensor_PIP.SetHWInterface(pHW_interface);
    indexFinger.exo_joint_angle_sensor_DIP.SetHWInterface(pHW_interface);

    //N/m (Mc master: 9435K84) 800 // 11.4 mm initial length between clamps - compressed to 7.5 mm
    indexFinger.exo_sea_MCP.SetSEAParameters(5630 * 0.3, 0.15, 0, 0, -M_PI / 6, M_PI / 5, 0.025 / 2);
    // N/m (Mc master: 9435K78) Full ROM Limits: -M_PI/4,M_PI/5
    indexFinger.exo_sea_PIP.SetSEAParameters(5630 * 0.5 * 0.5, 0.05, 0, 0, -M_PI / 6, M_PI / 5, 0.0234 / 2);

    // Assign all middle phw interfacnesx and parameters
    middleFinger.exo_joint_angle_sensor_MCP_abad.SetHWInterface(pHW_interface);
    middleFinger.exo_joint_angle_sensor_MCP_fe.SetHWInterface(pHW_interface);
    middleFinger.exo_joint_angle_sensor_MCP_PIP.SetHWInterface(pHW_interface);
    middleFinger.exo_joint_angle_sensor_PIP.SetHWInterface(pHW_interface);
    middleFinger.exo_joint_angle_sensor_DIP.SetHWInterface(pHW_interface);

    //N/m (Mc master: 9435K84) 800 // 11.4 mm initial length between clamps - compressed to 7.5 mm
    middleFinger.exo_sea_MCP.SetSEAParameters(5630 * 0.3, 0.15, 0, 0, -M_PI / 6, M_PI / 5, 0.025 / 2);
    // N/m (Mc master: 9435K78) Full ROM Limits: -M_PI/4,M_PI/5
    middleFinger.exo_sea_PIP.SetSEAParameters(5630 * 0.5 * 0.5, 0.05, 0, 0, -M_PI / 6, M_PI / 5, 0.0234 / 2);

    // Assign all thumb interfaces and parameters
    thumb.exo_joint_angle_sensor_CMC_abad.SetHWInterface(pHW_interface);
    thumb.exo_joint_angle_sensor_CMC_fe.SetHWInterface(pHW_interface);
    thumb.exo_joint_angle_sensor_CMC_MCP.SetHWInterface(pHW_interface);
    thumb.exo_joint_angle_sensor_MCP.SetHWInterface(pHW_interface);
    thumb.exo_joint_angle_sensor_MCP_IP.SetHWInterface(pHW_interface);
    thumb.exo_joint_angle_sensor_IP.SetHWInterface(pHW_interface);

    thumb.exo_sea_CMC_fe.SetSEAParameters(5630 * 0.3, 0.3, 0, 0, -M_PI / 6, M_PI / 6, 0.025 /
                                                                                      2); // N/m (Mc master: 9435K84) // 11.4 mm initial length between clamps - compressed to 7.5 mm
    thumb.exo_sea_CMC_abad.SetSEAParameters(5630 * 0.3, 0.2, 0, 0, -M_PI / 5, M_PI / 5,
                                            0.025 / 2); // N/m (Mc master: 9435K84)
    thumb.exo_sea_MCP.SetSEAParameters(5630 * 0.3, 0.3, 0, 0, -M_PI / 4, M_PI / 4,
                                       0.024 / 2); // N/m (Mc master: 9435K84) 1.2
    thumb.exo_sea_IP.SetSEAParameters(5630 * 0.5 * 0.51, 0.5, 0, 0, -M_PI / 5, M_PI / 5,
                                      0.024 / 2); // N/m (Mc master: 9435K78) Change the stiffness}

}

void my_app::setup(){

    cout <<"Setting up EtherCAT..."<<endl;

    running_cnt = 0;
    startTime = 0.0;
    run_flag = true;
    motorsHomed = false; //false;

    pHW_interface->esmacatInterfaceSetup();
    pHW_interface->UpdateAllSensorData();

    // Index finger setup
    indexFinger.exo_joint_angle_sensor_MCP_abad.AssignIndexOfExoJointAngleSensorInHWInterface(7);
    indexFinger.exo_joint_angle_sensor_MCP_fe.AssignIndexOfExoJointAngleSensorInHWInterface(6);
    indexFinger.exo_joint_angle_sensor_MCP_PIP.AssignIndexOfExoJointAngleSensorInHWInterface(8);
    indexFinger.exo_joint_angle_sensor_PIP.AssignIndexOfExoJointAngleSensorInHWInterface(9);
    indexFinger.exo_joint_angle_sensor_DIP.AssignIndexOfExoJointAngleSensorInHWInterface(10);

    indexFinger.exo_sea_MCP.SetJointAngleSensorSEA(&indexFinger.exo_joint_angle_sensor_MCP_fe);
    indexFinger.exo_sea_PIP.SetJointAngleSensorSEA(&indexFinger.exo_joint_angle_sensor_PIP);

    indexFinger.exo_sea_MCP.SetMotorIndexSEA(pHW_interface, 4);
    indexFinger.exo_sea_PIP.SetMotorIndexSEA(pHW_interface, 5);

    // Middle finger setup
    middleFinger.exo_joint_angle_sensor_MCP_abad.AssignIndexOfExoJointAngleSensorInHWInterface(12);
    middleFinger.exo_joint_angle_sensor_MCP_fe.AssignIndexOfExoJointAngleSensorInHWInterface(11);
    middleFinger.exo_joint_angle_sensor_MCP_PIP.AssignIndexOfExoJointAngleSensorInHWInterface(13);
    middleFinger.exo_joint_angle_sensor_PIP.AssignIndexOfExoJointAngleSensorInHWInterface(14);
    middleFinger.exo_joint_angle_sensor_DIP.AssignIndexOfExoJointAngleSensorInHWInterface(15);

    middleFinger.exo_sea_MCP.SetJointAngleSensorSEA(&indexFinger.exo_joint_angle_sensor_MCP_fe);
    middleFinger.exo_sea_PIP.SetJointAngleSensorSEA(&indexFinger.exo_joint_angle_sensor_PIP);

    middleFinger.exo_sea_MCP.SetMotorIndexSEA(pHW_interface, 6);
    middleFinger.exo_sea_PIP.SetMotorIndexSEA(pHW_interface, 7);

    // Thumb setup
    thumb.exo_joint_angle_sensor_CMC_fe.AssignIndexOfExoJointAngleSensorInHWInterface(0);
    thumb.exo_joint_angle_sensor_CMC_abad.AssignIndexOfExoJointAngleSensorInHWInterface(1);
    thumb.exo_joint_angle_sensor_CMC_MCP.AssignIndexOfExoJointAngleSensorInHWInterface(2);
    thumb.exo_joint_angle_sensor_MCP.AssignIndexOfExoJointAngleSensorInHWInterface(3);
    thumb.exo_joint_angle_sensor_MCP_IP.AssignIndexOfExoJointAngleSensorInHWInterface(4);
    thumb.exo_joint_angle_sensor_IP.AssignIndexOfExoJointAngleSensorInHWInterface(5);

    thumb.exo_sea_CMC_fe.SetJointAngleSensorSEA(&thumb.exo_joint_angle_sensor_CMC_fe);
    thumb.exo_sea_CMC_abad.SetJointAngleSensorSEA(&thumb.exo_joint_angle_sensor_CMC_abad);
    thumb.exo_sea_MCP.SetJointAngleSensorSEA(&thumb.exo_joint_angle_sensor_MCP);
    thumb.exo_sea_IP.SetJointAngleSensorSEA(&thumb.exo_joint_angle_sensor_IP);

    thumb.exo_sea_CMC_fe.SetMotorIndexSEA(pHW_interface, 0);
    thumb.exo_sea_CMC_abad.SetMotorIndexSEA(pHW_interface, 1);
    thumb.exo_sea_MCP.SetMotorIndexSEA(pHW_interface, 2);
    thumb.exo_sea_IP.SetMotorIndexSEA(pHW_interface, 3);

    cout<< "Sensor Data Updated..."<<endl;

    cout <<"Hand Structure Assembled" << endl;
}


void my_app::loop(){

    //TODO: Implement high level control (e.g. AAN, Impedance)
    run_flag = run_flag && !kbhit(); //check for keystroke or other stop condition triggered by program


    //
    if(!motorsHomed && run_flag){
        // motorsHomed = pHW_interface->homeMotors();
        // paria's addition to not home motor 8
        motorsHomed = pHW_interface->homeMotors_but_eight(); // modified to only home 5&6
        startTime = elapsed_time_ms;
    }
    else if(run_flag){

        running_cnt++;
        cout << "manager running..." << endl;

        // Updates the sensor data
        pHW_interface->UpdateAllSensorData();

        // smooth sensor data
        smoothSensorData(10);

        // commands the output
        pHW_interface->UpdateAllOutputData();

    }
    else{
        ExitManager();
    }

}


bool my_app::ExitManager(){

    stop();

    return true;
}

// keyboard input reading
int my_app::kbhit(){
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}