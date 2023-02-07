//
// Created by aldo on 11/23/21.
//

#include <stdio.h>
#include <chrono>
#include "my_app.h"


double movingAvg(double *ptrArrNumbers, int len, double nextNum)
{
    double sum = 0;

    for (int i = 1 ; i < len; i++)
    {
        ptrArrNumbers[i-1] = ptrArrNumbers[i];
        sum += ptrArrNumbers[i];
    }

    ptrArrNumbers[len - 1] = nextNum;
    sum += nextNum;

    //return the average
    return sum / (double)len;
}

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
    joint_angles[0] = indexFinger.smoothedSensorData[0];
    joint_angles[1] = indexFinger.smoothedSensorData[1];
    joint_angles[2] = indexFinger.smoothedSensorData[2];
    joint_angles[3] = indexFinger.smoothedSensorData[3];
    joint_angles[4] = indexFinger.smoothedSensorData[4];

    //! Update the middle finger
    joint_angles[5] = middleFinger.smoothedSensorData[0];
    joint_angles[6] = middleFinger.smoothedSensorData[1];
    joint_angles[7] = middleFinger.smoothedSensorData[2];
    joint_angles[8] = middleFinger.smoothedSensorData[3];
    joint_angles[9] = middleFinger.smoothedSensorData[4];

    //! Update the thumb
    joint_angles[10] = thumb.smoothedSensorData[0];
    joint_angles[11] = thumb.smoothedSensorData[1];
    joint_angles[12] = thumb.smoothedSensorData[2];
    joint_angles[13] = thumb.smoothedSensorData[3];
    joint_angles[14] = thumb.smoothedSensorData[4];
    joint_angles[15] = thumb.smoothedSensorData[5];

    flag = true;
    return flag;
}

void my_app::smoothSensorData()
{
    indexFinger.smoothedSensorData[0] = movingAvg(indexFinger.MCP_abad_buffer,10,indexFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad());
    indexFinger.smoothedSensorData[1] = movingAvg(indexFinger.MCP_fe_buffer,10,indexFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad());
    indexFinger.smoothedSensorData[2] = movingAvg(indexFinger.MCP_PIP_buffer,10,indexFinger.exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad());
    indexFinger.smoothedSensorData[3] = movingAvg(indexFinger.PIP_buffer,10,indexFinger.exo_joint_angle_sensor_PIP.GetExoJointAngleRad());
    indexFinger.smoothedSensorData[4] = movingAvg(indexFinger.DIP_buffer,10,indexFinger.exo_joint_angle_sensor_DIP.GetExoJointAngleRad());

    //! Update the middle finger
    middleFinger.smoothedSensorData[0] = movingAvg(middleFinger.MCP_abad_buffer,10,middleFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad());
    middleFinger.smoothedSensorData[1] = movingAvg(middleFinger.MCP_abad_buffer,10,middleFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad());
    middleFinger.smoothedSensorData[2] = movingAvg(middleFinger.MCP_abad_buffer,10,middleFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad());
    middleFinger.smoothedSensorData[3] = movingAvg(middleFinger.MCP_abad_buffer,10,middleFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad());
    middleFinger.smoothedSensorData[4] = movingAvg(middleFinger.MCP_abad_buffer,10,middleFinger.exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad());

    //! Update the thumb
    thumb.smoothedSensorData[0] = movingAvg(thumb.CMC_abad_buffer,10,thumb.exo_joint_angle_sensor_CMC_abad.GetExoJointAngleRad());
    thumb.smoothedSensorData[1] = movingAvg(thumb.CMC_fe_buffer,10,thumb.exo_joint_angle_sensor_CMC_fe.GetExoJointAngleRad());
    thumb.smoothedSensorData[2] = movingAvg(thumb.CMC_MCP_buffer,10,thumb.exo_joint_angle_sensor_CMC_MCP.GetExoJointAngleRad());
    thumb.smoothedSensorData[3] = movingAvg(thumb.MCP_buffer,10,thumb.exo_joint_angle_sensor_MCP.GetExoJointAngleRad());
    thumb.smoothedSensorData[4] = movingAvg(thumb.MCP_IP_buffer,10,thumb.exo_joint_angle_sensor_MCP_IP.GetExoJointAngleRad());
    thumb.smoothedSensorData[5] = movingAvg(thumb.IP_buffer,10,thumb.exo_joint_angle_sensor_IP.GetExoJointAngleRad());
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
    motorsHomed = true;
    if(!motorsHomed && run_flag){
        motorsHomed = pHW_interface->homeMotors();
        // paria's addition to not home motor 8
        //motorsHomed = pHW_interface->homeMotors_but_eight(); // modified to only home 5&6
        startTime = elapsed_time_ms;
        motorsHomed = 1;
    }
    /*
    else if(!sensorsZeroed && run_flag)
    {
        cout << "Please place your hand in the zero configuration." << endl;
        cout << "Press 'S' to save and continue" << endl << flush;


        for (;;)
        {
            char input;
            scanf("%c", &input);
            if (input == 's' || input == 'S') {
                cout << "Collecting" << endl << flush;

                // collect data for two seconds
                auto StartTime = std::chrono::system_clock::now();
                auto EndTime = std::chrono::system_clock::now();
                while (std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime).count() < 2000) {
                    // Updates the sensor data
                    pHW_interface->UpdateAllSensorData();
                    // smooth sensor data
                    smoothSensorData();
                    EndTime = std::chrono::system_clock::now();
                }

                indexFinger.zero_offset[0] = indexFinger.smoothedSensorData[0];
                indexFinger.zero_offset[1] = indexFinger.smoothedSensorData[1];
                indexFinger.zero_offset[2] = indexFinger.smoothedSensorData[2];
                indexFinger.zero_offset[3] = indexFinger.smoothedSensorData[3];
                indexFinger.zero_offset[4] = indexFinger.smoothedSensorData[4];

                middleFinger.zero_offset[0] = middleFinger.smoothedSensorData[0];
                middleFinger.zero_offset[1] = middleFinger.smoothedSensorData[1];
                middleFinger.zero_offset[2] = middleFinger.smoothedSensorData[2];
                middleFinger.zero_offset[3] = middleFinger.smoothedSensorData[3];
                middleFinger.zero_offset[4] = middleFinger.smoothedSensorData[4];

                thumb.zero_offset[0] = thumb.smoothedSensorData[0];
                thumb.zero_offset[1] = thumb.smoothedSensorData[1];
                thumb.zero_offset[2] = thumb.smoothedSensorData[2];
                thumb.zero_offset[3] = thumb.smoothedSensorData[3];
                thumb.zero_offset[4] = thumb.smoothedSensorData[4];
                break;
            }
        }
    }*/
    else if( run_flag){

        running_cnt++;
        //cout << running_cnt << endl;
        // Updates the sensor data
        pHW_interface->UpdateAllSensorData();

        // smooth sensor data
        smoothSensorData();

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