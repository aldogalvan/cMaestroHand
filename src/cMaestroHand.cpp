//
// Created by aldo on 1/24/22.
//

#include "cMaestroHand.h"

using namespace chai3d;

// Pi
    double Pi = 3.14159;

// Initial hand pose
    std::vector<double> vecstart
    {
            0 * 30 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180),
            0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
            0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
            0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
            0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180)
    };

// First principal component (used to drive the hand shape from one sensor)
    std::vector<double> PC1{
            0.03445656, 0.191433, -0.0310968, -0.07660658, -0.11885343,
            0.00317037, 0.22895402, 0.00754185, 0.31359066, 0.21799484,
            -0.00417658, 0.2861582, 0.00334999, 0.35693029, 0.27684938,
            -0.01066, 0.28191222, 0.00776594, 0.35906561, 0.23993875,
            -0.03012474, 0.25982226, 0.00517352, 0.27148087, 0.2919739
    };

    // Hand geometry (phalanxes lenght)
    std::vector<std::vector<cVector3d>> fingertransl =
            {
                    /********************************************************/
                    // Thumb
                    {
                            cVector3d(-0.060f, 0.040f, -0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.04622f),
                            cVector3d(0.03157f, 0.0f, 0.0f),
                    },
                    /********************************************************/
                    // Index
                    {
                            cVector3d(0.0f, 0.025f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.0f, 0.0f, 0.03978f),
                            cVector3d(0.02238f, 0.0f, 0.0f)
                    },
                    /********************************************************/
                    // Middle
                    {
                            cVector3d(0.0f, 0.0f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.0f, 0.0f, 0.04463f),
                            cVector3d(0.02633f, 0.0f, 0.0f)
                    },
                    /********************************************************/
                    // Ring
                    {
                            cVector3d(0.0f, -0.025f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.0f, 0.0f, 0.04137f),
                            cVector3d(0.02565f, 0.0f, 0.0f)
                    },
                    /********************************************************/
                    // Little
                    {
                            cVector3d(0.0f, -0.05f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.00f, 0.0f, 0.0f),
                            cVector3d(0.0f, 0.0f, 0.03274f),
                            cVector3d(0.01811f, 0.0f, 0.0f)
                    }
            };


    // GUI
    std::vector<string> inputname; // used by the GUI to name each input (joint angle)
    static float window_scale = 1.0f; // global window scale
    static bool run_check = false;


    //!Constructor
    cMaestroHand::cMaestroHand() {

        createIndexFinger();
        //createMiddleFinger();
        //createThumb();


        // Initialize esmacat application first
        //application = new my_app();
        //application->set_ethercat_adapter_name_through_terminal();
        //application->start();


        // create cHand
        h_hand = new chai3d::cHand();

        std::vector<std::vector<cTransform>> newT = h_hand->t_default_Tkach25Dof;

        // Tkach model with modified fingers lenghts
        for (int fingerid = 0; fingerid < newT.size(); fingerid++)
        {
            for (int fingerndofcounter = 0; fingerndofcounter < newT[fingerid].size(); fingerndofcounter++)
            {
                //cout << "Before: ";
                //printcTransform(newT[fingerid][fingerndofcounter],"T");

                newT[fingerid][fingerndofcounter].set(
                        fingertransl[fingerid][fingerndofcounter],
                        newT[fingerid][fingerndofcounter].getLocalRot()
                );

                //cout << "After: ";
                //printcTransform(newT[fingerid][fingerndofcounter],"T");
            }
        }


        h_hand->initialize(newT);


        for (int i = 0; i < h_hand->getdof(); i++)
        {
            inputname.push_back("Joint " + std::to_string(i + 1));
        }

        std::cout << "cMaestroHand Constructed" << endl;

    }


    void cMaestroHand::createIndexFinger() {
        h_index = new cMaestroDigit();
    }

    void cMaestroHand::createMiddleFinger() {
        h_middle = new cMaestroDigit();
    }

    void cMaestroHand::createThumb() {
        // h_thumb = new cMaestroThumb(h_hand);
    }

    // this method updates the join angle values
    void cMaestroHand::updateJointAngles(cVector3d& a_thumbPos, cVector3d& a_idxPos, cVector3d& a_midPos ,
                                         cVector3d a_globalPos)
    {


        // Updates array with angles
        //double robot_angles[16];
        //application->updateJointAngles(robot_angles);

        // TODO: SMOOTH SENSOR VALUES
        /*
        for(int i = 0; i < count; i++){
            newAvg = movingAvg(arrNumbers, &sum, pos, len, joint_angles);
            printf("The new average is %d\n", newAvg);
            pos++;
            if (pos >= len){
                pos = 0;
            }
        }
         */

        // Updates thumb
        /*
        double thumb_robot_angles[6];
        thumb_robot_angles[0] = robot_angles[10]; thumb_robot_angles[1] = robot_angles[11]; thumb_robot_angles[2] = robot_angles[12];
        thumb_robot_angles[3] = robot_angles[13]; thumb_robot_angles[4] = robot_angles[14]; thumb_robot_angles[5] = robot_angles[15];
        Eigen::Vector3d thumb_pos = h_thumb->updateJointAngles(thumb_robot_angles);
        a_thumbPos = cVector3d(thumb_pos);
        */

        // Updates index
        /*
        double idx_robot_angles[5];
        idx_robot_angles[0] = robot_angles[0]; idx_robot_angles[1] = robot_angles[1]; idx_robot_angles[2] = robot_angles[2];
        idx_robot_angles[3] = robot_angles[3]; idx_robot_angles[4] = robot_angles[4];
        Eigen::Vector3d idx_pos = h_index->updateJointAngles(idx_robot_angles , a_globalPos.eigen());
        a_idxPos = cVector3d(idx_pos);

        // Updates middle finger
        double mid_robot_angles[5];
        mid_robot_angles[0] = robot_angles[5]; mid_robot_angles[1] = robot_angles[6]; mid_robot_angles[2] = robot_angles[7];
        mid_robot_angles[3] = robot_angles[8]; mid_robot_angles[4] = robot_angles[9];
        Eigen::Vector3d mid_pos = h_index->updateJointAngles(mid_robot_angles , a_globalPos.eigen());
        a_midPos = cVector3d(mid_pos);
         */

        double idx_robot_angles[5];
        idx_robot_angles[0] = 0; idx_robot_angles[1] = 0; idx_robot_angles[2] = 0; idx_robot_angles[3] = 0;
        idx_robot_angles[4] = 0;
        Eigen::Vector3d idx_pos = h_index->updateJointAngles(idx_robot_angles , a_globalPos.eigen());



    }

    bool cMaestroHand::commandJointTorque(double a_stiffness,double a_damping) {

        double index_torque[2];
        if (use_idx)
            h_index->commandJointTorque(a_stiffness,a_damping);

        double middle_torque[2];
        if (use_middle)
            h_middle->commandJointTorque(a_stiffness,a_damping);

        double thumb_torque[4];

        //h_thumb->commandJointTorque(thumb_torque);

        double command_torque[8];
        double desired_angle[8];
        double actual_torque[8];

        command_torque[0] = index_torque[0];
        command_torque[1] = index_torque[1];
        command_torque[2] = middle_torque[0];
        command_torque[3] = middle_torque[1];
        command_torque[4] = thumb_torque[0];
        command_torque[5] = thumb_torque[1];
        command_torque[6] = thumb_torque[2];
        command_torque[7] = thumb_torque[3];

        application->commandJointTorque(command_torque, desired_angle, actual_torque);

    }

void cMaestroHand::updateCHandAngles(void) {


        double thumb_angles;

        // TODO: MODIFY JOINT ANGLES OF RING AND PINKY BASED ON OTHER VALUES
        std::vector<double> vec;


        //if (!use_idx && !use_middle && !use_thumb) {
        //    vec = vecstart;
        //}
        //else {
            vec = std::vector<double>{
                    0 * 30 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                    -0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 100 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 100 * 10 * (Pi / 180),
                    100 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180)
            };
        //}

        // update hand model kinematics
        h_hand->updateAngles(vec);
        h_hand->updateKinematics();

        // TODO: FIX THIS
        auto tip_pos = h_hand->getFingertipCenters();

    }

std::vector<cVector3d*> cMaestroHand::testTrajectory(vector<double> vec) {

    // update hand model kinematics
    h_hand->updateAngles(vec);
    h_hand->updateKinematics();

    // update chand and get fingertip pos
    auto tip_pos = h_hand->getFingertipCenters();

    return tip_pos;
}

    /*
    int movingAvg(int *ptrArrNumbers, double*& ptrSum, int pos, int len, double* nextNum)
    {
        for (int i = 0 ; i < 16 ; i++)
        {
            //Subtract the oldest number from the prev sum, add the new number
            *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
            //Assign the nextNum to the position in the array
            ptrArrNumbers[pos] = nextNum;
            //return the average
            return *ptrSum / len;
        }
    }
     */