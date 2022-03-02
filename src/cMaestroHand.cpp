//
// Created by aldo on 1/24/22.
//

#include "cMaestroHand.h"

namespace chai3d {

// Pi
    double Pi = 3.14159;

// Initial hand pose
    std::vector<double> vecstart{
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

        if (1)
            createIndexFinger();

        if (0)
            createMiddleFinger();

        if (0)
            createThumb();

        /*
        // Initialize esmacat application first
        application = new my_app();
        application->set_ethercat_adapter_name_through_terminal();
        application->start();
         */

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

        double cylr = 0.0065;

    }


    void cMaestroHand::createIndexFinger() {
        h_index = new cMaestroDigit(h_hand);
    }

    void cMaestroHand::createMiddleFinger() {
        h_middle = new cMaestroDigit(h_hand);
    }

    void cMaestroHand::createThumb() {
        // h_thumb = new cMaestroThumb(h_hand);
    }


    double *cMaestroHand::updateJointAngles() {

        // Updates array with angles
        double joint_angles[16];
        application->updateJointAngles(joint_angles);

        // Updates index finger
        double index_angles[5];
        index_angles[0] = joint_angles[0];
        index_angles[1] = joint_angles[1];
        index_angles[2] = joint_angles[2];
        index_angles[3] = joint_angles[3];
        index_angles[4] = joint_angles[4];

        h_index->updateJointAngles(index_angles);

        // Updates middle finger
        double middle_angles[5];
        middle_angles[0] = joint_angles[5];
        middle_angles[1] = joint_angles[6];
        middle_angles[2] = joint_angles[7];
        middle_angles[3] = joint_angles[8];
        middle_angles[4] = joint_angles[9];

        h_middle->updateJointAngles(middle_angles);

        double thumb_angles[6];
        thumb_angles[0] = joint_angles[10];
        thumb_angles[1] = joint_angles[11];
        thumb_angles[2] = joint_angles[12];
        thumb_angles[3] = joint_angles[13];
        thumb_angles[4] = joint_angles[14];
        thumb_angles[5] = joint_angles[15];

        h_thumb->updateJointAngles(thumb_angles);


    }

    bool cMaestroHand::commandJointTorque() {
        double index_torque[2];
        h_index->commandJointTorque(index_torque);

        double middle_torque[2];
        h_middle->commandJointTorque(middle_torque);

        double thumb_torque[4];
        h_thumb->commandJointTorque(thumb_torque);

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

    void cMaestroHand::updateKinematics(void) {

        // This function gets the joint angles for finger
        double* index_angles = h_index->getJointAngles();

        double* middle_angles = h_middle->getJointAngles();

        std::vector<double> vec{
                0 * 30 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180)
        };

        // update hand model kinematics
        h_hand->updateAngles(vec);
        h_hand->updateKinematics();
    }

}