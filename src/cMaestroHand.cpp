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
    cMaestroHand::cMaestroHand(cWorld *a_world) {

        m_world = a_world;
        createIndexFinger();
        //createMiddleFinger();
        //createThumb();

        /*
        // Initialize esmacat application first
        application = new my_app();
        application->set_ethercat_adapter_name_through_terminal();
        application->start();
         */

        // create cHand
        h_hand = new chai3d::cHand(a_world);

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


    void cMaestroHand::updateJointAngles(std::vector<cVector3d*>& a_pos) {

        // Updates array with angles
        double joint_angles[16];
        application->updateJointAngles(joint_angles);



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

        // Updates index finger
        auto index_angles = M3KL1(joint_angles[0],joint_angles[1],joint_angles[2],
                                  joint_angles[3],joint_angles[4]);

        h_index->updateJointAngles(index_angles);

        // Updates middle finger
        auto middle_angles = M3KL1(joint_angles[5],joint_angles[6],joint_angles[7],
                                  joint_angles[8],joint_angles[9]);

        h_middle->updateJointAngles(middle_angles);

        double thumb_angles[6];
        thumb_angles[0] = joint_angles[10];
        thumb_angles[1] = joint_angles[11];
        thumb_angles[2] = joint_angles[12];
        thumb_angles[3] = joint_angles[13];
        thumb_angles[4] = joint_angles[14];
        thumb_angles[5] = joint_angles[15];

        //h_thumb->updateJointAngles(thumb_angles);

        a_pos = updateCHandAngles(index_angles,middle_angles,thumb_angles);
    }

    bool cMaestroHand::commandJointTorque(void) {


        double index_torque[2];
        if (use_idx)
            h_index->commandJointTorque(index_torque);

        double middle_torque[2];
        if (use_middle)
            h_middle->commandJointTorque(middle_torque);

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

std::vector<cVector3d*> cMaestroHand::updateCHandAngles(double* idx_finger, double* middle_finger, double* thumb) {

        // TODO: MODIFY JOINT ANGLES OF RING AND PINKY BASED ON OTHER VALUES
        std::vector<double> vec;


        //if (!use_idx && !use_middle && !use_thumb) {
        //    vec = vecstart;
        //}
        //else {
            vec = std::vector<double>{
                    0 * 30 * (Pi / 180), 2 * 10 * (Pi / 180), -2 * 10 * (Pi / 180), -0 * 10 * (Pi / 180),
                    -0 * 10 * (Pi / 180),
                    -2 * 10 * (Pi / 180), 1 * 10 * (Pi / 180), 0.5 * 10 * (Pi / 180), .5 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180),
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

        return tip_pos;
    }

    void commandFingertipForce(const cVector3d & force_idx , const cVector3d & force_mid,
                               const cVector3d& force_thumb)
    {

    }
    double* cMaestroHand::M3KL1(const double A1, const double B1, const double C1, const double PHI1, const double PHI3)
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