//
// Created by aldo on 1/24/22.
//

#include "cMaestroHand.h"

using namespace chai3d;

// Pi
double Pi = 3.14159;


// Hand geometry (phalanxes lenght)
vector<vector<cVector3d>> fingertransl =
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
vector<string> inputname; // used by the GUI to name each input (joint angle)
static float window_scale = 1.0f; // global window scale
static bool run_check = false;


//!Constructor
cMaestroHand::cMaestroHand(bool a_useThumb, bool a_useIdx, bool a_useMiddle) {

    // create thumb if true
    use_thumb = a_useThumb;
    if (use_thumb)
        createThumb();
    //create index if true
    use_idx = a_useIdx;
    if (use_idx)
        createIndexFinger();
    //create middle if true
    use_middle = a_useMiddle;
    if (use_middle)
        createMiddleFinger();



    // Initialize esmacat application first
    //application = new my_app();
    //application->set_ethercat_adapter_name_through_terminal();
    //application->start();


    // create cHand
    h_hand = new chai3d::cHand();

    vector<vector<cTransform>> newT = h_hand->t_default_Tkach25Dof;

    // Tkach model with modified fingers lenghts
    for (int fingerid = 0; fingerid < newT.size(); fingerid++)
    {
        for (int fingerndofcounter = 0; fingerndofcounter < newT[fingerid].size(); fingerndofcounter++)
        {
            newT[fingerid][fingerndofcounter].set(
                    fingertransl[fingerid][fingerndofcounter],
                    newT[fingerid][fingerndofcounter].getLocalRot()
            );
        }
    }

    h_hand->initialize(newT);

    for (int i = 0; i < h_hand->getdof(); i++)
    {
        inputname.push_back("Joint " + to_string(i + 1));
    }

    cout << "cMaestroHand Constructed" << endl;

}

void cMaestroHand::createIndexFinger()
{
    h_index = new cMaestroDigit();
}

void cMaestroHand::createMiddleFinger()
{
    h_middle = new cMaestroDigit();
}

void cMaestroHand::createThumb() {

    h_thumb = new cMaestroThumb();
}

// this method updates the join angle values
void cMaestroHand::updateJointAngles(cVector3d& a_thumbPos, cVector3d& a_idxPos, cVector3d& a_midPos ,
                                     const Vector3d a_globalPos , const Vector3d a_globalRot)
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
    a_idxPos = cVector3d(h_index->updateJointAngles(idx_robot_angles,
                                                         a_globalPos,
                                                         a_globalRot));

    double thumb_robot_angles[5];
    thumb_robot_angles[0] = 0; thumb_robot_angles[1] = 0; thumb_robot_angles[2] = 0; thumb_robot_angles[3] = 0;
    thumb_robot_angles[4] = 0;
    a_thumbPos = cVector3d(h_thumb->updateJointAngles(thumb_robot_angles,
                                                    a_globalPos,
                                                    a_globalRot));
}

bool cMaestroHand::torqueControlProxy(double a_stiffness,double a_damping) {

    double index_torque[2];
    if (use_idx)
        h_index->commandJointTorque(a_stiffness,a_damping);

    double middle_torque[2];
    if (use_middle)
        h_middle->commandJointTorque(a_stiffness,a_damping);

    double thumb_torque[4];
    if (use_thumb)
        h_thumb->commandJointTorque(a_stiffness,a_damping);

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

void cMaestroHand::computeHandProxy( Vector3d& a_goalThumb, Vector3d& a_goalIdx, Vector3d& a_goalMid,
                                    bool thumbCollision, bool idxCollision, bool midCollision)
{
    if(use_idx)
        a_goalIdx = h_index->computeHandProxy(a_goalIdx,idxCollision);

    if(use_middle)
        a_goalMid = h_middle->computeHandProxy(a_goalMid,midCollision);

    if(use_thumb)
        a_goalThumb = h_index->computeHandProxy(a_goalThumb,thumbCollision);
}

bool cMaestroHand::torqueControlInverseDynamics( const Vector3d a_thumbForce,  const Vector3d a_idxForce,
                                                const Vector3d a_midForce)
{
    double* idx_torque;
    if (use_idx)
        idx_torque = h_index->computeInverseDynamics(a_idxForce);

    double* mid_torque;
    if(use_middle)
        mid_torque = h_middle->computeInverseDynamics(a_midForce);

    double* thumb_torque;
    if (use_thumb)
        thumb_torque = h_thumb->computeInverseDynamics(a_thumbForce);

}

void cMaestroHand::updateVisualizer(void) {

        // TODO: MODIFY JOINT ANGLES OF RING AND PINKY BASED ON OTHER VALUES
        vector<double> vec;

        vec = vector<double>{
                // mcp = 2 pip = 4 dip = 5 (idx and mid)
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180),
                0 * 10 * (Pi / 180)
        };

        if (use_idx)
        {
            auto idx_angles = h_index->getJointAngles();
            vec[5] = idx_angles[0];
            vec[6] = idx_angles[1];
            vec[8] = idx_angles[2];
            vec[9] = idx_angles[3];
        }
        if (use_middle)
        {
            auto mid_angles = h_middle->getJointAngles();
            vec[10] = mid_angles[0];
            vec[11] = mid_angles[1];
            vec[13] = mid_angles[2];
            vec[14] = mid_angles[3];
        }
        if (use_thumb)
        {
            auto thumb_angles = h_thumb->getJointAngles();
            vec[0] = thumb_angles[0];
            vec[1] = thumb_angles[1];
            vec[3] = thumb_angles[2];
            vec[4] = thumb_angles[3];
        }

        // update hand model kinematics
        h_hand->updateAngles(vec);
        h_hand->updateKinematics();

    }

vector<cVector3d*> cMaestroHand::testTrajectory(vector<double> vec) {

    // update hand model kinematics
    h_hand->updateAngles(vec);
    h_hand->updateKinematics();

    // update chand and get fingertip pos
    auto tip_pos = h_hand->getFingertipCenters();

    return tip_pos;
}
