#include "chai3d.h"
#include "cHand.h"
#include "cMaestroDigit.h"
#include "cMaestroThumb.h"
#include "esmacat_application.h"
#include "my_app.h"

//
// Created by aldo on 1/24/22.
//

#ifndef MAESTRO_CHAI3D_CMAESTROHAND_H
#define MAESTRO_CHAI3D_CMAESTROHAND_H

using namespace chai3d;
using namespace Eigen;

    class cMaestroHand{

    public:

        //--------------------------------------------------------------------------
        // CONSTRUCTOR & DESRUCTOR
        //--------------------------------------------------------------------------

        //! Constructor for hMaestroHand
        cMaestroHand();

        //! Destructor for hMaestroHand
        ~cMaestroHand()
        {
            // Application closing
            std::cout << "Application closing!" << endl;
            std::cout.flush();

            //application->stop();
            // delete application;
        };


    private:

        //--------------------------------------------------------------------------
        // PRIVATE METHODS
        //--------------------------------------------------------------------------

        // This function creates a index finger object
        void createIndexFinger(void);

        // This function creates a middle finger object
        void createMiddleFinger(void);

        // This function creates a thumb object
        void createThumb(void);



    public:

        // This function updates the cHand visualizer with new coordinates
        void updateVisualizer(void);

        // Updates the joint angle values from the Maestro
        void updateJointAngles(cVector3d& a_thumbPos, cVector3d& a_idxPos, cVector3d& a_midPos,
                               const Vector3d a_globalPos , const Vector3d a_globalRot);

        // Commands the desired joint torque to the Maestro using the proxy algorithm
        // the stiffnesses are angular stiffness to be used for joint control
        bool torqueControlProxy(double a_stiffness, double a_damping);

        // Commands the desired joint torque using an inverse kinematics algorithm
        bool torqueControlInverseDynamics( Vector3d a_thumbForce,  Vector3d a_idxForce,
                                             Vector3d a_midForce);

        // commands the desired joint position
        bool positionalControl();

        // this is a test trajectory
        std::vector<cVector3d*> testTrajectory(vector<double> vec);

    public:

        //--------------------------------------------------------------------------
        // PUBLIC MEMBERS
        //--------------------------------------------------------------------------

        // cHand
        cHand *h_hand;


    private:

        //--------------------------------------------------------------------------
        // PRIVATE MEMBERS
        //--------------------------------------------------------------------------

        // Esmacat application
        my_app *application;

        // Index finger chai3d
        cMaestroDigit *h_index;
        bool use_idx = 1;

        // Middle finger chai3d
        cMaestroDigit *h_middle;
        bool use_middle = 0;

        // Thumb chai3d
        cMaestroThumb *h_thumb;
        bool use_thumb = 0;

    private:

        // trhe vector of joint angles for the whole hand
        vector<double> h_angles;
    };

#endif //MAESTRO_CHAI3D_CMAESTROHAND_H
