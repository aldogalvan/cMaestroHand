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


        // Smooths the sensor values with a running avg
        double** smoothSensorValues(double* noisy);


    public:

        // This function updates the cHand visualizer with new coordinates
        void updateCHandAngles(void);

        // Updates the joint angle values from the Maestro
        void updateJointAngles(cVector3d& a_thumbPos, cVector3d& a_idxPos, cVector3d& a_midPos,
                               cVector3d a_globalPos);

        // Commands the desired joint torque to the Maestro
        bool commandJointTorque(double a_stiffness, double a_damping);

        // this is a test trajectory
        std::vector<cVector3d*> testTrajectory(vector<double> vec);

    public:

        //--------------------------------------------------------------------------
        // PUBLIC METHOD
        //--------------------------------------------------------------------------


        // computes a collision between each digit and a generic object
        bool computeCollisions(cGenericObject* a_object);

        // calculates the new proxy location
        void calculateProxy(double radius , cVector3d proxyPos , cVector3d curPos);


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
        bool use_idx = 0;

        // Middle finger chai3d
        cMaestroDigit *h_middle;
        bool use_middle = 0;

        // Thumb chai3d
        cMaestroThumb *h_thumb;
        bool use_thumb;

        // Running Average
        double run_avg[16];
        int num_sensors = 16;
        int count = 10;

        //

    private:

        // trhe vector of joint angles for the whole hand
        vector<double> h_angles;
    };

#endif //MAESTRO_CHAI3D_CMAESTROHAND_H
