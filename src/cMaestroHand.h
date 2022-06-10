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
        cMaestroHand(cWorld * a_world);

        //! Destructor for hMaestroHand
        ~cMaestroHand() {

            delete h_hand;
            /*
            application->stop();
            delete application;
             */
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


        // Commands the desired joint torque to the Maestro
        bool commandJointTorque(void);

        // Compute parameters
        double* M3KL1(const double A1, const double B1, const double C1,
                     const double PHI1, const double PHI3);

        // Smooths the sensor values with a running avg
        double** smoothSensorValues(double* noisy);

    public:

        // This function updates the cHand visualizer with new coordinates
        std::vector<cVector3d*> updateCHandAngles(double* idx_finger = 0, double* middle_finger = 0, double* thumb = 0);

    public:

        //--------------------------------------------------------------------------
        // PUBLIC METHOD
        //--------------------------------------------------------------------------

        void commandFingertipForce(const cVector3d & force_idx = 0, const cVector3d & force_mid = 0,
                                   const cVector3d& force_thumb = 0);

        // Updates the joint angle values from the Maestro
        void updateJointAngles(std::vector<cVector3d*>& a_pos);


    public:

        //--------------------------------------------------------------------------
        // PUBLIC MEMBERS
        //--------------------------------------------------------------------------

        // cHand
        cHand *h_hand;

        // Array of index finger angles
        double index_angles[5];

        // Array of middle finger angles
        double middle_angles[5];

        // Array of thumb angles
        double thumb_angles[6];

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

    private:
        cWorld* m_world;
    };

#endif //MAESTRO_CHAI3D_CMAESTROHAND_H
