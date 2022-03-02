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

namespace chai3d {

    class cMaestroHand{

    public:

        //--------------------------------------------------------------------------
        // CONSTRUCTOR & DESRUCTOR
        //--------------------------------------------------------------------------

        //! Constructor for hMaestroHand
        cMaestroHand();

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

        // This function creates the palm for the visualizer
        void createPalm(void);

        // This function creates a index finger object
        void createIndexFinger(void);

        // This function creates a middle finger object
        void createMiddleFinger(void);

        // This function creates a thumb object
        void createThumb(void);

        // Updates the joint angle values from the Maestro
        double *updateJointAngles(void);

        // Commands the desired joint torque to the Maestro
        bool commandJointTorque(void);

    public:

        //--------------------------------------------------------------------------
        // PUBLIC METHODS
        //--------------------------------------------------------------------------

        // This function updates the cHand visualizer with new coordinates
        void updateKinematics(void);



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

        // Middle finger chai3d
        cMaestroDigit *h_middle;

        // Thumb chai3d
        cMaestroThumb *h_thumb;

    };

#endif //MAESTRO_CHAI3D_CMAESTROHAND_H
}