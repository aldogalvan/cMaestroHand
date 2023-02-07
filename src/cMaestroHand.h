#include "chai3d.h"
#include "cHand.h"
#include "cMaestroDigit.h"
#include "cMaestroThumb.h"
#include "esmacat_application.h"
#include "my_app.h"

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
        cMaestroHand(bool a_useThumb , bool a_useIdx , bool a_useMiddle);

        //! Destructor for hMaestroHand
        ~cMaestroHand()
        {
            // Application closing
            std::cout << "Application closing!" << endl;
            std::cout.flush();

            application->stop();
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

        // this function performs calibaration
        // TODO: Perform calibration
        void calibrate(void);



    public:

        // This function updates the cHand visualizer with new coordinates
        void updateVisualizer(void);

        // performs the calibration
        //! THE USER SHOULD HAVE THEIR HAND IN THE ZERO CONFIGURATION
        void calibration(void);

        // Updates the joint angle values from the Maestro
        void updateJointAngles(cVector3d& a_thumbPos, cVector3d& a_idxPos, cVector3d& a_midPos,
                               const Vector3d a_globalPos , const Vector3d a_globalRot);

        // Commands the desired joint torque to the Maestro using the proxy algorithm
        // the stiffnesses are angular stiffness to be used for joint control
        bool torqueControlProxy(double a_stiffness, double a_damping);

        // commands the desired joint position
        bool positionalControl();

        // compute proxy hand algorithm
        void computeHandProxy( Vector3d& a_goalThumb,  Vector3d& a_goalIdx,  Vector3d& a_goalMid,
                              bool thumbCollision = 0, bool idxCollision = 0, bool midCollision = 0);

        // render the ghost hand
        void renderGhostHand(void);



    public:

        //--------------------------------------------------------------------------
        // PUBLIC MEMBERS
        //--------------------------------------------------------------------------

        // cHand
        cHand *h_hand;

        // ghost hand
        cHand* h_ghost_hand;


    public:

        //--------------------------------------------------------------------------
        // PRIVATE MEMBERS
        //--------------------------------------------------------------------------

        // Esmacat application
        my_app *application;

        // Index finger chai3d
        cMaestroDigit *h_index;
        double* idx_angle_offset;
        bool use_idx = 1;

        // Middle finger chai3d
        cMaestroDigit *h_middle;
        double* middle_angle_offset;
        bool use_middle = 0;

        // Thumb chai3d
        cMaestroThumb *h_thumb;
        double* thumb_angle_offset;
        bool use_thumb = 0;

    private:

        // trhe vector of joint angles for the whole hand
        vector<double> h_angles;
    };

#endif //MAESTRO_CHAI3D_CMAESTROHAND_H
