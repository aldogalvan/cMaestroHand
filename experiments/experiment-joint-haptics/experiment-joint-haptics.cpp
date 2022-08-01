//
// Created by aldo on 7/27/22.
//

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "cMaestroHand.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------

using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/

enum MouseStates
{
    MOUSE_IDLE,
    MOUSE_MOVE_CAMERA
};

cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small sphere (cursor) representing the haptic device
cMaestroHand* hand;

// hand radius
double radius;

// stiffess
double stiffness = 0;

//damping
double damping = 0;

// epsilon values
double epsilonBaseValue;
double epsilonMinimalValue;
double epsilon;
double epsilonCollisionDetection;
double epsilonInitialValue;

// collision events
int numCollisionEvents = 0;

// algorithm counter
int algoCounter = 0;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// mouse state
MouseStates mouseState = MOUSE_IDLE;

// last mouse position
double mouseX, mouseY;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// the BOX!
cShapeBox* box;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// marker to track point
cShapeSphere* marker;

//------------------------------------------------------------------------------
// EXPERIMENT VARIABLES
//------------------------------------------------------------------------------

//a label to display trial number
cLabel* labelTrial;

// subject number
int  subject_number;

// trial number
int trial_number = 0;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback to handle mouse click
void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods);

// callback to handle mouse motion
void mouseMotionCallback(GLFWwindow* a_window, double a_posX, double a_posY);

// callback to handle mouse scroll
void mouseScrollCallback(GLFWwindow* a_window, double a_offsetX, double a_offsetY);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// this function computes a penalty force
cVector3d computePenaltyForce(const cVector3d a_goal,cVector3d& a_proxy);

// this function computes a proxy force
cVector3d computeProxyForce(const cVector3d a_goal, cVector3d& a_proxy);

// simulation loop
void loop(void);

// this function computes the proxy force
cVector3d computeProxyForce(const cVector3d a_goal, cVector3d& a_proxy);

//==============================================================================
/*
    DEMO:   01-mydevice.cpp

    This application illustrates how to program forces, torques and gripper
    forces to your haptic device.

    In this example the application opens an OpenGL window and displays a
    3D cursor for the device connected to your computer. If the user presses
    onto the user button (if available on your haptic device), the color of
    the cursor changes from blue to green.

    In the main haptics loop function  "updateHaptics()" , the position,
    orientation and user switch status are read at each haptic cycle.
    Force and torque vectors are computed and sent back to the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "Experiment Joint Level Haptics" << endl;
    cout << "-----------------------------------" << endl;
    cout << "Please input the subject number:" << endl;
    cin >> subject_number;
    cout << endl << endl;
    cout << "[1] - Move To Next Trial" << endl;
    cout << "[2] - Go To Last Trial" << endl;
    cout << "[q] - Exit application" << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }


    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set mouse position callback
    glfwSetCursorPosCallback(window, mouseMotionCallback);

    // set mouse button callback
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    // set mouse scroll callback
    glfwSetScrollCallback(window, mouseScrollCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif

    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (0.25, 0.25, 0.20),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(1.0, 0.0, 0.0);

    // create a sphere (cursor) to represent the haptic device
    hand = new cMaestroHand();

    // gets the fingertip radius
    radius = hand->h_hand->radius();

    // insert cursor inside world
    world->addChild(hand->h_hand);

    // create the example box
    box = new cShapeBox(.05,.05,.05);
    box->setLocalPos(.075,0.03,-.045);
    box->m_material->setGreenLight();
    world->addChild(box);

    //! delete later
    marker = new cShapeSphere(radius+0.0001);
    world->addChild(marker);
    marker->m_material->setRed();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);

    labelTrial = new cLabel(font);
    camera->m_frontLayer->addChild(labelTrial);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        switch (trial_number)
        {
            case 1:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 2:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 3:
                stiffness = 10; damping = 10 ;
                loop();
                break;
            case 4:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 5:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 6:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 7:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 8:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 9:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 10:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 11:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 12:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 13:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 14:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 15:
                stiffness = 10; damping = 10;
                loop();
                break;
            case 16:
                goto exit_loop;
            default:
                stiffness = 0; damping = 0;
                loop();

        }
    }
    exit_loop:;

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{

    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

        // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_1)
    {
        trial_number -= 1;
    }

        // option - toggle fullscreen
    else if (a_key == GLFW_KEY_2)
    {
        trial_number += 1;
    }

}

//------------------------------------------------------------------------------

void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods)
{
    if (a_button == GLFW_MOUSE_BUTTON_RIGHT && a_action == GLFW_PRESS)
    {
        // store mouse position
        glfwGetCursorPos(window, &mouseX, &mouseY);

        // update mouse state
        mouseState = MOUSE_MOVE_CAMERA;
    }

    else
    {
        // update mouse state
        mouseState = MOUSE_IDLE;
    }
}

//------------------------------------------------------------------------------

void mouseMotionCallback(GLFWwindow* a_window, double a_posX, double a_posY)
{
    if (mouseState == MOUSE_MOVE_CAMERA)
    {
        // compute mouse motion
        int dx = a_posX - mouseX;
        int dy = a_posY - mouseY;
        mouseX = a_posX;
        mouseY = a_posY;

        // compute new camera angles
        double azimuthDeg = camera->getSphericalAzimuthDeg() - 0.5 * dx;
        double polarDeg = camera->getSphericalPolarDeg() - 0.5 * dy;

        // assign new angles
        camera->setSphericalAzimuthDeg(azimuthDeg);
        camera->setSphericalPolarDeg(polarDeg);
    }
}

//------------------------------------------------------------------------------

void mouseScrollCallback(GLFWwindow* a_window, double a_offsetX, double a_offsetY)
{
    double r = camera->getSphericalRadius();
    r = cClamp(r + 0.1 * a_offsetY, 0.5, 3.0);
    camera->setSphericalRadius(r);
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // delete resources
    delete hand;
    delete hapticsThread;
    delete world;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    // update haptic and graphic rate data
    labelTrial->setText("Trial: " + cStr(trial_number));

    // update position of label
    labelTrial->setLocalPos((int)(0.5 * (width - labelTrial->getWidth())), 1000);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    hand->updateVisualizer();

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // fingertip positions and proxies
    cVector3d thumb_pos;
    cVector3d idx_pos;
    cVector3d mid_pos;
    cVector3d thumb_proxy;
    cVector3d idx_proxy;
    cVector3d mid_proxy;

    // global position of the hand
    cVector3d global_pos;

    //initialize position
    global_pos = hand->h_hand->getGlobalPos();

    hand->updateJointAngles(thumb_pos,idx_pos,mid_pos,global_pos.eigen(),Eigen::Vector3d(0,0,0));

    idx_proxy = idx_pos;

    // main haptic simulation loop
    while(simulationRunning)
    {

        /////////////////////////////////////////////////////////////////////
        // UPDATE THE JOINT ANGLES AND RETURN NEW POSITION
        /////////////////////////////////////////////////////////////////////

        hand->updateJointAngles(thumb_pos,idx_pos,mid_pos,global_pos.eigen(),Eigen::Vector3d(0,0,0));

        marker->setLocalPos(idx_pos);

        /////////////////////////////////////////////////////////////////////
        // COMPUTE ANY COLLISIONS
        /////////////////////////////////////////////////////////////////////

        cVector3d force = computePenaltyForce(idx_pos,idx_proxy);

        // signal frequency counter
        freqCounterHaptics.signal(1);


    }

    // exit haptics thread
    simulationFinished = true;
}

cVector3d computePenaltyForce(const cVector3d a_goal, cVector3d& a_proxy)
{
    // declare collision settings and recorder
    cCollisionSettings collisionSettings;
    cCollisionRecorder collisionRecorder;
    collisionSettings.m_collisionRadius = radius;

    cVector3d force(0,0,0);

    // compute collision
    if (box->computeCollisionDetection(a_goal,a_goal,collisionRecorder,collisionSettings))
    {

        // compute force based on penetration depth
        cVector3d normal = collisionRecorder.m_nearestCollision.m_localNormal;
        double depth = collisionRecorder.m_nearestCollision.m_squareDistance;
        cout << 1 << endl;
        // return the force
        force =  normal * sqrt(depth);
    }
    else
    {
        a_proxy = a_goal;
    }
    return force;
}

cVector3d computeProxyForce(const cVector3d a_goal, cVector3d& a_proxy)
{
    // declare collision settings and recorder
    cCollisionSettings collisionSettings;
    cCollisionRecorder collisionRecorder;
    collisionSettings.m_collisionRadius = radius;

    cVector3d force(0,0,0);

    // compute collision
    if (box->computeCollisionDetection(a_proxy,a_goal,collisionRecorder,collisionSettings))
    {

    }
    else
    {
        a_proxy = a_goal;
    }
}

void loop(void)
{
    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // render graphics
    updateGraphics();

    // swap buffers
    glfwSwapBuffers(window);

    // process events
    glfwPollEvents();

    // signal frequency counter
    freqCounterGraphics.signal(1);
}

void thankYou(void)
{

}