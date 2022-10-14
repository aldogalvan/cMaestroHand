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

// angular stiffness
double ang_stiffness = 1;

// angular damping
double ang_damping = 1;

// object stiffness
double stiffness = 1;

// object damping
double damping = 1;

// object static friction constant
double us = 0.6;

// object dynamic friction constant
double uk = 0.6;

// slipping or not
bool slipping;

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

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

cShapeBox* box;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// this function computes the force with a simple penalty algorithm
cVector3d computePenaltyAlgorithm(const cVector3d& goal);

// this function finds the proxy positions
void computeNextBestProxyPosition(cVector3d& proxy , const cVector3d& goal);

// this function checks for collisions with moving objects
void testFrictionAndMoveProxy(cVector3d& proxy);

// this functions computes the first round of constraints
bool computeProxyWithConstraints0(cVector3d& proxy , const cVector3d& goal);

// this function computes a second round of proxy positions
// Very unecessary to compute two
bool computeProxyWithConstraints1(cVector3d& proxy , const cVector3d& goal);

// this function performs proxy optimization
void jointOptimization(Eigen::VectorXd & joint_angles, Eigen::Vector3d curPos, const Eigen::Vector3d desPos ,
                       const int max_iterations , const double er);


// sets the error thresholds
void setEpsilonBaseValue(double a_value);

// goal achieved critiera
bool goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal);



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
    cout << "CHAI3D" << endl;
    cout << "Demo: 01-MaestroHand" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


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
    light->setDir(-1.0, 0.0, 0.0);

    // create a sphere (cursor) to represent the haptic device
    hand = new cMaestroHand(true,true,false);

    // gets the fingertip radius
    radius = hand->h_hand->radius();

    // sets the epsilon values
    setEpsilonBaseValue(0.001);

    // insert cursor inside world
    world->addChild(hand->h_hand);

    // create the example box
    box = new cShapeBox(.05,.05,.05);
    box->setLocalPos(.075,0.03,-.045);
    box->m_material->setGreenLight();
    world->addChild(box);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


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
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

        // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

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

    // global rotation of the hand
    cVector3d global_rot(0,0,0);

    // initialize position
    global_pos = hand->h_hand->getGlobalPos();

    // initialize rot
    // global_rot = hand->h_hand->getLocalRot();

    // initialize the values
    //hand->updateJointAngles(thumb_pos,idx_pos,mid_pos , global_pos);

    double t = 0;
    double Pi = 3.14159;

    std::vector<double> vec = std::vector<double>
            {
                    0 * 30 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), -0 * 10 * (Pi / 180),
                    -0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), t * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), t * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), t * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), 0 * 10 * (Pi / 180), t * 10 * (Pi / 180),
                    0 * 10 * (Pi / 180)
            };

    // transformation from hand to mcp (always angle zero)
    cTransform t0 = ( cVector3d(0,0.025,0),cMatrix3d(cVector3d(1, 0, 0), 0));

    // transformation for abad (always zero too)
    cTransform t1 = ( cVector3d(0,0.0,0),cMatrix3d(cVector3d(0, 0, 1), 0));

    // transformation for fe
    cTransform t2 = ( cVector3d(0,0.0,0.03978),cMatrix3d(cVector3d(0, 1, 0), 0));

    // transformation for PIP fe
    cTransform t3 = ( cVector3d(0.02238,0.0,0),cMatrix3d(cVector3d(0, 1, 0), 0));

    // transformation for DIP fe
    cTransform t4 = ( cVector3d(0.01566,0.0,0),cMatrix3d(cVector3d(0, 0, 1), 0));

    // the fina
    cTransform tf = t0*t1*t2*t3*t4;


    //std::cout << *hand->h_hand->getFingertipCenters()[1] << std::endl << std::endl;

    hand->updateJointAngles(thumb_pos,idx_pos,mid_pos,global_pos.eigen(),global_rot.eigen());

    // main haptic simulation loop
    while(simulationRunning)
    {

        /////////////////////////////////////////////////////////////////////
        // UPDATE THE JOINT ANGLES AND RETURN NEW POSITION
        /////////////////////////////////////////////////////////////////////
        hand->updateJointAngles(thumb_pos,idx_pos,mid_pos,global_pos.eigen(),Eigen::Vector3d(0,0,0));

        /////////////////////////////////////////////////////////////////////
        // COMPUTE THE NEXT PROXY POSITION
        /////////////////////////////////////////////////////////////////////

        // compute proxy for index finger
        computeNextBestProxyPosition(idx_proxy , idx_pos);

        //std::cout << t4.getLocalPos() << std::endl;

        auto pos_vector = hand->h_hand->getFingertipCenters();
        cVector3d pos = *pos_vector[1];
        //std::cout << pos << std::endl;

        /////////////////////////////////////////////////////////////////////
        // COMMAND A NEW FORCE USING SOME ALGORITHM
        /////////////////////////////////////////////////////////////////////

        // command force using inverse kinematics

        // signal frequency counter
        freqCounterHaptics.signal(1);


    }

    // exit haptics thread
    simulationFinished = true;
}

//! penalty algorithm
cVector3d computePenaltyAlgorithm(const cVector3d& goal)
{
    // declare collision settings and recorder
    cCollisionSettings collisionSettings;
    cCollisionRecorder collisionRecorder;
    collisionSettings.m_collisionRadius = radius;

    // compute collision
    box->computeCollisionDetection(goal,goal,collisionRecorder,collisionSettings);

    // compute force based on penetration depth
    cVector3d normal = collisionRecorder.m_nearestCollision.m_localNormal;
    double depth = collisionRecorder.m_nearestCollision.m_squareDistance;

    // return the force
    return normal*sqrt(depth);
}


//! proxy algorithm
void computeNextBestProxyPosition(cVector3d& proxy , const cVector3d& goal)
{
    bool hit0, hit1;

    // search for a first contact
    hit0 = computeProxyWithConstraints0(proxy , goal);
    if (!hit0)
    {
        return;
    }
    cout << "collision" <<endl;
    // NOT REALLY NECESSARY IF JUST ONE BLOCK
    /*
    // search for a second contact
    hit1 = computeProxyConstraints1(proxy);
    m_proxyGlobalPos = m_nextBestProxyGlobalPos;
    if (!hit1)
    {
        m_contactPointLocalPos0 = cTranspose(m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalRot()) * (m_proxyGlobalPos - m_collisionRecorderConstraint0.m_nearestCollision.m_object->getGlobalPos());
        return;
    }
     */

}


bool computeProxyWithConstraints0(cVector3d& proxy , const cVector3d& goal)
{

    // declare collision settings and recorder
    cCollisionSettings collisionSettings;
    cCollisionRecorder collisionRecorder;

    // we define the goal position of the proxy.
    cVector3d goalGlobalPos = goal;

    // to address numerical errors of the computer, we make sure to keep the proxy
    // slightly above any triangle and not directly on it. If we are using a radius of
    // zero, we need to define a default small value for epsilon
    epsilonInitialValue = fabs(0.0001 * radius);
    if (epsilonInitialValue < epsilonBaseValue)
    {
        epsilonInitialValue = epsilonBaseValue;
    }

    // the epsilon value is dynamic (can be reduced); we set it to its initial
    // value if the proxy is not touching any triangle.
    if (numCollisionEvents == 0)
    {
        epsilon = epsilonInitialValue;
        slipping = true;
    }

    cVector3d nextBestProxyGlobalPos;

    // if the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(proxy, goalGlobalPos))
    {
        nextBestProxyGlobalPos = proxy;
        algoCounter = 0;
        return (false);
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position

    // compute the distance between the proxy and the goal positions
    double distanceProxyGoal = cDistance(proxy, goalGlobalPos);

    // a vector from the proxy to the goal
    cVector3d vProxyToGoal;
    cVector3d vProxyToGoalNormalized;

    if (distanceProxyGoal > epsilon)
    {
        goalGlobalPos.subr(proxy, vProxyToGoal);
        vProxyToGoal.normalizer(vProxyToGoalNormalized);
    }
    else
    {
        vProxyToGoal.zero();
        vProxyToGoalNormalized.zero();
    }

    // test whether the path from the proxy to the goal is obstructed;
    // for this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos =  goalGlobalPos + cMul(epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    collisionSettings.m_collisionRadius = radius;

    // search for a collision between the first segment (proxy-device)
    // and the environment.
    collisionRecorder.clear();
    bool hit = box->computeCollisionDetection(proxy,
                                              targetPos,
                                              collisionRecorder,
                                              collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(collisionRecorder.m_nearestCollision.m_squareDistance);

        if (collisionDistance > (distanceProxyGoal + C_SMALL))
        {
            hit = false;
        }


        if (hit)
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < epsilon)
            {
                epsilon = collisionDistance;
                if (epsilon < epsilonMinimalValue)
                {
                    epsilon = epsilonMinimalValue;
                }
            }
        }
    }

    // if no collision occurs, then we move the proxy to its goal, and we're done
    if (!hit)
    {
        numCollisionEvents = 0;
        algoCounter = 0;
        slipping = true;
        nextBestProxyGlobalPos = goalGlobalPos;
        return (false);
    }

    // a first collision has occurred
    algoCounter = 1;


    //--------------------------------------------------------------------------
    // FIRST COLLISION OCCURES:
    //--------------------------------------------------------------------------

    // we want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // so we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(collisionRecorder.m_nearestCollision.m_globalNormal);

    // now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // if only ASCII art were a little more expressive...
    double distanceTriangleProxy = epsilon / fabs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, epsilon); }

    // we compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // a vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    proxy.subr(collisionRecorder.m_collisions[0].m_globalPos, vCollisionToProxy);

    // move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  we still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    collisionRecorder.m_collisions[0].m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    nextBestProxyGlobalPos = nextProxyPos;

    // if the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        numCollisionEvents = 1;
        algoCounter = 0;
        return (true);
        proxy = nextBestProxyGlobalPos;
    }

    proxy = nextBestProxyGlobalPos;
    return (true);
}

// ! UNNECESSARY BUT CAN ADD IF NEEDED
bool computeProxyWithConstraints1(cVector3d& proxy ,  cVector3d& goal)
{

}

/*
void testFrictionAndMoveProxy(const cVector3d& a_goal,
                              const cVector3d& a_proxy,
                              cVector3d& a_normal,
                              cGenericObject* a_parent)
{
    // check if friction is enabled
    if (!a_parent->m_material->getUseHapticFriction())
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    // compute penetration depth; how far is the device "behind" the
    // plane of the obstructing surface
    cVector3d projectedGoal = cProjectPointOnPlane(m_deviceGlobalPos, a_proxy, a_normal);
    double penetrationDepth = cSub(m_deviceGlobalPos,projectedGoal).length();

    // find the appropriate friction coefficient
    double mud = a_parent->m_material->getDynamicFriction();
    double mus = a_parent->m_material->getStaticFriction();

    // no friction; don't try to compute friction cones
    if ((mud == 0) && (mus == 0))
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    // the corresponding friction cone radii
    double atmd = atan(mud);
    double atms = atan(mus);

    // compute a vector from the device to the proxy, for computing
    // the angle of the friction cone
    cVector3d vDeviceProxy = cSub(a_proxy, m_deviceGlobalPos);
    vDeviceProxy.normalize();

    // now compute the angle of the friction cone...
    double theta = acos(vDeviceProxy.dot(a_normal));

    // manage the "slip-friction" state machine

    // if the dynamic friction radius is for some reason larger than the
    // static friction radius, always slip
    if (mud > mus)
    {
        m_slipping = true;
    }

        // if we're slipping...
    else if (m_slipping)
    {
        if (theta < (atmd * m_frictionDynHysteresisMultiplier))
        {
            m_slipping = false;
        }
        else
        {
            m_slipping = true;
        }
    }

        // if we're not slipping...
    else
    {
        if (theta > atms)
        {
            m_slipping = true;
        }
        else
        {
            m_slipping = false;
        }
    }

    // the friction coefficient we're going to use...
    double mu;
    if (m_slipping)
    {
        mu = mud;
    }
    else
    {
        mu = mus;
    }

    // calculate the friction radius as the absolute value of the penetration
    // depth times the coefficient of friction
    double frictionRadius = fabs(penetrationDepth * mu);

    // calculate the distance between the proxy position and the current
    // goal position.
    double r = a_proxy.distance(a_goal);

    // if this distance is smaller than C_SMALL, we consider the proxy
    // to be at the same position as the goal, and we're done...
    if (r < C_SMALL)
    {
        m_nextBestProxyGlobalPos = a_proxy;
    }

        // if the proxy is outside the friction cone, update its position to
        // be on the perimeter of the friction cone...
    else if (r > frictionRadius)
    {
        m_nextBestProxyGlobalPos = cAdd(a_goal, cMul(frictionRadius/r, cSub(a_proxy, a_goal)));
    }

        // otherwise, if the proxy is inside the friction cone, the proxy
        // should not be moved (set next best position to current position)
    else
    {
        m_nextBestProxyGlobalPos = a_proxy;
    }

    // we're done; record the fact that we're still touching an object...
    return;
}
*/

void setEpsilonBaseValue(double a_value)
{
    epsilonBaseValue = a_value;
    epsilonMinimalValue = 0.01 * epsilonBaseValue;
    epsilon = epsilonBaseValue;
    epsilonCollisionDetection = 1.0 * epsilon;
}

bool goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal)
{
    return (a_proxy.distance(a_goal) < (epsilonBaseValue));
}