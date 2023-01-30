//------------------------------------------------------------------------------
#include "chai3d.h"
#include "cMaestroHand.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1925 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

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

// vector with haptic points
// thumb = 0, idx = 1, mid = 2
vector<cToolCursor*> haptic_points(3);

// a few mesh objects
cMesh* object0;
cMesh* object1;
cMesh* object2;
cMesh* object3;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a label to explain what is happening
cLabel* labelMessage;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// tool radius
double radius;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;


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


//==============================================================================
/*
    DEMO:    16-friction.cpp

    This example illustrates how friction is implemented with the
    finger-proxy force algorithm on meshes.
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
    cout << "Demo: 16-friction" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
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
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(2.0, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set orthographic camera mode
    if (stereoMode == C_STEREO_DISABLED)
    {
        camera->setOrthographicView(1.3);
    }

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(1.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-1.0, 0.0,-0.4);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------


    // create a sphere (cursor) to represent the haptic device
    hand = new cMaestroHand(false,true,false);

    // create haptic points for each digit
    for (int i = 0; i < 3; i++)
    {
        haptic_points[i] = new cToolCursor(world);
        haptic_points[i]->setRadius(hand->h_hand->radius());
        world->addChild(haptic_points[i]);
        haptic_points[i]->start();
    }

    // gets the fingertip radius
    radius = hand->h_hand->radius();

    // insert cursor inside world
    world->addChild(hand->h_hand);


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------


    // properties
    double maxStiffness = 1000;


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 0:
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    object0 = new cMesh();

    // create a box
    cCreateBox(object0, 2.0, 0.5, 0.05);

    // create collision detector
    object0->createAABBCollisionDetector(radius);

    // add object to world
    world->addChild(object0);

    // set the position of the object
    object0->setLocalPos(0.0, -0.3, 0.2);

    // set material color
    object0->m_material->setBlueRoyal();

    // set haptic properties
    object0->m_material->setStiffness(0.05 * maxStiffness);
    object0->m_material->setStaticFriction(0.0);
    object0->m_material->setDynamicFriction(0.4);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 1:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    object1 = new cMesh();

    // create a box
    cCreateBox(object1, 2.0, 0.5, 0.05);

    // create collision detector
    object1->createAABBCollisionDetector(radius);

    // add object to world
    world->addChild(object1);

    // set the position of the object
    object1->setLocalPos(0.0, 0.3, 0.2);

    // set material color
    object1->m_material->setRedFireBrick();

    // set haptic properties
    object1->m_material->setStiffness(0.05 * maxStiffness);
    object1->m_material->setStaticFriction(0.0);
    object1->m_material->setDynamicFriction(0.0);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 2:
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    object2 = new cMesh();

    // create a box
    cCreateBox(object2, 2.0, 0.5, 0.05);

    // create collision detector
    object2->createAABBCollisionDetector(radius);

    // add object to world
    world->addChild(object2);

    // set the position of the object
    object2->setLocalPos(0.0,-0.3,-0.2);

    // set material color
    object2->m_material->setGreenDarkOlive();

    // set haptic properties
    object2->m_material->setStiffness(0.05 * maxStiffness);
    object2->m_material->setStaticFriction(0.4);
    object2->m_material->setDynamicFriction(0.2);


    ////////////////////////////////////////////////////////////////////////
    // OBJECT 3:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    object3 = new cMesh();

    // create a box
    cCreateBox(object3, 2.0, 0.5, 0.05);

    // create collision detector
    object3->createAABBCollisionDetector(radius);

    // add object to world
    world->addChild(object3);

    // set the position of the object
    object3->setLocalPos(0.0, 0.3,-0.2);

    // set material color
    object3->m_material->setGrayDarkSlate();

    // set haptic properties
    object3->m_material->setStiffness(0.05 * maxStiffness);
    object3->m_material->setStaticFriction(1.0);
    object3->m_material->setDynamicFriction(0.7);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setGrayLevel(0.4);
    camera->m_frontLayer->addChild(labelRates);

    // create a small message
    labelMessage = new cLabel(font);
    labelMessage->m_fontColor.setBlack();
    labelMessage->setText("dynamic/static friction properties");
    camera->m_frontLayer->addChild(labelMessage);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.8f, 0.8f, 0.8f),
                                cColorf(0.8f, 0.8f, 0.8f));


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

    // update position of message label
    labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getWidth())), 40);

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

    // Updates the visual representation
    hand->updateVisualizer();

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // fingertip positions and proxies
    vector<cVector3d> proxy(3);

    // global position of the hand
    cVector3d global_pos;

    // global rotation of the hand
    cVector3d global_rot(0,0,0);

    // initialize position
    global_pos = hand->h_hand->getGlobalPos();

    // update the hand joint angles
    cVector3d thumb_pos;
    cVector3d idx_pos;
    cVector3d mid_pos;
    //cVector3d thumb_proxy;
    //cVector3d idx_proxy;
    //cVector3d mid_proxy;

    hand->updateJointAngles(thumb_pos,idx_pos,mid_pos,global_pos.eigen(),global_rot.eigen());


    // main haptic simulation loop
    while(simulationRunning)
    {

        world->computeGlobalPositions();
        /////////////////////////////////////////////////////////////////////
        // UPDATE THE JOINT ANGLES AND RETURN NEW POSITION
        /////////////////////////////////////////////////////////////////////

        hand->updateJointAngles(thumb_pos,idx_pos,mid_pos,global_pos.eigen(),Eigen::Vector3d(0,0,0));
        //pos = hand->h_hand->getFingertipCenters();
        //std::cout << idx_pos << std::endl;

        int i = 1;
        haptic_points[i]->setDeviceLocalPos(idx_pos);
        //haptic_points[i]->updateFromDevice();
        haptic_points[i]->computeInteractionForces();
        auto temp = haptic_points[i]->getHapticPoint(0);
        proxy[i] = temp->getGlobalPosProxy();
        auto thumb_pos_eigen = thumb_pos.eigen();
        auto idx_pos_eigen = proxy[i].eigen();
        auto mid_pos_eigen = mid_pos.eigen();

        if (temp->getNumCollisionEvents() > 0){
            auto before_idx = idx_pos_eigen;
            hand->computeHandProxy(thumb_pos_eigen,idx_pos_eigen,mid_pos_eigen,
                                   false,true,false);
            auto return_idx = idx_pos_eigen;

        }
        else
        {
            hand->computeHandProxy(thumb_pos_eigen,idx_pos_eigen,mid_pos_eigen,
                                   false,true,false);
        }

        /////////////////////////////////////////////////////////////////////
        // COMMAND A NEW FORCE USING SOME ALGORITHM
        /////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);

    }

    // exit haptics thread
    simulationFinished = true;
}