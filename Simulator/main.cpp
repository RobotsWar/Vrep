#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include "VREPClient.hpp"

#include "function.h"
#include "kinematic.h"

double l1[4], l2[4], l3[4];

// Functions
Function rise;
Function step;

#define L0      35
#define L1      32
#define L2      60
#define L3_1    85
#define L3_2    35

float signs[3] = {1, 1, 1};
float frontH = 0;
float t = 0;
float alt = 15;
float h = -55;
float r = 90;
bool backLegs = false;
bool back = false;
float smoothBackLegs = 0.0;
float smoothBack = -1.0;
#define GAIT_WALK       0
#define GAIT_TROT       1
int gait = GAIT_TROT;
float freq = 2.2;
float crab = 0;

float dx = 80;
float dy = -80;
float turn = 0;

/**
 * Initializing functions
 */
void setup_functions()
{
    rise.clear();
    step.clear();
    
    if (gait == GAIT_WALK) {
        // Rising the legs
        rise.addPoint(0.0, 0.0);
        rise.addPoint(0.1, 1.0);
        rise.addPoint(0.3, 1.0);
        rise.addPoint(0.35, 0.0);
        rise.addPoint(1.0, 0.0);

        // Taking the leg forward
        step.addPoint(0.0, -0.5);
        step.addPoint(0.12, -0.5);
        step.addPoint(0.3, 0.5);
        step.addPoint(0.35, 0.5);
        step.addPoint(1.0, -0.5);
    }

    if (gait == GAIT_TROT) {
        // Rising the legs
        rise.addPoint(0.0, 0.0);
        rise.addPoint(0.1, 1.0);
        rise.addPoint(0.4, 1.0);
        rise.addPoint(0.5, 0.0);
        rise.addPoint(1.0, 0.0);

        // Taking the leg forward
        step.addPoint(0.0, -0.5);
        step.addPoint(0.1, -0.5);
        step.addPoint(0.4, 0.5);
        step.addPoint(1.0, -0.5);
    }
}

/**
 * Computing the servo values
 */
void tick()
{
    // Setting up functions
    setup_functions();

    // Incrementing and normalizing t
    t += freq*0.01;
    if (t > 1.0) {
        t -= 1.0;
    }
    if (t < 0.0) t += 1.0;

    // Smoothing 180
    if (backLegs && smoothBackLegs < 1) {
        smoothBackLegs += 0.02;
    }
    if (!backLegs && smoothBackLegs > 0) {
        smoothBackLegs -= 0.02;
    }
    if (back && smoothBack < 1) {
        smoothBack += 0.04;
    }
    if (!back && smoothBack > -1) {
        smoothBack -= 0.04;
    }
        
    for (int i=0; i<4; i++) {
        // Defining in which group of opposite legs this leg is
        bool group = ((i&1)==1);

        // This defines the phase of the gait
        float legPhase;
       
        if (gait == GAIT_WALK) {
            float phases[] = {0.0, 0.5, 0.75, 0.25};
            legPhase = t + phases[i];
        }
        if (gait == GAIT_TROT) {
            legPhase = t + group*0.5;
        }

        float x, y, z, a, b, c;

        // Computing the order in the referencial of the body
        float xOrder = step.getMod(legPhase)*dx;
        float yOrder = step.getMod(legPhase)*dy;

        // Computing the order in the referencial of the leg
        float bodyAngle = -(i*M_PI/2.0 - (M_PI/4.0))*smoothBack;
        if (group) {
            bodyAngle -= DEG2RAD(crab*(-smoothBack));
        } else {
            bodyAngle += DEG2RAD(crab*(-smoothBack));
        }
        float vx = xOrder*cos(bodyAngle)-yOrder*sin(bodyAngle);
        float vy = xOrder*sin(bodyAngle)+yOrder*cos(bodyAngle);

        float enableRise = (abs(dx)>0.5 || abs(dy)>0.5 || abs(turn)>5) ? 1 : 0;

        // This is the x,y,z order in the referencial of the leg
        x = r + vx;
        y = vy;
        z = h + rise.getMod(legPhase)*alt*enableRise;
        if (i < 2) z += frontH;
    
        // Computing inverse kinematics
        if (computeIK(x, y, z, &a, &b, &c, L1, L2, backLegs ? L3_2 : L3_1)) {
            if (group) {
                a += crab*(-smoothBack);
            } else {
                a -= crab*(-smoothBack);
            }

           l1[i] = signs[0]*smoothBack*(a + step.getMod(legPhase)*turn);
           l2[i] = signs[1]*smoothBack*(b);
           l3[i] = signs[2]*smoothBack*(c - 180*smoothBackLegs);
        }
    }
}

/**
 * VREPClient instance
 * Globale variable
 */
static VREPClient VREP;

using namespace std;

static void print_image(unsigned char* image, int* resolution)
{
    static int num = 0;
    num++;
    std::stringstream ss;
    ss << "cam" << num << ".ppm";
    FILE* f = fopen(ss.str().c_str(),"w");
    fprintf(f,"P3\n%d %d\n255\n",resolution[0],resolution[1]);
    for(int i=resolution[0]-1; i>=0; i--)
    {
        for(int j=0; j<resolution[1]; j++)
        {
            int index = 3*(i * resolution[1] + j);
            fprintf(f,"%hhu %hhu %hhu  ", image[index], image[index+1], image[index+2]);
        }
        fprintf(f,"\n");
    }

    fclose(f);
    return;
}

static void exiting(bool success = true)
{
    //Close server connection
    VREP.stop();
    VREP.disconnect();
    if (success) {
        exit(EXIT_SUCCESS);
    } else {
        exit(EXIT_FAILURE);
    }
}

static void signal_handler(int sig, siginfo_t *siginfo, void *context)
{
    cout << endl << "Exiting..." << endl;
    exiting();
}

static void attachSignalHandler()
{
    struct sigaction action;
    bzero(&action, sizeof(action));
    action.sa_sigaction = &signal_handler;
    action.sa_flags = SA_SIGINFO;
    if (sigaction(SIGINT, &action, NULL) < 0) {
        cerr << "Unable to register signal handler" << endl;
        exit(EXIT_FAILURE);
    }
}

static void displayInitialState()
{
    //Display founded motors
    size_t countMotors = VREP.countMotors();
    cout << "Registered motors: " << countMotors << endl;
    for (size_t i=0;i<countMotors;i++) {
        cout << "[" << i << "] ";
        cout << VREP.getMotor(i).getName() << " ";
        cout << "minPos=" << VREP.getMotor(i).getMinPos() << " ";
        cout << "maxPos=" << VREP.getMotor(i).getMaxPos() << " ";
        cout << "TorqueMax=" << VREP.getMotor(i).getTorqueMax() << endl;
    }
    //And force sensors
    size_t countForceSensors = VREP.countForceSensors();
    cout << "Registered force sensors: " << countForceSensors << endl;
    for (size_t i=0;i<countForceSensors;i++) {
        cout << "[" << i << "] ";
        cout << VREP.getForceSensor(i).getName() << endl;
    }
    //And vision sensors
    size_t countVisionSensors = VREP.countVisionSensors();
    cout << "Registered vision sensors: " << countVisionSensors << endl;
    for (size_t i=0;i<countVisionSensors;i++) {
        cout << "[" << i << "] ";
        cout << VREP.getVisionSensor(i).getName() << endl;
    }
}

static void displayState()
{
    size_t countMotors = VREP.countMotors();
    //Display motor states
    for (size_t i=0;i<countMotors;i++) {
        cout << "   #[" << i << "] ";
        cout << VREP.getMotor(i).getName() << " ";
        cout << "pos=" << VREP.getMotor(i).readPos() << " ";
        cout << "torque=" << VREP.getMotor(i).readTorque() << endl;
    }
    size_t countForceSensors = VREP.countForceSensors();
    //Display force sensors value
    for (size_t i=0;i<countForceSensors;i++) {
        cout << "   *[" << i << "] ";
        cout << VREP.getForceSensor(i).getName() << " ";
        cout << "force=" << VREP.getForceSensor(i).readForceNorm() << " ";
        cout << "torque=" << VREP.getForceSensor(i).readTorqueNorm() << endl;
    }
    size_t countVisionSensors = VREP.countVisionSensors();
    //Display vision sensors value
    for (size_t i=0;i<countVisionSensors;i++) {
        cout << "   *[" << i << "] ";
        cout << VREP.getVisionSensor(i).getName() << " ";
    }
    //Display accelerometer sensor
    cout << "   -    Accelerometer ";
    cout << "X=" << VREP.readAccelerometerX() << " ";
    cout << "Y=" << VREP.readAccelerometerY() << " ";
    cout << "Z=" << VREP.readAccelerometerZ() << endl;
    //Display position tracker
    cout << "   -    Position tracker ";
    cout << "X=" << VREP.readPositionTrackerX() << " ";
    cout << "Y=" << VREP.readPositionTrackerY() << " ";
    cout << "Z=" << VREP.readPositionTrackerZ() << endl;
}

static void moveMotorsStep(double t)
{
    tick();
    //double pos = sin(t);
    //for (size_t i=0;i<VREP.countMotors();i++) {
    //    VREP.getMotor(i).writePos(pos);
    //}
    for (int i=0; i<4; i++) {
        VREP.getMotor(3*i).writePos(l1[i]*M_PI/180);
    }
    for (int i=0; i<4; i++) {
        VREP.getMotor(1+3*i).writePos(l2[i]*M_PI/180);
    }
    for (int i=0; i<4; i++) {
        VREP.getMotor(2+3*i).writePos(-l3[i]*M_PI/180);
    }
}

int main(int argc, char* argv[])
{
    //Network parameters
    int port = 0;
    char* ip = NULL;
    
    //Parse input arguments
    if (argc != 3) {
        cerr << "Bad usage. Usage: ./command [ip address] [port number]" << endl;
        cerr << "Provide network parameters to connect to V-REP server" << endl;
        return EXIT_FAILURE;
    } else {
        ip = argv[1];
        port = atoi(argv[2]);
    }

    //Signal attaching
    attachSignalHandler();

    try {
        //Connection to V-REP
        cout << "Connecting to V-REP server " << ip << ":" << port << endl;
        VREP.connect(ip, port);
        //Display initial state
        displayInitialState();
        //Main Loop
        cout << "Starting simulation" << endl;
        VREP.start();
        for (double t=0;t<60.0;t+=0.050) {
            //Display state
            cout << "Simulation step t=" << t << endl;
            displayState();
            //Do next step
            VREP.nextStep();
            //Compute motors move
            moveMotorsStep(t);

            print_image(VREP.getVisionSensor(0).getImage(),
                VREP.getVisionSensor(0).getResolution());
        }
        //End simulation
        cout << "Stopping simulation" << endl;
        VREP.stop();
    } catch (string str) {
        cerr << "Exception error: " << str << endl;
        exiting(false);
    }

    exiting();
}

