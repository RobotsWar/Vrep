#include <zmq.h>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <HardwareImage.h>
#include <HardwareServer.h>
#include "VREPClient.hpp"

HardwareServer server;

/**
 * VREPClient instance
 * Globale variable
 */
static VREPClient VREP;

using namespace std;

static void print_image(unsigned char* image, int* resolution)
{
    HardwareImage img;
    img.width = resolution[0];
    img.height = resolution[1];
    img.data = std::string((char*)image, img.width*img.height);
    server.sendImage(img);
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
    for (int i=0; i<12; i++) {
        if ((i%3)==0) {
            VREP.getMotor(i).writePos(server.getAngle(i)*M_PI/180.0);
        } else {
            VREP.getMotor(i).writePos(-server.getAngle(i)*M_PI/180.0);
        }
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
        server.run();

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

