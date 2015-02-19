#include <stdio.h>
#include <iostream>
#include <sstream>
extern "C" {
    #include "extApi.h"
    #include "extApiCustom.h"
    #include "extApiPlatform.h"
}
#include "VREPClient.hpp"

VREPClient::VREPClient() :
    _id(-1),
    _initStreaming(true),
    _motors(),
    _motorsByName(),
    _forceSensors(),
    _visionSensors(),
    _accelerometerXRead(0),
    _accelerometerYRead(0),
    _accelerometerZRead(0),
    _positionTrackerXRead(0),
    _positionTrackerYRead(0),
    _positionTrackerZRead(0)
{
}

void VREPClient::connect(const char* ip, int port)
{
    //Connection to server
    _id = simxStart(ip, port, 1, 0, 
        VREPClient::CONNECTION_TIMEOUT, 
        VREPClient::CONNECTION_CYCLE);
    if (_id == -1) {
        throw std::string("Unable to connect to V-REP Server");
    }
    //Enabling synchonous mode
    if (simxSynchronous(_id, true) != simx_error_noerror) {
        throw std::string("Unable to enable synchronous mode");
    }
    //Retrieve joints
    scanMotors();
    //Retrieve force sensors
    scanForceSensors();
		//Retrieve vision sensors
		scanVisionSensors();
}

void VREPClient::disconnect() const
{
    simxFinish(_id);
}
        
size_t VREPClient::countMotors() const
{
    return _motors.size();
}

size_t VREPClient::countForceSensors() const
{
    return _forceSensors.size();
}

size_t VREPClient::countVisionSensors() const
{
    return _visionSensors.size();
}


const Motor& VREPClient::getMotor(size_t index) const
{
    if (index >= _motors.size()) {
        throw std::string("Invalid motor index");
    } else {
        return _motors[index];
    }
}

Motor& VREPClient::getMotor(const std::string& name)
{
    if (_motorsByName.find(name) != _motorsByName.end()) {
        return *(_motorsByName[name]);
    }
    throw std::string("Invalid motor name");
}

Motor& VREPClient::getMotor(size_t index)
{
    if (index >= _motors.size()) {
        throw std::string("Invalid motor index");
    } else {
        return _motors[index];
    }
}

const ForceSensor& VREPClient::getForceSensor(size_t index) const
{
    if (index >= _forceSensors.size()) {
        throw std::string("Invalid force sensor index");
    } else {
        return _forceSensors[index];
    }
}

ForceSensor& VREPClient::getForceSensor(size_t index)
{
    if (index >= _forceSensors.size()) {
        throw std::string("Invalid force sensor index");
    } else {
        return _forceSensors[index];
    }
}

const VisionSensor& VREPClient::getVisionSensor(size_t index) const
{
    if (index >= _visionSensors.size()) {
        throw std::string("Invalid force sensor index");
    } else {
        return _visionSensors[index];
    }
}

VisionSensor& VREPClient::getVisionSensor(size_t index)
{
    if (index >= _visionSensors.size()) {
        throw std::string("Invalid force sensor index");
    } else {
        return _visionSensors[index];
    }
}
double VREPClient::readAccelerometerX() const
{
    return _accelerometerXRead;
}
double VREPClient::readAccelerometerY() const
{
    return _accelerometerYRead;
}
double VREPClient::readAccelerometerZ() const
{
    return _accelerometerZRead;
}

double VREPClient::readPositionTrackerX() const
{
    return _positionTrackerXRead;
}
double VREPClient::readPositionTrackerY() const
{
    return _positionTrackerYRead;
}
double VREPClient::readPositionTrackerZ() const
{
    return _positionTrackerZRead;
}

void VREPClient::start()
{
    //Start simulation
    simxInt error = simxStartSimulation(_id, simx_opmode_oneshot_wait);
    if (error != simx_error_noerror) {
        throw std::string("Unable to start the simulation");
    }
    if (_initStreaming) {
        _initStreaming = false;
        //Start joint data streaming
        for (size_t i=0;i<_motors.size();i++) {
            //Position
            simxFloat pos;
            simxInt error = simxGetJointPosition(_id, _motors[i].getHandle(), &pos, 
                simx_opmode_streaming);
            if (error != simx_error_noerror && error != simx_error_novalue_flag) {
                throw std::string("Unable to set up joint position streaming");
            }
            //Torque
            simxFloat torque;
            error = simxJointGetForce(_id, _motors[i].getHandle(), &torque, 
                simx_opmode_streaming);
            if (error != simx_error_noerror && error != simx_error_novalue_flag) {
                throw std::string("Unable to set up joint torque streaming");
            }
        }
        //Start force sensor data streaming
        for (size_t i=0;i<_forceSensors.size();i++) {
            simxInt error = simxReadForceSensor(_id, _forceSensors[i].getHandle(), 
                NULL, NULL, NULL, simx_opmode_streaming);
            if (error != simx_error_noerror && error != simx_error_novalue_flag) {
                throw std::string("Unable to set up force sensor streaming");
            }
        }

        //Start vision sensor data streaming
        for (size_t i=0;i<_visionSensors.size();i++) {
            simxInt error = simxReadVisionSensor(_id, _visionSensors[i].getHandle(), 
                NULL, NULL, NULL, simx_opmode_streaming);
            if (error != simx_error_noerror && error != simx_error_novalue_flag) {
                throw std::string("Unable to set up vision sensor streaming");
            }
        }

        //Start accelerometer data streaming
        simxFloat accelero;
        error = simxGetFloatSignal(_id, "accelerometerX", &accelero, simx_opmode_streaming);
        if (error != simx_error_noerror && error != simx_error_novalue_flag) {
            throw std::string("Unable to set up accelerometer X streaming");
        }
        error = simxGetFloatSignal(_id, "accelerometerY", &accelero, simx_opmode_streaming);
        if (error != simx_error_noerror && error != simx_error_novalue_flag) {
            throw std::string("Unable to set up accelerometer Y streaming");
        }
        error = simxGetFloatSignal(_id, "accelerometerZ", &accelero, simx_opmode_streaming);
        if (error != simx_error_noerror && error != simx_error_novalue_flag) {
            throw std::string("Unable to set up accelerometer Z streaming");
        }
        //Start position tracker data streaming
        simxFloat posTracker;
        error = simxGetFloatSignal(_id, "positionTrackerX", &posTracker, simx_opmode_streaming);
        if (error != simx_error_noerror && error != simx_error_novalue_flag) {
            throw std::string("Unable to set up tracker X streaming");
        }
        error = simxGetFloatSignal(_id, "positionTrackerY", &posTracker, simx_opmode_streaming);
        if (error != simx_error_noerror && error != simx_error_novalue_flag) {
            throw std::string("Unable to set up tracker Y streaming");
        }
        error = simxGetFloatSignal(_id, "positionTrackerZ", &posTracker, simx_opmode_streaming);
        if (error != simx_error_noerror && error != simx_error_novalue_flag) {
            throw std::string("Unable to set up tracker Z streaming");
        }
    } 
    //Simulation step to initialize communication (streaming/buffer)
    error = simxSynchronousTrigger(_id);
    error = simxSynchronousTrigger(_id);
    error = simxSynchronousTrigger(_id);
    if (error != simx_error_noerror) {
        throw std::string("Unable to step the simulation");
    }
    //Pause communication
    if (simxPauseCommunication(_id, 1) != 0) {
        throw std::string("Unable to pause communication");
    }
}

void VREPClient::stop()
{
    simxInt error;
    //Resume communication
    if (simxPauseCommunication(_id, 0) != 0) {
        throw std::string("Unable to resume communication");
    }
    //Sync with V-REP
    error = simxSynchronousTrigger(_id);
    error = simxSynchronousTrigger(_id);
    error = simxSynchronousTrigger(_id);
    if (error != simx_error_noerror) {
        throw std::string("Unable to step the simulation");
    }
    //Stop simulation
    error = simxStopSimulation(_id, simx_opmode_oneshot_wait);
    if (error != simx_error_noerror) {
        throw std::string("Unable to stop the simulation");
    }
}

void VREPClient::nextStep()
{
    //Update motor data
    for (size_t i=0;i<_motors.size();i++) {
        _motors[i].update(*this);
    }
    //Update force sensors data
    for (size_t i=0;i<_forceSensors.size();i++) {
        _forceSensors[i].update(*this);
    }
    //Update vision sensors data
    for (size_t i=0;i<_visionSensors.size();i++) {
        _visionSensors[i].update(*this);
    }
    //Update accelerometer sensor
    readAccelerometer();
    //Update position tracker
    readPositionTracker();
    
    //Resume communication
    if (simxPauseCommunication(_id, 0) != 0) {
        throw std::string("Unable to resume communication");
    }
    //Simulation step
    simxInt error = simxSynchronousTrigger(_id);
    if (error != simx_error_noerror) {
        throw std::string("Unable to step the simulation");
    }
    //Pause communication
    if (simxPauseCommunication(_id, 1) != 0) {
        throw std::string("Unable to pause communication");
    }
}

void VREPClient::scanMotors()
{
    simxInt motorCount = 0;
    simxInt* motorArray = NULL;
    //Get joints
    if (
        simxGetObjects(_id, sim_object_joint_type, &motorCount, &motorArray, 
        simx_opmode_oneshot_wait) != simx_error_noerror
    ) {
        throw std::string("Unable to retrieve motor handles");
    }
    //Save joints
    _motors.clear();
    _motorsByName.clear();
    for (int i=0;i<motorCount;i++) {
        _motors.push_back(motorArray[i]);
    }
    //Load joints data
    for (int i=0;i<motorCount;i++) {
        Motor *m = &_motors[i];
        m->load(*this);
        _motorsByName[m->getName()] = m;
    }
}
 
void VREPClient::scanForceSensors()
{
    simxInt sensorCount = 0;
    simxInt* sensorArray = NULL;
    //Get force sensor
    if (
        simxGetObjects(_id, sim_object_forcesensor_type, &sensorCount, &sensorArray, 
        simx_opmode_oneshot_wait) != simx_error_noerror
    ) {
        throw std::string("Unable to retrieve force sensor handles");
    }
    //Save force sensor
    _forceSensors.clear();
    for (int i=0;i<sensorCount;i++) {
        _forceSensors.push_back(sensorArray[i]);
    }
    //Load force sensor data
    for (int i=0;i<sensorCount;i++) {
        _forceSensors[i].load(*this);
    }
}

void VREPClient::scanVisionSensors()
{
    simxInt sensorCount = 0;
    simxInt* sensorArray = NULL;
    //Get force sensor
    if (
        simxGetObjects(_id, sim_object_visionsensor_type, &sensorCount, &sensorArray, 
        simx_opmode_oneshot_wait) != simx_error_noerror
    ) {
        throw std::string("Unable to retrieve vision sensor handles");
    }
    //Save force sensor
    _visionSensors.clear();
    for (int i=0;i<sensorCount;i++) {
        _visionSensors.push_back(sensorArray[i]);
    }
    //Load force sensor data
    for (int i=0;i<sensorCount;i++) {
        _visionSensors[i].load(*this);
    }
}


std::string VREPClient::getNameFromHandle(simxInt handle) const
{
    simxChar* name = NULL;
    simxInt error = 0;
    error = simxCustomGetObjectName(_id, handle, &name, simx_opmode_oneshot_wait);
    if (error == simx_error_noerror) {
        return std::string(name);
    } else {
        throw std::string("Unable to fetch object name from handle");
    }
}

simxInt VREPClient::getMotorType(simxInt handle) const
{
    simxInt type = -1;
    simxInt error = 0;
    error = simxCustomGetJointType(_id, handle, &type, simx_opmode_oneshot_wait);
    if (error == simx_error_noerror) {
        return type;
    } else {
        throw std::string("Unable to fetch joint type");
    }
}

void VREPClient::getMotorInterval
    (simxInt handle, bool& cyclic, double& min, double& max) const
{
    simxChar cyclicChar;
    simxFloat interval[2];
    simxInt error = 0;

    error = simxCustomGetJointInterval(_id, handle, &cyclicChar, 
        (simxFloat*)interval, simx_opmode_oneshot_wait);
    if (error == simx_error_noerror) {
        cyclic = (bool)cyclicChar;
        min = (double)interval[0];
        max = (double)interval[1];
        return;
    } else {
        throw std::string("Unable to fetch joint type");
    }
}

bool VREPClient::getMotorDynamic(simxInt handle) const
{
    simxInt value;
    simxInt error = 0;

    error = simxGetObjectIntParameter(_id, handle, 2000, &value, 
        simx_opmode_oneshot_wait);
    if (error == simx_error_noerror) {
        return (bool)value;
    } else {
        throw std::string("Unable to fetch joint dynamic");
    }
}

bool VREPClient::getMotorPositionControl(simxInt handle) const
{
    simxInt value;
    simxInt error = 0;

    error = simxGetObjectIntParameter(_id, handle, 2001, &value, 
        simx_opmode_oneshot_wait);
    if (error == simx_error_noerror) {
        return (bool)value;
    } else {
        throw std::string("Unable to fetch joint position control");
    }
}

void VREPClient::writeMotorPosition(simxInt handle, simxFloat pos) const
{
    simxInt error = simxSetJointTargetPosition(_id, handle, pos, simx_opmode_oneshot);
    if (error != simx_error_noerror && error != simx_error_novalue_flag) {
        throw std::string("Unable to send joint target position");
    }
}
        
void VREPClient::writeMotorTorqueMax(simxInt handle, simxFloat force) const
{
    simxInt error = simxSetJointForce(_id, handle, force, simx_opmode_oneshot);
    if (error != simx_error_noerror && error != simx_error_novalue_flag) {
        throw std::string("Unable to send joint maximum torque");
    }
}

double VREPClient::readMotorPosition(simxInt handle) const
{
    simxFloat pos;
    simxInt error = simxGetJointPosition(_id, handle, &pos, simx_opmode_buffer);
    if (error != simx_error_noerror) {
        throw std::string("Unable to read joint current position");
    } 

    return pos;
}

double VREPClient::readMotorTorque(simxInt handle) const
{
    simxFloat torque;
    simxInt error = simxJointGetForce(_id, handle, &torque, simx_opmode_buffer);
    if (error != simx_error_noerror) {
        throw std::string("Unable to read joint current torque");
    } 

    return torque;
}

void VREPClient::readForceSensor(simxInt handle,
    double& forceX, double& forceY, double& forceZ,
    double& torqueX, double& torqueY, double& torqueZ) const
{
    simxFloat force[3];
    simxFloat torque[3];
    simxUChar state;
    simxInt error = simxReadForceSensor(_id, handle, &state, force, 
        torque, simx_opmode_buffer);
    if (error != simx_error_noerror) {
        throw std::string("Unable to read force sensor");
    } else if (state == 0x01) {
        forceX = force[0];
        forceY = force[1];
        forceZ = force[2];
        torqueX = torque[0];
        torqueY = torque[1];
        torqueZ = torque[2];
    }
}

void VREPClient::readAccelerometer()
{
    simxInt error;
    simxFloat accX, accY, accZ;
    error = simxGetFloatSignal(_id, "accelerometerX", &accX, 
        simx_opmode_buffer);
    if (error != simx_error_noerror) {
        accX = 0.0;
    }
    error = simxGetFloatSignal(_id, "accelerometerY", &accY, 
        simx_opmode_buffer);
    if (error != simx_error_noerror) {
        accY = 0.0;
    }
    error = simxGetFloatSignal(_id, "accelerometerZ", &accZ, 
        simx_opmode_buffer);
    if (error != simx_error_noerror) {
        accZ = 0.0;
    }

    _accelerometerXRead = accX;
    _accelerometerYRead = accY;
    _accelerometerZRead = accZ;
}

void VREPClient::readPositionTracker()
{
    simxInt error;
    simxFloat posX, posY, posZ;
    error = simxGetFloatSignal(_id, "positionTrackerX", &posX, 
        simx_opmode_buffer);
    if (error != simx_error_noerror) {
        posX = 0.0;
    }
    error = simxGetFloatSignal(_id, "positionTrackerY", &posY, 
        simx_opmode_buffer);
    if (error != simx_error_noerror) {
        posY = 0.0;
    }
    error = simxGetFloatSignal(_id, "positionTrackerZ", &posZ, 
        simx_opmode_buffer);
    if (error != simx_error_noerror) {
        posZ = 0.0;
    }

    _positionTrackerXRead = posX;
    _positionTrackerYRead = posY;
    _positionTrackerZRead = posZ;
}

/**
 * Read state on a vision Sensor. Does not perform detection
 **/
void VREPClient::readVisionSensor(simxInt handle, simxFloat** auxValues,
																	simxInt** auxValuesCount,simxInt operationMode)
{
	simxInt error=-1;
	static bool first_call=true;
	if(first_call)
	{
		error = simxReadVisionSensor(_id, handle, NULL,
				auxValues, auxValuesCount, simx_opmode_streaming);
		first_call=false;
	}
	else
	{
		error = simxReadVisionSensor(_id, handle, NULL,
				auxValues, auxValuesCount, simx_opmode_buffer);
	}
	if(error!=simx_error_noerror) {
    throw std::string("Unable to read vision sensor");
	}
	return;
}

/**
 * Retrieve the depth buffer from vision sensor
 **/
void VREPClient::getVisionSensorDepthBuffer(simxInt sensorHandle, simxInt* resolution, simxFloat** buffer)
{
	static int first_call = true;
	simxInt error = -1;
	if (first_call)
	{
		error = simxGetVisionSensorDepthBuffer(_id, sensorHandle, resolution, buffer, simx_opmode_streaming_split+4000);
		first_call=false;
	}
	else
	{
		error = simxGetVisionSensorDepthBuffer(_id, sensorHandle, resolution, buffer, simx_opmode_buffer);
	}
	if(error==simx_return_novalue_flag)
	{
		std::cout<<"Vision sensor no depth buffer returned"<<std::endl;
	}
	else if(error!=simx_error_noerror) {
		std::ostringstream stringstream;
		stringstream <<"Unable to get depth buffer from vision sensor, code: "<<error;
    throw stringstream.str();
	}
	return;
}

/**
 * Get image in float from vision sensor
 **/
void VREPClient::getVisionSensorImage(simxInt sensorHandle, simxInt* resolution, simxUChar** image)
{
	static int first_call = true;
	simxInt error=-1;
	if (first_call)
	{
		error=simxGetVisionSensorImage(_id, sensorHandle, resolution,
				image, 0x00, simx_opmode_streaming+500);
//		first_call=false;
	}
	else
	{
		error=simxGetVisionSensorImage(_id, sensorHandle, resolution,
				image, 0x00, simx_opmode_buffer);
	}
	if(error==simx_return_novalue_flag)
	{
		std::cout<<"Vision sensor no image returned"<<std::endl;
	}
	else if(error!=simx_error_noerror) {
		std::stringstream stringstream;
		stringstream <<"Unable to get image from vision sensor, code: "<<error;
    throw stringstream.str();
	}
	return;
}


