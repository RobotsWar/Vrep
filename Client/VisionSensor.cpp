#include <math.h>
#include <stdio.h>
extern "C" {
    #include "extApi.h"
    #include "extApiCustom.h"
    #include "extApiPlatform.h"
}
#include "VisionSensor.hpp"
#include "VREPClient.hpp"

VisionSensor::VisionSensor(simxInt handle) :
    Object(handle)
{
}

VisionSensor::~VisionSensor()
{
	if(_image)
		delete _image;
}

void VisionSensor::update(VREPClient& VREP)
{
	if(_image)
		delete _image;
	VREP.getVisionSensorImage(_handle, (simxInt*) _resolution, &_image);
	unsigned char* tmp = new unsigned char[_resolution[0]*_resolution[1]*3];
	for(int i=0; i< _resolution[0]*_resolution[1]*3; i++)
	{
		tmp[i]=_image[i];
	}
	_image=tmp;
	//VREP.getVisionSensorDepthBuffer(_handle, (simxInt*) _resolution, (simxFloat**) &_depth_buffer);
}

unsigned char* VisionSensor::getImage() const
{
	return _image;
}

float* VisionSensor::getDepthBuffer() const
{
	return _depth_buffer;
}

int* VisionSensor::getResolution() const
{
	return (int*) _resolution;
}
