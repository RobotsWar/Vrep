#ifndef VISIONSENSOR_HPP
#define VISIONSENSOR_HPP

#include <iostream>
#include "Object.hpp"
class VREPClient;

/**
 * Represent V-REP Vision Sensor Object
 */
class VisionSensor : public Object
{
    public:

        /**
         * Initialize the force sensor with the given V-REP
         * handle
         */
        VisionSensor(simxInt handle);

        /**
         * Free initialized memory
         **/
        ~VisionSensor();
        
        /**
         * Read sensor values from V-REP server
         * Automaticaly called by VREPClient
         */
        void update(VREPClient& VREP);

		/**
		 * Get the image from last update
		 **/
		unsigned char* getImage() const;

		/**
		 * Get depth buffer from last update
		 **/
		float* getDepthBuffer() const;

		/**
		 * Get resolution
		 **/
		int* getResolution() const;

    private:
				simxUChar* _image;
				float* _depth_buffer;
				int _resolution[2];
};

#endif

