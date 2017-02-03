//===============================================================================//
// Name			: device.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.0
// Description	: Inertial device driver (virtual base class)
//===============================================================================//

#include <fstream>
#include <string>

#include "utils.hpp"

using namespace std;

#ifndef DEVICE_HPP_
#define DEVICE_HPP_

//! base class "Device" for the drivers of the inertial devices
class Device
{
	public:
		string name;			//!< name of the device
        
        //! constructor
        //! @param[in] n    name of the device
		Device(string n)
        {
            name = n;
            //DEBUG:cout<<"Device: " <<name <<endl;
        }

		//! print device information
		void printInfo()
        {
            cout<<"Device: " <<name <<endl;
        }
        
        //! extract actual acceleration values from an offline sample
        //! @param[in] &line    one line transmitted by the device
        //! @return             matrix with the tri-axial acceleration values in m/s^2
        virtual mat extractActual(string &line) = 0;
	
        //! fill actual acceleration values from an online sample
        //! @param[in] &ax    x acceleration value
	//! @param[in] &ay    y acceleration value
	//! @param[in] &az    z acceleration value
        //! @return             matrix with the tri-axial acceleration values in m/s^2
	// Function added by Enrique Coronado.
	virtual mat fillActual(float ax, float ay, float az) = 0;

};

#endif
