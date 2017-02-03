//===============================================================================//
// Name			: GwatchR.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.0
// Description	: Inertial device driver for the LG G watch R smartwatch
//===============================================================================//

#include "device.hpp"

using namespace std;

#ifndef GWATCHR_HPP_
#define GWATCHR_HPP_

//! derivate class "GwatchR", driver of the LG G watch R smartwatch
class GwatchR: public Device
{
	public:
        //! constructor
        //! (call to Device::Device constructor)
        //! @param[in] n    name of the device
        GwatchR(string n): Device(n){}
        
        //! extract actual acceleration values from an offline sample
        //! @param[in] &line    one line transmitted by the device
        //! @return             matrix with the tri-axial acceleration values in m/s^2
        mat extractActual(string &line)
        {
            // file format:
            // ax[float] ay[float] az[float]
            
            // read one line
            //DEBUG:cout <<"Line: " <<line <<endl;
            std::stringstream ss;
            ss<<line;
            float ax, ay, az;
            ss>>ax >>ay >>az;
            
            // no conversion required
            mat noisySample = zeros<mat>(1, 3);
            noisySample(0, 0) = ax;
            noisySample(0, 1) = ay;
            noisySample(0, 2) = az;
            //DEBUG:cout <<"noisySample: " <<noisySample(0,0) <<", "
            //DEBUG:                       <<noisySample(0,1) <<", "
            //DEBUG:                       <<noisySample(0,2) <<endl;
            return noisySample;
	}


        //! fill actual acceleration values from an online sample
        //! @param[in] &ax    x acceleration value
	//! @param[in] &ay    y acceleration value
	//! @param[in] &az    z acceleration value
        //! @return             matrix with the tri-axial acceleration values in m/s^2
	// Function added by Enrique Coronado.
	mat fillActual(float ax, float ay, float az)
        {
            // file format:
            // ax[float] ay[float] az[float]
            
            // no conversion required
            mat noisySample = zeros<mat>(1, 3);
            noisySample(0, 0) = ax;
            noisySample(0, 1) = ay;
            noisySample(0, 2) = az;
            //DEBUG:cout <<"noisySample: " <<noisySample(0,0) <<", "
            //DEBUG:                       <<noisySample(0,1) <<", "
            //DEBUG:                       <<noisySample(0,2) <<endl;
            
            return noisySample;
	}
        
        //! destructor
		~GwatchR()
		{
			//DEBUG:cout<<endl <<"Destroying GwatchR object" <<endl;
		}
};

#endif
