
#include<iostream>
#include <yarp/os/all.h>

#include "MiddlewareInterface.h"

using namespace std;



int main()
{

    //MWI::Port imuPort;
    cout << "Hello World!" << endl;

    //INITIALISE AND CHECK YARP
    yarp::os::Network yarpNet;
    if ( !yarpNet.checkNetwork() )
    {
        std::cerr << "[error] %s found no YARP network (try running \"yarp detect --write\")." << std::endl;
        return -1;
    }
    else
    {
        std::cerr << "[success] YARP network found." << std::endl;
    }



    //Setup imu middleware port
    MWI::Port imuPort("/inertial");
    std::stringstream dataIndices, imudata;

    // i = 0,1,2 --> euler angles (deg)
     // i = 3,4,5 --> linear acceleration (m/s²)
     // i = 6,7,8 --> angular speed (deg/s)
     // i = 9,10,11 --> magnetic field (arbitrary units)
    dataIndices << "3 4 5" ;//yarp imu port Linear acceleration in X Y Z [m/s^2]


    //READ SENSOR
    double a_x,a_y,a_z;

    imuPort.Read(dataIndices, imudata);
    imudata >> a_x;
    imudata >> a_y;
    imudata >> a_z;



    return 0;
}

