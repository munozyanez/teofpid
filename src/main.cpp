
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
    dataIndices << "3 4 5" ;//yarp imu port Linear acceleration in X Y Z [m/s^2]

    //READ SENSOR
    double a_x,a_y,a_z;

    imuPort.Read(dataIndices, imudata);
    imudata >> a_x;
    imudata >> a_y;
    imudata >> a_z;



    return 0;
}

