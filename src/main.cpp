
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



    MWI::Port imuPort("/inertial:i");

    return 0;
}

