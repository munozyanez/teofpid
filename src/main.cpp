
#include<iostream>
#include <yarp/os/all.h>

using namespace std;



int main()
{
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



    //OPEN imu port
    yarp::os::BufferedPort<yarp::os::Bottle> imuData;
    imuData.open("/inertial:i");
    if (imuData.isClosed())
    {
        std::cerr << "Can not open /inertial:i" << std::endl;
    }
    else
    {
        yarp::os::Network::connect("/inertial", "/inertial:i");

    }

    //READ SENSOR
    yarp::os::Bottle* imuBottle;
    double a_x,a_y,a_z;

    imuBottle = imuData.read(false); //false is not waiting for now
    if (imuBottle==NULL)
    {
        std::cerr << "No data from imu" << std::endl;
    }
    else
    {
        a_x = imuBottle->get(3).asDouble();//Linear acceleration in X [m/s^2]
        a_y = imuBottle->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        a_z = imuBottle->get(5).asDouble(); //Linear acceleration in Z [m/s^2]
    }

    //Time::delay(10);  //Wait for port to open [s]

    return 0;
}

