#include "MiddlewareInterface.h"

using namespace MWI;

MiddlewareInterface::MiddlewareInterface()
{

}

Port::Port(std::string portname)
{

    //OPEN imu port
    PortBuffer.open(portname);
    if (PortBuffer.isClosed())
    {
        std::cerr << "Can not open "<< portname  << std::endl;
    }
    else
    {
        yarp::os::Network::connect("/inertial", "/inertial:i");

    }

    //READ SENSOR
    double a_x,a_y,a_z;

    PortData = PortBuffer.read(false); //false is not waiting for now
    if (PortData==NULL)
    {
        std::cerr << "No data from imu" << std::endl;
    }
    else
    {
        a_x = PortData->get(3).asDouble();//Linear acceleration in X [m/s^2]
        a_y = PortData->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        a_z = PortData->get(5).asDouble(); //Linear acceleration in Z [m/s^2]
    }

    //Time::delay(10);  //Wait for port to open [s]
}
