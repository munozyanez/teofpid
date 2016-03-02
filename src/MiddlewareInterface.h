#ifndef MIDDLEWAREINTERFACE_H
#define MIDDLEWAREINTERFACE_H

//yarp implementation for MiddlewareInterface

#include<string>
#include<iostream>
#include <yarp/os/all.h>

namespace MWI
{



class MiddlewareInterface
{
public:
    MiddlewareInterface();
};


class Port : MiddlewareInterface
{

public:
    Port();
    Port(std::string portname);



private:
    yarp::os::BufferedPort<yarp::os::Bottle> PortBuffer;
    yarp::os::Bottle* PortData;


};


}
#endif // MIDDLEWAREINTERFACE_H
