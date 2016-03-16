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
    Port(const std::string portname);
    bool Read(std::istream &indices, std::ostream &data);
    bool ReadAllData(std::ostream &data);
    bool ShowAllData();
    bool Setup(std::string portname);

private:
    yarp::os::BufferedPort<yarp::os::Bottle> PortBuffer;
    yarp::os::Bottle* onePortData;
    std::string yarpPortString;


};

class Joint : MiddlewareInterface
{
public:
    bool GetPos();
private:

};

}
#endif // MIDDLEWAREINTERFACE_H
