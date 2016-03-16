#include "MiddlewareInterface.h"

using namespace MWI;

MiddlewareInterface::MiddlewareInterface()
{

}

Port::Port(const std::string portname)
{
    Port::Setup(portname);

}

bool Port::Setup(std::string portname)
{
    yarpPortString = portname;
    //OPEN imu port
    PortBuffer.open(yarpPortString+":i");
    if (PortBuffer.isClosed())
    {
        std::cerr << "Can not open "<< yarpPortString+":i"  << std::endl;
    }
    else
    {
        yarp::os::Network::connect(yarpPortString, yarpPortString+":i");
        std::cout << "Connected to "<< PortBuffer.getName() << std::endl;
        //std::cout << PortBuffer.getName();

    }
    //Time::delay(10);  //Wait for port to open [s]

    return true;

}


bool Port::Read(std::istream &indices, std::ostream& data)
{

    int index;

    onePortData = PortBuffer.read(false); //waiting data. TODO: manage wait.
    if (onePortData==NULL)
    {
        std::cerr << "No data in " << yarpPortString << std::endl;
    }
    else
    {

        while(indices >> index)
        {
            data << onePortData->get(index).asString();
        }

    }
    return true;
}

bool Port::ReadAllData(std::ostream& data)
{

    //int index,indices;

    onePortData = PortBuffer.read(false); //waiting data. TODO: manage wait.
    if (onePortData==NULL)
    {
        std::cerr << "No data in " << yarpPortString << std::endl;
    }
    else
    {
        for(int index=0; index<onePortData->size(); index++)
        {
            data << onePortData->get(index).asString();
        }

    }
    return true;
}


bool Port::ShowAllData()
{

    //int index,indices;
    onePortData = PortBuffer.read(false); //waiting data. TODO: manage wait.


    if (onePortData==NULL)
    {
        std::cerr << "No data in " << yarpPortString << std::endl;
    }
    else
    {
        std::cout << onePortData->size();
        for(int index=0; index<onePortData->size(); index++)
        {
            std::cout << onePortData->get(index).asString();
        }

    }
    return true;
}


// Joint

bool Joint::GetPos()
{
    //CONNECT TO ROBOT LEFT LEG
   /* Property optionsLeftLeg;                                //YARP class for storing name-value (key-value) pairs
    optionsLeftLeg.put("device","remote_controlboard");     //YARP device
    optionsLeftLeg.put("remote","/teo/leftLeg");            //To what will be connected
    optionsLeftLeg.put("local","/juan/leftLeg");            //How will be called on YARP network
    PolyDriver deviceLeftLeg(optionsLeftLeg);               //YARP multi-use driver with the given options
    if(!deviceLeftLeg.isValid())
    {
      printf("[error] /teo/leftLeg device not available.\n");
      deviceLeftLeg.close();
      Network::fini();
      return 1;
    }
    IVelocityControl *velLeftLeg;                 //Velocity controller
    if ( ! deviceLeftLeg.view(velLeftLeg) )
    {
        printf("[error] Problems acquiring robot left leg IVelocityControl interface.\n");
        return false;
    } else printf("[success] TEO_push acquired robot left leg IVelocityControl interface.\n");
    velLeftLeg->setVelocityMode();

    //CONNECT TO ROBOT RIGHT LEG
    Property optionsRightLeg;                                //YARP class for storing name-value (key-value) pairs
    optionsRightLeg.put("device","remote_controlboard");      //YARP device
    optionsRightLeg.put("remote","/teo/rightLeg");            //To what will be connected
    optionsRightLeg.put("local","/juan/rightLeg");            //How will be called on YARP network
    PolyDriver deviceRightLeg(optionsRightLeg);               //YARP multi-use driver with the given options
    if(!deviceRightLeg.isValid())
    {
      printf("[error] /teo/rightLeg device not available.\n");
      deviceRightLeg.close();
      Network::fini();
      return 1;
    }
    IVelocityControl *velRightLeg;                 //Velocity controller
    if ( ! deviceRightLeg.view(velRightLeg) )
    {
        printf("[error] Problems acquiring robot right leg IVelocityControl interface.\n");
        return false;
    } else printf("[success] TEO_push acquired robot right leg IVelocityControl interface.\n");
    velRightLeg->setVelocityMode();*/
}

bool Joint::SetPos(double)
{
    std::cout << "TODO";
}

Robot::Robot(std::istream& config)
{
    std::string name,value;
    while(config >> name)
    {
        config >> value;
        robotOptions.put(name,value);
        std::cout << name << value;
    }
    deviceDriver.open(robotOptions);               //YARP multi-use driver with the given options
    if(!deviceDriver.isValid())
    {
      std::cerr << "Not avilable: " << robotOptions.toString() << std::endl;
      deviceDriver.close();
      //return;
    }

    if ( ! deviceDriver.view(iVel) )
    {
        std::cerr << "vControl Not avilable." << std::endl;
        velAxes = 0;
    }
    else
    {
        iVel->setVelocityMode();
        iVel->getAxes(&velAxes);
    }

    if ( ! deviceDriver.view(iEnc) )
    {
        std::cerr << "encoders Not avilable." << std::endl;
        encoderAxes=0;
    }
    else
    {
        iEnc->getAxes(&encoderAxes);

    }

}

bool Robot::GetJoints(std::ostream &positions)
{
    double* encValuePtr;

    if (encoderAxes == 0)
    {
        std::cerr << "encoderAxes = 0" << std::endl;
        return false;
    }

    iEnc->getEncoders(encValuePtr);

    for (int i=0; i<encoderAxes; i++)
    {
        positions << *encValuePtr << " ";
        encValuePtr++;

    }


    return true;
}

bool Robot::GetJoint(int encoderAxe, double& encoderValue)
{

    if (encoderAxe > encoderAxes)
    {
        std::cerr << "No such axe number" << std::endl;
        return false;
    }

    iEnc->getEncoder(encoderAxe, &encoderValue);

    return true;
}

bool Robot::SetJointVel(int axe, double &value)
{

    if (axe > velAxes)
    {
        std::cerr << "No such axe number" << std::endl;
        return false;
    }

    iVel->velocityMove(axe, value);

    return true;

}





