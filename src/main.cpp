
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream

#include <yarp/os/all.h>


#include "MiddlewareInterface.h"
#include "fpid.h"

using namespace std;



int main()
{

    //MWI::Port imuPort;

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
    dataIndices << 3 << " " <<  "4 5" ;// X Y Z [m/s^2]

    int indices [] = {3, 4, 5};
    std::vector<double> imuAccel (3);



    //READ SENSOR
    double a_x,a_y,a_z;

    imuPort.Read(dataIndices, imudata);
    //imuPort.ShowAllData();
    imudata >> a_x;
    imudata >> a_y;
    imudata >> a_z;

    std::cout << a_x << a_y << a_z <<std::endl;

    imuPort.Read(indices,imuAccel);

    std::cout << "vector" << imuAccel[0] << imuAccel[1] << imuAccel[2] <<std::endl;

    //Robot teo right arm
    std::stringstream robConfig;
    //YARP device
    robConfig << "device remote_controlboard" << " ";
    //To what will be connected
    robConfig << "remote /teo/rightArm" << " ";
    //How will be called on YARP network
    robConfig << "local /local/rightArm/" << " ";
    MWI::Robot rightArm(robConfig);


    double jointPos;
    rightArm.GetJoint(3,jointPos);
    std::cout << "raj4: " << jointPos  <<std::endl;


  /*   double v1,v0;
    v1=1;
    v0=0;
    rightArm.SetJointVel(3,v1);
    yarp::os::Time::delay(1);
    rightArm.SetJointVel(3,v0);
*/

    //controller

    fpid::Controller control;
    double signal,command;


    std::fstream gdata;
    gdata.open ("gdata.csv", std::fstream::out);



    time_t t;
    double target = 30;
    int jointNumber = 3;
   //control loop
    control.SetTarget(target);
    rightArm.SetJointPos(jointNumber,target);
    gdata << "time(NULL)" << ","
              << "target "<< ","
              << "jointPos" << ","
              << "signal" << ","
              << std::endl;
    while(control.Finished()==false)
    {
        rightArm.GetJoint(jointNumber,jointPos);
        signal = control.ControlSignal(jointPos);
        gdata << time(NULL) << ","
                  << target << ","
                  << jointPos << ","
                  << signal << ","
                  << std::endl;
        //std::cout << command << "" << std::endl;
        //command=double(std::min(signal,1.0));
        //rightArm.SetJointVel(3,signal);
        yarp::os::Time::delay(0.05);


    }
    //signal =0;
    //rightArm.SetJointVel(3,signal);

    gdata.close();



    return 0;
}

