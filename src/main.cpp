
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream

#include <yarp/os/all.h>


#include "MiddlewareInterface.h"
#include "fpid.h"

using namespace std;



int main()
{
    std::cout << "[error]";
    //MWI::Port imuPort;
    //INITIALISE AND CHECK YARP
    yarp::os::Network yarpNet;

    if ( !yarpNet.checkNetwork(2) )
    {
        std::cout << "[error] %s found no YARP network (try running \"yarp detect --write\")." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[success] YARP network found." << std::endl;
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
    jointPos=rightArm.GetJoint(3);
    std::cout << "raj4: " << jointPos  <<std::endl;




    //controller

    fpid::Controller control;
    double signal,command;


    std::fstream gdata;
    gdata.open ("gdata.csv", std::fstream::out);


    //control
/*
    //time_t t;
    double target = 0;
    double error;
    int jointNumber = 3;
   //control loop
    control.SetTarget(target);

    while(control.Finished()==false)
    {
        rightArm.GetJoint(jointNumber,jointPos);
        error=target-jointPos;
        signal = control.ControlSignal(error);

        std::cout << time(NULL) << ","
                  << target << ","
                  << jointPos << ","
                  << signal << ","
                  << std::endl;
        //std::cout << command << "" << std::endl;
        //command=double(std::min(signal,1.0));
        rightArm.SetJointVel(jointNumber,signal);
        yarp::os::Time::delay(0.2);


    }

    signal=0;
    rightArm.SetJointVel(jointNumber,signal);
*/

    double T=0.5;
    int loops = 6/T;
    double vel = -2;
    int jointNumber = 3;

    double lastJointPos;
    lastJointPos=rightArm.GetJoint(jointNumber);

    double actualVel;
    rightArm.SetJointVel(jointNumber, vel);


    for(int i=0; i<loops; i++)
    {
        jointPos=rightArm.GetJoint(jointNumber);

        actualVel = (jointPos-lastJointPos)/T;
        gdata << T << ","
              << vel << ","
              << actualVel << ","
              << jointPos << ","
              << std::endl;
        //std::cout << command << "" << std::endl;

        lastJointPos = jointPos;
        yarp::os::Time::delay(T);


    }
    //step =0;
    rightArm.SetJointVel(jointNumber, 0);


    gdata.close();



    return 0;
}

