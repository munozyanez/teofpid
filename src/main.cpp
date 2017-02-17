
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream

#include <yarp/os/all.h>

#include <chrono>
#include <ctime>

#include "MiddlewareInterface.h"
#include "LibraryInterface.h"
#include "fpid.h"

//local functions
int velocityCurve(double Ts, double vel, int jointNumber, MWI::Robot& robot);


//MWI::Robot rightArm(robConfig);


using namespace std;



int main()
{
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
    double target = 45;
    double error;
    int jointNumber = 3;
   //control loop
    control.SetTarget(target);

    rightArm.DefaultPosition();
    yarp::os::Time::delay(5);

    while(control.Finished()==false)
    {
        jointPos = rightArm.GetJoint(jointNumber);
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
        yarp::os::Time::delay(0.01);

        gdata << target << ","
              << error << ","
              << jointPos << ","
              << std::endl;

    }

    signal=0;
    rightArm.SetJointVel(jointNumber,signal);
*/

    rightArm.DefaultPosition();
    yarp::os::Time::delay(7);

    velocityCurve(0.01, 20, 3, rightArm);



    return 0;
}


int velocityCurve(double Ts, double vel, int jointNumber, MWI::Robot& robot)
{
    int loops = 6/Ts;
    double totalTime=0;
    double actualTime,lastTime, elapsedTime;

    std::fstream gdata;
    gdata.open ("/home/buyus/Escritorio/velocityProfile.csv", std::fstream::out);

    //double Ts=0.05;
    //int loops = 6/Ts;
    //double vel = 5;
    //int jointNumber = 3;

    double lastJointPos,jointPos;
    int repeat=1;
    lastJointPos=robot.GetJoint(jointNumber);

    double actualVel;
    robot.SetJointVel(jointNumber, vel);


    for(int i=0; i<loops; i++)
    {
        jointPos=robot.GetJoint(jointNumber);


        if (jointPos==lastJointPos)
        {
            repeat++;
        }
        else
        {
     /*   lastTime = actualTime; //now actualTime is not actual but last
        actualTime = ((float)clock())/CLOCKS_PER_SEC;
        elapsedTime = actualTime-lastTime;
        totalTime +=elapsedTime;*/
        totalTime += Ts*repeat;
        actualVel = (jointPos-lastJointPos)/(Ts*repeat);

        //fprintf (gdata, "%f - %f - %f - %f \n",Ts*i,vel,actualVel,jointPos);
        gdata << totalTime << " - "
                 << repeat << " - "
              << vel << " - "
              << actualVel << " - "
              << jointPos << " "
              << std::endl;


        lastJointPos = jointPos;
        repeat=1;
        }


        //std::chrono::high_resolution_clock()
        yarp::os::Time::delay(Ts);


    }
   //step =0;
    robot.SetJointVel(jointNumber, 0);


    gdata.close();
}
