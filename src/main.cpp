
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream

#include <yarp/os/all.h>

#include <chrono>
#include <ctime>

#include "MiddlewareInterface.h"
#include "LibraryInterface.h"
#include "fpid.h"
#include "fcontrol.h"

//local functions
int velocityCurve(double Ts, double vel, int jointNumber, MWI::Robot& robot, std::vector<double> &pos);


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
  /*  MWI::Port imuPort("/inertial");
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




    double jointPos;
    jointPos=rightArm.GetJoint(3);
    std::cout << "raj4: " << jointPos  <<std::endl;




    //controller

    fpid::Controller control;
    double signal,command;


    std::fstream gdata;
    gdata.open ("gdata.csv", std::fstream::out);

*/
    //control

    //Robot teo right arm
    std::stringstream robConfig;
    //YARP device
    robConfig << "device remote_controlboard" << " ";
    //To what will be connected
    robConfig << "remote /teo/rightArm" << " ";
    //How will be called on YARP network
    robConfig << "local /local/rightArm/" << " ";
    MWI::Robot rightArm(robConfig);


    double signal,jointPos;


    rightArm.DefaultPosition();
    yarp::os::Time::delay(5);

    //time_t t;
    double Ts = 0.01;
    double target = 45;
    double error;
    int jointNumber = 3;
    long loops = 100;
    std::vector<double> positions(0,0), times(0,0);

    SystemBlock control(1,0,1,0);

    //control loop
    for (ulong i=0; i<loops; i++)
    {
        jointPos = rightArm.GetJoint(jointNumber);
        positions.push_back(jointPos);
        times.push_back(Ts*i);

        error=target-jointPos;
        signal = control.OutputUpdate(error);

        std::cout << time(NULL) << ","
                  << target << ","
                  << jointPos << ","
                  << signal << ","
                  << std::endl;
        //std::cout << command << "" << std::endl;
        //command=double(std::min(signal,1.0));
        rightArm.SetJointVel(jointNumber,signal);
        yarp::os::Time::delay(Ts);

    }

    signal=0;
    rightArm.SetJointVel(jointNumber,signal);


    //rightArm.DefaultPosition();
    //yarp::os::Time::delay(7);
/*
    std::vector curvePositions;
    velocityCurve(0.01, 20, 3, rightArm, curvePositions);*/



    std::vector<double> x,y;

    LibraryInterface li;


    for (int i=0; i<loops; i++)
    {
        x.push_back(i);
        y.push_back(i);
    }

    li.Plot(x,y,50,50);

    return 0;
}


int velocityCurve(double Ts, double vel, int jointNumber, MWI::Robot& robot, std::vector<double> &pos)
{
    int loops = 6/Ts;
    double totalTime=0;
    double actualTime,lastTime, elapsedTime;
    pos.clear();


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

        pos.push_back(jointPos);

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
