
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



    MWI::Robot rightArm("teo","rightArm");
    rightArm.SetControlMode(2);

    double Ts = 0.02;


    std::vector<double> motorNum(3,0);
    motorNum[0]=Ts*Ts;
    motorNum[1]=2*Ts*Ts;
    motorNum[2]=Ts*Ts;
    std::vector<double> motorDen(3,0);
    motorDen[0]=-2*Ts+4;
    motorDen[1]=-8;
    motorDen[2]=2*Ts+4;
    SystemBlock model(motorNum,motorDen);


    double signal,modelSignal,jointPos;



    //time_t t;
    double target = 30;
    double error, modelError;
    int jointNumber = 3;
    std::vector<double> realPos(0,0), times(0,0);
    std::vector<double> modelPos(1,0);

    /*//P control

    std::vector<double> controlNum(1,0);
    controlNum[0]=1;
    std::vector<double> controlDen(1,0);
    controlDen[0]=1;
*/
/*     //PID control 5,2,2
    std::vector<double> controlNum(3,0);
    controlNum[0]=Ts*Ts-5*Ts+4;
    controlNum[1]=2*Ts*Ts-8;
    controlNum[2]=Ts*Ts+5*Ts+4;
    std::vector<double> controlDen(3,0);
    controlDen[0]=-Ts;
    controlDen[1]=0;
    controlDen[2]=Ts;


    std::vector<double> controlNum(2,0);
    controlNum[0]=-0.5;
    controlNum[1]=1;
    std::vector<double> controlDen(2,0);
    controlDen[0]=+0.5;
    controlDen[1]=1;


    SystemBlock control(controlNum,controlDen);
    control.SetSaturation(-100,100);
    SystemBlock controlModel(control);
*/
    PIDBlock control(2,0.3,1,Ts);
    PIDBlock controlModel(control);



    rightArm.DefaultPosition();
    yarp::os::Time::delay(5);

    rightArm.SetControlMode(2);


    //control loop
    long loops = 12/Ts;

    for (ulong i=0; i<loops; i++)
    {
        jointPos = rightArm.GetJoint(jointNumber);

        realPos.push_back(jointPos);
        times.push_back(Ts*i);

        error=target-jointPos;
        signal = control.OutputUpdate(error);
        rightArm.SetJointVel(jointNumber,signal);


        modelError = target-modelPos[i];
        modelSignal = controlModel.OutputUpdate(modelError);
        modelPos.push_back( model.OutputUpdate(modelSignal) );

        //modelPos.push_back(model.OutputUpdate(error)*(0.5));

        std::cout << times[i]
                     << " ,real signal: " << signal
                     << " ,jointPos: " << jointPos

                     << ",modelSignal: " << modelSignal
                     << "modelPos[i]: " << modelPos[i]
                        << std::endl;
        //std::cout << command << "" << std::endl;
        //command=double(std::min(signal,1.0));
        yarp::os::Time::delay(Ts);

    }


    rightArm.SetJointVel(jointNumber,0.);

    //rightArm.DefaultPosition();
    //yarp::os::Time::delay(7);
/*
    std::vector curvePositions;
    velocityCurve(0.01, 20, 3, rightArm, curvePositions);*/


    LibraryInterface li;


    li.Plot(times,realPos,loops*Ts,target*1.5);
    li.Plot(times,modelPos,loops*Ts,target*1.5);

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
