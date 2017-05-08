
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
#include "IPlot.h"

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



    //control



    MWI::Robot rightArm("teo","rightArm");
    rightArm.SetControlMode(2);

    double Ts = 0.01;


    //standard model
/*
    double kt=1.09;
    std::vector<double> motorNum(3,0);
    motorNum[2]=Ts*Ts;
    motorNum[1]=2*Ts*Ts;
    motorNum[0]=Ts*Ts;
    std::vector<double> motorDen(3,0);
    motorDen[2]=2*Ts+4*kt;
    motorDen[1]=-8*kt;
    motorDen[0]=-2*Ts+4*kt;

*/    //constant acceleration model
    double km=3;//acceleration
    std::vector<double> motorNum(3,0);
    motorNum[0]=km*Ts*Ts;
    motorNum[1]=km*2*Ts*Ts;
    motorNum[2]=km*Ts*Ts;
    std::vector<double> motorDen(3,0);
    motorDen[0]=4;
    motorDen[1]=-8;
    motorDen[2]=4;

    //3rd order model sistem id matlab Ts=0.01
//    std::vector<double> motorNum(4,0);
//    motorNum[3]=0;
//    motorNum[2]=0.05005;
//    motorNum[1]=-0.09862;
//    motorNum[0]=0.04857;
//    std::vector<double> motorDen(4,0);
//    motorDen[3]=1.;
//    motorDen[2]=-2.83;
//    motorDen[1]=2.662;
//    motorDen[0]=-0.8317;

    SystemBlock model(motorNum,motorDen);
    //plotters
    IPlot pt(Ts),vt(Ts),at(Ts);
    IPlot ptTeo(Ts),vtTeo(Ts),atTeo(Ts);


    double ka=10.09;//acceleration
    //instantiate object motor
    SystemBlock acc(
                std::vector<double> {ka},
                std::vector<double> {1}
                );
    acc.SetSaturation(-40,40);


    //instantiate object motor
    SystemBlock vel(
                std::vector<double> {Ts,Ts},//{ka*Ts,ka*Ts},
                std::vector<double> {-2,+2}//{Ts-2,Ts+2}
                );

    vel.SetSaturation(-80,80);

    //instantiate object encoder
    SystemBlock encoder(
                std::vector<double> {Ts,Ts},
                std::vector<double> {-2,+2}
                );

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
    PIDBlock control(2,0.5,1,Ts);
    PIDBlock controlModel(control);



    rightArm.DefaultPosition();
    yarp::os::Time::delay(5);

    rightArm.SetControlMode(2);


    //control loop
    long loops = 10/Ts;

    for (ulong i=0; i<loops; i++)
    {
        jointPos = rightArm.GetJoint(jointNumber);

        realPos.push_back(jointPos);
        times.push_back(Ts*i);

        ptTeo.pushBack(jointPos);

        error=target-jointPos;
        signal = control.OutputUpdate(error);
        rightArm.SetJointVel(jointNumber,signal);

        modelError = target-encoder.GetState();

        //THE BLOCK DIAGRAM
        modelError > controlModel > acc > vel >  encoder;


//        modelSignal = controlModel.OutputUpdate(modelError);
        modelPos.push_back( encoder.GetState() );
        pt.pushBack(encoder.GetState());
        at.pushBack(acc.GetState());

        //modelPos.push_back(model.OutputUpdate(error)*(0.5));

        std::cout << times[i]
                     << " ,real signal: " << signal
                     << " ,jointPos: " << jointPos

                        << ",modelError: " << modelError
                     << ",modelSignal: " << controlModel.GetState()
                     << "modelPos: " << encoder.GetState()
                        << std::endl;
        //std::cout << command << "" << std::endl;
        //command=double(std::min(signal,1.0));
        yarp::os::Time::delay(Ts);

    }

    pt.Plot();
    ptTeo.Plot();

    pt.Save("pVt.txt");
    ptTeo.Save("pVtTeo.txt");

    rightArm.SetJointVel(jointNumber,0.);

    //rightArm.DefaultPosition();
    //yarp::os::Time::delay(7);
/*
    std::vector curvePositions;
    velocityCurve(0.01, 20, 3, rightArm, curvePositions);*/


    LibraryInterface li;


//    li.PlotAndSave(times,realPos,loops*Ts,target*1.5,"realPos.csv");
//    li.PlotAndSave(times,modelPos,loops*Ts,target*1.5,"modelPos.csv");

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

