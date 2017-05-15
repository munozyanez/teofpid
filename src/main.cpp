
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


#include <chrono>
#include <ctime>

#include "MiddlewareInterface.h"
#include "LibraryInterface.h"
#include "fpid.h"
#include "fcontrol.h"
#include "IPlot.h"

//local functions
//int velocityCurve(double Ts, double vel, int jointNumber, MWI::Robot& robot, std::vector<double> &pos);

using namespace std;



int main()
{



    MWI::Robot rightArm("teo","rightArm");
    if (rightArm.GetError()!=0)
    {
        std::cout << "MWI::Robot rightArm(\"teoSim\",\"rightArm\") not available. ERROR: " << rightArm.GetError() << std::endl;
        return -1;

    }
    rightArm.SetControlMode(2);

    double Ts = 0.01;


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

   // SystemBlock model(motorNum,motorDen);

    //plotters
    IPlot pt(Ts),vt(Ts),at(Ts);
    IPlot ptTeo(Ts),vtTeo(Ts),atTeo(Ts);

    double ka=10.09;//acceleration
    //instantiate object motor
    SystemBlock acc(
                std::vector<double> {ka},
                std::vector<double> {1}
                );
    //graph: acc.SetSaturation(-24,24);
    acc.SetSaturation(-5,5);


    //instantiate object motor
    SystemBlock modelVel(
                std::vector<double> {Ts,Ts},//{ka*Ts,ka*Ts},
                std::vector<double> {-2,+2}//{Ts-2,Ts+2}
                );

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-10,10);

    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {Ts,Ts},
                std::vector<double> {-2,+2}
                );

    double signal,modelSignal,jointPos;


    //old PIDBlock control(2,0.5,1,Ts);
    PIDBlock control(1,0,0,Ts);

    PIDBlock modelControl(control);

    //instantiate object motor
    SystemBlock controlLimit(
                std::vector<double> {1},//{ka*Ts,ka*Ts},
                std::vector<double> {1}//{Ts-2,Ts+2}
                );

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    controlLimit.SetSaturation(-100,100);


    rightArm.DefaultPosition();
    yarp::os::Time::delay(5);

    rightArm.SetControlMode(2);

    //time_t t;
    double target = +30;
    double error, modelError;
    int jointNumber = 3;
    std::vector<double> realPos(0,0), times(0,0);
    std::vector<double> modelPos(1,0);

    //control loop
    long loops = 10/Ts;

    for (ulong i=0; i<loops; i++)
    {
        jointPos = rightArm.GetJoint(jointNumber);

        realPos.push_back(jointPos);
        times.push_back(Ts*i);

        ptTeo.pushBack(jointPos);

        error=target-jointPos;
        error = error/(Ts*Ts);

        //signal = control.OutputUpdate(error);
        signal = error > control > controlLimit;
        rightArm.SetJointVel(jointNumber,signal);

        modelError = target-modelEncoder.GetState();
        //modelError = modelError/(Ts*Ts);


        //THE BLOCK DIAGRAM
        modelError > modelControl > acc > modelVel  >  modelEncoder;



        //plot data
        //modelPos.push_back( encoder.GetState() );
        pt.pushBack(modelEncoder.GetState());
        at.pushBack(acc.GetState());


        std::cout << times[i]
                     << " , real signal: " << signal
                     << " , jointPos: " << jointPos

                        << " , modelError: " << modelError
                        << " , modelVel: " << modelVel.GetState()

                     << " , modelSignal: " << modelControl.GetState()
                     << " , modelPos: " << modelEncoder.GetState()
                        << std::endl;
        yarp::os::Time::delay(Ts);

    }

    pt.Plot();
    ptTeo.Plot();

    pt.Save("pVt.txt");
    ptTeo.Save("pVtTeo.txt");

    rightArm.SetJointVel(jointNumber,0.);



    return 0;
}


//int velocityCurve(double Ts, double vel, int jointNumber, MWI::Robot& robot, std::vector<double> &pos)
//{
//    int loops = 6/Ts;
//    double totalTime=0;
//    double actualTime,lastTime, elapsedTime;
//    pos.clear();


//    std::fstream gdata;
//    gdata.open ("/home/buyus/Escritorio/velocityProfile.csv", std::fstream::out);


//    double lastJointPos,jointPos;
//    int repeat=1;
//    lastJointPos=robot.GetJoint(jointNumber);

//    double actualVel;
//    robot.SetJointVel(jointNumber, vel);


//    for(int i=0; i<loops; i++)
//    {
//        jointPos=robot.GetJoint(jointNumber);

//        pos.push_back(jointPos);

//        if (jointPos==lastJointPos)
//        {
//            repeat++;
//        }
//        else
//        {
//     /*   lastTime = actualTime; //now actualTime is not actual but last
//        actualTime = ((float)clock())/CLOCKS_PER_SEC;
//        elapsedTime = actualTime-lastTime;
//        totalTime +=elapsedTime;*/
//        totalTime += Ts*repeat;
//        actualVel = (jointPos-lastJointPos)/(Ts*repeat);

//        //fprintf (gdata, "%f - %f - %f - %f \n",Ts*i,vel,actualVel,jointPos);
//        gdata << totalTime << " - "
//                 << repeat << " - "
//              << vel << " - "
//              << actualVel << " - "
//              << jointPos << " "
//              << std::endl;


//        lastJointPos = jointPos;
//        repeat=1;
//        }


//        //std::chrono::high_resolution_clock()
//        yarp::os::Time::delay(Ts);


//    }
//   //step =0;
//    robot.SetJointVel(jointNumber, 0);


//    gdata.close();
//}

