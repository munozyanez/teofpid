
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


//#include <chrono>
//#include <ctime>

#include "MiddlewareInterface.h"
#include "LibraryInterface.h"
#include "fpid.h"
#include "fcontrol.h"
#include "IPlot.h"

//local functions
//int velocityCurve(double Ts, double vel, int jointNumber, MWI::Limb& robot, std::vector<double> &pos);

using namespace std;

#define ROBOT "teoSim"

int main()
{

    bool useRobot = false;
    MWI::Limb rightArm(ROBOT,"rightArm");

    if (useRobot)
    {
       // rightArm = MWI::Limb(ROBOT,"rightArm");
        if (rightArm.GetError()!=0)
        {
            std::cout << "MWI::Limb rightArm(\"rightArm\") not available. ERROR: " << rightArm.GetError() << std::endl;
            return -1;

        }
        rightArm.SetControlMode(1);
        rightArm.SetJointPositions(std::vector<double>{0,0,0,45,0,0});
        yarp::os::Time::delay(5);
        //rightArm.DefaultPosition();
        //yarp::os::Time::delay(5);
        rightArm.SetControlMode(2);



    }

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
    acc.SetSaturation(-10,10);


    //instantiate object motor
    SystemBlock modelVel(
                std::vector<double> {Ts,Ts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,1},
//                std::vector<double> {-1,0}
                );

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-15,15);

    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {Ts,Ts},
                std::vector<double> {-2,+2}
                );

    double signal,modelSignal,jointPos;


    double kp=43.2;
    //old PIDBlock control(2,0.5,1,Ts);
    PIDBlock control(2.381,0.468,0.077,Ts);

    PIDBlock modelControl(control);

    //instantiate object motor
    SystemBlock controlLimit(
                std::vector<double> {1},//{ka*Ts,ka*Ts},
                std::vector<double> {1}//{Ts-2,Ts+2}
                );

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    controlLimit.SetSaturation(-1000,1000);



    //time_t t;
    double target = 30.0;
    double error, modelError;
    int jointNumber = 3;

//    rightArm.SetControlMode(3);
//    rightArm.ShowControlModes();
//    rightArm.SetJointPositions(std::vector<double> {0,0,0,target,0,0});
//    rightArm.SetControlMode(1);

    //control loop
    long loops = 20/Ts;

    for (ulong i=0; i<loops; i++)
    {


        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();
        //modelError = modelError/(Ts);

        modelError > modelControl;
        ( modelControl.GetState()-modelVel.GetState() ) > acc > modelVel  >  modelEncoder;


        //ROBOT BLOCK DIAGRAM
        if (useRobot)
        {
            jointPos = rightArm.GetJoint(jointNumber);
            error=target-jointPos;
            //error = error/(Ts*Ts);
            signal = error > control > controlLimit;
            rightArm.SetJointVel(jointNumber,signal);
            yarp::os::Time::delay(Ts);
        }


        //plot data store
        //modelPos.push_back( encoder.GetState() );
        pt.pushBack(modelEncoder.GetState());
        at.pushBack(acc.GetState());
        ptTeo.pushBack(jointPos);


        std::cout << i*Ts
                     //                     << " , real signal: " << signal
                     //                     << " , jointPos: " << jointPos

                     //                        << " , GetJointVel: " << rightArm.GetJointVel(jointNumber)

                  << " , modelError: " << modelError
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelSignal: " << modelControl.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
                  << std::endl;

    }

    pt.Plot();
    pt.Save("pVt.txt");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("pVtTeo.txt");
        rightArm.SetJointVel(jointNumber,0.);

    }

    rightArm.SetJointVel(jointNumber,0.);


    return 0;
}


//int velocityCurve(double Ts, double vel, int jointNumber, MWI::Limb& robot, std::vector<double> &pos)
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

