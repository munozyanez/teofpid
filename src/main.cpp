
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


//#include <chrono>
//#include <ctime>

#include "MiddlewareInterface.h"
#include "LibraryInterface.h"
//#include "fpid.h"
#include "fcontrol.h"
#include "IPlot.h"

//local functions
//int velocityCurve(double Ts, double vel, int jointNumber, MWI::Limb& robot, std::vector<double> &pos);

using namespace std;

#define ROBOT "teo"
bool useRobot = false;

int main()
{

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
        rightArm.SetJointPositions(std::vector<double>{0,0,0,0,0,0});
        yarp::os::Time::delay(5);
        //rightArm.DefaultPosition();
        //yarp::os::Time::delay(5);
        rightArm.SetControlMode(2);



    }

    double Ts = 0.01;


//    //3rd order model sistem id matlab Ts=0.01
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
//    SystemBlock model(motorNum,motorDen);

    //plotters
    IPlot pt(Ts),vt(Ts),at(Ts);
    IPlot mpt(Ts),mvt(Ts),mat(Ts);
    IPlot ptTeo(Ts),vtTeo(Ts),atTeo(Ts);

    double ka=1;//10.09;//acceleration
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
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
//                std::vector<double> {Ts*Ts*ka,2*Ts*Ts*ka,Ts*Ts*ka},
//                std::vector<double> {Ts*Ts*ka+4,(2*Ts*Ts*ka-8),(Ts*Ts*ka+4)}
                );
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-24.4,24.4);

    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {Ts,Ts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                );

    double signal,modelSignal,jointPos;


    double kp=43.2;
    //old PIDBlock control(2,0.5,1,Ts); //handmade
    //PIDBlock control(2.381,0.468,0.077,Ts); //zieger nichols

    PIDBlock control(
//                1,0,0
//                3*0.4031242126480222,0,0.2629927380146635 //handmade

//                11.1,0,2.36 //root locus Mp=0.1 Tp=0.7
//                15.6,0,2.36 //root locus Mp=0.1 Tp=0.5
//                7.79,0,2.36 //root locus Mp=0.1 Tp=1
//                7.34,0,2.07 //root locus Mp=0.15 Tp=1
//                8.68,0,2.76 //root locus Mp=0.05 Tp=1


                //new
//                9.4178,0,3.758550252424463 //freq wc=4.34, pm = pi/3 Yei!was wrong!
                0.9999831907657302,0.0001,0.5971930582234168//it works??? wc=0.998 pm=pi/3
//                0.999,0,-0.3

                //old
//                20.15621063240111,0,5.498573992282151 //freq wc=6349206349206349, pm = 86.2
//                0.2015621063240111,0,0.5498573992282151 //freq wc=0.6349206349206349, pm = pi/3
//                0.4031242126480222,0,0.2629927380146635 //freq wc=0.6349206349206349, pm = pi/8

                //new
//                5.79,0,0.031 //zieger nichols open loop PD only

                //old
//                5.959,11.286,0.787 //zieger nichols open loop
//                43.200,8.981,51.984 //zieger nichols closed loop
//                2.435,3.156,0.303 //Aström y Hägglund (2005): AMIGO open loop


            //(0.4608251339122038,0,2.2376655644385313,Ts); //frequency phase margin
            //(11.1,0,2.36,Ts); //root locus
            ,Ts);


    PIDBlock modelControl(control);
    PIDBlock mC(control);

    //instantiate object motor
    SystemBlock controlLimit(
                std::vector<double> {1},//{ka*Ts,ka*Ts},
                std::vector<double> {1}//{Ts-2,Ts+2}
                );

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    controlLimit.SetSaturation(-1000,1000);



    //time_t t;
    double target = 30;
    double error, modelError;
    int jointNumber = 3;

//    rightArm.SetControlMode(3);
//    rightArm.ShowControlModes();
//    rightArm.SetJointPositions(std::vector<double> {0,0,0,target,0,0});
//    rightArm.SetControlMode(1);



    //control loop
    long loops = 15/Ts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {


        //if (i<=1/Ts) target=1; else target=0;
        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();
        //modelError = modelError/(Ts);

        //signal out from controller
        modelSignal = modelError > modelControl;


        //( modelControl.GetState()-modelVel.GetState() ) > acc > modelVel  >  modelEncoder;

        //next lines simulates model setjointVel

        if (  modelVel.GetState() > modelSignal )
        {
            //constant deceleration of model
            -15 > acc > modelVel  >  modelEncoder;

        }
        else
        {
            //constant acceleration of model
            15 > acc > modelVel  >  modelEncoder;

        }


        //ROBOT BLOCK DIAGRAM
        if (useRobot)
        {
            //vtTeo.pushBack( (rightArm.GetJoint(jointNumber)-jointPos)/Ts );

            jointPos = rightArm.GetJoint(jointNumber);
            error=target-jointPos;
            //error = error/(Ts*Ts);
            signal = error > control > controlLimit;
            //signal = signal*15/24.4; //correct signal as 15 value for vel equals to 24.4 deg/sec
            rightArm.SetJointVel(jointNumber,signal);
            yarp::os::Time::delay(Ts);
        }

        //plot data store
        //modelPos.push_back( encoder.GetState() );
        pt.pushBack(modelEncoder.GetState());
        vt.pushBack(modelVel.GetState());
        at.pushBack(acc.GetState());
        ptTeo.pushBack(jointPos);


        std::cout << i*Ts
                                          << " , real signal: " << signal
                                          << " , jointPos: " << jointPos

                     //                        << " , GetJointVel: " << rightArm.GetJointVel(jointNumber)

//                  << " , modelVel: " << modelVel.GetState()
                  << " , modelSignal: " << modelSignal
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
                  << std::endl;

    }

    pt.Plot();
    pt.Save("pVt.txt");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("pVtTeo.txt");
        //vtTeo.Plot();
        //vtTeo.Save("vVtTeo.txt");

        rightArm.SetJointVel(jointNumber,0.);

    }



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


//        double mV,mP;
//        if (  mV > mC.OutputUpdate(target-mP) )
//        {
//            //constant deceleration of model
//            mV=std::max(-15.,mV-10*Ts);


//        }
//        else
//        {
//            //constant acceleration of model
//            mV=std::min(15.,mV+10*Ts);

//        }
//        mP=mP+mV*Ts;
//        mvt.pushBack(mV);
//        mpt.pushBack(mP);
