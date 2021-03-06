
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


#include "MiddlewareInterface.h"
#include "fcontrol.h"
#include "IPlot.h"



using namespace std;

#define ROBOT "teo"
bool useRobot = 0;

int main()
{

    double dts = 0.01;

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
        rightArm.SetJointPositions(std::vector<double>{0,0,0,60,0,0});
        yarp::os::Time::delay(5);
        //rightArm.DefaultPosition();
        //yarp::os::Time::delay(5);
        rightArm.SetControlMode(2);



    }

    //instantiate object motor
    double ka=1;//10.09;//acceleration
    SystemBlock acc(
                std::vector<double> {ka},
                std::vector<double> {1}
                );
    //graph: acc.SetSaturation(-24,24);
    acc.SetSaturation(-9,9); // Was 10.
    //instantiate object motor
    SystemBlock modelVel(
                std::vector<double> {dts,dts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
//                std::vector<double> {Ts*Ts*ka,2*Ts*Ts*ka,Ts*Ts*ka},
//                std::vector<double> {Ts*Ts*ka+4,(2*Ts*Ts*ka-8),(Ts*Ts*ka+4)}
                );
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-16,16);// Was 24.4.
    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {dts,dts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                );


    //instantiate object control

    double N = 10;    // LPFfilter N

    //filter ABC Kmax=10	5,245	kp=1.789	kd=7.859	ki=68.538	m=0.99	l=0.857

    double kp=1.789;
    double ki=dts*68.538;
    double kd=dts*7.859;
    TimeSignal fodResponse(std::valarray<double>{
                               100.0,-100.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
                           }, dts);
    FSystemBlock fod(fodResponse);
    TimeSignal foiResponse(std::valarray<double>{
                               0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796,0.5517168700209796
                           }, dts);
    FSystemBlock foi(fodResponse);

    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;


    //time_t t;
    double target = 1;
    double error, modelError;
    int jointNumber = 3;

    IPlot pt(dts),vt(dts),at(dts),con(dts);
    IPlot ptTeo(dts),vtTeo(dts),atTeo(dts),conTeo(dts);


    //control loop
    long loops = 100/dts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {

        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();

        //signal out from controller
        modelSignal = ki*(modelError > foi) + kd*(modelError > fod) + modelError*kp;


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

            jointVel = (jointPos-jointLastPos)/dts;

            jointLastPos = jointPos;
//            jointPos = linFilter(rightArm.GetJoint(jointNumber),i*Ts);

            jointPos = rightArm.GetJoint(jointNumber);

            error=target-jointPos;
            //error = error/(Ts*Ts);
//            signal = error > control;
//            signal *= kd;
//            signal += error*kp;

            if (fabs(jointVel)>14.4)
            {
            //signal = signal*15/24.4; //correct signal as 15 value for vel equals to 24.4 deg/sec
            }

            rightArm.SetJointVel(jointNumber,signal);
            yarp::os::Time::delay(dts);
            //signal = signal*24.4/15;

            //plot data store
            ptTeo.pushBack(jointPos);
            vtTeo.pushBack(jointVel);
            conTeo.pushBack(signal);

            std::cout << i*dts

                      << " , signal: " << signal
                      << " , jointVel: " << jointVel
                      << " , jointPos: " << jointPos
                      << std::endl;

        }

        //plot data store
        pt.pushBack(modelEncoder.GetState());
        vt.pushBack(modelVel.GetState());
        at.pushBack(acc.GetState());
        con.pushBack(modelSignal);


        std::cout << i*dts

                  << " , modelSignal: " << modelSignal
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
                  << " , modelError: " << modelError


                  << std::endl;

    }

    pt.Plot();
    pt.Save("ptSim.csv");
    con.Save("conSim.csv");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("ptTeo.csv");
        conTeo.Save("conTeo.csv");
        //vtTeo.Plot();
        //vtTeo.Save("vtTeo.txt");

        rightArm.SetJointVel(jointNumber,0.);

    }


    return 0;

}
