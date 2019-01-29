
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


#include "MiddlewareInterface.h"
#include "fcontrol.h"
#include "IPlot.h"



using namespace std;

#define ROBOT "teo"
bool useRobot = 1;

int main()
{

    double dts = 0.01;
    double initialAngle = 50;


    //instantiate object motor model
    SystemBlock motor(
                std::vector<double> {1 ,2, 1},
                std::vector<double> { 39750, - 80000, + 40250 }
                );

    //start at 50 deg
    motor.Reset(initialAngle);

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
        rightArm.SetJointPositions(std::vector<double>{0,0,0,initialAngle,0,0});
        yarp::os::Time::delay(5);
        //rightArm.DefaultPosition();
        //yarp::os::Time::delay(5);
        rightArm.SetControlMode(2);



    }



//    //Pure integrator
//    SystemBlock con(
//                std::vector<double> {dts,dts},
//                std::vector<double> {-2,+2}
//                );
//    SystemBlock rcon(con);

    FractionalController1DOF con(-0.488,dts);
    FractionalController1DOF rcon(con);

    ToolsFControl tools;
    tools.SetSamplingTime(dts);

    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;



    double target = 40;
    double error, modelError;
    int jointNumber = 3;

    IPlot pt(dts),vt(dts),at(dts),cs(dts);
    IPlot rpt(dts),rvt(dts),rat(dts),rcs(dts);


    //control loop
    long loops = 20/dts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {


        tools.WaitSamplingTime();
        //MODEL BLOCK DIAGRAM
        //sum
        modelError = target-motor.GetState();
        //controller signal
        //kp+ki*s^ei
        modelSignal = 0.5449448 * modelError + 0.5449448 * (modelError > con);
        //actuator
        modelSignal > motor;
        //ROBOT BLOCK DIAGRAM
        if (useRobot)
        {

            jointVel = (jointPos-jointLastPos)/dts;
            rvt.pushBack(jointVel);

            jointLastPos = jointPos;
//            jointPos = linFilter(rightArm.GetJoint(jointNumber),i*Ts);

            jointPos = rightArm.GetJoint(jointNumber);

            error=target-jointPos;
            //error = error/(Ts*Ts);
            signal = error > rcon;


            if (fabs(jointVel)>14.4)
            {
            //signal = signal*15/24.4; //correct signal as 15 value for vel equals to 24.4 deg/sec
            }

            rightArm.SetJointVel(jointNumber,1*signal);
//            yarp::os::Time::delay(dts);
            //signal = signal*24.4/15;

            //plot data store
            rpt.pushBack(jointPos);
            rvt.pushBack(jointVel);
            rcs.pushBack(signal);

            std::cout << i*dts

                      << " , signal: " << signal
                      << " , jointVel: " << jointVel
                      << " , jointPos: " << jointPos
                      << std::endl;

        }

        //plot data store
        pt.pushBack(motor.GetState());
        cs.pushBack(modelSignal);


        std::cout << i*dts

                  << " , modelSignal: " << modelSignal
                  << " , modelPosition: " << motor.GetState()
                  << std::endl;

    }

    pt.Plot();
    pt.Save("/home/buyus/Escritorio/ptSim.csv");
    cs.Save("/home/buyus/Escritorio/conSim.csv");

    if (useRobot)
    {
        rpt.Plot();
        rpt.Save("/home/buyus/Escritorio/ptTeo.csv");
        rcs.Save("/home/buyus/Escritorio/conTeo.csv");
        //vtTeo.Plot();
        //vtTeo.Save("vtTeo.txt");


        rightArm.SetJointVel(jointNumber,0.);

    }


    return 0;

}
