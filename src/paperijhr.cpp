
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
    acc.SetSaturation(-10,10);
    //instantiate object motor
    SystemBlock modelVel(
                std::vector<double> {1,1},
                std::vector<double> {-1,+1},
//                std::vector<double> {0,dts*1},
//                std::vector<double> {-1,1}
//                std::vector<double> {Ts*Ts*ka,2*Ts*Ts*ka,Ts*Ts*ka},
//                std::vector<double> {Ts*Ts*ka+4,(2*Ts*Ts*ka-8),(Ts*Ts*ka+4)}
                dts/2);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-24.4,24.4);
    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {dts,dts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                );


    //instantiate object control

    double N = 20;    // LPFfilter N



    double kp;
    double ki;
    double kd;

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


    //time_t t;
    modelEncoder.Reset(60);
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
        modelError = target-modelEncoder.GetState();

        //kp+ki*s^ei
        modelSignal = 0.5449448 * modelError + 0.5449448 * (modelError > con);
//        modelSignal += kd*(modelError > simFs);
//        modelSignal += ki*(modelError > simF1s);

//        modelSignal = (modelError > con);

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
        pt.pushBack(modelEncoder.GetState());
        vt.pushBack(modelVel.GetState());
        at.pushBack(acc.GetState());
        cs.pushBack(modelSignal);


        std::cout << i*dts

                  << " , modelSignal: " << modelSignal
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
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
