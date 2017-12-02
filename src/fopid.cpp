
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
    double kp=0.996;
    double kd=0.01;
    double ki=0.094;
    double N = 10;    // LPFfilter N
    SystemBlock fopid(
                //matlab fod ts=0.01 m=0.669
//                std::vector<double> {1.7963096, - 14.815094, + 35.195846, - 33.176969, + 11},
//                std::vector<double> {0.5187250, - 2.3022224, + 4.0225029, - 3.2386161, + 1},
//                1 //fod gain


                //scilab fod ts=0.01 m=0.669
//                std::vector<double> {264.28273, - 1135.2014, + 1823.6329, - 1298.8201, + 346.10585},
//                std::vector<double> {0, - 0.7674134, + 2.5244259, - 2.7569404, + 1},
//                1 //fod gain


//                //fers1
//                std::vector<double> {0.0572,   -0.4467,    1.5270,   -2.9823,    3.6398,   -2.8427 ,   1.3874,   -0.3869,    0.0472},
//                std::vector<double> {0.0572,   -0.4467,    1.5270,   -2.9823,    3.6398,   -2.8427,    1.3874,   -0.3869,    0.0472},
//                10000 //fopd gain

                //fers2
//                std::vector<double> {0.0375,-0.2929, 1.0017,-1.9574, 2.3903,-1.8680, 0.9123,-0.2546, 0.0311},
//                std::vector<double> {1.0000,-7.7608,26.3466  -51.1018,61.9383  -48.0388,23.2828,-6.4471, 0.7809},
//                10000 //fopd gain

//                //fers2
//                std::vector<double> {1},
//                std::vector<double> {1},
//                1 //fopd gain

                //Gauss newton 4
                std::vector<double> {1},
                std::vector<double> {1},
                1 //fopd gain
                );

    //fod.SetSaturation(-16,16);

    SystemBlock control(fopid);
    control.SetSaturation(-1000,1000);


//    gndn =

//        2.8774  -13.0555   21.9888  -16.3213    4.5106


//    gndd =

//        0.3300   -1.9233    3.8557   -3.2625    1.0000


//    gnin =

//        0.0386   -0.1672    0.2701   -0.1930    0.0515


//    gnid =

//        0.9875   -3.9624    5.9623   -3.9874    1.0000
    SystemBlock gnd
            (
                std::vector<double> {2.8774,  -13.0555,   21.9888,  -16.3213,    4.5106},
                std::vector<double> {0.3300,   -1.9233,    3.8557,   -3.2625 ,   1.0000},
                1 //fopd gain
                );

    SystemBlock gni
            (
                std::vector<double> {0.0386 ,  -0.1672 ,   0.2701 ,  -0.1930  ,  0.0515},
                std::vector<double> {0.9875 ,  -3.9624  ,  5.9623  , -3.9874 ,   1.0000},
                1 //fopd gain
                );

    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;


    //time_t t;
    double target = 30;
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
        //modelSignal = modelError > fopid;
        modelSignal = modelError*kp + kd*gnd.OutputUpdate(modelError) + ki*gni.OutputUpdate(modelError);
//        modelSignal *= kd;
//        modelSignal += modelError*kp;

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
            signal = error > control;
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
