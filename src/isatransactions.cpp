
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
        rightArm.SetJointPositions(std::vector<double>{0,0,0,0,0,0});
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




    PIDBlock con(4.7598488,11.739026,0.0907297,dts);
    PIDBlock rcon(con);

//    //w30p70isom
//    SystemBlock con(vector<double>{    -878.10379 ,  2687.3388 , -2740.7164  , 931.48164},
//                           vector<double>{   0. ,  0.9493526,  -1.9493134 ,  1.},
//                           1);
    //w30p70monjes
//    SystemBlock con(vector<double>{   3.7361 , -25.1344  , -8.8191 ,  45.4944},
//                           vector<double>{  -0.1913 ,   0.1912  ,  1.3786  ,  1.0000},
//                           1);

//    //dts pure derivative
//    FactorSystemBlock con(vector<double>{-1},
//                           vector<double>{0},
//                           1 );

//        //dts=0.01 s^0.93
//    SystemBlock con(vector<double>{-4.7853  , 43.4275 ,  -6.6602, -138.0768 , 108.2231},
//                    vector<double>{0.0653 ,  -0.2690,   -0.8054  ,  0.5132  ,  1.0000},
//                    1);

//    //neck-fpd controller (iros)
//    SystemBlock con(vector<double>{-6.1207,  114.3922 , -86.2817, -345.2926 , 344.6960},
//                    vector<double>{0.0828  , -0.1748 ,  -0.8769 ,   0.3245,    1.0000},
//                    1);

//    FactorSystemBlock con(vector<double>{-0.5344   , 0.9415   , 0.7518  ,  0.1169},
//                    vector<double>{-0.9853  ,  0.7952 ,  -0.4924  ,  0.1693},
//                    108.22 );


//    SystemBlock rcon(con);

    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;


    //time_t t;
//    modelEncoder.Reset(60);
    double target = 10;
    double error, modelError;
    int jointNumber = 3;

    IPlot pt(dts),vt(dts),at(dts),cs(dts);
    IPlot rpt(dts),rvt(dts),rat(dts),rcs(dts);


    //control loop
    long loops = 20/dts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {

        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();

//        modelSignal = modelError*kp;
//        modelSignal += kd*(modelError > simFs);
//        modelSignal += ki*(modelError > simF1s);

        modelSignal = (modelError > con);

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
            yarp::os::Time::delay(dts);
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
