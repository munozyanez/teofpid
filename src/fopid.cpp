
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


#include "MiddlewareInterface.h"
#include "fcontrol.h"
#include "IPlot.h"



using namespace std;

#define ROBOT "teo"
bool useRobot = true;

int main()
{

    double dts = 0.01;//change with care!! z tfs depend on it!!

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
    acc.SetSaturation(-10,10); // Was 10.
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
    modelVel.SetSaturation(-24,24);// Was 24.4.
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


    //fod.SetSaturation(-16,16);




//    gndn =

//        2.8774  -13.0555   21.9888  -16.3213    4.5106


//    gndd =

//        0.3300   -1.9233    3.8557   -3.2625    1.0000


//    gnin =

//        0.0386   -0.1672    0.2701   -0.1930    0.0515


//    gnid =

//        0.9875   -3.9624    5.9623   -3.9874    1.0000
//    SystemBlock gnd
//            (//check coeffs to be from z^0 to z^n
//                std::vector<double> {2.8774,  -13.0555,   21.9888,  -16.3213,    4.5106},
//                std::vector<double> {0.3300,   -1.9233,    3.8557,   -3.2625 ,   1.0000},
//                1 //fopd gain
//                );

//    SystemBlock gni
//            (
//                std::vector<double> {0.0386 ,  -0.1672 ,   0.2701 ,  -0.1930  ,  0.0515},
//                std::vector<double> {0.9875 ,  -3.9624  ,  5.9623  , -3.9874 ,   1.0000},
//                1 //fopd gain
//                );

    SystemBlock gnd
            (//ed=0.263
                std::vector<double> {1.5931,   -9.8105,   21.1481,  -19.3126,    6.3821},
                std::vector<double> {0.0107,   -0.5207,    1.9530,   -2.4424,    1.0000},
                1 //fopd gain
                );

    SystemBlock gni
            (//ei=0.792
                std::vector<double> {-0.0101,    0.0414,   -0.0515,    0.0178,    0.0024},
                std::vector<double> {0.2901,   -1.7529,    3.6341,   -3.1713,    1.0000},
                1 //fopd gain
                );

    SystemBlock teognd(gnd);
    SystemBlock teogni(gni);


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
    long loops = 10/dts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {

        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();

        //cout<< kd*gnd.OutputUpdate(modelError) << endl;
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
            signal = error*kp + kd*teognd.OutputUpdate(error) + ki*teogni.OutputUpdate(error);

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
    pt.Save("/home/buyus/Escritorio/ptSim.csv");
    con.Save("/home/buyus/Escritorio/conSim.csv");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("/home/buyus/Escritorio/ptTeo.csv");
        conTeo.Save("/home/buyus/Escritorio/conTeo.csv");
        //vtTeo.Plot();
        //vtTeo.Save("vtTeo.txt");

        rightArm.SetJointVel(jointNumber,0.);

    }


    return 0;

}
