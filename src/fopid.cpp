
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
        rightArm.SetJointPositions(std::vector<double>{0,0,0,60,0,0});
        yarp::os::Time::delay(6);
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
    double kp;
    double kd;
    double ki;




    SystemBlock s_0_263
            (//ed=0.263
                std::vector<double> {1.5931,   -9.8105,   21.1481,  -19.3126,    6.3821},
                std::vector<double> {0.0107,   -0.5207,    1.9530,   -2.4424,    1.0000},
                1 //fopd gain
                );

    SystemBlock is_0_792
            (//ei=0.792
                std::vector<double> {-0.0101,    0.0414,   -0.0515,    0.0178,    0.0024},
                std::vector<double> {0.2901,   -1.7529,    3.6341,   -3.1713,    1.0000},
                1 //fopd gain
                );

    SystemBlock s_0_99 //tustin
            (
                std::vector<double> {29.2614, -232.0707,  563.9385, -550.6899,  189.5608},
                std::vector<double> {-0.1590,    0.9162,   -0.8099,   -0.9274,    1.0000},
                1 //fopd gain
                );

    SystemBlock s_1_00
            (
                std::vector<double> {-1,   1},
                std::vector<double> {1,   1},
                2/dts //fopd gain
                );

    SystemBlock is_1_00
            (
                std::vector<double> {1,   1},
                std::vector<double> {-1,   1},
                dts/2 //fopd gain
                );

    SystemBlock is_0_99
            (
                std::vector<double> {-0.0039,    0.0174,   -0.0239,    0.0103,    0.0000},
                std::vector<double> {0.3669,   -2.0217,    3.9423,   -3.2875,    1.0000},
                1 //fopd gain
                );

//    SystemBlock is_0_99 //tustin
//            (
//                std::vector<double> {-0.0018,    0.0066,   -0.0032,   -0.0068,    0.0053},
//                std::vector<double> {0.3458,   -1.9551,    3.8723,   -3.2630,    1.0000},
//                1 //fopd gain
//                );

    SystemBlock is_0_01
            (
                std::vector<double> {0.0323,   -0.6552,    2.1194,   -2.4293,    0.9330},
                std::vector<double> {0.0479,   -0.7567,    2.3423,   -2.6334,    1.0000},
                1 //fopd gain
                );

    SystemBlock s_0_01
            (
                std::vector<double> {0.0611,   -0.8462,    2.5515,   -2.8382,    1.0719},
                std::vector<double> {0.0441,   -0.7364,    2.3113,   -2.6190,    1.0000},
                1 //fopd gain
                );


  /*  SystemBlock s_0_01
            (
                std::vector<double> {0.0740,   -0.5111,    1.1659,   -1.0977,    0.3690},
                std::vector<double> {0.0000,   -0.2005,    1.1848,   -1.9751,    1.0000},
                1.0e+05 //fopd gain
                );

    SystemBlock is_0_99
            (
                std::vector<double> {0.0323,   -0.6552,    2.1194,   -2.4293,    0.9330},
                std::vector<double> {0.0479,   -0.7567,    2.3423,   -2.6334,    1.0000},
                1 //fopd gain
                );



    SystemBlock s_0_99
            (
                std::vector<double> {0.0611,   -0.8462,    2.5515,   -2.8382,    1.0719},
                std::vector<double> {0.0441,   -0.7364,    2.3113,   -2.6190,    1.0000},
                1 //fopd gain
                );

    SystemBlock is_0_01
            (
                std::vector<double> {-0.0039,    0.0174,   -0.0239,    0.0103,    0.0000},
                std::vector<double> {0.3669,   -2.0217,    3.9423,   -3.2875,    1.0000},
                1 //fopd gain
                );*/
//    SystemBlock s_0_47
//            (
//                std::vector<double> {9.1117,  -52.6568,  107.7351,  -94.1323,   29.9426},
//                std::vector<double> {0.0013,   -0.4027,    1.7128,   -2.3100,    1.0000},
//                1 //fopd gain
//                );

    SystemBlock s_0_47 //tustin
            (
                std::vector<double> {0.5465,   -9.4621,   28.8985,  -31.8348,   11.8524},
                std::vector<double> {-0.1853,    0.4272,    0.5655,   -1.8053,    1.0000},
                1 //fopd gain
                );



    //W=1-2

//    //ABC (3)//HS(11) w=1
//    kp=0.009;
//    ki=0;
//    kd=0.996;
//    SystemBlock gnd(s_0_01);//REVERSED FOR 0.99!!!!
//    SystemBlock gni(is_0_01);
//    SystemBlock teognd(gnd);
//    SystemBlock teogni(gni);

//    //PSO (4) 6? w=1 MAL??? //UNSTABLE
//    kp=0.995;
//    ki=0.09;
//    kd=0.006;
//    SystemBlock gnd(s_0_01);
//    SystemBlock gni(is_0_99);
//    SystemBlock teognd(gnd);
//    SystemBlock teogni(gni);


//    //PSO (7)//PD w=1 //UNSTABLE AT ROBOT BECAUSE OF SAMPLING TIME
//    kp=0.036;
//    ki=0;
//    kd=0.97;
//    SystemBlock gnd(s_0_99);
//    SystemBlock gni(is_0_01);
//    SystemBlock teognd(gnd);
//    SystemBlock teogni(gni);

//    //HS (8)
//    kp=0.996;
//    ki=0.094;
//    kd=0.01;
//    SystemBlock gnd(s_0_263);
//    SystemBlock gni(is_0_792);
//    SystemBlock teognd(gnd);
//    SystemBlock teogni(gni);

//    //HS (11)//PD w=1
//    kp=0.01;
//    ki=0;
//    kd=1.001;
//    SystemBlock gnd(s_0_99);
//    SystemBlock gni(is_0_99);
//    SystemBlock teognd(gnd);
//    SystemBlock teogni(gni);

    //isow1
    kp=0.541;
    ki=0.0;
    kd=0.541;
    SystemBlock gnd(s_0_47);
    SystemBlock gni(is_0_01);
    SystemBlock teognd(gnd);
    SystemBlock teogni(gni);

//    //cmon
//    kp=1.14;
//    ki=0.0;
//    kd=0.768;
//    SystemBlock gnd(s_0_669);
//        SystemBlock gni(is_0_00);
//        SystemBlock teognd(gnd);
//        SystemBlock teogni(gni);

//    //isow2
//    kp=1.17;
//    ki=0.0;
//    kd=0.64;
//    SystemBlock gnd(s_0_66);
//        SystemBlock gni(is_0_00);
//        SystemBlock teognd(gnd);
//        SystemBlock teogni(gni);




    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;


    modelEncoder.Reset(60);
    //time_t t;
    double target = 30;
    double error, modelError;
    int jointNumber = 3;


    IPlot pt(dts),vt(dts),at(dts),con(dts);
    IPlot ptTeo(dts),vtTeo(dts),atTeo(dts),conTeo(dts);


    //control loop
    long loops = 20/dts;
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
        std::cout << "ks "

                  << " , modelError*kp: " << modelError*kp
                  << " , kd*gnd.GetState(): " << kd*gnd.GetState()
                  << " , ki*gni.GetState(): " << ki*gni.GetState()
                  << std::endl;

        modelSignal=min(modelSignal,1000.);
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
            signal=min(signal,100.);

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
