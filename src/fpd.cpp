
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


#include "MiddlewareInterface.h"
#include "fcontrol.h"
#include "IPlot.h"



using namespace std;

#define ROBOT "teo"
bool useRobot = false;

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
                std::vector<double> {dts,dts},
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
                std::vector<double> {dts,dts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                );


    //instantiate object control
    double kp=1.73;
    double kd=0.53;
    double N = 20;    // LPFfilter N
    //matlab fod ts=0.01
//    SystemBlock fod(
//                std::vector<double> {1.7963096, - 14.815094, + 35.195846, - 33.176969, + 11},
//                std::vector<double> {0.5187250, - 2.3022224, + 4.0225029, - 3.2386161, + 1},
//                1 //fod gain
//                );
    //scilab fod ts=0.01
    TimeSignal fodResponse(std::valarray<double>{-0.00051888407113491789,-0.00052746812433587917,-0.00053638136350695495,-0.00054564306519840826,-0.00055527401378033128,-0.00056529664902646405,-0.00057573523108868049,-0.00058661602526877588,-0.0005979675093773242,-0.00060982060691916256,-0.0006222089498860617,-0.00063516917557133405,-0.00064874126258339555,-0.00066296891214196775,-0.00067789998183411665,-0.00069358698031932089,-0.00071008763305966597,-0.00072746553107950877,-0.00074579087710340019,-0.00076514134629187235,-0.00078560308231871061,-0.00080727185388361619,-0.00083025440214690768,-0.00085467001628905265,-0.00088065238281062261,-0.00090835176477960158,-0.0009379375806411322,-0.00096960146928260285,-0.0010035609499360713,-0.0010400638137362875,-0.0010793934204344491,-0.0011218751217684231,-0.0011678840962931769,-0.0012178549646455225,-0.0012722936671097735,-0.001331792238188773,-0.0013970473218361686,-0.0014688835597208784,-0.001548283388339181,-0.0016364253514327364,-0.0017347338518902563,-0.0018449444556900871,-0.0019691906139841608,-0.0021101203000537623,-0.0022710550760672767,-0.0024562103645235419,-0.0026710056655134609,-0.0029225097127125463,-0.003220092777727254,-0.0035764052894539742,-0.0040088857121424858,-0.0045421558917305991,-0.0052119626624636609,-0.0060719365587929824,-0.0072057608822289421,-0.0087504109229987782,-0.010943873065681772,-0.014232559724361386,-0.019544124861150036,-0.029106483763467354,-0.049610051419477511,-0.11121443419445756,-0.66950263317542635,0.99948938869709691
                           }, dts);
    FSystemBlock foc(fodResponse);

    FSystemBlock control(fodResponse);


    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;


    //time_t t;
    double target = 30;
    double error, modelError;
    int jointNumber = 3;

    IPlot pt(dts),vt(dts),at(dts);
    IPlot ptTeo(dts),vtTeo(dts),atTeo(dts);


    //control loop
    long loops = 10/dts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {

        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();

        //signal out from controller
        modelSignal = modelError > foc;
        modelSignal *= kd;
        modelSignal += modelError*kp;
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
            vtTeo.pushBack(jointVel);

            jointLastPos = jointPos;
//            jointPos = linFilter(rightArm.GetJoint(jointNumber),i*Ts);

            jointPos = rightArm.GetJoint(jointNumber);

            error=target-jointPos;
            //error = error/(Ts*Ts);
            signal = error > control;
            signal *= kd;
            signal += error*kp;

            if (fabs(jointVel)>14.4)
            {
            //signal = signal*15/24.4; //correct signal as 15 value for vel equals to 24.4 deg/sec
            }

            rightArm.SetJointVel(jointNumber,signal);
            yarp::os::Time::delay(dts);
            signal = signal*24.4/15;

            //plot data store
            ptTeo.pushBack(jointPos);
            vtTeo.pushBack(jointVel);

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


        std::cout << i*dts

                  << " , modelSignal: " << modelSignal
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
                  << std::endl;

    }

    pt.Plot();
    pt.Save("ptSim.txt");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("ptTeo.txt");
        //vtTeo.Plot();
        //vtTeo.Save("vtTeo.txt");


        rightArm.SetJointVel(jointNumber,0.);

    }


    return 0;

}
