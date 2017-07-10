
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
    TimeSignal fodResponse(std::valarray<double>{-0.0018596514746170906,-0.0018904162409326371,-0.0019223607913444103,-0.0019555542119289383,-0.0019900709927068803,-0.0020259915565703644,-0.0020634028505467916,-0.0021023990080078422,-0.0021430820918329433,-0.0021855629301333057,-0.0022299620580849226,-0.002276410781697661,-0.0023250523820658314,-0.0023760434819160639,-0.0024295556001621005,-0.0024857769249014509,-0.0025449143409597029,-0.0026071957550097826,-0.0026728727696878549,-0.0027422237684233165,-0.0028155574853210106,-0.0028932171500428185,-0.0029755853169337824,-0.0030630895117415534,-0.003156208859402352,-0.0032554818943444606,-0.0033615158027940351,-0.0034749974077995932,-0.0035967062861167604,-0.0037275305073062315,-0.003868485616859374,-0.0040207376571990139,-0.004185631247270994,-0.0043647240431168429,-0.0045598293064031577,-0.0047730688556585649,-0.0050069394238294995,-0.0052643964805107746,-0.0055489610231290176,-0.0058648568865041981,-0.0062171890508799458,-0.0066121776876027945,-0.0070574689662147635,-0.0075625530747782076,-0.0081393343062311635,-0.0088029204989191132,-0.00957273483789646,-0.010474111269096397,-0.011540632321603132,-0.012817636424673491,-0.014367622058336332,-0.016278832540614949,-0.018679382525080251,-0.021761480846085212,-0.025825043773393098,-0.031360983082152946,-0.039222228657172273,-0.051008697609650572,-0.070045049829038419,-0.10431600903312038,-0.17779964814929955,-0.3985863086836976,-2.3994599724781747,4.7221140384775255
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
