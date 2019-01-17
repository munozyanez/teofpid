
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream
#include <vector>      // std::vector

#include "MiddlewareInterface.h"
#include "fcontrol.h"
#include "IPlot.h"



using namespace std;

#define ROBOT "teo"
bool useRobot = 1;

int main()
{

    double dts = 0.1;

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


    double jointVel, jointPos=60;

    //time_t t;
    double target = 1;
    int jointNumber = 3;

    IPlot pt(dts),vt(dts),at(dts),con(dts);
    IPlot ptTeo(dts),vtTeo(dts),atTeo(dts),conTeo(dts);

    //input wave
    double w=1;
    double A=10;

    vector<double> pos;

    for (double t=0; t<10; t+=dts)
    {

        target=A*sin(w*t);
        vt.pushBack(target);

        //ROBOT BLOCK DIAGRAM
        if (useRobot)
        {

            jointVel = (rightArm.GetJoint(jointNumber) - jointPos)/dts;
            jointPos = rightArm.GetJoint(jointNumber);

            rightArm.SetJointVel(jointNumber,target);
            yarp::os::Time::delay(dts);

            //plot data store
            vtTeo.pushBack(jointVel);

            std::cout << t << " , jointVel: " << jointVel << std::endl;

        }

//        std::cout << t << " , modelVel: " << target << std::endl;

    }

    vt.Plot();
//    vt.Save("ptSim.csv");


    if (useRobot)
    {

        vtTeo.Plot();
        vtTeo.Save("vtTeo.txt");

        rightArm.SetJointVel(jointNumber,0.);

    }


    return 0;

}
