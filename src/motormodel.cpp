
#include "fcontrol.h"
#include "IPlot.h"

int main()
{

    double Ts = 0.001;


    //plotters
    IPlot pt(Ts),vt(Ts),at(Ts);
    IPlot mpt(Ts),mvt(Ts),mat(Ts);
    IPlot ptTeo(Ts),vtTeo(Ts),atTeo(Ts);

    double ka=0.1;//acceleration



    //instantiate object motor
    SystemBlock modelVel(
//                std::vector<double> {Ts,Ts},
//                std::vector<double> {-2,+2}
                std::vector<double> {0,ka*Ts*1},
                std::vector<double> {-1,1+ka*Ts}
                );

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-24.4,24.4);

    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {Ts,Ts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                );

    double signal,modelSignal,jointPos;

    PIDBlock modelControl(1,0,10,Ts);



    //time_t t;
    double target = 30.0;
    double error, modelError;
    int jointNumber = 3;

//    rightArm.SetControlMode(3);
//    rightArm.ShowControlModes();
//    rightArm.SetJointPositions(std::vector<double> {0,0,0,target,0,0});
//    rightArm.SetControlMode(1);



    //control loop
    long loops = 15/Ts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {


        (target-modelEncoder.GetState()) > modelControl > modelVel > modelEncoder;
        vt.pushBack(modelVel.GetState());
        pt.pushBack(modelEncoder.GetState());


    }

    pt.Plot();
    //vt.Plot();




    return 0;
}
