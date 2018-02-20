
#include "fcontrol.h"
#include "IPlot.h"

int main()
{

    double dts = 0.01;


    //plotters
    IPlot pt(dts),vt(dts),at(dts);
    IPlot mpt(dts),mvt(dts),mat(dts);
    IPlot ptTeo(dts),vtTeo(dts),atTeo(dts);

    double ka=1;//acceleration
    double tau=0.1;//systime



    //instantiate object motor
    SystemBlock modelVel(
//                std::vector<double> {Ts,Ts},
//                std::vector<double> {-2,+2}
                std::vector<double> {1,1},
                std::vector<double> {1-(2*tau/dts) , 1+(2*tau/dts)},
                ka);

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-24.4,24.4);

    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {1,1},
                std::vector<double> {-1,+1},
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                dts/2);

    double signal,modelSignal,jointPos;

    PIDBlock modelControl(0.009,0,0.996,dts);



    //time_t t;
    double target = 30.0;
    double error, modelError;
    int jointNumber = 3;

//    rightArm.SetControlMode(3);
//    rightArm.ShowControlModes();
//    rightArm.SetJointPositions(std::vector<double> {0,0,0,target,0,0});
//    rightArm.SetControlMode(1);



    //control loop
    long loops = 15/dts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {


        (target-modelEncoder.GetState()) > modelControl > modelVel > modelEncoder;

        signal=modelControl.GetState();
        modelVel.OutputUpdate(signal);
        vt.pushBack(modelVel.GetState());
        pt.pushBack(modelEncoder.GetState());

        std::cout << i*dts

                  << " , signal: " << signal
                  << " , (target-modelEncoder.GetState()): " << (target-modelEncoder.GetState())
                  << " , vel): " << modelVel.GetState()
                  << " , jointPos: " << jointPos
                  << std::endl;

    }

    pt.Plot();
    //vt.Plot();




    return 0;
}
