
#include "fcontrol.h"
#include "IPlot.h"

int main()
{

    double Ts = 0.001;


    //plotters
    IPlot pt(Ts),vt(Ts),at(Ts);
    IPlot mpt(Ts),mvt(Ts),mat(Ts);
    IPlot ptTeo(Ts),vtTeo(Ts),atTeo(Ts);

    double ka=1;//acceleration
    //instantiate object motor
    SystemBlock acc(
                std::vector<double> {ka},
                std::vector<double> {1}
                );
    //graph: acc.SetSaturation(-24,24);
    acc.SetSaturation(-10,10);


    //instantiate object motor
    SystemBlock modelVel(
//                std::vector<double> {Ts,Ts},
//                std::vector<double> {-2,+2}
                std::vector<double> {0,Ts*1},
                std::vector<double> {-1,1}
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

    PIDBlock control(
                1,0,0

            ,Ts);


    PIDBlock modelControl(control);
    PIDBlock mC(control);



    //time_t t;
    double target = 1.0;
    double error, modelError;
    int jointNumber = 3;

//    rightArm.SetControlMode(3);
//    rightArm.ShowControlModes();
//    rightArm.SetJointPositions(std::vector<double> {0,0,0,target,0,0});
//    rightArm.SetControlMode(1);



    //control loop
    long loops = 10/Ts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {


        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();
        //modelError = modelError/(Ts);

        //signal out from controller
        modelSignal=1;


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



        //plot data store
        //modelPos.push_back( encoder.GetState() );
        pt.pushBack(modelEncoder.GetState());
        vt.pushBack(modelVel.GetState());
        at.pushBack(acc.GetState());


        std::cout << i*Ts
                  << " , modelSignal: " << modelSignal
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
                  << std::endl;

    }

    vt.Plot();
    vt.Save("vchar.txt");



    return 0;
}
