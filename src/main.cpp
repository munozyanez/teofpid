
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


//#include <chrono>
//#include <ctime>

#include "MiddlewareInterface.h"
#include "LibraryInterface.h"
//#include "fpid.h"
#include "fcontrol.h"
#include "IPlot.h"

//local functions
//int velocityCurve(double Ts, double vel, int jointNumber, MWI::Limb& robot, std::vector<double> &pos);
double linFilter(double sensorValue, double time);
double sqFilter(double sensorValue, double time);



using namespace std;

#define ROBOT "teo"
bool useRobot = true;

int main()
{

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

    double Ts = 0.01;


//    //3rd order model sistem id matlab Ts=0.01
//    std::vector<double> motorNum(4,0);
//    motorNum[3]=0;
//    motorNum[2]=0.05005;
//    motorNum[1]=-0.09862;
//    motorNum[0]=0.04857;
//    std::vector<double> motorDen(4,0);
//    motorDen[3]=1.;
//    motorDen[2]=-2.83;
//    motorDen[1]=2.662;
//    motorDen[0]=-0.8317;
//    SystemBlock model(motorNum,motorDen);

    //plotters
    IPlot pt(Ts),vt(Ts),at(Ts);
    IPlot mpt(Ts),mvt(Ts),mat(Ts);
    IPlot ptTeo(Ts),vtTeo(Ts),atTeo(Ts);

    double ka=1;//10.09;//acceleration
    //instantiate object motor
    SystemBlock acc(
                std::vector<double> {ka},
                std::vector<double> {1}
                );
    //graph: acc.SetSaturation(-24,24);
    acc.SetSaturation(-10,10);


    //instantiate object motor
    SystemBlock modelVel(
                std::vector<double> {Ts,Ts},
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
                std::vector<double> {Ts,Ts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                );

    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;


    double kp=5;
    //old PIDBlock control(2,0.5,1,Ts); //handmade
    //PIDBlock control(2.381,0.468,0.077,Ts); //zieger nichols

    PIDBlock control(
//                1,0,0
//                3*0.4031242126480222,0,0.2629927380146635 //handmade

//                11.1,0,2.36 //root locus Mp=0.1 Tp=0.7
//                15.6,0,2.36 //root locus Mp=0.1 Tp=0.5
//                7.79,0,2.36 //root locus Mp=0.1 Tp=1
//                7.34,0,2.07 //root locus Mp=0.15 Tp=1
//                8.68,0,2.76 //root locus Mp=0.05 Tp=1


                //new
//                9.4178,0,3.758550252424463 //freq wc=4.34, pm = pi/3 Yei!was wrong!
//                0.9999831907657302,0.0001,0.5971930582234168//it works??? wc=0.998 pm=pi/3
//                kp,0,-1+kp/2

                //Karimi, Garcia and Longchamp 120 phim
//                0.81,0.6,1.29

                //final
//                0.962692110537679,0,0.2716365490916453 //pm100 w0.995
//                8.079676771524456,0,1.153531891914077//pm100 w9.95
//                13.56707776765517,0,0.3616952767655167//pm60 w9.95
//                1.891001013656414,0,0.3696249205163607//pm100 w1.99
                1.73454491840556,0,0.5290189748620653//pm120 w1.99

                //old
//                20.15621063240111,0,5.498573992282151 //freq wc=6349206349206349, pm = 86.2
//                0.2015621063240111,0,0.5498573992282151 //freq wc=0.6349206349206349, pm = pi/3
//                0.4031242126480222,0,0.2629927380146635 //freq wc=0.6349206349206349, pm = pi/8

                //test
//                3, 0, 1.3

                //new
//                5.79,0,0.031 //zieger nichols open loop PD only

                //old
//                5.959,11.286,0.787 //zieger nichols open loop
//                43.200,8.981,51.984 //zieger nichols closed loop
//                2.435,3.156,0.303 //Aström y Hägglund (2005): AMIGO open loop


            //(0.4608251339122038,0,2.2376655644385313,Ts); //frequency phase margin
            //(11.1,0,2.36,Ts); //root locus
            ,Ts);


    PIDBlock modelControl(control);
    PIDBlock mC(control);

    //instantiate object motor
    SystemBlock controlLimit(
                std::vector<double> {1},//{ka*Ts,ka*Ts},
                std::vector<double> {1}//{Ts-2,Ts+2}
                );

//    vel.SetSaturation(-5,18);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    controlLimit.SetSaturation(-1000,1000);



    //time_t t;
    double target = 30;
    double error, modelError;
    int jointNumber = 3;

//    rightArm.SetControlMode(3);
//    rightArm.ShowControlModes();
//    rightArm.SetJointPositions(std::vector<double> {0,0,0,target,0,0});
//    rightArm.SetControlMode(1);



    //control loop
    long loops = 20/Ts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {


        //target=10*i*Ts;
        //if (i<=1/Ts) target=1; else target=0;
        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();
        //modelError = modelError/(Ts);

        //signal out from controller
        modelSignal = modelError > modelControl;


        //( modelControl.GetState()-modelVel.GetState() ) > acc > modelVel  >  modelEncoder;

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

            jointVel = (jointPos-jointLastPos)/Ts;
            vtTeo.pushBack(jointVel);

            jointLastPos = jointPos;
            jointPos = linFilter(rightArm.GetJoint(jointNumber),i*Ts);

//            jointPos = rightArm.GetJoint(jointNumber);

            error=target-jointPos;
            //error = error/(Ts*Ts);
            signal = error > control > controlLimit;
            if (fabs(jointVel)>14.4)
            {
            signal = signal*15/24.4; //correct signal as 15 value for vel equals to 24.4 deg/sec
            }

            rightArm.SetJointVel(jointNumber,signal);
            yarp::os::Time::delay(Ts);
            signal = signal*24.4/15;
        }

        //plot data store
        //modelPos.push_back( encoder.GetState() );
        pt.pushBack(modelEncoder.GetState());
        vt.pushBack(modelVel.GetState());
        at.pushBack(acc.GetState());
        ptTeo.pushBack(jointPos);


        std::cout << i*Ts
                                          << " , real signal: " << signal
                                          << " , jointPos: " << jointPos
                                          << " , jointVel: " << jointVel

//                  << " , modelVel: " << modelVel.GetState()
                  << " , modelSignal: " << modelSignal
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
                  << std::endl;

    }

    pt.Plot();
    pt.Save("pVt.txt");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("pVtTeo.txt");
        //vtTeo.Plot();
        //vtTeo.Save("vVtTeo.txt");

        rightArm.SetJointVel(jointNumber,0.);

    }



    return 0;
}

double x0_linFilter=0,x1_linFilter=0;
double t0_linFilter=0,t1_linFilter=1;
double slope_linFilter=0,oldsl_linFilter=0;
double linFilter(double sensorValue, double time)
{

    if (x1_linFilter!=sensorValue)
    {
        //remove sensor precission failures
        if(x0_linFilter==sensorValue
                | (x0_linFilter==6*0.0879+sensorValue) | (x0_linFilter==6*0.0879-sensorValue)
                | (x0_linFilter==5*0.0879+sensorValue) | (x0_linFilter==5*0.0879-sensorValue)
                | (x0_linFilter==4*0.0879+sensorValue) | (x0_linFilter==4*0.0879-sensorValue)

                | (x0_linFilter==3*0.0879+sensorValue) | (x0_linFilter==3*0.0879-sensorValue)
                | (x0_linFilter==2*0.0879+sensorValue) | (x0_linFilter==2*0.0879-sensorValue)
                | (x0_linFilter==0.0879+sensorValue) | (x0_linFilter==0.0879-sensorValue) )
        {
            slope_linFilter=0;
            x1_linFilter=0.5*x1_linFilter+0.5*x0_linFilter;

        }
        else
        {
        x0_linFilter=x1_linFilter;
        t0_linFilter=t1_linFilter;
        x1_linFilter=sensorValue;
        t1_linFilter=time;
        oldsl_linFilter=slope_linFilter;
        slope_linFilter = (x1_linFilter-x0_linFilter) / (t1_linFilter-t0_linFilter);
        }
    }

    //x_linFilter = x1_linFilter + ( (time-t1_linFilter) * (x1_linFilter-x0_linFilter) / (t1_linFilter-t0_linFilter) );

    //if (fabs(slope_linFilter/oldsl_linFilter)>0.1)
    {
        return x1_linFilter + (time-t1_linFilter) * slope_linFilter ;
    }
    return sensorValue;
}

double x0_sqFilter=3,x1_sqFilter=2,x2_sqFilter=1;
double t0_sqFilter=-3,t1_sqFilter=-2,t2_sqFilter=-1;
double a_sqFilter=0,b_sqFilter=0,c_sqFilter=0;
double sqFilter(double sensorValue, double time)
{

    if (x2_sqFilter!=sensorValue)
    {

        x0_sqFilter=x1_sqFilter;
        t0_sqFilter=t1_sqFilter;
        x1_sqFilter=x2_sqFilter;
        t1_sqFilter=t2_sqFilter;
        x2_sqFilter=sensorValue;
        t2_sqFilter=time;


        c_sqFilter = x2_sqFilter;
        a_sqFilter = ( 1/(t0_sqFilter-t1_sqFilter) ) * ( (x0_sqFilter-x2_sqFilter)/t0_sqFilter - (x1_sqFilter-x2_sqFilter)/t1_sqFilter  );
        b_sqFilter = (x1_sqFilter-x2_sqFilter)/t1_sqFilter - a_sqFilter*t1_sqFilter;
        std::cout << a_sqFilter << ", "<< b_sqFilter << ", "<< c_sqFilter << std::endl;

    }

    double dt = time - t2_sqFilter;


    return a_sqFilter*dt*dt+b_sqFilter*dt+c_sqFilter;

}
//int velocityCurve(double Ts, double vel, int jointNumber, MWI::Limb& robot, std::vector<double> &pos)
//{
//    int loops = 6/Ts;
//    double totalTime=0;
//    double actualTime,lastTime, elapsedTime;
//    pos.clear();


//    std::fstream gdata;
//    gdata.open ("/home/buyus/Escritorio/velocityProfile.csv", std::fstream::out);


//    double lastJointPos,jointPos;
//    int repeat=1;
//    lastJointPos=robot.GetJoint(jointNumber);

//    double actualVel;
//    robot.SetJointVel(jointNumber, vel);


//    for(int i=0; i<loops; i++)
//    {
//        jointPos=robot.GetJoint(jointNumber);

//        pos.push_back(jointPos);

//        if (jointPos==lastJointPos)
//        {
//            repeat++;
//        }
//        else
//        {
//     /*   lastTime = actualTime; //now actualTime is not actual but last
//        actualTime = ((float)clock())/CLOCKS_PER_SEC;
//        elapsedTime = actualTime-lastTime;
//        totalTime +=elapsedTime;*/
//        totalTime += Ts*repeat;
//        actualVel = (jointPos-lastJointPos)/(Ts*repeat);

//        //fprintf (gdata, "%f - %f - %f - %f \n",Ts*i,vel,actualVel,jointPos);
//        gdata << totalTime << " - "
//                 << repeat << " - "
//              << vel << " - "
//              << actualVel << " - "
//              << jointPos << " "
//              << std::endl;


//        lastJointPos = jointPos;
//        repeat=1;
//        }


//        //std::chrono::high_resolution_clock()
//        yarp::os::Time::delay(Ts);


//    }
//   //step =0;
//    robot.SetJointVel(jointNumber, 0);


//    gdata.close();
//}


//        double mV,mP;
//        if (  mV > mC.OutputUpdate(target-mP) )
//        {
//            //constant deceleration of model
//            mV=std::max(-15.,mV-10*Ts);


//        }
//        else
//        {
//            //constant acceleration of model
//            mV=std::min(15.,mV+10*Ts);

//        }
//        mP=mP+mV*Ts;
//        mvt.pushBack(mV);
//        mpt.pushBack(mP);
