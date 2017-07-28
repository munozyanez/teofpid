
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
    TimeSignal fodResponse(std::valarray<double>{
                               -0.011299789124290166,-0.011486724889708144,-0.011680829370173359,-0.011882522352992503,-0.012092256461431698,-0.012310520368634632,-0.012537842390297996,-0.012774794508467928,-0.013021996887206816,-0.013280122950702022,-0.013549905106122764,-0.013832141207392457,-0.01412770187256604,-0.014437538787379134,-0.014762694151180922,-0.015104311450183067,-0.01546364777741218,-0.015842087960819113,-0.016241160812000201,-0.016662557870514003,-0.017108155095559754,-0.017580038051480437,-0.018080531250944457,-0.018612232466100331,-0.019178053000927897,-0.019781265148946597,-0.020425558352194256,-0.021115105949447988,-0.021854644878278706,-0.022649571310445316,-0.023506055998937292,-0.024431184160347286,-0.025433126094733513,-0.026521346578174579,-0.027706863521680718,-0.029002569718478609,-0.030423636052024857,-0.031988020824529492,-0.03371711865158232,-0.035636594795416546,-0.037777468616930904,-0.040177535706052175,-0.042883256436992714,-0.045952295982679081,-0.049456988327040291,-0.053489133137779865,-0.058166751397993766,-0.063643779611711887,-0.070124275099404471,-0.077883727487394971,-0.087301895915795846,-0.098914972729732722,-0.11350142023186792,-0.13222915581191841,-0.1569205589047018,-0.19055855379120321,-0.23832579322527719,-0.30994384397341768,-0.42561431702216701,-0.63385474130541331,-1.0803629378313908,-2.4219276017199745,-14.579824268822982,21.765978091289533
                           //-45.774487581906278,163.69226772483421,-61.684192438411202,23.66985832794083,-21.164812227231657,14.626768044157462,-12.982285763787011,10.68810110443993,-9.4184325395493858,8.4497509682891803,-7.3930895816943822,6.9829674163134499,-6.065888939707019,5.9302034230074607,-5.113308686202763,5.1242743916989948,-4.3840739423934068,4.4763350783053433,-3.797794207018268,3.9345953273037582,-3.3076078094745824,3.4666120124484867,-2.884158995177021,3.0508020842132781,-2.507918613318918,2.6720072035932256,-2.1651708315734934,2.3190015501746615,-1.8457581075821228,1.9829944022043922,-1.541726267371045,1.6566638879805531,-1.2464502074543944,1.3334761149990726,-0.95401781740907055,1.0071456007752921,-0.65874175749247144,0.67113845280509299,-0.35470991728145407,0.31813279938637962,-0.035297193289967943,-0.060662081233670229,0.30745058845546769,-0.47647200946878909,0.68369097031347392,-0.94445532432410495,1.1071397846110553,-1.4861950753256665,1.5973261821547808,-2.1341343887192927,2.1836059175298503,-2.940063420027788,2.912840661339235,-3.9928274133338211,3.8654209148434679,-5.4596109653096478,5.1926215568308898,-7.6979611014603551,7.2179645146858791,-11.636628041177779,10.781817738923392,-20.67971832496125,18.964344202368039,-65.460558952807077
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

    IPlot pt(dts),vt(dts),at(dts),con(dts);
    IPlot ptTeo(dts),vtTeo(dts),atTeo(dts),conTeo(dts);


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
    pt.Save("ptSim.csv");
    con.Save("conSim.csv");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("ptTeo.csv");
        conTeo.Save("conTeo.csv");
        //vtTeo.Plot();
        //vtTeo.Save("vtTeo.txt");


        rightArm.SetJointVel(jointNumber,0.);

    }


    return 0;

}
