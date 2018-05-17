#include <iostream>
#include <unistd.h>

#include "IPlot.h"

#include "fcontrol.h"

using namespace std;


int main()
{

    double dts=0.01;

    IPlot pt(dts);



    //motor dts=0.01
    FactorSystemBlock motor(vector<double>{-1, -1},
                      vector<double>{0.9048, 1},
                      0.0002381
                      );

//    //dts pure derivative
//    FactorSystemBlock con(vector<double>{1},
//                    vector<double>{0},
//                    1 );

//    //dts pd (s+10)
//    FactorSystemBlock con(vector<double>{0.9415  ,  0.7518  ,-0.5344    ,  0.1169},
//                          vector<double>{-0.9853  ,  0.7952 ,  -0.4924  ,  0.1693},
//                          108.22 );

    SystemBlock con(vector<double>{-4.7853  , 43.4275 ,  -6.6602, -138.0768 , 108.2231},
                    vector<double>{0.0653 ,  -0.2690,   -0.8054  ,  0.5132  ,  1.0000},
                    1);


//    //dts pd (s+10)
//    FactorSystemBlock con(vector<double>{ 0.1169,-0.5344 ,  0.7518 ,  0.9415     },
//                          vector<double>{ 0.1693,  -0.4924 ,  0.7952  , -0.9853  },
//                          108.22 );






    double consig;
    double target=1;
    double error=0;
    double show;

    for(double t=0;t<2;t+=dts)
    {

        //1-update inputs
        error = target - motor.GetState();

        //2-control diagram
        consig= con.OutputUpdate(error);
        (consig) >  motor;

        //3-update outputs

        //3-update graphs and cout
        show=motor.GetState();
        pt.pushBack(show);
        cout << "t: " << t << ", plotdata: " << show << endl;
        sleep(dts);

    }

    pt.Plot();


    return 0;
}
