#include<iostream>
#include <cmath>        // std::abs



namespace fpid{


class Controller
{
public:
    Controller();
    /**
     * @brief SetTarget: Sets the target for the controller.
     * @param newTarget: Desired control value.
     * @return
     */
    bool SetTarget(double newTarget);

    /**
     * @brief ControlSignal: The value for the controlled variable.
     * @param actual: Actual controlled parameter value.
     * @return
     */
    double ControlSignal(double actual);
    bool Finished();
private:
    double target;
    bool finished;
    //variables to check finish
    double repeat,range, maxRepeat;

};


}//namespace fpid
