#include "fpid.h"


fpid::Controller::Controller()
{
    finished=true;
    maxRepeat = 50;
    range = 0.1;
}

bool fpid::Controller::SetTarget(double newTarget)
{
    finished = false;
    target = newTarget;
    return true;
}

double fpid::Controller::ControlSignal(double actual)
{
    double delta = target-actual;
    if (repeat < maxRepeat)
    {
        if (std::abs(delta) < range)
        {
            repeat++;
            std::cout << repeat <<"," <<delta << "," << range << std::endl;
            return 0.1*(delta);
        }
        else
        {
            return 0.1*(delta);
        }


    }
    else
    {
        finished=true;
        return 0;
    }

}

bool fpid::Controller::Finished()
{
    return finished;
}
