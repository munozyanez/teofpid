#include "fpid.h"


fpid::Controller::Controller()
{
    finished=true;
    maxRepeat = 30;
    range = 1;
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
            return 1*(delta);
        }
        else
        {
            return 1*(delta);
        }


    }
    else
    {
        finished=true;
        return 0;
    }

}

bool fpid::Controller::WriteLog()
{

}

bool fpid::Controller::Finished()
{
    return finished;
}
