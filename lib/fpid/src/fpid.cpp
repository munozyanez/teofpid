#include "fpid.h"


fpid::Controller::Controller()
{
    finished=true;
    maxRepeat = 30;
    range = 3;
}

bool fpid::Controller::SetTarget(double newTarget)
{
    finished = false;
    target = newTarget;
    return true;
}

double fpid::Controller::ControlSignal(double error)
{
    //double delta = target-error;
    if (repeat < maxRepeat)
    {
        if (std::abs(error) < range)
        {
            repeat++;
            std::cout << repeat <<"," <<error << "," << range << std::endl;
            return 1*(error);
        }
        else
        {
            return 1*(error);
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
