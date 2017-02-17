#include "LibraryInterface.h"

#include <iostream>

LibraryInterface::LibraryInterface()
{

}

int LibraryInterface::Plot(std::vector<double> datax, std::vector<double> datay)
{
    PlotterParams newParams;
    newParams.setplparam("PAGESIZE", (char *)"letter");
    XPlotter newPlot(std::cin, std::cout, std::cerr, newParams);
    newPlot.openpl();
    //newPlot.erase();
    getchar();

    newPlot.closepl();


}
