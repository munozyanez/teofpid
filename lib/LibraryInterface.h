#ifndef LIBRARYINTERFACE_H
#define LIBRARYINTERFACE_H

#include <vector>


#include <plotter.h>
#include <plot.h>

//#include "mgl2/mgl.h"




class LibraryInterface
{
public:


    LibraryInterface();
    int Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley);


private:
    //PlotterParams puPlotParams;
};

#endif // LIBRARYINTERFACE_H
