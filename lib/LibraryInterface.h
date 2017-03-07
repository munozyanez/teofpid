#ifndef LIBRARYINTERFACE_H
#define LIBRARYINTERFACE_H

#include <vector>
#include "fcontrol/src/fcontrol.h"

#include <plotter.h>
#include <plot.h>

#include <fstream>      // std::fstream


//#include "mgl2/mgl.h"




class LibraryInterface
{
public:


    LibraryInterface();
    int Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley);


    long PlotAndSave(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley, std::string filename);
private:
    //PlotterParams puPlotParams;
};

#endif // LIBRARYINTERFACE_H
