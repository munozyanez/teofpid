#include "LibraryInterface.h"

#include <iostream>


#include <plplot/plplot.h>
#include <plplot/plstream.h>
#define NSIZE    101

LibraryInterface::LibraryInterface()
{

}

int LibraryInterface::Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley)
{
    PlotterParams newParams;
    newParams.setplparam("PAGESIZE", (char *)"a4");
    newParams.setplparam("BITMAPSIZE", (char *)"600x600");

    XPlotter plt(newParams);
    plt.fspace(0.0, 0.0, 10.0, 10.0);
    plt.fscale(2,2);
    plt.openpl();
    plt.pencolorname("red");
    plt.flinewidth (0);       // line thickness in user coordinates
    plt.alabel(0,0,"cosa");
    plt.fmove(0.1,0.1);
    plt.fcircle(0.3,0.2,0.5);
    //plt.fline(0,0,0.5,0.5);

    plt.endpath();
    plt.flushpl();

    for (ulong i=0; i<datax.size(); i++)
    {
        plt.fpoint(datax[i]/scalex,datay[i]/scaley);
        //plt.fmove(datax[i],datay[i]);
        //plt.endpath();
        //plt.flushpl();

    }

    plt.endpath();
    plt.flushpl();

/*        PLFLT x[NSIZE], y[NSIZE];
    PLFLT xmin = 0., xmax = 100., ymin = 0., ymax = 100.;
    int   i;

    // Prepare data to be plotted.
    for ( i = 0; i < NSIZE; i++ )
    {
        x[i] = (PLFLT) ( i ) / (PLFLT) ( NSIZE - 1 );
        y[i] = ymax * x[i] * x[i];
    }


    plstream p( (PLINT)100, (PLINT)100,"qtwidget");//qtwidget or wxwidgets

    p.init();
   // p.start( "xfig",(PLINT)100, (PLINT)100);

    //p.adv();
    p.env( xmin, xmax, ymin, ymax, 0, 0 );
    p.vpor(xmin, xmax, ymin, ymax);

  //  p.lab( "x", "y=100 x#u2#d", "Simple PLplot demo of a 2D line plot" );
   // p.line( NSIZE, x, y );


*/

}
