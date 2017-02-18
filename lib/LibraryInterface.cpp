#include "LibraryInterface.h"

#include <iostream>


#include <plplot/plplot.h>
#include <plplot/plstream.h>
#define NSIZE    101

LibraryInterface::LibraryInterface()
{

}

int LibraryInterface::Plot(std::vector<double> datax, std::vector<double> datay)
{
  /*  PlotterParams newParams;
    newParams.setplparam("PAGESIZE", (char *)"a4");
    XPlotter plt(std::cin, std::cout, std::cerr, newParams);
    plt.space(0.0,0.0,1000.0,1000.0);
    plt.openpl();
    plt.pencolorname("red");
    plt.move(100,100);
    plt.endpath();
    plt.flushpl();
    for (ulong i=0; i<datax.size(); i++)
    {
        plt.point(datax[i],datay[i]);
        plt.move(datax[i],datay[i]);
    }

*/


        PLFLT x[NSIZE], y[NSIZE];
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

    p.env( xmin, xmax, ymin, ymax, 0, 0 );
  //  p.lab( "x", "y=100 x#u2#d", "Simple PLplot demo of a 2D line plot" );
   // p.line( NSIZE, x, y );
/*

    // Initialize plplot
    plinit();
    // Create a labelled box to hold the plot.
    plenv( xmin, xmax, ymin, ymax, 0, 0 );
    pllab( "x", "y=100 x#u2#d", "Simple PLplot demo of a 2D line plot" );

    // Plot the data that was prepared above.
    plline( NSIZE, x, y );

    // Close PLplot library
    plend();
*/

}
