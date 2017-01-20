#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Tests from MatPlotLibCpp

void testMinimal() {
    plt::clf();
    plt::plot({1,2,3,4});
    plt::save("minimal.png");
}

void testBasic() {
    plt::clf();
    // Prepare data.
    int n = 5000;
    std::vector<double> x(n), y(n), z(n), w(n,2);
    for(int i=0; i<n; ++i) {
        x.at(i) = i*i;
        y.at(i) = sin(2*M_PI*i/360.0);
        z.at(i) = log(i);
    }

    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);
    // Plot a red dashed line from given x and y data.
    plt::plot(x, w,"r--");
    // Plot a line whose name will show up as "log(x)" in the legend.
    plt::named_plot("log(x)", x, z);

    // Set x-axis to interval [0,1000000]
    plt::xlim(0, 1000*1000);

    // Add graph title
    plt::title("Sample figure");
    // Enable legend.
    plt::legend();
    // save figure
    plt::save("basic.png");
}

void testModern() {
    // plot(y) - the x-coordinates are implicitly set to [0,1,...,n)
    //plt::plot({1,2,3,4});

    // Prepare data for parametric plot.
    int n = 5000; // number of data points
    std::vector<double> x(n),y(n);
    for(int i=0; i<n; ++i) {
        double t = 2*M_PI*i/n;
        x.at(i) = 16*sin(t)*sin(t)*sin(t);
        y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
    }

    // plot() takes an arbitrary number of (x,y,format)-triples.
    // x must be iterable (that is, anything providing begin(x) and end(x)),
    // y must either be callable (providing operator() const) or iterable.
    plt::plot(x, y, "r-", x, [](double d) { return 12.5+abs(sin(d)); }, "k-");

    plt::save("modern.png");
}

int main(int argc, char** argv) {
    testMinimal();
    testBasic();
    testModern();
}