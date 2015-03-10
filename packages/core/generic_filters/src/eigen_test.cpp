#include "movingaverage.h"
#include "exponentialsmoother.h"
#include "eigen3/Eigen/Dense"


#include <iostream>

typedef Eigen::Matrix<float,Eigen::Dynamic,1> Vec;
typedef Eigen::Matrix<float,5,1> Vec5;

int main(int argc, char *argv[])
{
    std::cout<<"hello \n";
    const int D = 2;
    Vec v(D);
    v(1)=1;

    MovingAverage<Vec> mv(4);

    mv.filter(v);
    v(1)=0;
    std::cout<<mv.filter(v)<<std::endl;
    //mv.filter(v);

    ExponentialSmoother<Vec> esmooth(0.9,v);
    v(1) = 1;
    std::cout<<esmooth.filter(v)<<std::endl;
    v(1) = 2;
    std::cout<<esmooth.filter(v)<<std::endl;
    v(1) = 2;


    const Vec& b = esmooth.filter(v);;
    Vec bb = esmooth.filter(v);
    std::cout<<b<<std::endl;
    //b(0) = 2.0;

    ApproximateMovingAverage<Vec> amv(4);
    std::cout<<amv.filter(v)<<std::endl;
    std::cout<<amv.filter(v)<<std::endl;

    return 0;
}

