#ifndef CURVE_H
#define CURVE_H

#include "vec3.h"
#include "Point3d.h"
#include <deque>

class Curve
{
    public:
        // calculate the Bezier curve based on Casteljau algorithm. amountSamples defines the amount of samples in the curve
        static std::deque<vec3> casteljau( std::deque<Point3d> controlPoints, int amountSamples );

    private:
        // calculate the intermediate k position on the Bezier curve (Casteljau) related to the factor u [0,1]
        static vec3 casteljauP( double u, int k, int i, std::deque<Point3d> controlPoints );
};

#endif // CURVE_H
