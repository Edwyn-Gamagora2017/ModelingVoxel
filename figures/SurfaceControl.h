#ifndef SURFACECONTROL_H
#define SURFACECONTROL_H

#include "data_structure/Figure.h"
#include "data_structure/Curve.h"
#include <deque>

class SurfaceControl :
    public Figure
{
    protected:
        std::deque< std::deque< Point3d > > controlPoints;

    public:
        SurfaceControl( std::deque< std::deque< Point3d > > controlPoints, Point3d * couleur, bool inverseNormal, bool doubleSense );
        ~SurfaceControl();

    protected:
        virtual void generatePointsAndFaces();
};

#endif // SURFACECONTROL_H
