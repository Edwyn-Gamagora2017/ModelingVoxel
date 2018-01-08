#ifndef SURFACE_H
#define SURFACE_H

#include "data_structure/Figure.h"
#include "data_structure/Curve.h"
#include "SurfaceControl.h"
#include <deque>

typedef enum SurfaceType{ Regle, Parametrique, Directed };

class Surface :
    public Figure
{
    protected:
        std::deque< std::deque< Point3d > > controlPoints;
        SurfaceControl * surfaceControl;
        int amountSamples;
        SurfaceType surfaceType;

    public:
        Surface( std::deque< std::deque< Point3d > > controlPoints, SurfaceType surfaceType, int amountSamples, Point3d * couleur, bool inverseNormal, bool doubleSense );
        ~Surface();

        SurfaceControl * getSurfaceControl();

        static Surface * exampleCasteljau();
        static Surface * exampleRegle();
        static Surface * exampleDirected();

    protected:
        virtual void generatePointsAndFaces();
        void generatePointsRegle();
        void generateCarreParametrique();
        void generateDirectedSurface();
};

#endif // SURFACE_H
