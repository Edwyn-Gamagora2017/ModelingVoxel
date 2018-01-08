#ifndef TRIANGLE_H
#define TRIANGLE_H

#pragma once
#include "Figure.h"

class Triangle
: public Figure
{
    protected:
        vec3 * pivot;
        double width;
        double height;

    public:
        Triangle( vec3 * pivot, double width, double height, point3d * couleur, bool inverseNormal, bool doubleSense );
        Triangle( vec3 * rotation, vec3 * scale, vec3 * translation, point3d * couleur, bool inverseNormal, bool doubleSense );
        ~Triangle();

    protected:
        virtual void generatePointsAndFaces();
};
#endif // TRIANGLE_H
