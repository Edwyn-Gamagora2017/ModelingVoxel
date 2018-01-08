#ifndef SQUARE_H
#define SQUARE_H

#pragma once
#include "data_structure/Figure.h"

class Square
: public Figure
{
    protected:
        vec3 * center;
        double width;
        double height;

    public:
        Square( vec3 * center, double width, double height, Point3d * couleur, bool inverseNormal, bool doubleSense );
        Square( vec3 * rotation, vec3 * scale, vec3 * translation, Point3d * couleur, bool inverseNormal, bool doubleSense );
        ~Square();

    protected:
        virtual void generatePointsAndFaces();
};
#endif // SQUARE_H
