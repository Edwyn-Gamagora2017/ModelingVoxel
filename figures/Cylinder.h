#ifndef CYLINDER_H
#define CYLINDER_H

#include "Figure.h"

class Cylinder : public Figure
{
    protected:
        double radius;
        double height;
        int nMeridiens;
        int nCutMeridiens;
        bool cutCenter;
        vec3 * center;
        bool istube;

    public:
        Cylinder( vec3 * center, double radius, double height, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense );
        Cylinder( vec3 * rotation, vec3 * scale, vec3 * translation, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense );
        ~Cylinder();

    protected:
        virtual void generatePointsAndFaces();
};

#endif
