#ifndef SPHERE_H
#define SPHERE_H

#include "Figure.h"

class Sphere : public Figure
{
    protected:
        double radius;
        int nMeridiens;
        int nParallels;
        int nCutMeridiens;
        bool cutCenter;
        vec3 * center;

    public:
        Sphere( vec3 * center, double radius, int nMeridiens, int nParallels, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense );
        Sphere( vec3 * rotation, vec3 * scale, vec3 * translation, int nMeridiens, int nParallels, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense );
        ~Sphere();

    protected:
        virtual void generatePointsAndFaces();
};

#endif // SPHERE_H
