#ifndef CONE_H
#define CONE_H

#include "Figure.h"

class Cone : public Figure
{
    protected:
        double radius;
        double height;
        double cut;     // ]0,1] : 11 means that the entire object must be shown
        int nMeridiens;
        int nCutMeridiens;
        bool cutCenter;
        vec3 * center;
        bool istube;

    public:
        Cone( vec3 * center, double radius, double height, double cut, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense );
        Cone( vec3 * rotation, vec3 * scale, vec3 * translation, double cut, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense );
        ~Cone();

    protected:
        virtual void generatePointsAndFaces();
};

#endif // CONE_H
