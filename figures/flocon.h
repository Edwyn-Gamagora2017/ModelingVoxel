#ifndef FLOCON_H
#define FLOCON_H

#include "figure.h"
#include "vec3.h"

class flocon: public figure
{
    vec3 * a;
    vec3 * b;
    vec3 * c;
    public:
        flocon( vec3 * a, vec3 * b, vec3 * c, point3d * couleur );
        ~flocon();

        vec3 * getA();
        vec3 * getB();
        vec3 * getC();

        // Hérité via figure
        virtual std::deque<point3d> getVertices();
};

#endif // FLOCON_H
