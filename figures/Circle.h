#ifndef CIRCLE_H
#define CIRCLE_H

#pragma once
#include "Figure.h"

class Circle : public Figure
{
    protected:
        vec3 * center;
        double radius;
        int nSections;
        int nCutSections;       // Number of sections to be not generated
        bool cutCenter;         // The cut must be a section until the center of the circle (true), or just a line between the last section and the first
        static const int minSections = 3;
        bool generateFaces;

    public:
        Circle( vec3 * center, double radius, int nSections, int nCutSections, bool cutCenter, bool generateFaces, point3d * couleur, bool inverseNormal, bool doubleSense );
        Circle( vec3 * rotation, vec3 * scale, vec3 * translation, int nSections, int nCutSections, bool cutCenter, bool generateFaces, point3d * couleur, bool inverseNormal, bool doubleSense );
        ~Circle();

        std::deque<FigureFace*> calculateFaces();

    protected:
        virtual void generatePointsAndFaces();
};
#endif // CIRCLE_H
