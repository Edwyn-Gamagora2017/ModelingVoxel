#ifndef FIGUREFACE_H
#define FIGUREFACE_H

#include <deque>
#include "Point3d.h"
class Point3d;
class Edge;

class FigureFace
{
    std::deque<Point3d *> points;
    std::deque<Edge *> edges;
    vec3 normal;

    // Face's ID
	int index;

    public:
        FigureFace( std::deque<Point3d *> points, vec3 normal, int index = -1 );
        virtual ~FigureFace();

        static std::deque<FigureFace *> TriangleFigureFace( Point3d * pointA, Point3d * pointB, Point3d * pointC, bool inverseNormal, bool doubleSense, int index = -1 );
        static std::deque<FigureFace *> SquareFigureFace( Point3d * pointA, Point3d * pointB, Point3d * pointC, Point3d * pointD, bool inverseNormal, bool doubleSense, int index = -1 );
        static std::deque<FigureFace *> SectionFigureFace( std::deque<Point3d*> pointsA, std::deque<Point3d*> pointsB, bool closed, bool inverseNormal, bool doubleSense, int index = -1 );
        static std::deque<FigureFace *> CouvercleFigureFace( std::deque<Point3d*> points, bool inverseNormal, bool doubleSense, bool cutCenter, int index = -1 );  // the center is the last element in the list

        std::deque<Point3d *> getPoints();
        std::deque<Edge *> getEdges();
        int getEdgesCount();
        vec3 getNormal();
        int getIndex();

        // Edges
        bool hasEdge(Edge * e);
        void includeEdge(Edge * e);
        void removeEdge(Edge * e);

        // Points
        void removeVertex(Point3d * p);

    private:
        static FigureFace * TriangleFigureFaceBasic( Point3d * pointA, Point3d * pointB, Point3d * pointC, bool inverseNormal, int index = -1 );
};

#endif // FIGUREFACE_H
