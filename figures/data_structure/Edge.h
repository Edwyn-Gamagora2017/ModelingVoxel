#ifndef EDGE_H
#define EDGE_H

#include "Point3d.h"
class Point3d;
#include "FigureFace.h"
#include <deque>

class Edge
{
    protected:
    // Points that compose the edge
	Point3d * pointA;
	Point3d * pointB;
    // Faces that contain the edge
	std::deque<FigureFace *> faces;

	// Edge's ID
	int index;

    public:
        /** \brief Creates an Edge. Adds it to both points
         *
         * \param _pointA Point3d* first point that composes the edge
         * \param _pointB Point3d* second point that composes the edge
         * \param _index int edge's ID
         *
         */
        Edge( Point3d * _pointA, Point3d * _pointB, int _index );
        virtual ~Edge();

        bool hasPoint( Point3d * p );
        bool hasPoints( Point3d * p1, Point3d * p2 );
        // Faces
        bool isAssociatedTo(FigureFace * f);
        void associateTo(FigureFace * f);
        void disassociateTo(FigureFace * f);

        /* GETTERS */
        Point3d * getPointA();
        Point3d * getPointB();
        std::deque<FigureFace *> getFaces();
        int getFacesCount();
        int getIndex();
};

#endif // EDGE_H
