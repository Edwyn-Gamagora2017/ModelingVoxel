#include "Triangle.h"

Triangle::Triangle( vec3 * pivot, double width, double height, point3d * couleur, bool inverseNormal, bool doubleSense )
: Figure::Figure(new vec3(0,0,0),new vec3(0,0,0),new vec3(0,0,0),couleur,inverseNormal,doubleSense)
{
    this->width = width;
    this->height = height;
    this->pivot = pivot;

    this->generatePointsAndFaces();
}

Triangle::Triangle( vec3 * rotation, vec3 * scale, vec3 * translation, point3d * couleur, bool inverseNormal, bool doubleSense )
: Triangle::Triangle(new vec3(0.,0.,0.),1.,1.,couleur,inverseNormal,doubleSense)
{
    this->rotation = rotation;
    this->scale = scale;
    this->translation = translation;
}

void Triangle::generatePointsAndFaces()
{
    // Points
    /*
    3
    1 2
    */
    this->points.push_back( new Point( this->pivot, 0) );
    this->points.push_back( new Point( new vec3( this->pivot->getX()+this->width,this->pivot->getY(),this->pivot->getZ()), 1 ));
    this->points.push_back( new Point( new vec3( this->pivot->getX(),this->pivot->getY()+this->height,this->pivot->getZ()), 2 ));
    // Faces
    std::deque<FigureFace*> t = FigureFace::TriangleFigureFace( this->points[0], this->points[1], this->points[2], this->inverseNormal, this->doubleSense );
    this->faces.insert( this->faces.end(), t.begin(), t.end() );
}

Triangle::~Triangle()
{}
