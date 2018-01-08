#include <math.h>
#include <deque>

#include "Circle.h"

/* au cas ou M_PI ne soit defini */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Circle::Circle( vec3 * center, double radius, int nSections, int nCutSections, bool cutCenter, bool generateFaces, point3d * couleur, bool inverseNormal, bool doubleSense )
: Figure::Figure( new vec3(0,0,0), new vec3(0,0,0), new vec3(0,0,0), couleur, inverseNormal, doubleSense )
{
    this->center = center;
    this->radius = radius;
    this->nSections = std::max( nSections, Circle::minSections );
    this->nCutSections = std::min( nCutSections, this->nSections-1 );
    if(this->nCutSections > this->nSections/2) this->cutCenter = true; else this->cutCenter = cutCenter;
    this->generateFaces = generateFaces;

    this->generatePointsAndFaces();
}

Circle::Circle( vec3 * rotation, vec3 * scale, vec3 * translation, int nSections, int nCutSections, bool cutCenter, bool generateFaces, point3d * couleur, bool inverseNormal, bool doubleSense )
: Circle::Circle(new vec3(0.,0.,0.),.5,360,0,false,generateFaces,couleur,inverseNormal,doubleSense)
{
    this->rotation = rotation;
    this->scale = scale;
    this->translation = translation;
}

void Circle::generatePointsAndFaces()
{
    // Points
    for(int i=0; i<this->nSections-(std::max(this->nCutSections-1,0)); i++){
        double angle = i * 2*M_PI/this->nSections;
        this->points.push_back( new Point( new vec3((cos( angle )*this->radius)+this->center->getX(), (sin( angle )*this->radius)+this->center->getY(), this->center->getZ()), i) );
	}
	if( this->nCutSections > 0 && this->cutCenter ) this->points.push_back( new Point( this->center, this->points.size() ) );

    // Faces
    if( this->generateFaces )
    {
        this->faces = calculateFaces();
    }
}

std::deque<FigureFace*> Circle::calculateFaces()
{
    if( !(this->nCutSections > 0 && this->cutCenter) ) this->points.push_back( new Point( this->center, this->points.size() ) );
    std::deque<FigureFace*> f = FigureFace::CouvercleFigureFace( this->points, this->inverseNormal, this->doubleSense, (this->nCutSections > 0 && this->cutCenter) );
    return f;
}

Circle::~Circle()
{}
