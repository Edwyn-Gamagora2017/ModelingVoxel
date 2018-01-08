#include "Cylinder.h"
#include "math.h"
#include <iostream>
#include "Circle.h"
#include "../utils.h"

Cylinder::Cylinder(vec3* center, double radius, double height, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d* couleur, bool inverseNormal, bool doubleSense )
: Figure::Figure(new vec3(0,0,0),new vec3(0,0,0),new vec3(0,0,0),couleur,inverseNormal,doubleSense)
{
    this->center = center;
    this->radius = radius;
    this->height = height;
    this->nMeridiens = nMeridiens;
    this->istube = istube;
    this->nCutMeridiens = nCutMeridiens;
    this->cutCenter = cutCenter;
    //this->points = new point3d*[nMeridiens*2+2];
    //this->faces = new point3d*[nMeridiens*4][4];
    this->generatePointsAndFaces();
}

Cylinder::Cylinder( vec3 * rotation, vec3 * scale, vec3 * translation, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense )
: Cylinder::Cylinder(new vec3(0.,0.,0.),1.,1.,nMeridiens,istube,nCutMeridiens,cutCenter,couleur,inverseNormal,doubleSense)
{
    this->rotation = rotation;
    this->scale = scale;
    this->translation = translation;
}

void Cylinder::generatePointsAndFaces()
{
    // Points
    vec3 * couvercleSupCenter = new vec3( this->center->getX(), this->center->getY(), this->center->getZ()+this->height/2. );
    vec3 * couvercleInfCenter = new vec3( this->center->getX(), this->center->getY(), this->center->getZ()-this->height/2. );
    // creating circles without faces
    Circle couvercleSup = Circle( couvercleSupCenter, this->radius, this->nMeridiens, this->nCutMeridiens, this->cutCenter, false, this->couleur, this->inverseNormal, this->istube );
    Circle couvercleInf = Circle( couvercleInfCenter, this->radius, this->nMeridiens, this->nCutMeridiens, this->cutCenter, false, this->couleur, !this->inverseNormal, this->istube );
    // Points cylinder
    std::deque<vec3*> pSup = couvercleSup.getPoints();
    std::deque<vec3*> pInf = couvercleInf.getPoints();
    this->points.insert( this->points.end(), pSup.begin(), pSup.end() );
    this->points.insert( this->points.end(), pInf.begin(), pInf.end() );

	// Faces
    // Lateral faces
    std::deque<FigureFace*> fSection = FigureFace::SectionFigureFace( pInf, pSup, true, this->inverseNormal, this->istube );
    this->faces.insert( this->faces.end(), fSection.begin(), fSection.end() );
	// Couvercles
	if(!this->istube)
    {
        std::deque<FigureFace*> fSup = couvercleSup.calculateFaces();
        std::deque<FigureFace*> fInf = couvercleInf.calculateFaces();
        this->faces.insert( this->faces.end(), fSup.begin(), fSup.end() );
        this->faces.insert( this->faces.end(), fInf.begin(), fInf.end() );
        // Points center couvercles
        this->points.push_back( couvercleSup.getPoints()[couvercleSup.getPoints().size()-1] );
        this->points.push_back( couvercleInf.getPoints()[couvercleInf.getPoints().size()-1] );
    }
    // updating indexes
    FOR(i,this->points.size())
    {
        this->points[i]->setIndex(i);
    }
}

Cylinder::~Cylinder()
{

}
