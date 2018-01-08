#include "Cone.h"

#include "../utils.h"
#include "Circle.h"
#include "math.h"
#include <iostream>

Cone::Cone( vec3 * center, double radius, double height, double cut, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense )
: Figure::Figure(new vec3(0,0,0),new vec3(0,0,0),new vec3(0,0,0),couleur,inverseNormal,doubleSense)
{
    this->center = center;
    this->radius = radius;
    this->height = height;
    this->cut = clamp( cut, 0., 1. );
    this->nMeridiens = nMeridiens;
    this->nCutMeridiens = nCutMeridiens;
    this->cutCenter = cutCenter;
    this->istube = istube;

    this->generatePointsAndFaces();
}

Cone::Cone( vec3 * rotation, vec3 * scale, vec3 * translation, double cut, int nMeridiens, bool istube, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense )
: Cone::Cone(new vec3(0.,0.,0.),1.,1.,cut,nMeridiens,istube,nCutMeridiens,cutCenter,couleur,inverseNormal,doubleSense)
{
    this->rotation = rotation;
    this->scale = scale;
    this->translation = translation;
}

void Cone::generatePointsAndFaces()
{
    bool isCut = !equalDouble( this->cut, 1. );
    double radiusCut = this->radius*(1-this->cut);
    double cutHeight = this->height*this->cut;

    vec3 * couvercleSupCenter = new vec3( this->center->getX(), this->center->getY(), this->center->getZ()+cutHeight/2. );
    vec3 * couvercleInfCenter = new vec3( this->center->getX(), this->center->getY(), this->center->getZ()-cutHeight/2. );

    // Points Circle inf
    Circle couvercleInf = Circle( couvercleInfCenter, this->radius, this->nMeridiens, this->nCutMeridiens, this->cutCenter, false, this->couleur, !this->inverseNormal, this->istube );
    std::deque<vec3*> pInf = couvercleInf.getPoints();
    this->points.insert( this->points.end(), pInf.begin(), pInf.end() );

    if( isCut )
    {
        // Points Circle sup
        Circle couvercleSup = Circle( couvercleSupCenter, radiusCut, this->nMeridiens, this->nCutMeridiens, this->cutCenter, false, this->couleur, this->inverseNormal, this->istube );
        std::deque<vec3*> pSup = couvercleSup.getPoints();
        this->points.insert( this->points.end(), pSup.begin(), pSup.end() );

        // Faces - Lateral and Sup
        if( !this->istube ){
            // Sup
            std::deque<FigureFace*> fSup = couvercleSup.calculateFaces();
            this->faces.insert( this->faces.end(), fSup.begin(), fSup.end() );
            // Points center couvercles
            this->points.push_back( couvercleSup.getPoints()[couvercleSup.getPoints().size()-1] );
        }
        // Lateral
        std::deque<FigureFace*> fSection = FigureFace::SectionFigureFace( pInf, pSup, true, this->inverseNormal, this->istube );
        this->faces.insert( this->faces.end(), fSection.begin(), fSection.end() );
    }
    else{
        // Faces - Sup
        pInf.push_back( new Point( couvercleSupCenter, 0 ) );

        std::deque<FigureFace*> fSup = FigureFace::CouvercleFigureFace( pInf, this->inverseNormal, this->istube );
        this->faces.insert( this->faces.end(), fSup.begin(), fSup.end() );
        // Points center couvercles
        this->points.push_back( couvercleSup.getPoints()[couvercleSup.getPoints().size()-1] );
    }
    // Points
    if( !this->istube || !isCut ) this->points.push_back( couvercleInfCenter );
    if( !this->istube ) this->points.push_back( couvercleInfCenter );

    // Faces - Inf
    if( !this->istube ){
        std::deque<FigureFace*> fInf = couvercleInf.getFaces();
        this->faces.insert( this->faces.end(), fInf.begin(), fInf.end() );
    }
}

Cone::~Cone()
{

}
