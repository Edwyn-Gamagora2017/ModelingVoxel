#include "SurfaceControl.h"

#include <iostream>

SurfaceControl::SurfaceControl( std::deque< std::deque< Point3d > > controlPoints, Point3d * couleur, bool inverseNormal, bool doubleSense )
: Figure::Figure(new vec3(0,0,0),new vec3(0,0,0),new vec3(0,0,0),couleur,inverseNormal,doubleSense)
{
    this->controlPoints = controlPoints;

    this->generatePointsAndFaces();
}
SurfaceControl::~SurfaceControl(){}

void SurfaceControl::generatePointsAndFaces(){
    // points created by Casteljau's curve
    for( int i=0; i<this->controlPoints.size(); i++ ){
        int initialIndex = this->points.size();
        for( int j = 0; j < this->controlPoints[i].size(); j++ ){
            this->points.push_back( new Point3d( this->controlPoints[i][j].getX(), controlPoints[i][j].getY(), controlPoints[i][j].getZ(), initialIndex+j ) );
        }
    }

    // Faces
    for( int i=0; i<this->controlPoints.size()-1; i++ ){
        int nPointsPerCurve = this->controlPoints[i].size();
        int indexStartSup = i*nPointsPerCurve;
        int indexStartInf = (i+1)*nPointsPerCurve;
        int indexEndSup = indexStartInf;
        int indexEndInf = (i+2)*nPointsPerCurve;

        std::deque<Point3d*> pSup, pInf;
        pSup.insert( pSup.end(), this->points.begin()+indexStartSup, this->points.begin()+indexEndSup );
        pInf.insert( pInf.end(), this->points.begin()+indexStartInf, this->points.begin()+indexEndInf );

        std::deque<FigureFace*> fSection = FigureFace::SectionFigureFace( pSup, pInf, false, this->inverseNormal, this->doubleSense );
        this->faces.insert( this->faces.end(), fSection.begin(), fSection.end() );
    }

    // TODO SetFaces to generate Edges
    this->centralizeFigure();
	this->normalizeFigure();
}
