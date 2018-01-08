#include "Surface.h"

#include <iostream>

Surface::Surface( std::deque< std::deque< Point3d > > controlPoints, SurfaceType surfaceType, int amountSamples, Point3d * couleur, bool inverseNormal, bool doubleSense )
: Figure::Figure(new vec3(0,0,0),new vec3(0,0,0),new vec3(0,0,0),couleur,inverseNormal,doubleSense)
{
    this->controlPoints = controlPoints;
    this->surfaceType = surfaceType;
    this->amountSamples = amountSamples;

    if( surfaceType != Directed ){
        this->surfaceControl = new SurfaceControl( controlPoints, couleur, inverseNormal, doubleSense );
    }

    this->generatePointsAndFaces();
}
Surface::~Surface(){
    delete( this->surfaceControl );
}

SurfaceControl * Surface::getSurfaceControl(){
    return this->surfaceControl;
}

void Surface::generateCarreParametrique(){
    std::deque< std::deque< Point3d > > guidelines; // Bezier curves

    // Obtaining guidelines based on control points
    // Guidelines created by Casteljau's curve
    for( int i=0; i<this->controlPoints.size(); i++ ){
        std::deque< Point3d > guidelineCurve;

        // Calculating curve and storing in guidelines
        std::deque< vec3 > curve = Curve::casteljau( this->controlPoints[i], this->amountSamples );
        for( int j = 0; j < curve.size(); j++ ){
            guidelineCurve.push_back( Point3d( curve[j].getX(), curve[j].getY(), curve[j].getZ(), 0 ) );
        }

        guidelines.push_back( guidelineCurve );
    }

    // Obtaining resulting curves based on guidelines
    // curves created by Casteljau's curve
    int nPointsPerGuideline = guidelines[0].size();
    for( int i=0; i<nPointsPerGuideline; i++ ){
        // Calculating curve and storing in guidelines
        int initialIndex = this->points.size();
        std::deque< Point3d > guidelinePoints;
        for( int j = 0; j < guidelines.size(); j++){
            guidelinePoints.push_back( guidelines[j][i] );
        }
        std::deque< vec3 > curve = Curve::casteljau( guidelinePoints, this->amountSamples );
        for( int j = 0; j < curve.size(); j++ ){
            this->points.push_back( new Point3d( curve[j].getX(), curve[j].getY(), curve[j].getZ(), initialIndex+j ) );
        }
    }

    // Faces
    int nPointsPerCurve = this->amountSamples+2;
    for( int i=0; i<nPointsPerCurve-1; i++ ){
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
}

void Surface::generatePointsRegle(){
    // points created by Casteljau's curve
    for( int i=0; i<this->controlPoints.size(); i++ ){
        int initialIndex = this->points.size();
        std::deque< vec3 > curve = Curve::casteljau( this->controlPoints[i], this->amountSamples );
        for( int j = 0; j < curve.size(); j++ ){
            this->points.push_back( new Point3d( curve[j].getX(), curve[j].getY(), curve[j].getZ(), initialIndex+j ) );
        }
    }

    // Faces
    for( int i=0; i<this->controlPoints.size()-1; i++ ){
        int nPointsPerCurve = this->amountSamples+2;
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
}

void Surface::generateDirectedSurface(){
    /*
        controlPoints[0] : generating curve's control points
        controlPoints[1] : direction curve's control points
    */

    std::deque< Point3d > generatingCurve;  // Bezier's curve
    std::deque< Point3d > directionCurve;   // Bezier's curve

    // Calculate Generating Curve and Direction Curve
    {
        std::deque< vec3 > aux_genCurve = Curve::casteljau( this->controlPoints[0], this->amountSamples );
        for( int j = 0; j < aux_genCurve.size(); j++ ){
            generatingCurve.push_back( Point3d( aux_genCurve[j].getX(), aux_genCurve[j].getY(), aux_genCurve[j].getZ(), 0 ) );
        }
    }
    {
        std::deque< vec3 > aux_dirCurve = Curve::casteljau( this->controlPoints[1], this->amountSamples );
        for( int j = 0; j < aux_dirCurve.size(); j++ ){
            directionCurve.push_back( Point3d( aux_dirCurve[j].getX(), aux_dirCurve[j].getY(), aux_dirCurve[j].getZ(), 0 ) );
        }
    }

    // Calculate Points : a copy of GENERATING CURVE is translated based on DIRECTION CURVE
    for( int i = 0; i < directionCurve.size(); i++ ){
        for( int j = 0; j < generatingCurve.size(); j++ ){
            vec3 translatedPoint = generatingCurve[j].toVector().addition( directionCurve[i].toVector() );
            this->points.push_back( new Point3d( translatedPoint.getX(), translatedPoint.getY(), translatedPoint.getZ(), this->points.size() ) );
        }
    }

    // Faces
    int nPointsPerCurve = directionCurve.size();
    for( int i=0; i<nPointsPerCurve-1; i++ ){
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
}

void Surface::generatePointsAndFaces(){
    switch( this->surfaceType ){
    case Regle:
        this->generatePointsRegle();
        break;
    case Directed:
        this->generateDirectedSurface();
        break;
    case Parametrique: default:
        this->generateCarreParametrique();
        break;
    }
    //this->generateSurface();

    // TODO SetFaces to generate Edges
    this->centralizeFigure();
	this->normalizeFigure();
}

Surface * Surface::exampleCasteljau(){
    std::deque< std::deque< Point3d > > controlPoints;
    std::deque< Point3d > controlPointsCurve;

    controlPointsCurve.push_back( Point3d( 0,0,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,1,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,2,0, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    controlPointsCurve.clear();
    controlPointsCurve.push_back( Point3d( 1,0,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 1,1,1, 0 ) );
    controlPointsCurve.push_back( Point3d( 1,2,1, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    controlPointsCurve.clear();
    controlPointsCurve.push_back( Point3d( 2,0,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 2,1,1, 0 ) );
    controlPointsCurve.push_back( Point3d( 2,2,1, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    controlPointsCurve.clear();
    controlPointsCurve.push_back( Point3d( 3,0,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 3,1,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 3,2,0, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    return new Surface( controlPoints, Parametrique, 5, new Point3d(1,1,0, 0), false, true );
}

Surface * Surface::exampleRegle(){
    std::deque< std::deque< Point3d > > controlPoints;
    std::deque< Point3d > controlPointsCurve;

    controlPointsCurve.push_back( Point3d( 0,0,1, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,1,1, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,2,0, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    controlPointsCurve.clear();
    controlPointsCurve.push_back( Point3d( 1,0,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 1,1,1, 0 ) );
    controlPointsCurve.push_back( Point3d( 1,2,2, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    return new Surface( controlPoints, Parametrique, 5, new Point3d(1,1,0, 0), false, true );
}

Surface * Surface::exampleDirected(){
    std::deque< std::deque< Point3d > > controlPoints;
    std::deque< Point3d > controlPointsCurve;

    controlPointsCurve.push_back( Point3d( -1,-1,-1, 0 ) );
    controlPointsCurve.push_back( Point3d( -1,1,-1, 0 ) );
    controlPointsCurve.push_back( Point3d( 1,1,-1, 0 ) );
    controlPointsCurve.push_back( Point3d( 1,-1,-1, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    controlPointsCurve.clear();
    controlPointsCurve.push_back( Point3d( 0,0,0, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,1,1, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,1,2, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,0,3, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,-1,4, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,-2,5, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,-1,6, 0 ) );
    controlPointsCurve.push_back( Point3d( 0,0,7, 0 ) );
    controlPoints.push_back( controlPointsCurve );

    return new Surface( controlPoints, Directed, 15, new Point3d(1,1,0, 0), false, true );
}
