#include "Sphere.h"
#include <iostream>
#include <math.h>
#include "Circle.h"
#include "../utils.h"

/* au cas ou M_PI ne soit defini */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Sphere::Sphere( vec3 * center, double radius, int nMeridiens, int nParallels, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense )
: Figure::Figure(new vec3(0,0,0),new vec3(0,0,0),new vec3(0,0,0),couleur,inverseNormal,doubleSense)
{
    this->center = center;
    this->radius = radius;
    this->nMeridiens = nMeridiens;
    this->nParallels = nParallels;
    this->nCutMeridiens = nCutMeridiens;
    this->cutCenter = cutCenter;

    this->generatePointsAndFaces();
}

Sphere::Sphere( vec3 * rotation, vec3 * scale, vec3 * translation, int nMeridiens, int nParallels, int nCutMeridiens, bool cutCenter, point3d * couleur, bool inverseNormal, bool doubleSense )
: Sphere::Sphere(new vec3(0.,0.,0.),1.,nMeridiens,nParallels,nCutMeridiens,cutCenter,couleur,inverseNormal,doubleSense)
{
    this->rotation = rotation;
    this->scale = scale;
    this->translation = translation;
}

void Sphere::generatePointsAndFaces()
{
    // Points
    vec3 * couvercleSupCenter = new vec3( this->center->getX(), this->center->getY(), this->center->getZ()+this->radius );
    vec3 * couvercleInfCenter = new vec3( this->center->getX(), this->center->getY(), this->center->getZ()-this->radius );
    int meridienPoints;
    for( int i = 0; i < this->nParallels; i++ )
    {
        double phi = (i+1)*M_PI/(double)(this->nParallels+1);
        vec3 * couvercleCenter = new vec3( this->center->getX(), this->center->getY(), this->center->getZ()+((cos(phi))*this->radius) );
        Circle couvercle = Circle( couvercleCenter, sin( phi )*this->radius, this->nMeridiens, this->nCutMeridiens, this->cutCenter, this->couleur, this->inverseNormal, this->inverseNormal );
        std::deque<vec3*> pointsCouvercle = couvercle.getPoints();
        meridienPoints = pointsCouvercle.size();
        this->points.insert( this->points.end(), pointsCouvercle.begin(), pointsCouvercle.end() );
        /*for( int j = 0; j < this->nMeridiens; j++ )
        {
            double teta = j*2*M_PI/(double)this->nMeridiens;
            this->points.push_back(
                new vec3(
                            (cos( teta )*sin( phi )*this->radius)+this->center->getX(),
                            (sin( teta )*sin( phi )*this->radius)+this->center->getY(),
                            (cos(phi))*this->radius+this->center->getZ()
                            )
            );
        }*/
    }
    this->points.push_back( couvercleSupCenter );
    this->points.push_back( couvercleInfCenter );
	// Faces
	// Lateral
	for( int i = 0; i < this->nParallels; i++ )
	{
	    int indexStartSup = i*meridienPoints;//i*this->nMeridiens;
	    int indexStartInf = (i+1)*meridienPoints;//(i+1)*this->nMeridiens;
	    std::deque<vec3*> pSup, pInf;
	    pSup.insert( pSup.begin(), this->points.begin()+indexStartSup, this->points.begin()+indexStartSup+meridienPoints );
	    if( i < this->nParallels-1 )
	    {
            pInf.insert( pInf.begin(), this->points.begin()+indexStartInf, this->points.begin()+indexStartInf+meridienPoints );
            std::deque<FigureFace*> fSection = FigureFace::SectionFigureFace( pInf, pSup, true, this->inverseNormal, this->doubleSense );
            this->faces.insert( this->faces.end(), fSection.begin(), fSection.end() );
	    }
        //Couvercle
        if( i == 0 )
        {
            // Sup
            std::deque<FigureFace*> fCouvercle = FigureFace::CouvercleFigureFace( couvercleSupCenter, pSup, this->inverseNormal, this->doubleSense );
            this->faces.insert( this->faces.end(), fCouvercle.begin(), fCouvercle.end() );
        }
        if( i == this->nParallels-1 )
        {
            // Inf
            std::deque<FigureFace*> fCouvercle = FigureFace::CouvercleFigureFace( couvercleInfCenter, pSup, !this->inverseNormal, this->doubleSense );
            this->faces.insert( this->faces.end(), fCouvercle.begin(), fCouvercle.end() );
        }
	}
}

Sphere::~Sphere()
{

}
/*
#include "Sphere.h"
#include "math.h"

Sphere::Sphere(double rayon, int nParalleles, int nMeridiens, vec3* translation, point3d* couleur): figure::figure(new vec3(0.,0.,0.), new vec3(1.,1.,1.),translation,couleur)
{
    this->rayon = rayon;
    this->nParalleles = nParalleles;
    this->nMeridiens = nMeridiens;

    // points
    for( int i = 0; i < this->nParalleles; i++ ){
        double phi = (i+1)*M_PI/(double)(this->nParalleles+1);
        for( int j = 0; j < this->nMeridiens; j++ )
        {
            double teta = j*2*M_PI/(double)this->nMeridiens;
            this->points.push_back(new point3d(cos( teta )*sin( phi )*this->rayon, sin( teta )*sin( phi )*this->rayon, cos(phi)*this->rayon));
        }
    }
    this->points.push_back( new point3d(0,0,this->rayon) );     // cSup
	this->points.push_back( new point3d(0,0,-this->rayon) );  // cInf

	// faces
	for(int i=0; i<this->nParalleles-1; i++){
        for(int j=0; j<this->nMeridiens; j++){
            this->faces.push_back( FigureFace::TriangleFigureFace(
                                                              this->points[ i*this->nMeridiens+j ],                        // iSup
                                                              this->points[ (i+1)*this->nMeridiens+j ], // iInf
                                                              this->points[ i*this->nMeridiens+((j+1)%this->nMeridiens) ]  // iSup + 1
                                                              ) );
            this->faces.push_back( FigureFace::TriangleFigureFace(
                                                              this->points[ (i+1)*this->nMeridiens+j ],  // iInf
                                                              this->points[ (i+1)*this->nMeridiens+((j+1)%this->nMeridiens) ],  // iInf+1
                                                              this->points[ i*this->nMeridiens+((j+1)%this->nMeridiens) ]   // iSup+1
                                                              ) );
        }
    }
    //couvercles bouchons
    for(int i=0; i<this->nMeridiens; i++){
        // superieur
        this->faces.push_back( FigureFace::TriangleFigureFace(
                                                              this->points[ this->points.size()-2 ],   // cSup
                                                              this->points[ i ],                       // iSup
                                                              this->points[ (i+1)%this->nMeridiens ]   // iSup+1
                                                              ) );

        // inferieur
        this->faces.push_back( FigureFace::TriangleFigureFace(
                                                              this->points[ this->points.size()-1 ],     // cInf
                                                              this->points[ (this->nParalleles-1)*this->nMeridiens+(i+1)%this->nMeridiens ],  // iInf+1
                                                              this->points[ (this->nParalleles-1)*this->nMeridiens+i ]   // iInf
                                                              ) );
    }
}

Sphere::~Sphere()
{

}

std::deque<point3d> Sphere::getVertices()
{
    point3d cSup(0,0,this->rayon);
	point3d cInf(0,0,-this->rayon);

	std::deque<point3d> paralleles[ this->nParalleles ];
	for( int i = 0; i < this->nParalleles; i++ ){
        double phi = (i+1)*M_PI/(double)(this->nParalleles+1);
        for( int j = 0; j < this->nMeridiens; j++ )
        {
            double teta = j*2*M_PI/(double)this->nMeridiens;
            paralleles[i].push_back(point3d(cos( teta )*sin( phi )*this->rayon, sin( teta )*sin( phi )*this->rayon, cos(phi)*this->rayon));
        }
	}

	std::deque<point3d> res;
	// corps
    for(int i=0; i<this->nParalleles-1; i++){
        for(int j=0; j<this->nMeridiens; j++){
            res.push_back( paralleles[ i   ][ j ] );
            res.push_back( paralleles[ i   ][ (j+1)%this->nMeridiens ] );
            res.push_back( paralleles[ i+1 ][ j ] );
            res.push_back( paralleles[ i   ][ j ] );

            res.push_back( paralleles[ i+1 ][ j ] );
            res.push_back( paralleles[ i   ][ (j+1)%this->nMeridiens ] );
            res.push_back( paralleles[ i+1 ][ (j+1)%this->nMeridiens ] );
            res.push_back( paralleles[ i+1 ][ j ] );
        }
    }
    // Poles
    for(int i=0; i<this->nMeridiens; i++){
        // superieur
        res.push_back( cSup );
        res.push_back( paralleles[0][ (i+1)%this->nMeridiens ] );
        res.push_back( paralleles[0][ i ] );
        res.push_back( cSup );
        // inferieur
        res.push_back( cInf );
        res.push_back( paralleles[this->nParalleles-1][ i ] );
        res.push_back( paralleles[this->nParalleles-1][ (i+1)%this->nMeridiens ] );
        res.push_back( cInf );
    }
	return res;
}
*/
