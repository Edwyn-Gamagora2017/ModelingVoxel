#include "FigureFace.h"

#include <math.h>
#include <iostream>

FigureFace::FigureFace(std::deque<Point3d *> points, vec3 normal, int index )
{
    this->points = points;
    this->normal = normal;
    this->index = index;
    // Associate Face to Points
    for(int i=0; i<this->points.size(); i++){
        this->points[i]->associateTo( this );
    }
}

std::deque<Point3d*> FigureFace::getPoints()
{
    return this->points;
}
std::deque<Edge*> FigureFace::getEdges()
{
    return this->edges;
}
int FigureFace::getEdgesCount(){
    return this->edges.size();
}
vec3 FigureFace::getNormal(){
    return this->normal;
}
int FigureFace::getIndex(){
    return this->index;
}

FigureFace::~FigureFace()
{
    for( int i=0; i<this->edges.size(); i++ ){
        this->edges[i]->disassociateTo( this );
    }
    for( int i=0; i<this->points.size(); i++ ){
        this->points[i]->disassociateTo( this );
    }
}

 // Edges
bool FigureFace::hasEdge(Edge * e){
    for(int i=0; i<this->edges.size(); i++){
        if( this->edges[i]->getIndex() == e->getIndex() ){
            return true;
        }
    }
    return false;
}
void FigureFace::includeEdge(Edge * e){
    if( !this->hasEdge( e ) ){
        this->edges.push_back( e );
    }
}
void FigureFace::removeEdge(Edge * e){
    for(int i=0; i<this->edges.size(); i++){
        if( this->edges[i]->getIndex() == e->getIndex() ){
            this->edges.erase( this->edges.begin()+i );
            break;
        }
    }
}

// Vertex
void FigureFace::removeVertex(Point3d * p){
    for(int i=0; i<this->points.size(); i++){
        if( this->points[i]->getIndex() == p->getIndex() ){
            this->points.erase( this->points.begin()+i );
            break;
        }
    }
}

FigureFace* FigureFace::TriangleFigureFaceBasic(Point3d* pointA, Point3d* pointB, Point3d* pointC, bool inverseNormal, int index)
{
    /*
    C
    | \
    A--B
    */
    std::deque<Point3d *> pointsT;
    pointsT.push_back( pointA );
    if( !inverseNormal )
        { pointsT.push_back( pointB ); pointsT.push_back( pointC ); }
    else
        { pointsT.push_back( pointC ); pointsT.push_back( pointB ); }
    //pointsT.push_back( pointA );
    vec3 normalTriangle = pointB->vectorFrom(*pointA).produitVectoriel( pointC->vectorFrom(*pointA) ).normalized();

    return new FigureFace( pointsT, (inverseNormal?normalTriangle.negative():normalTriangle), index );
}

std::deque<FigureFace*> FigureFace::TriangleFigureFace(Point3d* pointA, Point3d* pointB, Point3d* pointC, bool inverseNormal, bool doubleSense, int index)
{
    std::deque<FigureFace*> result;
    result.push_back( FigureFace::TriangleFigureFaceBasic( pointA, pointB, pointC, inverseNormal, index ) );
    if(doubleSense) result.push_back( FigureFace::TriangleFigureFaceBasic( pointA, pointB, pointC, !inverseNormal, index+1 ) );
    return result;
}

std::deque<FigureFace*> FigureFace::SquareFigureFace( Point3d * pointA, Point3d * pointB, Point3d * pointC, Point3d * pointD, bool inverseNormal, bool doubleSense, int index )
{
    /*
    C--D
    |  |
    A--B
    */
    std::deque<FigureFace*> result;
    std::deque<FigureFace*> t1 = TriangleFigureFace( pointA, pointB, pointC, inverseNormal, doubleSense, index );
    std::deque<FigureFace*> t2 = TriangleFigureFace( pointC, pointB, pointD, inverseNormal, doubleSense, index );
    result.insert( result.end(), t1.begin(), t1.end() );
    result.insert( result.end(), t2.begin(), t2.end() );
    return result;
}

std::deque<FigureFace*> FigureFace::SectionFigureFace( std::deque<Point3d*> pointsA, std::deque<Point3d*> pointsB, bool closed, bool inverseNormal, bool doubleSense, int index )
{
    /*
    B1--B2
    |   |
    A1--A2
    */
    std::deque<FigureFace*> result;
    int amountPoints = std::min( pointsA.size(), pointsB.size() );
    for( int i = 0; i < amountPoints-(closed?0:1); i++ )
    {
        std::deque<FigureFace*> faces = SquareFigureFace( pointsA[i], pointsA[(i+1)%amountPoints], pointsB[i], pointsB[(i+1)%amountPoints], inverseNormal, doubleSense, index );
        result.insert( result.end(), faces.begin(), faces.end() );
    }
    return result;
}

std::deque<FigureFace*> FigureFace::CouvercleFigureFace( std::deque<Point3d*> points, bool inverseNormal, bool doubleSense, bool cutCenter, int index )
{
    /*
         P2
         |
    P3 - C - P1
         |
         P4
    */
    std::deque<FigureFace*> result;
    // ATTENTION
    Point3d * centerPoint = points[ points.size() ];
    for( int i = 0; i < points.size()-(cutCenter?0:1); i++ )
    {
        std::deque<FigureFace*> t = TriangleFigureFace( centerPoint, points[i], points[(i+1)%points.size()], inverseNormal, doubleSense, index );
        result.insert( result.end(), t.begin(), t.end() );
    }
    return result;
}
