#include "Figure.h"
#include <iostream>
#include <math.h>

#include "CubeVolume.h"

Figure::Figure( vec3 * rotation, vec3 * scale, vec3 * translation, Point3d * couleur, bool inverseNormal, bool doubleSense )
:    VolumeForm::VolumeForm( vec3() )
{
    this->setRotation( rotation );
    this->setScale( scale );
    this->setTranslation( translation );
    this->setCouleur( couleur );
    this->setInverseNormal( inverseNormal );
    this->setDoubleSense( doubleSense );
}
Figure::~Figure(){
    /*for( int i = 0; i < this->points.size(); i++ )
    {
        if( this->points[i] ) delete this->points[i];
    }*/
}

void Figure::setRotation(vec3 * rotation){
    this->rotation = rotation;
}
void Figure::setScale(vec3 * scale){
    this->scale = scale;
}
void Figure::setTranslation(vec3 * translation){
    this->translation = translation;
}
void Figure::setCouleur(Point3d * couleur){
    this->couleur = couleur;
}
void Figure::setInverseNormal(bool inverseNormal){
    this->inverseNormal = inverseNormal;
}
void Figure::setDoubleSense(bool doubleSense){
    this->doubleSense = doubleSense;
}

vec3 * Figure::getRotation(){
    return this->rotation;
}
vec3 * Figure::getScale(){
    return this->scale;
}
vec3 * Figure::getTranslation(){
    return this->translation;
}
Point3d * Figure::getCouleur(){
    return this->couleur;
}
bool Figure::getInverseNormal(){
    return this->inverseNormal;
}
bool Figure::getDoubleSense(){
    return this->doubleSense;
}

std::deque<Point3d*> Figure::getPoints()
{
    return this->points;
}
std::deque<Edge*> Figure::getEdges()
{
    return this->edges;
}
std::deque<FigureFace*> Figure::getFaces()
{
    return this->faces;
}

Edge * Figure::hasEdge( Point3d * pA, Point3d * pB ){
    for(int i=0; i<this->edges.size(); i++){
        if( this->edges[i]->hasPoints( pA, pB ) ){
            return this->edges[i];
        }
    }
    return NULL;
}
void Figure::setPoints(std::deque< Point3d* > points)
{
    this->points = points;
}
void Figure::setFaces( std::deque< FigureFace* > faces )
{
    this->faces = faces;

    // Calculate Edges
    for(int i=0; i<this->faces.size(); i++){
        std::deque<Point3d*> pointsFace = this->faces[i]->getPoints();
        for(int j=0; j<pointsFace.size(); j++){
            Point3d * p1 = pointsFace[j];
            Point3d * p2 = pointsFace[(j+1)%pointsFace.size()];

            Edge * existingEdge = this->hasEdge( p1, p2 );
            if( existingEdge == NULL ){
                existingEdge = new Edge( p1, p2, this->edges.size() );
                this->edges.push_back( existingEdge );
            }
            // Associate Edge to face
            existingEdge->associateTo( this->faces[i] );
            this->faces[i]->includeEdge( existingEdge );
        }
    }
}

void Figure::centralizeFigure()
{
    if( this->points.size() > 0 )
    {
        vec3 centreResult = this->points[0]->toVector();

        for(int i=1; i<this->points.size(); i++)
        {
            centreResult = centreResult.addition( this->points[i]->toVector() );
        }

        centreResult = centreResult.division( this->points.size() );

        for(int i=0; i<this->points.size(); i++)
        {
            vec3 centeredPoint = this->points[i]->toVector().soustraction( centreResult );
            this->points[i]->set( centeredPoint.getX(), centeredPoint.getY(), centeredPoint.getZ() );
        }
    }
}

vec3 positiveValues(vec3 v)
{
    if( v.getX() < 0 ) v.setX( v.getX()*-1. );
    if( v.getY() < 0 ) v.setY( v.getY()*-1. );
    if( v.getZ() < 0 ) v.setZ( v.getZ()*-1. );
    return v;
}

void Figure::normalizeFigure()
{
    // TODO must be centralized
    if( this->points.size() > 0 )
    {
        double normeResult = this->points[0]->toVector().norme();

        for(int i=1; i<this->points.size(); i++)
        {
            double normePoint = this->points[i]->toVector().norme();
            if( normePoint > normeResult ) normeResult = normePoint;
        }

        for(int i=0; i<this->points.size(); i++)
        {
            vec3 normalizedPoint = this->points[i]->toVector().division( normeResult );
            this->points[i]->set( normalizedPoint.getX(), normalizedPoint.getY(), normalizedPoint.getZ() );
        }
    }
}

void Figure::printInfo(){
    std::cout << "========================" << std::endl << std::endl;

    std::deque<Point3d*> points = this->getPoints();
	std::cout << "Vertices: " << points.size() << std::endl;

	std::deque<Edge*> edges = this->getEdges();
	std::cout << "Edges: " << edges.size() << std::endl;

    std::deque<FigureFace*> faces = this->getFaces();
	std::cout << "Faces: " << faces.size() << std::endl;

	// Edges per Face
	int maxEdgesPerFace = 0;
	int minEdgesPerFace = INT_MAX;
	for(int i=0; i<faces.size(); i++){
        int countEdges = faces[i]->getEdgesCount();
        if( countEdges > maxEdgesPerFace ){
            maxEdgesPerFace = countEdges;
        }
        if( countEdges < minEdgesPerFace ){
            minEdgesPerFace = countEdges;
        }
    }
	std::cout << "Max Edges per Face: " << maxEdgesPerFace << std::endl;
	std::cout << "Min Edges per Face: " << minEdgesPerFace << std::endl;

	// Edges per Vertex
	int maxEdgesPerVertex = 0;
	int minEdgesPerVertex = INT_MAX;
    for(int i=0; i<points.size(); i++){
        int countEdges = points[i]->getEdgesCount();
        if( countEdges > maxEdgesPerVertex ){
            maxEdgesPerVertex = countEdges;
        }
        if( countEdges < minEdgesPerVertex ){
            minEdgesPerVertex = countEdges;
        }
    }
	std::cout << "Max Edges per Vertex: " << maxEdgesPerVertex << std::endl;
	std::cout << "Min Edges per Vertex: " << minEdgesPerVertex << std::endl;

	// Shared Edges
    int edgeSharedByZeroFace = 0;
    int edgeSharedByOneFace = 0;
    int edgeSharedByTwoFaces = 0;
    for(int i=0; i<edges.size(); i++){
        int countFaces = edges[i]->getFacesCount();
        if( countFaces == 2 ){
            edgeSharedByTwoFaces++;
        } else
        if( countFaces == 1 ){
            edgeSharedByOneFace++;
        } else
        if( countFaces == 0 ){
            edgeSharedByZeroFace++;
        }
    }
    std::cout << "Edges shared by 0 face: " << edgeSharedByZeroFace << std::endl;
	std::cout << "Edges shared by 1 face: " << edgeSharedByOneFace << std::endl;
	std::cout << "Edges shared by 2 faces: " << edgeSharedByTwoFaces << std::endl;
	std::cout << "========================" << std::endl << std::endl;
}

void Figure::removeFace( int index ){
    if( index < this->faces.size() ){
        FigureFace * f = this->faces[index];
        this->faces.erase( this->faces.begin()+index );
        delete( f );
    }
}

void Figure::generatePointsAndFaces(){}

std::deque<Point3d *> Figure::voxelVeticesInsideFigure( Voxel voxel ){
    std::deque<Point3d *> result;
    // check if there is at least one vertex inside the voxel
    for(int i=0; i<this->points.size(); i++){
        if( voxel.pointInside( this->points[i]->toVector() ) ){
            result.push_back( this->points[i] );
        }
    }
    return result;
}

bool Figure::pointInside( vec3 point ){
    return false;
}

CubeVolume Figure::getBoundingBox(){
    if( this->points.size() > 0 )
    {
        vec3 minPoint = this->points[0]->toVector();
        vec3 maxPoint = this->points[0]->toVector();

        for(int i=1; i<this->points.size(); i++)
        {
            // MIN
            if( minPoint.getX() > this->points[i]->getX()-this->center.getX() ){ minPoint.setX( this->points[i]->getX()-this->center.getX() ); }
            if( minPoint.getY() > this->points[i]->getY()-this->center.getY() ){ minPoint.setY( this->points[i]->getY()-this->center.getY() ); }
            if( minPoint.getZ() > this->points[i]->getZ()-this->center.getZ() ){ minPoint.setZ( this->points[i]->getZ()-this->center.getZ() ); }
            // MAX
            if( maxPoint.getX() < this->points[i]->getX()-this->center.getX() ){ maxPoint.setX( this->points[i]->getX()-this->center.getX() ); }
            if( maxPoint.getY() < this->points[i]->getY()-this->center.getY() ){ maxPoint.setY( this->points[i]->getY()-this->center.getY() ); }
            if( maxPoint.getZ() < this->points[i]->getZ()-this->center.getZ() ){ maxPoint.setZ( this->points[i]->getZ()-this->center.getZ() ); }
        }

        if( -minPoint.getX() > maxPoint.getX() ){ maxPoint.setX( -minPoint.getX() ); }
        if( -minPoint.getY() > maxPoint.getY() ){ maxPoint.setY( -minPoint.getY() ); }
        if( -minPoint.getZ() > maxPoint.getZ() ){ maxPoint.setZ( -minPoint.getZ() ); }

        return Voxel( this->center, ceil(maxPoint.getX()*2), ceil(maxPoint.getY()*2), ceil(maxPoint.getZ()*2) );
    }
    return Voxel( this->center, 1, 1, 1 );
}
