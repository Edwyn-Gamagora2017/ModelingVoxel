#include "Point3d.h"
#include "../../utils/utils.h"

Point3d::Point3d(double x, double y, double z, int index) {
	this->set(x, y, z);
	this->index = index;
}

Point3d::~Point3d() {
}

Point3d * Point3d::zero(int index) {
	return new Point3d(0, 0, 0, index);
}

double Point3d::getX() {
	return this->x;
}
double Point3d::getY() {
	return this->y;
}
double Point3d::getZ() {
	return this->z;
}
int Point3d::getIndex(){
    return this->index;
}
int Point3d::getEdgesCount(){
    return this->edges.size();
}

void Point3d::set(double x, double y, double z) {
	this->setX(x);
	this->setY(y);
	this->setZ(z);
}
void Point3d::setX(double x) {
	this->x = x;
}
void Point3d::setY(double y) {
	this->y = y;
}
void Point3d::setZ(double z) {
	this->z = z;
}

std::string Point3d::toString() {
	return "( " + doubleToStr(this->getX()) + " , " + doubleToStr(this->getY()) + " , " + doubleToStr(this->getZ()) + " )";
}

vec3 Point3d::vectorFrom( Point3d origin )
{
    return vec3( this->getX(), this->getY(), this->getZ() ).soustraction(vec3( origin.getX(), origin.getY(), origin.getZ() ));
}
vec3 Point3d::toVector()
{
    return this->vectorFrom( Point3d(0,0,0,this->index) );
}
Point3d Point3d::fromVector(vec3 vect, int index)
{
    return Point3d( vect.getX(), vect.getY(), vect.getZ(), index );
}

/*
Face
*/
bool Point3d::isAssociatedTo(FigureFace * f){
    if( f != NULL ){
        for( int i = 0; i<this->faces.size(); i++ ){
            if (this->faces[i]->getIndex() == f->getIndex()){
                return true;
            }
        }
    }

    return false;
}
void Point3d::associateTo(FigureFace * f){
    if( !this->isAssociatedTo( f ) ){
        this->faces.push_back( f );
    }
}
void Point3d::disassociateTo(FigureFace * f){
    f->removeVertex( this );

    for(int i=0; i<this->faces.size(); i++){
        if( this->faces[i]->getIndex() == f->getIndex() ){
            this->faces.erase( this->faces.begin()+i );
            break;
        }
    }

    // IMPORTANT do not delete it when there is no faces because it breaks index system
    // TODO index independent
}

/*
Edge
*/
bool Point3d::isAssociatedTo(Edge * e){
    if( e != NULL ){
        for( int i = 0; i<this->edges.size(); i++ ){
            if (this->edges[i]->getIndex() == e->getIndex()){
                return true;
            }
        }
    }

    return false;
}
void Point3d::associateTo(Edge * e){
    if( !this->isAssociatedTo( e ) ){
        this->edges.push_back( e );
    }
}

/*
Color
*/
Point3d Point3d::add( double x, double y, double z )
{
    return Point3d( this->getX()+x, this->getY()+y, this->getZ()+z, this->index );
}
