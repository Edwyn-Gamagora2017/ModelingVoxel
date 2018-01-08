#include "Edge.h"

Edge::Edge( Point3d * _pointA, Point3d * _pointB, int _index ){
    this->pointA = _pointA;
    this->pointB = _pointB;
    this->index = _index;

    this->pointA->associateTo( this );
    this->pointB->associateTo( this );
}
Edge::~Edge(){}

bool Edge::hasPoint( Point3d * p ){
    if( p != NULL ){
        return (this->pointA->getIndex() == p->getIndex() || this->pointB->getIndex() == p->getIndex() );
    }
    return false;
}
bool Edge::hasPoints( Point3d * p1, Point3d * p2 ){
    return this->hasPoint( p1 ) && this->hasPoint( p2 );
}
/*
Face
*/
bool Edge::isAssociatedTo(FigureFace * f){
    if( f != NULL ){
        for( int i = 0; i<this->faces.size(); i++ ){
            if (this->faces[i]->getIndex() == f->getIndex()){
                return true;
            }
        }
    }

    return false;
}
void Edge::associateTo(FigureFace * f){
    if( !this->isAssociatedTo( f ) ){
        this->faces.push_back( f );
    }
}
void Edge::disassociateTo(FigureFace * f){
    f->removeEdge( this );

    for(int i=0; i<this->faces.size(); i++){
        if( this->faces[i]->getIndex() == f->getIndex() ){
            this->faces.erase( this->faces.begin()+i );
            break;
        }
    }

    // TODO verify the need of this edge (if no more faces) and delete it
}

/* GETTERS */
Point3d * Edge::getPointA(){
    return this->pointA;
}
Point3d * Edge::getPointB(){
    return this->pointB;
}
std::deque<FigureFace *> Edge::getFaces(){
    return this->faces;
}
int Edge::getIndex(){
    return this->index;
}
int Edge::getFacesCount(){
    return this->faces.size();
}
