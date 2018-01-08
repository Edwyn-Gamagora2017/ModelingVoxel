#include "flocon.h"

#include <math.h>
# define M_PI           3.14159265358979323846  /* pi */

flocon::flocon( vec3 * a, vec3 * b, vec3 * c, point3d * couleur ) : figure::figure(new vec3(0.,0.,0.),new vec3(1.,1.,1.),new vec3(0.,0.,0.),couleur)
{
    this->a = a;
    this->b = b;
    this->c = c;
}

flocon::~flocon()
{
    //dtor
}

vec3 * flocon::getA(){
    return this->a;
}
vec3 * flocon::getB(){
    return this->b;
}
vec3 * flocon::getC(){
    return this->c;
}

std::deque<point3d> getVerticesLine( vec3 * a, vec3 * b ){
    std::deque<point3d> res;

	vec3 ab = b->soustraction(*a);
	vec3 m = a->addition( ab.multiplication( 1./3. ) );
    vec3 n = a->addition( ab.multiplication( 2./3. ) );
    vec3 i = a->addition( ab.division( 2. ) );
    vec3 in = n.soustraction( i );
    vec3 in_ = vec3( in.getY(), -in.getX(), in.getZ() );
    vec3 o = i.addition( in_.multiplication( -sin( M_PI/3 ) ) ); // o = i + in*sin(pi/3)

    res.push_back( point3d( a->getX(), a->getY(), a->getZ() ) );
    res.push_back( point3d( m.getX(), m.getY(), m.getZ() ) );
    res.push_back( point3d( o.getX(), o.getY(), o.getZ() ) );
    res.push_back( point3d( n.getX(), n.getY(), n.getZ() ) );
    res.push_back( point3d( b->getX(), b->getY(), b->getZ() ) );

	return res;
}

std::deque<point3d> floconRec( vec3 * a, vec3 * b, int currentRec, int maxRec ){
    std::deque<point3d> vertices = getVerticesLine(a,b);
    if( currentRec < maxRec ){
        std::deque<point3d> verticesResultat;
        for(int i=0;i<vertices.size()-1;i++){
            std::deque<point3d> resultat = floconRec( new vec3( vertices[i].getX(),vertices[i].getY(),vertices[i].getZ() ), new vec3( vertices[i+1].getX(),vertices[i+1].getY(),vertices[i+1].getZ()), currentRec+1, maxRec);
            for( int i = 0; i < resultat.size(); i++ ){
                verticesResultat.push_back( resultat[i] );
            }
        }
        return verticesResultat;
    }
    else{
        return vertices;
    }
}

std::deque<point3d> flocon::getVertices()
{
    // a-b
    std::deque<point3d> verticesResultat = floconRec( this->getA(), this->getB(), 1, 1 );
    // b-c
    std::deque<point3d> verticesResultat2 = floconRec( this->getB(), this->getC(), 1, 2 );
    // c-a
    std::deque<point3d> verticesResultat3 = floconRec( this->getC(), this->getA(), 1, 3 );

    for( int i = 0; i < verticesResultat2.size(); i++ ){
        verticesResultat.push_back( verticesResultat2[i] );
    }
    for( int i = 0; i < verticesResultat3.size(); i++ ){
        verticesResultat.push_back( verticesResultat3[i] );
    }

    return verticesResultat;
}
