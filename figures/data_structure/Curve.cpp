#include "Curve.h"

// calculate the intermediate k position on the Bezier curve (Casteljau) related to the factor u [0,1]
vec3 Curve::casteljauP( double u, int k, int i, std::deque<Point3d> controlPoints ){
    if( k == 0 ){
        return controlPoints[i].toVector();
    }
    else{
        return Curve::casteljauP( u, k-1, i, controlPoints ).multiplication( 1-u ).addition( Curve::casteljauP( u, k-1, i+1, controlPoints ).multiplication( u ) );
    }
}

// calculate the Bezier curve based on Casteljau algorithm. amountSamples defines the amount of samples in the curve
std::deque<vec3> Curve::casteljau( std::deque<Point3d> controlPoints, int amountSamples ){
    int amount = amountSamples+2;   // at least 2 samples will be created
    std::deque<vec3> result;
    for( int i=0; i<amount; i++ ){
        result.push_back( Curve::casteljauP( i/((double)amount-1), controlPoints.size()-1, 0, controlPoints ) );
    }
    return result;
}
