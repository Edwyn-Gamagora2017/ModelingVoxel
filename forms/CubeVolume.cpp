#include "CubeVolume.h"

CubeVolume::CubeVolume( vec3 center, float w, float h, float d ) :
    VolumeForm::VolumeForm( center )
{
    this->width = w;
    this->height = h;
    this->depth = d;
}

CubeVolume::~CubeVolume()
{
    //dtor
}

std::deque<vec3> CubeVolume::getVertices(){
    std::deque<vec3> result;
    // Front
    result.push_back( this->center.addition( vec3( -this->width/2., this->height/2., this->depth/2. ) ) );
    result.push_back( this->center.addition( vec3( this->width/2., this->height/2., this->depth/2. ) ) );
    result.push_back( this->center.addition( vec3( this->width/2., -this->height/2., this->depth/2. ) ) );
    result.push_back( this->center.addition( vec3( -this->width/2., -this->height/2., this->depth/2. ) ) );
    // Back
    result.push_back( this->center.addition( vec3( -this->width/2., this->height/2., -this->depth/2. ) ) );
    result.push_back( this->center.addition( vec3( this->width/2., this->height/2., -this->depth/2. ) ) );
    result.push_back( this->center.addition( vec3( this->width/2., -this->height/2., -this->depth/2. ) ) );
    result.push_back( this->center.addition( vec3( -this->width/2., -this->height/2., -this->depth/2. ) ) );

    return result;
}

bool CubeVolume::pointInside( vec3 point ){
    return point.getX() >= this->center.getX() - this->width/2. &&
            point.getX() <= this->center.getX() + this->width/2. &&
            point.getY() >= this->center.getY() - this->height/2. &&
            point.getY() <= this->center.getY() + this->height/2. &&
            point.getZ() >= this->center.getZ() - this->depth/2. &&
            point.getZ() <= this->center.getZ() + this->depth/2.;
}

CubeVolume CubeVolume::getBoundingBox(){
    return *this;
}
