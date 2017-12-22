#include "SphereVolume.h"
#include "CubeVolume.h"

SphereVolume::SphereVolume(vec3 center, float radius) :
    VolumeForm::VolumeForm( center )
{
    this->radius = radius;
}

SphereVolume::~SphereVolume()
{
    //dtor
}

bool SphereVolume::pointInside( vec3 point ){
    // distance from point to center is smaller than radius
    return( point.soustraction( center ).norme() <= this->radius );
}

CubeVolume SphereVolume::getBoundingBox(){
    return Voxel( this->center, this->radius*2, this->radius*2, this->radius*2 );
}

