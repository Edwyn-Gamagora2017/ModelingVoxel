#include "SphereVolume.h"

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
