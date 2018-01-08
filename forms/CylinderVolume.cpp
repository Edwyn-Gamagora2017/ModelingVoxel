#include "CylinderVolume.h"
#include "CubeVolume.h"
#include <math.h>

CylinderVolume::CylinderVolume( vec3 center, float height, float radius ) :
    VolumeForm::VolumeForm( center )
{
    this->radius = radius;
    this->height = height;
}

CylinderVolume::~CylinderVolume()
{
    //dtor
}

bool CylinderVolume::pointInside( vec3 point ){
    float dist = sqrt(pow(point.getX()-this->center.getX(),2)+pow(point.getZ()-this->center.getZ(),2));
    // distance from point to center is smaller than radius
    if( dist <= this->radius ){
        return point.getY() - center.getY() >= -height/2 && point.getY() - center.getY() <= height/2;
    }
    return false;
}

CubeVolume CylinderVolume::getBoundingBox(){
    return Voxel( this->center, this->radius*2, this->height, this->radius*2 );
}
