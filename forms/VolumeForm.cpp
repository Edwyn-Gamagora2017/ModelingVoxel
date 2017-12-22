#include "VolumeForm.h"
#include "CubeVolume.h"

VolumeForm::VolumeForm( vec3 center )
{
    this->center = center;
}

VolumeForm::~VolumeForm()
{
    //dtor
}

vec3 VolumeForm::getCenter(){
    return this->center;
}

std::deque<vec3> VolumeForm::voxelVeticesInside( Voxel voxel ){
    std::deque<vec3> result;
    for( vec3 vertex : voxel.getVertices() ){
        if( this->pointInside( vertex ) ){
            result.push_back( vertex );
        }
    }
    return result;
}

bool VolumeForm::isCenterInsideVoxel( Voxel voxel ){
    return voxel.pointInside( this->center );
}
