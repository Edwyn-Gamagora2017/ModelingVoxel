#ifndef VOLUMEFORM_H
#define VOLUMEFORM_H

#include <deque>

#include "../vec3.h"
class CubeVolume;

#define Voxel CubeVolume

class VolumeForm
{
    protected:
        vec3 center;

    public:
        /** Default constructor */
        VolumeForm( vec3 center );
        /** Default destructor */
        virtual ~VolumeForm();

        std::deque<vec3> voxelVeticesInside( Voxel voxel );
        bool isCenterInsideVoxel( Voxel voxel );

        virtual bool pointInside( vec3 point ) = 0;
};

#endif // VOLUMEFORM_H
