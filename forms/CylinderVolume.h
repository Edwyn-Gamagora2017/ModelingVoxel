#ifndef CYLINDERVOLUME_H
#define CYLINDERVOLUME_H

#include "VolumeForm.h"

class CylinderVolume :
    public VolumeForm
{
    protected:
        float radius;
        float height;

    public:
        /** Default constructor */
        CylinderVolume( vec3 center, float height, float radius );
        /** Default destructor */
        virtual ~CylinderVolume();

        bool pointInside( vec3 point );
        CubeVolume getBoundingBox();
};

#endif // CYLINDERVOLUME_H
