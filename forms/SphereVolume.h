#ifndef SPHEREVOLUME_H
#define SPHEREVOLUME_H

#include "VolumeForm.h"

class SphereVolume :
    public VolumeForm
{
    protected:
        float radius;

    public:
        /** Default constructor */
        SphereVolume( vec3 center, float radius );
        /** Default destructor */
        virtual ~SphereVolume();

        bool pointInside( vec3 point );
};

#endif // SPHEREVOLUME_H
