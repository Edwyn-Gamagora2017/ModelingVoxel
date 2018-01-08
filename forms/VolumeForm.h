#ifndef VOLUMEFORM_H
#define VOLUMEFORM_H

#include <deque>

#include "../figures/data_structure/vec3.h"
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

        vec3 getCenter();

        /** \brief given a voxel, verifies which vertices are inside the form
         *
         * \param voxel Voxel - the voxel to be evaluated
         * \return std::deque<vec3> - vertices that are inside the form
         *
         */
        virtual std::deque<vec3> voxelVeticesInside( Voxel voxel );

        /** \brief given a voxel, checks if the center of the form is inside de voxel (this way, it is useful to check if the form is inside the voxel)
         *
         * \param voxel Voxel - the voxel to be evaluated
         * \return bool - the center of the form is inside the voxel
         *
         */
        bool isCenterInsideVoxel( Voxel voxel );

        /** \brief indicates if a point is inside the form (implemented by each form)
         *
         * \param point vec3 - point to be verified
         * \return virtual bool - POINT is inside the form
         *
         */
        virtual bool pointInside( vec3 point ) = 0;

        /** \brief obtains the box that encloses the form (implemented by each form)
         *
         * \return virtual CubeVolume - the box that encloses the form
         *
         */
        virtual CubeVolume getBoundingBox() = 0;
};

#endif // VOLUMEFORM_H
