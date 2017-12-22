#ifndef FORMTREE_H
#define FORMTREE_H

#include "../CubeVolume.h"
#define Voxel CubeVolume

class FormTree
{
    public:
        enum FormOperation{ Intersection, Union };

    private:
        VolumeForm * leaf;
        FormTree * rightTree, * leftTree;
        FormOperation operation;

    public:
        FormTree( VolumeForm * leaf );
        FormTree( FormOperation operation, FormTree * rightTree, FormTree * leftTree );
        virtual ~FormTree();

        /** \brief given a voxel, verifies if some vertices are inside the forms
         *
         * \param voxel Voxel - the voxel to be evaluated
         * \return bool - there are vertices inside the form
         *
         */
        bool voxelVeticesInside( Voxel voxel );

        /** \brief given a voxel, checks if the center of the form is inside de voxel (this way, it is useful to check if the form is inside the voxel)
         *
         * \param voxel Voxel - the voxel to be evaluated
         * \return bool - the center of the form is inside the voxel
         *
         */
        bool isCenterInsideVoxel( Voxel voxel );

        /** \brief obtains the box that encloses the tree
         *
         * \return virtual CubeVolume - the box that encloses the tree
         *
         */
        CubeVolume getBoundingBox();

    protected:

    private:
};

#endif // FORMTREE_H
