#include "FormTree.h"
#include <algorithm>

FormTree::FormTree( VolumeForm * leaf ){
    this->leaf = leaf;
}
FormTree::FormTree( FormOperation operation, FormTree * rightTree, FormTree * leftTree ){
    this->operation = operation;
    this->rightTree = rightTree;
    this->leftTree = leftTree;
}

FormTree::~FormTree(){}

bool FormTree::voxelVeticesInside( Voxel voxel ){
    if( this->leaf != NULL ){
        return this->leaf->voxelVeticesInside( voxel ).size() > 0;
    }
    else{
        // TODO CHECK BRANCHES NULL
        bool left = this->leftTree != NULL && this->leftTree->voxelVeticesInside( voxel );
        bool right = this->rightTree != NULL && this->rightTree->voxelVeticesInside( voxel );
        switch( this->operation ){
        case FormTree::FormOperation::Intersection :
            return left && right;
            break;
        case FormTree::FormOperation::Union :
        default:
            return left || right;
            break;
        }
    }
}
bool FormTree::isCenterInsideVoxel( Voxel voxel ){
    if( this->leaf != NULL ){
        return this->leaf->isCenterInsideVoxel( voxel );
    }
    else{
        // TODO CHECK BRANCHES NULL
        bool left = this->leftTree != NULL && this->leftTree->isCenterInsideVoxel( voxel );
        bool right = this->rightTree != NULL && this->rightTree->isCenterInsideVoxel( voxel );
        switch( this->operation ){
        case FormTree::FormOperation::Intersection :
            return left && right;
            break;
        case FormTree::FormOperation::Union :
        default:
            return left || right;
            break;
        }
    }
}

CubeVolume FormTree::getBoundingBox(){
    if( this->leaf != NULL ){
        return this->leaf->getBoundingBox();
    }
    else{
        // TODO CHECK BRANCHES NULL
        CubeVolume left = this->leftTree->getBoundingBox();
        CubeVolume right = this->rightTree->getBoundingBox();
        float maxX = std::max( left.getCenter().getX()+left.getWidth()/2., right.getCenter().getX()+right.getWidth()/2. );
        float minX = std::min( left.getCenter().getX()-left.getWidth()/2., right.getCenter().getX()-right.getWidth()/2. );
        float maxY = std::max( left.getCenter().getY()+left.getHeight()/2., right.getCenter().getY()+right.getHeight()/2. );
        float minY = std::min( left.getCenter().getY()-left.getHeight()/2., right.getCenter().getY()-right.getHeight()/2. );
        float maxZ = std::max( left.getCenter().getZ()+left.getDepth()/2., right.getCenter().getZ()+right.getDepth()/2. );
        float minZ = std::min( left.getCenter().getZ()-left.getDepth()/2., right.getCenter().getZ()-right.getDepth()/2. );

        float width = (maxX-minX);
        float height = (maxY-minY);
        float depth = (maxZ-minZ);

        return CubeVolume( vec3( maxX-width/2., maxY-height/2., maxZ-depth/2. ), width, height, depth );
    }
}
