#include "FromTree.h"

FormTree::FormTree( VolumeForm * leaf ){
    this->leaf = leaf;
}
FormTree::FormTree( FormOperation operation, FormTree * rightTree, FormTree * leftTree ){
    this->operation = operation;
    this->rightTree = rightTree;
    this->leftTree = leftTree;
}

FromTree::~FromTree(){}

bool FormTree::voxelVeticesInside( Voxel voxel ){
    if( this->leaf != NULL ){
        return this->leaf->voxelVeticesInside( voxel );
    }
    else{
        bool left = this->leftTree != null && this->leftTree->voxelVeticesInside();
        bool right = this->rightTree != null && this->rightTree->voxelVeticesInside();
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
        bool left = this->leftTree != null && this->leftTree->isCenterInsideVoxel();
        bool right = this->rightTree != null && this->rightTree->isCenterInsideVoxel();
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
