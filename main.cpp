/**
 *	Tableau des points permettant de gérer les points de controles
 * On sélectionne le point avec un chiffre 0 pour P0, 1 pour P1, ...
 * On sélectionne ensuite si on veut faire monter, descendre amener vers la gauche ou la droite le point.
 *   d : translation à droite
 *   q : à gauche
 *   z : en haut
 *   s : en bas
 *
 */

 #include <windows.h>

#include <GL/glut.h>
#include <GL/glu.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <deque>
#include "figures/data_structure/vec3.h"
#include "utils/utils.h"

#include "CubeVolume.h"
#include "SphereVolume.h"
#include "CylinderVolume.h"
#include "FormTree.h"

#include "figures/data_structure/Point3d.h"
#include "figures/data_structure/Figure.h"
#include "utils/utils.h"
#include "utils/OffFile.h"

#define Voxel CubeVolume

/* au cas ou M_PI ne soit defini */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ESC 27

#define WIDTH  600
#define HEIGHT 600
float angleX=0.0f; //angle de rotation en Y de la scene
float angleY=0.0f; //angle de rotation en X de la scene

vec3 white( 1,1,1 );
vec3 red( 1,0,0 );
vec3 green( 0,1,0 );
vec3 blue( 0,0,1 );
Point3d * color_black = new Point3d( 0,0,0,0 );

// Parameters
bool drawVoxelEdges = false;
bool considerLight = true;
vec3 lightPosition(5,0,-5);
float voxel_dimension = 2;
float showThreshold = 3;
// Parameters Mesh
bool showMesh = false;
bool showSimplification = true;
bool showTool = false;

FormTree * tree;
SphereVolume * s, * s2, * s3;
CubeVolume * c, * c2, * c3;
CylinderVolume * cy;
Figure * f, * fSimp;

// Tool to add volume
CubeVolume * tool;
float toolMoveStep = 0.1;

struct spaceVoxel{
    public :
    Voxel * voxel;
    int value;
    std::deque< Point3d * > vertices;
};

spaceVoxel *** space;
int spaceW, spaceH, spaceD;
std::deque<spaceVoxel *> selectedVoxels;

void fillSpace( FormTree * formTree, float voxelDimension );
void fillSpaceWithFigure( Figure * figure, float voxelDimension );
Figure * Simplification( Figure * figure );

void fillSpace(){
    if( !showMesh ){
        fillSpace( tree, voxel_dimension );
    }else{
        fillSpaceWithFigure( f, voxel_dimension );

        fSimp = Simplification( f );
    }
}

/* initialisation d'OpenGL*/
static void init(void)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);

	s = new SphereVolume( vec3( 0,0.5,0 ), 0.5 );
	c = new CubeVolume( vec3( 0,-0.5,0 ), 1, 1, 1 );
	c2 = new CubeVolume( vec3( 0,0,0 ), 1, 1, 1 );

	s2 = new SphereVolume( vec3( -2,0,0 ), 1. );
	c3 = new CubeVolume( vec3( -3,0,0 ), 2, 2, 2 );

	cy = new CylinderVolume( vec3( 3,0,0 ), 1, 1 );

	tool = new CubeVolume( vec3( 1,0,0 ), 0.5, 0.5, 0.5 );

	// Mesh
    f    = OffFile::readFile( "maillages/triceratops" );

    FormTree * t1 = new FormTree(
                        FormTree::Union,
                        new FormTree( c ),
                        new FormTree(
                            FormTree::Intersection,
                            new FormTree(c2),
                            new FormTree(s)
                            )
                        );
    FormTree * t2 = new FormTree(
                            FormTree::Difference,
                            new FormTree( c3 ),
                            new FormTree(
                                s2
                                )
                            );

    FormTree * t3 = new FormTree( cy );

    if( !showMesh ){
        voxel_dimension = 0.1;
        tree = new FormTree(
                    FormTree::Union,
                    t1,
                    new FormTree(
                        FormTree::Union,
                        t2,
                        t3
                        )
                    );
    }
    else{
        voxel_dimension = 1;
    }
    fillSpace();
}

void drawSquare( vec3 p1, vec3 p2, vec3 p3, vec3 p4, vec3 color, GLenum mode, bool considerLight, vec3 lightPosition ){
    if( considerLight ){
        vec3 normal = p2.soustraction( p1 ).produitVectoriel( p3.soustraction( p1 ) );
        float factor = normal.normalized().produitScalaire( lightPosition.soustraction( p1 ).normalized() );
        if( factor < 0.1 ) factor = 0.1;
        glColor3f(color.getX()*factor,color.getY()*factor,color.getZ()*factor);
    }
    else{
        glColor3f(color.getX(),color.getY(),color.getZ());
    }
    glBegin( mode );
        glVertex3f( p1.getX(), p1.getY(), p1.getZ() );
        glVertex3f( p2.getX(), p2.getY(), p2.getZ() );
        glVertex3f( p3.getX(), p3.getY(), p3.getZ() );
        glVertex3f( p4.getX(), p4.getY(), p4.getZ() );
	glEnd();
}

void drawVoxel( Voxel voxel, vec3 color, GLenum mode, bool considerLight, vec3 lightPosition )
{
    std::deque<vec3> points = voxel.getVertices();

    // Front
    /*
    0 - 1
    3 - 2
    */
	drawSquare( points[0], points[1], points[2], points[3], color, mode, considerLight, lightPosition );
	// Back
	/*
    4 - 5
    7 - 6
    */
	drawSquare( points[5], points[4], points[7], points[6], color, mode, considerLight, lightPosition );

	// Left
	drawSquare( points[4], points[0], points[3], points[7], color, mode, considerLight, lightPosition );
	// Right
	drawSquare( points[1], points[5], points[6], points[2], color, mode, considerLight, lightPosition );
	// Top
	drawSquare( points[4], points[5], points[1], points[0], color, mode, considerLight, lightPosition );
	// Bottom
	drawSquare( points[3], points[2], points[6], points[7], color, mode, considerLight, lightPosition );
}

void drawVolumeForm( VolumeForm * form, float voxelDimension, vec3 color, GLenum mode, bool drawVoxelEdges, bool considerLight, vec3 lightPosition ){
    float EnvDimension = 10;

    FOR( w, EnvDimension/voxelDimension ){
        FOR( h, EnvDimension/voxelDimension ){
            FOR( d, EnvDimension/voxelDimension ){
                Voxel v( vec3( w*voxelDimension+voxelDimension/2.-EnvDimension/2., h*voxelDimension+voxelDimension/2.-EnvDimension/2., d*voxelDimension+voxelDimension/2.-EnvDimension/2. ), voxelDimension, voxelDimension, voxelDimension );
                if( form->voxelVeticesInside( v ).size() > 0 || form->isCenterInsideVoxel( v ) ){
                    drawVoxel( v, color, mode, considerLight, lightPosition );
                }
                else{
                    if( drawVoxelEdges ){
                        drawVoxel( v, white, GL_LINE_LOOP, false, lightPosition );
                    }
                }
            }
        }
    }
}

bool isSpaceVoxelVisible( spaceVoxel voxel ){
    return voxel.value > showThreshold;
}

void selectVisibleVoxels( spaceVoxel *** spaceMatrix ){
    selectedVoxels.clear();

    // Select Visible Voxels
    FOR( w, spaceW ){
        FOR( h, spaceH){
            FOR( d, spaceD ){
                if( isSpaceVoxelVisible( spaceMatrix[w][h][d] )
                   && (!(w > 0 && w < spaceW-1 && isSpaceVoxelVisible( (spaceMatrix[w+1][h][d]) ) && isSpaceVoxelVisible( (spaceMatrix[w-1][h][d]) )
                   && h > 0 && h < spaceH-1 && isSpaceVoxelVisible( (spaceMatrix[w][h+1][d]) ) && isSpaceVoxelVisible( (spaceMatrix[w][h-1][d]) )
                   && d > 0 && d < spaceD-1 && isSpaceVoxelVisible( (spaceMatrix[w][h][d+1]) ) && isSpaceVoxelVisible( (spaceMatrix[w][h][d-1]) ) ) || showMesh )
                )
                {
                    selectedVoxels.push_back( &space[w][h][d] );
                }
            }
        }
    }
    std::cout << "Visible Voxels : " << selectedVoxels.size() << std::endl;
}

void clearSpace(){
    // clear space
    if( space != NULL ){
        FOR( w, spaceW ){
            if( space[w] != NULL ){
                FOR( h, spaceH){
                    if( space[w][h] != NULL ){
                        delete space[w][h];
                    }
                }
                delete space[w];
            }
        }
        delete space;
    }
}

void fillSpace( FormTree * formTree, float voxelDimension ){
    clearSpace();

    CubeVolume box = formTree->getBoundingBox();
    float boxCenterX = box.getCenter().getX();
    float boxCenterY = box.getCenter().getY();
    float boxCenterZ = box.getCenter().getZ();

    // Matrix to store values
    spaceW = ceil(box.getWidth()/voxel_dimension)+2;
    spaceH = ceil(box.getHeight()/voxel_dimension)+2;
    spaceD = ceil(box.getDepth()/voxel_dimension)+2;

    // Calculating Voxels
    space = new spaceVoxel**[ spaceW ];
    FOR( w, spaceW ){
        space[w] = new spaceVoxel*[ spaceH ];
        FOR( h, spaceH){
            space[w][h] = new spaceVoxel[ spaceD ];
            FOR( d, spaceD ){

                Voxel * v = new Voxel( vec3( boxCenterX-box.getWidth()/2. + (w+0.5)*voxelDimension,
                              boxCenterY-box.getHeight()/2. + (h+0.5)*voxelDimension,
                              boxCenterZ-box.getDepth()/2. + (d+0.5)*voxelDimension ),
                        voxelDimension, voxelDimension, voxelDimension );

                space[w][h][d].voxel = v;
                if( formTree->voxelVeticesInside( *v ) || formTree->isCenterInsideVoxel( *v ) ){
                    space[w][h][d].value = 5;
                }
                else{
                    space[w][h][d].value = 0;
                }
            }
        }
    }

    selectVisibleVoxels( space );
}

void fillSpaceWithFigure( Figure * figure, float voxelDimension ){
    clearSpace();

    CubeVolume box = figure->getBoundingBox();
    float boxCenterX = box.getCenter().getX();
    float boxCenterY = box.getCenter().getY();
    float boxCenterZ = box.getCenter().getZ();

    // Matrix to store values
    spaceW = ceil(box.getWidth()/voxel_dimension)+1;
    spaceH = ceil(box.getHeight()/voxel_dimension)+1;
    spaceD = ceil(box.getDepth()/voxel_dimension)+1;

    // Calculating Voxels
    space = new spaceVoxel**[ spaceW ];
    FOR( w, spaceW ){
        space[w] = new spaceVoxel*[ spaceH ];
        FOR( h, spaceH){
            space[w][h] = new spaceVoxel[ spaceD ];
            FOR( d, spaceD ){

                Voxel * v = new Voxel( vec3( boxCenterX-box.getWidth()/2. + (w+0.5)*voxelDimension,
                              boxCenterY-box.getHeight()/2. + (h+0.5)*voxelDimension,
                              boxCenterZ-box.getDepth()/2. + (d+0.5)*voxelDimension ),
                        voxelDimension, voxelDimension, voxelDimension );

                space[w][h][d].voxel = v;
                std::deque<Point3d *> vertices = figure->voxelVeticesInsideFigure( *v );

                if( vertices.size() > 0 ){//|| figure->isCenterInsideVoxel( *v ) ){
                    space[w][h][d].value = 5;
                    space[w][h][d].vertices = vertices;
                }
                else{
                    space[w][h][d].value = 0;
                }
            }
        }
    }

    selectVisibleVoxels( space );
}

void drawSpace( std::deque<spaceVoxel*> voxels, spaceVoxel *** spaceMatrix, float voxelDimension, vec3 color, GLenum mode, bool drawVoxelEdges, bool considerLight, vec3 lightPosition ){
    FOR( i, voxels.size() ){
        drawVoxel( *(voxels[i]->voxel), color, mode, considerLight, lightPosition );
    }

    FOR( w, spaceW ){
        FOR( h, spaceH){
            FOR( d, spaceD ){
                if( drawVoxelEdges ){
                    //if( !isSpaceVoxelVisible( spaceMatrix[w][h][d]) ){
                        drawVoxel( *(spaceMatrix[w][h][d].voxel), red, GL_LINE_LOOP, false, lightPosition );
                    //}
                }
                if( showTool && (tool->voxelVeticesInside( *(spaceMatrix[w][h][d].voxel) ).size() > 0 || tool->isCenterInsideVoxel( *(spaceMatrix[w][h][d].voxel) )) ){
                    drawVoxel( *(spaceMatrix[w][h][d].voxel), blue, mode, considerLight, lightPosition );
                }
            }
        }
    }
}

Figure * Simplification( Figure * figure )
{
    std::deque<Point3d*> newPoints;
    std::deque<int> origPointsCenters;
	std::deque<FigureFace*> newFaces;

    FOR(i,figure->getPoints().size()){
        origPointsCenters.push_back( -1 );
    }

    // For each voxel, choose the center
    FOR( k, selectedVoxels.size() ){
        if( selectedVoxels[k]->vertices.size() > 0){
            // Calculate average
            vec3 center = selectedVoxels[k]->vertices[0]->toVector();

            for( int i=1; i<selectedVoxels[k]->vertices.size(); i++ ){
                center = center.addition( selectedVoxels[k]->vertices[i]->toVector() );
            }
            center = center.division( selectedVoxels[k]->vertices.size() );

            int newPointIndex = newPoints.size();
            newPoints.push_back( new Point3d( center.getX(), center.getY(), center.getZ(), newPointIndex ) );

            // Storing center for each original point
            for( int i=0; i<selectedVoxels[k]->vertices.size(); i++ ){
                origPointsCenters[ selectedVoxels[k]->vertices[i]->getIndex() ] = newPointIndex;
            }
        }
    }
    // For each face, set new points
    FOR( i, figure->getFaces().size() ){
        std::deque<Point3d *> pointsFace = figure->getFaces()[i]->getPoints();
        std::deque<int> pointsFaceCenter;
        std::deque<Point3d *> newPointsFace;

        bool includeFace = true;
        FOR( j, pointsFace.size() ){
            int centerIndex = origPointsCenters[ pointsFace[ j ]->getIndex() ];
            if( centerIndex > 0 ){
                newPointsFace.push_back( newPoints[ centerIndex ] );
                pointsFaceCenter.push_back( centerIndex );

                // Check if some other original point has the same center
                FOR( k, pointsFaceCenter.size()-1 ){
                    if( pointsFaceCenter[k] == centerIndex ){
                        // not include face
                        includeFace = false;
                        break;
                    }
                }
            }
            else{
                includeFace = false;
                break;
            }
        }
        if( includeFace ){
            vec3 normal = newPointsFace[1]->toVector().soustraction( newPointsFace[0]->toVector() ).produitVectoriel( newPointsFace[2]->toVector().soustraction( newPointsFace[0]->toVector() ) );
            newFaces.push_back( new FigureFace( newPointsFace, normal, newFaces.size() ) );
        }
    }

    Figure * result = new Figure( new vec3(0,0,0), new vec3(1,1,1), new vec3(0,0,0), new Point3d(1,1,1,-1), false, false );
	result->setPoints( newPoints );
	result->setFaces( newFaces );

	//result->centralizeFigure();
	return result;
}

Figure * SubDivision( Figure * figure )
{
    std::deque<Point3d*> newPoints;
	std::deque<FigureFace*> newFaces;

	std::deque<int> edgeDivised_newPointIndex;
	std::deque<int> oldPoint_newPointIndex;

	FOR( i, figure->getEdges().size() ){
        edgeDivised_newPointIndex.push_back( -1 );
	}
	FOR( i, figure->getPoints().size() ){
        oldPoint_newPointIndex.push_back( -1 );
	}

	// For each face, calculate new points
    FOR( i, figure->getFaces().size() ){
        std::deque<Edge*> faceEdges = figure->getFaces()[i]->getEdges() ;
        //std::deque<Point3d*> fPoints = figure->getFaces()[i]->getPoints() ;

        // Center Points
        FOR( j, faceEdges.size() ){
            if( edgeDivised_newPointIndex[ faceEdges[j]->getIndex() ] == -1 ){
                vec3 pointA = faceEdges[j]->getPointA()->toVector();
                vec3 pointB = faceEdges[j]->getPointB()->toVector();
                // Update old Points indexes
                if( oldPoint_newPointIndex[ faceEdges[j]->getPointA()->getIndex() ] == -1 ){
                    int pointIndex = newPoints.size();
                    newPoints.push_back( new Point3d( pointA.getX(), pointA.getY(), pointA.getZ(), pointIndex ));
                    oldPoint_newPointIndex[ faceEdges[j]->getPointA()->getIndex() ] = pointIndex;
                }
                if( oldPoint_newPointIndex[ faceEdges[j]->getPointB()->getIndex() ] == -1 ){
                    int pointIndex = newPoints.size();
                    newPoints.push_back( new Point3d( pointB.getX(), pointB.getY(), pointB.getZ(), pointIndex ));
                    oldPoint_newPointIndex[ faceEdges[j]->getPointB()->getIndex() ] = pointIndex;
                }

                vec3 newPoint = pointA.addition( pointB ).division(2);
                int newPointIndex = newPoints.size();
                newPoints.push_back( new Point3d( newPoint.getX(), newPoint.getY(), newPoint.getZ(), newPointIndex ) );
                edgeDivised_newPointIndex[ faceEdges[j]->getIndex() ] = newPointIndex;
            }
        }
    }

    // For each face, calculate new faces
    FOR( i, figure->getFaces().size() ){
        std::deque<Point3d*> facePoints = figure->getFaces()[i]->getPoints() ;

        // New faces
        // New centers
        std::deque<Point3d*> newFacePoints;
        // 2
        // 1 3
        // 0 5 4

        Edge * edge1 = figure->hasEdge( facePoints[0], facePoints[1] );
        Edge * edge2 = figure->hasEdge( facePoints[1], facePoints[2] );
        Edge * edge3 = figure->hasEdge( facePoints[2], facePoints[0] );

        Point3d * p0 = newPoints[ oldPoint_newPointIndex[ facePoints[0]->getIndex() ] ];
        Point3d * p1 = newPoints[ edgeDivised_newPointIndex[ edge1->getIndex() ] ];
        Point3d * p2 = newPoints[ oldPoint_newPointIndex[ facePoints[1]->getIndex() ] ];
        Point3d * p3 = newPoints[ edgeDivised_newPointIndex[ edge2->getIndex() ] ];
        Point3d * p4 = newPoints[ oldPoint_newPointIndex[ facePoints[2]->getIndex() ] ];
        Point3d * p5 = newPoints[ edgeDivised_newPointIndex[ edge3->getIndex() ] ];
        // 0 1 5
        newFacePoints.push_back( p0 );newFacePoints.push_back( p1 );newFacePoints.push_back( p5 );
        newFaces.push_back( new FigureFace( newFacePoints, getNormal( p0->toVector(), p1->toVector(), p5->toVector() ), newFaces.size() ) );
        newFacePoints.clear();

        // 1 2 3
        newFacePoints.push_back( p1 );newFacePoints.push_back( p2 );newFacePoints.push_back( p3 );
        newFaces.push_back( new FigureFace( newFacePoints, getNormal( p1->toVector(), p2->toVector(), p3->toVector() ), newFaces.size() ) );
        newFacePoints.clear();

        // 3 4 5
        newFacePoints.push_back( p3 );newFacePoints.push_back( p4 );newFacePoints.push_back( p5 );
        newFaces.push_back( new FigureFace( newFacePoints, getNormal( p3->toVector(), p4->toVector(), p5->toVector() ), newFaces.size() ) );
        newFacePoints.clear();

        // 1 3 5
        newFacePoints.push_back( p1 );newFacePoints.push_back( p3 );newFacePoints.push_back( p5 );
        newFaces.push_back( new FigureFace( newFacePoints, getNormal( p1->toVector(), p3->toVector(), p5->toVector() ), newFaces.size() ) );
        newFacePoints.clear();
    }
    Figure * result = new Figure( new vec3(0,0,0), new vec3(1,1,1), new vec3(0,0,0), new Point3d(1,1,1,-1), false, false );
	result->setPoints( newPoints );
	result->setFaces( newFaces );

	return result;
}

void DrawAxes();
void drawFigureFaces( Figure * f, GLenum mode, bool considerLight, Point3d * couleur = new Point3d(0,0,0,-1) );

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);

	//glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//rotation de la scene suivant les mouvements de la souris
	glRotatef(-angleY,0.0f,0.0f,1.0f);
	glRotatef(angleX,0.0f,1.0f,0.0f);

	/*
	drawVolumeForm( s, white, GL_POLYGON, drawVoxelEdges, considerLight, lightPosition );
	drawVolumeForm( s2, red, GL_POLYGON, drawVoxelEdges, considerLight, lightPosition );
	drawVolumeForm( c, green, GL_POLYGON, drawVoxelEdges, considerLight, lightPosition );
	*/
	if( !showMesh ){
        drawSpace( selectedVoxels, space, voxel_dimension, white, GL_POLYGON, drawVoxelEdges, considerLight, lightPosition );
        //drawSpace( tree, space, voxel_dimension, white, GL_POLYGON, drawVoxelEdges, considerLight, lightPosition );
	}
	else{
        if( !drawVoxelEdges ){
            // Draw Figure
            if( !showSimplification ){
                drawFigureFaces( f, GL_TRIANGLES, considerLight, f->getCouleur() );
                drawFigureFaces( f, GL_LINE_STRIP, considerLight, color_black );
            }
            else{
                drawFigureFaces( fSimp, GL_TRIANGLES, considerLight, f->getCouleur() );
                drawFigureFaces( fSimp, GL_LINE_STRIP, considerLight, color_black );
            }
        }
        else{
            // Draw Voxels
            drawSpace( selectedVoxels, space, voxel_dimension, white, GL_POLYGON, false, considerLight, lightPosition );
        }
	}

	DrawAxes();
	glutSwapBuffers();// echange des buffers
	//glFlush();
}

void drawFigureFaces( Figure * f, GLenum mode, bool considerLight, Point3d * couleur ){
	std::deque<FigureFace*> faces = f->getFaces();
	vec3 * scale = f->getScale();
    vec3 * translation = f->getTranslation();

    for(int i=0; i<faces.size(); i++)
    {
        glPushMatrix();
        //glBegin(GL_LINE_LOOP);
        //glBegin(GL_TRIANGLES);
        //glBegin(GL_POINTS);
        //glBegin(GL_POLYGON);
        glBegin(mode);

        for(int j=0; j< faces[i]->getPoints().size(); j++)
        {
            Point3d * p = faces[i]->getPoints()[j];

            if( considerLight ){
                // Color based on light
                float lightFactor = lightPosition.soustraction( p->toVector() ).normalized().produitScalaire( faces[i]->getNormal().normalized() );
                if( lightFactor < 0.1f ){
                    // simulating environment light
                    lightFactor = 0.1f;
                }
                glColor3f(lightFactor*couleur->getX(), lightFactor*couleur->getY(), lightFactor*couleur->getZ());
            }
            else{
                glColor3f(couleur->getX(), couleur->getY(), couleur->getZ());
            }

            glVertex3f(
              /*(p->getX()*scale->getX())+translation->getX(),
              (p->getY()*scale->getY())+translation->getY(),
              (p->getZ()*scale->getZ())+translation->getZ()*/
                p->getX(), p->getY(), p->getZ()
            );
        }
        // Closing figure
        Point3d * p = faces[i]->getPoints()[0];
        glVertex3f(p->getX(), p->getY(), p->getZ());

        glEnd();
        glPopMatrix();
    }
}

void DrawAxes(){
    glColor3f(0.0,1.0,0.0); //Y vert
    glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(0,20,0);
    glEnd();

    glColor3f(0.0,0.0,1.0); //Z bleu
    glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(0,0,20);
    glEnd();

    glColor3f(1.0,0.0,0.0); //X rouge
    glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(20,0,0);
	glEnd();
}

void useTool( int value, spaceVoxel *** spaceMatrix ){
    if( showTool ){
        FOR( w, spaceW ){
            FOR( h, spaceH){
                FOR( d, spaceD ){
                    if( tool->voxelVeticesInside( *(spaceMatrix[w][h][d].voxel) ).size() > 0 || tool->isCenterInsideVoxel( *(spaceMatrix[w][h][d].voxel) ) ){
                        spaceMatrix[w][h][d].value += value;
                    }
                }
            }
        }

        selectVisibleVoxels( spaceMatrix );
    }
}

/* Au cas ou la fenetre est modifiee ou deplacee */
void reshape(int w, int h)
{
   glViewport(0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if( !showMesh ){
        glOrtho(-5, 5, -5, 5, -5, 5);
   }
   else{
        int orthoD = 12;
        glOrtho(-orthoD, orthoD, -orthoD, orthoD, -orthoD, orthoD);
   }
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y)
{
   switch (key) {
    case 'g': case 'G':
        drawVoxelEdges = !drawVoxelEdges;
        break;
    case 's': case 'S':
        showSimplification = !showSimplification;
        break;
    case 'd': case 'D':
        if( showMesh ){
            fSimp = SubDivision( fSimp );
        }
        break;
    case '-':
        voxel_dimension *= 2.;
        fillSpace();
        break;
    case '+':
        voxel_dimension /= 2.;
        fillSpace();
        break;

    // TOOL
    // ENABLE
    case 't': case 'T':
        showTool = !showTool;
        break;
    // USE
    case '1':   // Add
        if( showTool ) useTool( -1, space );
        break;
    case '3':   // Remove
        if( showTool ) useTool( 1, space );
        break;
    case '7':   // Add
        showThreshold-=1;
        selectVisibleVoxels( space );
        break;
    case '9':   // Remove
        showThreshold+=1;
        selectVisibleVoxels( space );
        break;
    // MOVE
    // UP DOWN
    case '8':
        tool->setCenter( vec3( tool->getCenter().getX(), tool->getCenter().getY()+toolMoveStep, tool->getCenter().getZ() ) );
        break;
    case '2':
        tool->setCenter( vec3( tool->getCenter().getX(), tool->getCenter().getY()-toolMoveStep, tool->getCenter().getZ()) );
        break;
    // LEFT RIGHT
    case '4':
        tool->setCenter( vec3( tool->getCenter().getX()-toolMoveStep, tool->getCenter().getY(), tool->getCenter().getZ() ) );
        break;
    case '6':
        tool->setCenter( vec3( tool->getCenter().getX()+toolMoveStep, tool->getCenter().getY(), tool->getCenter().getZ() ) );
        break;
    // DIAGONAL DOWN
    /*case '1':
        tool->setCenter( vec3( tool->getCenter().getX()-toolMoveStep, tool->getCenter().getY()-toolMoveStep, tool->getCenter().getZ() ) );
        break;
    case '3':
        tool->setCenter( vec3( tool->getCenter().getX()+toolMoveStep, tool->getCenter().getY()-toolMoveStep, tool->getCenter().getZ() ) );
        break;
    // DIAGONAL TOP
    case '7':
        tool->setCenter( vec3( tool->getCenter().getX()-toolMoveStep, tool->getCenter().getY()+toolMoveStep, tool->getCenter().getZ() ) );
        break;
    case '9':
        tool->setCenter( vec3( tool->getCenter().getX()+toolMoveStep, tool->getCenter().getY()+toolMoveStep, tool->getCenter().getZ() ) );
        break;*/

   case ESC:
      exit(0);
      break;
   default :
       break;
   }

   glutPostRedisplay();
}

GLvoid manageMouse(int x, int y)
{
	angleX=x*720/WIDTH;
	angleY=-(y*180/HEIGHT-90)*4;

	glutPostRedisplay();
}

int main(int argc, char **argv)
{
   glutInitWindowSize(WIDTH, HEIGHT);
   glutInit(&argc, argv);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
   glutCreateWindow("Volume");
   init();
   glEnable(GL_DEPTH_TEST);
   glutReshapeFunc(reshape);
   glutPassiveMotionFunc(manageMouse);
   glutKeyboardFunc(keyboard);
   glutDisplayFunc(display);
   glutMainLoop();
   return 0;
}
