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
float voxel_dimension = 0.2;
// Parameters Mesh
bool showMesh = true;
bool showSimplification = true;

FormTree * tree;
SphereVolume * s, * s2, * s3;
CubeVolume * c, * c2, * c3;
CylinderVolume * cy;
Figure * f, * fSimp;

struct spaceVoxel{
    public :
    Voxel * voxel;
    int value;
    std::deque< Point3d * > vertices;
};

spaceVoxel *** space;
int spaceW, spaceH, spaceD;

void fillSpace( FormTree * formTree, float voxelDimension );
void fillSpaceWithFigure( Figure * figure, float voxelDimension );
Figure * Simplification( Figure * figure, spaceVoxel*** spaceMatrix );

/* initialisation d'OpenGL*/
static void init(void)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);

	s = new SphereVolume( vec3( 0,1.,0 ), 1. );
	c = new CubeVolume( vec3( 0,-0.5,0 ), 1, 1, 1 );
	c2 = new CubeVolume( vec3( 0,0,0 ), 2, 2, 2 );

	s2 = new SphereVolume( vec3( -2,0,0 ), 1. );
	c3 = new CubeVolume( vec3( -3,0,0 ), 2, 2, 2 );

	cy = new CylinderVolume( vec3( 3,0,0 ), 1, 1 );

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
        tree = new FormTree(
                    FormTree::Union,
                    t1,
                    new FormTree(
                        FormTree::Union,
                        t2,
                        t3
                        )
                    );
        fillSpace( tree, voxel_dimension );
    }else{
        voxel_dimension = 0.5;
        fillSpaceWithFigure( f, voxel_dimension );

        fSimp = Simplification( f, space );
        //tree = new FormTree( fSimp );
        //fillSpaceWithFigure( fSimp, voxel_dimension );
    }
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

void fillSpace( FormTree * formTree, float voxelDimension ){
    CubeVolume box = formTree->getBoundingBox();
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
                if( formTree->voxelVeticesInside( *v ) || formTree->isCenterInsideVoxel( *v ) ){
                    space[w][h][d].value = 1;
                }
                else{
                    space[w][h][d].value = 0;
                }
            }
        }
    }
}

void fillSpaceWithFigure( Figure * figure, float voxelDimension ){
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
                    space[w][h][d].value = 1;
                    space[w][h][d].vertices = vertices;
                }
                else{
                    space[w][h][d].value = 0;
                }
            }
        }
    }
}

void drawSpace( FormTree * formTree, spaceVoxel*** spaceMatrix, float voxelDimension, vec3 color, GLenum mode, bool drawVoxelEdges, bool considerLight, vec3 lightPosition ){
    CubeVolume box = formTree->getBoundingBox();
//drawVolumeForm( &box, voxelDimension, red, GL_LINE_LOOP, drawVoxelEdges, considerLight, lightPosition );
    float boxCenterX = box.getCenter().getX();
    float boxCenterY = box.getCenter().getY();
    float boxCenterZ = box.getCenter().getZ();

    FOR( w, spaceW ){
        FOR( h, spaceH){
            FOR( d, spaceD ){

                if( spaceMatrix[w][h][d].value > 0){
                    drawVoxel( *(spaceMatrix[w][h][d].voxel), color, mode, considerLight, lightPosition );
                }
                else{
                    if( drawVoxelEdges ){
                        drawVoxel( *(spaceMatrix[w][h][d].voxel), color, GL_LINE_LOOP, false, lightPosition );
                    }
                }
            }
        }
    }
}

Figure * Simplification( Figure * figure, spaceVoxel*** spaceMatrix )
{
    std::deque<Point3d*> newPoints;
    std::deque<int> origPointsCenters( figure->getPoints().size() );
	std::deque<FigureFace*> newFaces;

    // For each voxel, choose the center
    FOR( w, spaceW ){
        FOR( h, spaceH){
            FOR( d, spaceD ){
                if( spaceMatrix[w][h][d].value > 0 && spaceMatrix[w][h][d].vertices.size() > 0){
                    // Calculate average
                    vec3 center = spaceMatrix[w][h][d].vertices[0]->toVector();
                    for( int i=1; i<spaceMatrix[w][h][d].vertices.size(); i++ ){
                        center.addition( spaceMatrix[w][h][d].vertices[i]->toVector() );
                    }
                    center = center.division( spaceMatrix[w][h][d].vertices.size() );
                    int newPointIndex = newPoints.size();
                    newPoints.push_back( new Point3d( center.getX(), center.getY(), center.getZ(), newPointIndex ) );

                    // Storing center for each original point
                    for( int i=0; i<spaceMatrix[w][h][d].vertices.size(); i++ ){
                        origPointsCenters[ spaceMatrix[w][h][d].vertices[i]->getIndex() ] = newPointIndex;
                    }
                }
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
            newPointsFace.push_back( newPoints[ centerIndex ] );
            pointsFaceCenter.push_back( centerIndex );

            // Check if some other original point has the same center
            /*FOR( k, pointsFaceCenter.size()-1 ){
                if( pointsFaceCenter[k] == centerIndex ){
                    // not include face
                    includeFace = false;
                    break;
                }
            }*/
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
        drawSpace( tree, space, voxel_dimension, white, GL_POLYGON, drawVoxelEdges, considerLight, lightPosition );
	}
	else{
        if( !showSimplification ){
            drawFigureFaces( f, GL_TRIANGLES, considerLight, f->getCouleur() );
            drawFigureFaces( f, GL_LINE_STRIP, considerLight, color_black );
        }
        else{
            drawFigureFaces( fSimp, GL_TRIANGLES, considerLight, f->getCouleur() );
            drawFigureFaces( fSimp, GL_LINE_STRIP, considerLight, color_black );
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

//std::cout << "====" << std::endl;
        for(int j=0; j< faces[i]->getPoints().size(); j++)
        {
/*std::cout << faces[i]->getPoints()[j]->getIndex() << std::endl;
std::cout << faces[i]->getPoints()[j]->toString() << std::endl;*/
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

/* Au cas ou la fenetre est modifiee ou deplacee */
void reshape(int w, int h)
{
   glViewport(0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glOrtho(-6, 6, -6, 6, -6, 6);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y)
{
   switch (key) {
    case '0': case '1': case '2': case '3':
        //selectedControlPoint = key - '0';
        break;
    case 'g': case 'G':
        drawVoxelEdges = !drawVoxelEdges;
        break;
    case 's': case 'S':
        showSimplification = !showSimplification;
        break;

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
