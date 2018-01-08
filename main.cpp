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
#include "vec3.h"
#include "utils.h"

#include "CubeVolume.h"
#include "SphereVolume.h"
#include "CylinderVolume.h"
#include "FormTree.h"

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

// Parameters
bool drawVoxelEdges = false;
bool considerLight = true;
vec3 lightPosition(5,0,-5);

std::deque<FormTree*> form_list;
SphereVolume * s, * s2, * s3;
CubeVolume * c, * c2, * c3;
CylinderVolume * cy;

vec3 white( 1,1,1 );
vec3 red( 1,0,0 );
vec3 green( 0,1,0 );
vec3 blue( 0,0,1 );

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

    form_list.push_back( new FormTree(
                        FormTree::Union,
                        new FormTree( c ),
                        new FormTree(
                            FormTree::Intersection,
                            new FormTree(c2),
                            new FormTree(s)
                            )
                        )
                    );
    form_list.push_back( new FormTree(
                            FormTree::Difference,
                            new FormTree( c3 ),
                            new FormTree(
                                s2
                                )
                            )
                        );

    form_list.push_back( new FormTree( cy ) );
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

void drawFormTree( FormTree * formTree, float voxelDimension, vec3 color, GLenum mode, bool drawVoxelEdges, bool considerLight, vec3 lightPosition ){
    CubeVolume box = formTree->getBoundingBox();
//drawVolumeForm( &box, voxelDimension, red, GL_LINE_LOOP, drawVoxelEdges, considerLight, lightPosition );
    float boxCenterX = box.getCenter().getX();
    float boxCenterY = box.getCenter().getY();
    float boxCenterZ = box.getCenter().getZ();

    FOR( w, ceil(box.getWidth()/voxelDimension)+1 ){
        FOR( h, ceil(box.getHeight()/voxelDimension)+1 ){
            FOR( d, ceil(box.getDepth()/voxelDimension)+1 ){

                Voxel v( vec3( boxCenterX-box.getWidth()/2. + (w+0.5)*voxelDimension,
                              boxCenterY-box.getHeight()/2. + (h+0.5)*voxelDimension,
                              boxCenterZ-box.getDepth()/2. + (d+0.5)*voxelDimension ),
                        voxelDimension, voxelDimension, voxelDimension );

                if( formTree->voxelVeticesInside( v ) || formTree->isCenterInsideVoxel( v ) ){
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

void DrawAxes();

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
	FOR(i, form_list.size()){
        drawFormTree( form_list[i], 0.2, white, GL_POLYGON, drawVoxelEdges, considerLight, lightPosition );
	}

	DrawAxes();
	glutSwapBuffers();// echange des buffers
	//glFlush();
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
   glOrtho(-5, 5, -5, 5, -5, 5);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y)
{
   switch (key) {
    case '0': case '1': case '2': case '3':
        //selectedControlPoint = key - '0';
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
