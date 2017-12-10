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

#define Voxel CubeVolume

/* au cas ou M_PI ne soit defini */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ESC 27

float tx=0.0;
float ty=0.0;

SphereVolume * s;
CubeVolume * c;

vec3 white( 1,1,1 );

/* initialisation d'OpenGL*/
static void init(void)
{
	glClearColor(0.0, 0.0, 0.0, 0.0);

	s = new SphereVolume( vec3( 0,0,0 ), 2. );
	c = new CubeVolume( vec3( 0,0,0 ), 2, 2, 2 );
}

void drawSquare( vec3 p1, vec3 p2, vec3 p3, vec3 p4, vec3 color ){
    glColor3f(color.getX(),color.getY(),color.getZ());
    glBegin(GL_POLYGON);
        glVertex3f( p1.getX(), p1.getY(), p1.getZ() );
        glVertex3f( p2.getX(), p2.getY(), p2.getZ() );
        glVertex3f( p3.getX(), p3.getY(), p3.getZ() );
        glVertex3f( p4.getX(), p4.getY(), p4.getZ() );
	glEnd();
}

void drawVoxel( Voxel voxel, vec3 color )
{
    std::deque<vec3> points = voxel.getVertices();

    // Front
	drawSquare( points[0], points[1], points[2], points[3], color );
	// Back
	//drawSquare( points[4], points[5], points[6], points[7], color );
}

void drawVolumeForm( VolumeForm * form, vec3 color ){
    FOR( w, 10 ){
        FOR( h, 10 ){
            FOR( d, 10 ){
                Voxel v( vec3( w-5, h-5, d-5 ), 1, 1, 1 );
                if( form->voxelVeticesInside( v ).size() > 0 || form->isCenterInsideVoxel( v ) ){
                    drawVoxel( v, color );
                }
            }
        }
    }
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	drawVolumeForm( c, white );

	glFlush();
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

int main(int argc, char **argv)
{
   glutInitWindowSize(400, 400);
   glutInit(&argc, argv);
   glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
   glutCreateWindow("Volume");
   init();
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutDisplayFunc(display);
   glutMainLoop();
   return 0;
}
