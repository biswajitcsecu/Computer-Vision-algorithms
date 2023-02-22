#include <cstdlib>
#include <GL/gl.h>
#include <GL/glext.h>
#include <glut.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


    using namespace std;
    using namespace cv;

    GLuint texture;
    GLfloat angle = 0.0;

    int loadTexture(Mat image, GLuint *text)
    {
        if (image.empty())
                return -1;
        
        glGenTextures(1, text);
        glBindTexture( GL_TEXTURE_2D, *text );

        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols,image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image.data);
        return 0;

        }


    void plane (void){
        glBindTexture( GL_TEXTURE_2D, texture );
        glRotatef( angle, 1.0f, 1.0f, 1.0f );

        glBegin (GL_QUADS);
                glTexCoord2d(0.0, 0.0);
                glVertex2d(-1.0, -1.0);
                glTexCoord2d(1.0, 0.0); 
                glVertex2d(+1.0, -1.0);
                glTexCoord2d(1.0, 1.0);
                glVertex2d(+1.0, +1.0);
                glTexCoord2d(0.0, 1.0);
                glVertex2d(-1.0, +1.0);	
        glEnd();
    }	

    void display (void){
        glClearColor (0.0,0.0,0.0,1.0);
        glClear (GL_COLOR_BUFFER_BIT);
        glLoadIdentity();
        gluLookAt (0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        glEnable( GL_TEXTURE_2D );
        plane();
        glutSwapBuffers();
        angle =angle+0.005;
    }

    void FreeTexture( GLuint texture ){
        glDeleteTextures( 1, &texture );
    }

    void reshape (int w, int h){
        glViewport (0, 0, (GLsizei)w, (GLsizei)h);
        glMatrixMode (GL_PROJECTION);
        glLoadIdentity ();
        gluPerspective (60, (GLfloat)w / (GLfloat)h, 1.0, 100.0);
        glMatrixMode (GL_MODELVIEW);
    }

    int main (int argc, char **argv){
        glutInit (&argc, argv);
        glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
        glutInitWindowSize (800, 800);
        glutInitWindowPosition (100, 100);
        glutCreateWindow ("OpenCV Texture");
        glutDisplayFunc (display);
        glutIdleFunc (display);
        glutReshapeFunc (reshape);

        Mat image;
        image = imread("src.jpg");
        loadTexture(image, &texture);
        glutMainLoop ();
        FreeTexture( texture );	
        return 0;
    }	


