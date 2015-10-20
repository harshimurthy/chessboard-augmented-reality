#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <string>


using namespace std;
cv::VideoCapture *cap = NULL;
int width= 640;
	int height= 480; 
cv::Mat image;

double fovx,fovy,fx,fy,px,py,aspect,k1,k2,p1,p2,k3;
vector<cv::Point3f> objpoint;
vector<cv::Point2f> corners;
cv::Mat cameramatrix;
cv::Mat distortionCoefficients;
cv::Size patternsize(8,6);
bool teapot=false;
bool sphere=false;
// a useful function for displaying your coordinate system
void drawAxes(float length)
{
  glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT) ;

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) ;
  glDisable(GL_LIGHTING) ;

  glBegin(GL_LINES) ;
  glColor3f(1,0,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(length,0,0);

  glColor3f(0,1,0) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,length,0);

  glColor3f(0,0,1) ;
  glVertex3f(0,0,0) ;
  glVertex3f(0,0,length);
  glEnd() ;


  glPopAttrib() ;
}

void drawtea()
{
glPushMatrix();	
glColor3f(0.0,1.0,0.0);
    glTranslatef(3.5, 2.5, 0.0); 
    glRotatef(-90,1.,0.,0.);    
	glutWireTeapot(1.0);
  
  glPopMatrix();
glFlush();
}
  
void drawsphere()
{

glPushMatrix();
//glScalef(1.0,-1.0,-1.0);
glColor3f(0.0,0.0,1.0);
for(int i=0;i<6;i++)
{
	for(int j=0;j<8;j++)
	{
		glPushMatrix();
		glTranslatef(j*1.0,0.0,0.0);
		glutSolidSphere(0.5, 2, 2);
		glPopMatrix();
	}
	
	glTranslatef(0.0,1.0,0.0);
	glFlush();

}
glPopMatrix();
glFlush();
}

void display()
{
  // clear the window
  glClear( GL_COLOR_BUFFER_BIT );
 
  // show the current camera frame
   (*cap)>>image;
  //based on the way cv::Mat stores data, you need to flip it before displaying it
  cv::Mat tempimage;
  cv::flip(image, tempimage, 0);
  glDrawPixels( tempimage.size().width, tempimage.size().height, GL_BGR, GL_UNSIGNED_BYTE, tempimage.ptr() );

  //////////////////////////////////////////////////////////////////////////////////
  // Here, set up new parameters to render a scene viewed from the camera.

  //set viewport
  glViewport(0, 0, tempimage.size().width, tempimage.size().height);

  //set projection matrix using intrinsic camera params
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  //gluPerspective is arbitrarily set, you will have to determine these values based
  //on the intrinsic camera parameters
  gluPerspective(fovy, float(width)/float(height), 0.01, 50); 

  int chessflags=0;
  cv::Mat gimage;
  cvtColor(image,gimage,CV_BGR2GRAY);
  bool check = findChessboardCorners(gimage,patternsize,corners,chessflags|cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE|cv::CALIB_CB_FAST_CHECK);
  if(check)
	{
		cv::Mat rvec;
		cv::Mat tvec;
		cv::Mat rotation;
		cv::Mat viewMatrix=cv::Mat::zeros(4,4,CV_64FC1);
		cv::solvePnP(objpoint,corners,cameramatrix,distortionCoefficients,rvec,tvec);
		cout<<"\n rvec:"<<rvec;
		cout<<"\n tvec:"<<tvec;
		Rodrigues(rvec, rotation);
		cout<<"\n rotation:"<<rotation<<endl;
	

            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    viewMatrix.at<double>(row, col) = rotation.at<double>(row, col);
                }
                viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
            }
            viewMatrix.at<double>(3,3)=1.0f;
            //cout<<"viewmatrix"<<viewMatrix<<endl;
            cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
            cvToGl.at<double>(0, 0) = 1.0f;
            cvToGl.at<double>(1, 1) = -1.0f;
            cvToGl.at<double>(2, 2) = -1.0f;
            cvToGl.at<double>(3, 3) = 1.0f;
            viewMatrix = cvToGl * viewMatrix;

            
            //Mat glViewMatrix = cv::Mat::zeros(4, 4, CV_64F);
		
            cv::Mat glViewMatrix;
		//viewMatrix.at<double>(3,3)=1.0f;
            cv::transpose(viewMatrix , glViewMatrix);
            cout<<"glViewMatrix"<<glViewMatrix<<endl;

            
            glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
            glLoadMatrixd(&glViewMatrix.at<double>(0, 0));
		
		glPushMatrix();
  //drawAxes(1.0);		
	
//glScalef(1.0,-1.0,-1.0);
if(teapot)
	drawtea();
if(sphere)
	drawsphere();
glPopMatrix();}

  // show the rendering on the screen
  glutSwapBuffers();

  // post the next redisplay
 

	

}

void reshape( int w, int h )
{
  // set OpenGL viewport (drawable area)
  glViewport( 0, 0, w, h );
}

void mouse( int button, int state, int x, int y )
{
  if ( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
    {

    }
}

void keyboard( unsigned char key, int x, int y )
{
  switch ( key )
    {
    case 'q':
      // quit when q is pressed
      exit(0);
      break;

    case 't': if(teapot==true)
		{
		teapot=false;
		sphere=true;
		}
		else
		{
		teapot=true;
		sphere=false;
		}
	
    case 's': if(sphere==false)
		{
		sphere=true;
		teapot=false;
		}
		else
		{
		sphere=false;
		teapot=true;
		}

    default: cout<<"invalid";
      break;
    }
}

void idle()
{
  // grab a frame from the camera
  (*cap) >> image;
 glutPostRedisplay();
}

int main( int argc, char **argv )
{
  int w,h;
   ifstream calibration_file;
  if ( argc == 1 ) {
    // start video capture from camera
    cap = new cv::VideoCapture(0);
	calibration_file.open("cam.txt");
	calibration_file >> fovx >> fovy >> fx >>fy>>px>> py >>aspect>>k1>>k2>>p1>>p2>>k3;
	
  } else if ( argc == 2 ) {
    // start video capture from file
    cap = new cv::VideoCapture(argv[1]);
     cap = new cv::VideoCapture(argv[1]);
	calibration_file.open("input.txt");
	calibration_file >> fovx >> fovy >> fx >>fy>>px>> py >>aspect>>k1>>k2>>p1>>p2>>k3;
	
  } else {
    fprintf( stderr, "usage: %s [<filename>]\n", argv[0] );
    return 1;
  }

  // check that video is opened
  if ( cap == NULL || !cap->isOpened() ) {
    fprintf( stderr, "could not start video capture\n" );
    return 1;
  }
  cout<<"\n fovx"<<fovx<<"\n fovy"<<fovy<<"\n fx:"<<fx<<"\n fy:"<<fy<<"\n px:"<<px<<"\n py:"<<py<<"\n aspect:"<<aspect<<"\n k1:"<<k1<<"\n k2:"<<k2<<"\n k3:"<<k3<<"\n p1:"<<p1<<"\n p2:"<<p2;
calibration_file.close();
  // get width and height
  w = (int) cap->get( CV_CAP_PROP_FRAME_WIDTH );
  h = (int) cap->get( CV_CAP_PROP_FRAME_HEIGHT );
  // On Linux, there is currently a bug in OpenCV that returns 
  // zero for both widsth and height here (at least for video from file)
  // hence the following override to global variable defaults: 
  width = w ? w : width;
  height = h ? h : height;

double dist[] = {k1,k2,p1,p2,k3};
distortionCoefficients=cv::Mat(5,1,CV_64FC1,dist);
cout<<"\n dist coeff:"<<distortionCoefficients;
double camera[] = {fx,0.,width/2.0,
0.,fy,height/2.0,
0.,0.,1.};
cameramatrix=cv::Mat(3,3,CV_64FC1,camera);
cout<<"\n camera matrix:"<<cameramatrix<<endl;

for( int i = 0; i < 6; i++ )
	for( int j = 0; j < 8; j++ )
    		objpoint.push_back(cv::Point3f(j,i,0));
cout<<"Obj points:"<<objpoint<<endl;
    	cout<<"\n width:"<<width;
	cout<<"\n height:"<<height;	
  // initialize GLUT
  glutInit( &argc, argv );
  //glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE );
  glutInitWindowPosition( 20, 20 );
  glutInitWindowSize( width, height );
  glutCreateWindow( "OpenGL / OpenCV Example" );

  // set up GUI callback functions
  glutDisplayFunc( display );
  glutReshapeFunc( reshape );
  glutMouseFunc( mouse );
  glutKeyboardFunc( keyboard );
  glutIdleFunc( idle );

  // start GUI loop
  glutMainLoop();

  return 0;
}
