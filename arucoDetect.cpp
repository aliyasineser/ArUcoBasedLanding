#define _USE_MATH_DEFINES
// OpenCV libraries
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/shape.hpp"
#include <opencv2/aruco.hpp>

// Std libraries
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <list>
#include <ctime>
#include <signal.h>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <unistd.h>

// Graphical libraries
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <GL/glut.h>

//Namespaces
using namespace std;
using namespace cv;

namespace {
const char* about = "ArUco based landing";
const char* keys  = 
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}

// Hyperparameter
// Single board like rasperry or odroid
bool isSingleBoard = true;
bool saveOpenGLVideo = true;
bool saveOpenCVVideo = true;

// Side of the marker, hyperparameter!
float sideOfTheMarker = 0.2;
double halfOfTheSideOfTheMarker = sideOfTheMarker/2;

// OpenGL light info
GLfloat mat_specular[] = {1.0, 1.0,1.0,1.0};
GLfloat mat_shininess[] = {50.0};
GLfloat light_position[] = {1.0,1.0,1.0,0.0};

int limitPoints = 7;  // Store limit, change to desired. 
list<Point3d> points; // Container for points. List is used because easy to add and removal from both sides.
Point3d normalizedCoord; // Only normalized version will be drawn in OpenGL. If no marker detected, draw the last point.

// camera
VideoCapture cap;

//Signal handling for SIGINT ( CTRL + C )
void my_handler(int s){
    if(!isSingleBoard)
        destroyAllWindows();
    cap.release(); // opencv camera release
    //SDL_Quit(); // OpenGL release
    exit(0); 
}


void resize(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (w <= h)
        glOrtho(-2.0,2.0,-2.0*h/w,2.0*h/w,-10.0,10.0);
    else
        glOrtho(-2.0*w/h,2.0*w/h,-2.0,2.0,-10.0,10.0);
    glMatrixMode(GL_MODELVIEW);
}

// Find the mat type. Give the object.type() to use. For debug.
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

// Draw OpenGL scene.
void draw(double x,double y,double z){
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushMatrix();

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(2,2.5,2,0,0,0,0,0,1); // Perspective
    glPointSize(5.0);

    // Draw xyz axis so you can know where you are
    glBegin(GL_LINES);
    // draw line for x axis
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    // draw line for y axis
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    // draw line for Z axis
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();

    // Draw the desired point.
    glBegin(GL_POINTS);
    glColor3ub(180,180,180); // Almost white
    glVertex3d(x,y,z);
    glEnd();

    glFlush();
    if(!isSingleBoard)
        SDL_GL_SwapBuffers();

    glPopMatrix();
    glPopAttrib();
}

// Get Mat from 3D vector. 
cv::Mat DoubleMatFromVec3d(cv::Vec3d in)
{
    cv::Mat mat(3,1, CV_64FC1);
    mat.at <double>(0,0) = in [0];
    mat.at <double>(1,0) = in [1];
    mat.at <double>(2,0) = in [2];

    return mat;
};


// Sorts ids and corners ( since they are paralel arrays )
void sortByIds(vector< vector< Point2f > >& corners, vector< int >& ids){
    bool swapped = true;

    while(swapped){
        swapped = false;
        for(int i=0; i< ids.size()-1; ++i){
            if(ids[i] > ids[i+1]){
                auto tempCorner = corners[i];
                auto tempId = ids[i];

                corners[i] = corners[i+1];
                ids[i] = ids[i+1];

                corners[i+1] = tempCorner;
                ids[i+1] = tempId;
                swapped = true;
            }
        }
    
    }

}

// Given rotation matrix ( calculate with Rodrigues from rvec ), calculates euler angles;
void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}

// Create Homogeneous form for rvec and tvec.
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType ) {
   cv::Mat M;
   cv::Mat R,T;
   R_.copyTo ( R );
   T_.copyTo ( T );
   if ( R.type() ==CV_64F ) {
       assert ( T.type() ==CV_64F );
       cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_64FC1 );
       cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
       if ( R.total() ==3 ) {
           cv::Rodrigues ( R,R33 );
       } else if ( R.total() ==9 ) {
           cv::Mat R64;
           R.convertTo ( R64,CV_64F );
           R.copyTo ( R33 );
       }
       for ( int i=0; i<3; i++ )
           Matrix.at<double> ( i,3 ) =T.ptr<double> ( 0 ) [i];
       M=Matrix;
   } else if ( R.depth() ==CV_32F ) {
       cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_32FC1 );
       cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
       if ( R.total() ==3 ) {
           cv::Rodrigues ( R,R33 );
       } else if ( R.total() ==9 ) {
           cv::Mat R32;
           R.convertTo ( R32,CV_32F );
           R.copyTo ( R33 );
       }
       for ( int i=0; i<3; i++ )
           Matrix.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
       M=Matrix;
   }
   if ( forceType==-1 ) return M;
   else {
       cv::Mat MTyped;
       M.convertTo ( MTyped,forceType );
       return MTyped;
   }
}


int main(int argc, char **argv){
    // camera calibration matrix and distortion coefficients. Filled below, you put your camera values.
    cv::Mat cameraMatrix = cv::Mat(3,3, CV_64F);
    cv::Mat distCoeffs = cv::Mat(1,4, CV_64F);
    cv::Mat rotationMatrix = cv::Mat(3,3, CV_64F);
    cv::Mat translationVector = cv::Mat(1,3, CV_64F);

    int width = 640, height = 480; // Change as you desire, care that most of the cameras are not supporting all ratios!

    VideoWriter opencv_video;
    VideoWriter opengl_video;

    if(!isSingleBoard){
        if(saveOpenCVVideo)
            opencv_video = VideoWriter("opencvOut.avi",CV_FOURCC('M','J','P','G'),30, Size(width,height));
        if(saveOpenGLVideo)
            opengl_video = VideoWriter("openglOut.avi",CV_FOURCC('M','J','P','G'),30, Size(width,height));
    }else{
        if(saveOpenCVVideo)
            opencv_video = VideoWriter("opencvOut.avi",CV_FOURCC('M','J','P','G'), 10, Size(width,height));
        if(saveOpenGLVideo)
            opengl_video = VideoWriter("openglOut.avi",CV_FOURCC('M','J','P','G'), 10, Size(width,height));
    }
    
    if(!isSingleBoard)
        cap = VideoCapture(1); // Lap tops have built-in camera. If you have additional one like me, it may be 1. For built-in, mostly it is 0
    else
        cap = VideoCapture(0); // For singleBoard pc, you can list your cameras with v4l2 and use the one the needed. Mine is 2 at the moment.

    // Signal is needed because capture should be released in any case. I use sigint for closing, I handled it.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // OpenGL initiation part!
    if(!isSingleBoard){
        SDL_Init(SDL_INIT_VIDEO);
        atexit(SDL_Quit);
        SDL_WM_SetCaption("Point Cloud", NULL);
        SDL_SetVideoMode(width,height, 32, SDL_OPENGL); 
    }

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(70,(double)640/480,1,1000);
    
    // Set the resolution. Camera should support the resolution and you need to multiply your camera matrix with the ratio. 
    cap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,height);
    
    /*********************************CAMERA CALIBRATION PARAMETERS********************************************/
    /*
    //Yasin lap top camera
    cameraMatrix.at<double>(0,0) = 5.6010964821423499e+02; 
    cameraMatrix.at<double>(0,1) = 0.;
    cameraMatrix.at<double>(0,2) = 2.4071677032876121e+02; 
    cameraMatrix.at<double>(1,0) = 0.;
    cameraMatrix.at<double>(1,1) = 5.5645257571732964e+02; 
    cameraMatrix.at<double>(1,2) = 2.4544622899451417e+02;
    cameraMatrix.at<double>(2,0) = 0.;
    cameraMatrix.at<double>(2,1) = 0.;
    cameraMatrix.at<double>(2,2) = 1.;

    distCoeffs.at<double>(0,0) = 8.9028808110176924e-02;
    distCoeffs.at<double>(0,1) = 3.4434204639405364e-01;
    distCoeffs.at<double>(0,2) = -5.6353742972864032e-03; 
    distCoeffs.at<double>(0,3) = -4.4143095826370313e-02;
    distCoeffs.at<double>(0,4) = -5.3672160799182489e-01; 
    
    //Left Camera
    cameraMatrix.at<double>(0,0) = 2.8831113489990753e+02; //2.8472479568921011e+02
    cameraMatrix.at<double>(0,1) = 0.;
    cameraMatrix.at<double>(0,2) = 3.3163864833517891e+02; //3.3094050756452287e+02
    cameraMatrix.at<double>(1,0) = 0.;
    cameraMatrix.at<double>(1,1) = 2.8841686192769617e+02; //2.8461395687793708e+02
    cameraMatrix.at<double>(1,2) = 2.2631321843481521e+02; //2.2620082298460346e+02
    cameraMatrix.at<double>(2,0) = 0.;
    cameraMatrix.at<double>(2,1) = 0.;
    cameraMatrix.at<double>(2,2) = 1.;

    distCoeffs.at<double>(0,0) = -3.4021184396119591e-01; //-3.6631933640473163e-01
    distCoeffs.at<double>(0,1) = 1.2734659667603954e-01; //1.9504149383517827e-01
    distCoeffs.at<double>(0,2) = 2.5903170293088386e-04; //-4.8588171299185535e-04
    distCoeffs.at<double>(0,3) = -1.0025396099026702e-04; //2.7179556321942192e-04
    distCoeffs.at<double>(0,4) = -2.2112880530223068e-02; //-6.5477129929546005e-02
    
    //Right Camera
    cameraMatrix.at<double>(0,0) = 2.8772691423906861e+02;
    cameraMatrix.at<double>(0,1) = 0.;
    cameraMatrix.at<double>(0,2) = 3.3475781030131145e+02;
    cameraMatrix.at<double>(1,0) = 0.;
    cameraMatrix.at<double>(1,1) = 2.8800149297988565e+02;
    cameraMatrix.at<double>(1,2) = 2.3043582896947086e+02;
    cameraMatrix.at<double>(2,0) = 0.;
    cameraMatrix.at<double>(2,1) = 0.;
    cameraMatrix.at<double>(2,2) = 1.;

    distCoeffs.at<double>(0,0) = -3.3587960975322206e-01;
    distCoeffs.at<double>(0,1) = 1.1914816540239793e-01;
    distCoeffs.at<double>(0,2) = 1.4538720148265622e-03;
    distCoeffs.at<double>(0,3) = 5.6087663176624849e-05;
    distCoeffs.at<double>(0,4) = -1.8950809948691305e-02;

    */

    //Rotation Matrix and Translation Vector for the camera pair
    rotationMatrix.at<double>(0,0) = 9.9989675895720176e-01;
    rotationMatrix.at<double>(0,1) = 7.9448289867638973e-03;
    rotationMatrix.at<double>(0,2) = -1.1972932775832361e-02;
    rotationMatrix.at<double>(1,0) = -8.3397399926721516e-03;
    rotationMatrix.at<double>(1,1) = 9.9941050897513883e-01;
    rotationMatrix.at<double>(1,2) = -3.3302902079374093e-02;
    rotationMatrix.at<double>(2,0) = 1.1701288977636155e-02;
    rotationMatrix.at<double>(2,1) = 3.3399314999335399e-02;
    rotationMatrix.at<double>(2,2) = 9.9937358660004461e-01;

    translationVector.at<double>(0,0) = 1.9648803189425440e-01;
    translationVector.at<double>(1,0) = 9.0682265306119339e-04;
    translationVector.at<double>(2,0) = 2.3474219308294102e-03;

    // BLACK BOX RIGHT CAMERA
    cameraMatrix.at<double>(0,0) = 8.7030170819798286e+02;
    cameraMatrix.at<double>(0,1) = 0.;
    cameraMatrix.at<double>(0,2) = 320.;
    cameraMatrix.at<double>(1,0) = 0.;
    cameraMatrix.at<double>(1,1) = 8.7030170819798286e+02;
    cameraMatrix.at<double>(1,2) = 240.;
    cameraMatrix.at<double>(2,0) = 0.;
    cameraMatrix.at<double>(2,1) = 0.;
    cameraMatrix.at<double>(2,2) = 1.;

    distCoeffs.at<double>(0,0) = -4.7851541557875366e-01;
    distCoeffs.at<double>(0,1) = 1.0494014645561520e+00;
    distCoeffs.at<double>(0,2) = 0.;
    distCoeffs.at<double>(0,3) = 0.;
    distCoeffs.at<double>(0,4) = -3.0666278646347642e+00;
    /*****************************************************************************/

    /*   If you changed your resolution, you need to multiply your camera matrix with the ratio!
    cameraMatrix.at<double>(0,0) = cameraMatrix.at<double>(0,0) * (width / 640);
    cameraMatrix.at<double>(0,1) = cameraMatrix.at<double>(0,1)  * (width / 640);
    cameraMatrix.at<double>(0,2) = cameraMatrix.at<double>(0,2)  * (width / 640);

    cameraMatrix.at<double>(1,0) = cameraMatrix.at<double>(1,0)  * (height / 480);
    cameraMatrix.at<double>(1,1) = cameraMatrix.at<double>(1,1) * (height / 480);
    cameraMatrix.at<double>(1,2) = cameraMatrix.at<double>(1,2)  * (height / 480);
    */
   
    // Define the dictionary, 7X7 Aruco. As I remember, detailed markers have better accuracy.
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_7X7_100));

    // detect markers
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create(); // You can change the params if you need.

    // Variables to calculate fps
    long long int frameCounter = 0;
    int tick = 0;
    int fps;
    std::time_t timeBegin = std::time(0); // start the time
    vector< Vec3d > rvecs, tvecs;
    Point3d point;
    while(true){
        Mat img;
        if(!cap.read(img)){
            cerr << "Video camera is not working at the moment. Capture Error!" << endl;
            exit(1);
        }

        frameCounter++; 

        std::time_t timeNow = std::time(0) - timeBegin; // should we update?

        if (timeNow - tick >= 1) // yes
        {
            tick++;
            fps = frameCounter;
            frameCounter = 0;
        }

        vector< vector< Point2f > > corners;
        vector< int > ids;
        aruco::detectMarkers(img, dictionary, corners, ids, params); // ArUco marker detection, you can change the params if you want.

        Mat invertedTvec;
        if(!ids.empty()){
            sortByIds(corners, ids); // This part can be unncesessary, as I remember it is coming in a sorted way.
            aruco::drawDetectedMarkers(img, corners, noArray(), Scalar(0,0,250) ); // Debug
            aruco::estimatePoseSingleMarkers(corners, sideOfTheMarker, cameraMatrix, distCoeffs, rvecs, tvecs); // Pose estimation

            for(int i = 0; i < ids.size(); ++i){
                aruco::drawAxis(img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], halfOfTheSideOfTheMarker); // Debug

                // Invert to ArUco space
                Mat homo = getRTMatrix(DoubleMatFromVec3d(rvecs[i]), DoubleMatFromVec3d(tvecs[i]), -1); // Get the homogeneous form
                Mat invertedMat = homo.inv(); // Get to ArUco space, thanks to Homogeneous matrix
                invertedTvec = invertedMat.col(3); // Get the Tvec column
                invertedTvec.pop_back(); // There reason for pop operation is to remove the 4th element, "1".

                // Get the angles 
                Vec3d eulerAngles;
                cv::Mat rotMat = Mat(3,3,CV_64F);
                cv::Mat tmp = invertedMat(cv::Rect(0,0,3,3)); // First 3x3 part is rotation matrix, always.
                tmp.copyTo(rotMat);
                getEulerAngles(rotMat, eulerAngles); 

                // It may be different.
                // TODO: CHECK IF THE ANGLES ARE IN WRONG PLACES.
                double pitch   = eulerAngles[0];
                double roll = eulerAngles[1];
                double yaw  = eulerAngles[2];
                
                // Debug
                cout << "For id: " << ids[i] << endl;
                cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl << endl;
                // cout << invertedMat << endl;
                cout << "XYZ: " << invertedTvec << endl;

                // Store the point for further operations.
                point = Point3d(invertedTvec.at<double>(0),
                invertedTvec.at<double>(1),
                invertedTvec.at<double>(2));

                points.push_back(point); // Add the new point to the list.
                if(points.size() == limitPoints+1){
                    points.pop_front(); 
                    double xavg=0, yavg=0, zavg=0;
                    for(list<Point3d>::iterator it = points.begin(); it != points.end(); ++it){
                        xavg += it->x/limitPoints;
                        yavg += it->y/limitPoints;
                        zavg += it->z/limitPoints;
                    }
                    normalizedCoord.x = xavg;
                    normalizedCoord.y = yavg;
                    normalizedCoord.z = zavg;
                    
                }else if(points.size() < limitPoints) normalizedCoord = point; // If don't, use the last point.  

                draw(normalizedCoord.x, normalizedCoord.y, normalizedCoord.z); // Marker location is calculated and normalized, draw the scene.
            }
        } else{
            draw(normalizedCoord.x, normalizedCoord.y, normalizedCoord.z); // No marker detected, draw the same scene.
        }

        // draw fps to image
        cv::putText(img, cv::format("Average FPS=%d",fps), cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
        
        if(saveOpenCVVideo) // Save fram for openCV video
            opencv_video << img; 

        if(saveOpenGLVideo){ // Save frame for openGL video
            cv::Mat pixels( height, width, CV_8UC3 );
            glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data );
            cv::Mat cv_pixels( height, width, CV_8UC3 );
            for( int y=0; y<height; y++ ) for( int x=0; x<width; x++ ) 
            {
                cv_pixels.at<cv::Vec3b>(y,x)[2] = pixels.at<cv::Vec3b>(height-y-1,x)[0];
                cv_pixels.at<cv::Vec3b>(y,x)[1] = pixels.at<cv::Vec3b>(height-y-1,x)[1];
                cv_pixels.at<cv::Vec3b>(y,x)[0] = pixels.at<cv::Vec3b>(height-y-1,x)[2];
            }
            opengl_video << cv_pixels; 
        }

        // Show the stream
        if(!isSingleBoard){
            cv::namedWindow("Stream", cv::WINDOW_NORMAL);
            cv::resizeWindow("Stream", width, height);
            cv::imshow("Stream", img);
        
            // Get input. You can do whatever want here.
            char key = waitKey(1);
            if(key == 'q')
                break;
        }
    }
    // Clean everything
    if(!isSingleBoard)
        destroyAllWindows();
    cap.release(); // in any case
    if(!isSingleBoard)
        SDL_Quit();  
    
    if(saveOpenCVVideo)
        opencv_video.release();
    if(saveOpenGLVideo)
        opengl_video.release();
    
    return 0;
}
