#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/shape.hpp"
#include <opencv2/aruco.hpp>


#include <math.h>
#include <iostream>

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";
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

float sideOfTheMarker = 0.2;
double halfOfTheSideOfTheMarker = sideOfTheMarker/2;

// Get Mat from 3D vector. 
cv::Mat DoubleMatFromVec3d(cv::Vec3d in)
{
    cv::Mat mat(3,1, CV_64FC1);
    mat.at <double>(0,0) = in [0];
    mat.at <double>(1,0) = in [1];
    mat.at <double>(2,0) = in [2];

    return mat;
};

// TODO: REMOVAL
void inversePerspective(Mat rvec, Mat tvec, Mat& camRvec, Mat& camTvec){
    Mat R;
    Rodrigues(rvec, R);
    R = R.t();
    camTvec = -R*tvec;
    Rodrigues(R, camRvec);
}

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

double euclideanDistance(double xdiff, double ydiff, double zdiff){
    return sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff);
}

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


int main(){
    // camera calibration matrix and distortion coefficients
    cv::Mat cameraMatrix = cv::Mat(3,3, CV_64F);
    cv::Mat distCoeffs = cv::Mat(1,4, CV_64F);
    cv::Mat rotationMatrix = cv::Mat(3,3, CV_64F);
    cv::Mat translationVector = cv::Mat(1,3, CV_64F);

    // camera
    VideoCapture cap(1);

    int width = 640, height = 480;

    cap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,height);

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
    */
  
    /*
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
    */

    /*
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

    /*
    cameraMatrix.at<double>(0,0) = cameraMatrix.at<double>(0,0) * (width / 640);
    cameraMatrix.at<double>(0,1) = cameraMatrix.at<double>(0,1)  * (width / 640);
    cameraMatrix.at<double>(0,2) = cameraMatrix.at<double>(0,2)  * (width / 640);

    cameraMatrix.at<double>(1,0) = cameraMatrix.at<double>(1,0)  * (height / 480);
    cameraMatrix.at<double>(1,1) = cameraMatrix.at<double>(1,1) * (height / 480);
    cameraMatrix.at<double>(1,2) = cameraMatrix.at<double>(1,2)  * (height / 480);
    */
    
    // Define the dictionary
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_7X7_100));

    // detect markers
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    
    cerr << "Debug: Starting to infinite." << endl; // debug

    vector< Vec3d > rvecs, tvecs;
    while(true){
        Mat img;
        if(!cap.read(img)){
            cerr << "Video camera is not working at the moment. Capture Error!" << endl;
            exit(1);
        }

        vector< vector< Point2f > > corners;
        vector< int > ids;
        cerr << "Trying to detect markers" << endl; // debug
        aruco::detectMarkers(img, dictionary, corners, ids, params);

        cerr << "Markers are detected, starting pose estimation" << endl; // debug
        if(!ids.empty()){
            cout << "sort" << endl;
            sortByIds(corners, ids);
            aruco::drawDetectedMarkers(img, corners, noArray(), Scalar(0,0,250) );
            cout << "estimate part" << endl; // debug
            aruco::estimatePoseSingleMarkers(corners, sideOfTheMarker, cameraMatrix, distCoeffs, rvecs, tvecs);

            for(int i = 0; i < ids.size(); ++i){
                aruco::drawAxis(img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], halfOfTheSideOfTheMarker);

                Mat homo = getRTMatrix(DoubleMatFromVec3d(rvecs[i]), DoubleMatFromVec3d(tvecs[i]), -1);
                Mat invertedMat  = homo.inv();
                Mat invertedTvec = invertedMat.col(3);
                invertedTvec.pop_back();

                Vec3d eulerAngles;
                cv::Mat rotMat = Mat(3,3,CV_64F);
                cv::Mat tmp = invertedMat(cv::Rect(0,0,3,3));
                tmp.copyTo(rotMat);

                getEulerAngles(rotMat, eulerAngles); 
                
                // Multiply with (180.0 / 3.14159) if you want degrees
                double pitch   = eulerAngles[0];
                double roll = eulerAngles[1];
                double yaw  = eulerAngles[2];
                
                double realY = tvecs[i][1] - atan(pitch / 180.0 * 3.14159) * tvecs[i][2]; 
                double realX = tvecs[i][0] - atan(roll / 180.0 * 3.14159) * tvecs[i][2];

                cout << "For id: " << ids[i] << endl;
                cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl << endl;
                // cout << invertedMat << endl;
                // cout << rotMat << endl;
                cout << "XYZ: " << invertedTvec << endl;

            }
            
        }
            
        cv::namedWindow("Stream", cv::WINDOW_NORMAL);
        cv::resizeWindow("Stream", cv::Size(width,height));
        cv::imshow("Stream", img);
        
        char key = waitKey(1);
        if(key == 'q')
            break;
        else if(key == 'p'){
            cout << "aruco rvec: " << rvecs[0] << endl;
            cout << "aruco tvec: " << tvecs[0] << endl;
        }
    }

    cap.release();
    destroyAllWindows();

    return 0;
}
