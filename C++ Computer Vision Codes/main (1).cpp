#include <cstdlib>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace  std;


class SURFDetection{

public:

    SURFDetection(){};

    void static run(char* argv[]){
        //Load image--------------
        Mat src = imread("/home/picox/Projects/Netbeans/SURFDetectionDemo/ct2.jpg", IMREAD_COLOR );

        if ( src.empty() )
        {
            cout << "Could not open or find the image!\n" << endl;
        }
        //RGB to Gray image------------
        Mat gray_src;
        cvtColor(src,gray_src,COLOR_RGB2GRAY);

        //Detect the keypoints using SURF Detector-----------
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create( minHessian );
        std::vector<KeyPoint> keypoints;
        detector->detect( gray_src, keypoints );

        //Draw keypoints--------------------
        Mat img_keypoints;
        drawKeypoints( gray_src, keypoints, img_keypoints );

        //Show detected keypoints--------------
        namedWindow("RGBImage",WINDOW_OPENGL);
        imshow("RGBImage", src );
        namedWindow("SURF Keypoints",WINDOW_OPENGL);
        imshow("SURF Keypoints", img_keypoints );
        imwrite("outsur.png",img_keypoints);

        waitKey();
        destroyAllWindows();
        src.release();
        gray_src.release();
    }

};


int main( int argc, char* argv[] )
{
    try {
        SURFDetection *exobj=new SURFDetection();
        exobj->run(argv);
        delete exobj;
        return EXIT_SUCCESS;
    } catch (exception exp) {
        cerr<<"Exception occured\n"<<exp.what()<<endl;
    }
    return EXIT_SUCCESS;
}
