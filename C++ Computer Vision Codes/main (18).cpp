#include<iostream>
#include<vector>
#include<cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ximgproc/nonfree.hpp>


#define PI 3.141592
using namespace std;
using namespace cv;

int main()
{
    string filea ="/home/picox/Projects/Netbeans/RegistratationImages/sec1.jpg";
    string fileb ="/home/picox/Projects/Netbeans/RegistratationImages/sec2.jpg";

    Mat Img1 = imread(filea, IMREAD_COLOR);      //to be registrated image
    Mat Img2 = imread(fileb, IMREAD_COLOR);      //basic image

    if(Img1.empty()){
        cout<<"Nofilefound\n";
        return -1;
    }

    if(Img2.empty()){
        cout<<"Nofilefound\n";
        return -1;
    }

  //show images
    namedWindow("Target", WINDOW_OPENGL);
    namedWindow("Src", WINDOW_OPENGL);
    imshow("Target", Img1);
    imshow("Src", Img2);

    //define sift feature objection
    Ptr<SiftFeatureDetector> siftDetector1;
    Ptr<SiftFeatureDetector> siftDetector2;

    //difine KeyPoint
    vector<KeyPoint>keyPoints1;
    vector<KeyPoint>keyPoints2;

    //feature detection
    siftDetector1->detect(Img1, keyPoints1);
    siftDetector2->detect(Img2, keyPoints2);

    //draw key points
    Mat feature_pic1, feature_pic2;
    drawKeypoints(Img1, keyPoints1, feature_pic1, Scalar::all(-1));
    drawKeypoints(Img2, keyPoints2, feature_pic2, Scalar::all(-1));

    drawKeypoints(Img1, keyPoints1, feature_pic1, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(Img2, keyPoints2, feature_pic2, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //show result
    namedWindow("feature1", WINDOW_OPENGL);
    imshow("feature1", feature_pic1);
    namedWindow("feature2", WINDOW_OPENGL);
    imshow("feature2", feature_pic2);

    //compute feature descriptor
    Ptr<SiftDescriptorExtractor> descriptor;
    Mat description1;
    descriptor->compute(Img1, keyPoints1, description1);
    Mat description2;
    descriptor->compute(Img2, keyPoints2, description2);

    cout << keyPoints1.size() << endl;
    cout << description1.cols << endl;      //cols
    cout << description1.rows << endl;      //rows


    //BFMatch match
    //BruteForceMatcher<L2<float>>matcher;
    Ptr<FlannBasedMatcher> matcher;
    vector<DMatch>matches;   //define matched result
    matcher->match(description1, description2, matches);  //complete match between descriptors


    int i=0,j=0;
    

    double max_dist = 0;
    double min_dist = 100;
    for(int i=0; i<matches.size(); i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist)
            min_dist = dist;
        if(dist > max_dist)
            max_dist = dist;
    }
    cout<<"most distance："<<max_dist<<endl;
    cout<<"least distance："<<min_dist<<endl;

    //Filter out better matching points
    vector<DMatch> good_matches;
    double dThreshold = 0.5;    //Threshold
    for(int i=0; i<matches.size(); i++)
    {
        if(matches[i].distance < dThreshold * max_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }

    //RANSAC Eliminates mis-matching feature points：
    vector<KeyPoint> R_keypoint01,R_keypoint02;
    for (i=0;i<good_matches.size();i++)
    {
        R_keypoint01.push_back(keyPoints1[good_matches[i].queryIdx]);
        R_keypoint02.push_back(keyPoints2[good_matches[i].trainIdx]);
    }

    //Coordinate conversion
    vector<Point2f>p01,p02;
    for (i=0;i<good_matches.size();i++)
    {
        p01.push_back(R_keypoint01[i].pt);
        p02.push_back(R_keypoint02[i].pt);
    }

    //Calculate the base matrix and eliminate the mismatch
    vector<uchar> RansacStatus;
    Mat Fundamental= findHomography(p01,p02,RansacStatus,RANSAC);
    Mat dst;
    warpPerspective(Img1, dst, Fundamental,Size(Img1.cols,Img1.rows));

    namedWindow("warped",WINDOW_OPENGL);
    imshow("warped",dst );
    imwrite("path.jpg", dst);

    //Remove the mismatched pair
    vector<KeyPoint> RR_keypoint01,RR_keypoint02;
    vector<DMatch> RR_matches;
    int index=0;
    for (i=0;i<good_matches.size();i++)
    {
        if (RansacStatus[i]!=0)
        {
            RR_keypoint01.push_back(R_keypoint01[i]);
            RR_keypoint02.push_back(R_keypoint02[i]);
            good_matches[i].queryIdx=index;
            good_matches[i].trainIdx=index;
            RR_matches.push_back(good_matches[i]);
            index++;
        }
    }
    cout<<"Feature point pairs found："<<RR_matches.size()<<endl;

    //Draw a map after eliminating false matches
    Mat img_RR_matches;
    drawMatches(Img1,RR_keypoint01,Img2,RR_keypoint02,RR_matches,img_RR_matches, Scalar(0, 255, 0), Scalar::all(-1));
    namedWindow("Registered", WINDOW_OPENGL);
    imshow("Registered",img_RR_matches);
    imwrite("Registered.jpg", img_RR_matches);

    waitKey(0);
    destroyAllWindows();
    return EXIT_SUCCESS;
}
