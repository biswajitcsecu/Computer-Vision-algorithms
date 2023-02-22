#include <iostream>
#include <opencv2/opencv.hpp>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/img/CImage.h>
#include <features2d.hpp>
#include <mrpt/system/CTicTac.h>


using namespace mrpt::gui;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace std;
using namespace cv;


class GaussWindows{

public:    
    GaussWindows(){}
    void runs()
    {
        
        //Loadimagefile from disk---------------
        CImage inImg, outImg;
        inImg.loadFromFile( "/home/picox/Projects/Netbeans/imgGaussFiltering/test2.jpg");
        
        
        // Smoothed image----------------
        
        inImg.filterMedian(inImg,3);
        inImg.filterGaussian(outImg, 11, 11);  
       
        
        inImg.joinImagesHorz(inImg,outImg);
        
        // Window size----------------
         CDisplayWindow win("Gaussian Blured Image");
        //CDisplayWindow win1("Original Image");
        //CDisplayWindow win2("Smoothed Image");
        
         win.showImage(inImg);
        //win1.showImage(inImg);
        //win2.showImage(outImg);   
        mrpt::system::pause();
        
         win.~CDisplayWindow();
       // win1.~CDisplayWindow();
       // win2.~CDisplayWindow();
    }
};

int main(int argc, char** argv)
{
       GaussWindows  *gw =new GaussWindows();
       gw->runs();
        return 0;

}
