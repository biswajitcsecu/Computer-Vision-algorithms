#include <cstdlib>
#include <visp3/core/vpImageFilter.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <omp.h>
        
using namespace std;


void display(vpImage<unsigned char> &I, const std::string &title);
void display(vpImage<double> &D, const std::string &title);



void display(vpImage<unsigned char> &I, const std::string &title){
  vpDisplayX d(I);  
  d.init(I, 0, 0, title.c_str());
  vpDisplay::setTitle(I, title.c_str());
  vpDisplay::display(I);
  vpDisplay::displayText(I, 15, 15, "Click to continue...", vpColor::red);
  vpDisplay::flush(I);
  vpDisplay::getClick(I);
}

void display(vpImage<double> &D, const std::string &title){
  vpImage<unsigned char> I; 
  vpImageConvert::convert(D, I);
  display(I, title);
}


int main(int argc, char** argv) {
    try {
        vpImage<unsigned char> I;
        string file = "/home/picox/Projects/Netbeans/imageVSPfilter/raggs.jpg";
        try {
            vpImageIo::read(I, file);
        } catch (...) {
            std::cout << "Cannot read image \"" << std::endl;
            return -1;
        }
        
    display(I, "Original image");
    vpImage<double> F;
    
    vpImageFilter::gaussianBlur(I, F);
    display(F, "Blur (default)");
    
    vpImageFilter::gaussianBlur(I, F, 7, 2);
    display(F, "Blur (var=2)");
    
    vpImage<double> dIx;
    vpImageFilter::getGradX(I, dIx);
    display(dIx, "Gradient dIx");
    
    vpImage<double> dIy;
    vpImageFilter::getGradY(I, dIy);
    display(dIy, "Gradient dIy");
    

    vpImage<unsigned char> C;
    vpImageFilter::canny(I, C, 5, 15, 3);
    display(C, "Canny");

    vpMatrix K(3, 3); 
    K[0][0] = 1;
    K[0][1] = 0;
    K[0][2] = -1;
    K[1][0] = 2;
    K[1][1] = 0;
    K[1][2] = -2;
    K[2][0] = 1;
    K[2][1] = 0;
    K[2][2] = -1;
    vpImage<double> Gx;
    vpImageFilter::filter(I, Gx, K);
    display(Gx, "Sobel x");
    
    size_t nlevel = 3;
    std::vector<vpImage<unsigned char> > pyr(nlevel);
    pyr[0] = I;
#pragma omp parallel for
    for (size_t i = 1; i < nlevel; i++) {
      vpImageFilter::getGaussPyramidal(pyr[i - 1], pyr[i]);
      display(pyr[i], "Pyramid");
    }
    
    return 0;
    
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
    return EXIT_SUCCESS;
}

