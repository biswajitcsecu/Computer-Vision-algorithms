
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <omp.h>


using namespace std;
using namespace cv;


class PythagoreanFuzzy{
private:  
    Mat src, image, imagef, imageif,imagepf,  imagei2fu, imagei2fl, imagei2fhc, imagedf, imager;
    
public:
    Mat imageFuzzification(Mat image);
    Mat imageIntuFuzzy(Mat image, float alpha);
    Mat imagePythaFuzzy(Mat imagex, Mat imagey);
    Mat imageT2PFuzzyU(Mat image,float alpha );
    Mat imageT2PFuzzyL(Mat image,float alpha );
    float imageFuzzIndex(Mat image);
    float imageFuzziDiver(Mat imagex, Mat imagey);
    Mat imageFuzziHamacherConorm(Mat imagex, Mat imagey, float alpha ); 
    Mat imageDeFuzzification(Mat image);
     
     
    void rundemo(){        
        string filename = "/home/picox/Projects/Netbeans/PythagoreanFuzzyEnhance/rust4.jpg";
        src = imread(samples::findFile(filename), IMREAD_COLOR );
        
        if(src.empty())   {
            printf("Cannot read image file: %s\n", filename.c_str());        
        }
                
        //image conversion
        image = src.clone();
        src.convertTo(image, CV_32F);  
        
        //fuzzy Processing
        imagef  =  imageFuzzification ( image);
     
        //Kaufmann's measure-----------
        int nl= src.rows; 
        int nc= src.cols * src.channels();       
        float fuInd =  0.0f;
        fuInd =  imageFuzzIndex(imagef);
        fuInd = (fuInd/nl );
        cout<<"Fuzzy index value =\t"<<fuInd<<setw(5)<<"\n";
        
        //hesitation degree-----------
        imageif  = imageIntuFuzzy(imagef, fuInd);
        
        //Pythagorean Fuzzy--------------
        imagepf = imagePythaFuzzy(imagef, imageif );
        
        //upper member-ship function----------
         imagei2fu = imageT2PFuzzyU( imagepf, fuInd );
        
        //lower   member-ship function-------------
          imagei2fl= imageT2PFuzzyL( imagepf, fuInd );
          
          //Fuzzy divergence--------------
          float fudev=  0.0f;
          fudev =  imageFuzziDiver( imagei2fu,  imagei2fl);
          fudev = fudev/9;
          cout<<"Fuzzy index value =\t"<<fudev<<setw(5)<<"\n";
          
          //Hamacher T-conorm----------
          imagei2fhc= imageFuzziHamacherConorm(imagei2fu, imagei2fl,  fudev );
     
          //DeFuzzification
          imagedf=imageDeFuzzification(imagei2fhc);           
          imagedf.convertTo(imager, CV_8U);          
        
        // Create a window
        string window_name1 ="Src image";        
        namedWindow(window_name1, WINDOW_OPENGL);
        imshow(window_name1, src);
        
        string window_name2 ="Result image";
        namedWindow(window_name2, WINDOW_OPENGL);
        imshow(window_name2, imager);        
        waitKey(0);
        destroyAllWindows();    
    }    
};



Mat PythagoreanFuzzy ::  imageFuzzification(Mat image) {
    Mat imagef = image.clone();
    int nl= src.rows; 
    int nc= src.cols * src.channels();
#pragma omp parallel
    for (int j=0; j<nl; j++) {
        float* data= image.ptr<float>(j);
        float* dataf= image.ptr<float>(j);
        #pragma omp parallel for
        for (int i=0; i<nc; i++) {
            dataf[i]= (255.f-data[i])/(255.f); 
        } 
    }
    return imagef;
}


float PythagoreanFuzzy :: imageFuzzIndex(Mat image){
    int nl= src.rows; 
    int nc= src.cols * src.channels();
    int n =nl* nc ;
    float sum =0.0f;
    
    #pragma omp parallel
    for (int j=0; j<nl; j++) {
        float* data= image.ptr<float>(j);
        #pragma omp parallel for
        for (int i=0; i<nc; i++) {
           sum += abs(MIN (data[i], (1-data[i]))/n) ;         
        } 
    }    
    return sum;
}

Mat PythagoreanFuzzy :: imageIntuFuzzy(Mat image, float alpha){
    Mat imageif = image.clone();
    int nl= src.rows; 
    int nc= src.cols * src.channels();
#pragma omp parallel
    for (int j=0; j<nl; j++) {
        float* data= image.ptr<float>(j);
        float* dataf= image.ptr<float>(j);
        #pragma omp parallel for
        for (int i=0; i<nc; i++) {
            dataf[i]=abs (1.0- data[i] -((1.0- data[i] )/(1.0+ alpha*data[i])))/nl;  
        } 
    }
    return imageif;    
}


Mat PythagoreanFuzzy :: imagePythaFuzzy(Mat imagex, Mat imagey ){
    Mat imagepf = imagex.clone();
    int nl= src.rows; 
    int nc= src.cols * src.channels();
#pragma omp parallel
    for (int j=0; j<nl; j++) {
        float* datax= imagex.ptr<float>(j);
        float* datay= imagey.ptr<float>(j);
        float* datapf= imagex.ptr<float>(j);
        #pragma omp parallel for
        for (int i=0; i<nc; i++) {
            float a= abs(1.0-datax[i]*datax[i]-datay[i]*datay[i])/nc;
            datapf[i]= pow(  a, 0.5 )/2.0; 
        } 
    }
    return imagepf;        
}

Mat PythagoreanFuzzy ::  imageT2PFuzzyU(Mat image,float alpha ){
        Mat imagefu = image.clone();
        int nl= src.rows; 
        int nc= src.cols * src.channels();
#pragma omp parallel
    for (int j=0; j<nl; j++) {
        float* data= image.ptr<float>(j);
        float* datapu= image.ptr<float>(j);
#pragma omp parallel for
        for (int i=0; i<nc; i++) {
            datapu[i]= pow(data[i],alpha); 
        } 
    }
        return imagefu;           
    }

Mat PythagoreanFuzzy ::  imageT2PFuzzyL(Mat image,float alpha ){
        Mat imagefl = image.clone();
        int nl= src.rows; 
        int nc= src.cols * src.channels();
#pragma omp parallel
    for (int j=0; j<nl; j++) {
        float* data= image.ptr<float>(j);
        float* datapl= image.ptr<float>(j);
#pragma omp parallel for
        for (int i=0; i<nc; i++) {
            datapl[i]= pow(data[i],(1.0/alpha));    
        } 
    }
        return imagefl;           
    }

float  PythagoreanFuzzy ::  imageFuzziDiver(Mat imagex, Mat imagey){
            int nl= src.rows; 
            int nc= src.cols * src.channels();
            int n = src.rows* src.cols ;
            float sum =0.0f;

#pragma omp parallel
            for (int j=0; j<nl; j++) {
                float* datax= imagex.ptr<float>(j);
                float* datay= imagey.ptr<float>(j);
#pragma omp parallel for
                for (int i=0; i<nc; i++) {
                    float p=abs(datax[i]+1);
                    float q=datay[i];                    
                    float r=(datax[i]-datay[i]);
                    float s = ((p+q)-p*q);
                    sum += abs(2.0f*r-( (1.0f-s-q) - (1.0f-s-p)))/(n) ;
                 }                
            }    
            return sum;
       }

    

Mat PythagoreanFuzzy ::  imageFuzziHamacherConorm(Mat imagex, Mat imagey, float alpha ){
            Mat imagerf = imagex.clone();
            int nl= src.rows; 
            int nc= src.cols * src.channels();
#pragma omp parallel
            for (int j=0; j<nl; j++) {
                float* datax= imagex.ptr<float>(j);
                float* datay= imagey.ptr<float>(j);
                float* datarf= imagex.ptr<float>(j);
#pragma omp parallel for
                for (int i=0; i<nc; i++) {
                    float a= datax[i];
                    float b= datay[i];
                    datarf[i]=(a+b+(alpha-2)*a*b)/(1.0-(1-alpha)*a*b) ;   
                } 
            }
            return imagerf;              
    }
    
    
Mat PythagoreanFuzzy ::  imageDeFuzzification(Mat image){
    Mat imager = image.clone();
    int nl= src.rows; 
    int nc= src.cols * src.channels();
#pragma omp parallel
    for (int j=0; j<nl; j++) {
        float* data= image.ptr<float>(j);
        float* datar= image.ptr<float>(j);
#pragma omp parallel for
        for (int i=0; i<nc; i++) {
            datar[i]= data[i];  

        } 
    }
    return imager;          
}


int main(int argc, char** argv) {
    
    try{
    PythagoreanFuzzy *pf = new PythagoreanFuzzy();
    pf->rundemo();
    
    }catch(Exception eb){
        cerr<<eb.what()<<"Error found"<<"\n";
        return EXIT_FAILURE;
    }    
    return EXIT_SUCCESS;
}

