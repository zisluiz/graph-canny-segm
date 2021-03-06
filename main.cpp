//
//  main.cpp
//  PSOQuatGraphSeg
//
//  Created by Giorgio on 25/10/15.
//  Copyright (c) 2015 Giorgio. All rights reserved.

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <boost/thread.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/highgui/highgui.hpp"

#include "configprop.h"
#include "GraphCannySeg.h"

// Camera parameters for ACCV dataset
//#define FX 572.41140
//#define FY 573.57043
//#define CX 325.26110
//#define CY 242.04899

// Camera parameters for Rutgers dataset
//#define FX 575.8157348632812
//#define FY 575.8157348632812
//#define CX 319.5
//#define CY 239.5

// Camera parameters for ICCV challenge dataset
//#define FX 571.9737
//#define FY 571.0073
//#define CX 319.5000
//#define CY 239.5000


#define tomm_ 1000.0f
#define tocast_ 0.5f

ConfigProperties *config;

//ACCV
double fx = 0;
double fy = 0;
double cx = 0;
double cy = 0;

/*Segmentation Params*/
//params challenge
int k=0; //50;
int kx=0;
int ky=0;
int ks=0;
float kdv=0;
float kdc=0;
float min_size=0;
float sigma=0;
float max_ecc = 0;
float max_L1 = 0;
float max_L2 = 0;

//params rutgers
//int k=50000;
//int kx=1050;
//int ky=1500;
//int ks=500;
//float kdv=4.5f;
//float kdc=0.1f;
//float min_size=500.0f;
//float sigma=0.8f;
//float max_ecc = 0.978f;
//float max_L1 = 3800.0f;
//float max_L2 = 950.0f;


int DTH = 0; //[mm]
int plusD = 0; //7; //for depth boundary
int point3D = 0; //10//for contact boundary
int g_angle = 0; //140;//148;//2.f/3.f*M_PI;
int l_angle = 0; //M_PI/3.f;
int Lcanny = 0;
int Hcanny = 0;
int FarObjZ = 0; //875;//1800; //[mm]

std::string trackBarsWin = "Trackbars";
//Segmentation Results
GraphCanny::GraphCannySeg<GraphCanny::hsv>* gcs=0;
std::vector<GraphCanny::SegResults> vecSegResult;
//used to load rgb and depth images fromt the dataset
cv::Mat kinect_rgb_img;
cv::Mat kinect_depth_img_mm;

bool showDebug = false;
bool showImages = false;
std::string writeImageSeg;

// for loading ACCV dataset
cv::Mat loadDepth( std::string a_name )
{
    cv::Mat lp_mat;
    std::ifstream l_file(a_name.c_str(),std::ofstream::in|std::ofstream::binary );

    if( l_file.fail() == true )
    {
        printf("cv_load_depth: could not open file for writing!\n");
        return lp_mat;
    }
    int l_row;
    int l_col;

    l_file.read((char*)&l_row,sizeof(l_row));
    l_file.read((char*)&l_col,sizeof(l_col));

    /*IplImage * lp_image = cvCreateImage(cvSize(l_col,l_row),IPL_DEPTH_16U,1);

    for(int l_r=0;l_r<l_row;++l_r)
    {
        for(int l_c=0;l_c<l_col;++l_c)
        {
            l_file.read((char*)&CV_IMAGE_ELEM(lp_image,unsigned short,l_r,l_c),sizeof(unsigned short));
        }
    }
    l_file.close();

    lp_mat= cv::Mat(lp_image);
    return lp_mat;*/

     printf("######################### Entrou loadDepth!\n");

    return cv::Mat(cv::Size(l_col,l_row),CV_16UC1);
}


void on_trackbar( int, void* )
{

    if(gcs)
      delete gcs;

    float k_vec[9] = {static_cast<float>(fx), 0, static_cast<float>(cx), 0, static_cast<float>(fy), static_cast<float>(cy), 0.f,0.f,1.f};
    float kfloat = (float)k/10000.f;
    float kxfloat = (float)kx/1000.f;
    float kyfloat = (float)ky/1000.f;
    float ksfloat = (float)ks/1000.f;
    float gafloat = ((float)g_angle)*deg2rad;
    float lafloat = ((float)l_angle)*deg2rad;
    float lcannyf = (float)Lcanny/1000.f;
    float hcannyf = (float)Hcanny/1000.f;
    //GraphCanny::GraphCannySeg<GraphCanny::hsv> gcs(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);

    gcs = new GraphCanny::GraphCannySeg<GraphCanny::hsv>(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);    
    gcs->showImages = showImages;    
    gcs->showDebug = showDebug;
    gcs->writeImageSeg = writeImageSeg;
    gcs->run();

    vecSegResult = gcs->vecSegResults;

    if (showDebug)
        printf("Número de objetos: %d\n", vecSegResult.size());
        
    if (showImages) {
        //text.zeros(480, 640,CV_8UC1);
        cv::Mat text = cv::Mat::zeros(230, 640,CV_8UC1);
        char text_[200]={};        
        sprintf(text_, "DTH: %d plusD: %d point3D: %d",DTH,plusD,point3D);
        std::string tstring(text_);
        std::cout<<tstring<<"\n";
        cv::putText(text, tstring, cv::Point(50,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

        sprintf(text_, "K: %f Kx: %.2f Ky: %f Ks: %f",kfloat,kxfloat,kyfloat,ksfloat);
        tstring = std::string(text_);
        std::cout<<tstring<<"\n";
        cv::putText(text, tstring, cv::Point(50,100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

        sprintf(text_, "G_angle: %d  L_angle: %d  Zmax: %d",g_angle,l_angle, FarObjZ);
        tstring = std::string(text_);
        std::cout<<tstring<<"\n";
        cv::putText(text, tstring, cv::Point(50,150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

        sprintf(text_, "Low Canny : %f  High Canny: %f",lcannyf,hcannyf);
        tstring = std::string(text_);
        std::cout<<tstring<<"\n";
        cv::putText(text, tstring, cv::Point(50,200), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

        cv::imshow( trackBarsWin, text );
        cv::waitKey(5);
    }
}

void initTrackbarsSegmentation(std::string rgb_file_path,  std::string depth_file_path, std::string _showDebug, std::string _showImages)
{   
    showDebug = "true" == _showDebug;
    showImages = "true" == _showImages;

    if (showDebug)
        cout << "Initializating";

    config = new ConfigProperties(std::string("config.properties"));
    fx = std::stod(config->config["fx"]);
    fy = std::stod(config->config["fy"]);
    cx = std::stod(config->config["cx"]);
    cy = std::stod(config->config["cy"]);
    k = std::stoi(config->config["k"]);
    kx = std::stoi(config->config["kx"]);
    ky = std::stoi(config->config["ky"]);
    ks = std::stoi(config->config["ks"]);
    kdv = std::stof(config->config["kdv"]);
    kdc = std::stof(config->config["kdc"]);
    min_size = std::stof(config->config["min_size"]);
    sigma = std::stof(config->config["sigma"]);
    max_ecc = std::stof(config->config["max_ecc"]);
    max_L1 = std::stof(config->config["max_L1"]);
    max_L2 = std::stof(config->config["max_L2"]);
    DTH = std::stoi(config->config["DTH"]);
    plusD = std::stoi(config->config["plusD"]);
    point3D = std::stoi(config->config["point3D"]);
    g_angle = std::stoi(config->config["g_angle"]);
    l_angle = std::stoi(config->config["l_angle"]);
    Lcanny = std::stoi(config->config["Lcanny"]);
    Hcanny = std::stoi(config->config["Hcanny"]);
    FarObjZ = std::stoi(config->config["FarObjZ"]);
    writeImageSeg = config->config["write_image_seg"]; 
    delete config;

    /* Load the RGB and DEPTH */


    //CHALLENGE DATASET-1
    // std::string rgb_name= "img_1164.png";
    // std::string obj_name= "Shampoo";

    // rgb_file_path = "/Volumes/HD-PNTU3/datasets/"+obj_name+"/RGB/"+rgb_name;
    // depth_file_path = "/Volumes/HD-PNTU3/datasets/"+obj_name+"/Depth/"+rgb_name;

    kinect_rgb_img = cv::imread(rgb_file_path);//,cv::IMREAD_UNCHANGED);
    kinect_depth_img_mm = cv::imread(depth_file_path,cv::IMREAD_UNCHANGED);// in mm

    //cv::imshow("kinect_rgb_img",kinect_rgb_img);
    //cv::imshow("kinect_depth_img_mm",kinect_depth_img_mm);

    /*if (showDebug) {
        for(int i=250;i<350;i++)
                for(int j=200;j<250;j++)
                    printf("%d ",kinect_depth_img_mm.at<uint16_t>(j,i));
    }*/
 
    if (showImages) {
        /*Create the TrackBars for Segmentation Params*/
        cv::namedWindow(trackBarsWin,0);

        cv::createTrackbar("k", trackBarsWin, &k, 800,on_trackbar);
        cv::createTrackbar("kx", trackBarsWin, &kx, 3000,on_trackbar);
        cv::createTrackbar("ky", trackBarsWin, &ky, 3000,on_trackbar);
        cv::createTrackbar("ks", trackBarsWin, &ks, 3000,on_trackbar);
        cv::createTrackbar("DTH", trackBarsWin, &DTH, 500,on_trackbar);
        cv::createTrackbar("plusD", trackBarsWin, &plusD, 500,on_trackbar);
        cv::createTrackbar("Point3D", trackBarsWin, &point3D, 500,on_trackbar);
        cv::createTrackbar("G Angle", trackBarsWin, &g_angle, 360,on_trackbar);
        cv::createTrackbar("L Angle", trackBarsWin, &l_angle, 360,on_trackbar);
        cv::createTrackbar("H Canny th", trackBarsWin, &Hcanny, 1000,on_trackbar);
        cv::createTrackbar("L Canny th", trackBarsWin, &Lcanny, 1000,on_trackbar);
        cv::createTrackbar("FarObjZ", trackBarsWin, &FarObjZ, 80000,on_trackbar);
    }

    on_trackbar( 0, 0 );

    if (showImages) {
        /// Wait until user press some key
        cv::waitKey(0);
    }
}

inline float min3(const float &a, const float &b, const float &c)
{ return std::min(a, std::min(b, c)); }

inline float max3(const float &a, const float &b, const float &c)
{ return std::max(a, std::max(b, c)); }

const int imageWidth = 640;
const int imageHeight = 480;

inline
float edgeFunction(
        const float* a, const float* b, const float* c)
{
    return(
        (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]));
}
inline void printVector(const float* w, int size)
{
    printf("[ ");
    for (int i=0; i<size; ++i) {
        if (i==size-1) {
            printf("%f ",w[i]);
        }
        else
            printf("%f, ",w[i]);
    }
    printf("]\n");
}


int main( int argc, char *argv[] )
{
    if (argc != 5) {
        printf("usage: %s rgb_file_path depth_file_path showDebug showImages\n",argv[0]);
        return -1;
    }

    initTrackbarsSegmentation(argv[1], argv[2], argv[3], argv[4]);

    delete gcs;

    return 0;
}