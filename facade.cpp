#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "GraphCannySeg.h"

class Facade
{
    public:
        Facade();
        void segmentImage(char* rgbFilePath, char* depthFilePath, bool showDebug, bool showImages, char* jsonBuffer);
    private:
    int k=38; //50;
    int kx=2000;
    int ky=30;
    int ks=50;
    float kdv=4.5f;
    float kdc=0.1f;
    float min_size=500.0f;
    float sigma=0.8f;
    float max_ecc = 0.978f;
    float max_L1 = 3800.0f;
    float max_L2 = 950.0f;
    int DTH = 30; //[mm]
    int plusD = 30; //7; //for depth boundary
    int point3D = 5; //10//for contact boundary
    int g_angle = 154; //140;//148;//2.f/3.f*M_PI;
    int l_angle = 56; //M_PI/3.f;
    int Lcanny = 50;
    int Hcanny = 75;
    int FarObjZ = 28000; //875;//1800; //[mm]   

    //ACCV
    double fx=572.41140;
    double fy=573.57043;
    double cx = 325.26110;
    double cy = 242.04899;    
    float k_vec[9] = {static_cast<float>(fx), 0, static_cast<float>(cx), 0, static_cast<float>(fy), static_cast<float>(cy), 0.f,0.f,1.f}; 
};

Facade::Facade()
{
}

void Facade::segmentImage(char* rgbFilePath, char* depthFilePath, bool showDebug, bool showImages, char* jsonBuffer) {

    cv::Mat kinect_rgb_img = cv::imread(rgbFilePath);//,cv::IMREAD_UNCHANGED);
    cv::Mat kinect_depth_img_mm = cv::imread(depthFilePath,cv::IMREAD_UNCHANGED);// in mm

    float kfloat = (float)k/10000.f;
    float kxfloat = (float)kx/1000.f;
    float kyfloat = (float)ky/1000.f;
    float ksfloat = (float)ks/1000.f;
    float gafloat = ((float)g_angle)*deg2rad;
    float lafloat = ((float)l_angle)*deg2rad;
    float lcannyf = (float)Lcanny/1000.f;
    float hcannyf = (float)Hcanny/1000.f;
    //GraphCanny::GraphCannySeg<GraphCanny::hsv> gcs(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);

    GraphCanny::GraphCannySeg<GraphCanny::hsv>* gcs = new GraphCanny::GraphCannySeg<GraphCanny::hsv>(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);    
    gcs->showImages = showImages;    
    gcs->showDebug = showDebug;
    gcs->run();

    std::vector<GraphCanny::SegResults> vecSegResult = gcs->vecSegResults;    


    strcpy(jsonBuffer, "[");
    std::size_t maxLength = 2;//vecSegResult.size();

    for(std::size_t objs=0; objs < maxLength; ++objs) {
        GraphCanny::SegResults obj = vecSegResult[objs];

        strcat(jsonBuffer, "{");
        strcat(jsonBuffer, ("\"id\": "+std::to_string((objs+1))).c_str());
        strcat(jsonBuffer, ",\"points\":[");

        for(std::size_t pixel=0; pixel < obj.pxs.size(); ++pixel) {
            cv::Point3i point = obj.pxs[pixel]; 
            strcat(jsonBuffer, "{");
            strcat(jsonBuffer, ("\"x\":" + std::to_string(point.x)).c_str());
            strcat(jsonBuffer, (",\"y\":" + std::to_string(point.y)).c_str());
            strcat(jsonBuffer, (",\"z\":" + std::to_string(point.z)).c_str());
            strcat(jsonBuffer, "}");

            if (pixel < obj.pxs.size()-1)
                strcat(jsonBuffer, ",");
        }        

        strcat(jsonBuffer, "]}");

        if (objs < maxLength-1)
            strcat(jsonBuffer, ",");
    }

    strcat(jsonBuffer, "]");

    //printf("Preparing for print json format\n");

    if (showDebug)
        printf("Json results: %s\n", jsonBuffer);
}

extern "C"
{
    Facade* Facade_new() {return new Facade();}
    void Facade_segmentImage(Facade* facade, char* rgbFilePath, char* depthFilePath, bool showDebug, bool showImages, char* jsonBuffer) {
        facade->segmentImage(rgbFilePath, depthFilePath, showDebug, showImages, jsonBuffer);
    }
}
