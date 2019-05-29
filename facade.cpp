#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "GraphCannySeg.h"
#include "configprop.h"
#include "objectseg.h"

class Facade
{
    public:
        Facade(std::string configFilePath);
        ObjectSeg* segmentImage(const char* rgbFilePath, const char* depthFilePath, char *numObjects);
        void cleanupObjects(ObjectSeg* objectToClean, char *numObjects);
    private:
    bool showDebug;
    bool showImages;
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
    double fx = 572.41140;
    double fy = 573.57043;
    double cx = 325.26110;
    double cy = 242.04899;    
    
    float k_vec[9];
    float kfloat;
    float kxfloat;
    float kyfloat;
    float ksfloat;
    float gafloat;
    float lafloat;
    float lcannyf;    
    float hcannyf;
};

Facade::Facade(std::string configFilePath)
{
    ConfigProperties *config = new ConfigProperties(configFilePath);
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
    showDebug = config->config["show_debug"] == "true"; 
    showImages = config->config["show_image"] == "true"; 
    delete config; 

    k_vec[0] = static_cast<float>(fx);
    k_vec[1] = 0;
    k_vec[2] = static_cast<float>(cx);
    k_vec[3] = 0;
    k_vec[4] = static_cast<float>(fy);
    k_vec[5] = static_cast<float>(cy);
    k_vec[6] = 0.f;
    k_vec[7] = 0.f;
    k_vec[8] = 1.f;

    kfloat = (float)k/10000.f;
    kxfloat = (float)kx/1000.f;
    kyfloat = (float)ky/1000.f;
    ksfloat = (float)ks/1000.f;
    gafloat = ((float)g_angle)*deg2rad;
    lafloat = ((float)l_angle)*deg2rad;
    lcannyf = (float)Lcanny/1000.f;
    hcannyf = (float)Hcanny/1000.f;    
}

ObjectSeg* Facade::segmentImage(const char* rgbFilePath, const char* depthFilePath, char *numObjects) {
    cv::Mat kinect_rgb_img = cv::imread(rgbFilePath);//,cv::IMREAD_UNCHANGED);
    cv::Mat kinect_depth_img_mm = cv::imread(depthFilePath,cv::IMREAD_UNCHANGED);// in mm

    GraphCanny::GraphCannySeg<GraphCanny::hsv>* gcs = new GraphCanny::GraphCannySeg<GraphCanny::hsv>(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);    
    gcs->showImages = showImages;    
    gcs->showDebug = showDebug;
    gcs->run();

    std::vector<GraphCanny::SegResults> vecSegResult = gcs->vecSegResults;
    int maxLength = vecSegResult.size();
    strcpy(numObjects, std::to_string(maxLength).c_str());

    ObjectSeg* results = new ObjectSeg[maxLength];

    for(std::size_t objs=0; objs < maxLength; ++objs) {
        GraphCanny::SegResults obj = vecSegResult[objs];
        int numPoints = obj.pxs.size();

        results[objs] = ObjectSeg(objs+1, numPoints, new PointSeg[numPoints]);

        for(std::size_t pixel=0; pixel < numPoints; ++pixel) {
            cv::Point3i point = obj.pxs[pixel]; 
            results[objs].points[pixel].x = point.x;
            results[objs].points[pixel].y = point.y;
            results[objs].points[pixel].z = point.z;
        }        
    }

    if (showDebug)
        printf("Resulting a array of %d\n", maxLength);

    return results;
}

void Facade::cleanupObjects(ObjectSeg* objectToClean, char *numObjects) {
    if (objectToClean) {
        int numObjts = atoi(numObjects);

        for(std::size_t objs=0; objs < numObjts; ++objs) {
            if (objectToClean[objs].points) {
                delete[] objectToClean[objs].points;
            }
        }

        delete[] objectToClean;
    }      
} 

extern "C"
{   
    Facade* Facade_new(const char* configFilePath) {return new Facade(std::string(configFilePath));}
    ObjectSeg* Facade_segmentImage(Facade* facade, const char* rgbFilePath, const char* depthFilePath, char *numObjects) {
        return facade->segmentImage(rgbFilePath, depthFilePath, numObjects);
    }

    void Facade_cleanupObjects(Facade* facade, ObjectSeg* objectToClean, char *numObjects) {
        facade->cleanupObjects(objectToClean, numObjects);
    }      
}