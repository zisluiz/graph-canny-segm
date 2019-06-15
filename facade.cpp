#define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API

#include <boost/python.hpp>
#include "pyboostcvconverter.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "GraphCannySeg.h"
#include "configprop.h"
#include "objectseg.h"

namespace pbcvt {

    using namespace boost::python;

class Facade
{
    public:
        Facade(std::string configFilePath);
        boost::python::list segmentImage(cv::Mat rgbImage, cv::Mat depthImage);
        void cleanupObjects(boost::python::list objectToClean);
        int numObjects;
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

boost::python::list Facade::segmentImage(cv::Mat kinect_rgb_img, cv::Mat kinect_depth_img_mm) {
    GraphCanny::GraphCannySeg<GraphCanny::hsv>* gcs = new GraphCanny::GraphCannySeg<GraphCanny::hsv>(kinect_rgb_img, kinect_depth_img_mm, sigma, kfloat, min_size, kxfloat, kyfloat, ksfloat,k_vec,lcannyf,hcannyf,kdv, kdc,max_ecc,max_L1,max_L2,(uint16_t)DTH,(uint16_t)plusD,(uint16_t)point3D,gafloat,lafloat,(float)FarObjZ);    
    gcs->showImages = showImages;    
    gcs->showDebug = showDebug;
    gcs->run();

    std::vector<GraphCanny::SegResults> vecSegResult = gcs->vecSegResults;
    this->numObjects = vecSegResult.size();

    boost::python::list results;

    for(std::size_t objs=0; objs < this->numObjects; ++objs) {
        GraphCanny::SegResults obj = vecSegResult[objs];
        int numPoints = obj.pxs.size();

        boost::python::list points;
        results.append(ObjectSeg(objs+1, numPoints, points));

        for(std::size_t pixel=0; pixel < numPoints; ++pixel) {
            cv::Point3i point = obj.pxs[pixel]; 
            points.append(PointSeg(point.x,point.y,point.z));
        }        
    }

    if (showDebug)
        printf("Resulting a array of %d\n", this->numObjects);

    return results;
}

/*void Facade::cleanupObjects(ObjectSeg* objectToClean, char *numObjects) {
    if (objectToClean) {
        int numObjts = atoi(numObjects);

        for(std::size_t objs=0; objs < numObjts; ++objs) {
            if (objectToClean[objs].points) {
                delete[] objectToClean[objs].points;
            }
        }

        delete[] objectToClean;
    }      
}  */

Facade *Facade_new(const char* configFilePath) {
    return new Facade(std::string(configFilePath));
}

boost::python::list Facade_segmentImage(Facade *facade, PyObject *rgbImage, PyObject *depthImage) {
    cv::Mat kinect_rgb_img = pbcvt::fromNDArrayToMat(rgbImage);
    cv::Mat kinect_depth_img_mm = pbcvt::fromNDArrayToMat(depthImage);

    return facade->segmentImage(kinect_rgb_img, kinect_depth_img_mm);
}

/*void Facade_cleanupObjects(Facade *facade, boost::python::list objectToClean) {               
    facade->cleanupObjects(objectToClean);
}*/

#if (PY_VERSION_HEX >= 0x03000000)

    static void *init_ar() {
#else
        static void init_ar(){
#endif
        Py_Initialize();

        import_array();
        return NUMPY_IMPORT_ARRAY_RETVAL;
    }

    BOOST_PYTHON_MODULE (libgraph_canny_segm) {
        //using namespace XM;
        init_ar();

        //initialize converters
        to_python_converter<cv::Mat,pbcvt::matToNDArrayBoostConverter>();
        matFromNDArrayBoostConverter();

        //expose module-level functions
        class_<Facade, boost::shared_ptr<Facade>>("Facade", init<const char*>())
            .def_readwrite("numObjects", &Facade::numObjects);   

        class_<PointSeg>("PointSeg")
            .def_readwrite("x", &PointSeg::x)
            .def_readwrite("y", &PointSeg::y)
            .def_readwrite("z", &PointSeg::z);

        class_<ObjectSeg>("ObjectSeg")
            .def_readwrite("id", &ObjectSeg::id)
            .def_readwrite("pointsLength", &ObjectSeg::pointsLength)            
            .def_readwrite("points", &ObjectSeg::points);
        class_<boost::python::list>("ObjectSeg_vector");                 
  
        //expose module-level functions
        def("Facade_new", &Facade_new, return_value_policy<manage_new_object>());
        def("Facade_segmentImage", &Facade_segmentImage);
        //def("Facade_cleanupObjects", Facade_cleanupObjects);
    }

} //end namespace pbcvt