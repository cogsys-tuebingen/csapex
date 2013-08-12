#ifndef ROI_HPP
#define ROI_HPP
#include <opencv2/opencv.hpp>
namespace cv_roi {
struct ROI {
    ROI() : rect(0,0,0,0), rotation(0.0){}
    cv::Rect rect;
    double   rotation;
};

struct TerraID {
    TerraID() : prob(0.f), id(-1){}

    float prob;
    int   id;
};

struct TerraROI {
    TerraID id;
    ROI     roi;
};
}
#endif // ROI_HPP
