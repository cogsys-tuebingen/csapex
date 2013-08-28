#ifndef TERRA_MAP_HPP
#define TERRA_MAP_HPP
#include <opencv2/core/core.hpp>
#include <iostream>

namespace cv_terramap {
inline int gcd(int a, int b)
{
    int temp;
    while (b != 0) {
        temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

inline void  prepare_map(const cv::Mat &img, const int cell_size)
{
    if(gcd(img.rows, cell_size) != 1 || gcd(img.cols, cell_size) != 1) {
        std::cerr << "GCD of rows, cols and cellsize must be 1!" <<std::endl;
        return;
    }

    int map_rows = img.rows / cell_size;
    int map_cols = img.cols / cell_size;

    for(int i = 0 ; i < map_rows ; i++) {
        for(int j = 0 ; j < map_cols ; j++) {


        }
    }
}

inline void  prepare_map(const cv::Mat &img, const int cell_size)
{
    if(gcd(img.rows, cell_size) != 1 || gcd(img.cols, cell_size) != 1) {
        std::cerr << "GCD of rows, cols and cellsize must be 1!" <<std::endl;
        return;
    }

    int map_rows = img.rows / cell_size;
    int map_cols = img.cols / cell_size;

    for(int i = 0 ; i < map_rows ; i++) {
        for(int j = 0 ; j < map_cols ; j++) {


        }
    }
}


//void FeatureExtractor::extract(const cv::Mat &image, const cv::Rect roi, const KeypointParams &params, const int max_octave,
//                          const bool color_extension, const bool large,
//                          cv::Mat &descriptors)

//AttrTerrainFeature::Params p_ = p;
///// EXTRACT
//cv::Mat descriptors;
//p.extractor->extract(_img, _roi,
//                     p.key, p.max_octave,
//                     p.color_extension, p.large_descriptor,
//                     descriptors);

///// PREDICT
//if(descriptors.empty())
//    return AttrTerrainFeature(-1, -1.f);

//if(descriptors.type() != CV_32FC1) {
//    descriptors.convertTo(descriptors, CV_32FC1);
//}

//int   classID = -1;
//float prob    = 0.f;
//if(descriptors.rows > 1) {
//    if(p_.use_max_prob)
//        p_.classifier->predictClassProbMultiSampleMax(descriptors, classID, prob);
//    else
//        p_.classifier->predictClassProbMultiSample(descriptors, classID, prob);
//} else {
//    p_.classifier->predictClassProb(descriptors, classID, prob);
//}

//return AttrTerrainFeature(classID, prob);
//cv::Rect roi(cell_size * j, cell_size * i, cell_size, cell_size);

//cv::Mat descriptors;
//cv::Mat img_roi(_img, _roi);

//p.extractor->extract(img_roi, p.color_extension, p.large_descriptors, descriptors);

//if(descriptors.type() != CV_32FC1) {
//    descriptors.convertTo(descriptors, CV_32FC1);
//}

///// PREDICT
//int   classID;
//float prob;
//p.classifier->predictClassProb(descriptors, classID, prob);

//return AttrTerrainClassPt(classID, prob);


}
#endif // TERRA_MAP_HPP
