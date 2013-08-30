#ifndef TERRA_MAP_HPP
#define TERRA_MAP_HPP
#include <opencv2/core/core.hpp>
#include <iostream>
#include "math.hpp"
#include "extractor.h"
#include "randomforest.h"

using namespace cv_extraction;


inline void check_dimension(const int rows, const int cols, const int cell_size)
{
    /// CHECK DIMENSION
    if(cv_math::gcd(rows, cell_size) != 1 || cv_math::gcd(cols, cell_size) != 1) {
        std::cerr << "GCD of rows, cols and cellsize must be 1!" << std::endl;
        return;
    }
}

/**
 * @brief As dynamic size implementation according to classes parameter.
 * @param img                   - the image to generate a terramat of
 * @param cell_size             - the cellsize the regions should have
 * @param classes               - the amount of classes that are required
 * @param extractor             - the extractor to use
 * @param classifier            - the classifier to
 * @param terraMat              - the terramatrix
 * @param channel_mapping       - the channel class id mapping
 */
inline void  prepare_terra_mat(const cv::Mat &img, const int cell_size, const int classes,
                               Extractor::Ptr extractor, RandomForest::Ptr classifier,
                               cv::Mat terraMat, std::map<uchar, uchar> &channel_mapping)
{
    /// PREPARE MATRICES
    int map_rows = img.rows / cell_size;
    int map_cols = img.cols / cell_size;

    std::vector<cv::Mat> layers;
    for(int i = 0 ; i < classes ; i++) {
        layers.push_back(cv::Mat(map_rows, map_cols, CV_32FC1, cv::Scalar::all(0)));
    }

    /// CALCULATE
    cv::Rect roi(0,0,cell_size,cell_size);
    std::map<int, float>probs;

    for(int i = 0 ; i < map_rows ; i++) {
        for(int j = 0 ; j < map_cols ; j++) {
            /// PREPARE
            roi.x = cell_size * j;
            roi.y = cell_size * i;
            ids.clear();
            probs.clear();
            /// EXTRACT
            cv::Mat descriptors;
            extractor->extract(img, roi, descriptors);
            /// CLASSIFY
            classifier->predictClassProbs(descriptors, probs);
            /// ENTER
            if(channel_mapping.empty()) {
                int channel = 0;
                for(std::map<int, float>::iterator it = probs.begin() ; it != probs.end() ; it++, channel++) {
                    if(it->first > 255) {
                        std::cerr << "Only classIDs < 255 supported. Please change class labels!" << std::endl;
                        std::cerr << "Aborted!" << std::endl;
                        return;
                    }
                    channel_mapping.insert(std::make_pair(it->first, channel));
                    layers[channel].at<float>(i,j) = it->second;

                }
            } else {
                for(std::map<int, float>::iterator it = probs.begin() ; it != probs.end() ; it++) {
                    int channel = channel_mapping[it->first];
                    layers[channel].at<float>(i,j) = it->second;
                }
            }
        }
    }

    cv::merge(layers, terraMat);
}

/**
 * @brief As fixed size implementation according to template parameter.
 * @param img                   - the image to generate a terramat of
 * @param cell_size             - the cellsize the regions should have
 * @param extractor             - the extractor to use
 * @param classifier            - the classifier to
 * @param terraMat              - the terramatrix
 * @param channel_mapping       - the channel class id mapping
 */
template<int classes = 5>
inline void  prepare_terra_mat(const cv::Mat &img, const int cell_size,
                               Extractor::Ptr extractor, RandomForest::Ptr classifier,
                               cv::Mat &terraMat, std::map<uchar, uchar> &channel_mapping)
{
    /// PREPARE MATRICES
    int map_rows = img.rows / cell_size;
    int map_cols = img.cols / cell_size;
    terraMat = cv::Mat(map_rows, map_cols, CV_32FC(classes), cv::Scalar::all(0));

    /// CALCULATE
    cv::Rect roi(0,0,cell_size,cell_size);
    std::map<int, float>probs;

    for(int i = 0 ; i < map_rows ; i++) {
        for(int j = 0 ; j < map_cols ; j++) {
            /// PREPARE
            roi.x = cell_size * j;
            roi.y = cell_size * i;
            ids.clear();
            probs.clear();
            /// EXTRACT
            cv::Mat descriptors;
            extractor->extract(img, roi, descriptors);
            /// CLASSIFY
            classifier->predictClassProbs(descriptors, probs);
            /// ENTER
            if(channel_mapping.empty()) {
                int channel = 0;
                for(std::map<int, float>::iterator it = probs.begin() ; it != probs.end() ; it++, channel++) {
                    if(it->first > 255) {
                        std::cerr << "Only classIDs < 255 supported. Please change class labels!" << std::endl;
                        std::cerr << "Aborted!" << std::endl;
                        return;
                    }
                    channel_mapping.insert(std::make_pair(it->first, channel));
                    assert(channel_mapping.size() <= classes);
                    layers[channel].at<float>(i,j) = it->second;
                }
            } else {
                for(std::map<int, float>::iterator it = probs.begin() ; it != probs.end() ; it++) {
                    int channel = channel_mapping[it->first];
                    assert(channel_mapping.size() <= classes);
                    layers[channel].at<float>(i,j) = it->second;
                }
            }
        }
    }
}
#endif // TERRA_MAP_HPP
