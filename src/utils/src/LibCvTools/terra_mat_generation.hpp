#ifndef TERRA_MAP_HPP
#define TERRA_MAP_HPP
#include <opencv2/core/core.hpp>
#include <iostream>
#include "math.hpp"
#include "extractor.h"
#include "randomforest.h"

using namespace cv_extraction;


inline void getRois(const int rows, const int cols, const int cell_size, std::vector<cv::Rect> &rois)
{
    cv::Rect roi(0,0, cell_size, cell_size);
    for(int i = 0 ; i < rows ; i++) {
        for(int j = 0 ; j < cols ; j++) {
            roi.x = cell_size * j;
            roi.y = cell_size * i;
            rois.push_back(roi);
        }
    }
}

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
                               cv::Mat &terraMat, std::map<int, int> &channel_mapping,
                               bool use_max_prob)
{
    /// PREPARE MATRICES
    int map_rows = img.rows / cell_size;
    int map_cols = img.cols / cell_size;

    std::vector<cv::Mat> layers;
    for(int i = 0 ; i < classes ; i++) {
        layers.push_back(cv::Mat(map_rows, map_cols, CV_32FC1, cv::Scalar::all(0)));
    }

    /// CALCULATE
    std::map<int, float>    probs;
    std::vector<cv::Rect>   rois;
    std::vector<cv::Mat>    descriptors;
    getRois(map_rows, map_cols, cell_size, rois);
    extractor->extract(img, rois, descriptors);

    for(int i = 0 ; i < map_rows ; i++) {
        for(int j = 0 ; j < map_cols ; j++) {
            probs.clear();
            cv::Mat &d = descriptors[i * map_cols + j];

            /// CLASSIFY
            if(d.rows > 1) {
                if(use_max_prob) {
                    classifier->predictClassProbsMultiSampleMax(d, probs);
                } else
                    classifier->predictClassProbsMultiSample(d, probs);
            } else {
                classifier->predictClassProbs(d, probs);
            }
            /// ENTER
            for(std::map<int, float>::iterator it = probs.begin() ; it != probs.end() ; it++) {
                int channel = channel_mapping[it->first];
                layers[channel].at<float>(i,j) = it->second;
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
template<int classes>
inline void  prepare_terra_mat_fixed(const cv::Mat &img, const int cell_size,
                                     Extractor::Ptr extractor, RandomForest::Ptr classifier,
                                     cv::Mat &terra_mat, std::map<uchar, uchar> &channel_mapping,
                                     bool use_max_prob)
{
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, CV_BGR2GRAY);

    /// PREPARE MATRICES
    int map_rows = img.rows / cell_size;
    int map_cols = img.cols / cell_size;
    terra_mat = cv::Mat(map_rows, map_cols, CV_32FC(classes), cv::Scalar::all(0));

    std::vector<cv::Rect>   rois;
    std::vector<cv::Mat>    descriptors;
    getRois(map_rows, map_cols, cell_size, rois);
    extractor->extract(img, rois, descriptors);


    /// CALCULATE
    std::map<int, float>probs;
    int    step     = terra_mat.step / terra_mat.elemSize1();
    int    channels = terra_mat.channels();
    float *data     = (float*) terra_mat.data;
    for(int i = 0 ; i < map_rows ; i++) {
        for(int j = 0 ; j < map_cols ; j++) {
            int pixel_pos = step * i + j * channels;
            probs.clear();
            /// EXTRACT
            cv::Mat &d = descriptors[i * map_cols + j];

            /// CLASSIFY
            if(d.rows > 1) {
                if(use_max_prob)
                    classifier->predictClassProbsMultiSampleMax(d, probs);
                else
                    classifier->predictClassProbsMultiSample(d, probs);
            } else {
                classifier->predictClassProbs(d, probs);
            }

            /// ENTER
            for(std::map<int, float>::iterator it = probs.begin() ; it != probs.end() ; it++) {
                int channel = channel_mapping[it->first];
                assert(channel_mapping.size() <= classes);
                data[pixel_pos + channel] = it->second;
            }
        }
    }
}
#endif // TERRA_MAP_HPP
