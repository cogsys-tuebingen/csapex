#include "cmp_extractor.h"
#include <cmath>

CMPExtractor::CMPExtractor()
{
}

void CMPExtractor::set(cv::DescriptorExtractor *extractor)
{
    extractor_.reset(extractor);
}

void CMPExtractor::extract(const cv::Mat &image, cv::Mat &descriptors)
{
    std::vector<cv::KeyPoint> key_points;
    cv::KeyPoint k(image.cols / 2.0, image.rows / 2.0, image.cols);
    k.octave = 0;
    key_points.push_back(k);
    extract(image, key_points, descriptors);
}

void CMPExtractor::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors)
{
    extractor_->compute(image, key_points, descriptors);
}

