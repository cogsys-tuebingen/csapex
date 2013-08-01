/// HEADER
#include "extractor.h"

/// PROJECT
#include <common/global.hpp>

Extractor::Extractor()
{
}

void Extractor::extract(const cv::Mat& frame, const cv::Mat& mask_roi, std::vector<cv::KeyPoint> &keypoints, cv::Mat& descriptors) const
{
    assert(valid());

    if(frame.rows < 20 || frame.cols < 20) {
        WARN("frame is too small to extract features: " << frame.rows << "x" << frame.cols);
        return;
    }

    cv::Mat gray;
    if(frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
    }

    try {
        detector->detect(gray, keypoints, mask_roi);
    } catch(cv::Exception& e) {
        ERROR("Extraction of keypoints failed");
    }

    try {
        descriptor_extractor->compute(gray, keypoints, descriptors);
    } catch(cv::Exception& e) {
        ERROR("Extraction of descriptors failed");
    }

}

bool Extractor::valid() const
{
    return !descriptor_extractor.empty() && !detector.empty();
}
