#include "pattern_extractor.h"
#include "extractor_params.h"

using namespace cv_extraction;

PatternExtractor::PatternExtractor() :
    type_(NOT_SET),
    k(0.0)
{
}

void PatternExtractor::extract(const Mat &image, const Rect &roi, cv::Mat &descriptors)
{
    cv::Mat img_roi = cv::Mat(image, roi);

    if(image.type() != CV_8UC1) {
        std::cerr << "img.type != CV_8UC1" << std::endl;
        return;
    }

    switch(type_) {
    case LBP:
        extractLBP(img_roi, descriptors);
        break;
    case LTP:
        extractLTP(img_roi, descriptors);
        break;
    default:
        std::cerr << "No pattern set, computation not possible!" << std::endl;
    }

    if(ext_params_->combine_descriptors) {
        descriptors = descriptors.reshape(0, 1);
    }

}

void PatternExtractor::extract(const Mat &image, const std::vector<cv::Rect> &rois, std::vector<Mat> &descriptors)
{
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    bool use_color = ext_params_->color_extension;
    for(std::vector<cv::Rect>::const_iterator it = rois.begin() ; it != rois.end() ; it++) {
        cv::Mat d;
        extract(gray_image, *it, d);
        if(use_color) {
            cv::Mat     img_roi(image, *it);
            cv::Vec2b   mean = cv_extraction::Extractor::extractMeanColorRGBYUV(img_roi);
            cv_extraction::Extractor::addColorExtension(d, mean);
        }

        d.convertTo(d, CV_32FC1);
        descriptors.push_back(d);
    }
}

void PatternExtractor::set(cv_local_patterns::LBP *bp)
{
    type_ = LBP;
    pattern_.reset(bp);
}

void PatternExtractor::set(cv_local_patterns::LTP *tp)
{
    type_ = LTP;
    pattern_.reset(tp);
}

void PatternExtractor::extractLBP(const Mat &image, Mat &descriptors)
{
    cv_local_patterns::LBP *ptr = static_cast<cv_local_patterns::LBP*>(pattern_.get());
    ptr->stdExtraction<uchar>(image);
    descriptors = ptr->getHistogram();
}

void PatternExtractor::extractLTP(const Mat &image, Mat &descriptors)
{
    cv_local_patterns::LTP *ptr = static_cast<cv_local_patterns::LTP*>(pattern_.get());
    ptr->stdExtraction<uchar>(image, k);
    cv::Mat pos = ptr->getPos();
    cv::Mat neg = ptr->getNeg();

    descriptors = cv::Mat_<int>(2, 256, 0);
    cv::Mat roi_pos(descriptors, cv::Rect(0,0,256,1));
    cv::Mat roi_neg(descriptors, cv::Rect(0,1,256,1));
    pos.copyTo(roi_pos);
    neg.copyTo(roi_neg);

}

void PatternExtractor::setK(const double value)
{
    k = value;
}

void PatternExtractor::setParams(const ParamsLBP &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
}

void PatternExtractor::setParams(const ParamsLTP &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
    setK(params.k);
}

cv_local_patterns::LBP * PatternExtractor::getExtractor(const cv_extraction::ParamsLBP &params)
{
    cv_local_patterns::LBP *lbp = new cv_local_patterns::LBP;
    return lbp;
}

cv_local_patterns::LTP *PatternExtractor::getExtractor(const cv_extraction::ParamsLTP &params)
{
    cv_local_patterns::LTP *ltp = new cv_local_patterns::LTP;
    return ltp;
}

