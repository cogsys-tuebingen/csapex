#include "extractor.h"
#include <cmath>

CVExtractor::CVExtractor()
{
}

void CVExtractor::set(cv::DescriptorExtractor *extractor)
{
    extractor_.reset(extractor);
}

void CVExtractor::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors)
{
    extractor_->compute(image, key_points, descriptors);

    if(descriptors.empty() || descriptors.rows == 0) {
        std::cerr << "Something while extraction went wrong!" << std::endl;
    }
}

void CVExtractor::extract(const Mat &image, const cv::Rect roi, const KeypointParams &params, const int max_octave,
                          const bool color_extension, const bool large,
                          cv::Mat &descriptors)
{
    cv::Mat         roi_img;
    cv::Mat         roi_col(image, roi);
    cv::Scalar      mean = extractMeanColorRGBHSV(roi_col);
    KeypointParams  kp = params;

    if(kp.soft_crop)
        roi_img = image;
    else
        roi_img = roi_col;

    if(kp.calc_angle)
        kp.angle = calcAngle(roi_col);

    CVExtractor::KeyPoints k;
    if(kp.octave == -1 && max_octave != 1) {
        k = prepareOctaveKeypoints(roi, kp, max_octave);
    } else {
        k = prepareKeypoint(roi, kp);
    }

    extract(roi_img, k, descriptors);

    if(large && !k.empty()) {
        descriptors = descriptors.reshape(0, 1);
    }

    if(color_extension) {
        addColorExtension(descriptors, mean);
    }
}

CVExtractor::KeyPoints CVExtractor::prepareKeypoint(const cv::Rect &rect, const KeypointParams &params)
{
    /// TODO : CHECK THE KEYPOINT PROPERTIES FOR DIFFERENT EXTRACTORS
    KeyPoints key_points;
    cv::KeyPoint k(rect.width / 2.0, rect.height / 2.0, rect.height / 2.0 * params.scale, params.angle);
    if(params.soft_crop) {
        k.pt.x += rect.x;
        k.pt.y += rect.y;
    }
    k.octave = params.octave;

    key_points.push_back(k);
    return key_points;
}

CVExtractor::KeyPoints CVExtractor::prepareOctaveKeypoints(const cv::Rect &rect, const KeypointParams &params, const int max_octave)
{
    KeyPoints key_points;
    cv::KeyPoint k(rect.width / 2.0, rect.height / 2.0, rect.height / 2.0 * params.scale, params.angle);
    if(params.soft_crop) {
        k.pt.x += rect.x;
        k.pt.y += rect.y;
    }

    for(int octave = 0 ; octave < max_octave ; octave++) {
        k.octave = octave;
        key_points.push_back(k);
    }
    return key_points;
}

double CVExtractor::calcAngle(const Mat &image)
{
    cv::Mat process;
    if(image.type() == CV_8UC3) {
        cv::cvtColor(image, process, CV_BGR2GRAY);
    }

    bool zero_continue_y = image.rows % 2 == 1;
    bool zero_continue_x = image.cols % 2 == 1;

    double m10 = 0;
    double m01 = 0;
    int y = -image.rows / 2;
    int x = -image.cols / 2;

    for(int i = 0 ; i < image.rows ; i++) {
        for(int j = 0 ; j < image.cols ; j++){
            if(!zero_continue_x || x + j != 0)
                m10 += (x + j) * process.at<uchar>(i,j);
            if(!zero_continue_y || y + i != 0)
                m01 += (y + i) * process.at<uchar>(i,j);

        }
    }

    return std::atan2(m01, m10);

}

PatternExtractor::PatternExtractor() :
    type(NOT_SET),
    k(0.0)
{
}

void PatternExtractor::extract(const cv::Mat &image, cv::Mat &descriptors)
{
    switch(type) {
    case LBP:
        extractLBP(image, descriptors);
        break;
    case LTP:
        extractLTP(image, descriptors);
        break;
    default:
        std::cerr << "No pattern set, computation not possible!" << std::endl;
    }
}

void PatternExtractor::extract(const Mat &image,
                               const bool color_extension,
                               const bool large, Mat &descriptors)
{
    extract(image, descriptors);
    if(color_extension) {
        cv::Scalar  mean = extractMeanColorRGBHSV(image);

        if(large) {
            descriptors.reshape(0, 1);
        }

        if(color_extension) {
            addColorExtension(descriptors, mean);
        }

    }
}

void PatternExtractor::set(cv_local_patterns::LBP *bp)
{
    type = LBP;
    pattern = bp;
}

void PatternExtractor::set(cv_local_patterns::LTP *tp)
{
    type = LTP;
    pattern = tp;
}

void PatternExtractor::setK(const double value)
{
    k = value;
}

void PatternExtractor::extractLBP(const Mat &image, Mat &descriptors)
{
    cv_local_patterns::LBP *ptr = static_cast<cv_local_patterns::LBP*>(pattern);
    ptr->stdExtraction<uchar>(image);
    descriptors = ptr->getHistogram();
}

void PatternExtractor::extractLTP(const Mat &image, Mat &descriptors)
{
    cv_local_patterns::LTP *ptr = static_cast<cv_local_patterns::LTP*>(pattern);
    ptr->stdExtraction<uchar>(image, k);
    cv::Mat pos = ptr->getPos();
    cv::Mat neg = ptr->getNeg();

    descriptors = cv::Mat_<int>(2, 256, 0);
    cv::Mat roi_pos(descriptors, cv::Rect(0,0,256,1));
    cv::Mat roi_neg(descriptors, cv::Rect(0,1,256,1));
    pos.copyTo(roi_pos);
    neg.copyTo(roi_neg);

}
