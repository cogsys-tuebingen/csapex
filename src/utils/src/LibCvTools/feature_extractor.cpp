#include "feature_extractor.h"
#include <cmath>

using namespace cv_extraction;

FeatureExtractor::FeatureExtractor()
{
}

void FeatureExtractor::set(cv::DescriptorExtractor *extractor)
{
    extractor_.reset(extractor);
}

void FeatureExtractor::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors)
{
    extractor_->compute(image, key_points, descriptors);

    if(descriptors.empty() || descriptors.rows == 0) {
        std::cerr << "Something while extraction went wrong!" << std::endl;
    }
}

void FeatureExtractor::extract(const cv::Mat &image, const cv::Rect roi, const KeypointParams &params, const int max_octave,
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

    FeatureExtractor::KeyPoints k;
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

FeatureExtractor::KeyPoints FeatureExtractor::prepareKeypoint(const cv::Rect &rect, const KeypointParams &params)
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

FeatureExtractor::KeyPoints FeatureExtractor::prepareOctaveKeypoints(const cv::Rect &rect, const KeypointParams &params, const int max_octave)
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

double FeatureExtractor::calcAngle(const cv::Mat &image)
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

cv::DescriptorExtractor *FeatureExtractor::getExtractor(const ParamsORB &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::ORB(500,
                        params.scale,
                        params.octaves,
                        params.edge_threshold,
                        0,
                        params.WTA_K,
                        cv::ORB::HARRIS_SCORE,
                        params.patch_size);

    if(params.opp)
        makeOpp(ptr);

    return ptr;
}

cv::DescriptorExtractor *FeatureExtractor::getExtractor(const ParamsSURF &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::SurfDescriptorExtractor(params.octaves,
                                            params.octave_layers,
                                            params.extended);
    if(params.opp)
        makeOpp(ptr);
    return ptr;
}

cv::DescriptorExtractor *FeatureExtractor::getExtractor(const ParamsSIFT &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::SiftDescriptorExtractor(params.magnification,
                                            params.normalize,
                                            params.recalculate_angles);
    if(params.opp)
        makeOpp(ptr);

    return ptr;
}

cv::DescriptorExtractor *FeatureExtractor::getExtractor(const ParamsBRISK &params)
{
    cv::DescriptorExtractor* ptr =
            new cv::BRISK(params.thresh,
                          params.octaves,
                          params.scale);

    if(params.opp)
        makeOpp(ptr);

    return ptr;
}

cv::DescriptorExtractor *FeatureExtractor::getExtractor(const ParamsBRIEF &params)
{
    return new cv::BriefDescriptorExtractor(params.bytes);
}

cv::DescriptorExtractor *FeatureExtractor::getExtractor(const ParamsFREAK &params)
{
    cv::DescriptorExtractor *ptr =
            new cv::FREAK(params.orientation_normalized,
                          params.scale_normalized,
                          params.pattern_scale,
                          params.octaves);
    if(params.opp)
        makeOpp(ptr);

    return ptr;
}

void FeatureExtractor::makeOpp(cv::DescriptorExtractor *ptr)
{
    ptr = new cv::OpponentColorDescriptorExtractor(ptr);
}
