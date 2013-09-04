#include "feature_extractor.h"
#include "extractor_params.h"
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

void FeatureExtractor::extract(const cv::Mat &image, const cv::Rect &roi, cv::Mat &descriptors)
{
    if(image.empty())
        return;
    if(image.type() != CV_8UC1) {
        std::cerr << "img.type != CV_8UC1" << std::endl;
        return;
    }


    cv::Mat         roi_img;
    KeypointParams  kp = *key_params_;
    ExtractorParams ep = *ext_params_;

    if(kp.soft_crop)
        roi_img = image;
    else
        roi_img = cv::Mat(image, roi);

    if(kp.calc_angle)
        kp.angle = calcAngle(roi_img);

    FeatureExtractor::KeyPoints k;
    if(kp.octave == -1 && ep.octaves != 1) {
        k = prepareOctaveKeypoints(roi, kp, ep.octaves);
    } else {
        k = prepareKeypointVec(roi, kp);
    }

    extract(roi_img, k, descriptors);

    if(ep.combine_descriptors && !k.empty()) {
        descriptors = descriptors.reshape(0, 1);
    }

    if(descriptors.type() != CV_32FC1) {
        descriptors.convertTo(descriptors, CV_32FC1);
    }
}

void FeatureExtractor::extract(const cv::Mat &image, const std::vector<cv::Rect> &rois, std::vector<cv::Mat> &descriptors)
{
    if(image.empty())
        return;

    cv::Mat image_gray;
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);

    KeypointParams              kp = *key_params_;
    ExtractorParams             ep = *ext_params_;
    FeatureExtractor::KeyPoints keys;
    std::vector<cv::Vec2b>      means;
    cv::Mat                     d;
    for(std::vector<cv::Rect>::const_iterator it = rois.begin() ; it != rois.end() ; it++) {
        if(kp.octave == -1 && ep.octaves != 1) {
            FeatureExtractor::KeyPoints oct_keys = prepareOctaveKeypoints(*it, kp, ep.octaves);
            keys.insert(keys.end(), oct_keys.begin(), oct_keys.end());
        } else {
            keys.push_back(prepareKeypoint(*it, kp));
        }

        if(ep.color_extension) {
            cv::Mat     img_roi(image, *it);
            means.push_back(cv_extraction::Extractor::extractMeanColorRGBYUV(img_roi));
        }

    }

    extract(image_gray, keys, d);

    int tuples = d.rows / ep.octaves;

    if(ep.combine_descriptors && !d.empty() && kp.octave == -1) {
        d = d.reshape(0, tuples);
    }

    cv::Rect roi(0,0, d.cols, ep.octaves);

    for(int i = 0 ; i < tuples ; i++) {
        roi.y = i * ep.octaves;
        cv::Mat roi_tuple(d, roi);
        cv::Mat descr = roi_tuple.clone();
        if(ext_params_->color_extension)
            cv_extraction::Extractor::addColorExtension(descr, means[i]);

        if(descr.type() != CV_32FC1) {
            descr.convertTo(descr, CV_32FC1);
        }
        descriptors.push_back(descr);
    }
}

void FeatureExtractor::setParams(const ParamsORB &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
}

void FeatureExtractor::setParams(const ParamsSURF &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
}

void FeatureExtractor::setParams(const ParamsSIFT &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
}

void FeatureExtractor::setParams(const ParamsBRISK &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
}

void FeatureExtractor::setParams(const ParamsBRIEF &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
}

void FeatureExtractor::setParams(const ParamsFREAK &params)
{
    set(getExtractor(params));
    ext_params_.reset(new ExtractorParams(params));
}

void FeatureExtractor::setKeyPointParams(const KeypointParams &key)
{
    key_params_.reset(new KeypointParams(key));
}

KeypointParams FeatureExtractor::keypointParams()
{
    return *key_params_;
}

cv::KeyPoint FeatureExtractor::prepareKeypoint(const cv::Rect &rect, const KeypointParams &params)
{
    cv::KeyPoint k(rect.width / 2.0, rect.height / 2.0, rect.height / 2.0 * params.scale, params.angle);
    if(params.soft_crop) {
        k.pt.x += rect.x;
        k.pt.y += rect.y;
    }
    k.octave = params.octave;
    return k;
}


FeatureExtractor::KeyPoints FeatureExtractor::prepareKeypointVec(const cv::Rect &rect, const KeypointParams &params)
{
    /// TODO : CHECK THE KEYPOINT PROPERTIES FOR DIFFERENT EXTRACTORS
    KeyPoints key_points;
    key_points.push_back(prepareKeypoint(rect, params));
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
