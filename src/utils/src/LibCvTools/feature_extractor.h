#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include "extractor.hpp"
#include "extractor_params.h"

namespace cv_extraction {
class FeatureExtractor : public Extractor
{
public:
    typedef boost::shared_ptr<FeatureExtractor>         Ptr;
    typedef boost::shared_ptr<cv::DescriptorExtractor>  CvExPtr;
    typedef std::vector<cv::KeyPoint>                   KeyPoints;

    FeatureExtractor();

    void set(cv::DescriptorExtractor* extractor);
    void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors);
    void extract(const cv::Mat &image, const cv::Rect roi, const KeypointParams &params,
                 const int max_octave, const bool color_extension, const bool large,
                 cv::Mat &descriptors);

    static KeyPoints prepareKeypoint(const cv::Rect &rect, const KeypointParams &params);
    static KeyPoints prepareOctaveKeypoints(const cv::Rect &rect, const KeypointParams &params, const int max_octave);
    static double    calcAngle(const cv::Mat &image);
    static cv::DescriptorExtractor *getExtractor(const ParamsORB   &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsSURF  &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsSIFT  &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsBRISK &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsBRIEF &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsFREAK &params);
    static void makeOpp(cv::DescriptorExtractor *ptr);

protected:
    CvExPtr          extractor_;

};

}
#endif // FEATURE_EXTRACTOR_H
