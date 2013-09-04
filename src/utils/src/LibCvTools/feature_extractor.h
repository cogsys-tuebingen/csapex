#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include "extractor.h"
namespace cv_extraction {
class ExtractorParams;
class KeypointParams;
class ParamsORB;
class ParamsSURF;
class ParamsSIFT;
class ParamsBRISK;
class ParamsBRIEF;
class ParamsFREAK;

class FeatureExtractor : public Extractor
{
public:
    typedef boost::shared_ptr<FeatureExtractor>         Ptr;
    typedef boost::shared_ptr<cv::DescriptorExtractor>  CvExPtr;
    typedef std::vector<cv::KeyPoint>                   KeyPoints;

    FeatureExtractor();

    /// SET THE PARAMETERS UP
    void set(cv::DescriptorExtractor* extractor);
    void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors);
    void extract(const cv::Mat &image, const cv::Rect &roi, cv::Mat &descriptors);
    void extract(const cv::Mat &image, const std::vector<cv::Rect> &rois, std::vector<cv::Mat> &descriptors);

    void setParams(const cv_extraction::ParamsORB              &params);
    void setParams(const cv_extraction::ParamsSURF             &params);
    void setParams(const cv_extraction::ParamsSIFT             &params);
    void setParams(const cv_extraction::ParamsBRISK            &params);
    void setParams(const cv_extraction::ParamsBRIEF            &params);
    void setParams(const cv_extraction::ParamsFREAK            &params);
    void setKeyPointParams(const cv_extraction::KeypointParams &key);
    KeypointParams keypointParams();

    /// PUBLIC STATIC METHODS
    static cv::KeyPoint prepareKeypoint(const cv::Rect &rect, const KeypointParams &params);
    static KeyPoints prepareKeypointVec(const cv::Rect &rect, const KeypointParams &params);
    static KeyPoints prepareOctaveKeypoints(const cv::Rect &rect, const KeypointParams &params,
                                            const int max_octave, const int tupel_start_idx = 0);
    static double    calcAngle(const cv::Mat &image);

    static cv::DescriptorExtractor *getExtractor(const ParamsORB   &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsSURF  &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsSIFT  &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsBRISK &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsBRIEF &params);
    static cv::DescriptorExtractor *getExtractor(const ParamsFREAK &params);
    static void makeOpp(cv::DescriptorExtractor *ptr);

protected:
    CvExPtr                             extractor_;
    boost::shared_ptr<KeypointParams>   key_params_;

};

}
#endif // FEATURE_EXTRACTOR_H
