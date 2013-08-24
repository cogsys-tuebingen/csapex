#ifndef CMP_EXTRACTOR_EXTENDED_H
#define CMP_EXTRACTOR_EXTENDED_H
#include <utils/LibCvTools/feature_extractor.h>
#include <utils/LibCvTools/pattern_extractor.h>
#include <utils/LibCvTools/extractor_params.h>
#include <roi.hpp>
#include "cmp_state_publisher.hpp"

class CMPCVExtractorExt : public cv_extraction::FeatureExtractor
{
public:
    typedef boost::shared_ptr<CMPCVExtractorExt> Ptr;

    CMPCVExtractorExt();
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois);
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois,
                       CMPStatePublisher::Ptr state);


    void setParams(const cv_extraction::ParamsORB              &params);
    void setParams(const cv_extraction::ParamsSURF             &params);
    void setParams(const cv_extraction::ParamsSIFT             &params);
    void setParams(const cv_extraction::ParamsBRISK            &params);
    void setParams(const cv_extraction::ParamsBRIEF            &params);
    void setParams(const cv_extraction::ParamsFREAK            &params);
    void setKeyPointParams(const cv_extraction::KeypointParams &key);

private:
    cv_extraction::ExtractorParams::Type type_;
    cv_extraction::KeypointParams        key_;
    int                                  max_octave_;
    bool                                 color_extension_;
    bool                                 combine_descriptors_;

    void  setCommonParameters(const cv_extraction::ExtractorParams &params);

};

class CMPPatternExtractorExt : public cv_extraction::PatternExtractor
{
public:
    typedef boost::shared_ptr<CMPPatternExtractorExt> Ptr;
    CMPPatternExtractorExt();

    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois);
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois,
                       CMPStatePublisher::Ptr state);


    void setParams(const cv_extraction::ParamsLBP &params);
    void setParams(const cv_extraction::ParamsLTP &params);
private:
    cv_extraction::ExtractorParams::Type type_;
    bool                                 color_extension_;
    bool                                 combine_descriptors_;
};


#endif // CMP_EXTRACTOR_EXTENDED_H
