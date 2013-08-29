#ifndef CMP_EXTRACTOR_EXTENDED_H
#define CMP_EXTRACTOR_EXTENDED_H
#include <utils/LibCvTools/feature_extractor.h>
#include <utils/LibCvTools/pattern_extractor.h>
#include <utils/LibCvTools/extractor_params.h>
#include <roi.hpp>
#include "cmp_state_publisher.hpp"

class CMPFeatureExtractorExt : public cv_extraction::FeatureExtractor
{
public:
    typedef boost::shared_ptr<CMPFeatureExtractorExt> Ptr;

    CMPFeatureExtractorExt();
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois);
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois,
                       CMPStatePublisher::Ptr state);
};


#endif // CMP_EXTRACTOR_EXTENDED_H
