#ifndef CMP_EXTRACTOR_EXTENDED_H
#define CMP_EXTRACTOR_EXTENDED_H
#include <utils/LibCvTools/extractor.h>
#include <yaml-cpp/yaml.h>
#include <roi.hpp>
#include "params.hpp"

class CMPCVExtractorExt : public CVExtractor
{
public:
    typedef boost::shared_ptr<CMPCVExtractorExt> Ptr;

    CMPCVExtractorExt();
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois);

    void setParams(CMPParamsORB   &params);
    void setParams(CMPParamsSURF  &params);
    void setParams(CMPParamsSIFT  &params);
    void setParams(CMPParamsBRISK &params);
    void setParams(CMPParamsBRIEF &params);
    void setParams(CMPParamsFREAK &params);
    void setKeyPointParams(KeypointParams &key);

private:
    CMPExtractorParams::Type type_;
    KeypointParams           key_;
    int                      max_octave_;
    bool                     color_extension_;
    bool                     combine_descriptors_;

    void  setCommonParameters(CMPExtractorParams &params);

};

class CMPPatternExtractorExt : public PatternExtractor
{
public:
    typedef boost::shared_ptr<CMPPatternExtractorExt> Ptr;
    CMPPatternExtractorExt();

    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois);

    void setParams(const CMPParamsLBP &params);
    void setParams(const CMPParamsLTP &params);
private:
    CMPExtractorParams::Type type_;
    bool  color_extension_;
    bool  combine_descriptors_;
};


#endif // CMP_EXTRACTOR_EXTENDED_H
