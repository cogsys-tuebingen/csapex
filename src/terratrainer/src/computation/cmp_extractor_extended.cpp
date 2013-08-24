#include "cmp_extractor_extended.h"
#include "yaml.hpp"

using namespace cv_extraction;

CMPCVExtractorExt::CMPCVExtractorExt() :
    max_octave_(0),
    color_extension_(false),
    combine_descriptors_(false)
{
}

void CMPCVExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois)
{
    emitter << YAML::BeginSeq;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        cv::Mat desc;
        extract(img, it->roi.rect, key_, max_octave_, color_extension_, combine_descriptors_, desc);

        if(type_ == ExtractorParams::SURF || type_ == ExtractorParams::SIFT) {
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
        } else  {
            CMPYAML::writeDescriptorRows<uchar, int>(desc, it->id.id, emitter);
        }
    }

    emitter << YAML::EndSeq;
}

void CMPCVExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois,
                                      CMPStatePublisher::Ptr state)
{
    std::pair<int,int> counts(0, rois.size());

    emitter << YAML::BeginSeq;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++, counts.first++) {
        cv::Mat desc;
        extract(img, it->roi.rect, key_, max_octave_, color_extension_, combine_descriptors_, desc);

        if(type_ == ExtractorParams::SURF || type_ == ExtractorParams::SIFT) {
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
        } else  {
            CMPYAML::writeDescriptorRows<uchar, int>(desc, it->id.id, emitter);
        }

        state->publish(counts);
    }

    emitter << YAML::EndSeq;
}

void CMPCVExtractorExt::setParams(const ParamsORB &params)
{
    FeatureExtractor::set(getExtractor(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(const ParamsSURF &params)
{
    FeatureExtractor::set(getExtractor(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(const ParamsSIFT &params)
{
    FeatureExtractor::set(getExtractor(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(const ParamsBRISK &params)
{
    FeatureExtractor::set(getExtractor(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(const ParamsBRIEF &params)
{
    FeatureExtractor::set(getExtractor(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(const ParamsFREAK &params)
{
    FeatureExtractor::set(getExtractor(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setKeyPointParams(const KeypointParams &key)
{
    key_ = key;
}

void CMPCVExtractorExt::setCommonParameters(const cv_extraction::ExtractorParams &params)
{
    max_octave_             = params.octaves;
    type_                   = params.type;
    color_extension_        = params.color_extension;
    combine_descriptors_    = params.combine_descriptors;
}


CMPPatternExtractorExt::CMPPatternExtractorExt() :
    color_extension_(false),
    combine_descriptors_(true)
{
}

void CMPPatternExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois)
{
    emitter << YAML::BeginSeq;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        /// CALCULATION
        cv::Mat     roi(img, it->roi.rect);

        /// COLOR EXTENSION
        cv::Mat desc;
        extract(roi, color_extension_, combine_descriptors_, desc);
        CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
    }
    emitter << YAML::EndSeq;
}

void CMPPatternExtractorExt::extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois,
                   CMPStatePublisher::Ptr state)
{
    std::pair<int,int> counts(0, rois.size());

    emitter << YAML::BeginSeq;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++, counts.first++) {
        /// CALCULATION
        cv::Mat     roi(img, it->roi.rect);

        /// COLOR EXTENSION
        cv::Mat desc;
        extract(roi, color_extension_, combine_descriptors_, desc);
        CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
        state->publish(counts);
    }
    emitter << YAML::EndSeq;
}


void CMPPatternExtractorExt::setParams(const cv_extraction::ParamsLBP &params)
{
    PatternExtractor::set(new cv_local_patterns::LBP);
    k = 0;
    color_extension_ = params.color_extension;
    combine_descriptors_ = params.combine_descriptors;
}

void CMPPatternExtractorExt::setParams(const cv_extraction::ParamsLTP &params)
{
    PatternExtractor::set(new cv_local_patterns::LTP);
    k = params.k;
    color_extension_ = params.color_extension;
    combine_descriptors_ = params.combine_descriptors;
}

