#include "cmp_extractor_extended.h"
#include "cmp_extractors.hpp"
#include "yaml.hpp"

CMPCVExtractorExt::CMPCVExtractorExt() :
    max_octave_(0),
    color_extension_(false),
    combine_descriptors_(false)
{
}

void CMPCVExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois)
{
    emitter << YAML::BeginSeq;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        cv::Mat desc;
        extract(img, it->roi.rect, key_, max_octave_, color_extension_, combine_descriptors_, desc);

        if(type_ == CMPExtractorParams::SURF || type_ == CMPExtractorParams::SIFT) {
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
        } else  {
            CMPYAML::writeDescriptorRows<uchar, int>(desc, it->id.id, emitter);
        }
    }

    emitter << YAML::EndSeq;
}

void CMPCVExtractorExt::setParams(CMPParamsORB &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(CMPParamsSURF &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(CMPParamsSIFT &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(CMPParamsBRISK &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(CMPParamsBRIEF &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setParams(CMPParamsFREAK &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    setCommonParameters(params);
}

void CMPCVExtractorExt::setKeyPointParams(KeypointParams &key)
{
    key_ = key;
}

void CMPCVExtractorExt::setCommonParameters(CMPExtractorParams &params)
{
    max_octave_             = params.octaves;
    type_                   = params.type;
    color_extension_        = params.colorExtension;
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


void CMPPatternExtractorExt::setParams(const CMPParamsLBP &params)
{
    PatternExtractor::set(new cv_local_patterns::LBP);
    k = 0;
    color_extension_ = params.colorExtension;
    combine_descriptors_ = params.combine_descriptors;
}

void CMPPatternExtractorExt::setParams(const CMPParamsLTP &params)
{
    PatternExtractor::set(new cv_local_patterns::LTP);
    k = params.k;
    color_extension_ = params.colorExtension;
    combine_descriptors_ = params.combine_descriptors;
}
