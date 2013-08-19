#include "cmp_extractor_extended.h"
#include "cmp_extractors.hpp"
#include "yaml.hpp"

CMPCVExtractorExt::CMPCVExtractorExt() :
    angle_(0.f),
    scale_(1.f),
    soft_crop_(true),
    octave_(0),
    max_octave_(0),
    color_extension_(false),
    combine_descriptors_(false)
{
}

void CMPCVExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois)
{
    emitter << YAML::BeginSeq;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        /// CALCULATION
        cv::Mat     roi(img, it->roi.rect);
        cv::Scalar  mean = extractMeanColorRGBHSV(roi);


        /// COLOR EXTENSION
        cv::Mat desc;
        extract(roi, desc);
        if(combine_descriptors_)
            CMPYAML::writeDescriptor<float>(desc, it->id.id, mean, color_extension_, emitter);
        else
            writeSeperated(desc, it->id.id, mean, emitter);
    }
    emitter << YAML::EndSeq;
}

void CMPCVExtractorExt::setParams(CMPParamsORB &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    color_extension_ = params.colorExtension;
}

void CMPCVExtractorExt::setParams(CMPParamsSURF &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
}

void CMPCVExtractorExt::setParams(CMPParamsSIFT &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
}

void CMPCVExtractorExt::setParams(CMPParamsBRISK &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
}

void CMPCVExtractorExt::setParams(CMPParamsBRIEF &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
}

void CMPCVExtractorExt::setParams(CMPParamsFREAK &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
}

void CMPCVExtractorExt::setKeyPointParams(CMPKeypointParams &key)
{
    angle_           = key.angle;
    scale_           = key.scale;
    octave_          = key.octave;
    soft_crop_       = key.soft_crop;
}


void CMPCVExtractorExt::writeSeperated(const Mat &desc, const int id, const Scalar &mean, YAML::Emitter &emitter)
{
    for(int j = 0 ; j < desc.rows ; j++) {
        cv::Mat roi(desc, cv::Rect(0,j,desc.cols,1));
        CMPYAML::writeDescriptor<float>(roi, id, mean, color_extension_, emitter);
    }
}

void CMPCVExtractorExt::reset()
{
    angle_     = 0.f;
    scale_     = 1.f;
    soft_crop_ = true;
    octave_    = 0;
    max_octave_= 0;
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
        cv::Scalar  mean = extractMeanColorRGBHSV(roi);


        /// COLOR EXTENSION
        cv::Mat desc;
        extract(roi, desc);
        if(combine_descriptors_)
            CMPYAML::writeDescriptor<int>(desc, it->id.id, mean, color_extension_, emitter);
        else
            writeSeperated(desc, it->id.id, mean, emitter);
    }
    emitter << YAML::EndSeq;
}


void CMPPatternExtractorExt::setParams(const CMPParamsLBP &params)
{
    PatternExtractor::set(new cv_local_patterns::LBP);
    k_ = 0;
    color_extension_ = params.colorExtension;
}

void CMPPatternExtractorExt::setParams(const CMPParamsLTP &params)
{
    PatternExtractor::set(new cv_local_patterns::LTP);
    k_ = params.k;
    color_extension_ = params.colorExtension;
}

void CMPPatternExtractorExt::writeSeperated(const cv::Mat &desc, const int id, const cv::Scalar &mean, YAML::Emitter &emitter)
{
    for(int j = 0 ; j < desc.cols ; j++) {
        cv::Mat roi(desc, cv::Rect(j,0,1,desc.rows));
        CMPYAML::writeDescriptor<int>(roi, id, mean, color_extension_, emitter);
    }
}
