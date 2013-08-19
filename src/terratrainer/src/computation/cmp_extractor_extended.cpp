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
        cv::Mat     roi;
        cv::Mat     roi_col(img, it->roi.rect);
        cv::Scalar  mean = extractMeanColorRGBHSV(roi_col);
        if(soft_crop_)
            roi = img;
        else
            roi = roi_col;

        cv::Mat desc;

        /// MULTIOCTAVE
        CVExtractor::KeyPoints k;
        if(type_ == CMPExtractorParams::SURF || type_ == CMPExtractorParams::SIFT)
            k = prepareKeypoint(it->roi.rect, soft_crop_, scale_, angle_);
        else
            k = prepareNeighbouredKeypoint(it->roi.rect, soft_crop_, scale_, angle_);

        extract(roi, k, desc);
        if(combine_descriptors_) {
            if(type_ == CMPExtractorParams::FREAK) {
                CMPYAML::writeDescriptor<uchar, int>(desc, it->id.id, mean, color_extension_, emitter);
            } else  {
                CMPYAML::writeDescriptor<float, float>(desc, it->id.id, mean, color_extension_, emitter);
            }
        } else {
            if(type_ == CMPExtractorParams::FREAK) {
                CMPYAML::writeDescriptorsSeperated<uchar, int>(desc, it->id.id, mean, color_extension_, emitter);
            } else  {
                CMPYAML::writeDescriptorsSeperated<float, float>(desc, it->id.id, mean, color_extension_, emitter);
            }


        }
    }
    emitter << YAML::EndSeq;
}

void CMPCVExtractorExt::setParams(CMPParamsORB &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    color_extension_ = params.colorExtension;
    type_ = params.type;
}

void CMPCVExtractorExt::setParams(CMPParamsSURF &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
    type_ = params.type;
}

void CMPCVExtractorExt::setParams(CMPParamsSIFT &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
    type_ = params.type;
}

void CMPCVExtractorExt::setParams(CMPParamsBRISK &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
    type_ = params.type;
}

void CMPCVExtractorExt::setParams(CMPParamsBRIEF &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
    type_ = params.type;
}

void CMPCVExtractorExt::setParams(CMPParamsFREAK &params)
{
    CVExtractor::set(CMPExtractors::prepare(params));
    max_octave_ = params.octaves;
    color_extension_ = params.colorExtension;
    type_ = params.type;
}

void CMPCVExtractorExt::setKeyPointParams(CMPKeypointParams &key)
{
    angle_           = key.angle;
    scale_           = key.scale;
    octave_          = key.octave;
    soft_crop_       = key.soft_crop;
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
            CMPYAML::writeDescriptor<int, int>(desc, it->id.id, mean, color_extension_, emitter);
        else
            CMPYAML::writeDescriptorsSeperated<int, int>(desc, it->id.id, mean, color_extension_, emitter);
    }
    emitter << YAML::EndSeq;
}


void CMPPatternExtractorExt::setParams(const CMPParamsLBP &params)
{
    PatternExtractor::set(new cv_local_patterns::LBP);
    k = 0;
    color_extension_ = params.colorExtension;
}

void CMPPatternExtractorExt::setParams(const CMPParamsLTP &params)
{
    PatternExtractor::set(new cv_local_patterns::LTP);
    k = params.k;
    color_extension_ = params.colorExtension;
}
