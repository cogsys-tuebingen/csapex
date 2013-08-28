#include "cmp_extractor_extended.h"
#include "yaml.hpp"

using namespace cv_extraction;

CMPFeatureExtractorExt::CMPFeatureExtractorExt()
{
}

void CMPFeatureExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois)
{
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        cv::Mat desc;
        extract(img, it->roi.rect, desc);

        if(ext_params_->type == ExtractorParams::SURF || ext_params_->type == ExtractorParams::SIFT) {
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
        } else  {
            CMPYAML::writeDescriptorRows<uchar, int>(desc, it->id.id, emitter);
        }
    }

}

void CMPFeatureExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> rois,
                                      CMPStatePublisher::Ptr state)
{
    std::pair<int,int> counts(0, rois.size());

    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++, counts.first++) {
        cv::Mat desc;
        extract(img, it->roi.rect, desc);

        if(ext_params_->type == ExtractorParams::SURF || ext_params_->type == ExtractorParams::SIFT) {
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
        } else  {
            CMPYAML::writeDescriptorRows<uchar, int>(desc, it->id.id, emitter);
        }

        state->publish(counts);
    }

}

CMPPatternExtractorExt::CMPPatternExtractorExt()
{
}

void CMPPatternExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois)
{
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        /// CALCULATION
        cv::Mat     roi(img, it->roi.rect);

        /// COLOR EXTENSION
        cv::Mat desc;
        extract(roi, desc);
        CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
    }
}

void CMPPatternExtractorExt::extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois,
                   CMPStatePublisher::Ptr state)
{
    std::pair<int,int> counts(0, rois.size());

    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++, counts.first++) {
        /// CALCULATION
        cv::Mat     roi(img, it->roi.rect);

        /// COLOR EXTENSION
        cv::Mat desc;
        extract(roi, desc);
        CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
        state->publish(counts);
    }
}
