#ifndef CMP_EXTRACTION_HPP
#define CMP_EXTRACTION_HPP
#include <utils/LibCvTools/extractor.h>
#include <utils/LibCvTools/extractor_params.h>
#include <roi.hpp>
#include "yaml.hpp"
#include "cmp_state_publisher.hpp"

using namespace cv_extraction;

namespace CMPExtraction {
void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, const cv_extraction::Extractor::Ptr extractor,
                   std::vector<cv_roi::TerraROI> rois)
{
    std::vector<cv::Rect> rects;
    std::vector<cv::Mat>  descriptors;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; ++it)
        rects.push_back(it->roi.rect);

    extractor->extract(img, rects, descriptors);
    for(int i = 0 ; i < descriptors.size() ; ++i) {
        cv::Mat desc = descriptors[i];
        CMPYAML::writeDescriptorRows<float, float>(desc, rois[i].id.id, emitter);
    }
}

void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, const cv_extraction::Extractor::Ptr extractor,
                   std::vector<cv_roi::TerraROI> rois, CMPStatePublisher::Ptr state)
{
    std::vector<cv::Rect> rects;
    std::vector<cv::Mat>  descriptors;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; ++it)
        rects.push_back(it->roi.rect);

    state->publish(std::make_pair(0,0));
    extractor->extract(img, rects, descriptors);

    for(int i = 0 ; i < descriptors.size() ; ++i) {
        cv::Mat desc = descriptors[i];
        CMPYAML::writeDescriptorRows<float, float>(desc, rois[i].id.id, emitter);
        state->publish(std::make_pair(0,i));
    }
}
}

#endif // CMP_EXTRACTION_HPP
