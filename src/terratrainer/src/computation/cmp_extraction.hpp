#ifndef CMP_EXTRACTION_HPP
#define CMP_EXTRACTION_HPP
#include <utils/LibCvTools/extractor.h>
#include <utils/LibCvTools/extractor_params.h>
#include <roi.hpp>
#include "yaml.hpp"
#include "cmp_state_publisher.hpp"

using namespace cv_extraction;

namespace CMPExtraction {
void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img_gray, const cv::Mat &img, const bool color_ext, const cv_extraction::Extractor::Ptr extractor,
                   std::vector<cv_roi::TerraROI> rois)
{
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        cv::Mat desc;
        extractor->extract(img_gray, it->roi.rect, desc);

        if(color_ext) {
            cv::Mat     img_roi(img, it->roi.rect);
            cv::Vec2f   mean = cv_extraction::Extractor::extractMeanColorRGBYUV(img_roi);
            cv_extraction::Extractor::addColorExtension(desc, mean);
        }

        switch(extractor->params().type) {
        case ExtractorParams::SURF:
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
            break;
        case ExtractorParams::SIFT:
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
            break;
        case ExtractorParams::LBP:
            CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
            break;
        case ExtractorParams::LTP:
            CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
            break;
        default:
            CMPYAML::writeDescriptorRows<uchar, int>(desc, it->id.id, emitter);
        }
    }
}

void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img_gray, const cv::Mat &img, const bool color_ext, const cv_extraction::Extractor::Ptr extractor,
                   std::vector<cv_roi::TerraROI> rois, CMPStatePublisher::Ptr state)
{
    std::pair<int,int> counts(0, rois.size());

    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++, counts.first++) {
        cv::Mat desc;
        extractor->extract(img_gray, it->roi.rect, desc);

        if(color_ext) {
            cv::Mat     img_roi(img, it->roi.rect);
            cv::Vec2f   mean = cv_extraction::Extractor::extractMeanColorRGBYUV(img_roi);
            cv_extraction::Extractor::addColorExtension(desc, mean);
        }

        switch(extractor->params().type) {
        case ExtractorParams::SURF:
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
            break;
        case ExtractorParams::SIFT:
            CMPYAML::writeDescriptorRows<float, float>(desc, it->id.id, emitter);
            break;
        case ExtractorParams::LBP:
            CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
            break;
        case ExtractorParams::LTP:
            CMPYAML::writeDescriptorRows<int, int>(desc, it->id.id, emitter);
            break;
        default:
            CMPYAML::writeDescriptorRows<uchar, int>(desc, it->id.id, emitter);
        }

        state->publish(counts);
    }

}

}

#endif // CMP_EXTRACTION_HPP
