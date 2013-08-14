#include "cmp_extractor_extended.h"

CMPExtractorExt::CMPExtractorExt()
{
}

void CMPExtractorExt::extractToYAML(YAML::Emitter  &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois, const bool soft_crop)
{
    emitter << YAML::BeginSeq;
    for(std::vector<cv_roi::TerraROI>::iterator it = rois.begin() ; it != rois.end() ; it++) {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "class" << YAML::Value << it->id.id;
        KeyPoints k = prepareKeypoint(it->roi.rect);
        cv::Mat roi;

        if(soft_crop)
            roi = cv::Mat(img, it->roi.rect);
        else
            roi = img;

        cv::Mat desc;
        extract(roi,k, desc);
        emitter << YAML::Key << "descrStep" << YAML::Value << desc.cols;
        emitter << YAML::Key << "descr" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        writeMatrix(desc, emitter);
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;
}


void CMPExtractorExt::writeMatrix(const cv::Mat &mat, YAML::Emitter &emitter)
{
    for(int i = 0 ; i < mat.rows ; i++) {
        for(int j = 0 ; j < mat.cols ; j++)
            emitter << mat.at<float>(i,j);
    }
}
