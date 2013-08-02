#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include "extractors.hpp"

CMPCore::CMPCore() :
    extractor_(new CMPExtractor)
{
}

bool CMPCore::loadImage(const std::string image_path)
{
    raw_image_ = cv::imread(image_path);
    return !raw_image_.empty();
}

cv::Mat CMPCore::getImage() const
{
    return raw_image_.clone();
}

void CMPCore::setRois(const std::vector<ROI> &rois)
{
    rois_ = rois;
}

void CMPCore::compute()
{
    /// STEP 1
    extract();
}

void CMPCore::setWorkPath(const std::string &work_path)
{
    work_path_ = work_path;
}

void CMPCore::extract()
{
    std::ofstream  out((work_path_ + "/compile.yaml").c_str());
    YAML::Emitter  emitter;
    emitter << YAML::BeginSeq;
    for(std::vector<ROI>::iterator it = rois_.begin() ; it != rois_.end() ; it++) {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "class" << YAML::Value << it->classId;
        CMPCore::Keys k = prepareKeypoint(it->bounding);
        cv::Mat roi(raw_image_, it->bounding);
        cv::Mat desc;
        extractor_->extract(roi,k, desc);
        emitter << YAML::Key << "descrStep" << YAML::Value << desc.cols;
        emitter << YAML::Key << "descr" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        writeMatrix(desc, emitter);
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;
        out << emitter.c_str();
    }
    emitter << YAML::EndSeq;
    out << emitter.c_str();
    out.close();
}

void CMPCore::writeMatrix(const cv::Mat &mat, YAML::Emitter &emitter)
{
    for(int i = 0 ; i < mat.rows ; i++) {
        for(int j = 0 ; j < mat.cols ; j++)
            emitter << mat.at<float>(i,j);
    }
}

CMPCore::Keys CMPCore::prepareKeypoint(cv::Rect rect)
{
    /// caseses
    CMPCore::Keys key_points;
    cv::KeyPoint k(rect.width / 2.0, rect.height / 2.0, rect.height);
    k.octave = 0;
    key_points.push_back(k);
    return key_points;
}
