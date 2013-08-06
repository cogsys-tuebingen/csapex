#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include "extractors.hpp"

CMPCore::CMPCore() :
    extractor_(new CMPExtractorExt),
    random_(new CMPRandomForestExt),
    work_path_("/tmp"),
    file_extraction_("/extract.yaml"),
    file_tree_("/random.yaml")
{
}

void CMPCore::setWorkPath(const std::string &work_path)
{
    work_path_ = work_path;
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

bool CMPCore::loadClass(const std::string class_path)
{
    /// TODO LOAD
}

void CMPCore::compute()
{
    /// STEP 1 - THE EXTRACTION
    extract();
    /// STEP 2 - THE TRAINING
    train();
}

void CMPCore::setRois(const std::vector<ROI> &rois)
{
    rois_ = rois;
}

void CMPCore::addClass(int classID)
{
    classIDs_.push_back(classID);
}

void CMPCore::removeClass(int classID)
{
    std::vector<int>::iterator it = std::find(classIDs_.begin(), classIDs_.end(), classID);
    if(it != classIDs_.end()) {
        classIDs_.erase(it);
    }
}

void CMPCore::setRandomForestParams(const CMPForestParams &params)
{
    random_->setParams(params);
}

void CMPCore::extract()
{
    std::ofstream  out((work_path_ + file_extraction_).c_str());
    YAML::Emitter  emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "classCount" << YAML::Value << classIDs_.size();
    emitter << YAML::Key << "classes"    << YAML::Value << YAML::Flow;
    emitter << YAML::BeginSeq;
    for(std::vector<int>::iterator it = classIDs_.begin() ; it != classIDs_.end() ; it++) {
        emitter << *it;
    }
    emitter << YAML::EndSeq;
    emitter << YAML::Key << "data" << YAML::Value;
    extractor_->extractToYAML(emitter, raw_image_, rois_);
    emitter << YAML::EndMap;
    out << emitter.c_str();
    out.close();
}

void CMPCore::train()
{
    bool trained = random_->trainFromData(work_path_ + file_extraction_);
    if(trained) {
            random_->save(work_path_ + file_tree_);
    } else {
        std::cerr << "Couldn't train classifier!" << std::endl;
    }
}
