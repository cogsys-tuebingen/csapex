#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include <time.h>
#include "extractors.hpp"
#include "yaml.hpp"

CMPCore::CMPCore() :
    extractor_(new CMPExtractorExt),
    random_(new CMPRandomForestExt),
    work_path_("/tmp"),
    file_extraction_("/extract.yaml"),
    file_forest_("/forest.yaml")
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

bool CMPCore::load(const std::string path, const std::vector<int> &classRegister)
{
    classIDs_ = classRegister;
    return random_->load(path.c_str());
}


std::string CMPCore::forestPath()
{
    return work_path_ + file_forest_;
}

void CMPCore::compute()
{
    /// STEP 1 - THE EXTRACTION
    extract();
    /// STEP 2 - THE TRAINING
    train();
}

void CMPCore::computeGrid(int cell_size)
{

}

void CMPCore::computeQuadtree(int min_cell_size)
{

}

void CMPCore::setRandomForestParams(const CMPForestParams &params)
{
    random_->setParams(params);
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

void CMPCore::getClasses(std::vector<int> &classes)
{
    classes = classIDs_;
}

void CMPCore::extract()
{
    std::ofstream  out((work_path_ + file_extraction_).c_str());
    YAML::Emitter  emitter;
    emitter << YAML::BeginMap;
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
        random_->save(work_path_ + file_forest_);
    } else {
        std::cerr << "Couldn't train classifier!" << std::endl;
    }
}
