#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include <time.h>
#include "extractors.hpp"

CMPCore::CMPCore() :
    extractor_(new CMPExtractorExt),
    random_(new CMPRandomForestExt),
    work_path_("/tmp"),
    file_extraction_("/extract.yaml"),
    file_forest_("/forest.yaml"),
    file_forest_meta_("/forest_meta.yaml")
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

bool CMPCore::loadForest(const std::string path, const std::string filename)
{
    classIDs_.clear();
    std::ifstream in((path + filename).c_str());
    if(!in.is_open()) {
        std::cerr << "Couldn't open meta file!" << std::endl;
        return false;
    }

    YAML::Parser parser(in);
    YAML::Node   doc;
    parser.GetNextDocument(doc);
    std::string forest;
    int class_count;
    try {
        doc["forest"] >> forest;
        doc["classCount"] >> class_count;

        const YAML::Node &docClasses = doc["classes"];
        for(YAML::Iterator it = docClasses.begin() ; it != docClasses.end() ; it++) {
            const YAML::Node &entry = (*it);
            int classID;
            entry >> classID;
            classIDs_.push_back(classID);
        }
    } catch (YAML::Exception e) {
        std::cerr << "Meta file corrupt! " << e.what() << std::endl;
        return false;
    }

    if(class_count != classIDs_.size()) {
        std::cerr << "Corrupt file, class count is not matching!" << std::endl;
        return false;
    }

    return random_->load((path + forest).c_str());
}

void CMPCore::compute()
{
    /// STEP 1 - THE EXTRACTION
    extract();
    /// STEP 2 - THE TRAINING
    train();
}

std::string CMPCore::metaPath()
{
    return work_path_ + file_forest_meta_;
}

std::string CMPCore::forestPath()
{
    return work_path_ + file_forest_;
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
    writeClasses(emitter);
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
        writeMetaFile();
    } else {
        std::cerr << "Couldn't train classifier!" << std::endl;
    }
}

void CMPCore::writeClasses(YAML::Emitter &emitter)
{
    emitter << YAML::Key << "classes" << YAML::Value << YAML::Flow;
    emitter << YAML::BeginSeq;
    for(std::vector<int>::iterator it = classIDs_.begin() ; it != classIDs_.end() ; it++) {
        emitter << *it;
    }
    emitter << YAML::EndSeq;
}

void CMPCore::writeMetaFile()
{
    std::ofstream out((work_path_ + file_forest_meta_).c_str());
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "forest"     << YAML::Value << file_forest_;
    emitter << YAML::Key << "classCount" << YAML::Value << classIDs_.size();
    writeClasses(emitter);
    emitter << YAML::EndMap;
    out << emitter.c_str();
    out.close();
}
