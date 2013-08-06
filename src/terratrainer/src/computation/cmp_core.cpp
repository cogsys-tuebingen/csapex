#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include "extractors.hpp"

CMPCore::CMPCore() :
    extractor_(new CMPExtractor),
    random_(new CMPRandomForest),
    file_extraction_("/extract.yaml"),
    file_tree_("/random.yaml")
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

bool CMPCore::loadClass(const std::string class_path)
{
    /// TODO LOAD
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

void CMPCore::compute()
{
    /// STEP 1 - THE EXTRACTION
    extract();
    /// STEP 2 - THE TRAINING
    train();
}

void CMPCore::setWorkPath(const std::string &work_path)
{
    work_path_ = work_path;
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
    emitter << YAML::BeginSeq;
    for(std::vector<ROI>::iterator it = rois_.begin() ; it != rois_.end() ; it++) {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "class" << YAML::Value << it->classID;
        CMPCore::Keys k = prepareKeypoint(it->bounding);
        cv::Mat roi(raw_image_, it->bounding);
        cv::Mat desc;
        extractor_->extract(roi,k, desc);
        emitter << YAML::Key << "descrStep" << YAML::Value << desc.cols;
        emitter << YAML::Key << "descr" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        writeMatrix(desc, emitter);
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
    out << emitter.c_str();
    out.close();
}

void CMPCore::train()
{
    cv::Mat classes;
    cv::Mat data;
    cv::Mat var_type;
    std::vector<int> classIDs;
    if(readTrainingData(data, classes, var_type, classIDs)) {
        random_->train(data, classes, var_type, classIDs);
        if(random_->isTrained()) {
            random_->save(work_path_ + file_tree_);
        } else {
            std::cerr << "Couldn't train classifier!" << std::endl;
        }
    }
}

bool CMPCore::readTrainingData(cv::Mat &data, cv::Mat &classes, cv::Mat &var_type, std::vector<int> &classIDs)
{
    std::ifstream in((work_path_ + file_extraction_).c_str());
    if(!in.is_open()) {
        std::cerr << "Couldn't open extraction file!" << std::endl;
        return false;
    }

    YAML::Parser parser(in);
    YAML::Node   doc;

    parser.GetNextDocument(doc);

    /// METADATA
    int classCount;
    doc["classCount"] >> classCount;

    const YAML::Node &docClasses = doc["classes"];
    for(YAML::Iterator it = docClasses.begin() ; it != docClasses.end() ; it++) {
        const YAML::Node &entry = (*it);
        int classID;
        entry >> classID;
        classIDs.push_back(classID);
    }

    std::vector<float>  descriptor_classIDs;
    std::vector<float>  descriptors;
    int   data_step = 0;
    const YAML::Node &docData = doc["data"];
    for(YAML::Iterator it = docData.begin() ; it != docData.end() ; it++) {
        const YAML::Node &entry = (*it);

        int classID;
        int descr_step;

        entry["class"]      >> classID;
        entry["descrStep"]  >> descr_step;

        if(data_step == 0) {
            data_step = descr_step;
        } else if(data_step != descr_step) {
            std::cerr << "Dropped descriptor due to variing length!" << std::endl;
            continue;
        }

        descriptor_classIDs.push_back(classID);

        const YAML::Node &descriptor = entry["descr"];
        for(YAML::Iterator it = descriptor.begin() ; it != descriptor.end() ; it++){
            float value;
            (*it) >> value;
            descriptors.push_back(value);
        }
    }

    if(data_step == 0 || descriptors.size() == 0 || descriptor_classIDs.size() == 0) {
        std::cerr << "Either no valid descriptor data or corrupted yaml file!" << std::endl;
        return false;
    }

    /// FILL MATRICES
    classes = cv::Mat(descriptor_classIDs.size(), 1, CV_32F);
    data    = cv::Mat(descriptors.size() / data_step, data_step, CV_32F);
    var_type= cv::Mat(data_step + 1, 1, CV_8U);
    memcpy(classes.data, descriptor_classIDs.data(), sizeof(float) * descriptor_classIDs.size());
    memcpy(data.data, descriptors.data(), sizeof(float) * descriptors.size());
    var_type.setTo(cv::Scalar(CV_VAR_NUMERICAL));
    var_type.at<uchar>(data_step, 0) = CV_VAR_CATEGORICAL;
    return true;
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
    /// TODO : CHECK THE KEYPOINT PROPERTIES FOR DIFFERENT EXTRACTORS
    CMPCore::Keys key_points;
    cv::KeyPoint k(rect.width / 2.0, rect.height / 2.0, rect.height);
    k.octave = 0;
    key_points.push_back(k);
    return key_points;
}
