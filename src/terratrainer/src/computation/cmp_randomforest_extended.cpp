#include "cmp_randomforest_extended.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

CMPRandomForestExt::CMPRandomForestExt() :
    CMPRandomForest()
{
    setParams(CMPForestParams());
}

void CMPRandomForestExt::setParams(const CMPForestParams &params)
{
    /// RESET THE TREE
    forest_.reset(new cv::RandomTrees);

    /// BUFFER ARRAY DATA
    priors_ = params.priors;

    /// RESET THE TRAINING PARAMETERS
    params_ = cv::RandomTreeParams(
                params.max_depth,           // max depth
                params.min_samples,         // min sample count
                params.regression,          // regression accuracy: N/A here
                params.surrogates,          // compute surrogate split, no missing data
                params.max_categories,      // max number of categories (use sub-optimal algorithm for larger numbers)
                priors_.data(),             // the array of priors
                params.variable_importance, // calculate variable importance
                params.nactive_variables,   // number of variables randomly selected at node and used to find the best split(s).
                params.max_trees,           // max number of trees in the forest
                params.accurracy,           // forrest accuracy
                params.termination_criteria // termination cirteria
                );
    is_trained_ = false;
}

void CMPRandomForestExt::train(const cv::Mat &data, const cv::Mat &classes, const cv::Mat &var_type)
{
    try {
        is_trained_ = forest_->train(data, CV_ROW_SAMPLE, classes, cv::Mat(), cv::Mat(), var_type, cv::Mat(), params_);
    } catch (cv::Exception e) {
        std::cerr << e.what() << std::endl;
    }
}

bool CMPRandomForestExt::trainFromData(const std::string &path)
{
    cv::Mat classes;
    cv::Mat data;
    cv::Mat var_type;
    readTrainingData(path, data, classes, var_type);
    train(data, classes, var_type);
    return isTrained();
}

void CMPRandomForestExt::save(const std::string &path)
{
    forest_->save(path.c_str());
}

bool CMPRandomForestExt::readTrainingData(const std::string &training_data, cv::Mat &data, cv::Mat &classes, cv::Mat &var_type)
{
    std::ifstream in(training_data.c_str());
    if(!in.is_open()) {
        std::cerr << "Couldn't open extraction file!" << std::endl;
        return false;
    }

    YAML::Parser parser(in);
    YAML::Node   doc;

    /// METADATA
    std::vector<float>  descriptor_classIDs;
    std::vector<float>  descriptors;
    int   data_step = 0;
    try {
        parser.GetNextDocument(doc);
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
    } catch (YAML::Exception e) {
        std::cerr << "Curropt training file! " << e.what() << std::endl;
        return false;
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
