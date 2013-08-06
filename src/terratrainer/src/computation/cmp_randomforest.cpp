#include "cmp_randomforest.h"

CMPRandomForest::CMPRandomForest() :
    is_trained_(false)
{
    setParams(CMPForestParams());
}

void CMPRandomForest::setParams(const CMPForestParams &params)
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

void CMPRandomForest::train(const cv::Mat &data, const cv::Mat &classes, const cv::Mat &var_type, std::vector<int> &classIDs)
{
    classIDs_ = classIDs;
    try {
        is_trained_ = forest_->train(data, CV_ROW_SAMPLE, classes, cv::Mat(), cv::Mat(), var_type, cv::Mat(), params_);
    } catch (cv::Exception e) {
        std::cerr << e.what() << std::endl;
    }
}

void CMPRandomForest::save(const std::string &path)
{
    forest_->save(path.c_str());
}

bool CMPRandomForest::isTrained()
{
    return is_trained_;
}

void CMPRandomForest::predictClass(const cv::Mat &sample, int &classID)
{
    classID = forest_->predict(sample);
}

void CMPRandomForest::predictClassProb(const cv::Mat &sample, int &classID, float &prob)
{
    std::vector<float> probs;
    int                index;
    prediction(sample, probs, index);

    classID = classIDs_[index];
    prob    = probs[index];
}

void CMPRandomForest::predictClassProbs(const cv::Mat &sample, std::vector<int> &classIDs, std::vector<float> &probs)
{
    int index;
    prediction(sample, probs, index);
    classIDs = classIDs_;
}

void CMPRandomForest::getClassIDs(std::vector<int> classIDs)
{
    classIDs = classIDs_;
}

void CMPRandomForest::prediction(const cv::Mat &sample, std::vector<float> &probs, int &maxClassIndex)
{
    int ntrees = forest_->get_tree_count();
    int maxClassID;
    std::map<int,int> votes;
    std::map<int,int> indexes;

    for(int i = 0 ; i < classIDs_.size() ; i++) {
        votes.insert(std::pair<int,int>(classIDs_[i], 0));
        indexes.insert(std::pair<int,int>(classIDs_[i], i));
    }
    int max_vote = 0;
    for(int i = 0 ; i < ntrees ; i++) {
        CvDTreeNode* prediction = forest_->get_tree(i)->predict(sample);
        int tree_classID = prediction->value;

        votes[tree_classID]++;
        if(votes[tree_classID] > max_vote) {
            max_vote = votes[tree_classID];
            maxClassID = tree_classID;
        }
    }

    for(int i = 0 ; i < classIDs_.size() ; i++) {
        probs.push_back(votes[i] / (float) ntrees);
    }
    maxClassIndex = indexes[maxClassID];
}
