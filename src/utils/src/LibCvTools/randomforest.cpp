#include "randomforest.h"

RandomForest::RandomForest() :
    forest_(new cv::RandomTrees()),
    is_trained_(false)
{
}

bool RandomForest::load(const std::string &path)
{
    try {
        forest_->load(path.c_str());
        is_trained_ = true;
        return true;
    } catch (cv::Exception e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool RandomForest::isTrained()
{
    return is_trained_;
}

void RandomForest::predictClass(const cv::Mat &sample, int &classID)
{
    try {
        classID = forest_->predict(sample);
    } catch (cv::Exception e) {
        std::cerr << "predictClass " << e.what() << std::endl;
    }
}

void RandomForest::predictClassProb(const cv::Mat &sample, int &classID, float &prob)
{
    std::map<int, float> probs;
    prediction(sample, probs, classID);
    prob    = probs[classID];
}

void RandomForest::predictClassProbs(const cv::Mat &sample, std::map<int, float> &probs)
{
    probs.clear();
    int classID;
    prediction(sample, probs, classID);
}

void RandomForest::predictClassProbs(const cv::Mat &sample, std::vector<int> &classIDs, std::vector<float> &probs)
{
    std::map<int, float> probs_map;
    int classID;
    prediction(sample, probs_map, classID);
    for(std::map<int, float>::iterator it = probs_map.begin() ; it != probs_map.end() ; it++) {
        classIDs.push_back(it->first);
        probs.push_back(it->second);
    }
}



void RandomForest::predictClassProbMultiSample(const cv::Mat &samples, int &classID, float &prob)
{
    AccProbIndex index;
    classID = -1;
    prob    =  0.f;

    for(int i = 0 ; i < samples.rows ; i++) {
        cv::Mat descr(samples.row(i));
        int   tmp_id   = -1;
        float tmp_prob = 0.f;
        predictClassProb(descr, tmp_id, tmp_prob);

        if(tmp_id != -1) {
            if(index.find(tmp_id) == index.end()) {
                AccProb acc;
                acc.norm = 1;
                acc.prob = tmp_prob;
                index.insert(AccProbEntry(tmp_id, acc));
            } else {
                index[tmp_id].prob += tmp_prob;
                index[tmp_id].norm++;
            }
        }
    }


    for(AccProbIndex::iterator it = index.begin() ; it != index.end() ; it++) {
        AccProbEntry e = *it;

        double tmp_prob = e.second.prob / e.second.norm;
        if(tmp_prob >= prob) {
            prob    = tmp_prob;
            classID = e.first;
        }
    }
}

void RandomForest::predictClassProbMultiSampleMax(const cv::Mat &samples, int &classID, float &prob)
{
    classID = -1;
    prob    =  0.f;

    for(int i = 0 ; i < samples.rows ; i++) {
        cv::Mat descr(samples.row(i));
        int   tmp_id   = -1;
        float tmp_prob = 0.f;
        predictClassProb(descr, tmp_id, tmp_prob);

        if(tmp_prob >= prob) {
            prob    = tmp_prob;
            classID = tmp_id;
        }
    }
}

void RandomForest::predictClassProbsMultiSample(const cv::Mat &samples, std::map<int, float> &probs)
{
    AccProbIndex index;
    for(int i = 0 ; i < samples.rows ; i++) {
        cv::Mat descr(samples.row(i));
        std::map<int, float> pred;
        predictClassProbs(descr, pred);

        for(std::map<int, float>::iterator it = pred.begin() ; it != pred.end() ; it++) {
            if(index.find(it->first) == index.end()) {
                AccProb acc;
                acc.norm = 1;
                acc.prob = it->second;
                index.insert(AccProbEntry(it->first, acc));
            } else {
                index[it->first].prob += it->second;
                index[it->first].norm++;
            }
        }
    }


    for(AccProbIndex::iterator it = index.begin() ; it != index.end() ; it++) {
        AccProbEntry e = *it;

        double tmp_prob = e.second.prob / e.second.norm;
        probs.insert(std::make_pair(e.first, (float) tmp_prob));
    }
}

void RandomForest::predictClassProbsMultiSampleMax(const cv::Mat &samples, std::map<int, float> &probs)
{
    for(int i = 0 ; i < samples.rows ; i++) {
        cv::Mat descr(samples.row(i));
        std::map<int, float> tmp_probs;
        predictClassProbs(descr,tmp_probs);

        for(std::map<int, float>::iterator it = tmp_probs.begin() ; it != tmp_probs.end() ; it++) {
            if(probs.find(it->first) == probs.end()) {
                probs.insert(*it);
            } else {
                if(probs[it->first] < it->second) {
                    probs[it->first] = it->second;
                }
            }
        }
    }
}

void RandomForest::prediction(const cv::Mat &sample, std::map<int, float> &probs, int &maxClassID)
{
    int ntrees = forest_->get_tree_count();
    std::map<int,float> votes;

    /// COUNT
    try {
        int max_vote = 0;
        for(int i = 0 ; i < ntrees ; i++) {

            CvDTreeNode* prediction = forest_->get_tree(i)->predict(sample);
            int tree_classID = prediction->value;

            if(votes.find(tree_classID) == votes.end()) {
                votes.insert(std::pair<int,float>(tree_classID, 0));
            }

            votes[tree_classID] += 1;
            if(votes[tree_classID] > max_vote) {
                max_vote    = votes[tree_classID];
                maxClassID  = tree_classID;
            }
        }

        /// NORMALIZE
        for(std::map<int,float>::iterator it = votes.begin() ; it != votes.end() ; it++) {
            it->second /= (float) ntrees;
        }

    } catch (cv::Exception e) {
        std::cerr << "predictClass " << e.what() << std::endl;
    }
    probs = votes;
}



