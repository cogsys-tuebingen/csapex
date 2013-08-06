#ifndef CMP_RANDOMFOREST_H
#define CMP_RANDOMFOREST_H
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include "params.hpp"

class CMPRandomForest
{
public:
    typedef boost::shared_ptr<CMPRandomForest> Ptr;
    typedef boost::shared_ptr<cv::RandomTrees> CvForestPtr;

    CMPRandomForest();
    /// TRAINING
    void setParams(const CMPForestParams &params);
    void train(const cv::Mat &data, const cv::Mat &classes, const cv::Mat &var_type, std::vector<int> &classIDs);
    void save(const std::string &path);
    bool isTrained();

    /// USAGE
    void predictClass(const cv::Mat &sample, int &classID);
    void predictClassProb(const cv::Mat &sample, int &classID, float &prob);
    void predictClassProbs(const cv::Mat &sample, std::vector<int> &classIDs, std::vector<float> &probs);
    void getClassIDs(std::vector<int> classIDs);

private:
    CvForestPtr          forest_;
    cv::RandomTreeParams params_;

    std::vector<float>   priors_;
    std::vector<int>     classIDs_;
    bool                 is_trained_;

    void prediction(const cv::Mat &sample, std::vector<float> &probs, int &maxClassIndex);

};

#endif // CMP_RANDOMFOREST_H
