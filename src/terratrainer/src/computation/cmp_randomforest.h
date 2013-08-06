#ifndef CMP_RANDOMFOREST_H
#define CMP_RANDOMFOREST_H
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>


class CMPRandomForest
{
public:
    typedef boost::shared_ptr<CMPRandomForest> Ptr;
    typedef boost::shared_ptr<cv::RandomTrees> CvForestPtr;

    CMPRandomForest();
    bool load(const std::string &path);
    bool isTrained();

    /// USAGE
    void predictClass(const cv::Mat &sample, int &classID);
    void predictClassProb(const cv::Mat &sample, int &classID, float &prob);
    void predictClassProbs(const cv::Mat &sample, std::vector<int> &classIDs, std::vector<float> &probs);

protected:
    CvForestPtr          forest_;
    std::vector<float>   priors_;
    bool                 is_trained_;

    void prediction(const cv::Mat &sample, std::map<int, float> &probs, int &maxClassID);
};

#endif // CMP_RANDOMFOREST_H
