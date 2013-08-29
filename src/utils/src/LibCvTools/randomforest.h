#ifndef CMP_RANDOMFOREST_H
#define CMP_RANDOMFOREST_H
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>


class RandomForest
{
public:
    typedef boost::shared_ptr<RandomForest> Ptr;
    typedef boost::shared_ptr<cv::RandomTrees> CvForestPtr;



    RandomForest();
    bool load(const std::string &path);
    bool isTrained();

    /// USAGE
    void predictClass(const cv::Mat &sample, int &classID);
    void predictClassProb(const cv::Mat &sample, int &classID, float &prob);
    void predictClassProbs(const cv::Mat &sample, std::map<int, float> &probs);
    void predictClassProbs(const cv::Mat &sample, std::vector<int> &classIDs, std::vector<float> &probs);
    void predictClassProbMultiSample    (const cv::Mat &samples, int &classID, float &prob);
    void predictClassProbMultiSampleMax (const cv::Mat &samples, int &classID, float &prob);
protected:
    struct AccProb {
        AccProb() : prob(0), norm(1){}

        double prob;
        int    norm;
    };
    typedef std::pair<int, AccProb> AccProbEntry;
    typedef std::map<int, AccProb>  AccProbIndex;


    CvForestPtr          forest_;
    std::vector<float>   priors_;
    bool                 is_trained_;

    void prediction         (const cv::Mat &sample, std::map<int, float> &probs, int &maxClassID);
};

#endif // CMP_RANDOMFOREST_H
