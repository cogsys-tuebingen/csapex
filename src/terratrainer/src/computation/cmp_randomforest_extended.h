#ifndef CMP_RANDOMFOREST_EXTENDED_H
#define CMP_RANDOMFOREST_EXTENDED_H
#include <utils/LibCvTools/randomforest.h>
#include "params.hpp"

class CMPRandomForestExt : public RandomForest
{
public:
    typedef boost::shared_ptr<CMPRandomForestExt> Ptr;
    CMPRandomForestExt();

    /// TRAINING
    void setParams(const CMPForestParams &params);
    void train(const cv::Mat &data, const cv::Mat &classes, const cv::Mat &var_type);
    bool trainFromData(const std::string &path);
    void save(const std::string &path);

protected:
    bool readTrainingData(const std::string &training_data, cv::Mat &data, cv::Mat &classes, cv::Mat &var_type);

private:
    cv::RandomTreeParams params_;
    bool    combine_descriptors_;
};

#endif // CMP_RANDOMFOREST_EXTENDED_H
