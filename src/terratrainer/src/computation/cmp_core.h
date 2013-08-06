#ifndef CMP_CORE_H
#define CMP_CORE_H
/// COMPONENT
#include "params.hpp"
#include "cmp_extractor.h"
#include "cmp_randomforest.h"
#include "extractors.hpp"
/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <utils/LibCvTools/grid.hpp>
#include <utils/LibCvTools/quad_tree.hpp>

class CMPCore
{
public:
    struct ROI {
        cv::Rect    bounding;
        int         id;
        int         classID;
        double      rotation;
    };

    typedef boost::shared_ptr<CMPCore> Ptr;

    CMPCore();

    /// IMAGE
    bool    loadImage(const std::string image_path);
    cv::Mat getImage() const;
    bool    loadClass(const std::string class_path);

    /// EXTRACTION
    void    setRois(const std::vector<ROI> &rois);
    void    addClass(int classID);
    void    removeClass(int classID);
    void    compute();
    void    setWorkPath(const std::string &work_path);

    /// CREATE EXTRACTOR
    template<class Parameters>
    void setExtractorParameters(Parameters &param)
    {
        cv::DescriptorExtractor* ptr = CMPExtractors::create(param);
        if(param.opp)
            CMPExtractors::makeOpp(ptr);
        extractor_->set(ptr);
        type_ = param.type;
    }

    /// SET TREE PARAMS
    void setRandomForestParams(const CMPForestParams &params);


private:
    typedef std::vector<cv::KeyPoint> Keys;

    cv::Mat                  raw_image_;
    CMPExtractor::Ptr        extractor_;
    CMPExtractorParams::Type type_;
    CMPRandomForest::Ptr     random_;


    std::string              work_path_;
    std::string              file_extraction_;
    std::string              file_tree_;
    std::vector<ROI>         rois_;
    std::vector<int>         classIDs_;

    void extract();
    void train();
    bool readTrainingData(cv::Mat &data, cv::Mat &classes, cv::Mat &var_type, std::vector<int> &classIDs);
    void writeMatrix(const cv::Mat &mat, YAML::Emitter &emitter);
    Keys prepareKeypoint(cv::Rect rect);
};

#endif // CMP_CORE_H
