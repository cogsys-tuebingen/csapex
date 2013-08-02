#ifndef CMP_CORE_H
#define CMP_CORE_H
/// COMPONENT
#include "params.hpp"
#include "cmp_extractor.h"
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
        int         classId;
        double      rotation;
    };

    typedef boost::shared_ptr<CMPCore> Ptr;

    CMPCore();

    /// IMAGE
    bool    loadImage(const std::string image_path);
    cv::Mat getImage() const;

    /// EXTRACTION
    void    setRois(const std::vector<ROI> &rois);
    void    compute();
    void    setWorkPath(const std::string &work_path);

    template<class Parameters>
    void create(Parameters &param)
    {
        cv::DescriptorExtractor* ptr = CMPExtractors::create(param);
        if(param.opp)
            CMPExtractors::makeOpp(ptr);
        extractor_->set(ptr);
        type_ = param.type;
    }

private:
    typedef std::vector<cv::KeyPoint> Keys;

    cv::Mat                  raw_image_;
    CMPExtractor::Ptr        extractor_;
    CMPParams::Type          type_;

    std::string              work_path_;
    std::vector<ROI>         rois_;

    void makeOPP();
    void extract();
    void writeMatrix(const cv::Mat &mat, YAML::Emitter &emitter);
    Keys prepareKeypoint(cv::Rect rect);
};

#endif // CMP_CORE_H
