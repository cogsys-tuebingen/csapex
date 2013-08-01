#ifndef CMP_CORE_H
#define CMP_CORE_H
/// COMPONENT
#include "params.hpp"
#include "cmp_extractor.h"
#include "extractors.hpp"
/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
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

    bool    load(const std::string image_path);
    cv::Mat getImage() const;


    void addROI(const ROI  &roi);



    template<class Parameters>
    void create(Parameters &param)
    {
        cv::DescriptorExtractor* ptr = CMPExtractors::create(param);
        if(param.opp)
            CMPExtractors::makeOpp(ptr);
        extractor_->set(ptr);
    }

private:
    cv::Mat                  raw_image_;
    CMPExtractor::Ptr        extractor_;

    void makeOPP();
};

#endif // CMP_CORE_H
