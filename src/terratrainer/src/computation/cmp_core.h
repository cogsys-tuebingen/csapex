#ifndef CMP_CORE_H
#define CMP_CORE_H
/// COMPONENT
#include <computation/params.hpp>
/// SYSTEM
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <boost/shared_ptr.hpp>

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

    void setParams(CMPParams &params);

    void createORB  (CMPParamsORB &params);
    void createSIFT (CMPParamsSIFT &params);
    void createSURF (CMPParamsSURF &params);
    void createBRISK(CMPParamsBRISK &params);
    void createBRIEF(CMPParamsBRIEF &params);
    void createFREAK(CMPParamsFREAK &params);

private:
    cv::Mat      raw_image_;
    cv::DescriptorExtractor* extractor_;

    void makeOPP();
};

#endif // CMP_CORE_H
