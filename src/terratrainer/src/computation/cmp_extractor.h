#ifndef CMP_EXTRACTOR_H
#define CMP_EXTRACTOR_H
#include <boost/shared_ptr.hpp>
#include <opencv2/nonfree/features2d.hpp>

class CMPExtractor
{
public:
    typedef boost::shared_ptr<CMPExtractor>             Ptr;
    typedef boost::shared_ptr<cv::DescriptorExtractor>  CvExPtr;
    typedef std::vector<cv::KeyPoint>                   KeyPoints;

    struct ROI {
        cv::Rect    bounding;
        int         id;
        int         classID;
        double      rotation;
    };

    CMPExtractor();

    void set(cv::DescriptorExtractor* extractor);
    void extract(const cv::Mat &image, cv::Mat &descriptors);
    void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors);

protected:
    CvExPtr          extractor_;

    KeyPoints prepareKeypoint(cv::Rect rect);

};

#endif // CMP_EXTRACTOR_H
