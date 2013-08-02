#ifndef CMP_EXTRACTOR_H
#define CMP_EXTRACTOR_H
#include <boost/shared_ptr.hpp>
#include <opencv2/nonfree/features2d.hpp>

class CMPExtractor
{
public:
    typedef boost::shared_ptr<CMPExtractor> Ptr;
    typedef boost::shared_ptr<cv::DescriptorExtractor> CvExPtr;

    CMPExtractor();

    void set(cv::DescriptorExtractor* extractor);
    void extract(const cv::Mat &image, cv::Mat &descriptors);
    void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors);

private:
    CvExPtr extractor_;
};

#endif // CMP_EXTRACTOR_H
