#ifndef CMP_EXTRACTOR_H
#define CMP_EXTRACTOR_H
#include <boost/shared_ptr.hpp>

namespace cv {
class DescriptorExtractor;
}

class CMPExtractor
{
public:
    typedef boost::shared_ptr<CMPExtractor> Ptr;
    typedef boost::shared_ptr<cv::DescriptorExtractor> CvExPtr;

    CMPExtractor();

    void set(cv::DescriptorExtractor* extractor);

private:
    CvExPtr extractor_;
};

#endif // CMP_EXTRACTOR_H
