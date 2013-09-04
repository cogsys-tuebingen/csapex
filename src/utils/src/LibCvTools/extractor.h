#ifndef EXTRACTOR_HPP
#define EXTRACTOR_HPP

#include <boost/shared_ptr.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "extractor_params.h"

namespace cv_extraction {
class Extractor {
public:
    typedef boost::shared_ptr<Extractor> Ptr;

    virtual void extract(const cv::Mat &image, cv::Mat &descriptors){}
    virtual void extract(const cv::Mat &image, const cv::Rect &roi, cv::Mat &descriptors){}

    ExtractorParams params();
    static cv::Vec2b extractMeanColorRGBYUV(const cv::Mat &img);
    static void      addColorExtension(cv::Mat &descriptor, const cv::Vec2b &color);

    static void read(const YAML::Node &document, Extractor::Ptr &extractor,
                     ExtractorParams::Ptr &params, KeypointParams &key);

    /**
     * color image + descriptors roi -> color extension
     */

protected:
    Extractor(){}
    boost::shared_ptr<ExtractorParams>  ext_params_;
};

}
#endif // EXTRACTOR_HPP
