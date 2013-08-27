#ifndef EXTRACTOR_HPP
#define EXTRACTOR_HPP

#include <boost/shared_ptr.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cv_extraction {
class Extractor {
public:
    typedef boost::shared_ptr<Extractor> Ptr;

    virtual void extract(const cv::Mat &image, cv::Mat &descriptors){}
    virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors){}

    static cv::Scalar extractMeanColorRGBHSV(const cv::Mat &img)
    {
        cv::Mat convert;
        cv::cvtColor(img, convert, CV_BGR2HSV);

        return cv::mean(convert);
    }

    static void addColorExtension(cv::Mat &descriptor, const cv::Scalar &color)
    {

        cv::Mat tmp(descriptor.rows, descriptor.cols + 3, descriptor.type(),cv::Scalar::all(0));
        cv::Mat tmp_roi(tmp, cv::Rect(0,0, descriptor.cols, descriptor.rows));
        tmp.col(tmp.cols - 3).setTo(color[0]);
        tmp.col(tmp.cols - 2).setTo(color[1]);
        tmp.col(tmp.cols - 1).setTo(color[2]);
        descriptor.copyTo(tmp_roi);
        tmp.copyTo(descriptor);
    }

protected:
    Extractor(){}
};

}
#endif // EXTRACTOR_HPP
