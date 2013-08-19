#ifndef CMP_EXTRACTOR_H
#define CMP_EXTRACTOR_H
#include <boost/shared_ptr.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "local_patterns.hpp"

class Extractor {
public:

    virtual void extract(const cv::Mat &image, cv::Mat &descriptors){}
    virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors){}

    static cv::Scalar extractMeanColorRGBHSV(const cv::Mat &img)
    {
        cv::Mat convert;
        cv::cvtColor(img, convert, CV_BGR2HSV);

        return cv::mean(convert);
    }

protected:
    Extractor(){}
};


class CVExtractor : public Extractor
{
public:
    typedef boost::shared_ptr<CVExtractor>                Ptr;
    typedef boost::shared_ptr<cv::DescriptorExtractor>  CvExPtr;
    typedef std::vector<cv::KeyPoint>                   KeyPoints;

    CVExtractor();

    void set(cv::DescriptorExtractor* extractor);
    void extract(const cv::Mat &image, cv::Mat &descriptors);
    void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors);

    KeyPoints prepareKeypoint(const cv::Rect &rect, const bool soft_crop, const float scale = 1.f, const float angle = -1.f);
    KeyPoints prepareNeighbouredKeypoint(const cv::Rect &rect, const bool soft_crop, const float scale = 1.f, const float angle = -1.f);
    KeyPoints prepareOctavedKeypoint(const cv::Rect &rect, const float scale = 1.f, const float angle = -1.f);

protected:
    CvExPtr          extractor_;

};

class PatternExtractor : public Extractor
{
public:
    typedef boost::shared_ptr<PatternExtractor> Ptr;

    PatternExtractor();
    void set (cv_local_patterns::LBP *bp);
    void set (cv_local_patterns::LTP *tp);
    void setK(const double value);

    void extract(const cv::Mat &image, cv::Mat &descriptors);
protected:
    enum Type{NOT_SET, LBP, LTP};
    Type type;
    cv_local_patterns::LocalPattern *pattern;

    double k;

    void extractLBP(const cv::Mat &image, cv::Mat &descriptors);
    void extractLTP(const cv::Mat &image, cv::Mat &descriptors);

};

#endif // CMP_EXTRACTOR_H
