#ifndef CMP_EXTRACTOR_H
#define CMP_EXTRACTOR_H
#include <boost/shared_ptr.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "local_patterns.hpp"

class Extractor {
public:
    struct KeypointParams {
        KeypointParams() : angle(0.f), scale(0.5f), octave(-1), soft_crop(true), calc_angle(false){}

        float angle;
        float scale;
        int   octave;
        bool  soft_crop;
        bool  calc_angle;
    };

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


class CVExtractor : public Extractor
{
public:
    typedef boost::shared_ptr<CVExtractor>              Ptr;
    typedef boost::shared_ptr<cv::DescriptorExtractor>  CvExPtr;
    typedef std::vector<cv::KeyPoint>                   KeyPoints;

    CVExtractor();

    void set(cv::DescriptorExtractor* extractor);
    void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &key_points, cv::Mat &descriptors);
    void extract(const Mat &image, const cv::Rect roi, const KeypointParams &params,
                 const int max_octave, const bool color_extension, const bool large,
                 cv::Mat &descriptors);

    static KeyPoints prepareKeypoint(const cv::Rect &rect, const KeypointParams &params);
    static KeyPoints prepareOctaveKeypoints(const cv::Rect &rect, const KeypointParams &params, const int max_octave);
    static double    calcAngle(const cv::Mat &image);


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
    void extract(const Mat &image,
                 const bool color_extension, const bool large,
                 cv::Mat &descriptors);

protected:
    enum Type{NOT_SET, LBP, LTP};
    Type type;
    cv_local_patterns::LocalPattern *pattern;

    double k;

    void extractLBP(const cv::Mat &image, cv::Mat &descriptors);
    void extractLTP(const cv::Mat &image, cv::Mat &descriptors);

};

#endif // CMP_EXTRACTOR_H
