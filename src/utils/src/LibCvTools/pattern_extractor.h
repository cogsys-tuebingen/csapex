#ifndef PATTERN_EXTRACTOR_H
#define PATTERN_EXTRACTOR_H

#include "extractor.hpp"
#include "local_patterns.hpp"

namespace cv_extraction {
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
}

#endif // PATTERN_EXTRACTOR_H
