#ifndef PATTERN_EXTRACTOR_H
#define PATTERN_EXTRACTOR_H

#include "extractor.hpp"
#include "local_patterns.hpp"
namespace cv_extraction {
class ExtractorParams;
class ParamsLTP;
class ParamsLBP;

class PatternExtractor : public Extractor
{
public:
    typedef boost::shared_ptr<PatternExtractor> Ptr;

    PatternExtractor();
    void set (cv_local_patterns::LBP *bp);
    void set (cv_local_patterns::LTP *tp);
    void extract(const cv::Mat &image, cv::Mat &descriptors);

    void setK(const double value);
    void setParams(const cv_extraction::ParamsLBP &params);
    void setParams(const cv_extraction::ParamsLTP &params);

    static cv_local_patterns::LBP *getExtractor(const cv_extraction::ParamsLBP &params);
    static cv_local_patterns::LTP *getExtractor(const cv_extraction::ParamsLTP &params);


protected:
    enum Type{NOT_SET, LBP, LTP};
    Type type_;
    cv_local_patterns::LocalPattern::Ptr pattern_;
    boost::shared_ptr<ExtractorParams>   ext_params_;

    double k;

    void extractLBP(const cv::Mat &image, cv::Mat &descriptors);
    void extractLTP(const cv::Mat &image, cv::Mat &descriptors);

};
}

#endif // PATTERN_EXTRACTOR_H
