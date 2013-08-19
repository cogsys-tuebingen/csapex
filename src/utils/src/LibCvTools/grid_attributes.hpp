#ifndef GRID_ATTRIBUTES_HPP
#define GRID_ATTRIBUTES_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "histogram.hpp"
#include "extractor.h"
#include "randomforest.h"
/**
 * @class This can be used as an attribute to attach to a grid cell.
 */
namespace cv_grid {
class AttrScalar {
public:
    struct Params {
        cv::Scalar       eps;
        cv::Vec<bool, 4> ignore;
    };

    AttrScalar()
    {
    }

    AttrScalar(const cv::Scalar &_scalar, const cv::Scalar &_eps, const cv::Vec<bool, 4> _ignore) :
        scalar(_scalar),
        eps(_eps),
        ignore(_ignore)
    {
    }

    AttrScalar(const AttrScalar &h) :
        scalar(h.scalar),
        eps(h.eps),
        ignore(h.ignore)
    {
    }

    bool operator == (const AttrScalar &attr) const
    {
        bool res = true;
        res &=  ignore[0] || std::abs(scalar[0] - attr.scalar[0]) <= eps[0];
        res &=  ignore[1] || std::abs(scalar[1] - attr.scalar[1]) <= eps[1];
        res &=  ignore[2] || std::abs(scalar[2] - attr.scalar[2]) <= eps[2];
        res &=  ignore[3] || std::abs(scalar[3] - attr.scalar[3]) <= eps[3];
        return res;
    }

    static AttrScalar generate(const cv::Mat &img, const AttrScalar::Params &p)
    {
        cv::Scalar value = cv::mean(img);
        return AttrScalar(value, p.eps, p.ignore);
    }

    cv::Scalar      scalar;
    cv::Scalar      eps;
    cv::Vec<bool,4> ignore;
    int pixels;
};

class AttrHistogram {
public:
    enum ExtendedMethods {CV_COMP_SQRD = -1};

    struct Params {
        cv::Mat     bins;
        cv::Mat     ranges;
        cv::Scalar  eps;
        int         method;
    };

    AttrHistogram()
    {
    }

    AttrHistogram(const std::vector<cv::MatND> &_histograms, const cv::Scalar & _eps, const int _method, const double _pixels) :
        histograms(_histograms),
        eps(_eps),
        method(_method),
        pixels(_pixels)
    {
    }

    AttrHistogram(const AttrHistogram &h) :
        histograms(h.histograms),
        eps(h.eps),
        method(h.method),
        pixels(h.pixels)
    {
    }

    bool operator == (const AttrHistogram &attr) const
    {
        assert(histograms.size() == attr.histograms.size());
        bool res = true;

        for(int i = 0 ; i < histograms.size() ; i++) {
            res &= compareHist_(histograms[i], attr.histograms[i], method) <= eps[i];
        }

        return res;
    }

    static AttrHistogram generate(const cv::Mat &_img, const AttrHistogram::Params &p)
    {

        std::vector<cv::MatND>  histograms;
        cv_histogram::full_channel_histogram(_img, histograms, cv::Mat(), p.bins, p.ranges);

        return AttrHistogram(histograms, p.eps, p.method, _img.cols * _img.rows);
    }

    std::vector<cv::MatND>  histograms;
    cv::Scalar              eps;
    int                     method;
    double                  pixels;

private:
    double compareHist_(const cv::MatND &h1, const cv::MatND &h2, const int method) const
    {
        if(method >= 0)
            return cv::compareHist(h1, h2, method);

        if(method == CV_COMP_SQRD) {
            cv::Mat result;
            cv::subtract(h1, h2, result);
            cv::multiply(result, result, result);
            return cv::sum(result)[0] / pixels;
        }

        return INFINITY;
    }
};

class AttrTerrainClass {
public:
    struct Params {
        CVExtractor   *extractor;
        RandomForest  *classifier;
        float         angle;
        float         scale;
    };

    AttrTerrainClass()
    {
    }

    AttrTerrainClass(const int _classID, const float _probability) :
        classID(_classID),
        probability(_probability)
    {
    }

    AttrTerrainClass(const AttrTerrainClass &t) :
        classID(t.classID),
        probability(t.probability)
    {
    }

    bool operator == (const AttrTerrainClass &attr) const
    {
        return classID == attr.classID;
    }

    static AttrTerrainClass generate(const cv::Mat &_img, const AttrTerrainClass::Params &p)
    {
        /// EXTRACT
        cv::Mat descriptors;
        cv::Rect roi(0,0, _img.cols, _img.rows);
        CVExtractor::KeyPoints k = p.extractor->prepareKeypoint(roi, p.scale);
        p.extractor->extract(_img, k, descriptors);
        /// PREDICT
        int   classID;
        float prob;
        p.classifier->predictClassProb(descriptors, classID, prob);

        return AttrTerrainClass(classID, prob);
    }

    int     classID;
    float   probability;
};
}
#endif // GRID_ATTRIBUTES_HPP
