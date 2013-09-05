#ifndef GRID_ATTRIBUTES_HPP
#define GRID_ATTRIBUTES_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "histogram.hpp"
#include "feature_extractor.h"
#include "pattern_extractor.h"
#include "extractor_params.h"
#include "randomforest.h"
/**
 * @class This can be used as an attribute to attach to a grid cell.
 */
namespace cv_grid {
class AttrScalar {
public:
    struct Params {
        cv::Scalar       eps;
        cv::Mat          image;
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

    static AttrScalar generate(const cv::Rect &_roi, const AttrScalar::Params &p)
    {
        cv::Mat img_roi(p.image, _roi);
        cv::Scalar value = cv::mean(img_roi);
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
        cv::Mat     image;
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

        for(int i = 0 ; i < histograms.size() ; ++i) {
            res &= compareHist_(histograms[i], attr.histograms[i], method) <= eps[i];
        }

        return res;
    }

    static AttrHistogram generate(const cv::Rect &_roi, const AttrHistogram::Params &p)
    {

        cv::Mat img_roi(p.image, _roi);
        std::vector<cv::MatND>  histograms;
        cv_histogram::full_channel_histogram(img_roi, histograms, cv::Mat(), p.bins, p.ranges);

        return AttrHistogram(histograms, p.eps, p.method, img_roi.cols * img_roi.rows);
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

class AttrTerrain {
public:
    struct Params {
        cv_extraction::Extractor::Ptr    extractor;
        cv_extraction::KeypointParams    key;
        cv::Mat                          image;
        cv::Mat                          image_gray;
        RandomForest::Ptr                classifier;
        bool                             use_max_prob;
        bool                             color_ext;
    };

    AttrTerrain()
    {
    }

    AttrTerrain(const int _classID, const float _probability) :
        classID(_classID),
        probability(_probability)
    {
    }

    AttrTerrain(const AttrTerrain &t) :
        classID(t.classID),
        probability(t.probability)
    {
    }

    bool operator == (const AttrTerrain &attr) const
    {
        return classID == attr.classID;
    }

    static AttrTerrain generate(const cv::Rect &_roi,  const AttrTerrain::Params &p)
    {
        AttrTerrain::Params p_ = p;
        /// EXTRACT
        cv::Mat descriptors;
        p.extractor->extract(p.image_gray, _roi, descriptors);

        /// PREDICT
        if(descriptors.empty())
            return AttrTerrain(-1, -1.f);

        if(descriptors.type() != CV_32FC1) {
            descriptors.convertTo(descriptors, CV_32FC1);
        }

        if(p.color_ext) {
            cv::Mat     img_roi(p.image, _roi);
            cv::Vec2b   mean = cv_extraction::Extractor::extractMeanColorRGBYUV(img_roi);
            cv_extraction::Extractor::addColorExtension(descriptors, mean);
        }

        int   classID = -1;
        float prob    = 0.f;
        if(descriptors.rows > 1) {
            if(p_.use_max_prob)
                p_.classifier->predictClassProbMultiSampleMax(descriptors, classID, prob);
            else
                p_.classifier->predictClassProbMultiSample(descriptors, classID, prob);
        } else {
            p_.classifier->predictClassProb(descriptors, classID, prob);
        }

        return AttrTerrain(classID, prob);
    }

    int     classID;
    float   probability;
};
}
#endif // GRID_ATTRIBUTES_HPP
