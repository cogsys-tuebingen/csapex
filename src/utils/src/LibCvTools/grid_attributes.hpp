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

    static AttrScalar generate(const cv::Mat &_img, const cv::Rect &_roi, const AttrScalar::Params &p)
    {
        cv::Mat img_roi(_img, _roi);
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

    static AttrHistogram generate(const cv::Mat &_img, const cv::Rect &_roi, const AttrHistogram::Params &p)
    {

        cv::Mat img_roi(_img, _roi);
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

class AttrTerrainClassCV {
public:
    struct Params {
        CVExtractor              *extractor;
        RandomForest             *classifier;
        int                       max_octave;
        bool                      color_extension;
        bool                      large_descriptor;
        bool                      use_max_prob;
        Extractor::KeypointParams key;
    };

    AttrTerrainClassCV()
    {
    }

    AttrTerrainClassCV(const int _classID, const float _probability) :
        classID(_classID),
        probability(_probability)
    {
    }

    AttrTerrainClassCV(const AttrTerrainClassCV &t) :
        classID(t.classID),
        probability(t.probability)
    {
    }

    bool operator == (const AttrTerrainClassCV &attr) const
    {
        return classID == attr.classID;
    }

    static AttrTerrainClassCV generate(const cv::Mat &_img, const cv::Rect &_roi,  const AttrTerrainClassCV::Params &p)
    {
        AttrTerrainClassCV::Params p_ = p;
        /// EXTRACT
        cv::Mat descriptors;
        p.extractor->extract(_img, _roi,
                             p.key, p.max_octave,
                             p.color_extension, p.large_descriptor,
                             descriptors);

        /// PREDICT
        if(descriptors.empty())
            return AttrTerrainClassCV(-1, -1.f);

        if(descriptors.type() != CV_32FC1) {
            descriptors.convertTo(descriptors, CV_32FC1);
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

        return AttrTerrainClassCV(classID, prob);
    }

    int     classID;
    float   probability;
};

class AttrTerrainClassPt {
public:
    struct Params {
        PatternExtractor   *extractor;
        RandomForest       *classifier;
    };

    AttrTerrainClassPt()
    {
    }

    AttrTerrainClassPt(const int _classID, const float _probability) :
        classID(_classID),
        probability(_probability)
    {
    }

    AttrTerrainClassPt(const AttrTerrainClassCV &t) :
        classID(t.classID),
        probability(t.probability)
    {
    }

    bool operator == (const AttrTerrainClassCV &attr) const
    {
        return classID == attr.classID;
    }

    static AttrTerrainClassPt generate(const cv::Mat &_img, const cv::Rect &_roi,  const AttrTerrainClassPt::Params &p)
    {
        cv::Mat descriptors;
        cv::Mat img_roi(_img, _roi);
        p.extractor->extract(img_roi, descriptors);

        /// PREDICT
        int   classID;
        float prob;
        p.classifier->predictClassProb(descriptors, classID, prob);

        return AttrTerrainClassPt(classID, prob);
    }

    int     classID;
    float   probability;
};
}
#endif // GRID_ATTRIBUTES_HPP
