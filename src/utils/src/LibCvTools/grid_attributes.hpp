#ifndef GRID_ATTRIBUTES_HPP
#define GRID_ATTRIBUTES_HPP
#include <opencv2/opencv.hpp>
#include "histogram.hpp"
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
};

class AttrHistogram {
    /*
        CV_COMP_CORREL Correlation
        CV_COMP_CHISQR Chi-Square
        CV_COMP_INTERSECT Intersection
        CV_COMP_BHATTACHARYYA Bhattacharyya distance
        CV_COMP_HELLINGER Synonym for CV_COMP_BHATTACHARYYA
    */
public:
    struct Params {
        cv::Mat     bins;
        cv::Mat     ranges;
        cv::Scalar  eps;
        int         method;
    };

    AttrHistogram()
    {
    }

    AttrHistogram(const std::vector<cv::MatND> &_histograms, const cv::Scalar & _eps, const int _method) :
        histograms(_histograms),
        eps(_eps),
        method(_method)
    {
    }

    AttrHistogram(const AttrHistogram &h) :
        histograms(h.histograms),
        eps(h.eps),
        method(h.method)
    {
    }

    bool operator == (const AttrHistogram &attr) const
    {
        assert(histograms.size() == attr.histograms.size());
        bool res = true;

        for(int i = 0 ; i < histograms.size() ; i++) {
            res &= cv::compareHist(histograms[i], attr.histograms[i], method) <= eps[i] || i == 0;
            std::cout << "%%%" << cv::compareHist(histograms[i], attr.histograms[i], method) << std::endl;
        }


        return res;
    }

    //    prepare(bins, ranges);
    //        const cv::Mat &_bins, const cv::Mat &_ranges,
    //        const double _eps, const int _method = CV_COMP_CHISQR
    static AttrHistogram generate(const cv::Mat &_img, const AttrHistogram::Params &p)
    {

        std::vector<cv::MatND>  histograms;
        cv_histogram::full_channel_histogram(_img, histograms, cv::Mat(), p.bins, p.ranges);

        return AttrHistogram(histograms, p.eps, p.method);
    }

    std::vector<cv::MatND>  histograms;
    cv::Scalar              eps;
    int                     method;

private:

};
}
#endif // GRID_ATTRIBUTES_HPP
