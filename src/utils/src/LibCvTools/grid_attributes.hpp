#ifndef GRID_ATTRIBUTES_HPP
#define GRID_ATTRIBUTES_HPP
#include <opencv2/opencv.hpp>
#include "histogram.hpp"
/**
 * @class This can be used as an attribute to attach to a grid cell.
 */
namespace cv_grid {
class AttrHLS {
public:
    struct Params {
        cv::Scalar eps;
    };

    /**
     * @brief AttributeHLS default constructor.
     */
    AttrHLS()
    {
    }

    /**
     * @brief AttributeHLS constructor.
     * @param _hls      the color value assigned to the cell
     * @param _eps      the maximum error to this value for comparision with another attribute
     */
    AttrHLS(const cv::Scalar &_hls, const cv::Scalar &_eps) :
        hls(_hls),
        eps(_eps)
    {
    }

    /**
     * @brief AttributeHLS copy constructor.
     * @param h         the attribute to copy
     */
    AttrHLS(const AttrHLS &h) :
        hls(h.hls),
        eps(h.eps)
    {
    }

    /**
     * @brief Compare two attributes considering an error.
     * @param attr      the other attribute
     * @return          if the attributes are equal
     */
    bool operator == (const AttrHLS &attr) const
    {
        bool res = true;
        res &= std::abs(hls[0] - attr.hls[0]) <= eps[0];
        res &= std::abs(hls[2] - attr.hls[2]) <= eps[2];
        return res;
    }

    /**
     * @brief Produce an attribute, processing the image part in the most common way.
     * @param img           the source image
     * @param eps           the mean error
     * @return              an attribute
     */
    static AttrHLS generate(const cv::Mat &img, const AttrHLS::Params &p)
    {
        cv::Scalar value = cv::mean(img);
        return AttrHLS(value, p.eps);
    }

    cv::Scalar hls;
    cv::Scalar eps;
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
        cv::Mat bins;
        cv::Mat ranges;
        cv::Scalar eps;
        int method;
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
            res &= cv::compareHist(histograms[i], attr.histograms[i], method) <= eps[i];
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
