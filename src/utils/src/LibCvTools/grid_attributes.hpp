#ifndef GRID_ATTRIBUTES_HPP
#define GRID_ATTRIBUTES_HPP
#include <opencv2/opencv.hpp>
#include "histogram.hpp"
/**
 * @class This can be used as an attribute to attach to a grid cell.
 */
class AttributeColorHLS {
public:
    /**
     * @brief AttributeHLS default constructor.
     */
    AttributeColorHLS()
    {
    }

    /**
     * @brief AttributeHLS constructor.
     * @param _hls      the color value assigned to the cell
     * @param _eps      the maximum error to this value for comparision with another attribute
     */
    AttributeColorHLS(const cv::Scalar &_hls, const cv::Scalar &_eps) :
        hls(_hls),
        eps(_eps)
    {
    }

    /**
     * @brief AttributeHLS copy constructor.
     * @param h         the attribute to copy
     */
    AttributeColorHLS(const AttributeColorHLS &h) :
        hls(h.hls),
        eps(h.eps)
    {
    }

    /**
     * @brief Compare two attributes considering an error.
     * @param attr      the other attribute
     * @return          if the attributes are equal
     */
    bool operator == (const AttributeColorHLS &attr) const
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
    static AttributeColorHLS generate(const cv::Mat &img, const cv::Scalar &eps)
    {
        cv::Scalar value = cv::mean(img);
        return AttributeColorHLS(value, eps);
    }

    cv::Scalar hls;
    cv::Scalar eps;
};

class AttributeHistogram {
    enum ColorRoom{HSV, HLS, HSVFULL, HLSFULL, RGB8NORM, BGR8NORM};
public:
    AttributeHistogram()
    {
    }

    AttributeHistogram(const std::vector<cv::Mat> &_histograms, const double _eps) :
        histograms(histograms),
        eps(_eps)
    {
    }

    AttributeHistogram(const AttributeHistogram &h) :
        histograms(h.histograms),
        eps(h.eps)
    {
    }

    bool operator == (const AttributeHistogram &attr) const
    {
        bool res = true;


        return res;
    }

    static AttributeHistogram generate(const cv::Mat &_img, const std::vector<int> &bins, const double _eps, const ColorRoom c,
                                       const std::vector<float> &normalize = std::vector<float>())
    {
        /// pushback 256 , 256 , 256 for bgr / rgb
        /// puschback 180 , 256, 256 for hsv / hls

        cv::Mat working_copy = _img.clone();
        std::vector<int> ranges;
        switch(c) {
        case RGB8NORM:
            ranges.push_back(255);
            cv_histogram::normalize_rgb(working_copy, working_copy);
            break;
        case BGR8NORM:
            ranges.push_back(255);
            cv_histogram::normalize_rgb(working_copy, working_copy);
            break;
        case HSV:
            ranges.push_back(180);
            ranges.push_back(255);
            ranges.push_back(255);
            break;
        case HLS:
            ranges.push_back(180);
            ranges.push_back(255);
            ranges.push_back(255);
            break;
        default:
            ranges.push_back(255);
        }

        std::vector<cv::Mat>   histograms;
        cv_histogram::generate_histogram(working_copy, histograms, bins, ranges, normalize);

        return AttributeHistogram(histograms, _eps);
    }

    std::vector<cv::Mat> histograms;
    double               eps;

private:

};

#endif // GRID_ATTRIBUTES_HPP
