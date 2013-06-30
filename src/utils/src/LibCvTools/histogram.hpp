#ifndef CV_HISTOGRAM_HPP
#define CV_HISTOGRAM_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @namespace cv_histogram is a namespace containing functions that make calculating histograms easier.
 * @author Hanten, Richard
 */
namespace cv_histogram {
const cv::Scalar COLOR_BLUE    = cv::Scalar(255, 0, 0);
const cv::Scalar COLOR_GREEN   = cv::Scalar(0, 255, 0);
const cv::Scalar COLOR_RED     = cv::Scalar(0, 0, 255);
const cv::Scalar COLOR_CYAN    = cv::Scalar(255, 255,0);
const cv::Scalar COLOR_WHITE   = cv::Scalar(255,255,255);

/**
 * @brief Normalize a rgb image.
 * @param src       the source image
 * @param dst       the destination image
 */
inline void normalize_rgb(const cv::Mat &src, cv::Mat &dst)
{
    cv::Vec3d mean;
    cv::Vec3d vari;

    dst = src.clone();

    cv::meanStdDev(src, mean, vari);
    cv::sqrt(vari, vari);

    uchar* dst_ptr = (uchar*) dst.data;

    for(int y = 0 ; y < src.rows ; y++) {
        for(int x = 0 ; x < src.cols ; x++) {
            for(int c = 0 ; c < src.channels() ; c++) {
                double val = dst_ptr[y *  src.step + x * src.channels() + c];
                val = (val - mean[c]) / vari[c] * 3 + 127;
                dst_ptr[y *  src.step + x * src.channels() + c] = val;
            }
        }
    }
}

inline void single_channel_histogram(const cv::Mat &src, std::vector<cv::MatND> &histograms, const cv::Mat &mask,
                                     const cv::Mat &bins, const cv::Mat &ranges,
                                     bool uniform = true, bool accumulate = false)
{
    assert(bins.rows == src.channels() || bins.cols == src.channels());
    assert(ranges.rows == src.channels() * 2 || ranges.cols == src.channels() * 2);

    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    int channel_count = channels.size();
    histograms.resize(channel_count);

    for(uint i = 0 ; i < channel_count ; i++) {
        int   ch[] = {0};
        int   b[] = {bins.at<int>(i)};
        float r_ch_values[] = {ranges.at<float>(2 * i), ranges.at<float>(i * 2 + 1)};
        const float* r_ch[] = {r_ch_values};
        cv::calcHist(&channels[i], 1, ch, mask, histograms[i], 1, b, r_ch, uniform , accumulate);
    }
}

inline void full_channel_equalize(const cv::Mat &src, cv::Mat &dst)
{
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for(int i = 0 ; i < channels.size() ; i++) {
        cv::equalizeHist(channels[i], channels[i]);
    }
    cv::merge(channels, dst);
}

template<int norm>
inline void normalize(const cv::Mat &src, cv::Mat &dst, const std::vector<double> channel_factors, bool round = true)
{
    assert(channel_factors.size() % 2 == 0);

    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for(int i = 0 ; i < channel_factors.size() / 2 ; i += 2) {
        cv::normalize(channels[i], channels[i], channel_factors[i], channel_factors[i+1], norm);
    }
    cv::merge(channels, dst);
}

/**
 * @brief Render histograms as image to watch.
 * @param histograms        the histograms to render;
 * @param bins              the amount of bins
 * @param histogram_color   the histogram specific color to be used
 * @param dst               the image to write to
 */
inline void render_histogram(const std::vector<cv::Mat> &histograms, const std::vector<int> bins, const std::vector<cv::Scalar> histogram_colors, cv::Mat &dst, const double scale = 1.0)
{
    assert(!dst.empty());
    assert(bins.size() == histograms.size());
    assert(bins.size() < histogram_colors.size());

    for( int i = 0 ; i < histograms.size() ; i++) {
        cv::Mat histogram = histograms[i].clone();
        cv::normalize(histogram, histogram, dst.rows * scale, cv::NORM_MINMAX);
        int bin_w = cvRound( dst.cols/ (double) bins[i] );
        for( int j = 1; j < bins[i]; j++ )
        {
            cv::line(dst,
                     cv::Point(bin_w*(j-1), dst.rows - cvRound(histogram.at<float>(j - 1))),
                     cv::Point(bin_w*j, dst.rows - cvRound(histogram.at<float>(j))),
                     histogram_colors[i]);
        }
    }
}
}


#endif // CV_HISTOGRAM_HPP
