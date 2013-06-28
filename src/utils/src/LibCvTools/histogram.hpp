#ifndef CV_HISTOGRAM_HPP
#define CV_HISTOGRAM_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @namespace cv_histogram is a namespace containing functions that make calculating histograms easier.
 * @author Hanten, Richard
 */
namespace cv_histogram {
static cv::Scalar COLOR_BLUE    = cv::Scalar(255, 0, 0);
static cv::Scalar COLOR_GREEN   = cv::Scalar(0, 255, 0);
static cv::Scalar COLOR_RED     = cv::Scalar(0, 0, 255);
static cv::Scalar COLOR_CYAN    = cv::Scalar(255, 0, 255);
static cv::Scalar COLOR_WHITE   = cv::Scalar(255,255,255);

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


/**
 * @brief Generates histogram images with certain parameters.
 * @param image         the image to generate a histogram of no special color room required
 * @param histograms    the histograms will be written to here
 * @param bins          the amount of bins per channel - if only one given, it will be used for all
 * @param ranges        range from 0 to given number for each channel - if only one given, it will be used for all
 * @param normalize     normalization values for each channels - if only one given, it  will be used for all
 * @param uniform       if the histogram should be uniform
 * @param accumulate    if the histogram should be calculated accumulative
 */
inline void generate_histogram(const cv::Mat &image, std::vector<cv::Mat> &histograms, const cv::Mat mask,
                               const std::vector<int> &bins, const std::vector<int> &ranges,
                               const std::vector<float> &normalize,
                               const bool uniform = false, bool accumulate = true)
{
    int norm_factor_count   = normalize.size();
    int plane_count         = image.channels();
    int bins_count          = bins.size();
    int range_count         = ranges.size();

    std::vector<cv::Mat> planes;
    cv::split(image, planes);
    histograms.resize(plane_count);

    std::vector< std::vector<float> > hist_ranges;
    hist_ranges.resize(range_count);
    for(int i = 0 ; i < range_count; i++) {
        hist_ranges[i].push_back(0.f);
        hist_ranges[i].push_back(ranges[i]);
    }
    for(unsigned int i = 0 ; i < plane_count ; i++) {
        int b = bins_count > 1 ? bins[i] : bins[0];
        const float* r = { range_count > 1 ? (float*) hist_ranges[i].data() : (float*) hist_ranges[0].data() };
        cv::calcHist(&planes[i], 1, 0, mask, histograms[i], 1, &b, &r, uniform, accumulate);
    }

    if(norm_factor_count > 0) {
        for(unsigned int i = 0 ; i < histograms.size(); i++) {
            float n = norm_factor_count > 1 ? normalize[i] : normalize[0];
            cv::normalize(histograms[i], histograms[i], 0, n, cv::NORM_MINMAX, -1, cv::Mat() );
        }
    }
}

inline void generate_histogram(const cv::Mat &image, std::vector<cv::Mat> &histograms,
                               const std::vector<int> &bins, const std::vector<int> &ranges,
                               const std::vector<float> &normalize,
                               const bool uniform = false, bool accumulate = true)
{
    generate_histogram(image, histograms, cv::Mat(), bins,  ranges, normalize, uniform, accumulate);
}

/**
 * @brief Genearate a histogram for a rgb image return bins in vectors.
 * @param image             the image to generate a histogram of
 * @param histograms        to save the bins to
 * @param bins              the amount of bins for each channel
 * @param normalize         the normalization factor for all channels
 * @param uniform           if the histogram should be uniform
 * @param accumulate        if the histogram should be calculated accumulative
 */
inline void generate_histogram_rgb(const cv::Mat &image, std::vector<cv::Mat> &histograms, const int bins = 256,
                                   const float normalize = 0.f, const bool uniform = true, bool accumulate = false)
{
    std::vector<int>    bins_per_channel;
    std::vector<float>  norm_per_channel;
    std::vector<int>    hist_range_per_channel;
    bins_per_channel.push_back(bins);
    if(normalize > 0.f)
        norm_per_channel.push_back(normalize);
    hist_range_per_channel.push_back(256);

    generate_histogram(image, histograms, bins_per_channel,hist_range_per_channel, norm_per_channel, uniform, accumulate);
}

/**
 * @brief Genearate a histogram for a rgb image return bins in vectors.
 * @param image             the image to generate a histogram of
 * @param histogram_bins    to save the bins to
 * @param bins              the amount of bins for each channel
 * @param normalize         the normalization factor for all channels
 * @param uniform           if the histogram should be uniform
 * @param accumulate        if the histogram should be calculated accumulative
 */
inline void generate_histogram_rgb(const cv::Mat &image, std::vector<std::vector<float> > &histogram_bins, const int bins = 256,
                                   const float normalize = 0.f, const bool uniform = true, bool accumulate = false)
{
    std::vector<cv::Mat> histograms;
    generate_histogram_rgb(image, histograms, bins, normalize, uniform, accumulate);
    for(int i = 0 ; i < histograms.size() ; i++) {
        histogram_bins.push_back(std::vector<float>());
        for(int j = 0 ; j < bins ; j++) {
            histogram_bins[i].push_back(histograms[i].at<float>(j));
        }
    }
}

/**
 * @brief Render histograms as image to watch.
 * @param histograms        the histograms to render;
 * @param bins              the amount of bins
 * @param histogram_color   the histogram specific color to be used
 * @param dst               the image to write to
 */
inline void render_histogram(const std::vector<cv::Mat> &histograms, const std::vector<int> bins, const std::vector<cv::Scalar> histogram_colors, cv::Mat &dst)
{
    assert(!dst.empty());
    assert(bins.size() == histograms.size());
    assert(bins.size() == histogram_colors.size());
    for( int i = 0 ; i < histograms.size() ; i++) {
        int bin_w = cvRound( dst.cols/ (double) bins[i] );
        for( int j = 1; j < bins[i]; j++ )
        {
            cv::line(dst,
                     cv::Point(bin_w*(j-1), dst.rows - cvRound(histograms[i].at<float>(j - 1))),
                     cv::Point(bin_w*j, dst.rows - cvRound(histograms[i].at<float>(j))),
                     histogram_colors[i]);
        }
    }

}

inline void equalize_hist(const cv::Mat &src, cv::Mat &dst)
{
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for(int i = 0 ; i < channels.size() ; i++) {
        cv::equalizeHist(channels[i], channels[i]);
    }
    cv::merge(channels, dst);
}

template<int norm>
inline void normalize(const cv::Mat &src, cv::Mat &dst, const std::vector<double> channel_factors)
{
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    for(int i = 0 ; i < channels.size() ; i++) {
        cv::normalize(channels[i], channels[i], channel_factors[i], 0, norm);
    }
    cv::merge(channels, dst);
}
}


#endif // CV_HISTOGRAM_HPP
