/// HEADER
#include "adaptive_background_remover.h"

/// SYSTEM
#include <utils/LibUtil/Stopwatch.h>

using namespace background_subtraction;

REGISTER_REMOVER(AdaptiveBackgroundRemover);

AdaptiveBackgroundRemover::AdaptiveBackgroundRemover()
    : BackgroundRemover("adaptive"), n(0), scale(2), takes_max(5), takes(takes_max)
{
}

namespace
{
inline double update_pixel(double decay, int ws, int i, int j, int n, double* mean_data, double* M2_data, const cv::Mat& frame, int channels, int frame_channels, int channel)
{
    double& mean = mean_data[i * ws + j * channels + channel];
    double& M2 = M2_data[i * ws + j * channels + channel];
    uchar& x = frame.data[i * frame.step + j * frame_channels + channel];

    double delta = x - mean;
    mean += delta / n;
    M2 *= decay;
    M2 += delta * (x - mean);

    return M2;
}

inline void reset_pixel(int ws, int i, int j, int n, double* mean_data, double* M2_data, const cv::Mat& frame, int channels, int frame_channels, int channel)
{
    double& M2 = M2_data[i * ws + j * channels + channel];
    M2 *= 0.1;
}
}

void AdaptiveBackgroundRemover::applyConfig(GlobalConfig &config)
{
    setMaxDistance(config.max_dist);
    setMaxStdDev(config.max_std_dev);
    setDecay(config.decay);
}

void AdaptiveBackgroundRemover::add(const cv::Mat& frame)
{
    n++;

    int ch = mean.channels();
    int f_ch = frame.channels();

    double* mean_data = (double*) mean.data;
    double* M2_data = (double*) M2.data;
    int ws = mean.step / sizeof(double);

    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            double v = 0;
            v += update_pixel(decay, ws, i, j, n, mean_data, M2_data, frame, ch, f_ch, 0);
            v += update_pixel(decay, ws, i, j, n, mean_data, M2_data, frame, ch, f_ch, 1);
            v += update_pixel(decay, ws, i, j, n, mean_data, M2_data, frame, ch, f_ch, 2);

            double var = v / (n-1);

            if(var < std::pow(4.0, 2)) {
                reset_pixel(ws, i, j, n, mean_data, M2_data, frame, ch, f_ch, 0);
                reset_pixel(ws, i, j, n, mean_data, M2_data, frame, ch, f_ch, 1);
                reset_pixel(ws, i, j, n, mean_data, M2_data, frame, ch, f_ch, 2);
            }
        }
    }
}

void AdaptiveBackgroundRemover::fp2char(const cv::Mat& fp_mat, cv::Mat& frame, double f)
{
    int ch = fp_mat.channels();
    int f_ch = frame.channels();

    double* sum_data = (double*) fp_mat.data;
    int ws = fp_mat.step / sizeof(double);

    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            frame.data[i * frame.step + j * f_ch] = f * sum_data[i * ws + j * ch];
            frame.data[i * frame.step + j * f_ch+1] = f * sum_data[i * ws + j * ch+1];
            frame.data[i * frame.step + j * f_ch+2] = f * sum_data[i * ws + j * ch+2];
        }
    }
}

void AdaptiveBackgroundRemover::sum2Img(cv::Mat frame)
{
    double f = 1.0;
    cv::Mat& fp_mat = mean;

    fp2char(fp_mat, frame, f);
}

void AdaptiveBackgroundRemover::setBackground(const cv::Mat& frame_big)
{
    cv::Mat frame;
    cv::Size size(frame_big.cols / scale, frame_big.rows / scale);
    cv::resize(frame_big, frame, size);

    takes = takes_max;
    mean = cv::Mat(frame.rows, frame.cols, CV_64FC3, cv::Scalar::all(0));
    M2 = cv::Mat(frame.rows, frame.cols, CV_64FC3, cv::Scalar::all(0));
    n = 0;

    add(frame);

    has_background = true;
}

void AdaptiveBackgroundRemover::replaceBackground(const cv::Mat& frame)
{
    BackgroundRemover::setBackground(frame);
}

void AdaptiveBackgroundRemover::calcDifference(const cv::Mat& frame, const cv::Mat& reference, cv::Mat& result)
{
    uchar* diff_data = result.data;
    uchar* frame_data = frame.data;
    uchar* ref_data = reference.data;

    unsigned channels = frame.channels();

    assert(frame.channels() == reference.channels());

    // segmentation
    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            int jc = j * channels;
            unsigned char f_r = frame_data[i * frame.step + jc + 2];
            unsigned char f_g = frame_data[i * frame.step + jc + 1];
            unsigned char f_b = frame_data[i * frame.step + jc];

            unsigned char ref_r = ref_data[i * reference.step + jc + 2];
            unsigned char ref_g = ref_data[i * reference.step + jc + 1];
            unsigned char ref_b = ref_data[i * reference.step + jc];

            int dr = f_r - ref_r;
            int dg = f_g - ref_g;
            int db = f_b - ref_b;

            double dist_sqr = (dr*dr + dg*dg + db*db);

            double v = std::min(255.0,  sqrt(dist_sqr));
            diff_data[i * result.step + j] = v;
        }
    }
}

void AdaptiveBackgroundRemover::updateBackground(const cv::Mat& frame, const cv::Mat& blured)
{
    unsigned channels = frame.channels();
    double max_dist_to_adapt_sqr = max_dist * max_dist;
    double max_variance_to_adapt = std::pow(max_std_dev, 2.0);

    // build difference texture
    uchar* frame_data = frame.data;
    uchar* bg_data = background.data;

    double* mean_data = (double*) mean.data;
    double* M2_data = (double*) M2.data;
    int mean_ws = mean.step / sizeof(double);

    int changed = 0;

    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            int jc = j * channels;

            const unsigned char& f_r = frame_data[i * frame.step + jc + 2];
            const unsigned char& f_g = frame_data[i * frame.step + jc + 1];
            const unsigned char& f_b = frame_data[i * frame.step + jc];

            const double& mean_r = mean_data[i * mean_ws + jc+2];
            const double& mean_g = mean_data[i * mean_ws + jc+1];
            const double& mean_b = mean_data[i * mean_ws + jc];

            const double var_r = M2_data[i * mean_ws + jc+2] / (n-1);
            const double var_g = M2_data[i * mean_ws + jc+1] / (n-1);
            const double var_b = M2_data[i * mean_ws + jc] / (n-1);

            double var_max = std::max(var_r, std::max(var_g, var_b));

            if(var_max < max_variance_to_adapt) {
                unsigned char& bg_r = bg_data[i * background.step + jc + 2];
                unsigned char& bg_g = bg_data[i * background.step + jc + 1];
                unsigned char& bg_b = bg_data[i * background.step + jc];

                int dr = mean_r - bg_r;
                int dg = mean_g - bg_g;
                int db = mean_b - bg_b;

                double dist_sqr = dr * dr + dg * dg + db * db;
                if(dist_sqr < max_dist_to_adapt_sqr || var_max <= 0.1) {

                    changed++;
                    bg_r += (f_r - bg_r) / 10.0;
                    bg_g += (f_g - bg_g) / 10.0;
                    bg_b += (f_b - bg_b) / 10.0;
                }
            }
        }
    }

    if(changed > 0) {
        replaceBackground(background);
    }

    std::cout << "changed " << changed << " pixels" << std::endl;
}

void AdaptiveBackgroundRemover::segmentation(const cv::Mat& frame_big, cv::Mat& mask_big)
{
    cv::Mat frame, blured, mask;

    cv::Size size(frame_big.cols / scale, frame_big.rows / scale);
    cv::resize(frame_big, frame, size);
    cv::GaussianBlur(frame, blured, blur_kernel, sigma_blur);

    add(blured);

    if(takes > 0) {
        takes--;

        cv::Mat normal(frame.rows, frame.cols, frame.type());
        sum2Img(normal);
        replaceBackground(normal);
    }

    if(takes == 0) {
        Stopwatch w;
        updateBackground(frame, blured);
        std::cout << "updating background: " << w.msElapsed() << "ms" << std::endl;
    }

    cv::Mat diff(blured.rows, blured.cols, CV_8U);;
    calcDifference(blured, background_blured, diff);

    debug = cv::Mat (blured.rows, blured.cols, CV_8UC3);
    fp2char(M2, debug, 1.0 / (n-1));
//    fp2char(mean, debug, 1.0);
//    diff.copyTo(debug);

    cv::threshold(diff, mask, difference_threshold / 1.0, 255, CV_THRESH_BINARY);

    cv::resize(mask, mask_big, frame_big.size());

//    cv::Mat bg;
//    cv::resize(background, bg, frame_big.size());
//    cv::imwrite("background.png", bg);

//    cv::Mat d;
//    cv::resize(diff, d, frame_big.size());
//    cv::imwrite("difference.png", d);
}
