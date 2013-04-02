#ifndef ADAPTIVE_BACKGROUND_REMOVER_H
#define ADAPTIVE_BACKGROUND_REMOVER_H

/// COMPONENT
#include "background_remover.h"

/// SYSTEM
#include <deque>

namespace background_subtraction
{
/**
 * @brief The AdaptiveBackgroundRemover class implements a BackgroundRemover with an adaptive background model
 */
class AdaptiveBackgroundRemover : public BackgroundRemover
{
public:
    /**
     * @brief AdaptiveBackgroundRemover
     */
    AdaptiveBackgroundRemover();

    /**
     * @brief setBackground Overrides the background setting method to be able to collect multiple frames
     * @param frame the first frame to use
     */
    void setBackground(const cv::Mat& frame);


    /**
     * @brief getBackground Override: Accessor
     * @return background image
     */
    cv::Mat getBackground() {
        return background_blured;
    }

    /**
     * @brief getDebugImage Override: Accessore
     * @return the debug image
     */
    cv::Mat getDebugImage() {
        return debug;
    }

    /**
     * @brief setMaxDistance Setter
     * @param dist maximum distance between mean and background for adaption
     */
    void setMaxDistance(double dist) {
        max_dist = dist;
    }

    /**
     * @brief setMaxStdDev Setter
     * @param v maximum standard deviation in variance for adaption
     */
    void setMaxStdDev(double v) {
        max_std_dev = v;
    }

    /**
     * @brief setDecay Setter
     * @param d decay factor for variance and mean
     */
    void setDecay(double d) {
        decay = d;
    }

protected:
    /**
     * @brief segmentation Template method for subclasses
     * @param original the current frame
     * @param mask Output: the generated mask
     */
    void segmentation(const cv::Mat& frame, cv::Mat& mask);

private:
    void replaceBackground(const cv::Mat& frame);
    void calcDifference(const cv::Mat& frame, const cv::Mat& reference, cv::Mat& result);
    void updateBackground(const cv::Mat& frame, const cv::Mat& blured);
    void add(const cv::Mat& frame);
    void sum2Img(cv::Mat normal);
    void fp2char(const cv::Mat& fp_mat, cv::Mat& frame, double f);

private:
    cv::Mat mean;
    cv::Mat M2;

    cv::Mat debug;

    int n;
    int scale;

    double max_dist;
    double max_std_dev;
    double decay;

    int takes_max;
    int takes;
};

}
#endif // ADAPTIVE_BACKGROUND_REMOVER_H
