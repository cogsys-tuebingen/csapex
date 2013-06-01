/*
 * hough_peak.h
 *
 *  Created on: Mar 8, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef HOUGH_PEAK_H
#define HOUGH_PEAK_H

/// PROJECT
#include <data/frame.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

struct HoughData {
    enum Index { INDEX_FIRST = 0,
                 INDEX_X = 0,
                 INDEX_Y,
                 INDEX_SIGMA,
                 INDEX_THETA,
                 INDEX_COUNT
               };

    struct Cluster {
        cv::Vec4d mean;
        unsigned magnitude;
        double variance;
        std::vector<const cv::DMatch*> matches;

        double distanceToSqr(int itheta, int isigma, int itx, int ity) const {
            return std::pow(isigma - mean[INDEX_SIGMA], 2) + std::pow(itheta - mean[INDEX_THETA], 2) +
                   std::pow(itx - mean[INDEX_X], 2) + std::pow(ity - mean[INDEX_Y], 2);
        }
    };

    struct Peak {
        Peak(unsigned v, const cv::Vec4i& center)
            : magnitude(v), count(v) {
            for(int index = INDEX_FIRST; index < INDEX_COUNT; ++index) {
                accum[index] += v * center[index];
            }

            updateCenter();
        }

        Peak(unsigned magnitude, int itheta, int isigma, int itx, int ity)
            : magnitude(magnitude), count(magnitude) {
            accum[INDEX_X] += magnitude * itx;
            accum[INDEX_Y] += magnitude * ity;
            accum[INDEX_SIGMA] += magnitude * isigma;
            accum[INDEX_THETA] += magnitude * itheta;

            updateCenter();
        }

        static bool compare(const Peak& lhs, const Peak& rhs) {
            return lhs.magnitude > rhs.magnitude;
        }

        void updateCenter() {
            mean = accum * (1.0 / count);
        }

        double distanceToSqr(const Peak& rhs) const {
            double res = 0;
            for(int index = INDEX_FIRST; index < INDEX_COUNT; ++index) {
                res += std::pow(mean[index] - rhs.mean[index], 2);
            }
            return res;
        }

        double distanceToSqr(int itheta, int isigma, int itx, int ity) {
            return std::pow(isigma - mean[INDEX_SIGMA], 2) + std::pow(itheta - mean[INDEX_THETA], 2) +
                   std::pow(itx - mean[INDEX_X], 2) + std::pow(ity - mean[INDEX_Y], 2);
        }

        void operator += (const Peak& rhs) {
            magnitude += rhs.magnitude;
            for(int index = INDEX_FIRST; index < INDEX_COUNT; ++index) {
                accum[index] += rhs.accum[index];
            }
            count += rhs.magnitude;
            updateCenter();
        }

        unsigned magnitude;
        cv::Vec4d mean;

    private:
        cv::Vec4i accum;
        unsigned count;
    };
};

/***
 * INTERFACES
 */
struct HoughAlgorithm {
    virtual int filter(std::vector<std::vector<cv::DMatch> >& matches,
                       std::vector<HoughData::Cluster>& clusters) = 0;

    virtual void match2index(const cv::DMatch& match, int& itheta, int& isigma, int& itx, int& ity) const = 0;

    cv::Mat hough;
    std::stringstream log;
    int min_count;
};

struct HoughAlgorithm_Debug : public HoughAlgorithm {
    double scale_factor;
    cv::Mat debug_raw;
    cv::Mat debug;
};

/***
 * DEBUG IMPLEMENTATION
 */
template <bool>
struct HoughDebugFields : public HoughAlgorithm {
    void initDebug(int scaling, int dim_tx, int dim_ty)  {}
    void finishDebug() {}
};

template <>
struct HoughDebugFields<true> : public HoughAlgorithm_Debug {

    void initDebug(int scaling, int dim_tx, int dim_ty)  {
        scale_factor = scaling / 4.0;
        debug_raw = cv::Mat(dim_ty, dim_tx, CV_32SC3, cv::Scalar::all(255));
    }

    void finishDebug() {
        // scale debug image so that the size is independant of the scale
        double min, max;
        cv::minMaxIdx(debug_raw, &min, &max);
        debug_raw.convertTo(debug, debug.type(), (max > 255.0) ? 255.0 / max : 1.0);

        cv::resize(debug, debug, cv::Size(), scale_factor, scale_factor);
    }
};


template <bool debug_enabled, bool use_theta = true>
class HoughPeak : public HoughDebugFields<debug_enabled>
{
    typedef HoughDebugFields<debug_enabled> Parent;

    using Parent::hough;
    using Parent::min_count;
    using Parent::log;

public:
    HoughPeak(int cluster_count, int scaling, int octaves, const Matchable& a, const Matchable& b);
    ~HoughPeak();


    int filter(std::vector<std::vector<cv::DMatch> >& matches,
               std::vector<HoughData::Cluster>& clusters);

    inline void match2index(const cv::DMatch& match, int& itheta, int& isigma, int& itx, int& ity) const;

private:
    void initHough();

    inline void vote(int itheta, int isigma, int itx, int ity);

    void populateHistogram(const std::vector<std::vector<cv::DMatch> >& matches);

    void calculateThreshold();

    void findPeaks();
    void findPeakCandidates(std::deque<HoughData::Peak>& peaks_candidates);

    void drawDebugProjection();

    void assignMatchesToPeaks(const std::vector<std::vector<cv::DMatch> >& matches, std::vector<HoughData::Cluster>& clusters);

private:
    int padding;
    int padding_dist;
    int max_clusters;
    int scaling;

    double max_distance;

    int dim_theta;
    int dim_sigma;
    int dim_tx;
    int dim_ty;

    int octaves;
    const std::vector<cv::KeyPoint>& a_keypoints;
    const std::vector<cv::KeyPoint>& b_keypoints;

    unsigned char max;
    unsigned char threshold;
    unsigned long sum;
    unsigned int count;
    std::vector<HoughData::Peak> peaks;
};

#endif // HOUGH_PEAK_H
