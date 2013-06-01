/*
 * hough_peak.cpp
 *
 *  Created on: 3 8, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "hough_peak.h"

/// SYSTEM
#include <utils/LibUtil/Stopwatch.h>

template <bool debug_enabled, bool use_theta>
HoughPeak<debug_enabled, use_theta>::HoughPeak(int cluster_count, int scaling, int octaves, const Matchable& a, const Matchable& b)
    : max_clusters(cluster_count), scaling(scaling), octaves(octaves), a_keypoints(a.keypoints), b_keypoints(b.keypoints)
{
    padding = 2;
    padding_dist = 4;

    min_count = 4;
    max_distance = 4 * padding;//2 * 32.0 / scaling;

    dim_theta = 10;
    //    dim_theta = 0;
    dim_sigma = 2 * octaves;
    dim_tx = 1.5 * std::max(a.getDimensions().width, b.getDimensions().width) / scaling;
    dim_ty = 1.5 * std::max(a.getDimensions().height, b.getDimensions().height) / scaling;

    initHough();
}

template <bool debug_enabled, bool use_theta>
HoughPeak<debug_enabled, use_theta>::~HoughPeak()
{
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::initHough()
{
    if(use_theta) {
        int sizes[4];
        sizes[HoughData::INDEX_THETA] = (dim_theta);
        sizes[HoughData::INDEX_SIGMA] = (dim_sigma);
        sizes[HoughData::INDEX_X] = (dim_tx);
        sizes[HoughData::INDEX_Y] = (dim_ty);

        hough = cv::Mat(HoughData::INDEX_COUNT, sizes, CV_32SC1, cv::Scalar::all(0));

    } else {
        int sizes[3];
        sizes[HoughData::INDEX_SIGMA] = (dim_sigma);
        sizes[HoughData::INDEX_X] = (dim_tx);
        sizes[HoughData::INDEX_Y] = (dim_ty);

        hough = cv::Mat(HoughData::INDEX_COUNT-1, sizes, CV_32SC1, cv::Scalar::all(0));
    }
}

template <bool debug_enabled, bool use_theta>
int HoughPeak<debug_enabled, use_theta>::filter(std::vector<std::vector<cv::DMatch> >& matches,
        std::vector<HoughData::Cluster>& clusters)
{

    clusters.clear();

    HoughDebugFields<debug_enabled>::initDebug(scaling, dim_tx, dim_ty);

    Stopwatch stopwatch;

    // go through all matches and populate the histogram
    populateHistogram(matches);
    if(debug_enabled) {
        log << "populate: " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }

    // find maximum value and compute average
    calculateThreshold();
    if(debug_enabled) {
        log << "threshold: " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }

    // find all possible peaks();
    findPeaks();
    if(debug_enabled) {
        log << "peaks: " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }

    drawDebugProjection();
    if(debug_enabled) {
        log << "draw: " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }

    // match peaks and matches that belong together
    assignMatchesToPeaks(matches, clusters);
    if(debug_enabled) {
        log << "assign (clusters=" << clusters.size() << "): " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }

    HoughDebugFields<debug_enabled>::finishDebug();

    return sum;
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::vote(int itheta, int isigma, int itx, int ity)
{
    if(debug_enabled) {
        assert(0 <= itheta);
        assert(itheta < dim_theta);
        assert(0 <= isigma);
        assert(isigma < dim_sigma);
        assert(0 <= itx);
        assert(itx < dim_tx);
        assert(0 <= ity);
        assert(ity < dim_ty);
    }

    unsigned idx;

    if(use_theta) {
        idx = (itheta) * hough.step[HoughData::INDEX_THETA]
              + (isigma) * hough.step[HoughData::INDEX_SIGMA]
              + (itx) * hough.step[HoughData::INDEX_X]
              + (ity) * hough.step[HoughData::INDEX_Y];
    } else {
        idx = (isigma) * hough.step[HoughData::INDEX_SIGMA]
              + (itx) * hough.step[HoughData::INDEX_X]
              + (ity) * hough.step[HoughData::INDEX_Y];
    }

    int& magnitude = (int&) hough.data[idx];

    if(magnitude == 0) {
        count++;
    }

    magnitude++;

    if(magnitude > max) {
        max = magnitude;
    }

    sum ++;
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::populateHistogram(const std::vector<std::vector<cv::DMatch> >& all_matches)
{
    max = 0;
    sum = 0;
    count = 0;

    for(std::vector<std::vector<cv::DMatch> >::const_iterator it = all_matches.begin(); it != all_matches.end(); ++it) {
        if(it->size() == 0) {
            continue;
        }

        const std::vector<cv::DMatch>& matches = *it;

        int n = matches.size();
        if(n == 0) {
            continue;
        }

        for(int i = 0; i < n; ++i) {
            const cv::DMatch& match = matches[i];

            int isigma, itheta, itx, ity;

            match2index(match, itheta, isigma, itx, ity);

            if(debug_enabled) {
                assert(0 <= itheta);
                assert(itheta < dim_theta);
            }

            if(itx < padding_dist || itx >= dim_tx-padding_dist ||
                    ity < padding_dist || ity >= dim_ty-padding_dist) {

            } else {
                if(isigma < padding) {
                    isigma = padding;
                } else if(isigma >= dim_sigma-padding) {
                    isigma = dim_sigma-padding - 1;
                }

                // vote
                for(int dx = -padding_dist; dx <= padding_dist; ++dx) {
                    for(int dy = -padding_dist; dy <= padding_dist; ++dy) {
                        for(int dsigma = -padding; dsigma <= padding; ++dsigma) {
                            for(int dtheta = -padding; dtheta <= padding; ++dtheta) {
                                int theta_wrap = (dim_theta + itheta+dtheta) % dim_theta;
                                vote(theta_wrap, isigma+dsigma, itx+dx, ity+dy);
                            }
                        }
                    }
                }
            }
        }
    }
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::calculateThreshold()
{
    double avg = sum / (double) count;

    // set the threshold between average and maximum
    threshold = avg + (max - avg) * 0.025;
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::findPeaks()
{
    Stopwatch stopwatch;

    std::deque<HoughData::Peak> peaks_candidates;
    findPeakCandidates(peaks_candidates);

    peaks.clear();

    if(debug_enabled) {
        log << "peak (fill): " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }

    // abort, if no peak candidates are found
    if(peaks_candidates.empty()) {
        return;
    }

    // sort the peaks by their histogram counts
    std::sort(peaks_candidates.begin(), peaks_candidates.end(), HoughData::Peak::compare);

    if(debug_enabled) {
        assert(peaks_candidates.front().magnitude >= peaks_candidates.back().magnitude);
        log << "peak (sort): " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }

    // init by using the best candidate first
    peaks.push_back(peaks_candidates.front());
    peaks_candidates.pop_front();

    if(debug_enabled) {
        log << "peak (candidates=" << peaks_candidates.size() << ")\n";
    }

    // merge all the other peaks if appropriate
    double max_dist_sqr = std::pow(max_distance, 2);
    int max_peak_count = max_clusters;

    for(std::deque<HoughData::Peak>::iterator candidate = peaks_candidates.begin(); candidate != peaks_candidates.end(); ++candidate) {
        // find closest peak
        std::vector<HoughData::Peak>::iterator closest_peak = peaks.end();
        double closest_distance = max_dist_sqr;

        for(std::vector<HoughData::Peak>::iterator peak = peaks.begin(); peak != peaks.end(); ++peak) {
            double distance_sqr = peak->distanceToSqr(*candidate);
            if(distance_sqr <= closest_distance) {
                closest_peak = peak;
                closest_distance = distance_sqr;
            }
        }

        if(closest_peak != peaks.end()) {
            // if there is a close by peak -> merge
            *closest_peak += *candidate;
        } else {
            // no close peak? use the current one
            if((int) peaks.size() < max_peak_count) {
                peaks.push_back(*candidate);
            }

        }
    }

    // sort the peaks by their histogram counts
    std::sort(peaks.begin(), peaks.end(), HoughData::Peak::compare);

    if(debug_enabled) {
        log << "peak (peaks=" << peaks.size() << ")\n";
        log << "peak (rest): " << stopwatch.usElapsed() * 1e-3 << "ms\n";
        stopwatch.reset();
    }
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::findPeakCandidates(std::deque<HoughData::Peak> &peaks_candidates)
{
    int* raw_data = (int*) hough.data;
    long i = 0;

    // unwrapped loop to improve speed
    if(use_theta) {
        for(int tx = 0; tx < dim_tx; ++tx) {
            for(int ty = 0; ty < dim_ty; ++ty) {
                for(int tsigma = 0; tsigma < dim_sigma; ++tsigma) {
                    for(int ttheta = 0; ttheta < dim_theta; ++ttheta) {
                        const int& magnitude = raw_data[i++];
                        if(magnitude > threshold) {
                            HoughData::Peak p(magnitude, ttheta, tsigma, tx, ty);
                            peaks_candidates.push_back(p);
                        }
                    }
                }
            }
        }
    } else {
        int* raw_data = (int*) hough.data;
        long i = 0;
        for(int tx = 0; tx < dim_tx; ++tx) {
            for(int ty = 0; ty < dim_ty; ++ty) {
                for(int tsigma = 0; tsigma < dim_sigma; ++tsigma) {
                    const int& magnitude = raw_data[i++];
                    if(magnitude > threshold) {
                        HoughData::Peak p(magnitude, -1, tsigma, tx, ty);
                        peaks_candidates.push_back(p);
                    }
                }
            }
        }
    }

}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::drawDebugProjection()
{
}

template <>
void HoughPeak<true, true>::drawDebugProjection()
{
    int* raw_data = (int*) hough.data;

    long i = 0;

    for(int tx = 0; tx < dim_tx; ++tx) {
        for(int ty = 0; ty < dim_ty; ++ty) {
            for(int ts = 0; ts < dim_sigma; ++ts) {
                for(int tt = 0; tt < dim_theta; ++tt) {
                    int magnitude = raw_data[i++];

                    cv::Vec4i at;
                    at[HoughData::INDEX_THETA] = tt;
                    at[HoughData::INDEX_SIGMA] = ts;
                    at[HoughData::INDEX_X] = tx;
                    at[HoughData::INDEX_Y] = ty;
                    assert(magnitude == hough.at<int>(at));

                    cv::Vec3i& pixel = debug_raw.at<cv::Vec3i>(ty, tx);
                    if(magnitude > threshold) {
                        pixel[0] -= magnitude;
                        pixel[2] -= magnitude;
                    } else if(magnitude > 0) {
                        pixel[0] -= magnitude;
                        pixel[1] -= magnitude;
                    }
                }
            }
        }
    }
}


template <>
void HoughPeak<true, false>::drawDebugProjection()
{
    int* raw_data = (int*) hough.data;

    long i = 0;

    for(int tx = 0; tx < dim_tx; ++tx) {
        for(int ty = 0; ty < dim_ty; ++ty) {
            for(int ts = 0; ts < dim_sigma; ++ts) {
                int magnitude = raw_data[i++];

                cv::Vec4i at;
                at[HoughData::INDEX_SIGMA] = ts;
                at[HoughData::INDEX_X] = tx;
                at[HoughData::INDEX_Y] = ty;
                assert(magnitude == hough.at<int>(at));

                cv::Vec3i& pixel = debug_raw.at<cv::Vec3i>(ty, tx);
                if(magnitude > threshold) {
                    pixel[0] -= magnitude;
                    pixel[2] -= magnitude;
                } else if(magnitude > 0) {
                    pixel[0] -= magnitude;
                    pixel[1] -= magnitude;
                }
            }
        }
    }
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::match2index(const cv::DMatch& match, int& itheta, int& isigma, int& itx,  int& ity) const
{
    assert((unsigned) match.queryIdx < a_keypoints.size());
    assert((unsigned) match.trainIdx < b_keypoints.size());

    const cv::KeyPoint& ka = a_keypoints[match.queryIdx];
    const cv::KeyPoint& kb = b_keypoints[match.trainIdx];

    int dsigma = ka.octave - kb.octave;
    float dtheta = Angle(ka.angle - kb.angle).toDegrees();
    int tx = (ka.pt.x - kb.pt.x) / scaling;
    int ty = (ka.pt.y - kb.pt.y) / scaling;

    isigma = octaves + dsigma;
    itheta = (180 + dtheta) / 360 * dim_theta;
    itx = dim_tx / 2 + tx;
    ity = dim_ty / 2 + ty;
}

template <bool debug_enabled, bool use_theta>
void HoughPeak<debug_enabled, use_theta>::assignMatchesToPeaks(const std::vector<std::vector<cv::DMatch> >& all_matches, std::vector<HoughData::Cluster>& clusters)
{
    clusters.clear();

    for(std::vector<HoughData::Peak>::iterator peak = peaks.begin(); peak != peaks.end(); ++peak) {
        HoughData::Cluster cluster;
        cluster.mean = peak->mean;
        cluster.magnitude = peak->magnitude;
        clusters.push_back(cluster);
    }

    double max_dist_sqr = std::pow(max_distance, 2);

    for(std::vector<std::vector<cv::DMatch> >::const_iterator it = all_matches.begin(); it != all_matches.end(); ++it) {
        if(it->size() == 0) {
            continue;
        }

        const std::vector<cv::DMatch>& matches = *it;

        int n = matches.size();
        if(n == 0) {
            continue;
        }

        for(int i = 0; i < n; ++i) {
            const cv::DMatch& match = matches[i];

            std::vector<HoughData::Peak>::iterator best_peak = peaks.end();
            double best_distance = max_dist_sqr;
            int best_cluster_id = -1;

            int cluster_id = 0;
            for(std::vector<HoughData::Peak>::iterator peak = peaks.begin(); peak != peaks.end(); ++peak) {
                int isigma, itheta, itx, ity;

                match2index(match, itheta, isigma, itx, ity);

                double distance_sqr = peak->distanceToSqr(itheta, isigma, itx, ity);
                if(distance_sqr < best_distance) {
                    best_distance = distance_sqr;
                    best_peak = peak;
                    best_cluster_id = cluster_id;
                }

                cluster_id++;
            }

            if(best_peak != peaks.end()) {
                assert(best_cluster_id >= 0);
                assert(best_cluster_id < (int) clusters.size());
                clusters[best_cluster_id].matches.push_back(&match);
            }
        }
    }

    for(std::vector<HoughData::Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();) {
        if((int) cluster->matches.size() < min_count) {
            clusters.erase(cluster);
        } else {
            cluster->variance = cluster->matches.size();
            ++cluster;
        }
    }
}


// instantiation
template class HoughPeak<true, true>;
template class HoughPeak<true, false>;
template class HoughPeak<false, true>;
template class HoughPeak<false, false>;
