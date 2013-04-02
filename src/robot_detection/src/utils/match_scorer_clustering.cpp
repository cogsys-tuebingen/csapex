/// HEADER
#include "match_scorer_clustering.h"

/// COMPONENT
#include "matcher.h"

/// PROJECT
#include <common/global.hpp>
#include <config/config.h>
#include <data/frame.h>
#include <data/matchable.h>
#include <data/matchable_pose.h>
#include <utils/hough_peak.h>
#include <utils/opencv_utils.hpp>

MatchScorerClustering::MatchScorerClustering(Matcher& matcher)
    : MatchScorer(matcher)
{
}

double MatchScorerClustering::calculateScore(Matchable& frame, Matchable& reference, int* no_of_features) const
{
    Config cfg = Config::getGlobal();

    int k = 8;
    int scaling = 20;
    HoughPeak<false,true> hough_imp(k, scaling, cfg.octaves, reference, frame);
    HoughAlgorithm& hough = hough_imp;
    hough.min_count = 8;

    std::vector<std::vector<cv::DMatch> > matches;
    matcher.match(&reference, &frame, matches);

    std::vector<HoughData::Cluster> clusters;
    hough.filter(matches, clusters);
//    debug = hough.debug;

    reference.last_roi = cv::Rect();
    reference.last_matches.clear();

    if(clusters.size() > 1) {
        for(int i = 0; i < (int) clusters.size(); ++i) {
            std::cout << clusters[i].magnitude << " ";
        }
        std::cout << std::endl;
        for(int i = 1; i < (int) clusters.size(); ++i) {
            assert(clusters[i-1].magnitude >= clusters[i].magnitude);
        }
    }

//    cv::imshow("debug", hough.debug);
//    cv::imshow("frame", dynamic_cast<Frame*>(frame)->getImage());
//    cv::imshow("reference", dynamic_cast<MatchablePose*>(reference)->image);
//    cv::waitKey(0);


    if(clusters.size() == 0) {
        return INFINITY;
    }

    cv::Point lt(frame.getDimensions().width, frame.getDimensions().height);
    cv::Point rb(0, 0);

    int n = clusters[0].matches.size();
    reference.last_matches.resize(n);

    cv::Mat distances(n,1,CV_64F);

    std::set<int> keypoints_a;
    std::set<int> keypoints_b;

    for(int i = 0; i < n; ++i) {
        const HoughData::Cluster& cluster = clusters[0];
        const cv::DMatch& match = *cluster.matches[i];
        std::vector<cv::DMatch> v(1);
        v[0] = match;
        reference.last_matches[i] = v;

        const cv::Point& pt = frame.keypoints[match.trainIdx].pt;
        if(pt.x < lt.x) lt.x = pt.x;
        if(pt.x > rb.x) rb.x = pt.x;
        if(pt.y < lt.y) lt.y = pt.y;
        if(pt.y > rb.y) rb.y = pt.y;

        keypoints_a.insert(match.trainIdx);
        keypoints_b.insert(match.queryIdx);

        int itheta, isigma, itx, ity;
        hough.match2index(match, itheta, isigma, itx, ity);

        distances.at<double>(i) = cluster.distanceToSqr(itheta, isigma, itx, ity);
    }

    reference.last_roi = cv::Rect(lt, rb) + cv::Size(10, 10);

    cv::Mat mean;
    cv::Mat stddev;
    cv::meanStdDev(distances, mean, stddev);

    double kp_ratio = keypoints_a.size() / (double) keypoints_b.size();

    double ratio = 100.0 * std::max(kp_ratio, 1.0 / kp_ratio);
    double std = stddev.at<double>(0);

    if(std == 0) {
        WARN("std dev was zero!");
        return INFINITY;
    }
    return ratio * std;

//    double max = 1000.0;
//    int count = clusters[0].matches.size();
//    if(count > max) {
//        WARN("cluster too large!!!" << count);
//    }
//    return std::max(max - count, 0.0) / overall_count;
}
