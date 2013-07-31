/// HEADER
#include "match_scorer_reprojection.h"

/// COMPONENT
#include "matcher.h"

/// PROJECT
#include <data/matchable.h>
#include <data/matchable_pose.h>

/// SYSTEM
#include <Eigen/Core>

MatchScorerReprojection::MatchScorerReprojection(Matcher& matcher)
    : MatchScorer(matcher)
{
}

double MatchScorerReprojection::calculateScore(Matchable& frame, Matchable& reference, int* no_of_features) const
{
    std::vector<cv::KeyPoint> filtered_test_keypoints;
    std::vector<cv::KeyPoint> filtered_frame_keypoints;

    matcher.matchFiltered(&reference, &frame, filtered_test_keypoints, filtered_frame_keypoints);

    if(no_of_features) {
        *no_of_features = filtered_frame_keypoints.size();
    }

    // find homography needs at least 4 points
    bool valid = (int) filtered_test_keypoints.size() >= matcher.getMinPointCount()
                 && (int) filtered_frame_keypoints.size() >= matcher.getMinPointCount();

    if(!valid) {
        reference.last_roi = cv::Rect();

        return INFINITY;

    } else {
        std::vector<cv::Point2f> test_points;
        std::vector<cv::Point2f> frame_points;

        for(unsigned j = 0; j < filtered_frame_keypoints.size(); ++j) {
            test_points.push_back(filtered_test_keypoints[j].pt);
            frame_points.push_back(filtered_frame_keypoints[j].pt);
        }

        cv::Mat mask;
        cv::Mat H = cv::findHomography(test_points, frame_points, CV_RANSAC, 3.0, mask);

        // reprojection error
        double reprojection_error = 0;

        std::vector<cv::Point2f> test_points_proj;
        cv::perspectiveTransform(test_points, test_points_proj, H);

        for(unsigned i=0; i < test_points.size(); ++i) {
            double d = cv::norm(frame_points[i] - test_points_proj[i]);
            reprojection_error += d;
        }

        return reprojection_error;
    }
}
