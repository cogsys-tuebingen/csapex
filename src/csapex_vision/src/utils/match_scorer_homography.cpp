/// HEADER
#include "match_scorer_homography.h"

/// COMPONENT
#include "matcher.h"

/// PROJECT
#include <common/global.hpp>
#include <data/frame.h>
#include <data/painter.h>
#include <data/matchable.h>
#include <data/matchable_pose.h>

/// SYSTEM
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

MatchScorerHomography::MatchScorerHomography(Matcher& matcher)
    : MatchScorer(matcher)
{
}

namespace
{
void cameraPoseFromHomography(const cv::Mat& H, cv::Mat& rot, cv::Mat& trans)
{
    rot = cv::Mat::eye(3, 3, CV_64FC1);
    trans = cv::Mat::zeros(3, 1, CV_64FC1);

    assert(H.type() == rot.type());
    assert(H.type() == trans.type());

    double norm1 = (double)cv::norm(H.col(0));
    double norm2 = (double)cv::norm(H.col(1));
    double tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    cv::Mat p1 = H.col(0);       // Pointer to first column of H
    cv::Mat p2 = rot.col(0);    // Pointer to first column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose

    p1 = H.col(1);           // Pointer to second column of H
    p2 = rot.col(1);        // Pointer to second column of pose (empty)

    cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose

    p1 = rot.col(0);
    p2 = rot.col(1);

    cv::Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
    cv::Mat c2 = rot.col(2);    // Pointer to third column of pose
    p3.copyTo(c2);       // Third column is the crossproduct of columns one and two

    cv::Mat tmp = H.col(2) / tnorm;
    tmp.copyTo(trans.col(0));  //vector t [R|t] is the last column of pose
}
}

double MatchScorerHomography::calculateScore(Matchable& frame, Matchable& reference, int* no_of_features) const
{
    Painter painter(&frame);

    std::vector<cv::KeyPoint> filtered_test_keypoints;
    std::vector<cv::KeyPoint> filtered_frame_keypoints;

    matcher.matchFiltered(&reference, &frame, filtered_test_keypoints, filtered_frame_keypoints);

    painter.drawKeypointsRoiHighlight(filtered_frame_keypoints, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    if(no_of_features) {
        *no_of_features= filtered_frame_keypoints.size();
    }

    // find homography needs at least 4 points
    bool valid = (int) filtered_test_keypoints.size() >= matcher.getMinPointCount()
                 && (int) filtered_frame_keypoints.size() >= matcher.getMinPointCount();

    cv::Mat last_homography;

    if(!valid) {
        reference.last_roi = cv::Rect();
        reference.last_matches = std::vector<std::vector<cv::DMatch> >();

        return INFINITY;

    } else {
        std::vector<cv::Point2f> test_points;
        std::vector<cv::Point2f> frame_points;

        for(unsigned j = 0; j < filtered_frame_keypoints.size(); ++j) {
            test_points.push_back(filtered_test_keypoints[j].pt);
            frame_points.push_back(filtered_frame_keypoints[j].pt);
        }

        cv::Mat H = cv::findHomography(test_points, frame_points, CV_RANSAC, 3.0);

        if(painter.canPaint()) {
            std::vector<cv::Point2f> scene_corners = apply_homography(H, reference.getDimensions(), 0);

            int x = std::numeric_limits<int>::max();
            int y = std::numeric_limits<int>::max();
            int X = 0;
            int Y = 0;

            for(unsigned i=0; i < scene_corners.size(); ++i) {
                const cv::Point& pt = scene_corners[i];

                if(pt.x < x) {
                    x = pt.x;
                }
                if(pt.x > X) {
                    X = pt.x;
                }
                if(pt.y < y) {
                    y = pt.y;
                }
                if(pt.y > Y) {
                    Y = pt.y;
                }
            }

            reference.last_roi.x = x;
            reference.last_roi.y = y;
            reference.last_roi.width = X-x;
            reference.last_roi.height = Y-y;

            if(reference.last_roi.width > 10 &&  reference.last_roi.height > 10) {
                painter.drawLineRoi(scene_corners[0], scene_corners[1], cv::Scalar(0));
                painter.drawLineRoi(scene_corners[1], scene_corners[2], cv::Scalar(0));
                painter.drawLineRoi(scene_corners[2], scene_corners[3], cv::Scalar(0));
                painter.drawLineRoi(scene_corners[3], scene_corners[0], cv::Scalar(0));
            }
        }

        double q = qualityOf(H/*, current_frame->color*/);

        if(q == 0) {
            painter.displayImage("debug");
        }
        return q;
    }
}


double MatchScorerHomography::qualityOf(const cv::Mat& H, const cv::Mat& image)
{
    if(cv::determinant(H) == 0) {
        return INFINITY;
    }

//    std::vector<cv::Point2f> corners;
//    std::vector<cv::Point2f> scene_corners;

//    if(image.cols == 0 || image.rows == 0){
//        scene_corners = apply_homography(H, &corners);
//    } else {
//        scene_corners = apply_homography(H, image, 0, &corners);
//    }

    //    /// check if quadrilateral is self-intersecting
    //    LineSegment a(scene_corners[0], scene_corners[1]);
    //    LineSegment b(scene_corners[1], scene_corners[2]);
    //    LineSegment c(scene_corners[2], scene_corners[3]);
    //    LineSegment d(scene_corners[3], scene_corners[0]);

    //    if(a.intersects(c) || b.intersects(d)){
    //        INFO( "INFINITY: intersection" );
    //        return INFINITY;
    //    }

    //    if(!cv::isContourConvex(scene_corners)){
    //        INFO( "INFINITY: not convex" );
    //        return INFINITY;
    //    }

    //    corners[0] = cvPoint(0, 0);
    //    corners[1] = cvPoint(100, 0);
    //    corners[2] = cvPoint(100, 100);
    //    corners[3] = cvPoint(0, 100);
    //    if(scene_corners[0].x > scene_corners[1].x || scene_corners[0].x > scene_corners[2].x
    //            || scene_corners[3].x > scene_corners[1].x || scene_corners[3].x > scene_corners[2].x){
    //        INFO( "INFINITY: mirrored" );
    //        return INFINITY;
    //    }

    //    cv::RotatedRect min_rect = cv::minAreaRect(scene_corners);

    //    double area = min_rect.size.area();
    //    if(area < 20 * 20){
    //        return INFINITY;
    //    }
    //    double quality = area;

//    double mse = 0;
//    for(unsigned j=0; j < 4; ++j){
//        cv::Point2f diff = corners[j] - scene_corners[j];
//        mse += diff.dot(diff);
//    }
//    double quality = mse;

//    double scale = cv::boundingRect(corners).area() / cv::boundingRect(scene_corners).area();

    //return std::pow(H.at<double>(2,0), 2) + std::pow(H.at<double>(2,1), 2);

    cv::Mat rot, trans;
    cameraPoseFromHomography(H, rot, trans);

    Eigen::Matrix3d mat;
    cv::cv2eigen(rot, mat);
    Eigen::Vector3d euler = mat.eulerAngles(0, 1, 2);

    double yaw = euler(0);
    double pitch = euler(1);
    double roll = euler(2);

    double quality = /*std::pow(cv::norm(trans), 2) + */std::pow(yaw, 2) + std::pow(pitch, 2) + std::pow(roll, 2);
    if(quality == 0) {
        assert(cv::determinant(H) == 0);

        INFO("warning: quality is 0");
        INFO(rot.at<double>(0, 0) << "\t" << rot.at<double>(0, 1) << "\t" << rot.at<double>(0, 2) << "\n" <<
             rot.at<double>(1, 0) << "\t" << rot.at<double>(1, 1) << "\t" << rot.at<double>(1, 2) << "\n" <<
             rot.at<double>(2, 0) << "\t" << rot.at<double>(2, 1) << "\t" << rot.at<double>(2, 2));
        INFO("H was");
        INFO(H.at<double>(0, 0) << "\t" << H.at<double>(0, 1) << "\t" << H.at<double>(0, 2) << "\n" <<
             H.at<double>(1, 0) << "\t" << H.at<double>(1, 1) << "\t" << H.at<double>(1, 2) << "\n" <<
             H.at<double>(2, 0) << "\t" << H.at<double>(2, 1) << "\t" << H.at<double>(2, 2));
    }

// scale!!!!!!
    return quality;
}


std::vector<cv::Point2f> MatchScorerHomography::apply_homography(const cv::Mat& H, const cv::Rect& roi, int padding,
        std::vector<cv::Point2f> * obj_corners)
{
    int cols = roi.width;
    int rows = roi.height;

    std::vector<cv::Point2f> scene_corners(4);
    std::vector<cv::Point2f> corners(4);
    corners[0] = cvPoint(padding,padding);
    corners[1] = cvPoint(cols-padding, padding);
    corners[2] = cvPoint(cols-padding, rows-padding);
    corners[3] = cvPoint(padding, rows-padding);

    cv::perspectiveTransform(corners, scene_corners, H);

    if(obj_corners != NULL) {
        *obj_corners = corners;
    }

    return scene_corners;
}

std::vector<cv::Point2f> MatchScorerHomography::apply_homography(const cv::Mat& H, std::vector<cv::Point2f> * obj_corners)
{
    std::vector<cv::Point2f> scene_corners(4);
    std::vector<cv::Point2f> corners(4);
    int d = 1000;
    corners[0] = cvPoint(0, 0);
    corners[1] = cvPoint(d, 0);
    corners[2] = cvPoint(d, d);
    corners[3] = cvPoint(0, d);

    cv::perspectiveTransform(corners, scene_corners, H);

    if(obj_corners != NULL) {
        *obj_corners = corners;
    }

    return scene_corners;
}
