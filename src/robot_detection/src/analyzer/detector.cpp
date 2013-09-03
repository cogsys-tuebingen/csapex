/// HEADER
#include "detector.h"

/// PROJECT
#include <data/frame_buffer.h>
#include <data/painter.h>
#include <data/matchable_pose.h>
#include <db/database.h>
#include <db_strategy/db_strategy.h>
#include <db_strategy/factory.h>
#include <roi/roi_manager.h>
#include <roi/nth_full_frame_roi.h>
#include <roi/viola_jones_roi.h>
#include <roi/screen_space_roi_tracker.h>
#include <utils/extractor.h>
#include <utils/matcher.h>

/// SYSTEM
#include <Eigen/Geometry>
#include <fstream>


Detector::Detector()
    : roi_manager(new RoiManager)
{
    if(db_strategy->loadConfig()) {
        replaceDatabaseStrategy(DatabaseStrategyFactory::create());
    }

    if(!db_strategy->load()) {
        ERROR("couldn't load the database");
    }

    // TODO: make factory
    roi_manager->maintain(new NthFullFrameRoi(0));
//    roi_manager->maintain(new ViolaJonesRoi);
//    roi_manager->maintain(new ScreenSpaceRoiTracker);
}

void Detector::configChanged()
{
    INFO("config now: " << config("score_threshold").as<double>());
}

Detector::~Detector()
{
    delete roi_manager;
}

void Detector::analyzeCurrentFrame(Frame::Ptr frame)
{
    current_frame = frame;

    roi_manager->newFrame(frame.get());

    roi_manager->forEachRoi(boost::bind(&Detector::analyzeRoi, this, frame, _1));

    display_debug_image();
}

void Detector::analyzeRoi(Frame::Ptr frame, const Roi& roi)
{
    if(frame == Frame::NULL_FRAME) {
        ERROR("asked to analyze NULL FRAME");
        return;
    }

    extractFeatures(frame, roi.rect);

    cv::Rect best_roi;
    double score;
    MatchablePose* best_match = db_strategy->detect(frame, best_roi, &score);

    INFO("best score: " << score << ",\tthreshold: " << config("score_threshold").as<double>());

    if(best_match != MatchablePose::NULL_POSE && score <= 2 * config("score_threshold").as<double>()) {
        if(best_roi.width < 10 || best_roi.height < 10) {
            WARN("found a target in an area of "
                 << best_roi.width << "x" << best_roi.height << "px, ignoring...");
            return;
        }

        handle_pose(best_match, frame, roi);

        drawDebugInfo(best_roi, best_match);

        cv::Rect best_roi_screen = best_roi + roi.rect.tl();
        roi_manager->setFrameSize(frame->getDimensions());
        roi_manager->roiContainsTarget(Roi(best_roi_screen));
//        Painter(frame).drawRectangle(best_roi_screen & frame->getDimensions(), cv::Scalar(0,255,255));

    } else {
        Painter(frame.get()).drawRectangleRoi(best_roi, cv::Scalar(255, 127, 0), 1);
    }
}

void Detector::handle_pose(MatchablePose* best_match, Frame::Ptr frame, const Roi& roi)
{
    double distance = best_match->distance;
    if(distance <= 0) {
        INFO("WARNING: no distance measurement");
        // TODO: find a way around this evil hack
        distance = 3;
    } else {
        INFO("distance: " << distance << " m");
    }

    cv::Point center(roi.rect.x + roi.rect.width / 2.0, roi.rect.y + roi.rect.height / 2.0);

    // TODO: calculate correct transformation (calibrated camera!)
    Angle fov_x = Angle::fromDegrees(30.0);
    Angle fov_y = Angle::fromDegrees(30.0);
    double dx = frame->getWidth() / 2.0 - center.x;
    double dy = frame->getHeight() / 2.0 - center.y;

    //        double yaw_view = fov_x / 2 - fov_x * dx / half_width;
    //        double yaw_view = fov_x ( 1 / 2 - dx / half_width);
    Angle yaw(fov_x.toRadians() * (0.5 - dx / frame->getWidth()));
    Angle tilt(fov_y.toRadians() * (0.5 - dy / frame->getWidth()));

    Eigen::Quaterniond orientation;
    //orientation = Eigen::AngleAxisd(yaw.toRadians(), Eigen::Vector3d::UnitZ()) *
    //        Eigen::AngleAxisd(tilt.toRadians(), Eigen::Vector3d::UnitY());
    orientation = Eigen::AngleAxisd(yaw.toRadians(), Eigen::Vector3d::UnitZ());

    Eigen::Vector3d trans(distance, 0, 0);

    Pose pose;
    pose.orientation = best_match->orientation.toQuaternion();
    pose.position = orientation * trans;

    pose_detected(pose);
}


void Detector::drawDebugInfo(cv::Rect roi, MatchablePose* pose)
{
    try {
        cv::Mat matches;
        cv::drawMatches(pose->image, pose->keypoints, current_frame->getImageRoi(), current_frame->keypoints,
                        pose->last_matches, matches,
                        cv::Scalar::all(-1), cv::Scalar::all(0), std::vector<std::vector<char> >(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        FrameBuffer::setImage(0, "best match", matches);
    } catch(cv::Exception& e) {
        ERROR("debug drawing produces OpenCV exception.");
    }

//    cv::Point center(roi.x + roi.width / 2.0, roi.y + roi.height / 2.0);
    Painter painter(current_frame.get());
    painter.visualizeOrientation(pose->orientation, NULL, cv::Point(roi.x + pose->last_roi.width / 2, roi.y + pose->last_roi.height / 2));
    painter.drawRectangleRoi(roi);
}

void Detector::tick(double dt)
{
    Analyzer::tick(dt);
}
