/// HEADER
#include "trainer.h"

/// PROJECT
#include <data/frame_buffer.h>
#include <data/painter.h>
#include <db_strategy/db_strategy.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <exception>
#include <opencv2/opencv.hpp>

namespace bfs = boost::filesystem;

Trainer::Trainer()
    : state_changed_(false), frame(0)
{
    state = TRAINING_PAUSED;

    // import references
    bfs::directory_iterator end; // default construction yields past-the-end
    for(bfs::directory_iterator it(config("ref_dir").as<std::string>()); it != end; ++it) {
        std::string path(it->path().string());
        std::string file(it->path().filename().c_str());
        std::string yaw_str = file.substr(0, file.find_first_of('_'));
        double yaw;
        std::stringstream converter;
        converter << yaw_str;
        converter >> yaw;

        cv::Mat ref = cv::imread(path);

        reference_imgs_.push_back(std::pair<double, cv::Mat>(yaw, ref));
    }
}

Trainer::~Trainer()
{
}

bool Trainer::addNegativeExample(Frame::Ptr /*frame*/)
{
    assert(state == TRAINING);
//    context.negative_examples_.push_back(frame);
    return true;
}

bool Trainer::addValidationExample(Frame::Ptr frame)
{
    assert(state == TRAINING);
    db_strategy->addValidationExample(frame);
    return true;
}

void Trainer::validate()
{
    db_strategy->validate();
    db_strategy->save();
    //    context.db_strategy->dump_reference(trainer.object_references_path);
}

void Trainer::reset()
{
    db_strategy->clear();
    state = TRAINING_PAUSED;
}


void Trainer::startTraining()
{
    if(state == TRAINING) {
        return;
    }

    state = TRAINING;
    state_changed_ = true;
}

void Trainer::pauseTraining()
{
    if(state == TRAINING_PAUSED) {
        return;
    }

    state = TRAINING_PAUSED;
    state_changed_ = true;
}


void Trainer::stopTraining()
{
    if(state == TRAINING_STOPPED) {
        return;
    }

    state = TRAINING_STOPPED;
    state_changed_ = true;
}

void Trainer::findMatchingReference()
{
    cv::Mat reference;
    double best_dist = INFINITY;
    double current_yaw = current_frame->orientation.toRadians();

    for(int i = 0, n = reference_imgs_.size(); i < n; ++i) {
        double dist = std::abs(reference_imgs_[i].first - current_yaw);
        if(dist < best_dist) {
            best_dist = dist;
            reference = reference_imgs_[i].second;
        }
    }

    FrameBuffer::setImageAside(0, "reference", current_frame->getDebugImage(), reference);
}

void Trainer::training(Frame::Ptr frame)
{
    db_strategy->train(frame);
    Painter(frame.get()).drawRoi();
}

void Trainer::stopped(Frame::Ptr /*frame*/)
{
    if(state_changed_) {
        db_strategy->validate();
    }
}

void Trainer::paused(Frame::Ptr frame)
{
    Painter(frame.get()).drawRoi();
}

void Trainer::analyzeCurrentFrame(Frame::Ptr frame)
{
    extractFeatures(frame);

    switch(state) {
    case TRAINING_STOPPED:
        stopped(frame);
        break;

    case TRAINING:
        training(frame);
        break;

    case TRAINING_PAUSED:
    default:
        paused(frame);
        break;
    }

    if(state != TRAINING_STOPPED) {
        Painter painter(frame.get());
        painter.drawKeypointsRoiOverlay(cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        painter.visualizeOrientation(frame->orientation);
        display_debug_image();
        findMatchingReference();
    }

}

void Trainer::tick(double dt)
{
    Analyzer::tick(dt);

    state_changed_ = false;
}
