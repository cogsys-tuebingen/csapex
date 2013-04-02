/// HEADER
#include "roi_manager.h"

/// COMPONENT
#include "roi_provider.h"

/// PROJECT
#include <data/painter.h>
#include <utils/rectangle_cluster.h>

RoiManager::RoiManager()
{
}

RoiManager::~RoiManager()
{
    for(std::vector<RoiProvider*>::iterator it = providers_.begin(); it != providers_.end(); ++it) {
        delete *it;
    }
}

void RoiManager::newFrame(Frame* current_frame)
{
    current_frame_ = current_frame;
    for(std::vector<RoiProvider*>::iterator it = providers_.begin(); it != providers_.end(); ++it) {
        (*it)->newFrame(current_frame_);
    }
}

void RoiManager::maintain(RoiProvider* provider)
{
    providers_.push_back(provider);
}

void RoiManager::forEachRoi(boost::function<void(Roi)> callback)
{
    if(providers_.empty()) {
        callback(Roi(cv::Rect(0, 0, current_frame_->getWidth(), current_frame_->getHeight())));

    } else {
        std::vector<Roi> rois;

        // get all rois from providers and call back function
        for(std::vector<RoiProvider*>::iterator it = providers_.begin(); it != providers_.end(); ++it) {
            (*it)->addRois(rois);
        }

        for(std::vector<Roi>::const_iterator it = rois.begin(); it != rois.end(); ++it) {
            Painter(current_frame_).drawRectangle(it->rect, it->color, 1);

            callback(Roi(*it));
        }
    }
}

void RoiManager::setFrameSize(const cv::Rect& size)
{
    for(std::vector<RoiProvider*>::iterator it = providers_.begin(); it != providers_.end(); ++it) {
        (*it)->setFrameSize(size);
    }
}

void RoiManager::roiContainsTarget(const Roi& roi)
{
    for(std::vector<RoiProvider*>::iterator it = providers_.begin(); it != providers_.end(); ++it) {
        (*it)->roiContainsTarget(roi);
    }
}
