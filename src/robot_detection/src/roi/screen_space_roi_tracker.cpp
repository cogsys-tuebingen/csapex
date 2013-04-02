/// HEADER
#include "screen_space_roi_tracker.h"

/// PROJECT
#include <common/global.hpp>

ScreenSpaceRoiTracker::ScreenSpaceRoiTracker()
{
}

void ScreenSpaceRoiTracker::addRois(std::vector<Roi> &rois)
{
    rois.insert(rois.end(), tracked_.begin(), tracked_.end());

    WARN("tracking " << rois.size() << " rois in screen space");
}

void ScreenSpaceRoiTracker::roiContainsTarget(const Roi& roi)
{
    Roi copy = roi;
    copy.rect &= frame_size_;

    Roi* cover;
    if(isTracking(copy, cover)) {
        return;
    }

    copy.color = cv::Scalar(0, 0, 255);

    tracked_.push_back(copy);
}

void ScreenSpaceRoiTracker::newFrame(Frame* current_frame)
{
    RoiProvider::newFrame(current_frame);

    // grow areas a little
    for(std::vector<Roi>::iterator it = tracked_.begin(); it != tracked_.end(); ++it) {
        it->grow(25 /*pixels*/, 25 /*pixels*/);
        it->rect &= frame_size_;
    }
}

bool ScreenSpaceRoiTracker::isTracking(const Roi& roi, Roi*& covered_by)
{
    for(std::vector<Roi>::iterator it = tracked_.begin(); it != tracked_.end(); ++it) {
        if(it->covers(roi)) {
            covered_by = &*it;
            return true;
        }
    }

    return false;
}
