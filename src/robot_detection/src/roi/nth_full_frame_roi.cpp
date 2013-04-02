/// HEADER
#include "nth_full_frame_roi.h"

NthFullFrameRoi::NthFullFrameRoi(unsigned n_)
    : n(n_), counter_(0)
{
}

void NthFullFrameRoi::addRois(std::vector<Roi> &rois)
{
    if(counter_ == 0) {
        counter_ = n;

        rois.push_back(Roi(0,0, current_frame_->getWidth(), current_frame_->getHeight()));

    } else {
        --counter_;
    }
}
