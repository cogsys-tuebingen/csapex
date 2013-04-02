/// HEADER
#include "roi_provider.h"

RoiProvider::RoiProvider()
{
}

RoiProvider::~RoiProvider()
{

}

void RoiProvider::newFrame(Frame* current_frame)
{
    current_frame_ = current_frame;
}


void RoiProvider::setFrameSize(const cv::Rect& size)
{
    frame_size_ = size;
}
