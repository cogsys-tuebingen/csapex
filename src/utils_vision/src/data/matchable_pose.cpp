/// HEADER
#include "matchable_pose.h"

MatchablePose* MatchablePose::NULL_POSE(new MatchablePose);
long MatchablePose::IMAGE_ID = 0;
std::string MatchablePose::IMAGE_PATH;


MatchablePose::MatchablePose()
    : saved(false)
{
}

MatchablePose::MatchablePose(const Frame::Ptr frame)
    : Matchable(*frame),
      saved(false)
{
    frame->image_roi.copyTo(image);
    frame->mask_roi.copyTo(mask);
}

MatchablePose::MatchablePose(const MatchablePose& pose)
    : Matchable(keypoints, descriptors),
      image(cv::Mat()),
      saved(pose.saved)
{
    pose.image.copyTo(image);
    pose.mask.copyTo(mask);
}

cv::Rect MatchablePose::getDimensions() const
{
    return cv::Rect(0, 0, image.cols, image.rows);
}
