/// HEADER
#include "image_provider_bag.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::ImageProviderBag, csapex::MessageProvider)

using namespace csapex;

ImageProviderBag::ImageProviderBag()
    : view_(NULL), initiated(false)
{
    frames_ = 0;
}

void ImageProviderBag::load(const std::string& file)
{
    bag.open(file);
}

std::vector<std::string> ImageProviderBag::getExtensions() const
{
    return boost::assign::list_of<std::string> (".bag");
}

bool ImageProviderBag::hasNext()
{
    return initiated;
}

void ImageProviderBag::doInit()
{
    // TODO: make indepenant of topic, show list of all possible topics
    view_ = new rosbag::View(bag, rosbag::TopicQuery("/marlin/camera/image_raw"));
    for(rosbag::View::iterator it = view_->begin(); it != view_->end(); ++it) {
        frames_++;
    }
    view_it = view_->begin();
    frames_--;

    initiated = true;
}

void ImageProviderBag::reallyNext(cv::Mat& img, cv::Mat& mask)
{
    if(!initiated) {
        return;
    }

    if(next_frame != -1) {
        view_it = view_->begin();
        for(state.current_frame=0; state.current_frame != next_frame; ++state.current_frame) {
            view_it++;
        }
        next_frame = -1;
    }
    assert(hasNext());

    if(view_it != view_->end()) {
        sensor_msgs::ImageConstPtr img_msg = view_it->instantiate<sensor_msgs::Image>();
        assert(img_msg != NULL);

        cv_bridge::CvImageConstPtr mat = cv_bridge::toCvShare(img_msg, "bgr8");
        mat->image.copyTo(last_frame_);

        view_it++;
        state.current_frame++;
    }

    img = last_frame_;

    if(!slider_->isSliderDown()) {
        slider_->setValue(state.current_frame);
    }

    if(state.current_frame == frames_) {
        setPlaying(false);
    }
}
