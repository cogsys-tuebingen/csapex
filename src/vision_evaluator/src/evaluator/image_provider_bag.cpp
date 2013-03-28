/// HEADER
#include "image_provider_bag.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

REGISTER_IMAGE_PROVIDERS(ImageProviderBag, bag)

using namespace vision_evaluator;

ImageProviderBag::ImageProviderBag(const std::string& file)
    : bag(file), view(bag, rosbag::TopicQuery("/camera/image_raw"))
{
    view_it = view.begin();
    frames_ = 0;

    for(rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
//        sensor_msgs::ImageConstPtr img = it->instantiate<sensor_msgs::Image>();
//        assert(img != NULL);

        frames_++;
    }
    frames_--;
}

ImageProvider* ImageProviderBag::createInstance(const std::string& path)
{
    return new ImageProviderBag(path);
}

bool ImageProviderBag::hasNext()
{
    return true;
}

void ImageProviderBag::reallyNext()
{
    if(next_frame != -1) {
        view_it = view.begin();
        for(current_frame=0; current_frame != next_frame; ++current_frame) {
            view_it++;
        }
        next_frame = -1;
    }
    assert(hasNext());

    sensor_msgs::ImageConstPtr img = view_it->instantiate<sensor_msgs::Image>();
    assert(img != NULL);

    cv_bridge::CvImageConstPtr mat = cv_bridge::toCvShare(img, "bgr8");
    mat->image.copyTo(last_frame_);

    view_it++;
    current_frame++;

    if(!slider_->isSliderDown()) {
        slider_->setValue(current_frame);
    }

    provide(last_frame_);

//    provide(last_frame_);

    if(current_frame == frames_) {
        setPlaying(false);
    }
}
