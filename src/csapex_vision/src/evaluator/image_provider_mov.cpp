/// HEADER
#include "image_provider_mov.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::ImageProviderMov, csapex::MessageProvider)

using namespace csapex;

ImageProviderMov::ImageProviderMov()
{
}

void ImageProviderMov::load(const std::string& movie_file)
{
    capture_.open(movie_file);
    fps_ = capture_.get(CV_CAP_PROP_FPS);
    frames_ = capture_.get(CV_CAP_PROP_FRAME_COUNT);
}

ImageProviderMov::~ImageProviderMov()
{
    capture_.release();
}

std::vector<std::string> ImageProviderMov::getExtensions() const
{
    return boost::assign::list_of<std::string> (".avi")(".mpg")(".mp4")(".mov")(".flv");
}

bool ImageProviderMov::hasNext()
{
    //    if(!capture_.isOpened()){
    //        return false;
    //    }

    return capture_.isOpened();
}

void ImageProviderMov::reallyNext(cv::Mat& img, cv::Mat& mask)
{
    if(!capture_.isOpened()) {
        std::cerr << "cannot display, capture not open" << std::endl;
        return;
    }

    if(next_frame != -1) {
        state.current_frame = capture_.get(CV_CAP_PROP_POS_FRAMES);
        capture_ >> last_frame_;
        capture_.set(CV_CAP_PROP_POS_FRAMES, next_frame);
    }

    capture_ >> last_frame_;

    img = last_frame_;

    state.current_frame = capture_.get(CV_CAP_PROP_POS_FRAMES);

    if(!slider_->isSliderDown()) {
        slider_->setValue(state.current_frame);
    }

    if(state.current_frame == frames_) {
        setPlaying(false);
    }
}
