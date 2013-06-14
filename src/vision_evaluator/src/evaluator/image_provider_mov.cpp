/// HEADER
#include "image_provider_mov.h"

REGISTER_IMAGE_PROVIDERS(ImageProviderMov, mov, mpeg)
REGISTER_IMAGE_PROVIDERS(ImageProviderMov, mpg, mp4, avi)

using namespace vision_evaluator;

ImageProviderMov::ImageProviderMov(const std::string& movie_file)
    : capture_(movie_file)
{
    fps_ = capture_.get(CV_CAP_PROP_FPS);
    frames_ = capture_.get(CV_CAP_PROP_FRAME_COUNT);
}

ImageProvider* ImageProviderMov::createInstance(const std::string& path)
{
    return new ImageProviderMov(path);
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
        capture_.set(CV_CAP_PROP_POS_FRAMES, next_frame);
    }

    capture_ >> last_frame_;

    img = last_frame_;

    current_frame = capture_.get(CV_CAP_PROP_POS_FRAMES);

    if(!slider_->isSliderDown()) {
        slider_->setValue(current_frame);
    }

    if(current_frame == frames_) {
        setPlaying(false);
    }
}
